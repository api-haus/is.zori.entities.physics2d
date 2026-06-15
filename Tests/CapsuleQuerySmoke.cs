using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif
using UnityEngine;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Smoke for the capsule additions to the spatial-query surface (<see cref="PhysicsQueries2D.CapsuleCast"/>,
    /// <see cref="PhysicsQueries2D.OverlapCapsule"/>) and the closest-point query
    /// (<see cref="PhysicsQueries2D.ClosestPoint"/>). Built from the queries' own decision points against a real
    /// stepped Box2D world (no mocks): a capsule swept toward a wall hits it nearest-first and resolves the owning
    /// entity; a capsule swept away misses; an overlap reports a body the capsule penetrates and not one it
    /// clears; the closest-point query returns the nearest of two bodies with a correct surface point, distance,
    /// and outward normal, and reports nothing beyond its search radius.
    /// </summary>
    /// <remarks>
    /// Mirrors <see cref="FilteringQuerySmoke"/>'s harness: a dedicated disposable <see cref="World"/> holding the
    /// package's FixedStep systems, bodies authored directly via <see cref="DirectPhysics2DAuthoring"/>, the world
    /// created (and stepped) by <c>group.Update()</c>, then the static query helpers read the created world.
    /// </remarks>
    public sealed class CapsuleQuerySmoke
    {
        const float Dt = 1f / 60f;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DCapsuleQuerySmokeWorld", out group, Dt);

        static PhysicsWorld GetWorld(EntityManager em)
        {
            var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        // A static box wall authored directly (unfiltered, so QueryFilter.Everything hits it).
        static Entity SpawnWall(EntityManager em, float2 center, float2 size)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition { bodyType = PhysicsBody.BodyType.Static, initialPosition = center },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = size,
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
        }

        [UnityTest]
        public IEnumerator CapsuleCast_HitsNearestWall_ResolvesOwningEntity_AndMissesAway()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A near wall whose left face is at X=3 and a far wall whose left face is at X=8, both tall enough to
            // span the capsule. A vertical capsule centred at the origin swept +X must hit the near wall first.
            var near = SpawnWall(em, new float2(4f, 0f), new float2(2f, 4f)); // left face at X=3
            var far = SpawnWall(em, new float2(9f, 0f), new float2(2f, 4f)); // left face at X=8

            group.Update(); // create the bodies (no step needed for a static-world query)

            var pw = GetWorld(em);
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);

            // A vertical capsule (caps on Y at ±0.5, radius 0.5 → 1×2 capsule) centred at the origin, swept +X.
            float2 c1 = new float2(0f, -0.5f);
            float2 c2 = new float2(0f, 0.5f);
            const float r = 0.5f;

            var n = PhysicsQueries2D.CapsuleCast(pw, c1, c2, r, new float2(1f, 0f), 20f, hitLayerMask: 0ul, hits);

            Assert.Greater(n, 0, "Capsule swept +X hit nothing — the cast found no wall ahead.");
            var hit = hits[0];
            Assert.AreEqual(
                near,
                hit.entity,
                $"CapsuleCast resolved the wrong nearest entity: got {hit.entity}, expected the near wall "
                    + $"{near} (far was {far}). Nearest-first ordering or the shape→entity packing is wrong."
            );
            Assert.IsFalse(isnan(hit.point.x) || isnan(hit.point.y), $"CapsuleCast hit point is NaN: {hit.point}.");
            // The capsule's right cap edge (origin radius 0.5) starts at X=0.5 and must reach the near wall face
            // at X=3, so the contact fraction along the 20-unit cast is ~ (3 - 0.5) / 20 = 0.125.
            Assert.Less(
                abs(hit.fraction - 0.125f),
                0.05f,
                $"CapsuleCast fraction {hit.fraction:F3} is not near the expected ~0.125 (cap edge X=0.5 to "
                    + "wall face X=3 over a 20-unit cast)."
            );
            Assert.Less(
                hit.normal.x,
                -0.5f,
                $"CapsuleCast normal {hit.normal} does not face back toward the cast origin (-X)."
            );

            // The same capsule swept the OTHER way (-X) hits nothing (no body to the left).
            var nAway = PhysicsQueries2D.CapsuleCast(pw, c1, c2, r, new float2(-1f, 0f), 20f, 0ul, hits);
            Assert.AreEqual(0, nAway, "Capsule swept -X (away from both walls) reported a phantom hit.");

            Debug.Log(
                $"[PHYSICS2D-CAPSULE] cast +X hit entity={hit.entity} at point={hit.point}, "
                    + $"normal={hit.normal}, fraction={hit.fraction:F3}; -X missed (n={nAway})."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator OverlapCapsule_ReportsPenetratedBody_NotAClearedOne()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A wall the capsule penetrates (left face at X=0.4, the capsule's right cap reaches X=0.5) and one
            // well clear of it (left face at X=5).
            var penetrated = SpawnWall(em, new float2(1.4f, 0f), new float2(2f, 4f)); // left face X=0.4
            var clear = SpawnWall(em, new float2(6f, 0f), new float2(2f, 4f)); // left face X=5

            group.Update();

            var pw = GetWorld(em);
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);

            float2 c1 = new float2(0f, -0.5f);
            float2 c2 = new float2(0f, 0.5f);
            const float r = 0.5f;

            var n = PhysicsQueries2D.OverlapCapsule(pw, c1, c2, r, 0ul, hits);
            Assert.Greater(n, 0, "OverlapCapsule reported no overlap with the penetrated wall.");

            var sawPenetrated = false;
            var sawClear = false;
            for (var i = 0; i < hits.Length; i++)
            {
                if (hits[i].entity == penetrated)
                    sawPenetrated = true;
                if (hits[i].entity == clear)
                    sawClear = true;
            }
            Assert.IsTrue(sawPenetrated, $"OverlapCapsule did not report the penetrated wall {penetrated}.");
            Assert.IsFalse(sawClear, $"OverlapCapsule falsely reported the cleared wall {clear} as overlapping.");

            Debug.Log($"[PHYSICS2D-CAPSULE] overlap reported {n} hit(s); penetrated={penetrated} clear={clear}.");

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator ClosestPoint_ReturnsNearestBody_WithSurfacePoint_DistanceAndNormal()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A near wall (left face at X=2) and a far wall (left face at X=10). A query point at the origin is
            // closest to the near wall, at distance 2, with the surface point at (2, 0) and a +X normal.
            var near = SpawnWall(em, new float2(3f, 0f), new float2(2f, 4f)); // left face X=2
            var far = SpawnWall(em, new float2(11f, 0f), new float2(2f, 4f)); // left face X=10

            group.Update();

            var pw = GetWorld(em);
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);

            var found = PhysicsQueries2D.ClosestPoint(
                pw,
                new float2(0f, 0f),
                maxDistance: 6f,
                hitLayerMask: 0ul,
                hits,
                out var result
            );

            Assert.IsTrue(found, "ClosestPoint found nothing within 6 units of a wall 2 units away.");
            Assert.AreEqual(
                near,
                result.entity,
                $"ClosestPoint returned the wrong nearest entity: got {result.entity}, expected the near wall "
                    + $"{near} (far was {far})."
            );
            Assert.Less(
                abs(result.distance - 2f),
                0.05f,
                $"ClosestPoint distance {result.distance:F3} is not near the expected 2 (origin to wall face X=2)."
            );
            Assert.Less(
                abs(result.point.x - 2f),
                0.1f,
                $"ClosestPoint surface point X={result.point.x:F3} is not near the wall face X=2."
            );
            Assert.Greater(
                result.normal.x,
                0.5f,
                $"ClosestPoint normal {result.normal} does not point from the query point toward the body (+X)."
            );

            // Nothing within a search radius smaller than the 2-unit gap.
            var foundClose = PhysicsQueries2D.ClosestPoint(
                pw,
                new float2(0f, 0f),
                maxDistance: 1f,
                hitLayerMask: 0ul,
                hits,
                out _
            );
            Assert.IsFalse(foundClose, "ClosestPoint with a 1-unit search radius reported a wall 2 units away.");

            Debug.Log(
                $"[PHYSICS2D-CLOSEST] nearest entity={result.entity} point={result.point} "
                    + $"distance={result.distance:F3} normal={result.normal}; sub-gap radius found nothing."
            );

            world.Dispose();
            yield break;
        }
    }
}
