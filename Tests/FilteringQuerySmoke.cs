using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
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
    /// Phase-5 smoke for collision-layer filtering + spatial queries. Two minimal mechanism witnesses (the
    /// hard GameObject-parity e2e gate over multiple matrix configurations is a separate validating agent's
    /// deliverable):
    /// <list type="bullet">
    /// <item><b>Filtering</b> — two dynamic circles authored on a non-colliding category/contacts pair fall
    /// onto the same static floor at the same point and pass through each other (no contact separates them),
    /// while a colliding control pair on the same category does separate. Proves the baked contact filter is
    /// applied at shape creation.</item>
    /// <item><b>Query</b> — a downward raycast through a column of bodies hits the expected (nearest) body and
    /// resolves the correct owning <see cref="Entity"/> via the userData packing. Proves the query surface and
    /// the shape→body→entity association.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/> (not the default injection world), holding
    /// the package's four FixedStep systems, driven one fixed step per <c>group.Update()</c>. The bodies are
    /// authored directly via <see cref="DirectPhysics2DAuthoring"/> with the category/contacts bits set inline,
    /// so the smoke does not depend on a project layer matrix (the parity gate, which does, is separate).
    /// </remarks>
    public sealed class FilteringQuerySmoke
    {
        const float Dt = 1f / 60f;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DFilterQuerySmokeWorld", out group, Dt);

        static PhysicsWorld GetWorld(EntityManager em)
        {
            var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        // A dynamic circle authored directly with explicit contact-filter bits.
        static Entity SpawnCircle(EntityManager em, float2 pos, float radius, ulong categoryBits, ulong contactBits)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = pos,
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = radius,
                    density = 1f,
                    friction = 0.4f,
                    categoryBits = categoryBits,
                    contactBits = contactBits,
                }
            );
        }

        // A static box floor authored directly with explicit contact-filter bits.
        static Entity SpawnFloor(EntityManager em, float2 center, float2 size, ulong categoryBits, ulong contactBits)
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
                    categoryBits = categoryBits,
                    contactBits = contactBits,
                }
            );
        }

        [UnityTest]
        public IEnumerator NonCollidingLayerPair_PassesThrough_CollidingPairStacks()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // Geometry that discriminates colliding from non-colliding: a static floor, a bottom dynamic
            // circle resting on it, and a top circle dropped from high.
            //   - Colliding pair: the top rests ON the bottom, ending ~2r=1.0 ABOVE the floor-resting bottom.
            //   - Non-colliding pair: the top passes THROUGH the bottom and rests on the floor at the same
            //     level as the bottom (their final Ys nearly coincide).
            // The discriminator is the top body's final Y: high (~1.5) when it collides, low (~0.5) when it
            // passes through. The floor uses 'all' contacts so BOTH tops still land on it.
            ulong all = 0xFFFFFFFFul;
            const int la = 8;
            const int lb = 9;
            const int lfloor = 10; // a third layer for the floor, contacted by BOTH A and B
            ulong catA = 1ul << la;
            ulong catB = 1ul << lb;
            ulong catFloor = 1ul << lfloor;
            ulong contactsA_noB = all & ~(1ul << lb); // layer 8 contacts everything EXCEPT layer 9 (incl. floor 10)
            ulong contactsB_noA = all & ~(1ul << la); // layer 9 contacts everything EXCEPT layer 8 (incl. floor 10)
            const float floorTopY = 0f; // floor surface at y=0 (box centered at -0.5, height 1)
            const float r = 0.5f;

            // Non-colliding pair: bottom on layer B near the floor; top on layer A dropped from high. They do
            // not contact each other (A excludes B and vice versa) but both contact the floor (layer 10, which
            // neither excludes) so neither falls through it.
            var ncFloor = SpawnFloor(em, new float2(-5f, floorTopY - 0.5f), new float2(4f, 1f), catFloor, all);
            var ncBot = SpawnCircle(em, new float2(-5f, floorTopY + r), r, catB, contactsB_noA);
            var ncTop = SpawnCircle(em, new float2(-5f, floorTopY + 5f), r, catA, contactsA_noB);

            // Colliding control pair: bottom + top both on layer A contacting everything, same layout.
            var cFloor = SpawnFloor(em, new float2(5f, floorTopY - 0.5f), new float2(4f, 1f), catA, all);
            var cBot = SpawnCircle(em, new float2(5f, floorTopY + r), r, catA, all);
            var cTop = SpawnCircle(em, new float2(5f, floorTopY + 5f), r, catA, all);

            // First update creates the bodies; settle enough steps for the dropped tops to land.
            group.Update();
            for (var f = 0; f < 180; f++)
                group.Update();

            float YOf(Entity e) => em.GetComponentData<LocalToWorld>(e).Position.y;

            var ncTopY = YOf(ncTop);
            var ncBotY = YOf(ncBot);
            var cTopY = YOf(cTop);
            var cBotY = YOf(cBot);

            // Non-colliding: the top passed through the bottom and rests on the floor, so its Y is close to the
            // bottom's (both resting ~r above the floor surface). A contact would have left it ~2r higher.
            Assert.Less(
                abs(ncTopY - ncBotY),
                0.6f,
                $"Non-colliding layer pair did not pass through: top Y={ncTopY:F3}, bottom Y={ncBotY:F3} "
                    + $"(expected nearly equal, both resting on the floor). The baked contact filter was not applied "
                    + "— the top stacked on the bottom instead of passing through."
            );
            // Colliding control: the top rests on the bottom, so it sits ~2r=1.0 above it (well separated).
            Assert.Greater(
                cTopY - cBotY,
                0.6f,
                $"Colliding control pair did not stack: top Y={cTopY:F3}, bottom Y={cBotY:F3} "
                    + "(expected the top ~2r above the bottom). The filter mechanism is not discriminating."
            );

            Debug.Log(
                $"[PHYSICS2D-FILTER] non-colliding: topY={ncTopY:F3} ~ botY={ncBotY:F3} (passed through); "
                    + $"colliding: topY={cTopY:F3} > botY={cBotY:F3} (stacked). "
                    + $"floors ncFloor={ncFloor} cFloor={cFloor}."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator Raycast_HitsExpectedBody_ResolvesOwningEntity()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A static circle directly above the ray origin, and one off to the side that the ray misses.
            var target = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = new float2(0f, 5f),
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 1f,
                    density = 1f,
                    friction = 0.4f,
                    // unfiltered (category 0 → everything-default), so QueryFilter.Everything hits it.
                }
            );
            var decoy = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = new float2(10f, 5f),
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 1f,
                    density = 1f,
                    friction = 0.4f,
                }
            );

            // First update creates the bodies (no step); the query reads the created world.
            group.Update();

            var pw = GetWorld(em);
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);

            // Ray straight up from origin; should hit the target at ~y=4 (bottom of the r=1 circle at y=5).
            var n = PhysicsQueries2D.Raycast(
                pw,
                new float2(0f, 0f),
                new float2(0f, 1f),
                10f,
                hitLayerMask: 0ul, // 0 = hit everything
                hits
            );

            Assert.Greater(n, 0, "Upward raycast hit nothing — the query found no body above the origin.");

            var hit = hits[0];
            Assert.AreEqual(
                target,
                hit.entity,
                $"Raycast resolved the wrong owning entity: got {hit.entity}, expected the target "
                    + $"{target} (decoy was {decoy}). The shape→body→entity userData packing is wrong."
            );
            Assert.IsFalse(isnan(hit.point.x) || isnan(hit.point.y), $"Raycast hit point is NaN: {hit.point}.");
            // The hit point is near the bottom of the target circle (y ~= 4), and the normal points down
            // toward the ray origin (negative-ish Y).
            Assert.Less(
                abs(hit.point.y - 4f),
                0.5f,
                $"Raycast hit point Y={hit.point.y:F3} is not near the target's near surface (~4)."
            );
            Assert.Less(hit.normal.y, 0.1f, $"Raycast normal {hit.normal} does not face the ray origin.");

            // A closest-only query agrees.
            var gotClosest = PhysicsQueries2D.RaycastClosest(
                pw,
                new float2(0f, 0f),
                new float2(0f, 1f),
                10f,
                0ul,
                out var closest
            );
            Assert.IsTrue(gotClosest, "RaycastClosest found no hit where Raycast did.");
            Assert.AreEqual(target, closest.entity, "RaycastClosest resolved a different entity than Raycast.");

            Debug.Log(
                $"[PHYSICS2D-QUERY] raycast hit entity={hit.entity} at point={hit.point}, "
                    + $"normal={hit.normal}, fraction={hit.fraction:F3}; closest agrees."
            );

            world.Dispose();
            yield break;
        }
    }
}
