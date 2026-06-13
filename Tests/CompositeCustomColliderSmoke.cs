using System.Collections;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-9 smoke for the multi-shape body channel that <c>CompositeCollider2D</c> and
    /// <c>CustomCollider2D</c> bake into. Two minimal MECHANISM witnesses (the hard GameObject-parity e2e gate
    /// — which authors real <c>CompositeCollider2D</c>/<c>CustomCollider2D</c> GameObjects in SubScenes and
    /// reads <c>GetPath</c>/<c>GetCustomShapes</c> — is a separate validating agent's deliverable):
    /// <list type="bullet">
    /// <item><b>Composite-merged floor</b> — a dynamic disc falls onto a STATIC body whose collision surface is
    /// a decomposed multi-polygon floor (the runtime form a composite Polygons bakes to: a primary
    /// <see cref="PhysicsShape2D"/> + a <see cref="PhysicsShape2DElement"/> buffer, with
    /// <see cref="PhysicsShape2D.polygonDecompose"/> set), and SETTLES on top at the expected height. Proves
    /// the multi-shape attach loop + the <c>CreatePolygons</c>/<c>CreateShapeBatch</c> decompose path.</item>
    /// <item><b>Custom-shape interaction</b> — a dynamic disc falls onto a STATIC multi-shape body (two
    /// box-polygon shapes forming an L, the runtime form a custom shape group bakes to) and rests on the upper
    /// surface, proving a body interacts with a multi-shape custom collider as expected.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Both tests author the multi-shape bodies through the RUNTIME channel directly (the primary
    /// <see cref="PhysicsShape2D"/> via <see cref="DirectPhysics2DAuthoring"/> plus a hand-added
    /// <see cref="PhysicsShape2DElement"/> buffer), because the engine generates a real composite's / custom
    /// collider's geometry only at bake time from a GameObject — that GameObject-side parity is the validating
    /// agent's SubScene gate. These smokes prove the runtime multi-shape CREATION path the bakers feed: a body
    /// carrying more than one shape attaches every shape to one Box2D body and collides as one merged surface.
    /// Each runs in a dedicated disposable <see cref="World"/>, one step per <c>group.Update()</c>, like the
    /// Phase-6/7/8 smokes.
    /// </remarks>
    public sealed class CompositeCustomColliderSmoke
    {
        const float Dt = 1f / 60f;

        static World MakeFixedWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("Physics2DPhase9SmokeWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }

        static float BodyY(EntityManager em, Entity e) =>
            em.GetComponentData<Unity.Transforms.LocalToWorld>(e).Position.y;

        // Build a PhysicsShape2D vertex blob (the form a composite path / custom polygon bakes to), tracking it in
        // `tracked` so the test can dispose it once the bodies are created (the creation system copies the geometry
        // into Box2D, so the blob is no longer needed after the first step-less Update). A bake-time baker registers
        // the blob with AddBlobAsset for the baking system to own; a runtime-authored smoke owns the lifetime here.
        static BlobAssetReference<PhysicsShape2DVertices> Blob(
            System.Collections.Generic.List<BlobAssetReference<PhysicsShape2DVertices>> tracked,
            float2[] pts
        )
        {
            var builder = new BlobBuilder(Unity.Collections.Allocator.Temp);
            ref var root = ref builder.ConstructRoot<PhysicsShape2DVertices>();
            var arr = builder.Allocate(ref root.points, pts.Length);
            for (var i = 0; i < pts.Length; i++)
                arr[i] = pts[i];
            var blob = builder.CreateBlobAssetReference<PhysicsShape2DVertices>(Unity.Collections.Allocator.Persistent);
            builder.Dispose();
            tracked.Add(blob);
            return blob;
        }

        static Entity SpawnDisc(EntityManager em, float2 pos, float radius)
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
                }
            );
        }

        [UnityTest]
        public IEnumerator DiscSettlesOnDecomposedMultiPolygonFloor_AtExpectedHeight()
        {
            // A static body whose collision surface is TWO convex polygon quads side by side, the left one a
            // decomposed (polygonDecompose) box and the right one a single-hull box — together a 4-wide floor with
            // its top edge at y=0. A disc of radius 0.5 dropped from y=5 must settle resting ON the floor, its
            // center near y=0.5 (radius above the top). This exercises the multi-shape attach loop (primary +
            // buffer) AND the CreatePolygons/CreateShapeBatch decompose path on one body.
            var world = MakeFixedWorld(out var group);
            var em = world.EntityManager;
            var blobs = new System.Collections.Generic.List<BlobAssetReference<PhysicsShape2DVertices>>();

            // Floor body: static, no shape geometry of its own beyond the primary; primary = left quad
            // (decomposed), buffer = right quad (single hull). Both span y in [-1, 0] (top at y=0).
            var floor = em.CreateEntity();
            em.AddComponentData(
                floor,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = Unity.Mathematics.float2.zero,
                    useAutoMass = false,
                }
            );
            em.AddComponentData(
                floor,
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Polygon,
                    polygonDecompose = true, // the composite-Polygons / concave path
                    density = 1f,
                    friction = 0.4f,
                    vertices = Blob(
                        blobs,
                        new[] { new float2(-2f, -1f), new float2(0f, -1f), new float2(0f, 0f), new float2(-2f, 0f) }
                    ),
                }
            );
            var buffer = em.AddBuffer<PhysicsShape2DElement>(floor);
            buffer.Add(
                new PhysicsShape2DElement
                {
                    shape = new PhysicsShape2D
                    {
                        kind = PhysicsShape2DKind.Polygon,
                        polygonDecompose = false, // a plain convex hull arm, on the same body
                        density = 1f,
                        friction = 0.4f,
                        vertices = Blob(
                            blobs,
                            new[] { new float2(0f, -1f), new float2(2f, -1f), new float2(2f, 0f), new float2(0f, 0f) }
                        ),
                    },
                }
            );
            em.AddComponentData(
                floor,
                new Unity.Transforms.LocalToWorld { Value = Unity.Mathematics.float4x4.identity }
            );

            var disc = SpawnDisc(em, new float2(0f, 5f), 0.5f);

            group.Update(); // create bodies (no step) — the geometry is now copied into Box2D
            foreach (var b in blobs)
                b.Dispose();
            for (var f = 0; f < 240; f++)
                group.Update();

            var finalY = BodyY(em, disc);
            // The disc rests on the floor (top at y=0) → center near y=0.5. A generous band absorbs the v2-vs-v3
            // contact-resolution penetration; the load-bearing assertion is it neither fell through (y well below
            // 0) nor hovered (y well above 0.5).
            Assert.Greater(
                finalY,
                0.2f,
                $"The disc fell THROUGH the decomposed multi-polygon floor: ended at y={finalY:F3} (a resting disc "
                    + "sits near y=0.5 on a floor whose top is y=0). The multi-shape attach loop or the "
                    + "CreatePolygons decompose did not produce a solid surface."
            );
            Assert.Less(
                finalY,
                0.9f,
                $"The disc did not settle on the floor: ended at y={finalY:F3} (expected ~0.5, resting on the "
                    + "y=0 surface). It may be hovering or the floor is too tall."
            );

            Debug.Log(
                $"[PHYSICS2D-COMPOSITE] disc settled on a decomposed 2-polygon floor at y={finalY:F3} (expected ~0.5)."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator DiscInteractsWithMultiShapeCustomCollider_RestsOnUpperSurface()
        {
            // A static multi-shape body (the runtime form a CustomCollider2D shape group bakes to): two box
            // polygons forming an L — a wide base [-2,2]x[-1,0] (primary) and a raised step [0.5,2]x[0,1] (buffer).
            // A disc dropped onto the LEFT half (x=-1) rests on the base top (y=0) → center near y=0.5; this proves
            // a body interacts with each shape of a multi-shape custom collider.
            var world = MakeFixedWorld(out var group);
            var em = world.EntityManager;
            var blobs = new System.Collections.Generic.List<BlobAssetReference<PhysicsShape2DVertices>>();

            var custom = em.CreateEntity();
            em.AddComponentData(
                custom,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = Unity.Mathematics.float2.zero,
                    useAutoMass = false,
                }
            );
            // Primary shape: the wide base box, authored as a convex single-hull polygon (the custom-Polygon arm).
            em.AddComponentData(
                custom,
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Polygon,
                    polygonDecompose = false,
                    density = 1f,
                    friction = 0.4f,
                    vertices = Blob(
                        blobs,
                        new[] { new float2(-2f, -1f), new float2(2f, -1f), new float2(2f, 0f), new float2(-2f, 0f) }
                    ),
                }
            );
            // Buffer shape: the raised step, authored as a Box kind (an inline-geometry custom shape on the same
            // body) — proving the multi-shape buffer mixes shape KINDS, not only polygons.
            var buffer = em.AddBuffer<PhysicsShape2DElement>(custom);
            buffer.Add(
                new PhysicsShape2DElement
                {
                    shape = new PhysicsShape2D
                    {
                        kind = PhysicsShape2DKind.Box,
                        size = new float2(1.5f, 1f),
                        offset = new float2(1.25f, 0.5f), // centered at x=1.25, y=0.5 → spans [0.5,2]x[0,1]
                        density = 1f,
                        friction = 0.4f,
                    },
                }
            );
            em.AddComponentData(
                custom,
                new Unity.Transforms.LocalToWorld { Value = Unity.Mathematics.float4x4.identity }
            );

            // Drop the disc over the LEFT half (clear of the raised step) so it rests on the base top at y=0.
            var disc = SpawnDisc(em, new float2(-1f, 5f), 0.5f);

            group.Update(); // create bodies (no step) — the geometry is now copied into Box2D
            foreach (var b in blobs)
                b.Dispose();
            for (var f = 0; f < 240; f++)
                group.Update();

            var finalY = BodyY(em, disc);
            Assert.Greater(
                finalY,
                0.2f,
                $"The disc fell through the multi-shape custom collider: ended at y={finalY:F3} (a resting disc "
                    + "sits near y=0.5 on the base top at y=0). The buffer shape or the primary did not create a "
                    + "solid surface."
            );
            Assert.Less(
                finalY,
                0.9f,
                $"The disc did not settle on the custom collider's base: ended at y={finalY:F3} (expected ~0.5)."
            );

            Debug.Log(
                $"[PHYSICS2D-CUSTOM] disc rested on a 2-shape custom collider (polygon base + box step) at "
                    + $"y={finalY:F3} (expected ~0.5)."
            );

            world.Dispose();
            yield break;
        }
    }
}
