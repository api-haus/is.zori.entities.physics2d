using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-9 GameObject-parity gate for the two collider-authoring paths Phase 9 shipped — a real
    /// <see cref="CompositeCollider2D"/> and a real <see cref="CustomCollider2D"/> baked through the actual
    /// Phase-9 <c>Baker&lt;T&gt;</c>s in a SubScene — plus the <see cref="EdgeCollider2D"/> material/layer
    /// regression. The Phase-9 smoke authored the runtime multi-shape channel directly and never ran the
    /// bakers; this gate bakes the real components and <b>counts</b> baked shapes, so a dead
    /// <c>usedByComposite</c> exclusion guard or a wrong merge is caught by a count assertion, not masked by a
    /// settle pose (a double-baked overlapping merge settles at the same height as a correct single merge).
    /// </summary>
    /// <remarks>
    /// <para><b>The shape-count witnesses (decision points pinned).</b> Each test reads the baked composite/custom
    /// STATIC body entity two ways: (1) the ECS-side authored shape count — the primary <see cref="PhysicsShape2D"/>
    /// (1) plus the optional <c>DynamicBuffer&lt;PhysicsShape2DElement&gt;</c> length — which is one shape per
    /// merged PATH (composite) or per custom SHAPE, the direct witness the baker enumerated paths/shapes (NOT the
    /// child count, NOT zero); and (2) the Box2D-side <c>PhysicsBody.shapeCount</c> + <c>GetShapes</c> geometry-kind
    /// histogram after creation, which pins the geometry kind each path produces (Polygon fragments vs ChainSegments
    /// vs Circle/Capsule). The exclusion-zero witness is the count of baked STATIC body entities: a composite scene
    /// must bake exactly ONE static body (the composite), and zero standalone child shapes — if the exclusion guard
    /// were dead, each merged child would bake its own static body and overlapping shape.</para>
    ///
    /// <para><b>World isolation.</b> Each SubScene test loads its parent scene with <c>LoadSceneMode.Single</c>,
    /// which tears the prior scene's entities out of the default world before the next bakes, and disables the
    /// <c>FixedStepSimulationSystemGroup</c> through the bake-wait (the <c>PhysicsParityHarness</c> discipline), so a
    /// thrown test cannot leak stepped bodies into a later one. The settle-envelope assertions reuse
    /// <see cref="PhysicsParityHarness.RunParity"/> against the GameObject <c>Physics2D.Simulate</c> oracle.</para>
    /// </remarks>
    public sealed class Phase9CompositeCustomGate
    {
        const int LoadTimeoutFrames = 600;
        const float Dt = 1f / 60f;

        // -----------------------------------------------------------------------------------------------
        // Shared bake reader: load a parent SubScene, wait for bake, create the Box2D bodies with ONE group
        // Update (no step), and return the baked static body's ECS + Box2D shape facts.

        struct BakedBodyFacts
        {
            public int staticBodyEntityCount; // how many STATIC body entities baked (exclusion witness)
            public int ecsShapeCount; // primary PhysicsShape2D (1) + buffer length, on the static body
            public int box2dShapeCount; // PhysicsBody.shapeCount after creation
            public int circleShapes;
            public int capsuleShapes;
            public int polygonShapes;
            public int segmentShapes; // Segment + ChainSegment (open chain pieces)
        }

        static IEnumerator LoadBakeAndCreate(
            string parentScenePath,
            System.Action<World, EntityManager> onReady
        )
        {
            SceneManager.LoadScene(parentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");
            var em = world.EntityManager;

            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            // Hold the step off through the bake-wait so the disc never integrates before we read counts.
            fixedGroup.Enabled = false;

            // Wait for ANY baked body with a shape (the composite/custom static body + the disc) to stream.
            var bodyQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>()
            );
            var framesWaited = 0;
            while (bodyQuery.CalculateEntityCount() < 1 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.Greater(
                bodyQuery.CalculateEntityCount(),
                0,
                $"No baked body appeared after {framesWaited} frames — the SubScene '{parentScenePath}' did "
                    + "not stream/bake. Build the fixtures first via "
                    + "-executeMethod Zori.Entities.Physics2D.Tests.Editor.Phase9CompositeCustomFixtureBuilder.BuildAll."
            );

            // One group Update creates the Box2D bodies + shapes (PhysicsWorld2DSystem skips its step on the
            // creation frame), so PhysicsBody2D handles exist for the Box2D shape-count read, with no step taken.
            var savedRate = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
            fixedGroup.Enabled = true;
            fixedGroup.Update();

            onReady(world, em);

            // Restore the rate manager + disable so the caller (or a subsequent RunParity) controls stepping.
            fixedGroup.RateManager = savedRate;
            fixedGroup.Enabled = false;
        }

        // Read the single STATIC multi-shape body's facts. The composite/custom body is the static one; the disc
        // is dynamic. Asserts exactly one static body exists (the exclusion witness — a merged child baking its
        // own static body would inflate this).
        static BakedBodyFacts ReadStaticBodyFacts(World world, EntityManager em)
        {
            var facts = new BakedBodyFacts();

            var query = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2D>()
            );
            using var entities = query.ToEntityArray(Allocator.Temp);

            Entity staticEntity = Entity.Null;
            foreach (var e in entities)
            {
                var def = em.GetComponentData<PhysicsBody2DDefinition>(e);
                if (def.bodyType != PhysicsBody.BodyType.Static)
                    continue;
                facts.staticBodyEntityCount++;
                staticEntity = e;
            }

            if (staticEntity == Entity.Null)
                return facts; // caller asserts staticBodyEntityCount

            // ECS-side authored shape count: primary (1) + optional buffer length.
            facts.ecsShapeCount = 1;
            if (em.HasBuffer<PhysicsShape2DElement>(staticEntity))
                facts.ecsShapeCount += em.GetBuffer<PhysicsShape2DElement>(staticEntity).Length;

            // Box2D-side shape count + kind histogram on the created body.
            var body = em.GetComponentData<PhysicsBody2D>(staticEntity).body;
            if (body.isValid)
            {
                facts.box2dShapeCount = body.shapeCount;
                var shapes = body.GetShapes(Allocator.Temp);
                for (var i = 0; i < shapes.Length; i++)
                {
                    switch (shapes[i].shapeType)
                    {
                        case PhysicsShape.ShapeType.Circle:
                            facts.circleShapes++;
                            break;
                        case PhysicsShape.ShapeType.Capsule:
                            facts.capsuleShapes++;
                            break;
                        case PhysicsShape.ShapeType.Polygon:
                            facts.polygonShapes++;
                            break;
                        case PhysicsShape.ShapeType.Segment:
                        case PhysicsShape.ShapeType.ChainSegment:
                            facts.segmentShapes++;
                            break;
                    }
                }
                if (shapes.IsCreated)
                    shapes.Dispose();
            }

            return facts;
        }

        // -----------------------------------------------------------------------------------------------
        // (A) Composite — Polygons (concave L): the shape-COUNT + exclusion-zero + decompose-kind witnesses.

        [UnityTest]
        public IEnumerator CompositePolygons_BakesOneStaticBodyWithDecomposedConvexFragments()
        {
            BakedBodyFacts facts = default;
            yield return LoadBakeAndCreate(
                "Assets/EntitiesPhysics2DFixture/P9CompositePolygons.unity",
                (world, em) => facts = ReadStaticBodyFacts(world, em)
            );

            Debug.Log(
                $"[PHYSICS2D-P9-COMPOSITE-POLY] staticBodies={facts.staticBodyEntityCount} "
                    + $"ecsShapes={facts.ecsShapeCount} box2dShapes={facts.box2dShapeCount} "
                    + $"poly={facts.polygonShapes} seg={facts.segmentShapes} circ={facts.circleShapes} "
                    + $"cap={facts.capsuleShapes}"
            );

            // Exclusion witness: exactly ONE static body baked. If the usedByComposite guard were dead, each of
            // the 3 merged child boxes would bake its own static body + overlapping shape — i.e. >= 4 static
            // bodies. Exactly one means the children contributed ZERO standalone bodies/shapes.
            Assert.AreEqual(
                1,
                facts.staticBodyEntityCount,
                $"Expected exactly ONE baked static body (the composite). Saw {facts.staticBodyEntityCount}. "
                    + "More than one means the usedByComposite exclusion guard is dead and the merged child "
                    + "boxes each baked their own standalone static body."
            );

            // ECS shape count is one PhysicsShape2D per merged PATH, NOT the 3-child count and NOT zero. A
            // tilemap-style L merges to a single closed path, so the baker emits exactly one primary shape and
            // no buffer (ecsShapeCount == 1). (A merge that produced multiple disjoint paths would be > 1; the
            // load-bearing fact is it is the PATH count, decoupled from the 3 child boxes.)
            Assert.AreEqual(
                1,
                facts.ecsShapeCount,
                $"Composite L merged to {facts.ecsShapeCount} ECS shapes; expected 1 (one PhysicsShape2D per "
                    + "merged path — an L is a single closed path). NOT the 3-child count, NOT zero."
            );

            // The geometry kind a Polygons composite produces: decomposed convex Polygon fragments, NOT chain
            // segments. A concave L cannot be one convex hull, so CreatePolygons must yield >= 2 Polygon
            // fragments. Zero polygons (or any chain segments) means the Polygons->Polygon+decompose mapping is
            // wrong.
            Assert.GreaterOrEqual(
                facts.polygonShapes,
                2,
                $"Composite Polygons L produced {facts.polygonShapes} Box2D Polygon shapes; expected >= 2 "
                    + "convex fragments from CreatePolygons decomposing the concave L. A single fragment means "
                    + "the concave path was NOT decomposed; zero means the Polygon mapping failed."
            );
            Assert.AreEqual(
                0,
                facts.segmentShapes,
                $"Composite Polygons produced {facts.segmentShapes} chain segments; expected 0 (Polygons bakes "
                    + "to solid Polygon fragments, not a chain — Outlines bakes the chain)."
            );
            Assert.AreEqual(facts.box2dShapeCount, facts.polygonShapes, "All Box2D shapes should be polygons.");
        }

        [UnityTest]
        public IEnumerator CompositePolygons_DiscSettlesOnMergedSurface_ParityWithGameObject()
        {
            // Settle envelope vs the GameObject Physics2D.Simulate oracle: the disc rests on the merged L base
            // top (y=0) -> center ~0.5. The composite floor is a STATIC rigidbody, excluded from the compared
            // set; the dynamic disc is the one compared body. A generous band absorbs v2-vs-v3 contact noise.
            yield return PhysicsParityHarness.RunParity(
                "P9CompositePolygons",
                "P9CompositePolygons_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.05f,
                    positionGrowthPerStep = 0.01f,
                    // The compared body is a symmetric CIRCLE — its rotation angle carries no physical
                    // observable, and the v3 sub-stepping solver lets a settled disc roll slightly in/near the
                    // concave merge corner where the v2 GameObject solver holds it still (measured ~0.5 rad
                    // cross-solver roll). A circle's angle parity is therefore the wrong disqualifier here; the
                    // position band + settle region + travel do the load-bearing correctness work. Wide cap
                    // absorbs the irrelevant cross-solver disc roll.
                    angleCapRadians = 1.2f,
                    settleRegionMin = new float2(-2.5f, 0.2f),
                    settleRegionMax = new float2(0.5f, 1.2f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // -----------------------------------------------------------------------------------------------
        // (A) Composite — Outlines (closed ring): the chain-loop kind witness.

        [UnityTest]
        public IEnumerator CompositeOutlines_BakesOneStaticBodyWithClosedChainSegments()
        {
            BakedBodyFacts facts = default;
            yield return LoadBakeAndCreate(
                "Assets/EntitiesPhysics2DFixture/P9CompositeOutlines.unity",
                (world, em) => facts = ReadStaticBodyFacts(world, em)
            );

            Debug.Log(
                $"[PHYSICS2D-P9-COMPOSITE-OUT] staticBodies={facts.staticBodyEntityCount} "
                    + $"ecsShapes={facts.ecsShapeCount} box2dShapes={facts.box2dShapeCount} "
                    + $"poly={facts.polygonShapes} seg={facts.segmentShapes}"
            );

            Assert.AreEqual(
                1,
                facts.staticBodyEntityCount,
                $"Expected exactly ONE baked static body (the composite). Saw {facts.staticBodyEntityCount} — "
                    + "the usedByComposite exclusion guard is dead if > 1."
            );

            // Outlines bakes one Edge shape (kind) per merged path; a SOLID bar merges to a single closed outer
            // outline -> one ECS shape. (NOT the 3-child count, NOT zero. A hollow frame would merge to outer +
            // inner = 2 paths, which is why the fixture is a solid bar.)
            Assert.AreEqual(
                1,
                facts.ecsShapeCount,
                $"Composite bar merged to {facts.ecsShapeCount} ECS shapes; expected 1 (one Edge shape per "
                    + "merged outline path; a solid bar is a single closed outline). NOT the 3-child count, "
                    + "NOT zero."
            );

            // The geometry kind: a closed loop chain decomposes into ChainSegment shapes (one per edge of the
            // loop), NOT Polygon shapes. The merged rectangular outline has >= 4 corners -> >= 4 chain segments.
            // Zero segments (or any polygons) means Outlines->Edge(loop)->CreateChain mapping is wrong.
            Assert.GreaterOrEqual(
                facts.segmentShapes,
                4,
                $"Composite Outlines bar produced {facts.segmentShapes} Box2D chain segments; expected >= 4 "
                    + "(a closed rectangular outline loop). Zero means the Outlines->loop-chain mapping failed."
            );
            Assert.AreEqual(
                0,
                facts.polygonShapes,
                $"Composite Outlines produced {facts.polygonShapes} polygon shapes; expected 0 (Outlines bakes "
                    + "a chain loop, not solid polygons — Polygons bakes the polygons)."
            );
        }

        [UnityTest]
        public IEnumerator CompositeOutlines_DiscRestsOnLoopSurface_ParityWithGameObject()
        {
            // The disc rests on the ring's top edge (y=0) -> center ~0.5. This is the Outlines one-sidedness /
            // winding probe vs the GameObject oracle: a wrong-winding loop would let the disc fall through.
            yield return PhysicsParityHarness.RunParity(
                "P9CompositeOutlines",
                "P9CompositeOutlines_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.05f,
                    positionGrowthPerStep = 0.01f,
                    angleCapRadians = 1.2f, // symmetric disc: roll angle is not a parity observable (see CompositePolygons)
                    settleRegionMin = new float2(-1.0f, 0.2f),
                    settleRegionMax = new float2(1.0f, 1.2f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // -----------------------------------------------------------------------------------------------
        // (B) Custom — known PhysicsShapeGroup2D: the shape-count + per-kind witnesses.

        [UnityTest]
        public IEnumerator CustomGroup_BakesOneShapePerCustomShape_KindsMatchGroup()
        {
            BakedBodyFacts facts = default;
            yield return LoadBakeAndCreate(
                "Assets/EntitiesPhysics2DFixture/P9CustomGroup.unity",
                (world, em) => facts = ReadStaticBodyFacts(world, em)
            );

            Debug.Log(
                $"[PHYSICS2D-P9-CUSTOM] staticBodies={facts.staticBodyEntityCount} "
                    + $"ecsShapes={facts.ecsShapeCount} box2dShapes={facts.box2dShapeCount} "
                    + $"poly={facts.polygonShapes} circ={facts.circleShapes} cap={facts.capsuleShapes} "
                    + $"seg={facts.segmentShapes}"
            );

            Assert.AreEqual(
                1,
                facts.staticBodyEntityCount,
                $"Expected exactly ONE baked static body (the custom collider). Saw {facts.staticBodyEntityCount}."
            );

            // The group has exactly 3 shapes (Polygon + Circle + Capsule), so the baker emits 3 ECS shapes:
            // one primary + two buffer elements. NOT zero, NOT some other count.
            Assert.AreEqual(
                3,
                facts.ecsShapeCount,
                $"Custom group baked {facts.ecsShapeCount} ECS shapes; expected 3 (one per custom shape: "
                    + "Polygon + Circle + Capsule)."
            );
            Assert.AreEqual(
                3,
                facts.box2dShapeCount,
                $"Custom group created {facts.box2dShapeCount} Box2D shapes; expected 3."
            );

            // Each kind appears exactly once — the per-shapeType mapping (Polygon/Circle/Capsule) is correct.
            Assert.AreEqual(1, facts.polygonShapes, "Expected exactly 1 Polygon shape from the custom group.");
            Assert.AreEqual(1, facts.circleShapes, "Expected exactly 1 Circle shape from the custom group.");
            Assert.AreEqual(1, facts.capsuleShapes, "Expected exactly 1 Capsule shape from the custom group.");
        }

        [UnityTest]
        public IEnumerator CustomGroup_DiscRestsOnPolygonBase_ParityWithGameObject()
        {
            // The disc rests on the custom polygon base top (y=0) -> center ~0.5, proving the baked custom
            // shapes form a solid interactable surface matching the GameObject CustomCollider2D oracle.
            yield return PhysicsParityHarness.RunParity(
                "P9CustomGroup",
                "P9CustomGroup_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.05f,
                    positionGrowthPerStep = 0.01f,
                    angleCapRadians = 1.2f, // symmetric disc: roll angle is not a parity observable (see CompositePolygons)
                    settleRegionMin = new float2(-1.0f, 0.2f),
                    settleRegionMax = new float2(1.0f, 1.2f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // -----------------------------------------------------------------------------------------------
        // (C) EdgeCollider2D material + layer regression pins.

        [UnityTest]
        public IEnumerator EdgeMaterial_BouncyEdgeReboundsDisc_AndFilteredDiscPassesThrough()
        {
            // Own the layer matrix in SetUp-equivalent fashion: disable EdgeLayer<->PassThroughDiscLayer so the
            // baked chain's contactFilter must exclude the pass-through disc. (The SubScene bakes live and reads
            // the persisted/runtime matrix; we set it before LoadScene streams.) Save/restore to leave no global
            // state behind.
            const int edgeLayer = 8;
            const int passLayer = 9;
            var prevIgnore = UnityEngine.Physics2D.GetIgnoreLayerCollision(edgeLayer, passLayer);
            UnityEngine.Physics2D.IgnoreLayerCollision(edgeLayer, passLayer, true);

            float bouncingFinalY = 0f;
            float bouncingMinY = float.PositiveInfinity; // the contact low point (first descent onto the edge)
            float reboundMaxY = float.NegativeInfinity; // the highest y AFTER the contact low point
            var hitContact = false; // crossed below the contact threshold at least once
            float passThroughFinalY = 0f;
            var sawBouncing = false;
            var sawPassThrough = false;

            // Drive the scene directly (not RunParity — this asserts a bounce trajectory + a pass-through, not a
            // settle envelope). Load, bake, create, then step and track the dynamic discs by their start x.
            SceneManager.LoadScene("Assets/EntitiesPhysics2DFixture/P9EdgeMaterial.unity", LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world.");
            var em = world.EntityManager;
            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.Enabled = false;

            var dynQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>(),
                ComponentType.ReadOnly<PhysicsShape2D>()
            );
            var framesWaited = 0;
            while (dynQuery.CalculateEntityCount() < 3 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.GreaterOrEqual(
                dynQuery.CalculateEntityCount(),
                3,
                $"Edge-material fixture did not bake its 3 bodies (edge + 2 discs); saw "
                    + $"{dynQuery.CalculateEntityCount()} after {framesWaited} frames. Build the fixtures first."
            );

            var savedRate = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
            fixedGroup.Enabled = true;
            fixedGroup.Update(); // create bodies, no step

            // Identify the two dynamic discs by start x: bouncing at x~0, pass-through at x~5.
            Entity bouncing = Entity.Null;
            Entity passThrough = Entity.Null;
            using (var ents = dynQuery.ToEntityArray(Allocator.Temp))
            {
                foreach (var e in ents)
                {
                    var def = em.GetComponentData<PhysicsBody2DDefinition>(e);
                    if (def.bodyType != PhysicsBody.BodyType.Dynamic)
                        continue;
                    if (abs(def.initialPosition.x - 0f) < 1f)
                    {
                        bouncing = e;
                        sawBouncing = true;
                    }
                    else if (abs(def.initialPosition.x - 5f) < 1f)
                    {
                        passThrough = e;
                        sawPassThrough = true;
                    }
                }
            }
            Assert.IsTrue(sawBouncing && sawPassThrough, "Did not find both dynamic discs by start x.");

            // Contact threshold: the disc (radius 0.5) rests on the edge (y=0) at center y~0.5; once it has
            // descended below y=1.0 it has reached the edge. The rebound is the highest y reached AFTER that
            // first contact — a zero-restitution (dropped-material) disc cannot rebound, so this separates a
            // real 0.85-bounce from the start-height artifact (the disc starts at y=4, so a naive max-y over
            // the whole run is trivially ~4 regardless of the material).
            const float contactThreshold = 1.0f;
            for (var s = 0; s < 240; s++)
            {
                fixedGroup.Update();
                var by = em.GetComponentData<LocalToWorld>(bouncing).Position.y;
                if (!hitContact)
                {
                    bouncingMinY = min(bouncingMinY, by);
                    if (by < contactThreshold)
                        hitContact = true;
                }
                else
                {
                    reboundMaxY = max(reboundMaxY, by);
                }
                bouncingFinalY = by;
                passThroughFinalY = em.GetComponentData<LocalToWorld>(passThrough).Position.y;
            }

            fixedGroup.RateManager = savedRate;
            fixedGroup.Enabled = false;
            UnityEngine.Physics2D.IgnoreLayerCollision(edgeLayer, passLayer, prevIgnore);

            Debug.Log(
                $"[PHYSICS2D-P9-EDGE] bouncing: startY=4 contactMinY={bouncingMinY:F3} hitContact={hitContact} "
                    + $"reboundMaxY={reboundMaxY:F3} finalY={bouncingFinalY:F3} | "
                    + $"passThrough finalY={passThroughFinalY:F3} (edge at y=0, layers {edgeLayer}x{passLayer} "
                    + "ignored)"
            );

            // The disc must actually reach the edge first (descend below the contact threshold), or the rebound
            // measurement is meaningless.
            Assert.IsTrue(
                hitContact,
                $"The bouncing disc never descended to the edge (min y={bouncingMinY:F3} stayed above "
                    + $"{contactThreshold}). It cannot have bounced off a surface it never reached."
            );

            // Material pin (adversarial): after the disc first hits the edge near y=0.5, a 0.85-bounce surface
            // rebounds it WELL back up. The rebound is measured strictly AFTER the contact low point, so the
            // disc's y=4 start height does not inflate it. The old material-dropping bug left the chain at
            // default (0) restitution -> a contacted disc damps and settles, never rebounding above ~1. Require
            // a post-contact rebound above y=1.8, impossible with a dropped (zero-restitution) material.
            Assert.Greater(
                reboundMaxY,
                1.8f,
                $"After hitting the edge (min y={bouncingMinY:F3}), the disc rebounded only to y={reboundMaxY:F3} "
                    + "(expected > 1.8 from a 0.85-bounce surface). The edge's PhysicsMaterial2D bounciness was "
                    + "NOT applied to the baked chain — the EdgeCollider2D material is being silently dropped "
                    + "(the latent Phase-9 bug regressed)."
            );
            // It must still have rested ON the edge (above it), not fallen through.
            Assert.Greater(
                bouncingFinalY,
                0f,
                $"The bouncing disc ended below the edge (finalY={bouncingFinalY:F3}) — it fell through, the "
                    + "edge surface is not solid."
            );

            // Filter pin: the pass-through disc is on a layer the edge layer ignores, so it falls THROUGH the
            // edge (final y well below 0). If the chain ignored the baked filter, it would rest on the edge
            // (final y > 0).
            Assert.Less(
                passThroughFinalY,
                -1f,
                $"The pass-through disc rested on the edge (finalY={passThroughFinalY:F3}) instead of falling "
                    + "through. The baked chain's contactFilter did NOT honour the disabled "
                    + $"layer-{edgeLayer}x{passLayer} pair — the EdgeCollider2D layer filter is dropped."
            );
        }
    }
}
