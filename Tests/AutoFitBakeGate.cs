using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-C end-to-end BAKE-CORRECTNESS gate for the auto-fit utility. The fixture
    /// (<c>AutoFitBakeFixtureBuilder</c>) populates custom-authoring shapes by running the REAL
    /// <see cref="Authoring.PhysicsShape2DAutoFit"/> at build time, then bakes them through the normal SubScene
    /// importer. This gate reads the baked <see cref="PhysicsShape2D"/> components and proves:
    /// <list type="bullet">
    /// <item>a fitted Box bakes to the SAME <c>PhysicsShape2D.size</c> as a hand-authored Box of the identical
    /// local geometry — bit-identical, the unscaled-local-fields claim through the real baker;</item>
    /// <item>a fitted falling Circle baked with the auto-fit radius actually COLLIDES at its fitted extent —
    /// it rests on the floor at <c>floorTop + fittedRadius</c>, not interpenetrating and not floating.</item>
    /// </list>
    /// </summary>
    public sealed class AutoFitBakeGate
    {
        const int LoadTimeoutFrames = 600;
        const float Dt = 1f / 60f;
        const string ParentScenePath = "Assets/EntitiesPhysics2DFixture/AutoFitBake.unity";

        // Mirror of AutoFitBakeFixtureBuilder constants (runtime asmdef cannot reference the Editor builder).
        const float BoxW = 4f;
        const float BoxH = 2f;
        const float XFitted = -10f;
        const float XHand = 10f;
        const float CircleRadius = 1f;
        const float CircleStartX = 0f;
        const float FloorTop = 0.5f;

        const float Eps = 1e-5f;

        [UnityTest]
        public IEnumerator FittedBox_BakesBitIdenticalToHandAuthored_AndFittedCircleRestsAtFittedRadius()
        {
            SceneManager.LoadScene(ParentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");
            var em = world.EntityManager;

            // Disable the simulation group while we wait for the SubScene to stream + bake, so the dynamic
            // circle does not pre-step a nondeterministic number of times before we drive it.
            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            fixedGroup.Enabled = false;

            var shapeQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );

            var framesWaited = 0;
            while (shapeQuery.CalculateEntityCount() < 4 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            var count = shapeQuery.CalculateEntityCount();
            Assert.GreaterOrEqual(
                count,
                4,
                $"Only {count} baked shapes after {framesWaited} frames — build the fixture first via "
                    + "-executeMethod Zori.Entities.Physics2D.Tests.Editor.AutoFitBakeFixtureBuilder.Build."
            );

            using var shapes = shapeQuery.ToComponentDataArray<PhysicsShape2D>(Allocator.Temp);
            using var defs = shapeQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);

            var haveFitted = false;
            var haveHand = false;
            PhysicsShape2D fittedBox = default;
            PhysicsShape2D handBox = default;
            for (var i = 0; i < shapes.Length; i++)
            {
                if (shapes[i].kind != PhysicsShape2DKind.Box)
                    continue;
                var x = defs[i].initialPosition.x;
                if (abs(x - XFitted) < 0.25f)
                {
                    fittedBox = shapes[i];
                    haveFitted = true;
                }
                else if (abs(x - XHand) < 0.25f)
                {
                    handBox = shapes[i];
                    haveHand = true;
                }
            }

            Assert.IsTrue(
                haveFitted && haveHand,
                $"Missing a baked box body (fitted={haveFitted}, hand={haveHand}). Fixture not built correctly."
            );

            Debug.Log(
                "[PHYSICS2D-PHASEC-AUTOFIT-BAKE] "
                    + $"fittedBox(size={fittedBox.size} angleRad={fittedBox.boxAngleRadians} "
                    + $"radius={fittedBox.radius} offset={fittedBox.offset}) | "
                    + $"handBox(size={handBox.size} angleRad={handBox.boxAngleRadians} "
                    + $"radius={handBox.radius} offset={handBox.offset})"
            );

            // --- BAKE-CORRECTNESS: the fitted box bakes BIT-IDENTICAL to the hand-authored box. ---
            Assert.AreEqual(
                handBox.size.x,
                fittedBox.size.x,
                0f,
                $"Fitted box width {fittedBox.size.x} != hand-authored {handBox.size.x}. The auto-fit emitted a "
                    + "different BoxSize than a hand-author would for the same 4x2 source — the unscaled-local-"
                    + "fields claim is broken."
            );
            Assert.AreEqual(handBox.size.y, fittedBox.size.y, 0f, "Fitted box height != hand-authored.");
            Assert.AreEqual(
                handBox.boxAngleRadians,
                fittedBox.boxAngleRadians,
                Eps,
                "Fitted box angle != hand-authored 0 (axis-aligned rectangle source)."
            );
            Assert.AreEqual(handBox.radius, fittedBox.radius, 0f, "Fitted box corner radius != hand 0.");
            Assert.AreEqual(handBox.offset.x, fittedBox.offset.x, Eps, "Fitted box offset.x != hand.");
            Assert.AreEqual(handBox.offset.y, fittedBox.offset.y, Eps, "Fitted box offset.y != hand.");

            // Self-check: the baked size must actually equal the source extent (not vacuously equal because both
            // are zero or a default).
            Assert.AreEqual(
                BoxW,
                fittedBox.size.x,
                Eps,
                $"Fitted box width {fittedBox.size.x} != the 4-unit source extent."
            );
            Assert.AreEqual(BoxH, fittedBox.size.y, Eps, "Fitted box height != the 2-unit source extent.");

            // --- COLLIDES AT FITTED EXTENT: drive the dynamic circle; it must rest at floorTop + fittedRadius. ---
            // Confirm the fitted circle baked the right radius first.
            var haveCircle = false;
            PhysicsShape2D fittedCircle = default;
            for (var i = 0; i < shapes.Length; i++)
                if (shapes[i].kind == PhysicsShape2DKind.Circle)
                {
                    fittedCircle = shapes[i];
                    haveCircle = true;
                }
            Assert.IsTrue(haveCircle, "No baked Circle shape — the fitted falling circle is missing.");
            Assert.AreEqual(
                CircleRadius,
                fittedCircle.radius,
                1e-3f,
                $"Fitted circle radius {fittedCircle.radius} != the ring source radius {CircleRadius}."
            );

            // Diagnostic: log every baked body's kind + initial position + size so the rest height can be
            // reconciled against the real floor geometry (the floor top = floorPos.y + size.y/2).
            for (var i = 0; i < shapes.Length; i++)
                Debug.Log(
                    $"[PHYSICS2D-PHASEC-AUTOFIT-BAKE-BODY] kind={shapes[i].kind} "
                        + $"initPos={defs[i].initialPosition} bodyType={defs[i].bodyType} "
                        + $"size={shapes[i].size} radius={shapes[i].radius} offset={shapes[i].offset}"
                );

            // Drive the simulation: re-enable the group with a fixed-rate manager and step until the circle
            // settles, then read its rest height from LocalToWorld.
            var savedRate = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
            fixedGroup.Enabled = true;

            // First update creates the bodies (no integration on the creation frame).
            fixedGroup.Update();

            var dynQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            // Step long enough to fall ~5.5m and settle (300 steps @ 1/60 = 5s).
            var restY = float.NaN;
            for (var s = 0; s < 300; s++)
            {
                fixedGroup.Update();
                restY = ReadDynamicY(dynQuery);
            }

            fixedGroup.RateManager = savedRate;

            Debug.Log(
                $"[PHYSICS2D-PHASEC-AUTOFIT-BAKE] fitted circle rest Y = {restY}, expected ~"
                    + $"{FloorTop + CircleRadius} (floorTop {FloorTop} + fitted radius {CircleRadius})."
            );

            Assert.IsFalse(float.IsNaN(restY), "No dynamic body found — the fitted circle did not bake/create.");

            // The circle must rest with its CENTRE at floorTop + radius (the collision happens at the fitted
            // extent). A radius that baked too small → the circle sinks below; too large → it floats above.
            var expectedY = FloorTop + CircleRadius;
            Assert.AreEqual(
                expectedY,
                restY,
                0.06f,
                $"Fitted circle rested at Y={restY}, not at floorTop+fittedRadius={expectedY}. The body does NOT "
                    + "collide at its fitted extent — a wrong baked radius would sink it (too small) or float it "
                    + "(too large)."
            );

            // Disqualifier: it must actually have fallen (not stuck at its start Y=6).
            Assert.Less(
                restY,
                3f,
                $"Fitted circle never fell (rest Y={restY}) — a no-op bake/create, not a real settle."
            );
            yield break;
        }

        static float ReadDynamicY(EntityQuery dynQuery)
        {
            using var ltws = dynQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            using var defs = dynQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            for (var i = 0; i < ltws.Length; i++)
            {
                if (defs[i].bodyType == Unity.U2D.Physics.PhysicsBody.BodyType.Static)
                    continue;
                return ltws[i].Value.c3.y;
            }
            return float.NaN;
        }
    }
}
