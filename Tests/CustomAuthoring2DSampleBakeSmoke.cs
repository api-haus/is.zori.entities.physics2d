using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The Phase-E sample-scene smoke: the <c>CustomAuthoring2D</c> sample SubScene (nine authored GameObjects —
    /// the five shape kinds, a material-template + per-field-override body, a filtered pair, a static box floor,
    /// and a static edge wall) streams and bakes, and the authored Phase-A/B fields land on the baked
    /// <see cref="PhysicsShape2D"/> components. This proves the shippable sample bakes through the normal importer
    /// (the committed <c>Samples~</c> copy is byte-identical content) and that the complete authoring surface the
    /// docs describe actually produces the expected runtime components.
    /// </summary>
    /// <remarks>
    /// Runtime-only: the scene is authored by the editor builder
    /// <c>CustomAuthoring2DSampleSceneBuilder</c> and registered in build settings, so
    /// <c>SceneManager.LoadScene</c> opens it and the SubScene auto-loads + bakes on PlayMode enter. The test
    /// pumps frames (yield <c>null</c> — <c>WaitForEndOfFrame</c> does not tick in batchmode) until the baked
    /// shapes appear, then asserts the bake outcome. It does not step the simulation — settling is the consumer's
    /// Play, not a bake smoke; the load-bearing fact here is "the authored components bake to the expected
    /// runtime data".
    /// </remarks>
    public sealed class CustomAuthoring2DSampleBakeSmoke
    {
        const string SceneName = "CustomAuthoring2DSample";
        const int LoadTimeoutFrames = 600;

        const int ExpectedShapes = 9; // floor + edge + 7 body shapes
        // Every GameObject bakes a body definition: the 7 dynamic bodies AND the 2 collider-only static GameObjects
        // (floor + edge) through the shape baker's static-body fallback. So the definition count is the full 9.
        const int ExpectedBodyDefinitions = 9;
        const int FilterCategory = 8;

        [UnityTest]
        public IEnumerator SampleScene_Bakes_WithAuthoredFields()
        {
            SceneManager.LoadScene(SceneName, LoadSceneMode.Single);
            yield return null; // let the load + SubScene streaming begin

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");

            var shapeQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var bodyQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            var framesWaited = 0;
            while (
                shapeQuery.CalculateEntityCount() < ExpectedShapes
                && framesWaited < LoadTimeoutFrames
            )
            {
                framesWaited++;
                yield return null;
            }

            var shapeCount = shapeQuery.CalculateEntityCount();
            Assert.AreEqual(
                ExpectedShapes,
                shapeCount,
                $"Expected {ExpectedShapes} baked shapes after {framesWaited} frames, saw {shapeCount} — the "
                    + "sample SubScene did not stream/bake the full set. Build it first via "
                    + "CustomAuthoring2DSampleSceneBuilder.BuildSampleScene."
            );

            // Every authored GameObject bakes a body definition — the 7 dynamic bodies and the 2 collider-only
            // static GameObjects (floor + edge) via the shape baker's static-body fallback.
            var bodyCount = bodyQuery.CalculateEntityCount();
            Assert.AreEqual(
                ExpectedBodyDefinitions,
                bodyCount,
                $"Expected {ExpectedBodyDefinitions} baked body definitions (7 dynamic + 2 static-fallback), "
                    + $"saw {bodyCount}."
            );

            // Spot-check the Phase-A/B authored fields on the baked shapes: the material-template body's inherited
            // bounciness + overridden friction, and the filtered pair's explicit category bits.
            using var shapes = shapeQuery.ToComponentDataArray<PhysicsShape2D>(Allocator.Temp);

            var materialBodyFound = false;
            var filteredPairCount = 0;
            ulong filterBit = 1ul << FilterCategory;
            foreach (var s in shapes)
            {
                // MaterialBody: a Box that inherits bounciness 0.8 from the template and overrides friction to 0.1.
                if (Mathf.Abs(s.bounciness - 0.8f) < 1e-4f && Mathf.Abs(s.friction - 0.1f) < 1e-4f)
                    materialBodyFound = true;

                // The two filtered circles author categoryBits == contactBits == (1 << 8) via OverrideFilterBits.
                if (s.categoryBits == filterBit && s.contactBits == filterBit)
                    filteredPairCount++;
            }

            Assert.IsTrue(
                materialBodyFound,
                "No baked shape carried the material-template body's inherited bounciness 0.8 + overridden "
                    + "friction 0.1 — the Phase-B template inheritance / per-field override did not bake."
            );
            Assert.AreEqual(
                2,
                filteredPairCount,
                "Expected 2 baked shapes with the explicit filter category/contact bits (1 << 8) — the "
                    + "Phase-A OverrideFilterBits pair did not bake its explicit filter."
            );

            Debug.Log(
                $"[CA2D-SAMPLE-SMOKE] shapes={shapeCount} bodies={bodyCount} "
                    + $"materialTemplateBaked={materialBodyFound} filteredPair={filteredPairCount} "
                    + $"loadFrames={framesWaited}"
            );

            // Frame-pumping ticked the per-frame system groups over this world; the default world persists across
            // PlayMode tests, so drain all tracked jobs before returning to keep the suite isolated (no dangling
            // job survives into the next test's world operations).
            world.EntityManager.CompleteAllTrackedJobs();
        }
    }
}
