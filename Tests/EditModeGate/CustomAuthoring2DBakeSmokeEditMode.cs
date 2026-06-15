using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using Zori.Entities.Physics2D.Tests.Editor;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>CustomAuthoring2DBakeSmoke</c>: the <c>CustomAuthoring2D</c> bake fixture (nine
    /// authored GameObjects — the five shape kinds, a material-template + per-field-override body, a filtered
    /// pair, a static box floor, and a static edge wall) streams and bakes, and the authored fields land on the
    /// baked <see cref="PhysicsShape2D"/> components. Loaded synchronously by the EditMode harness; the bake
    /// assertions (counts + spot-checks) are copied verbatim from the PlayMode gate. This is a bake smoke only —
    /// no simulation step.
    /// </summary>
    public sealed class CustomAuthoring2DBakeSmokeEditMode : Physics2DEditModeHarness
    {
        const int ExpectedShapes = 9; // floor + edge + 7 body shapes

        // Every GameObject bakes a body definition: the 7 dynamic bodies AND the 2 collider-only static GameObjects
        // (floor + edge) through the shape baker's static-body fallback. So the definition count is the full 9.
        const int ExpectedBodyDefinitions = 9;
        const int FilterCategory = 8;

        [Test]
        public void Scene_Bakes_WithAuthoredFields()
        {
            LoadSubScene(Physics2DFixtures.CustomAuthoring2D, "CustomAuthoring2D");

            var shapeQuery = Query(ComponentType.ReadOnly<PhysicsShape2D>(), ComponentType.ReadOnly<LocalToWorld>());
            var bodyQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            var shapeCount = shapeQuery.CalculateEntityCount();
            Assert.AreEqual(
                ExpectedShapes,
                shapeCount,
                $"Expected {ExpectedShapes} baked shapes, saw {shapeCount} — the "
                    + "fixture SubScene did not stream/bake the full set. Build it first via "
                    + "CustomAuthoring2DSceneBuilder.BuildScene."
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
                $"[CA2D-BAKE-SMOKE-EDITMODE] shapes={shapeCount} bodies={bodyCount} "
                    + $"materialTemplateBaked={materialBodyFound} filteredPair={filteredPairCount}"
            );
        }
    }
}
