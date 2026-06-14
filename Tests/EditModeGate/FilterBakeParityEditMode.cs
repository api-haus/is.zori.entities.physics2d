using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using UnityEngine;
using Zori.Entities.Physics2D.Tests.Editor;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>FilterBakeParityGate</c>: four static collider-only circles on distinct layers bake to
    /// bodies whose <c>PhysicsShape2D</c> carries the per-layer category bit and the persisted layer-collision
    /// matrix row. Assertions copied verbatim from the PlayMode gate; the body↔layer key is the baked
    /// <c>initialPosition.y</c>.
    /// </summary>
    public sealed class FilterBakeParityEditMode : Physics2DEditModeHarness
    {
        [Test]
        public void BakedFilter_CarriesPerLayerCategoryAndPersistedMatrixRow()
        {
            LoadSubScene(Physics2DFixtures.FilterBake, "FilterBake");

            var query = Query(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );
            Assert.AreEqual(4, query.CalculateEntityCount(), "Expected 4 baked filter bodies.");

            using var shapes = query.ToComponentDataArray<PhysicsShape2D>(Allocator.Temp);
            using var defs = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);

            var found = new System.Collections.Generic.Dictionary<int, PhysicsShape2D>();
            for (var i = 0; i < shapes.Length; i++)
            {
                var y = defs[i].initialPosition.y;
                if (abs(y - Physics2DFixtures.FbYA) < 0.25f)
                    found[Physics2DFixtures.FbLA] = shapes[i];
                else if (abs(y - Physics2DFixtures.FbYB) < 0.25f)
                    found[Physics2DFixtures.FbLB] = shapes[i];
                else if (abs(y - Physics2DFixtures.FbYDefault) < 0.25f)
                    found[Physics2DFixtures.FbLDefault] = shapes[i];
                else if (abs(y - Physics2DFixtures.FbYX) < 0.25f)
                    found[Physics2DFixtures.FbLX] = shapes[i];
            }

            foreach (
                var layer in new[]
                {
                    Physics2DFixtures.FbLA,
                    Physics2DFixtures.FbLB,
                    Physics2DFixtures.FbLDefault,
                    Physics2DFixtures.FbLX,
                }
            )
            {
                Assert.IsTrue(found.ContainsKey(layer), $"Missing baked body for layer {layer}.");
                var shape = found[layer];
                Assert.AreEqual(
                    (uint)(1 << layer),
                    shape.categoryBits,
                    $"Layer {layer} baked categoryBits {shape.categoryBits:X}, expected {(1u << layer):X}."
                );
                Assert.AreEqual(
                    (uint)UnityEngine.Physics2D.GetLayerCollisionMask(layer),
                    shape.contactBits,
                    $"Layer {layer} baked contactBits {shape.contactBits:X} != the persisted matrix row "
                        + $"{(uint)UnityEngine.Physics2D.GetLayerCollisionMask(layer):X}."
                );
            }
        }
    }
}
