using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using Zori.Entities.Physics2D.Tests.Editor;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of the FallingBody vertical slice: a SubScene-authored Dynamic <c>Rigidbody2D</c> +
    /// <c>CircleCollider2D</c> bakes to one ECS entity that, under gravity, falls — world-Y strictly decreasing
    /// across fixed steps, no lateral X drift. Proves authoring → bake → create → Simulate → write-back →
    /// LocalToWorld end to end in EditMode. Assertions and step counts copied verbatim from the PlayMode
    /// <c>FallingBodyValidation</c>.
    /// </summary>
    public sealed class FallingBodyValidationEditMode : Physics2DEditModeHarness
    {
        const int SettleSteps = 120;

        [Test]
        public void FallingBody_LocalToWorldYStrictlyDecreases()
        {
            LoadSubScene(Physics2DFixtures.FallingBody, "FallingBody");

            var query = Query(ComponentType.ReadOnly<PhysicsBody2D>(), ComponentType.ReadOnly<LocalToWorld>());
            var bakedQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            Assert.Greater(
                bakedQuery.CalculateEntityCount(),
                0,
                "No baked falling body appeared — the SubScene did not stream/bake."
            );

            // First update: PhysicsWorld2DSystem creates the body (no integration on the creation frame), and the
            // write-back populates LocalToWorld at the authored pose, so the sampled initial Y is the true start.
            CreateBodies();
            Assert.Greater(
                query.CalculateEntityCount(),
                0,
                "Body creation did not run: the baked entity never gained a live PhysicsBody2D after a fixed step."
            );

            float SampleY()
            {
                using var ltw = query.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
                return ltw[0].Position.y;
            }
            float SampleX()
            {
                using var ltw = query.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
                return ltw[0].Position.x;
            }

            var initialY = SampleY();
            var initialX = SampleX();

            var prevY = initialY;
            var sawStrictDecrease = false;
            for (var f = 0; f < SettleSteps; f++)
            {
                Step(1);
                var y = SampleY();
                if (y < prevY - 1e-5f)
                    sawStrictDecrease = true;
                prevY = y;
            }

            var finalY = SampleY();
            var finalX = SampleX();

            Debug.Log(
                $"[PHYSICS2D-SLICE-EDITMODE] initial=( {initialX:F4}, {initialY:F4} ) "
                    + $"final=( {finalX:F4}, {finalY:F4} ) dropped={(initialY - finalY):F4} "
                    + $"sawStrictDecrease={sawStrictDecrease}"
            );

            Assert.Less(
                finalY,
                initialY - 0.5f,
                $"Body did not fall: initialY={initialY}, finalY={finalY}. Gravity step or write-back is not "
                    + "advancing LocalToWorld."
            );
            Assert.IsTrue(
                sawStrictDecrease,
                "World-Y never strictly decreased between consecutive fixed steps — the pose is not advancing."
            );
            Assert.AreEqual(
                initialX,
                finalX,
                1e-3f,
                $"Body drifted laterally: initialX={initialX}, finalX={finalX}. A pure-gravity fall must not move in X."
            );
        }
    }
}
