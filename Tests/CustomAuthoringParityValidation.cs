using System.Collections;
using Unity.Mathematics;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-3 parity gates for the customisable authoring surface. The tight gate is the hard-to-falsify
    /// one: a body authored via the custom <see cref="Authoring.PhysicsBody2DAuthoring"/>/<see
    /// cref="Authoring.PhysicsShape2DAuthoring"/> must produce the SAME runtime simulation as the equivalent
    /// built-in-authored body, because both bake to the same <see cref="PhysicsBody2DDefinition"/>/<see
    /// cref="PhysicsShape2D"/> and run the same Box2D-v3 solver in one ECS world. The broad gate runs a
    /// custom-authored scene through the GameObject oracle the built-in path uses.
    /// </summary>
    public sealed class CustomAuthoringParityValidation
    {
        const float Dt = 1f / 60f;

        /// <summary>
        /// TIGHT: custom-authored circle vs built-in-authored circle, both baked into one ECS world, stepped
        /// in lockstep, compared by start-relative displacement. Same solver on both → near-exact agreement.
        /// The tolerance is far below the GameObject v2-vs-v3 band: two identical v3 bodies at the same
        /// fixed dt differ only by floating-point noise across the two creation paths, so a few millimetres
        /// over 120 steps is generous. A real regression (the custom baker emitting a different definition)
        /// diverges by metres.
        /// </summary>
        [UnityTest]
        public IEnumerator CustomAuthoredCircle_MatchesBuiltInAuthored_NearExact()
        {
            yield return CustomAuthoringParityHarness.RunCustomVsBuiltInParity(
                "CustomVsBuiltIn",
                Dt,
                stepCount: 120,
                nearExactMeters: 5e-3f,
                nearExactRadians: 1e-3f
            );
        }

        /// <summary>
        /// BROAD: a custom-authored circle falling onto a custom-authored static floor, run against the
        /// GameObject oracle built live from the custom authoring fields. Same generous v2-vs-v3 envelope the
        /// built-in collider-on-floor fixtures use, proving a custom-authored scene reaches the same
        /// GameObject-physics band a built-in one does.
        /// </summary>
        [UnityTest]
        public IEnumerator CustomAuthoredCircleOnFloor_AgreesWithGameObjectReference()
        {
            // Contact scene: a circle falls onto a box floor and settles. A contact phase diverges faster
            // between the v2-iteration (GameObject) and v3-substep (package) solvers than free fall, so this
            // uses the SAME proven contact-scene envelope the built-in collider-on-floor fixtures use
            // (ColliderShapeParityValidation: 1.5e-1 base + 3.0e-3 growth + 9e-2 angle), not the tighter
            // free-fall band. The measured worst on this fixture is ~0.21 m at the bounce/settle step, well
            // inside this band; a real bake regression diverges by metres.
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                positionBaseMeters = 1.5e-1f,
                positionGrowthPerStep = 3.0e-3f,
                angleCapRadians = 9e-2f,
                // Falls from y=5 onto a floor whose top is at 0.5; a 0.5-radius circle rests near y≈1.
                settleRegionMin = new float2(-0.5f, 0.0f),
                settleRegionMax = new float2(0.5f, 2.0f),
                minTravelMeters = 2f,
            };

            yield return CustomAuthoringParityHarness.RunCustomAuthoredGameObjectParity(
                "CustomCircleOnFloor",
                "CustomCircleOnFloor_Sub",
                Dt,
                stepCount: 180,
                envelope
            );
        }
    }
}
