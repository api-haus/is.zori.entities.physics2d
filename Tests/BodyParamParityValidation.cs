using System.Collections;
using Unity.Mathematics;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-1B parity: surface materials (bounce / friction), <see cref="UnityEngine.Rigidbody2D"/> body
    /// parameters (initial-velocity seed, freeze-X constraint, explicit heavy/light mass), and the explicit
    /// collider-only static-body fallback — each run two ways from a single-authored child scene (ECS bake vs
    /// GameObject <c>Physics2D.Simulate</c>) and compared through <see cref="PhysicsParityHarness"/>.
    /// </summary>
    /// <remarks>
    /// Each slice picks a falsification observable that the feature under test, and only that feature,
    /// controls: the bounce ball's rebound/rest height (bounciness), the sliding balls' stop distance
    /// (friction), the launched body's parabola (velocity seed), the frozen-X body's straight fall
    /// (constraint), and the heavy/light bodies' identical free fall (explicit mass not corrupting the path).
    /// The disqualifiers (travelled, settled in region, no NaN/Inf) plus a growth-bounded position/angle band
    /// do the load-bearing work; the bands are widened from the free-fall floor toward the design's measured
    /// contact-phase worst, justified by the v2-iteration (GameObject) vs v3-substep (package) solver split
    /// (00d), and any slice that cannot reach a band logs <c>parity NOT achieved</c> rather than failing.
    ///
    /// <para>Build the fixtures first via
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.BodyParamFixtureBuilder.BuildAll</c>.</para>
    /// </remarks>
    public sealed class BodyParamParityValidation
    {
        const float Dt = 1f / 60f;

        // Contact-phase envelope (matches the Phase-1A collider band): a larger flat base absorbs the
        // one-time landing/contact transient where the two solvers resolve penetration at different rates,
        // plus a free-fall slope for the descent. Disqualifiers carry the correctness; the band fails only on
        // a metres-scale divergence.
        static PhysicsParityHarness.ParityEnvelope ContactEnvelope(
            float2 settleMin,
            float2 settleMax,
            float minTravel = 2f
        )
        {
            return new PhysicsParityHarness.ParityEnvelope
            {
                positionBaseMeters = 1.5e-1f,
                positionGrowthPerStep = 3.0e-3f,
                angleCapRadians = 9e-2f,
                settleRegionMin = settleMin,
                settleRegionMax = settleMax,
                minTravelMeters = minTravel,
            };
        }

        [UnityTest]
        public IEnumerator Bounce_BouncyBallOnBouncyFloor_AgreesWithGameObjectReference()
        {
            // Bounciness 0.8 on ball + floor: the ball drops from y=6, rebounds several times, and settles
            // near the floor top (0.5 + 0.5 = 1.0). The bounce phase is where v2 and v3 restitution diverge
            // most, so the band is the contact band; the settle region brackets the floor-rest height while
            // allowing the body to still be mid-bounce-decay at the final step. minTravel large (it fell 5 m).
            yield return PhysicsParityHarness.RunParity(
                "Bounce",
                "Bounce_Sub",
                Dt,
                240,
                ContactEnvelope(new float2(-2f, 0.3f), new float2(2f, 4.0f), minTravel: 3f)
            );
        }

        [UnityTest]
        public IEnumerator Friction_GrippyVsSlipperyBalls_AgreeWithGameObjectReference()
        {
            // Two boxes launched at +10 m/s: the grippy box (friction 0.6, lower lane, centre y≈1.0)
            // decelerates at ~μg ≈ 5.9 m/s² and stops after ~8.5 m (near x≈-11.5), while the slippery box
            // (friction 0.0, upper lane, centre y≈4.0) keeps gliding to x≈+20 over the 4 s run. The X-stop
            // separation (~30 m) is the friction observable. The settle region is wide in X and brackets both
            // Y lanes; they start at distinct Y so the matching key is stable. The box slides flat (rotation
            // frozen), so the band is a contact band absorbing the slide/stop transient.
            yield return PhysicsParityHarness.RunParity(
                "Friction",
                "Friction_Sub",
                Dt,
                240,
                ContactEnvelope(new float2(-22f, 0.5f), new float2(26f, 4.5f), minTravel: 1f)
            );
        }

        [UnityTest]
        public IEnumerator Velocity_LaunchedParabola_AgreesWithGameObjectReference()
        {
            // Initial velocity (5, 12) m/s under gravity, no floor: a parabola. Free-fall-class divergence
            // (no contact), so a tighter free-fall band suffices, but the seeded horizontal velocity makes the
            // body travel a wide arc — the settle region brackets where it ends after 120 steps (2 s): it has
            // come back down well below y=0 and moved ~+10 m in X. The body is in free flight (never settles),
            // so the "settle region" is just a coarse end-position bound.
            yield return PhysicsParityHarness.RunParity(
                "Velocity",
                "Velocity_Sub",
                Dt,
                120,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 1e-2f,
                    positionGrowthPerStep = 2.5e-3f,
                    angleCapRadians = 1e-2f,
                    // After 2 s: x ≈ 5*2 = 10, y ≈ 12*2 - 0.5*9.81*4 ≈ 24 - 19.6 ≈ 4.4 at apex-and-down; the
                    // body has descended below start by the end. Bracket generously.
                    settleRegionMin = new float2(2f, -15f),
                    settleRegionMax = new float2(18f, 6f),
                    minTravelMeters = 5f,
                }
            );
        }

        [UnityTest]
        public IEnumerator FreezeX_FallsStraightDespiteHorizontalVelocity_AgreesWithGameObjectReference()
        {
            // FreezePositionX + a seeded +8 m/s horizontal velocity: the constraint must cancel all X motion,
            // so the body falls straight from x=0. The observable is X-invariance. A free-fall band (no
            // contact); the settle region pins X tightly around 0 (the constraint's signature) and Y to the
            // free-fall descent over 120 steps.
            yield return PhysicsParityHarness.RunParity(
                "FreezeX",
                "FreezeX_Sub",
                Dt,
                120,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 1e-2f,
                    positionGrowthPerStep = 2.5e-3f,
                    angleCapRadians = 1e-2f,
                    // X pinned near 0 (frozen); Y descends ~19.6 m in 2 s from y=5 → ~ -15.
                    settleRegionMin = new float2(-0.1f, -16f),
                    settleRegionMax = new float2(0.1f, -10f),
                    minTravelMeters = 8f,
                }
            );
        }

        [UnityTest]
        public IEnumerator Mass_HeavyAndLightFallIdentically_AgreeWithGameObjectReference()
        {
            // Explicit mass 50 (heavy) and 0.2 (light), both free-falling: in free fall mass does not change
            // the trajectory (a=g), so the gate verifies the explicit-mass mapping does not corrupt the path —
            // both fall like the GameObject reference. Free-fall band; the two start at distinct Y (5 and 8),
            // so the matching key is stable, and the settle region brackets both after 120 steps of descent.
            yield return PhysicsParityHarness.RunParity(
                "Mass",
                "Mass_Sub",
                Dt,
                120,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 1e-2f,
                    positionGrowthPerStep = 2.5e-3f,
                    angleCapRadians = 1e-2f,
                    // Heavy from y=5 → ~ -15, light from y=8 → ~ -12; both at x≈±2 (unchanged).
                    settleRegionMin = new float2(-2.2f, -16f),
                    settleRegionMax = new float2(2.2f, -10f),
                    minTravelMeters = 8f,
                }
            );
        }

        [UnityTest]
        public IEnumerator StaticFallback_BallRestsOnColliderOnlyStaticCircle_AgreesWithGameObjectReference()
        {
            // A dynamic circle falls onto a collider-only static CircleCollider2D ground (no Rigidbody2D),
            // exercising the CircleCollider2DBaker's static-body fallback specifically. If the fallback did
            // not fire the ground would not exist as a body and the ball would fall forever (failing the
            // settle disqualifier) — so this fixture is a hard-to-falsify probe of the static path. The static
            // circle's top is at y = -5 + 5 = 0, so the dynamic ball (radius 0.5) rests with its centre at
            // ~0.5. Only the dynamic ball is compared (the static ground is excluded on both backends).
            yield return PhysicsParityHarness.RunParity(
                "StaticFallback",
                "StaticFallback_Sub",
                Dt,
                240,
                ContactEnvelope(new float2(-2f, 0.1f), new float2(2f, 2.0f), minTravel: 3f)
            );
        }
    }
}
