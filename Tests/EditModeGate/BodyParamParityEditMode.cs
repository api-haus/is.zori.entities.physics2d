using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Tests.Editor;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>BodyParamParityValidation</c>: surface materials, body parameters, and the
    /// collider-only static-body case, each run ECS-baked vs the GameObject reference. Envelopes and step counts
    /// copied verbatim from the PlayMode gate.
    /// </summary>
    public sealed class BodyParamParityEditMode : Physics2DEditModeHarness
    {
        const float Dt = 1f / 60f;

        ParityEnvelope ContactEnvelope(float2 settleMin, float2 settleMax, float minTravel = 2f) =>
            new()
            {
                positionBaseMeters = 1.5e-1f,
                positionGrowthPerStep = 3.0e-3f,
                angleCapRadians = 9e-2f,
                settleRegionMin = settleMin,
                settleRegionMax = settleMax,
                minTravelMeters = minTravel,
            };

        [Test]
        public void Bounce_BouncyBallOnBouncyFloor_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.Bounce,
                "Bounce",
                Dt,
                240,
                ContactEnvelope(new float2(-2f, 0.3f), new float2(2f, 4.0f), minTravel: 3f)
            );

        [Test]
        public void Friction_GrippyVsSlipperyBalls_AgreeWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.Friction,
                "Friction",
                Dt,
                240,
                ContactEnvelope(new float2(-22f, 0.5f), new float2(26f, 4.5f), minTravel: 1f)
            );

        [Test]
        public void Velocity_LaunchedParabola_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.Velocity,
                "Velocity",
                Dt,
                120,
                new ParityEnvelope
                {
                    positionBaseMeters = 1e-2f,
                    positionGrowthPerStep = 2.5e-3f,
                    angleCapRadians = 1e-2f,
                    settleRegionMin = new float2(2f, -15f),
                    settleRegionMax = new float2(18f, 6f),
                    minTravelMeters = 5f,
                }
            );

        [Test]
        public void FreezeX_FallsStraightDespiteHorizontalVelocity_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.FreezeX,
                "FreezeX",
                Dt,
                120,
                new ParityEnvelope
                {
                    positionBaseMeters = 1e-2f,
                    positionGrowthPerStep = 2.5e-3f,
                    angleCapRadians = 1e-2f,
                    settleRegionMin = new float2(-0.1f, -16f),
                    settleRegionMax = new float2(0.1f, -10f),
                    minTravelMeters = 8f,
                }
            );

        [Test]
        public void Mass_HeavyAndLightFallIdentically_AgreeWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.Mass,
                "Mass",
                Dt,
                120,
                new ParityEnvelope
                {
                    positionBaseMeters = 1e-2f,
                    positionGrowthPerStep = 2.5e-3f,
                    angleCapRadians = 1e-2f,
                    settleRegionMin = new float2(-2.2f, -16f),
                    settleRegionMax = new float2(2.2f, -10f),
                    minTravelMeters = 8f,
                }
            );

        [Test]
        public void StaticFallback_BallRestsOnColliderOnlyStaticCircle_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.StaticFallback,
                "StaticFallback",
                Dt,
                240,
                ContactEnvelope(new float2(-2f, 0.1f), new float2(2f, 2.0f), minTravel: 3f)
            );
    }
}
