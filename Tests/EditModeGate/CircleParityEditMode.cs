using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Tests.Editor;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>CircleParityValidation</c>: the Circle falling-body fixture run two ways — ECS-baked
    /// vs the GameObject reference (additively-opened built-in body stepped by <c>Physics2D.Simulate</c>) — and
    /// compared. Validates the EditMode harness's GameObject-reference side. Envelope and step count copied
    /// verbatim from the PlayMode gate.
    /// </summary>
    public sealed class CircleParityEditMode : Physics2DEditModeHarness
    {
        const float Dt = 1f / 60f;
        const int Steps = 120;

        [Test]
        public void Circle_FallingBody_AgreesWithGameObjectReference()
        {
            var envelope = new ParityEnvelope
            {
                positionBaseMeters = 1e-2f,
                positionGrowthPerStep = 2.0e-3f,
                angleCapRadians = 1e-2f,
                settleRegionMin = new float2(-0.5f, -12f),
                settleRegionMax = new float2(0.5f, -7f),
                minTravelMeters = 0.5f,
            };

            RunParity(Physics2DFixtures.FallingBody, "FallingBody", Dt, Steps, envelope);
        }
    }
}
