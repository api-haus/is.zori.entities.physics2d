using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Tests.Editor;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>JointParityValidation</c>: each of the nine joint kinds (a dynamic body jointed to a
    /// static anchor) run ECS-baked vs the GameObject reference. Envelopes and step counts copied verbatim.
    /// </summary>
    public sealed class JointParityEditMode : Physics2DEditModeHarness
    {
        const float Dt = 1f / 60f;

        [Test]
        public void Hinge_PendulumSwing_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.HingeJoint,
                "HingeJoint",
                Dt,
                150,
                new ParityEnvelope
                {
                    positionBaseMeters = 0.25f,
                    positionGrowthPerStep = 1.2e-2f,
                    angleCapRadians = 3.2f,
                    settleRegionMin = new float2(-1.3f, 3.6f),
                    settleRegionMax = new float2(1.3f, 6.4f),
                    minTravelMeters = 0.1f,
                }
            );

        [Test]
        public void Slider_AxisConfinedMotion_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.SliderJoint,
                "SliderJoint",
                Dt,
                120,
                new ParityEnvelope
                {
                    positionBaseMeters = 2.0e-2f,
                    positionGrowthPerStep = 3.0e-3f,
                    angleCapRadians = 1.0e-2f,
                    settleRegionMin = new float2(8.0f, 4.5f),
                    settleRegionMax = new float2(16.0f, 5.5f),
                    minTravelMeters = 6f,
                }
            );

        [Test]
        public void Wheel_SuspensionTravel_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.WheelJoint,
                "WheelJoint",
                Dt,
                180,
                new ParityEnvelope
                {
                    positionBaseMeters = 0.12f,
                    positionGrowthPerStep = 2.0e-3f,
                    angleCapRadians = 1.0f,
                    settleRegionMin = new float2(-0.4f, 4.4f),
                    settleRegionMax = new float2(0.4f, 5.2f),
                    minTravelMeters = 0.03f,
                }
            );

        [Test]
        public void Distance_RigidSeparationHeld_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.DistanceJoint,
                "DistanceJoint",
                Dt,
                150,
                new ParityEnvelope
                {
                    positionBaseMeters = 0.25f,
                    positionGrowthPerStep = 1.4e-2f,
                    angleCapRadians = 3.2f,
                    settleRegionMin = new float2(-3.3f, 1.6f),
                    settleRegionMax = new float2(3.3f, 5.3f),
                    minTravelMeters = 0.5f,
                }
            );

        [Test]
        public void Spring_OscillatesTowardRest_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.SpringJoint,
                "SpringJoint",
                Dt,
                180,
                new ParityEnvelope
                {
                    positionBaseMeters = 0.3f,
                    positionGrowthPerStep = 3.0e-3f,
                    angleCapRadians = 1.0e-1f,
                    settleRegionMin = new float2(-0.5f, 3.4f),
                    settleRegionMax = new float2(0.5f, 4.8f),
                    minTravelMeters = 0.4f,
                }
            );

        [Test]
        public void Fixed_RelativePoseLocked_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.FixedJoint,
                "FixedJoint",
                Dt,
                150,
                new ParityEnvelope
                {
                    positionBaseMeters = 5.0e-2f,
                    positionGrowthPerStep = 1.0e-3f,
                    angleCapRadians = 5.0e-2f,
                    settleRegionMin = new float2(1.7f, 4.6f),
                    settleRegionMax = new float2(2.3f, 5.4f),
                    minTravelMeters = 1.0e-5f,
                }
            );

        [Test]
        public void Relative_OffsetMaintained_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.RelativeJoint,
                "RelativeJoint",
                Dt,
                150,
                new ParityEnvelope
                {
                    positionBaseMeters = 0.3f,
                    positionGrowthPerStep = 6.0e-3f,
                    angleCapRadians = 0.5f,
                    settleRegionMin = new float2(-2.7f, 4.3f),
                    settleRegionMax = new float2(-1.3f, 5.5f),
                    minTravelMeters = 2.0f,
                }
            );

        [Test]
        public void Friction_RelativeMotionDamps_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.FrictionJoint,
                "FrictionJoint",
                Dt,
                120,
                new ParityEnvelope
                {
                    positionBaseMeters = 0.3f,
                    positionGrowthPerStep = 4.0e-3f,
                    angleCapRadians = 0.5f,
                    settleRegionMin = new float2(-0.3f, 4.3f),
                    settleRegionMax = new float2(6.0f, 5.4f),
                    minTravelMeters = 2.0e-2f,
                }
            );

        [Test]
        public void Target_PullsToWorldTarget_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.TargetJoint,
                "TargetJoint",
                Dt,
                150,
                new ParityEnvelope
                {
                    positionBaseMeters = 0.35f,
                    positionGrowthPerStep = 3.0e-3f,
                    angleCapRadians = 0.5f,
                    settleRegionMin = new float2(2.4f, 4.2f),
                    settleRegionMax = new float2(3.6f, 5.4f),
                    minTravelMeters = 1.5f,
                }
            );
    }
}
