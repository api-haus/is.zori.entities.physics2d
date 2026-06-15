using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Tests.Editor;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>ColliderShapeParityValidation</c>: each of Box/Capsule/Polygon/Edge resting on a
    /// static floor run ECS-baked vs the GameObject reference. Envelopes and step count copied verbatim.
    /// </summary>
    public sealed class ColliderShapeParityEditMode : Physics2DEditModeHarness
    {
        const float Dt = 1f / 60f;
        const int Steps = 180;

        ParityEnvelope ContactEnvelope(float2 settleMin, float2 settleMax) =>
            new()
            {
                positionBaseMeters = 1.5e-1f,
                positionGrowthPerStep = 3.0e-3f,
                angleCapRadians = 9e-2f,
                settleRegionMin = settleMin,
                settleRegionMax = settleMax,
                minTravelMeters = 2f,
            };

        [Test]
        public void Box_OnFloor_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.BoxOnFloor,
                "BoxOnFloor",
                Dt,
                Steps,
                ContactEnvelope(new float2(-1.5f, 0.3f), new float2(1.5f, 2.0f))
            );

        [Test]
        public void Capsule_OnFloor_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.CapsuleOnFloor,
                "CapsuleOnFloor",
                Dt,
                Steps,
                ContactEnvelope(new float2(-1.5f, 0.8f), new float2(1.5f, 2.5f))
            );

        [Test]
        public void Polygon_OnFloor_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.PolygonOnFloor,
                "PolygonOnFloor",
                Dt,
                Steps,
                ContactEnvelope(new float2(-1.5f, 0.3f), new float2(1.5f, 2.0f))
            );

        [Test]
        public void Edge_OnFloor_AgreesWithGameObjectReference() =>
            RunParity(
                Physics2DFixtures.EdgeOnFloor,
                "EdgeOnFloor",
                Dt,
                Steps,
                ContactEnvelope(new float2(-2.0f, 0.0f), new float2(2.0f, 2.0f))
            );
    }
}
