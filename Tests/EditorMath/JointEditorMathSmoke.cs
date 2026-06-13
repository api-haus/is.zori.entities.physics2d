using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests.EditorMath
{
    /// <summary>
    /// Pure-math smoke for the Phase-F2 joint editor's testable core, <see cref="PhysicsJoint2DGizmos"/> — the
    /// handle-position-from-joint (axis direction / endpoint, limit-arc points) and the joint-field-from-drag
    /// (axis angle, the angle / translation limits). The interactive <c>Handles</c> drag itself is manual QA (it
    /// cannot run in batchmode); these tests pin the deterministic math the drag binds to. Lives in the same
    /// EditMode assembly that already pins <see cref="PhysicsShape2DGizmos"/>.
    /// </summary>
    [TestFixture]
    public sealed class JointEditorMathSmoke
    {
        const float Eps = 1e-4f;

        [Test]
        public void AxisDirection_AtKnownAngles_IsTheUnitDirection()
        {
            Assert.IsTrue(Approx(PhysicsJoint2DGizmos.AxisDirection(0f), new float2(1f, 0f)));
            Assert.IsTrue(Approx(PhysicsJoint2DGizmos.AxisDirection(90f), new float2(0f, 1f)));
            Assert.IsTrue(Approx(PhysicsJoint2DGizmos.AxisDirection(180f), new float2(-1f, 0f)));
        }

        [Test]
        public void AxisEndpoint_IsAnchorPlusDirectionTimesLength()
        {
            var end = PhysicsJoint2DGizmos.AxisEndpoint(new float2(2f, 1f), 90f, 3f);
            Assert.IsTrue(Approx(end, new float2(2f, 4f)));
        }

        [Test]
        public void AngleFromAxisDrag_RecoversTheDirectionAngle()
        {
            // drag the axis endpoint to straight up from the anchor → 90°
            var deg = PhysicsJoint2DGizmos.AngleFromAxisDrag(new float2(1f, 1f), new float2(1f, 5f));
            Assert.AreEqual(90f, deg, Eps);
            // a 45° NE drag
            var deg2 = PhysicsJoint2DGizmos.AngleFromAxisDrag(Unity.Mathematics.float2.zero, new float2(2f, 2f));
            Assert.AreEqual(45f, deg2, Eps);
        }

        [Test]
        public void AngleLimitArcPoints_SpanTheRightAngleAtTheRightRadius()
        {
            // a quarter-circle limit arc from 0° to 90° at radius 2 about (1, 0)
            var center = new float2(1f, 0f);
            var pts = PhysicsJoint2DGizmos.AngleLimitArcPoints(center, 0f, 90f, 2f, 8);
            Assert.AreEqual(9, pts.Length);
            // first point at 0° (center + (2,0)), last at 90° (center + (0,2))
            Assert.IsTrue(Approx(pts[0], center + new float2(2f, 0f)));
            Assert.IsTrue(Approx(pts[8], center + new float2(0f, 2f)));
            // every point sits at the radius
            foreach (var p in pts)
                Assert.AreEqual(2f, length(p - center), Eps);
        }

        [Test]
        public void AngleLimitArcPoints_NegativeSpan_SweepsTheOtherWay()
        {
            // upper below lower sweeps the negative direction (an empty/inverted range, but the geometry holds)
            var pts = PhysicsJoint2DGizmos.AngleLimitArcPoints(Unity.Mathematics.float2.zero, 30f, -30f, 1f, 4);
            Assert.IsTrue(
                Approx(pts[0], PhysicsJoint2DGizmos.AngleLimitMarker(Unity.Mathematics.float2.zero, 30f, 1f))
            );
            Assert.IsTrue(
                Approx(pts[4], PhysicsJoint2DGizmos.AngleLimitMarker(Unity.Mathematics.float2.zero, -30f, 1f))
            );
        }

        [Test]
        public void AngleLimitMarker_IsThePointOnTheArc()
        {
            var center = new float2(0f, 0f);
            Assert.IsTrue(Approx(PhysicsJoint2DGizmos.AngleLimitMarker(center, 0f, 3f), new float2(3f, 0f)));
            Assert.IsTrue(Approx(PhysicsJoint2DGizmos.AngleLimitMarker(center, 90f, 3f), new float2(0f, 3f)));
        }

        [Test]
        public void AngleFromLimitDrag_RoundTripsThroughTheMarker()
        {
            var center = new float2(2f, -1f);
            const float deg = 37f;
            const float radius = 2.5f;
            var marker = PhysicsJoint2DGizmos.AngleLimitMarker(center, deg, radius);
            var recovered = PhysicsJoint2DGizmos.AngleFromLimitDrag(center, marker);
            Assert.AreEqual(deg, recovered, Eps);
        }

        [Test]
        public void TranslationLimitSegment_EndpointsAlongTheAxis()
        {
            // a horizontal axis (0°): lower −2, upper 3 → endpoints at anchor + (−2,0) and (3,0)
            PhysicsJoint2DGizmos.TranslationLimitSegment(new float2(1f, 1f), 0f, -2f, 3f, out var a, out var b);
            Assert.IsTrue(Approx(a, new float2(-1f, 1f)));
            Assert.IsTrue(Approx(b, new float2(4f, 1f)));
        }

        [Test]
        public void TranslationFromLimitDrag_RoundTripsTheProjection()
        {
            // axis at 90° (straight up): a point 4 up from the anchor projects to translation 4
            var anchor = new float2(3f, 2f);
            PhysicsJoint2DGizmos.TranslationLimitSegment(anchor, 90f, 0f, 4f, out _, out var b);
            var t = PhysicsJoint2DGizmos.TranslationFromLimitDrag(anchor, 90f, b);
            Assert.AreEqual(4f, t, Eps);
            // a drag perpendicular to the axis projects to ~0 (only the axial component counts)
            var perp = anchor + new float2(5f, 0f);
            var t2 = PhysicsJoint2DGizmos.TranslationFromLimitDrag(anchor, 90f, perp);
            Assert.AreEqual(0f, t2, Eps);
        }

        static bool Approx(float2 a, float2 b) => length(a - b) < Eps;
    }
}
