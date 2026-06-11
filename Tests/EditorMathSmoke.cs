using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Pure-math smoke for the Phase-D editor's testable core, <see cref="PhysicsShape2DGizmos"/> — the
    /// handle-position-from-shape, the shape-field-from-drag-delta, and the gizmo outline-point generation. The
    /// interactive <c>Handles</c> drag itself is manual QA (it cannot run in batchmode); these tests pin the
    /// deterministic math the drag binds to.
    /// </summary>
    [TestFixture]
    public sealed class EditorMathSmoke
    {
        const float Eps = 1e-4f;

        [Test]
        public void BoxOutline_AxisAligned_IsTheFourCorners()
        {
            var pts = PhysicsShape2DGizmos.BoxOutline(
                Unity.Mathematics.float2.zero,
                new float2(2f, 4f),
                0f
            );
            Assert.AreEqual(4, pts.Length);
            // corners at (±1, ±2) in some CCW order starting bottom-left
            Assert.IsTrue(Approx(pts[0], new float2(-1f, -2f)));
            Assert.IsTrue(Approx(pts[1], new float2(1f, -2f)));
            Assert.IsTrue(Approx(pts[2], new float2(1f, 2f)));
            Assert.IsTrue(Approx(pts[3], new float2(-1f, 2f)));
        }

        [Test]
        public void BoxCornerHandlePositions_MatchTheCorners()
        {
            var center = new float2(3f, -1f);
            var size = new float2(2f, 2f);
            var corners = PhysicsShape2DGizmos.BoxCornerHandlePositions(center, size, 0f);
            // the box's four corners are center ± (1,1)
            Assert.IsTrue(Approx(corners[0], center + new float2(-1f, -1f)));
            Assert.IsTrue(Approx(corners[2], center + new float2(1f, 1f)));
        }

        [Test]
        public void BoxOutline_Rotated90_SwapsExtents()
        {
            var pts = PhysicsShape2DGizmos.BoxOutline(
                Unity.Mathematics.float2.zero,
                new float2(2f, 4f),
                90f
            );
            // after a 90° rotation the bounding extents are (4, 2): max |x| ~ 2, max |y| ~ 1
            var maxX = 0f;
            var maxY = 0f;
            foreach (var p in pts)
            {
                maxX = max(maxX, abs(p.x));
                maxY = max(maxY, abs(p.y));
            }
            Assert.AreEqual(2f, maxX, Eps); // half of 4 (the original height now along x)
            Assert.AreEqual(1f, maxY, Eps); // half of 2
        }

        [Test]
        public void BoxSizeFromEdgeDrag_PlusXFace_DoublesTheDistance()
        {
            // drag the +X face to x = 3 about center 0 → full width 6, height unchanged
            var newSize = PhysicsShape2DGizmos.BoxSizeFromEdgeDrag(
                new float2(2f, 4f),
                0,
                new float2(3f, 0f),
                0f,
                Unity.Mathematics.float2.zero
            );
            Assert.AreEqual(6f, newSize.x, Eps);
            Assert.AreEqual(4f, newSize.y, Eps);
        }

        [Test]
        public void BoxSizeFromEdgeDrag_PlusYFace_OnRotatedBox_UsesLocalAxis()
        {
            // a box rotated 90°: dragging the +Y face handle along the world axis maps to the local y extent.
            // place the dragged point 5 units along the box's local +Y (which is world −X after +90°): local
            // un-rotation recovers localY = 5 → height 10.
            sincos(radians(90f), out var s, out var c);
            var localY = new float2(0f, 5f);
            var worldDragged = new float2(c * localY.x - s * localY.y, s * localY.x + c * localY.y);
            var newSize = PhysicsShape2DGizmos.BoxSizeFromEdgeDrag(
                new float2(2f, 4f),
                1,
                worldDragged,
                90f,
                Unity.Mathematics.float2.zero
            );
            Assert.AreEqual(10f, newSize.y, Eps);
            Assert.AreEqual(2f, newSize.x, Eps); // x unchanged
        }

        [Test]
        public void CircleRadiusFromDrag_IsTheDistance()
        {
            var r = PhysicsShape2DGizmos.CircleRadiusFromDrag(
                new float2(1f, 1f),
                new float2(4f, 5f)
            );
            Assert.AreEqual(5f, r, Eps); // |(3,4)| = 5
        }

        [Test]
        public void CircleOutline_IsARingOfTheRightRadius()
        {
            var pts = PhysicsShape2DGizmos.CircleOutline(new float2(2f, 0f), 3f, 32);
            Assert.AreEqual(32, pts.Length);
            foreach (var p in pts)
                Assert.AreEqual(3f, length(p - new float2(2f, 0f)), Eps);
        }

        [Test]
        public void AngleFromRotationDrag_RecoversTheAngle()
        {
            var deg = PhysicsShape2DGizmos.AngleFromRotationDrag(
                Unity.Mathematics.float2.zero,
                new float2(0f, 2f)
            );
            Assert.AreEqual(90f, deg, Eps);
        }

        [Test]
        public void CapsuleFieldsFromHandles_RoundTripsThroughGetCapsuleCenters()
        {
            // a vertical capsule: caps at (0,±2), radius 0.5 → the package size + vertical + angle must, when
            // round-tripped through GetCapsuleCenters, reproduce caps at (0,±2) with radius 0.5.
            var capA = new float2(0f, -2f);
            var capB = new float2(0f, 2f);
            PhysicsShape2DGizmos.CapsuleFieldsFromHandles(
                capA,
                capB,
                0.5f,
                out var size,
                out var vertical,
                out var angle,
                out var center
            );
            Assert.IsTrue(Approx(center, Unity.Mathematics.float2.zero));

            var auth = new UnityEngine.GameObject("capsule").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                auth.CapsuleSize = size;
                auth.CapsuleVertical = vertical;
                auth.CapsuleAngle = angle;
                auth.GetCapsuleCenters(out var r, out var c1, out var c2);
                Assert.AreEqual(0.5f, r, 1e-3f);
                // the two reconstructed centres are at (0, ±2) up to ordering
                var ys = new[] { c1.y, c2.y };
                System.Array.Sort(ys);
                Assert.AreEqual(-2f, ys[0], 1e-3f);
                Assert.AreEqual(2f, ys[1], 1e-3f);
                Assert.AreEqual(0f, abs(c1.x) + abs(c2.x), 1e-3f);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        [Test]
        public void PolygonOutline_AppliesTheOffset()
        {
            var verts = new[]
            {
                new float2(0f, 0f),
                new float2(1f, 0f),
                new float2(0f, 1f),
            };
            var pts = PhysicsShape2DGizmos.PolygonOutline(verts, new float2(10f, 5f));
            Assert.IsTrue(Approx(pts[0], new float2(10f, 5f)));
            Assert.IsTrue(Approx(pts[1], new float2(11f, 5f)));
            Assert.IsTrue(Approx(pts[2], new float2(10f, 6f)));
        }

        static bool Approx(float2 a, float2 b) => length(a - b) < Eps;
    }
}
