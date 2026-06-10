using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Baking;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Smoke check for Phase-12 collider transform-scale baking: the pure geometry-scale rules every baker
    /// applies, pinned deterministically without a SubScene. This is the fast, non-adversarial demonstration
    /// that the manual-QA bug is fixed — a body resting across the FULL width of an 18×-wide floor is exactly
    /// "the baked box half-extent equals 18× the authored half-extent." The dedicated adversarial GameObject-
    /// vs-ECS parity gate (a separate validating agent) pins the runtime contact parity; this smoke pins the
    /// bake math the gate would otherwise diagnose only indirectly.
    /// </summary>
    [TestFixture]
    public sealed class ColliderScaleBakeSmoke
    {
        // The manual-QA case: a 1×1 BoxCollider2D on a Transform with Scale X = 18.1822 must bake to a box
        // 18.1822 m wide (so a dropped body rests across its full width, not just the centre 1×1).
        [Test]
        public void WideFloor_BoxSizeScalesPerAxis()
        {
            var scale = new float2(18.1822f, 1f);
            var baked = Collider2DBaking.ScaleBoxSize(new float2(1f, 1f), scale);
            Assert.AreEqual(18.1822f, baked.x, 1e-4f, "Box width must follow the X transform scale.");
            Assert.AreEqual(1f, baked.y, 1e-4f, "Box height must follow the Y transform scale (unscaled).");
        }

        // A circle cannot become an ellipse — CircleCollider2D takes the LARGER absolute axis scale.
        [Test]
        public void Circle_NonUniformScale_TakesMaxAxis()
        {
            Assert.AreEqual(15f, Collider2DBaking.ScaleCircleRadius(5f, new float2(3f, 1f)), 1e-5f);
            Assert.AreEqual(15f, Collider2DBaking.ScaleCircleRadius(5f, new float2(1f, 3f)), 1e-5f);
            // The max is taken over absolute values, so a negative axis still grows the radius.
            Assert.AreEqual(15f, Collider2DBaking.ScaleCircleRadius(5f, new float2(-3f, 1f)), 1e-5f);
        }

        // A negative single axis mirrors the plane → winding flips; an even count does not.
        [Test]
        public void FlipsWinding_OnOddNegativeAxisCount()
        {
            Assert.IsTrue(Collider2DBaking.FlipsWinding(new float2(-1f, 1f)), "single mirror flips winding");
            Assert.IsTrue(Collider2DBaking.FlipsWinding(new float2(1f, -1f)), "single mirror flips winding");
            Assert.IsFalse(Collider2DBaking.FlipsWinding(new float2(-1f, -1f)), "double mirror is a rotation");
            Assert.IsFalse(Collider2DBaking.FlipsWinding(new float2(2f, 3f)), "positive scale never flips");
        }

        // The box extents stay positive under a flip (a symmetric box mirrors about its centre), while the
        // offset moves signed to the mirrored side.
        [Test]
        public void NegativeScale_BoxExtentsPositive_OffsetSigned()
        {
            var scale = new float2(-2f, 1f);
            var size = Collider2DBaking.ScaleBoxSize(new float2(3f, 1f), scale);
            Assert.AreEqual(6f, size.x, 1e-5f, "flipped extent is positive (mirror, not shrink)");
            var offset = Collider2DBaking.ScaleOffset(new float2(4f, 0f), scale);
            Assert.AreEqual(-8f, offset.x, 1e-5f, "offset moves to the mirrored side (signed)");
        }

        // The offset scales per-axis (a point in the collider's scaled local space).
        [Test]
        public void Offset_ScalesPerAxis()
        {
            var offset = Collider2DBaking.ScaleOffset(new float2(2f, 3f), new float2(10f, 0.5f));
            Assert.AreEqual(20f, offset.x, 1e-5f);
            Assert.AreEqual(1.5f, offset.y, 1e-5f);
        }
    }
}
