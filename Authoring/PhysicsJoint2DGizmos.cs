using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Authoring
{
    /// <summary>
    /// The PURE axis + limit + field-from-drag math behind the Phase-F2 2D joint editor — the testable core the
    /// <c>UnityEditor.Handles</c> / <c>Gizmos</c> shell binds, sibling to <see cref="PhysicsShape2DGizmos"/>.
    /// Every method takes and returns plain <see cref="Unity.Mathematics.float2"/> data (anchors are
    /// body-local, angles are DEGREES — the joint angular convention), with no <c>UnityEditor</c> / <c>Gizmos</c>
    /// dependency, so it lives in the Authoring assembly and is unit-testable by the existing package test path.
    /// The editor assembly converts the returned <see cref="Unity.Mathematics.float2"/> to <c>Vector3</c>
    /// (z = 0), sets <c>Handles.matrix</c> / <c>Gizmos.matrix</c> to the relevant body's
    /// <c>localToWorldMatrix</c>, and draws.
    /// </summary>
    /// <remarks>
    /// This separation is the brief's testability requirement: the handle-position-from-joint (the axis
    /// direction / endpoint, the limit-arc points), and the joint-field-from-drag (the axis angle, the angle /
    /// translation limits) are all pure functions here; the interactive drag itself (clicking a
    /// <c>Handles.FreeMoveHandle</c>) is the thin Editor shell and is manual QA. The limit visualizations return
    /// POLYLINES (not <c>DrawWireArc</c> calls) so the same geometry is both EditMode-testable AND drawable by
    /// the <c>Gizmos</c>-based drawer, which has no arc primitive.
    /// </remarks>
    public static class PhysicsJoint2DGizmos
    {
        // ----- axis (Slider / Wheel slide / suspension direction) -----

        /// <summary>The unit slide / suspension direction for an axis angle in DEGREES: <c>(cos, sin)</c>.</summary>
        public static float2 AxisDirection(float angleDeg)
        {
            sincos(radians(angleDeg), out var s, out var c);
            return new float2(c, s);
        }

        /// <summary>The far endpoint of the drawn axis handle: <paramref name="anchor"/> plus the axis
        /// direction times <paramref name="length"/> (the point a drag handle sits at).</summary>
        public static float2 AxisEndpoint(float2 anchor, float angleDeg, float length) =>
            anchor + AxisDirection(angleDeg) * length;

        /// <summary>The axis angle (DEGREES) implied by dragging the axis-direction handle to
        /// <paramref name="draggedPos"/> about <paramref name="anchor"/> — <c>atan2</c> of the offset. Zero
        /// offset keeps the angle at 0.</summary>
        public static float AngleFromAxisDrag(float2 anchor, float2 draggedPos)
        {
            var d = draggedPos - anchor;
            if (lengthsq(d) < 1e-12f)
                return 0f;
            return degrees(atan2(d.y, d.x));
        }

        // ----- angle limit (Hinge): an arc from lower to upper, with end markers -----

        /// <summary>
        /// A polyline sweeping the angle range [<paramref name="lowerDeg"/>, <paramref name="upperDeg"/>]
        /// around <paramref name="center"/> at <paramref name="radius"/> (the Hinge angle-limit arc). Returns
        /// <c>segments + 1</c> points, the first at <paramref name="lowerDeg"/> and the last at
        /// <paramref name="upperDeg"/>, sweeping the signed span (so an upper below the lower sweeps the
        /// negative direction). The caller draws consecutive points as line segments.
        /// </summary>
        public static float2[] AngleLimitArcPoints(
            float2 center,
            float lowerDeg,
            float upperDeg,
            float radius,
            int segments = 24
        )
        {
            var n = max(1, segments);
            var pts = new float2[n + 1];
            var lo = radians(lowerDeg);
            var hi = radians(upperDeg);
            for (var i = 0; i <= n; i++)
            {
                var a = lerp(lo, hi, (float)i / n);
                sincos(a, out var s, out var c);
                pts[i] = center + new float2(c, s) * radius;
            }
            return pts;
        }

        /// <summary>The point on the limit arc at one limit angle (DEGREES) — an end marker / a drag handle to
        /// set that limit.</summary>
        public static float2 AngleLimitMarker(float2 center, float limitDeg, float radius)
        {
            sincos(radians(limitDeg), out var s, out var c);
            return center + new float2(c, s) * radius;
        }

        /// <summary>The limit angle (DEGREES) implied by dragging a limit marker to
        /// <paramref name="draggedPos"/> about <paramref name="center"/> — <c>atan2</c> of the offset.</summary>
        public static float AngleFromLimitDrag(float2 center, float2 draggedPos)
        {
            var d = draggedPos - center;
            if (lengthsq(d) < 1e-12f)
                return 0f;
            return degrees(atan2(d.y, d.x));
        }

        // ----- translation limit (Slider): a segment along the axis from lower to upper -----

        /// <summary>
        /// The Slider translation-limit segment endpoints: <paramref name="a"/> at
        /// <paramref name="lower"/> metres and <paramref name="b"/> at <paramref name="upper"/> metres along
        /// the axis direction from <paramref name="anchor"/>.
        /// </summary>
        public static void TranslationLimitSegment(
            float2 anchor,
            float angleDeg,
            float lower,
            float upper,
            out float2 a,
            out float2 b
        )
        {
            var dir = AxisDirection(angleDeg);
            a = anchor + dir * lower;
            b = anchor + dir * upper;
        }

        /// <summary>The signed translation (metres) implied by dragging a translation-limit endpoint to
        /// <paramref name="draggedPos"/>: the projection of the offset from <paramref name="anchor"/> onto the
        /// axis direction.</summary>
        public static float TranslationFromLimitDrag(float2 anchor, float angleDeg, float2 draggedPos) =>
            dot(draggedPos - anchor, AxisDirection(angleDeg));
    }
}
