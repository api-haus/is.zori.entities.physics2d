using System.Collections.Generic;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Authoring
{
    /// <summary>
    /// The PURE outline + scene-handle math behind the Phase-D 2D editor — the testable core the
    /// <c>UnityEditor.Handles</c> / <c>Gizmos</c> shell binds. Every method takes and returns plain
    /// <see cref="Unity.Mathematics.float2"/> data in the shape's LOCAL space (the same pre-scale space the
    /// authoring fields and the baker use), with no <c>UnityEditor</c> or <c>Gizmos</c> dependency, so it lives
    /// in the Authoring assembly and is unit-testable by the existing package test path. The Editor assembly
    /// converts the returned <see cref="Unity.Mathematics.float2"/> to <c>Vector3</c> (z = 0), sets
    /// <c>Handles.matrix</c> / <c>Gizmos.matrix</c> to the GameObject's <c>localToWorldMatrix</c> (which carries
    /// the TRS visually), and draws.
    /// </summary>
    /// <remarks>
    /// This separation is the brief's testability requirement: the handle-position-from-shape, the
    /// shape-field-from-drag-delta, and the gizmo outline-point generation are all pure functions here; the
    /// interactive drag itself (clicking a <c>Handles.FreeMoveHandle</c>) is the thin Editor shell and is manual
    /// QA. The capsule helpers round-trip through the same two-centre form
    /// <see cref="PhysicsShape2DAuthoring.GetCapsuleCenters"/> produces, so a handle and the bake agree.
    /// </remarks>
    public static class PhysicsShape2DGizmos
    {
        // ----- outline generation (closed point loops for the gizmo + the scene wireframe) -----

        /// <summary>A ring of <paramref name="segments"/> points around <paramref name="center"/> at
        /// <paramref name="radius"/> (the circle outline, NOT closed-duplicated — the caller closes the loop by
        /// drawing back to point 0).</summary>
        public static float2[] CircleOutline(float2 center, float radius, int segments = 48)
        {
            var n = max(3, segments);
            var pts = new float2[n];
            for (var i = 0; i < n; i++)
            {
                var a = (2f * PI * i) / n;
                sincos(a, out var s, out var c);
                pts[i] = center + new float2(c, s) * radius;
            }
            return pts;
        }

        /// <summary>The 4 corners of a box of <paramref name="size"/> (full extents) centred at
        /// <paramref name="center"/> and rotated by <paramref name="angleDeg"/> about its centre (CCW from the
        /// bottom-left). The closed outline is these four points back to the first.</summary>
        public static float2[] BoxOutline(float2 center, float2 size, float angleDeg)
        {
            var h = size * 0.5f;
            sincos(radians(angleDeg), out var s, out var c);
            var local = new[]
            {
                new float2(-h.x, -h.y),
                new float2(h.x, -h.y),
                new float2(h.x, h.y),
                new float2(-h.x, h.y),
            };
            var pts = new float2[4];
            for (var i = 0; i < 4; i++)
                pts[i] = center + Rotate(local[i], s, c);
            return pts;
        }

        /// <summary>The closed stadium (capsule) outline from the two end-cap centres
        /// <paramref name="c1"/>/<paramref name="c2"/> and the end <paramref name="radius"/>: a cap arc at each
        /// end plus the two connecting side segments. <paramref name="capSegments"/> points per half-circle.</summary>
        public static float2[] CapsuleOutline(float2 c1, float2 c2, float radius, int capSegments = 16)
        {
            var axis = c2 - c1;
            var len = length(axis);
            float2 dir = len > 1e-6f ? axis / len : new float2(0f, 1f);
            // perpendicular (left normal)
            var perp = new float2(-dir.y, dir.x);
            var baseAngle = atan2(dir.y, dir.x);

            var n = max(2, capSegments);
            var pts = new List<float2>(2 * n + 2);
            // cap around c2 from +perp sweeping to -perp (half circle on the c2 side)
            for (var i = 0; i <= n; i++)
            {
                var a = baseAngle - PI * 0.5f + (PI * i) / n;
                sincos(a, out var s, out var co);
                pts.Add(c2 + new float2(co, s) * radius);
            }
            // cap around c1 from -perp sweeping to +perp (half circle on the c1 side)
            for (var i = 0; i <= n; i++)
            {
                var a = baseAngle + PI * 0.5f + (PI * i) / n;
                sincos(a, out var s, out var co);
                pts.Add(c1 + new float2(co, s) * radius);
            }
            // suppress unused-warning on perp (kept for clarity of the construction)
            _ = perp;
            return pts.ToArray();
        }

        /// <summary>The polygon outline: each vertex offset by <paramref name="offset"/> (the closed loop is the
        /// returned points back to point 0).</summary>
        public static float2[] PolygonOutline(IReadOnlyList<float2> verts, float2 offset)
        {
            var n = verts.Count;
            var pts = new float2[n];
            for (var i = 0; i < n; i++)
                pts[i] = verts[i] + offset;
            return pts;
        }

        /// <summary>The edge outline: each vertex offset by <paramref name="offset"/>. The caller draws it open
        /// (or closes back to point 0 when <paramref name="loop"/> is true).</summary>
        public static float2[] EdgeOutline(IReadOnlyList<float2> verts, float2 offset, bool loop)
        {
            var pts = PolygonOutline(verts, offset);
            _ = loop; // the loop flag governs whether the Editor shell draws the closing segment.
            return pts;
        }

        // ----- handle POSITIONS (where to place a draggable handle) -----

        /// <summary>The four corner positions of an oriented box (for corner handles).</summary>
        public static float2[] BoxCornerHandlePositions(float2 center, float2 size, float angleDeg) =>
            BoxOutline(center, size, angleDeg);

        /// <summary>The four edge-midpoint (face-centre) positions of an oriented box, in order
        /// +X, +Y, −X, −Y — the half-extent drag handles.</summary>
        public static float2[] BoxEdgeHandlePositions(float2 center, float2 size, float angleDeg)
        {
            var h = size * 0.5f;
            sincos(radians(angleDeg), out var s, out var c);
            var local = new[] { new float2(h.x, 0f), new float2(0f, h.y), new float2(-h.x, 0f), new float2(0f, -h.y) };
            var pts = new float2[4];
            for (var i = 0; i < 4; i++)
                pts[i] = center + Rotate(local[i], s, c);
            return pts;
        }

        /// <summary>The position of the rotation-ring handle: a point at <paramref name="ringRadius"/> from the
        /// box centre along the box's local +X axis (so dragging it around the ring reads as the box angle).</summary>
        public static float2 BoxRotationHandlePosition(float2 center, float angleDeg, float ringRadius)
        {
            sincos(radians(angleDeg), out var s, out var c);
            return center + Rotate(new float2(ringRadius, 0f), s, c);
        }

        // ----- field-from-drag math (compute the new authoring field from a dragged handle position) -----

        /// <summary>
        /// The new full box size after dragging the face-centre handle <paramref name="edgeIndex"/> (0:+X, 1:+Y,
        /// 2:−X, 3:−Y) to <paramref name="draggedLocalPos"/>. The box stays centred on <paramref name="center"/>
        /// (a symmetric resize about the centre — the simplest 2D box UX): the dragged position is un-rotated into
        /// the box frame, the moved axis's new half-extent is its distance from the centre, and the full size on
        /// that axis is twice that (the perpendicular axis is unchanged). Negative/zero is clamped to a tiny
        /// minimum.
        /// </summary>
        public static float2 BoxSizeFromEdgeDrag(
            float2 oldSize,
            int edgeIndex,
            float2 draggedLocalPos,
            float angleDeg,
            float2 center
        )
        {
            // un-rotate the dragged offset into the box's local axis frame
            sincos(radians(angleDeg), out var s, out var c);
            var d = draggedLocalPos - center;
            var localX = c * d.x + s * d.y; //  R(-angle) * d
            var localY = -s * d.x + c * d.y;
            var size = oldSize;
            const float minExtent = 1e-3f;
            switch (edgeIndex)
            {
                case 0:
                case 2:
                    size.x = max(minExtent, 2f * abs(localX));
                    break;
                case 1:
                case 3:
                    size.y = max(minExtent, 2f * abs(localY));
                    break;
            }
            return size;
        }

        /// <summary>The new circle radius after dragging the radius handle to
        /// <paramref name="draggedPos"/>: the distance from <paramref name="center"/>.</summary>
        public static float CircleRadiusFromDrag(float2 center, float2 draggedPos) =>
            max(1e-3f, length(draggedPos - center));

        /// <summary>The box/capsule z-angle (degrees) implied by a rotation-handle dragged to
        /// <paramref name="draggedPos"/> about <paramref name="center"/>.</summary>
        public static float AngleFromRotationDrag(float2 center, float2 draggedPos)
        {
            var d = draggedPos - center;
            if (lengthsq(d) < 1e-12f)
                return 0f;
            return degrees(atan2(d.y, d.x));
        }

        /// <summary>
        /// Map the two dragged capsule end-cap centres (<paramref name="capA"/>/<paramref name="capB"/>, in the
        /// shape's local space, already relative to the offset) and the dragged radius back to the package's
        /// size + vertical + angle authoring form. The spine direction sets the angle (folded so the long axis is
        /// X with the residual angle in (−90°, 90°]); the cap separation plus 2·radius is the major size; the
        /// minor size is 2·radius. The result round-trips through
        /// <see cref="PhysicsShape2DAuthoring.GetCapsuleCenters"/> to reproduce the dragged capsule.
        /// </summary>
        public static void CapsuleFieldsFromHandles(
            float2 capA,
            float2 capB,
            float radius,
            out float2 size,
            out bool vertical,
            out float angleDeg,
            out float2 center
        )
        {
            radius = max(1e-3f, radius);
            center = (capA + capB) * 0.5f;
            var axis = capB - capA;
            var half = 0.5f * length(axis); // half the centre-to-centre distance
            // GetCapsuleCenters places centres at ±(width/2 − radius) for a horizontal capsule, so
            //   width = 2·(half + radius),  height = 2·radius.
            var width = 2f * (half + radius);
            var height = 2f * radius;

            var deg = lengthsq(axis) < 1e-12f ? 0f : degrees(atan2(axis.y, axis.x));
            // normalise to (−90°, 90°]
            while (deg > 90f)
                deg -= 180f;
            while (deg <= -90f)
                deg += 180f;
            vertical = false;
            if (deg > 45f || deg <= -45f)
            {
                vertical = true;
                (width, height) = (height, width);
                deg += deg > 45f ? -90f : 90f;
            }
            size = new float2(width, height);
            angleDeg = deg;
        }

        // ----- helpers -----

        static float2 Rotate(float2 v, float s, float c) => new float2(c * v.x - s * v.y, s * v.x + c * v.y);
    }
}
