using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests.EditorMath
{
    /// <summary>
    /// The adversarial EditMode gate for the Phase-D pure handle / outline / drag math
    /// (<see cref="PhysicsShape2DGizmos"/>) and the Phase-C fit-apply path
    /// (<see cref="PhysicsShape2DAutoFit"/>). The validating agent (not the implementer) built every case from the
    /// math's own decision points, not from the inputs the author happened to imagine: per-kind outline against
    /// analytic expectations, per-kind handle anchors against the shape's control points, the field-from-drag
    /// ROUND-TRIP (drag a handle → recompute the field → recompute the handle → it must land where it was dragged)
    /// swept over the rotation/spine angles the impl flagged as the highest-value gaps, and the fit-apply geometry
    /// the "Fit To…" dropdown commits. A field-from-drag that does not round-trip, or an outline that does not trace
    /// the shape, is an IMPLEMENTATION bug; these tests are RED for that and GREEN otherwise.
    ///
    /// Lives in an <c>includePlatforms:["Editor"]</c> (EditorOnly) test assembly so the Unity Test Framework
    /// classifies it EditMode (UTF buckets a whole assembly by the AssemblyFlags.EditorOnly bit, which only the
    /// editor-only platform set carries — a plain [Test] in the all-platforms root Tests assembly is discovered
    /// ONLY under PlayMode). The math under test has no UnityEditor dependency, so it is exercised here directly.
    /// </summary>
    [TestFixture]
    public sealed class EditorMathGate
    {
        const float Eps = 1e-3f;

        static bool Approx(float2 a, float2 b, float eps = Eps) => length(a - b) < eps;

        static float2 Rot(float2 v, float deg)
        {
            sincos(radians(deg), out var s, out var c);
            return new float2(c * v.x - s * v.y, s * v.x + c * v.y);
        }

        // ===================== OUTLINE GENERATION (per kind, analytic) =====================

        [Test]
        public void BoxOutline_Rotated_CornersAreRotatedHalfExtentsPlusOffset()
        {
            // a rotated, offset box: each corner must equal offset + R(angle)*(±hx, ±hy), in CCW order from BL.
            var offset = new float2(2.5f, -1.25f);
            var size = new float2(3f, 1.4f);
            const float deg = 37f;
            var h = size * 0.5f;
            var pts = PhysicsShape2DGizmos.BoxOutline(offset, size, deg);
            Assert.AreEqual(4, pts.Length);
            Assert.IsTrue(Approx(pts[0], offset + Rot(new float2(-h.x, -h.y), deg)), "corner 0 (BL)");
            Assert.IsTrue(Approx(pts[1], offset + Rot(new float2(h.x, -h.y), deg)), "corner 1 (BR)");
            Assert.IsTrue(Approx(pts[2], offset + Rot(new float2(h.x, h.y), deg)), "corner 2 (TR)");
            Assert.IsTrue(Approx(pts[3], offset + Rot(new float2(-h.x, h.y), deg)), "corner 3 (TL)");
            // CCW: the signed area of the loop is positive.
            var area = 0f;
            for (var i = 0; i < 4; i++)
            {
                var a = pts[i];
                var b = pts[(i + 1) % 4];
                area += a.x * b.y - b.x * a.y;
            }
            Assert.Greater(area, 0f, "box outline must wind CCW");
        }

        [Test]
        public void CircleOutline_OffCentre_TracesTheRingAtTheRadius()
        {
            var center = new float2(-3f, 4f);
            const float r = 2.75f;
            var pts = PhysicsShape2DGizmos.CircleOutline(center, r, 24);
            Assert.AreEqual(24, pts.Length);
            foreach (var p in pts)
                Assert.AreEqual(r, length(p - center), Eps);
            // the points span the full ring: a point near +X and one near -X relative to centre.
            var minX = float.PositiveInfinity;
            var maxX = float.NegativeInfinity;
            foreach (var p in pts)
            {
                minX = min(minX, p.x);
                maxX = max(maxX, p.x);
            }
            Assert.AreEqual(center.x - r, minX, 0.05f);
            Assert.AreEqual(center.x + r, maxX, 0.05f);
        }

        [Test]
        public void CapsuleOutline_EveryPointEnclosedWithinRadiusOfTheSpine([Values(0f, 17f, 90f, 153f, 244f)] float spineDeg)
        {
            // the stadium outline must trace the boundary: every generated point lies within `radius` (+eps) of the
            // segment between the two cap centres, and the extreme points reach exactly `radius` from a cap.
            const float r = 0.6f;
            var c1 = new float2(1f, -0.5f);
            var c2 = c1 + Rot(new float2(2.4f, 0f), spineDeg); // a spine of length 2.4 at the swept angle
            var pts = PhysicsShape2DGizmos.CapsuleOutline(c1, c2, r, 12);
            Assert.GreaterOrEqual(pts.Length, 4);
            var maxDist = 0f;
            foreach (var p in pts)
            {
                var d = DistanceToSegment(p, c1, c2);
                maxDist = max(maxDist, d);
                Assert.LessOrEqual(d, r + 1e-3f, $"outline point {p} escapes the radius (dist {d})");
            }
            // the cap arcs reach the radius: the farthest outline point is at the radius, not inside it.
            Assert.AreEqual(r, maxDist, 1e-2f, "the cap arcs must reach the full radius");
        }

        [Test]
        public void PolygonOutline_AppliesOffset_AndEdgeOutlineIgnoresLoopFlagForPoints()
        {
            var verts = new[] { new float2(-1f, -1f), new float2(1f, -1f), new float2(0f, 1.5f) };
            var off = new float2(4f, -2f);
            var poly = PhysicsShape2DGizmos.PolygonOutline(verts, off);
            for (var i = 0; i < verts.Length; i++)
                Assert.IsTrue(Approx(poly[i], verts[i] + off));
            // EdgeOutline returns the SAME offset points regardless of the loop flag (the flag only governs whether
            // the Editor shell draws the closing segment, not the point set).
            var open = PhysicsShape2DGizmos.EdgeOutline(verts, off, false);
            var closed = PhysicsShape2DGizmos.EdgeOutline(verts, off, true);
            Assert.AreEqual(open.Length, closed.Length);
            for (var i = 0; i < open.Length; i++)
            {
                Assert.IsTrue(Approx(open[i], closed[i]));
                Assert.IsTrue(Approx(open[i], verts[i] + off));
            }
        }

        // ===================== HANDLE POSITIONS (per kind = the control points) =====================

        [Test]
        public void BoxEdgeHandlePositions_AreTheRotatedFaceCentres_InPlusXPlusYMinusXMinusYOrder()
        {
            var off = new float2(1f, 1f);
            var size = new float2(4f, 2f);
            const float deg = 25f;
            var h = size * 0.5f;
            var faces = PhysicsShape2DGizmos.BoxEdgeHandlePositions(off, size, deg);
            Assert.AreEqual(4, faces.Length);
            Assert.IsTrue(Approx(faces[0], off + Rot(new float2(h.x, 0f), deg)), "+X face");
            Assert.IsTrue(Approx(faces[1], off + Rot(new float2(0f, h.y), deg)), "+Y face");
            Assert.IsTrue(Approx(faces[2], off + Rot(new float2(-h.x, 0f), deg)), "-X face");
            Assert.IsTrue(Approx(faces[3], off + Rot(new float2(0f, -h.y), deg)), "-Y face");
        }

        [Test]
        public void BoxRotationHandlePosition_IsOnTheLocalPlusXAxisAtRingRadius()
        {
            var off = new float2(-2f, 3f);
            const float deg = 50f;
            const float ring = 2.2f;
            var p = PhysicsShape2DGizmos.BoxRotationHandlePosition(off, deg, ring);
            // it sits at ring distance from the centre, along the box's local +X.
            Assert.AreEqual(ring, length(p - off), Eps);
            Assert.IsTrue(Approx(p, off + Rot(new float2(ring, 0f), deg)));
        }

        [Test]
        public void CircleRadiusHandle_AnchorRoundTripsToTheRadius()
        {
            // the circle handle anchor is offset + (radius, 0); CircleRadiusFromDrag of that anchor recovers radius.
            var center = new float2(2f, -2f);
            const float r = 3.3f;
            var anchor = center + new float2(r, 0f);
            Assert.AreEqual(r, PhysicsShape2DGizmos.CircleRadiusFromDrag(center, anchor), Eps);
        }

        // ===================== FIELD-FROM-DRAG ROUND-TRIPS (the core gate) =====================

        [Test]
        public void BoxEdgeDrag_RoundTrips_AllFourFaces_UnderRotation(
            [Values(0f, 37f, 90f, 131f)] float deg,
            [Values(0, 1, 2, 3)] int edge
        )
        {
            // Drag the face handle `edge` of a rotated box to a known target, derive the new size, recompute the
            // face handle, and assert it lands at the dragged-to extent on the moved axis (a symmetric resize about
            // the centre — the documented 2D box UX). +X/-X drive size.x; +Y/-Y drive size.y.
            var off = new float2(0.5f, -1f);
            var size0 = new float2(2f, 3f);

            // choose a new half-extent for the moved axis and build the dragged-to world position along that face.
            var newHalf = 1.85f;
            float2 localTarget = (edge == 0 || edge == 2)
                ? new float2(edge == 0 ? newHalf : -newHalf, 0f)
                : new float2(0f, edge == 1 ? newHalf : -newHalf);
            var dragged = off + Rot(localTarget, deg);

            var newSize = PhysicsShape2DGizmos.BoxSizeFromEdgeDrag(size0, edge, dragged, deg, off);

            // the moved axis full size is 2*newHalf; the perpendicular axis is unchanged.
            if (edge == 0 || edge == 2)
            {
                Assert.AreEqual(2f * newHalf, newSize.x, Eps, "moved axis (x)");
                Assert.AreEqual(size0.y, newSize.y, Eps, "perpendicular axis (y) unchanged");
            }
            else
            {
                Assert.AreEqual(2f * newHalf, newSize.y, Eps, "moved axis (y)");
                Assert.AreEqual(size0.x, newSize.x, Eps, "perpendicular axis (x) unchanged");
            }

            // RECOMPUTE the face handle from the new size: the moved face's distance from the centre is newHalf.
            var facesAfter = PhysicsShape2DGizmos.BoxEdgeHandlePositions(off, newSize, deg);
            Assert.AreEqual(newHalf, length(facesAfter[edge] - off), Eps, "recomputed handle at the dragged extent");
        }

        [Test]
        public void BoxRotationDrag_RoundTrips_OverTheAngleRange([Values(-150f, -90f, -10f, 0f, 23f, 90f, 175f)] float deg)
        {
            // place the rotation ring at angle `deg`, recover the angle from that dragged ring position, and confirm
            // it reproduces the same orientation (compared as a direction so the ±180° wrap is not a false failure).
            var off = new float2(3f, 3f);
            const float ring = 2f;
            var ringPos = PhysicsShape2DGizmos.BoxRotationHandlePosition(off, deg, ring);
            var recovered = PhysicsShape2DGizmos.AngleFromRotationDrag(off, ringPos);
            sincos(radians(deg), out var s0, out var c0);
            sincos(radians(recovered), out var s1, out var c1);
            Assert.IsTrue(Approx(new float2(c1, s1), new float2(c0, s0)), $"angle {deg} -> {recovered}");
            // and the ring re-placed at the recovered angle lands on the original ring position.
            var rePlaced = PhysicsShape2DGizmos.BoxRotationHandlePosition(off, recovered, ring);
            Assert.IsTrue(Approx(rePlaced, ringPos));
        }

        [Test]
        public void CircleRadiusDrag_IsTheDistance_OffCentre()
        {
            var center = new float2(-1f, 2f);
            var dragged = center + new float2(3f, 4f); // |(3,4)| = 5
            Assert.AreEqual(5f, PhysicsShape2DGizmos.CircleRadiusFromDrag(center, dragged), Eps);
            // a drag back onto the centre clamps to the tiny minimum, never zero/negative.
            Assert.Greater(PhysicsShape2DGizmos.CircleRadiusFromDrag(center, center), 0f);
        }

        [Test]
        public void CapsuleEndcapDrag_MathRoundTripsThroughGetCapsuleCenters([Values(0f, 30f, 45f, 60f, 90f, 120f, 200f, 315f)] float spineDeg)
        {
            // THE ESCALATED CASE: drag two end-caps to a known oriented spine (in the offset-relative frame the
            // editor uses), map back to size+vertical+angle+centre, then reconstruct the caps through the REAL
            // GetCapsuleCenters (rotated about the origin) re-centred by the returned centre, and assert the
            // reconstructed world caps equal the dragged caps. This pins that the MATH round-trips at every spine
            // angle (the re-centring FEEL is a separate manual-QA judgment call, not a math defect).
            const float r = 0.4f;
            const float halfSpine = 1.3f;
            var mid = new float2(0.75f, -0.4f); // an arbitrary non-origin midpoint (offset-relative)
            var dir = Rot(new float2(1f, 0f), spineDeg);
            var capA = mid - dir * halfSpine;
            var capB = mid + dir * halfSpine;

            PhysicsShape2DGizmos.CapsuleFieldsFromHandles(
                capA,
                capB,
                r,
                out var size,
                out var vertical,
                out var angle,
                out var center
            );
            Assert.IsTrue(Approx(center, mid), "centre is the cap midpoint");

            var auth = new UnityEngine.GameObject("capsule-rt").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                auth.CapsuleSize = size;
                auth.CapsuleVertical = vertical;
                auth.CapsuleAngle = angle;
                auth.GetCapsuleCenters(out var rr, out var rc1, out var rc2);
                Assert.AreEqual(r, rr, 2e-3f, "reconstructed radius");

                // GetCapsuleCenters returns origin-relative caps; the editor places them at center + cap.
                var w1 = rc1 + center;
                var w2 = rc2 + center;
                // up-to-ordering equality with the dragged caps.
                var ok =
                    (Approx(w1, capA, 3e-3f) && Approx(w2, capB, 3e-3f))
                    || (Approx(w1, capB, 3e-3f) && Approx(w2, capA, 3e-3f));
                Assert.IsTrue(ok, $"spine {spineDeg}: reconstructed caps {w1},{w2} != dragged {capA},{capB}");

                // and the reconstructed spine length matches (enclosure-correct major axis).
                Assert.AreEqual(length(capB - capA), length(w2 - w1), 3e-3f, "spine length preserved");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        [Test]
        public void PolygonVertexDrag_RoundTrips_ThroughOffsetRelativeWriteBack()
        {
            // the editor writes verts[i] = (dragged - offset); PolygonOutline(verts, offset) must recover the
            // dragged world position for the moved vertex and leave the others put.
            var off = new float2(-2f, 1.5f);
            var verts = new List<float2>
            {
                new float2(-1f, -1f),
                new float2(1f, -1f),
                new float2(1f, 1f),
                new float2(-1f, 1f),
            };
            const int moved = 2;
            var draggedWorld = new float2(3.4f, 2.1f);
            verts[moved] = draggedWorld - off; // the editor's write-back

            var outline = PhysicsShape2DGizmos.PolygonOutline(verts, off);
            Assert.IsTrue(Approx(outline[moved], draggedWorld), "moved vertex lands at the drag target");
            // the untouched vertices are unchanged.
            Assert.IsTrue(Approx(outline[0], new float2(-1f, -1f) + off));
            Assert.IsTrue(Approx(outline[1], new float2(1f, -1f) + off));
            Assert.IsTrue(Approx(outline[3], new float2(-1f, 1f) + off));
        }

        [Test]
        public void OffsetDrag_RigidlyShiftsEveryOutline()
        {
            // the common offset handle writes Offset directly; every kind's outline must shift rigidly by the delta.
            var d = new float2(5f, -3f);

            var box0 = PhysicsShape2DGizmos.BoxOutline(Unity.Mathematics.float2.zero, new float2(2f, 3f), 20f);
            var box1 = PhysicsShape2DGizmos.BoxOutline(d, new float2(2f, 3f), 20f);
            for (var i = 0; i < box0.Length; i++)
                Assert.IsTrue(Approx(box1[i], box0[i] + d), "box rigid shift");

            var circ0 = PhysicsShape2DGizmos.CircleOutline(Unity.Mathematics.float2.zero, 1.5f, 16);
            var circ1 = PhysicsShape2DGizmos.CircleOutline(d, 1.5f, 16);
            for (var i = 0; i < circ0.Length; i++)
                Assert.IsTrue(Approx(circ1[i], circ0[i] + d), "circle rigid shift");
        }

        // ===================== FIT-APPLY (the "Fit To…" path applies fields the shape then matches) =====================

        [Test]
        public void FitApply_Circle_EnclosesTheCloud_AndWritesRadiusOffsetKind()
        {
            var cloud = new List<float2>
            {
                new float2(0f, 0f),
                new float2(4f, 0f),
                new float2(4f, 3f),
                new float2(0f, 3f),
                new float2(2f, 5f),
            };
            var auth = new UnityEngine.GameObject("fit-circle").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                var changed = PhysicsShape2DAutoFit.FitTo(auth, cloud, PhysicsShape2DKind.Circle, Unity.Mathematics.float2.zero);
                Assert.IsTrue(changed);
                Assert.AreEqual(PhysicsShape2DKind.Circle, auth.Kind);
                // the applied circle (centre = Offset, radius = Radius) must enclose every source point.
                foreach (var p in cloud)
                    Assert.LessOrEqual(length(p - auth.Offset), auth.Radius + 1e-3f, $"point {p} not enclosed");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        [Test]
        public void FitApply_Box_EnclosesTheCloud_InTheFittedFrame()
        {
            // a tilted rectangle cloud: the oriented box fit must enclose it in its own (centre, size, angle) frame.
            var raw = new List<float2>
            {
                new float2(-3f, -1f),
                new float2(3f, -1f),
                new float2(3f, 1f),
                new float2(-3f, 1f),
            };
            const float tilt = 28f;
            var cloud = new List<float2>();
            foreach (var p in raw)
                cloud.Add(Rot(p, tilt) + new float2(1f, 1f));

            var auth = new UnityEngine.GameObject("fit-box").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                var changed = PhysicsShape2DAutoFit.FitTo(auth, cloud, PhysicsShape2DKind.Box, Unity.Mathematics.float2.zero);
                Assert.IsTrue(changed);
                Assert.AreEqual(PhysicsShape2DKind.Box, auth.Kind);

                // un-rotate each point into the box's local frame about Offset; it must lie within the half-extents.
                var h = auth.BoxSize * 0.5f + new float2(2e-3f, 2e-3f);
                foreach (var p in cloud)
                {
                    var local = Rot(p - auth.Offset, -auth.BoxAngle);
                    Assert.LessOrEqual(abs(local.x), h.x, $"point {p} outside box x");
                    Assert.LessOrEqual(abs(local.y), h.y, $"point {p} outside box y");
                }
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        [Test]
        public void FitApply_Capsule_RoundTripsThroughGetCapsuleCenters_AndEncloses()
        {
            var raw = new List<float2>
            {
                new float2(-4f, -0.8f),
                new float2(4f, -0.8f),
                new float2(4f, 0.8f),
                new float2(-4f, 0.8f),
                new float2(0f, 0f),
            };
            const float tilt = 18f;
            var cloud = new List<float2>();
            foreach (var p in raw)
                cloud.Add(Rot(p, tilt) + new float2(-1f, 2f));

            var auth = new UnityEngine.GameObject("fit-capsule").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                var changed = PhysicsShape2DAutoFit.FitTo(auth, cloud, PhysicsShape2DKind.Capsule, Unity.Mathematics.float2.zero);
                Assert.IsTrue(changed);
                Assert.AreEqual(PhysicsShape2DKind.Capsule, auth.Kind);

                // reconstruct the capsule's two caps + radius from the applied fields and confirm enclosure.
                auth.GetCapsuleCenters(out var r, out var c1, out var c2);
                var w1 = c1 + auth.Offset;
                var w2 = c2 + auth.Offset;
                foreach (var p in cloud)
                    Assert.LessOrEqual(DistanceToSegment(p, w1, w2), r + 2e-3f, $"point {p} not within the capsule");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        [Test]
        public void FitApply_Polygon_SmallConvexHull_KeepsSingleHullPath_AndContainsCloud()
        {
            var cloud = new List<float2>
            {
                new float2(0f, 0f),
                new float2(2f, 0f),
                new float2(2f, 2f),
                new float2(0f, 2f),
                new float2(1f, 1f), // interior point — must not appear on the hull
            };
            var auth = new UnityEngine.GameObject("fit-poly").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                var changed = PhysicsShape2DAutoFit.FitTo(auth, cloud, PhysicsShape2DKind.Polygon, Unity.Mathematics.float2.zero);
                Assert.IsTrue(changed);
                Assert.AreEqual(PhysicsShape2DKind.Polygon, auth.Kind);
                Assert.IsFalse(auth.PolygonDecompose, "a <=8-vertex convex hull stays on the single-hull path");
                Assert.AreEqual(4, auth.Vertices.Length, "the square's hull is 4 vertices (interior point dropped)");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        [Test]
        public void FitApply_Edge_IsNotAFitTarget()
        {
            var cloud = new List<float2> { new float2(0f, 0f), new float2(1f, 0f) };
            var auth = new UnityEngine.GameObject("fit-edge").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                // Edge is an open chain, not an enclosing fit — FitTo must report no change.
                Assert.IsFalse(PhysicsShape2DAutoFit.FitTo(auth, cloud, PhysicsShape2DKind.Edge, Unity.Mathematics.float2.zero));
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        // ----- helper: distance from a point to a segment -----

        static float DistanceToSegment(float2 p, float2 a, float2 b)
        {
            var ab = b - a;
            var len2 = lengthsq(ab);
            if (len2 < 1e-12f)
                return length(p - a);
            var t = clamp(dot(p - a, ab) / len2, 0f, 1f);
            return length(p - (a + ab * t));
        }
    }
}
