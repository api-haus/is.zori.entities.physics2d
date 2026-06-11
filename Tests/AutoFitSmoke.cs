using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Smoke check for the Phase-C shape auto-fit math (<see cref="PhysicsShape2DAutoFit"/>): a known point set
    /// fits the expected primitive, and every fit genuinely ENCLOSES the source points. Pure math, no SubScene,
    /// no Unity asset — the fit core is callable on a plain <see cref="float2"/> cloud, exactly as the inspector
    /// "Fit" dropdown (Phase D) will drive it through the Unity-source adapters. The square→box / circle→circle /
    /// elongated→capsule / many-gon→decompose cases pin the four enclosing kinds.
    /// </summary>
    [TestFixture]
    public sealed class AutoFitSmoke
    {
        static List<float2> Cloud(params float2[] pts) => new List<float2>(pts);

        static bool AllInside(IReadOnlyList<float2> pts, float2 center, float radius)
        {
            for (var i = 0; i < pts.Count; i++)
                if (lengthsq(pts[i] - center) > radius * radius + 1e-4f)
                    return false;
            return true;
        }

        // A square point set fits an axis-aligned box of that size, centred on the square.
        [Test]
        public void Square_FitsBoxOfThatSize()
        {
            var cloud = Cloud(
                new float2(-1f, -1f),
                new float2(1f, -1f),
                new float2(1f, 1f),
                new float2(-1f, 1f)
            );
            var fit = PhysicsShape2DAutoFit.FitBox(cloud, oriented: false);
            Assert.AreEqual(0f, fit.center.x, 1e-5f, "box centred on the square");
            Assert.AreEqual(0f, fit.center.y, 1e-5f);
            Assert.AreEqual(2f, fit.size.x, 1e-5f, "box width = the square extent");
            Assert.AreEqual(2f, fit.size.y, 1e-5f, "box height = the square extent");
            Assert.AreEqual(0f, fit.angleDeg, 1e-5f, "axis-aligned box has no angle");
        }

        // An oriented (PCA) box on a 45°-rotated rectangle recovers a ~45° angle and a tight (non-AABB) size.
        [Test]
        public void DiagonalRect_OrientedBox_RecoversAngleAndTightSize()
        {
            // a 4×1 rectangle rotated 45°, sampled at its 4 corners
            sincos(radians(45f), out var s, out var c);
            float2 R(float2 p) => new float2(c * p.x - s * p.y, s * p.x + c * p.y);
            var cloud = Cloud(
                R(new float2(-2f, -0.5f)),
                R(new float2(2f, -0.5f)),
                R(new float2(2f, 0.5f)),
                R(new float2(-2f, 0.5f))
            );
            var fit = PhysicsShape2DAutoFit.FitBox(cloud, oriented: true);
            // the oriented size should be ~4×1 (tight), far smaller than the ~3.18² AABB
            var lo = min(fit.size.x, fit.size.y);
            var hi = max(fit.size.x, fit.size.y);
            Assert.AreEqual(4f, hi, 1e-3f, "oriented box recovers the long extent");
            Assert.AreEqual(1f, lo, 1e-3f, "oriented box recovers the short extent");
            Assert.AreEqual(
                45f,
                abs(fit.angleDeg),
                1f,
                "oriented box recovers the ~45° principal angle"
            );
        }

        // A ring of points fits the minimum-enclosing circle (Welzl): centre at origin, radius = the ring radius.
        [Test]
        public void Ring_FitsEnclosingCircle()
        {
            var cloud = new List<float2>();
            for (var i = 0; i < 16; i++)
            {
                var a = radians(360f / 16f * i);
                sincos(a, out var s, out var co);
                cloud.Add(new float2(co, s) * 3f);
            }
            var fit = PhysicsShape2DAutoFit.FitCircle(cloud);
            Assert.AreEqual(0f, fit.center.x, 1e-3f, "circle centred on the ring");
            Assert.AreEqual(0f, fit.center.y, 1e-3f);
            Assert.AreEqual(3f, fit.radius, 1e-3f, "radius = the ring radius (minimal enclosing)");
            Assert.IsTrue(AllInside(cloud, fit.center, fit.radius), "every ring point is enclosed");
        }

        // The MEC of a square is the circumscribed circle (radius = half-diagonal), and it encloses every corner.
        [Test]
        public void Square_EnclosingCircle_IsCircumscribed()
        {
            var cloud = Cloud(
                new float2(-1f, -1f),
                new float2(1f, -1f),
                new float2(1f, 1f),
                new float2(-1f, 1f)
            );
            var fit = PhysicsShape2DAutoFit.FitCircle(cloud);
            Assert.AreEqual(sqrt(2f), fit.radius, 1e-3f, "MEC radius = half the square diagonal");
            Assert.IsTrue(AllInside(cloud, fit.center, fit.radius));
        }

        // An elongated point set fits a capsule whose long axis spans the elongation and whose radius covers the
        // perpendicular half-extent; the result encloses the cloud.
        [Test]
        public void Elongated_FitsCapsule_Encloses()
        {
            // a horizontal 6-long, 2-wide cluster
            var cloud = Cloud(
                new float2(-3f, 0f),
                new float2(3f, 0f),
                new float2(0f, 1f),
                new float2(0f, -1f),
                new float2(-2f, 0.5f),
                new float2(2f, -0.5f)
            );
            var fit = PhysicsShape2DAutoFit.FitCapsule(cloud);
            // long axis is horizontal -> not vertical, size width >= height
            Assert.IsFalse(fit.vertical, "the major axis is horizontal");
            Assert.GreaterOrEqual(
                max(fit.size.x, fit.size.y),
                5.9f,
                "capsule long extent spans the elongation"
            );
            // reconstruct the capsule (centres + radius) and confirm every point is within radius of the segment
            var auth = new UnityEngine.GameObject("cap").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                auth.CapsuleSize = fit.size;
                auth.CapsuleVertical = fit.vertical;
                auth.CapsuleAngle = fit.angleDeg;
                auth.GetCapsuleCenters(out var r, out var c1, out var c2);
                c1 += fit.center;
                c2 += fit.center;
                for (var i = 0; i < cloud.Count; i++)
                {
                    var d = DistToSegment(cloud[i], c1, c2);
                    Assert.LessOrEqual(
                        d,
                        r + 1e-3f,
                        $"point {i} ({cloud[i]}) is within the capsule radius of its segment"
                    );
                }
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        // A convex pentagon's hull is the pentagon (5 verts), CCW, enclosing every input point.
        [Test]
        public void ConvexPentagon_HullIsItself()
        {
            var cloud = new List<float2>();
            for (var i = 0; i < 5; i++)
            {
                var a = radians(360f / 5f * i + 18f);
                sincos(a, out var s, out var co);
                cloud.Add(new float2(co, s) * 2f);
            }
            var hull = PhysicsShape2DAutoFit.ConvexHull(cloud);
            Assert.AreEqual(5, hull.Length, "the hull of a convex pentagon is the pentagon");
            Assert.IsTrue(PhysicsShape2DAutoFit.IsConvex(hull), "the hull is convex");
        }

        // A small convex polygon source fits a single-hull Polygon (decompose off); a >8-vertex outline fits the
        // decompose path (decompose on).
        [Test]
        public void Polygon_SmallConvex_SingleHull_LargeOutline_Decomposes()
        {
            var auth = new UnityEngine.GameObject("poly").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                // 5-gon -> single hull, decompose off
                var small = new List<float2>();
                for (var i = 0; i < 5; i++)
                {
                    var a = radians(360f / 5f * i + 18f);
                    sincos(a, out var s, out var co);
                    small.Add(new float2(co, s) * 2f);
                }
                Assert.IsTrue(
                    PhysicsShape2DAutoFit.FitTo(
                        auth,
                        small,
                        PhysicsShape2DKind.Polygon,
                        Unity.Mathematics.float2.zero
                    )
                );
                Assert.AreEqual(PhysicsShape2DKind.Polygon, auth.Kind);
                Assert.IsFalse(auth.PolygonDecompose, "a <=8-vertex convex hull stays single-hull");
                Assert.LessOrEqual(auth.Vertices.Length, PhysicsShape2DAutoFit.MaxPolygonVertices);

                // a 16-gon -> hull has 16 verts > 8 -> decompose on
                var big = new List<float2>();
                for (var i = 0; i < 16; i++)
                {
                    var a = radians(360f / 16f * i);
                    sincos(a, out var s, out var co);
                    big.Add(new float2(co, s) * 2f);
                }
                Assert.IsTrue(
                    PhysicsShape2DAutoFit.FitTo(
                        auth,
                        big,
                        PhysicsShape2DKind.Polygon,
                        Unity.Mathematics.float2.zero
                    )
                );
                Assert.IsTrue(
                    auth.PolygonDecompose,
                    "a >8-vertex outline takes the decompose path"
                );
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        // FitTo writes the authoring fields and offsets a circle by the source offset.
        [Test]
        public void FitTo_Circle_WritesFieldsAndOffset()
        {
            var auth = new UnityEngine.GameObject("circ").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                var cloud = Cloud(
                    new float2(-2f, 0f),
                    new float2(2f, 0f),
                    new float2(0f, 2f),
                    new float2(0f, -2f)
                );
                var srcOffset = new float2(10f, 5f);
                Assert.IsTrue(
                    PhysicsShape2DAutoFit.FitTo(auth, cloud, PhysicsShape2DKind.Circle, srcOffset)
                );
                Assert.AreEqual(PhysicsShape2DKind.Circle, auth.Kind);
                Assert.AreEqual(2f, auth.Radius, 1e-3f, "radius encloses the diamond");
                Assert.AreEqual(10f, auth.Offset.x, 1e-3f, "offset = source offset + fit centre");
                Assert.AreEqual(5f, auth.Offset.y, 1e-3f);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        // An empty cloud is a no-op (returns false, leaves the target unchanged).
        [Test]
        public void EmptyCloud_NoChange()
        {
            var auth = new UnityEngine.GameObject("empty").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                Assert.IsFalse(
                    PhysicsShape2DAutoFit.FitTo(
                        auth,
                        new List<float2>(),
                        PhysicsShape2DKind.Box,
                        Unity.Mathematics.float2.zero
                    )
                );
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(auth.gameObject);
            }
        }

        static float DistToSegment(float2 p, float2 a, float2 b)
        {
            var ab = b - a;
            var t = lengthsq(ab) < 1e-12f ? 0f : saturate(dot(p - a, ab) / lengthsq(ab));
            return length(p - (a + t * ab));
        }
    }
}
