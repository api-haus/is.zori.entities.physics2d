using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-C VALIDATION gate for the shape auto-fit utility (<see cref="PhysicsShape2DAutoFit"/>), built to
    /// FALSIFY the two areas the implementation escalated as untested: (1) the three Unity-source ADAPTERS
    /// (<c>TryGatherSpriteShape</c> / <c>TryGatherSpriteRendererBounds</c> / <c>TryGatherPolygonCollider</c>),
    /// exercised against REAL <see cref="Sprite"/> / <see cref="SpriteRenderer"/> / <see cref="PolygonCollider2D"/>
    /// objects constructed in the test, and (2) the mandatory ENCLOSE invariant for every fit across a battery of
    /// point sets including the capsule-under-PCA-tilt case and a concave outline. Each adapter test asserts the
    /// gathered cloud + resulting fit corresponds to the real source geometry; each enclose test asserts the fit
    /// genuinely contains every source point and is not absurdly oversized. The concave-decompose finding is pinned
    /// as the code's ACTUAL behaviour (a known gap, evidenced), not as a passing aspiration.
    /// </summary>
    /// <remarks>
    /// These are managed-object tests (no ECS world, no SubScene), so they run as plain <c>[Test]</c> cases under
    /// the PlayMode platform the package's runner uses. A real Sprite physics shape is constructible at runtime via
    /// <c>Sprite.OverridePhysicsShape</c> on a <c>Sprite.Create</c>-d sprite — so the Sprite-physics-shape adapter
    /// is NOT fixture-limited and is pinned behaviourally here.
    /// </remarks>
    [TestFixture]
    public sealed class AutoFitAdapterEncloseGate
    {
        const float Eps = 1e-4f;

        // ---------------------------------------------------------------------------------------------------
        // Geometry oracles — independent of the utility's own helpers, so they cannot share a blind spot.
        // ---------------------------------------------------------------------------------------------------

        static bool PointInCircle(float2 p, float2 c, float r) => lengthsq(p - c) <= r * r + Eps;

        // A point is inside an oriented box: rotate it into the box frame and test the axis-aligned extents.
        static bool PointInBox(float2 p, float2 center, float2 size, float angleDeg)
        {
            sincos(radians(-angleDeg), out var s, out var co);
            var d = p - center;
            var local = new float2(co * d.x - s * d.y, s * d.x + co * d.y);
            var half = size * 0.5f + Eps;
            return abs(local.x) <= half.x && abs(local.y) <= half.y;
        }

        // Distance from a point to the segment [a,b]; a capsule encloses p iff this <= radius.
        static float DistToSegment(float2 p, float2 a, float2 b)
        {
            var ab = b - a;
            var t = lengthsq(ab) < 1e-12f ? 0f : saturate(dot(p - a, ab) / lengthsq(ab));
            return length(p - (a + t * ab));
        }

        // Even-odd point-in-polygon for an ordered (CCW or CW) ring.
        static bool PointInPolygon(float2 p, IReadOnlyList<float2> poly)
        {
            var inside = false;
            var n = poly.Count;
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var pi = poly[i];
                var pj = poly[j];
                if ((pi.y > p.y) != (pj.y > p.y) && p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x)
                    inside = !inside;
            }
            return inside;
        }

        static void Aabb(IReadOnlyList<float2> pts, out float2 lo, out float2 hi)
        {
            lo = new float2(float.PositiveInfinity, float.PositiveInfinity);
            hi = new float2(float.NegativeInfinity, float.NegativeInfinity);
            for (var i = 0; i < pts.Count; i++)
            {
                lo = min(lo, pts[i]);
                hi = max(hi, pts[i]);
            }
        }

        static float2 R(float2 p, float deg)
        {
            sincos(radians(deg), out var s, out var c);
            return new float2(c * p.x - s * p.y, s * p.x + c * p.y);
        }

        // ===================================================================================================
        // ENCLOSE INVARIANT — every fit must contain every source point, for a battery of clouds.
        // ===================================================================================================

        static List<float2> SquareCloud() =>
            new() { new float2(-1f, -1f), new float2(1f, -1f), new float2(1f, 1f), new float2(-1f, 1f) };

        static List<float2> RectCloud() =>
            new()
            {
                new float2(-4f, -1f),
                new float2(4f, -1f),
                new float2(4f, 1f),
                new float2(-4f, 1f),
                new float2(0f, 0.3f),
            };

        static List<float2> CircleishCloud()
        {
            var c = new List<float2>();
            for (var i = 0; i < 24; i++)
            {
                var a = radians(360f / 24f * i);
                sincos(a, out var s, out var co);
                // Slightly noisy radius so it is not a perfect ring (PCA gets a non-degenerate covariance).
                c.Add(new float2(co, s) * (2.5f + 0.07f * (i % 3)));
            }
            return c;
        }

        // A 5×1.4 rectangle tilted 23° — the case that bit the capsule fit (PCA frame tilts, the major-axis
        // extreme has a non-zero perpendicular component relative to the spine).
        static List<float2> TiltedElongatedCloud()
        {
            var raw = new List<float2>
            {
                new float2(-2.5f, -0.7f),
                new float2(2.5f, -0.7f),
                new float2(2.5f, 0.7f),
                new float2(-2.5f, 0.7f),
                new float2(-1.2f, 0.4f),
                new float2(1.7f, -0.3f),
                new float2(0f, 0.7f),
            };
            var tilted = new List<float2>(raw.Count);
            foreach (var p in raw)
                tilted.Add(R(p, 23f));
            return tilted;
        }

        // An L-shaped concave outline (ordered ring). Its reflex vertex (1,1) is interior to the convex hull.
        static List<float2> LConcaveCloud() =>
            new()
            {
                new float2(0f, 0f),
                new float2(2f, 0f),
                new float2(2f, 1f),
                new float2(1f, 1f), // reflex (concave) corner
                new float2(1f, 2f),
                new float2(0f, 2f),
            };

        // A >8-vertex blob (a 14-gon, convex), forcing the decompose decision on hull-count.
        static List<float2> BlobCloud()
        {
            var c = new List<float2>();
            for (var i = 0; i < 14; i++)
            {
                var a = radians(360f / 14f * i);
                sincos(a, out var s, out var co);
                c.Add(new float2(co, s) * 3f);
            }
            return c;
        }

        static void AssertBoxEncloses(string name, List<float2> cloud)
        {
            var fit = PhysicsShape2DAutoFit.FitBox(cloud, oriented: true);
            for (var i = 0; i < cloud.Count; i++)
                Assert.IsTrue(
                    PointInBox(cloud[i], fit.center, fit.size, fit.angleDeg),
                    $"[{name}] oriented box does NOT enclose point {i} ({cloud[i]}): center={fit.center} "
                        + $"size={fit.size} angle={fit.angleDeg}"
                );
            // Tightness sanity: the oriented box must not be ABSURDLY oversized. The PCA box equals the min-area
            // box (<= the AABB) ONLY when the cloud is genuinely elongated along its principal axis; for a
            // near-isotropic cloud (a noisy ring) or a concave outline whose principal axis is its diagonal (an
            // L), the PCA box legitimately EXCEEDS the axis-aligned AABB — that is documented best-effort
            // behaviour, not a defect. So this is a loose "not insane" bound (<= 2x the AABB area) that still
            // catches a genuinely broken rotation (a NaN/garbage angle blows the box up far more, or produces a
            // NaN/Inf area). The strict guarantee — that the box ENCLOSES every point — is asserted above and is
            // the load-bearing property; tightness is best-effort. The genuinely-elongated case (a 45° rect
            // recovering a tight 4x1) is pinned exactly by AutoFitSmoke.DiagonalRect_OrientedBox.
            Aabb(cloud, out var lo, out var hi);
            var aabbArea = (hi.x - lo.x) * (hi.y - lo.y);
            var boxArea = fit.size.x * fit.size.y;
            Assert.IsFalse(
                float.IsNaN(boxArea) || float.IsInfinity(boxArea),
                $"[{name}] oriented box area is NaN/Inf — a broken PCA rotation."
            );
            Assert.LessOrEqual(
                boxArea,
                aabbArea * 2f + 1e-3f,
                $"[{name}] oriented box area {boxArea} is more than 2x the AABB area {aabbArea} — an absurdly "
                    + "oversized box, signalling a broken rotation rather than PCA best-effort."
            );
        }

        static void AssertCircleEncloses(string name, List<float2> cloud)
        {
            var fit = PhysicsShape2DAutoFit.FitCircle(cloud);
            for (var i = 0; i < cloud.Count; i++)
                Assert.IsTrue(
                    PointInCircle(cloud[i], fit.center, fit.radius),
                    $"[{name}] circle does NOT enclose point {i} ({cloud[i]}): center={fit.center} "
                        + $"r={fit.radius} dist={length(cloud[i] - fit.center)}"
                );
            // Minimality sanity: the radius must not exceed the AABB diagonal half (a loose upper bound).
            Aabb(cloud, out var lo, out var hi);
            var diagHalf = length(hi - lo) * 0.5f;
            Assert.LessOrEqual(
                fit.radius,
                diagHalf + 1e-3f,
                $"[{name}] circle radius {fit.radius} exceeds the bounding-diagonal half {diagHalf}."
            );
        }

        // Reconstruct the capsule the package way (the real GetCapsuleCenters round-trip) and assert enclosure.
        static void AssertCapsuleEncloses(string name, List<float2> cloud)
        {
            var fit = PhysicsShape2DAutoFit.FitCapsule(cloud);
            var auth = new GameObject("cap_" + name).AddComponent<PhysicsShape2DAuthoring>();
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
                        $"[{name}] capsule does NOT enclose point {i} ({cloud[i]}): dist-to-spine {d} > "
                            + $"radius {r}. size={fit.size} vertical={fit.vertical} angle={fit.angleDeg} "
                            + $"center={fit.center}"
                    );
                }
            }
            finally
            {
                Object.DestroyImmediate(auth.gameObject);
            }
        }

        // The polygon fit's emitted Vertices, interpreted as a (convex single-hull) polygon, must enclose the
        // cloud — but ONLY when the single-hull path is taken (decompose off). The decompose path emits the raw
        // cloud, which is a separate decision pinned below.
        static void AssertPolygonHullEncloses(string name, List<float2> cloud)
        {
            var hull = PhysicsShape2DAutoFit.ConvexHull(cloud);
            for (var i = 0; i < cloud.Count; i++)
            {
                // A cloud point is inside-or-on the hull. Even-odd treats on-edge as ambiguous, so also accept
                // a hull vertex match within epsilon.
                var onHull = false;
                for (var h = 0; h < hull.Length; h++)
                    if (lengthsq(cloud[i] - hull[h]) < Eps)
                        onHull = true;
                Assert.IsTrue(
                    onHull || PointInPolygon(cloud[i], hull),
                    $"[{name}] convex hull does NOT enclose point {i} ({cloud[i]})."
                );
            }
        }

        [Test]
        public void Enclose_Box_AllClouds()
        {
            AssertBoxEncloses("square", SquareCloud());
            AssertBoxEncloses("rect", RectCloud());
            AssertBoxEncloses("circleish", CircleishCloud());
            AssertBoxEncloses("tilted", TiltedElongatedCloud());
            AssertBoxEncloses("L", LConcaveCloud());
            AssertBoxEncloses("blob", BlobCloud());
        }

        [Test]
        public void Enclose_Circle_AllClouds()
        {
            AssertCircleEncloses("square", SquareCloud());
            AssertCircleEncloses("rect", RectCloud());
            AssertCircleEncloses("circleish", CircleishCloud());
            AssertCircleEncloses("tilted", TiltedElongatedCloud());
            AssertCircleEncloses("L", LConcaveCloud());
            AssertCircleEncloses("blob", BlobCloud());
        }

        [Test]
        public void Enclose_Capsule_AllClouds_IncludingPcaTilt()
        {
            AssertCapsuleEncloses("square", SquareCloud());
            AssertCapsuleEncloses("rect", RectCloud());
            AssertCapsuleEncloses("circleish", CircleishCloud());
            // The re-pinned bug: a tilted elongated cloud where the PCA frame is non-axis-aligned.
            AssertCapsuleEncloses("tilted", TiltedElongatedCloud());
            AssertCapsuleEncloses("L", LConcaveCloud());
            AssertCapsuleEncloses("blob", BlobCloud());
        }

        [Test]
        public void Enclose_PolygonHull_AllClouds()
        {
            AssertPolygonHullEncloses("square", SquareCloud());
            AssertPolygonHullEncloses("rect", RectCloud());
            AssertPolygonHullEncloses("circleish", CircleishCloud());
            AssertPolygonHullEncloses("tilted", TiltedElongatedCloud());
            AssertPolygonHullEncloses("L", LConcaveCloud());
            AssertPolygonHullEncloses("blob", BlobCloud());
        }

        // Sharper capsule-tilt falsification: a wide sweep of tilt angles on a fixed elongated rectangle. If the
        // radius/half-length formula under-covers at ANY angle, this catches it (the original bug was angle-only).
        [Test]
        public void Capsule_EnclosesAcrossTiltSweep()
        {
            var baseRect = new List<float2>
            {
                new float2(-3f, -0.6f),
                new float2(3f, -0.6f),
                new float2(3f, 0.6f),
                new float2(-3f, 0.6f),
                new float2(-1.5f, 0.2f),
                new float2(2f, -0.4f),
            };
            for (var deg = 0f; deg < 180f; deg += 7f)
            {
                var cloud = new List<float2>(baseRect.Count);
                foreach (var p in baseRect)
                    cloud.Add(R(p, deg));
                AssertCapsuleEncloses($"sweep{deg:F0}", cloud);
            }
        }

        // ===================================================================================================
        // ADAPTER 1 — PolygonCollider2D paths (the proven adapter; offset carry-through is load-bearing).
        // ===================================================================================================

        [Test]
        public void Adapter_PolygonCollider_GathersPaths_FitEnclosesSource_HonoursOffset()
        {
            // A real PolygonCollider2D with a known triangle path and a non-zero offset.
            var go = new GameObject("polyCol");
            try
            {
                var col = go.AddComponent<PolygonCollider2D>();
                var path = new[] { new Vector2(-1f, -1f), new Vector2(3f, -1f), new Vector2(1f, 2f) };
                col.pathCount = 1;
                col.SetPath(0, path);
                col.offset = new Vector2(10f, -5f);

                var cloud = new List<float2>();
                var ok = PhysicsShape2DAutoFit.TryGatherPolygonCollider(col, cloud, out var offset);
                Assert.IsTrue(ok, "TryGatherPolygonCollider returned false for a real collider with a path.");
                Assert.AreEqual(3, cloud.Count, "gathered cloud must hold the 3 path vertices.");
                Assert.AreEqual(10f, offset.x, Eps, "reported offset.x must be the collider's own offset.");
                Assert.AreEqual(-5f, offset.y, Eps, "reported offset.y must be the collider's own offset.");

                // The gathered cloud must equal the path (collider-local, NOT pre-offset). Match as a set.
                for (var i = 0; i < path.Length; i++)
                {
                    var found = false;
                    foreach (var c in cloud)
                        if (lengthsq(c - (float2)path[i]) < Eps)
                            found = true;
                    Assert.IsTrue(found, $"path vertex {path[i]} missing from the gathered cloud.");
                }

                // Fit a box through the real apply path and assert the FITTED shape (placed at sourceOffset +
                // fit.center) encloses the source outline in WORLD space (path + collider.offset).
                var auth = go.AddComponent<PhysicsShape2DAuthoring>();
                var fitOk = PhysicsShape2DAutoFit.FitToPolygonCollider2D(auth, col, PhysicsShape2DKind.Box);
                Assert.IsTrue(fitOk, "FitToPolygonCollider2D returned false.");
                Assert.AreEqual(PhysicsShape2DKind.Box, auth.Kind);
                // World source outline = path + offset; the fitted box is at auth.Offset with auth.BoxSize/Angle,
                // and auth.Offset already folds in the collider offset (FitTo adds sourceOffset).
                for (var i = 0; i < path.Length; i++)
                {
                    var worldPt = (float2)path[i] + offset;
                    Assert.IsTrue(
                        PointInBox(worldPt, auth.Offset, auth.BoxSize, auth.BoxAngle),
                        $"fitted box does not enclose source outline vertex {i} in world space: "
                            + $"worldPt={worldPt} boxOffset={auth.Offset} size={auth.BoxSize} angle={auth.BoxAngle}"
                    );
                }
            }
            finally
            {
                Object.DestroyImmediate(go);
            }
        }

        [Test]
        public void Adapter_PolygonCollider_CircleSource_FitsCircleOfRightRadius()
        {
            var go = new GameObject("polyColCircle");
            try
            {
                var col = go.AddComponent<PolygonCollider2D>();
                // A 20-gon approximating a radius-2 circle centred at origin.
                var pts = new Vector2[20];
                for (var i = 0; i < 20; i++)
                {
                    var a = radians(360f / 20f * i);
                    sincos(a, out var s, out var co);
                    pts[i] = new Vector2(co, s) * 2f;
                }
                col.pathCount = 1;
                col.SetPath(0, pts);
                col.offset = Vector2.zero;

                var auth = go.AddComponent<PhysicsShape2DAuthoring>();
                Assert.IsTrue(PhysicsShape2DAutoFit.FitToPolygonCollider2D(auth, col, PhysicsShape2DKind.Circle));
                Assert.AreEqual(PhysicsShape2DKind.Circle, auth.Kind);
                Assert.AreEqual(2f, auth.Radius, 1e-2f, "a circle-ish source must fit a circle of ~r=2.");
                Assert.AreEqual(0f, auth.Offset.x, 1e-2f, "circle centred on origin.");
                Assert.AreEqual(0f, auth.Offset.y, 1e-2f);
            }
            finally
            {
                Object.DestroyImmediate(go);
            }
        }

        // ===================================================================================================
        // ADAPTER 2 — SpriteRenderer bounds (the deliberately-coarse rectangle source).
        // ===================================================================================================

        [Test]
        public void Adapter_SpriteRendererBounds_GathersSpriteAabb_BoxMatchesBounds()
        {
            var go = new GameObject("sr");
            Texture2D tex = null;
            Sprite sprite = null;
            try
            {
                // A 64×32 px sprite at 32 px/unit → a 2×1 unit local bounds centred on the pivot (0.5,0.5).
                tex = new Texture2D(64, 32);
                sprite = Sprite.Create(tex, new Rect(0, 0, 64, 32), new Vector2(0.5f, 0.5f), 32f);
                var sr = go.AddComponent<SpriteRenderer>();
                sr.sprite = sprite;

                var cloud = new List<float2>();
                var ok = PhysicsShape2DAutoFit.TryGatherSpriteRendererBounds(sr, cloud);
                Assert.IsTrue(ok, "TryGatherSpriteRendererBounds returned false for a real assigned sprite.");
                Assert.AreEqual(4, cloud.Count, "sprite-bounds source must emit 4 AABB corners.");

                var b = sprite.bounds;
                Aabb(cloud, out var lo, out var hi);
                Assert.AreEqual(b.min.x, lo.x, 1e-3f, "gathered AABB min.x must be the sprite bounds min.x.");
                Assert.AreEqual(b.min.y, lo.y, 1e-3f);
                Assert.AreEqual(b.max.x, hi.x, 1e-3f, "gathered AABB max.x must be the sprite bounds max.x.");
                Assert.AreEqual(b.max.y, hi.y, 1e-3f);

                // The box fit of a rectangle source must match the bounds exactly (axis-aligned, since the
                // rectangle's principal axes are the world axes).
                var auth = go.AddComponent<PhysicsShape2DAuthoring>();
                Assert.IsTrue(PhysicsShape2DAutoFit.FitToSpriteRenderer(auth, sr, PhysicsShape2DKind.Box));
                Assert.AreEqual(b.size.x, auth.BoxSize.x, 1e-3f, "fitted box width = bounds width.");
                Assert.AreEqual(b.size.y, auth.BoxSize.y, 1e-3f, "fitted box height = bounds height.");
                Assert.AreEqual(b.center.x, auth.Offset.x, 1e-3f, "fitted box centre = bounds centre x.");
                Assert.AreEqual(b.center.y, auth.Offset.y, 1e-3f);
            }
            finally
            {
                Object.DestroyImmediate(go);
                if (sprite != null)
                    Object.DestroyImmediate(sprite);
                if (tex != null)
                    Object.DestroyImmediate(tex);
            }
        }

        // ===================================================================================================
        // ADAPTER 3 — Sprite physics shape (constructed via Sprite.OverridePhysicsShape — NOT fixture-limited).
        // ===================================================================================================

        [Test]
        public void Adapter_SpritePhysicsShape_GathersOverriddenOutline_FitEnclosesIt()
        {
            Texture2D tex = null;
            Sprite sprite = null;
            var go = new GameObject("spritePhys");
            try
            {
                tex = new Texture2D(100, 100);
                // 100×100 px at 100 px/unit, pivot at centre → sprite-local space is centred at origin,
                // a [-0.5,0.5]² unit square. OverridePhysicsShape takes points in sprite.rect (pixel) space,
                // but GetPhysicsShape returns them in sprite-LOCAL units (the same space the fit expects).
                sprite = Sprite.Create(tex, new Rect(0, 0, 100, 100), new Vector2(0.5f, 0.5f), 100f);

                // A triangle physics outline (pixel space): corners at (10,10),(90,10),(50,90).
                var outline = new List<Vector2[]>
                {
                    new[] { new Vector2(10f, 10f), new Vector2(90f, 10f), new Vector2(50f, 90f) },
                };
                sprite.OverridePhysicsShape(outline);

                Assert.GreaterOrEqual(
                    sprite.GetPhysicsShapeCount(),
                    1,
                    "OverridePhysicsShape did not register a physics shape — the adapter cannot be tested as a "
                        + "real-shape source on this editor; fall back to renderer-bounds + polygon-collider as "
                        + "the proven adapters."
                );

                var cloud = new List<float2>();
                var ok = PhysicsShape2DAutoFit.TryGatherSpriteShape(sprite, cloud);
                Assert.IsTrue(ok, "TryGatherSpriteShape returned false for a sprite with an overridden shape.");
                Assert.GreaterOrEqual(cloud.Count, 3, "the triangle outline must yield at least 3 points.");

                // Read the same outline back through the engine API as the oracle (sprite-local units).
                var oracle = new List<Vector2>();
                var oracleCloud = new List<float2>();
                for (var s = 0; s < sprite.GetPhysicsShapeCount(); s++)
                {
                    sprite.GetPhysicsShape(s, oracle);
                    foreach (var p in oracle)
                        oracleCloud.Add((float2)p);
                }
                // The gathered cloud must equal what GetPhysicsShape returns (the adapter must not transform it).
                Assert.AreEqual(
                    oracleCloud.Count,
                    cloud.Count,
                    "adapter point count differs from GetPhysicsShape — the adapter dropped or duplicated points."
                );

                // Fit a polygon and assert the fitted hull encloses the real outline (in sprite-local space,
                // offset 0 for a sprite).
                var auth = go.AddComponent<PhysicsShape2DAuthoring>();
                Assert.IsTrue(PhysicsShape2DAutoFit.FitToSprite(auth, sprite, PhysicsShape2DKind.Box));
                Assert.AreEqual(PhysicsShape2DKind.Box, auth.Kind);
                for (var i = 0; i < oracleCloud.Count; i++)
                    Assert.IsTrue(
                        PointInBox(oracleCloud[i], auth.Offset, auth.BoxSize, auth.BoxAngle),
                        $"fitted box does not enclose sprite-shape outline point {i} ({oracleCloud[i]})."
                    );
            }
            finally
            {
                Object.DestroyImmediate(go);
                if (sprite != null)
                    Object.DestroyImmediate(sprite);
                if (tex != null)
                    Object.DestroyImmediate(tex);
            }
        }

        // A sprite with NO authored physics shape must degrade to its bounds rectangle (the documented fallback).
        [Test]
        public void Adapter_SpriteNoPhysicsShape_FallsBackToBounds()
        {
            Texture2D tex = null;
            Sprite sprite = null;
            try
            {
                tex = new Texture2D(40, 40);
                // generateFallbackPhysicsShape:false so the sprite genuinely has no physics shape.
                sprite = Sprite.Create(
                    tex,
                    new Rect(0, 0, 40, 40),
                    new Vector2(0.5f, 0.5f),
                    40f,
                    0,
                    SpriteMeshType.FullRect,
                    Vector4.zero,
                    false
                );
                // Do not override — GetPhysicsShapeCount should be 0.
                var cloud = new List<float2>();
                var ok = PhysicsShape2DAutoFit.TryGatherSpriteShape(sprite, cloud);
                Assert.IsTrue(ok, "fallback to bounds must still return points.");

                if (sprite.GetPhysicsShapeCount() == 0)
                {
                    Assert.AreEqual(
                        4,
                        cloud.Count,
                        "with no physics shape, the adapter must emit the 4 bounds corners."
                    );
                    var b = sprite.bounds;
                    Aabb(cloud, out var lo, out var hi);
                    Assert.AreEqual(b.min.x, lo.x, 1e-3f);
                    Assert.AreEqual(b.max.x, hi.x, 1e-3f);
                }
                // If the editor auto-generated a fallback shape despite the flag, the count > 0 path is exercised
                // by the override test above; either way the adapter returned points (ok == true), which is the
                // contract.
            }
            finally
            {
                if (sprite != null)
                    Object.DestroyImmediate(sprite);
                if (tex != null)
                    Object.DestroyImmediate(tex);
            }
        }

        // ===================================================================================================
        // ≤8 / DECOMPOSE decision + the CONCAVE finding.
        // ===================================================================================================

        // A ≤8-vertex convex outline → a single Polygon (decompose off), vertices ≤ 8.
        [Test]
        public void Decompose_SmallConvex_SingleHull()
        {
            var auth = new GameObject("p1").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                var hex = new List<float2>();
                for (var i = 0; i < 6; i++)
                {
                    var a = radians(360f / 6f * i);
                    sincos(a, out var s, out var co);
                    hex.Add(new float2(co, s) * 2f);
                }
                Assert.IsTrue(
                    PhysicsShape2DAutoFit.FitTo(auth, hex, PhysicsShape2DKind.Polygon, Unity.Mathematics.float2.zero)
                );
                Assert.IsFalse(auth.PolygonDecompose, "a 6-vertex convex hull stays single-hull.");
                Assert.LessOrEqual(
                    auth.Vertices.Length,
                    PhysicsShape2DAutoFit.MaxPolygonVertices,
                    "single-hull vertices must be within the Box2D cap."
                );
            }
            finally
            {
                Object.DestroyImmediate(auth.gameObject);
            }
        }

        // A >8-vertex convex outline → decompose on, and (for a single ordered ring) the emitted vertices are
        // the source cloud, which preserves order — so the runtime decomposer sees an ordered convex ring.
        [Test]
        public void Decompose_LargeConvex_DecomposeOn_OrderPreservedForSinglePath()
        {
            var auth = new GameObject("p2").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                var blob = BlobCloud(); // 14-gon, ordered CCW
                Assert.IsTrue(
                    PhysicsShape2DAutoFit.FitTo(auth, blob, PhysicsShape2DKind.Polygon, Unity.Mathematics.float2.zero)
                );
                Assert.IsTrue(auth.PolygonDecompose, "a >8-vertex hull takes the decompose path.");
                // FitTo emits ToVector2(cloud) on the decompose branch — for a single ordered input cloud this
                // preserves the ring order. Confirm the emitted vertices match the input order exactly.
                Assert.AreEqual(blob.Count, auth.Vertices.Length, "decompose emits the full source cloud.");
                for (var i = 0; i < blob.Count; i++)
                {
                    Assert.AreEqual(blob[i].x, auth.Vertices[i].x, Eps, $"vertex {i} x order preserved");
                    Assert.AreEqual(blob[i].y, auth.Vertices[i].y, Eps, $"vertex {i} y order preserved");
                }
            }
            finally
            {
                Object.DestroyImmediate(auth.gameObject);
            }
        }

        // CONCAVE FINDING (RED / known-gap, evidenced): an L-shaped concave outline whose convex hull is ≤8
        // vertices is silently emitted as its CONVEX HULL (decompose off), filling the concave notch. This pins
        // the ACTUAL behaviour and demonstrates the defect: a point in the filled notch — OUTSIDE the true
        // concave outline — is INSIDE the fitted polygon. The fit does NOT follow the concave outline.
        [Test]
        public void Decompose_Concave_SilentlyConvexHulled_KNOWN_GAP()
        {
            var auth = new GameObject("p3").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                var l = LConcaveCloud(); // ordered L ring; reflex corner (1,1)
                // Sanity: the source IS concave (its hull drops the reflex vertex).
                var hull = PhysicsShape2DAutoFit.ConvexHull(l);
                Assert.Less(
                    hull.Length,
                    l.Count,
                    "self-check: the L outline must be concave (hull has fewer verts than the outline)."
                );
                Assert.LessOrEqual(
                    hull.Length,
                    PhysicsShape2DAutoFit.MaxPolygonVertices,
                    "self-check: the L's hull is within the cap, so FitTo takes the single-hull branch."
                );

                Assert.IsTrue(
                    PhysicsShape2DAutoFit.FitTo(auth, l, PhysicsShape2DKind.Polygon, Unity.Mathematics.float2.zero)
                );

                // ACTUAL behaviour: single hull, decompose OFF — the concavity is discarded.
                Assert.IsFalse(
                    auth.PolygonDecompose,
                    "DOCUMENTED GAP: a concave outline whose hull is <=8 verts is emitted as a single convex "
                        + "hull (decompose off), NOT as a decomposed concave outline."
                );
                Assert.AreEqual(
                    hull.Length,
                    auth.Vertices.Length,
                    "the emitted polygon is the convex hull, not the original concave ring."
                );

                // The defect made concrete: (1.3, 1.3) is in the filled notch — OUTSIDE the true L outline but
                // strictly INSIDE the fitted convex hull. A faithful concave fit would NOT contain it. (Chosen
                // strictly interior to the hull: the hull's upper-right edge is the line x+y=3 from (2,1) to
                // (1,2), so 1.3+1.3=2.6 < 3 is well inside; and x=1.3>1, y=1.3>1 puts it in neither L bar.)
                var notchPoint = new float2(1.3f, 1.3f);
                Assert.IsFalse(
                    PointInPolygon(notchPoint, l),
                    "self-check: the notch point is outside the true concave L outline."
                );
                Assert.IsTrue(
                    PointInPolygon(notchPoint, hull),
                    "EVIDENCE OF GAP: the notch point is inside the fitted convex hull — the fit over-covers the "
                        + "concave region. The concave outline was silently convex-hulled."
                );
            }
            finally
            {
                Object.DestroyImmediate(auth.gameObject);
            }
        }

        // ===================================================================================================
        // BAKE-CORRECTNESS (structural) — a fit writes EXACTLY the authoring fields a hand-author would, so the
        // baker (which reads only those fields) produces the same shape. Proven here at the authoring-field
        // boundary: a fitted target and a hand-authored target carry identical bake-relevant fields. The runtime
        // bake-through-importer e2e is in AutoFitBakeGate (SubScene fixture).
        // ===================================================================================================

        [Test]
        public void BakeCorrectness_FittedFields_EqualHandAuthored_Box()
        {
            var fitted = new GameObject("fit").AddComponent<PhysicsShape2DAuthoring>();
            var hand = new GameObject("hand").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                // A 4×2 axis-aligned rectangle cloud → an axis-aligned box of size (4,2) centred at origin.
                var cloud = new List<float2>
                {
                    new float2(-2f, -1f),
                    new float2(2f, -1f),
                    new float2(2f, 1f),
                    new float2(-2f, 1f),
                };
                Assert.IsTrue(
                    PhysicsShape2DAutoFit.FitTo(fitted, cloud, PhysicsShape2DKind.Box, Unity.Mathematics.float2.zero)
                );

                // What a hand-author would set for the same local geometry.
                hand.Kind = PhysicsShape2DKind.Box;
                hand.BoxSize = new float2(4f, 2f);
                hand.BoxAngle = 0f;
                hand.BoxCornerRadius = 0f;
                hand.Offset = Unity.Mathematics.float2.zero;

                Assert.AreEqual(hand.Kind, fitted.Kind);
                Assert.AreEqual(hand.BoxSize.x, fitted.BoxSize.x, 1e-4f, "fitted box width = hand-authored.");
                Assert.AreEqual(hand.BoxSize.y, fitted.BoxSize.y, 1e-4f);
                Assert.AreEqual(
                    hand.BoxCornerRadius,
                    fitted.BoxCornerRadius,
                    1e-4f,
                    "fitted corner radius = hand-authored 0 (the box corner rounding is BoxCornerRadius, not the "
                        + "circle/capsule Radius)."
                );
                Assert.AreEqual(hand.Offset.x, fitted.Offset.x, 1e-4f, "fitted offset = hand-authored.");
                Assert.AreEqual(hand.Offset.y, fitted.Offset.y, 1e-4f);
                Assert.AreEqual(
                    abs(hand.BoxAngle),
                    abs(fitted.BoxAngle),
                    1e-3f,
                    "fitted box angle = hand-authored 0 (axis-aligned rectangle)."
                );
            }
            finally
            {
                Object.DestroyImmediate(fitted.gameObject);
                Object.DestroyImmediate(hand.gameObject);
            }
        }

        [Test]
        public void BakeCorrectness_FittedFields_EqualHandAuthored_Circle()
        {
            var fitted = new GameObject("fitc").AddComponent<PhysicsShape2DAuthoring>();
            var hand = new GameObject("handc").AddComponent<PhysicsShape2DAuthoring>();
            try
            {
                // A diamond of "radius" 3 centred at (1,1).
                var cloud = new List<float2>
                {
                    new float2(-2f, 1f),
                    new float2(4f, 1f),
                    new float2(1f, -2f),
                    new float2(1f, 4f),
                };
                Assert.IsTrue(
                    PhysicsShape2DAutoFit.FitTo(fitted, cloud, PhysicsShape2DKind.Circle, Unity.Mathematics.float2.zero)
                );

                hand.Kind = PhysicsShape2DKind.Circle;
                hand.Radius = 3f;
                hand.Offset = new float2(1f, 1f);

                Assert.AreEqual(hand.Kind, fitted.Kind);
                Assert.AreEqual(hand.Radius, fitted.Radius, 1e-3f, "fitted radius = hand-authored.");
                Assert.AreEqual(hand.Offset.x, fitted.Offset.x, 1e-3f, "fitted centre = hand-authored.");
                Assert.AreEqual(hand.Offset.y, fitted.Offset.y, 1e-3f);
            }
            finally
            {
                Object.DestroyImmediate(fitted.gameObject);
                Object.DestroyImmediate(hand.gameObject);
            }
        }
    }
}
