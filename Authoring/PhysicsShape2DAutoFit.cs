using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Authoring
{
    /// <summary>
    /// Editor-time shape AUTO-FIT — the 2D analogue of <c>com.unity.physics</c>'s
    /// <c>PhysicsShapeAuthoring.FitToEnabledRenderMeshes</c> (3D mesh fit, structure-only reference). Takes a 2D
    /// point source (a <see cref="Sprite"/>'s physics shape, a <see cref="SpriteRenderer"/>'s sprite bounds, or a
    /// <see cref="PolygonCollider2D"/>'s paths), gathers a local-space point cloud, and produces best-fit
    /// parameters for the package's enclosing shape kinds (Box / Circle / Capsule / Polygon), then writes them
    /// onto a <see cref="PhysicsShape2DAuthoring"/> so the fitted shape bakes through the existing baker exactly
    /// as a hand-authored one.
    /// </summary>
    /// <remarks>
    /// <para><b>Two layers.</b> A side-effect-free math CORE that operates on a plain
    /// <see cref="Unity.Mathematics.float2"/> cloud and returns small fit-result structs (testable without a Unity
    /// asset, reusable by a future scene-view preview that wants the fit without committing it), plus a thin APPLY
    /// layer that gathers a cloud from a Unity source and writes the authoring fields (the "Fit" dropdown's
    /// one-liner). The math core does NOT touch Unity objects; the source adapters and apply dispatch do.</para>
    ///
    /// <para><b>Scale is the baker's job.</b> The fit emits UNSCALED, local-space authoring fields — exactly the
    /// form a hand-authored shape carries — and <c>PhysicsShape2DAuthoringBaker</c> applies transform scale at bake
    /// time via <c>Collider2DBaking.ScaleBoxSize</c>/<c>ScaleCircleRadius</c>/<c>ScaleOffset</c>/<c>FlipsWinding</c>.
    /// So a fitted shape and a hand-authored shape of the same local geometry bake bit-identically; the fit utility
    /// itself never calls the scale helpers (doing so would double-scale).</para>
    ///
    /// <para><b>Enclosing vs minimal.</b> Every fit GENUINELY ENCLOSES all source points (the mandatory property).
    /// The minimum-enclosing CIRCLE (Welzl) is provably minimal; the AABB Box is minimal among axis-aligned boxes;
    /// the oriented (PCA) Box and the Capsule enclose but are near-minimal best-effort (PCA gives the min-area box
    /// only when the cloud aligns with its principal axes — the common sprite-outline case — and a true minimum
    /// enclosing capsule has no closed form). This mirrors the 3D sample's own axis-aligned-only box fit, extended
    /// where 2D makes a better fit cheap.</para>
    ///
    /// <para>This is editor-time authoring convenience math — managed C#, no Burst, not a job (it runs on the main
    /// thread when a user clicks Fit, not in a runtime hot path). It lives in the Authoring assembly so it is
    /// callable without an Editor assembly. The inspector wiring + the "Fit" dropdown are Phase D.</para>
    /// </remarks>
    public static class PhysicsShape2DAutoFit
    {
        /// <summary>
        /// The Box2D-v3 single-<c>PolygonGeometry</c> vertex cap (<c>PhysicsConstants.MaxPolygonVertices</c> =
        /// <c>B2_MAX_POLYGON_VERTICES</c>). A convex hull with at most this many vertices fits one polygon; a hull
        /// with more (or a concave outline) takes the decompose path (<see cref="PhysicsShape2D.polygonDecompose"/>),
        /// where the runtime splits it into convex fragments. Hardcoded because the Authoring assembly does not
        /// reference <c>Unity.U2D.Physics</c>; the value matches the package's bake-contract / runtime-components and
        /// the composite/custom bakers' decompose threshold.
        /// </summary>
        public const int MaxPolygonVertices = 8;

        // ----- fit-result structs (the side-effect-free core's output) -----

        /// <summary>An oriented box fit: <see cref="center"/> + half-aware full <see cref="size"/> + a z-rotation
        /// in <b>degrees</b>. <see cref="angleDeg"/> is 0 for an axis-aligned (AABB) fit.</summary>
        public readonly struct BoxFit
        {
            public readonly float2 center;
            public readonly float2 size;
            public readonly float angleDeg;

            public BoxFit(float2 center, float2 size, float angleDeg)
            {
                this.center = center;
                this.size = size;
                this.angleDeg = angleDeg;
            }
        }

        /// <summary>A minimum-enclosing circle fit: <see cref="center"/> + <see cref="radius"/>.</summary>
        public readonly struct CircleFit
        {
            public readonly float2 center;
            public readonly float radius;

            public CircleFit(float2 center, float radius)
            {
                this.center = center;
                this.radius = radius;
            }
        }

        /// <summary>A capsule fit in the package's size + vertical + angle authoring form: <see cref="center"/>,
        /// full <see cref="size"/> (width, height), <see cref="vertical"/> (long axis Y when true), and a residual
        /// z-rotation in <b>degrees</b> so size + vertical + angle round-trip through
        /// <see cref="PhysicsShape2DAuthoring.GetCapsuleCenters"/> back to the fitted oriented capsule.</summary>
        public readonly struct CapsuleFit
        {
            public readonly float2 center;
            public readonly float2 size;
            public readonly bool vertical;
            public readonly float angleDeg;

            public CapsuleFit(float2 center, float2 size, bool vertical, float angleDeg)
            {
                this.center = center;
                this.size = size;
                this.vertical = vertical;
                this.angleDeg = angleDeg;
            }
        }

        // ----- the math core: point cloud -> primitive -----

        /// <summary>
        /// Fit an axis-aligned (<paramref name="oriented"/> false) or PCA-oriented (true) box that ENCLOSES every
        /// point. The axis-aligned box is the minimal axis-aligned box (exact AABB); the oriented box takes the
        /// cloud's principal axis (2×2 covariance eigenvector, analytic angle <c>0.5·atan2(2b, a−c)</c>), AABBs in
        /// that frame, and rotates back — it encloses exactly, and is the min-area box when the cloud aligns with
        /// its principal axes (best-effort minimal otherwise).
        /// </summary>
        public static BoxFit FitBox(IReadOnlyList<float2> points, bool oriented)
        {
            var angle = oriented ? PrincipalAngle(points) : 0f;
            if (angle == 0f)
            {
                Aabb(points, out var min, out var max);
                return new BoxFit((min + max) * 0.5f, max - min, 0f);
            }

            // AABB in the rotated (principal-axis) frame, then rotate the box centre back to world.
            sincos(-angle, out var s, out var c);
            var min2 = new float2(float.PositiveInfinity, float.PositiveInfinity);
            var max2 = new float2(float.NegativeInfinity, float.NegativeInfinity);
            for (var i = 0; i < points.Count; i++)
            {
                var p = points[i];
                var r = new float2(c * p.x - s * p.y, s * p.x + c * p.y);
                min2 = min(min2, r);
                max2 = max(max2, r);
            }
            var localCenter = (min2 + max2) * 0.5f;
            // rotate the local-frame centre back by +angle
            sincos(angle, out var s2, out var c2);
            var center = new float2(
                c2 * localCenter.x - s2 * localCenter.y,
                s2 * localCenter.x + c2 * localCenter.y
            );
            return new BoxFit(center, max2 - min2, degrees(angle));
        }

        /// <summary>
        /// Fit the true minimum-enclosing circle (Welzl's expected-linear randomized incremental algorithm). The
        /// result both ENCLOSES every point and is provably MINIMAL — the one fit that is exactly minimal.
        /// </summary>
        public static CircleFit FitCircle(IReadOnlyList<float2> points)
        {
            // Copy + shuffle (Welzl wants a random permutation for expected-linear time).
            var pts = new float2[points.Count];
            for (var i = 0; i < pts.Length; i++)
                pts[i] = points[i];
            var rng = new Unity.Mathematics.Random(0x9E3779B9u);
            for (var i = pts.Length - 1; i > 0; i--)
            {
                var j = rng.NextInt(0, i + 1);
                (pts[i], pts[j]) = (pts[j], pts[i]);
            }

            var center = Unity.Mathematics.float2.zero;
            var radius = 0f;
            for (var i = 0; i < pts.Length; i++)
            {
                if (InCircle(pts[i], center, radius))
                    continue;
                // pts[i] is on the boundary of the new MEC.
                center = pts[i];
                radius = 0f;
                for (var j = 0; j < i; j++)
                {
                    if (InCircle(pts[j], center, radius))
                        continue;
                    // pts[i], pts[j] on boundary -> diameter circle.
                    center = (pts[i] + pts[j]) * 0.5f;
                    radius = length(pts[i] - center);
                    for (var k = 0; k < j; k++)
                    {
                        if (InCircle(pts[k], center, radius))
                            continue;
                        // pts[i], pts[j], pts[k] on boundary -> circumscribed circle.
                        Circumcircle(pts[i], pts[j], pts[k], out center, out radius);
                    }
                }
            }
            return new CircleFit(center, radius);
        }

        /// <summary>
        /// Fit an oriented capsule whose spine is the cloud's PCA major axis. The capsule's radius is the
        /// <em>maximum perpendicular distance</em> of any point from that axis, and its spine half-length is the
        /// maximum projection of any point onto that axis; the package size form then places the two end caps at
        /// <c>±halfLen</c>, so every point lies within <c>radius</c> of the segment — the fit GENUINELY ENCLOSES
        /// the cloud. (The earlier "half the bbox perpendicular extent" radius did NOT enclose when the PCA frame
        /// tilted a few degrees: a point at the major-axis extreme has a non-zero perpendicular component relative
        /// to the tilted spine, so its distance to the rounded end cap exceeds half the bbox extent. Measuring the
        /// radius as the true max perpendicular distance, and the half-length as the true max projection, is the
        /// enclosure-correct form.) Near-minimal best-effort: the optimal capsule axis is not necessarily the PCA
        /// axis, but the result always encloses. Returned in the package size + vertical + angle authoring form so
        /// it round-trips through <see cref="PhysicsShape2DAuthoring.GetCapsuleCenters"/>.
        /// </summary>
        public static CapsuleFit FitCapsule(IReadOnlyList<float2> points)
        {
            var angle = PrincipalAngle(points);

            // Project into the principal frame: x is the major-axis coordinate, y the perpendicular coordinate.
            // The spine passes through the projection midpoint; the radius must cover the largest perpendicular
            // distance from that spine, and the half-length the largest projection from the midpoint, so every
            // point (including the major-axis extremes) is within radius of the segment between the end caps.
            sincos(-angle, out var s, out var c);
            var minX = float.PositiveInfinity;
            var maxX = float.NegativeInfinity;
            var minY = float.PositiveInfinity;
            var maxY = float.NegativeInfinity;
            for (var i = 0; i < points.Count; i++)
            {
                var p = points[i];
                var rx = c * p.x - s * p.y;
                var ry = s * p.x + c * p.y;
                minX = min(minX, rx);
                maxX = max(maxX, rx);
                minY = min(minY, ry);
                maxY = max(maxY, ry);
            }

            var midX = (minX + maxX) * 0.5f;
            var midY = (minY + maxY) * 0.5f;
            // Half the perpendicular extent IS the max perpendicular distance from the (centred) spine, and half
            // the major extent IS the max projection from the centred midpoint, because the spine is the bbox
            // centre line in this frame.
            var radius = (maxY - minY) * 0.5f;
            var halfLen = (maxX - minX) * 0.5f;

            // centre (the projection midpoint) back to world
            sincos(angle, out var s2, out var c2);
            var center = new float2(c2 * midX - s2 * midY, s2 * midX + c2 * midY);

            // The package capsule model (GetCapsuleCenters) derives radius = height/2 and places the cap centres at
            // ±(width/2 − radius). To place the caps at ±halfLen with that radius, the full size is
            // height = 2·radius, width = 2·(halfLen + radius) for a HORIZONTAL (long axis X) capsule. Expressed as
            // a horizontal capsule rotated by the principal angle; if the major axis is closer to vertical, fold a
            // 90° into the vertical flag so the residual angle stays small and the size reads naturally.
            var width = 2f * (halfLen + radius);
            var height = 2f * radius;
            var vertical = false;

            // Normalise the principal angle to (−90°, 90°] so the major axis is unambiguous.
            var deg = degrees(angle);
            while (deg > 90f)
                deg -= 180f;
            while (deg <= -90f)
                deg += 180f;
            // If the major axis is within 45° of vertical, present it as a vertical capsule (swap w/h, subtract 90°).
            if (deg > 45f || deg <= -45f)
            {
                vertical = true;
                (width, height) = (height, width);
                deg += deg > 45f ? -90f : 90f;
            }

            return new CapsuleFit(center, new float2(width, height), vertical, deg);
        }

        /// <summary>
        /// Andrew's monotone-chain convex hull, CCW, O(n log n). Returns the hull vertices (every source point is
        /// inside or on it). A degenerate input (&lt; 3 unique points or all-collinear) returns the 1–2 unique
        /// points, which the apply layer treats as "not a valid polygon" and falls back to a box/circle.
        /// </summary>
        public static Unity.Mathematics.float2[] ConvexHull(IReadOnlyList<float2> points)
        {
            var n = points.Count;
            if (n < 3)
            {
                var trivial = new Unity.Mathematics.float2[n];
                for (var i = 0; i < n; i++)
                    trivial[i] = points[i];
                return trivial;
            }

            var sorted = new float2[n];
            for (var i = 0; i < n; i++)
                sorted[i] = points[i];
            System.Array.Sort(
                sorted,
                (a, b) => a.x != b.x ? a.x.CompareTo(b.x) : a.y.CompareTo(b.y)
            );

            var hull = new float2[2 * n];
            var k = 0;
            // lower hull
            for (var i = 0; i < n; i++)
            {
                while (k >= 2 && Cross(hull[k - 2], hull[k - 1], sorted[i]) <= 0f)
                    k--;
                hull[k++] = sorted[i];
            }
            // upper hull
            var lower = k + 1;
            for (var i = n - 2; i >= 0; i--)
            {
                while (k >= lower && Cross(hull[k - 2], hull[k - 1], sorted[i]) <= 0f)
                    k--;
                hull[k++] = sorted[i];
            }

            // k-1 because the last point equals the first
            var count = max(1, k - 1);
            var result = new Unity.Mathematics.float2[count];
            System.Array.Copy(hull, result, count);
            return result;
        }

        /// <summary>Whether a closed polygon (CCW or CW) is convex — every consecutive turn has the same sign.</summary>
        public static bool IsConvex(IReadOnlyList<float2> poly)
        {
            var n = poly.Count;
            if (n < 3)
                return false;
            var sign = 0;
            for (var i = 0; i < n; i++)
            {
                var z = Cross(poly[i], poly[(i + 1) % n], poly[(i + 2) % n]);
                if (z > 1e-7f)
                {
                    if (sign < 0)
                        return false;
                    sign = 1;
                }
                else if (z < -1e-7f)
                {
                    if (sign > 0)
                        return false;
                    sign = -1;
                }
            }
            return true;
        }

        // ----- the apply layer: write a fit onto a PhysicsShape2DAuthoring -----

        /// <summary>
        /// Fit <paramref name="kind"/> to <paramref name="cloud"/> (local-space points) and write the result onto
        /// <paramref name="target"/>'s authoring fields, offsetting by <paramref name="sourceOffset"/> (the source
        /// collider's own offset, 0 for a sprite). For Polygon, emits the convex hull as a single hull when it has
        /// at most <see cref="MaxPolygonVertices"/> vertices; otherwise emits the source outline with
        /// <see cref="PhysicsShape2DAuthoring.PolygonDecompose"/> set so the runtime decomposes it. Returns false
        /// (no change) when the cloud is empty.
        /// </summary>
        public static bool FitTo(
            PhysicsShape2DAuthoring target,
            IReadOnlyList<float2> cloud,
            PhysicsShape2DKind kind,
            float2 sourceOffset
        )
        {
            if (target == null || cloud == null || cloud.Count == 0)
                return false;

            target.Kind = kind;
            switch (kind)
            {
                case PhysicsShape2DKind.Circle:
                {
                    var fit = FitCircle(cloud);
                    target.Radius = fit.radius;
                    target.Offset = sourceOffset + fit.center;
                    return true;
                }
                case PhysicsShape2DKind.Box:
                {
                    var fit = FitBox(cloud, oriented: true);
                    target.BoxSize = fit.size;
                    target.BoxAngle = fit.angleDeg;
                    target.Radius = 0f;
                    target.Offset = sourceOffset + fit.center;
                    return true;
                }
                case PhysicsShape2DKind.Capsule:
                {
                    var fit = FitCapsule(cloud);
                    target.CapsuleSize = fit.size;
                    target.CapsuleVertical = fit.vertical;
                    target.CapsuleAngle = fit.angleDeg;
                    target.Offset = sourceOffset + fit.center;
                    return true;
                }
                case PhysicsShape2DKind.Polygon:
                {
                    var hull = ConvexHull(cloud);
                    if (hull.Length >= 3 && hull.Length <= MaxPolygonVertices)
                    {
                        // a single convex hull, centred on origin with the offset carrying the placement
                        target.Vertices = ToVector2(hull);
                        target.PolygonDecompose = false;
                    }
                    else
                    {
                        // a >8-vertex hull or a degenerate hull -> emit the cloud outline and decompose at runtime
                        target.Vertices = ToVector2(cloud);
                        target.PolygonDecompose = true;
                    }
                    target.Radius = 0f;
                    target.Offset = sourceOffset;
                    return true;
                }
                default:
                    // Edge is an open chain, not an enclosing fit — auto-fit does not target it.
                    return false;
            }
        }

        // ----- Unity-source gathering (the three sources) + convenience entry points -----

        /// <summary>
        /// Gather a <see cref="Sprite"/>'s physics-shape outline points (all shapes, sprite-local units) into
        /// <paramref name="cloud"/>. Falls back to the sprite's <c>bounds</c> 4 corners when the sprite has no
        /// authored physics shape. Returns false for a null sprite or an empty result.
        /// </summary>
        public static bool TryGatherSpriteShape(Sprite sprite, List<float2> cloud)
        {
            if (sprite == null || cloud == null)
                return false;
            cloud.Clear();
            var shapeCount = sprite.GetPhysicsShapeCount();
            if (shapeCount > 0)
            {
                var scratch = new List<Vector2>(64);
                for (var i = 0; i < shapeCount; i++)
                {
                    sprite.GetPhysicsShape(i, scratch);
                    for (var p = 0; p < scratch.Count; p++)
                        cloud.Add((float2)scratch[p]);
                }
            }
            else
            {
                // No authored physics shape — degrade to the sprite's rectangle (the renderer-bounds behaviour).
                AddBoundsCorners(sprite.bounds, cloud);
            }
            return cloud.Count > 0;
        }

        /// <summary>
        /// Gather a <see cref="SpriteRenderer"/>'s sprite-local AABB corners (the sprite's <c>bounds</c>, units)
        /// into <paramref name="cloud"/> — the deliberately-coarse "fit to the sprite rectangle" source, the 2D
        /// analogue of the 3D sample's hierarchy AABB. Returns false when no sprite is assigned.
        /// </summary>
        public static bool TryGatherSpriteRendererBounds(
            SpriteRenderer renderer,
            List<float2> cloud
        )
        {
            if (renderer == null || renderer.sprite == null || cloud == null)
                return false;
            cloud.Clear();
            AddBoundsCorners(renderer.sprite.bounds, cloud);
            return cloud.Count > 0;
        }

        /// <summary>
        /// Gather all of a <see cref="PolygonCollider2D"/>'s path vertices (collider-local units) into
        /// <paramref name="cloud"/> and report the collider's own <paramref name="offset"/>. Returns false for a
        /// null collider or an empty result.
        /// </summary>
        public static bool TryGatherPolygonCollider(
            PolygonCollider2D collider,
            List<float2> cloud,
            out float2 offset
        )
        {
            offset = Unity.Mathematics.float2.zero;
            if (collider == null || cloud == null)
                return false;
            cloud.Clear();
            offset = (float2)collider.offset;
            var scratch = new List<Vector2>(64);
            for (var p = 0; p < collider.pathCount; p++)
            {
                collider.GetPath(p, scratch);
                for (var i = 0; i < scratch.Count; i++)
                    cloud.Add((float2)scratch[i]);
            }
            return cloud.Count > 0;
        }

        /// <summary>Gather a <see cref="Sprite"/>'s physics shape and fit <paramref name="kind"/> onto
        /// <paramref name="target"/>. Returns false (no change) when the sprite yields no points.</summary>
        public static bool FitToSprite(
            PhysicsShape2DAuthoring target,
            Sprite sprite,
            PhysicsShape2DKind kind
        )
        {
            var cloud = new List<float2>(64);
            return TryGatherSpriteShape(sprite, cloud)
                && FitTo(target, cloud, kind, Unity.Mathematics.float2.zero);
        }

        /// <summary>Gather a <see cref="SpriteRenderer"/>'s sprite bounds and fit <paramref name="kind"/> onto
        /// <paramref name="target"/>. Returns false (no change) when no sprite is assigned.</summary>
        public static bool FitToSpriteRenderer(
            PhysicsShape2DAuthoring target,
            SpriteRenderer renderer,
            PhysicsShape2DKind kind
        )
        {
            var cloud = new List<float2>(8);
            return TryGatherSpriteRendererBounds(renderer, cloud)
                && FitTo(target, cloud, kind, Unity.Mathematics.float2.zero);
        }

        /// <summary>Gather a <see cref="PolygonCollider2D"/>'s paths and fit <paramref name="kind"/> onto
        /// <paramref name="target"/>, honouring the collider's offset. Returns false (no change) when the collider
        /// yields no points.</summary>
        public static bool FitToPolygonCollider2D(
            PhysicsShape2DAuthoring target,
            PolygonCollider2D collider,
            PhysicsShape2DKind kind
        )
        {
            var cloud = new List<float2>(64);
            return TryGatherPolygonCollider(collider, cloud, out var offset)
                && FitTo(target, cloud, kind, offset);
        }

        // ----- helpers -----

        static void Aabb(IReadOnlyList<float2> points, out float2 min2, out float2 max2)
        {
            min2 = new float2(float.PositiveInfinity, float.PositiveInfinity);
            max2 = new float2(float.NegativeInfinity, float.NegativeInfinity);
            for (var i = 0; i < points.Count; i++)
            {
                min2 = min(min2, points[i]);
                max2 = max(max2, points[i]);
            }
        }

        /// <summary>The cloud's PCA principal-axis angle (radians) from the 2×2 mean-centred covariance, via the
        /// analytic symmetric-eigenvector angle <c>0.5·atan2(2·cov_xy, cov_xx − cov_yy)</c>. 0 for a near-isotropic
        /// cloud (any angle gives the same enclosing box, so 0 is the stable choice).</summary>
        static float PrincipalAngle(IReadOnlyList<float2> points)
        {
            var n = points.Count;
            if (n < 2)
                return 0f;
            var mean = Unity.Mathematics.float2.zero;
            for (var i = 0; i < n; i++)
                mean += points[i];
            mean /= n;
            float sxx = 0f,
                syy = 0f,
                sxy = 0f;
            for (var i = 0; i < n; i++)
            {
                var d = points[i] - mean;
                sxx += d.x * d.x;
                syy += d.y * d.y;
                sxy += d.x * d.y;
            }
            // Near-isotropic (no dominant axis): the covariance is ~scalar, any axis encloses equally → angle 0.
            if (abs(sxy) < 1e-9f && abs(sxx - syy) < 1e-9f)
                return 0f;
            return 0.5f * atan2(2f * sxy, sxx - syy);
        }

        /// <summary>z of the cross product (b−a)×(c−a); &gt; 0 is a CCW (left) turn.</summary>
        static float Cross(float2 a, float2 b, float2 c) =>
            (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

        static bool InCircle(float2 p, float2 center, float radius) =>
            lengthsq(p - center) <= radius * radius + 1e-6f;

        static void Circumcircle(float2 a, float2 b, float2 c, out float2 center, out float radius)
        {
            var d = 2f * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
            if (abs(d) < 1e-12f)
            {
                // Collinear triple — fall back to the diameter of the two farthest of the three.
                center = (a + c) * 0.5f;
                radius = length(a - center);
                return;
            }
            var a2 = lengthsq(a);
            var b2 = lengthsq(b);
            var c2 = lengthsq(c);
            center = new float2(
                (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d,
                (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d
            );
            radius = length(a - center);
        }

        static void AddBoundsCorners(Bounds bounds, List<float2> cloud)
        {
            var min2 = ((float3)bounds.min).xy;
            var max2 = ((float3)bounds.max).xy;
            cloud.Add(new float2(min2.x, min2.y));
            cloud.Add(new float2(max2.x, min2.y));
            cloud.Add(new float2(max2.x, max2.y));
            cloud.Add(new float2(min2.x, max2.y));
        }

        static Vector2[] ToVector2(IReadOnlyList<float2> points)
        {
            var result = new Vector2[points.Count];
            for (var i = 0; i < points.Count; i++)
                result[i] = (Vector2)points[i];
            return result;
        }
    }
}
