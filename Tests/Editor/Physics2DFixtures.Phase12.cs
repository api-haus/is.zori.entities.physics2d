using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Phase-12 collider transform-scale fixtures for the EditMode gate, ported verbatim from
    /// <c>Phase12ColliderScaleFixtureBuilder</c>: real built-in colliders on Transforms with non-unit, non-uniform,
    /// and negative scale (the floor under test) plus an unscaled dynamic disc faller (the compared body). Each
    /// populate method authors the same GameObjects the builder's corresponding child scene authored, into the
    /// <paramref name="root"/>'s scene; the EditMode harness loads it as a SubScene and (for the parity gates)
    /// opens it additively as live built-in bodies. Geometry, scales, sizes, points, and the composite/custom
    /// shapes are reproduced exactly.
    /// </summary>
    public static partial class Physics2DFixtures
    {
        public const float P12WideBoxScaleX = 18.1822f;

        // A unit-scale dynamic disc faller (the compared body). NeverSleep + matched friction are applied by the
        // harness at load; here we just author the body + collider at the drop pose.
        static GameObject MakeDisc(GameObject root, string name, Vector2 pos, float radius)
        {
            var go = NewChild(root, name, new Vector3(pos.x, pos.y, 0f));
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = go.AddComponent<CircleCollider2D>();
            circle.radius = radius;
            return go;
        }

        // ---------------------------------------------------------------------------------------------------
        // (1) Wide box floor: Scale X = 18.1822 on a 1×1 BoxCollider2D. Two discs near the edges.

        public static void P12WideBox(GameObject root)
        {
            // The QA floor: a unit BoxCollider2D, top at y=0, on Scale (18.1822, 1). The baked half-extent must
            // be 18.1822/2 ≈ 9.09 in x, so the floor spans roughly x ∈ [-9.09, 9.09] at y ∈ [-0.5, 0.5].
            var floor = NewChild(root, "WideBoxFloor", new Vector3(0f, -0.5f, 0f)); // top edge at y=0
            floor.transform.localScale = new Vector3(P12WideBoxScaleX, 1f, 1f);
            var box = floor.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);

            // Two discs dropped near the EDGES of the wide floor — x=±7 is outside the unscaled 1×1 centre
            // (which spans only x ∈ [-0.5, 0.5]) but inside the scaled floor (x ∈ [-9.09, 9.09]). A buggy
            // unscaled floor would let these fall past; a correctly-scaled floor catches them at y≈0.5.
            MakeDisc(root, "EdgeDiscLeft", new Vector2(-7f, 4f), 0.5f);
            MakeDisc(root, "EdgeDiscRight", new Vector2(7f, 6f), 0.5f);
        }

        // ---------------------------------------------------------------------------------------------------
        // (2) Non-uniform circle floor. Scale (3, 1.4) on a radius-1 circle. cmax rule → effective radius = 3.

        public static void P12NonUniformCircle(GameObject root)
        {
            // A CircleCollider2D radius 1, centered so that under the cmax rule (effective radius = max(3,1.4)=3)
            // its top sits at a known height. Place the circle center at y = -3 so a cmax=3 circle's top is at
            // y=0; the disc then rests at center y ≈ 0.5.
            var floor = NewChild(root, "NonUniformCircleFloor", new Vector3(0f, -3f, 0f));
            floor.transform.localScale = new Vector3(3f, 1.4f, 1f);
            var circle = floor.AddComponent<CircleCollider2D>();
            circle.radius = 1f;

            // Disc dropped straight down onto the circle's apex (x=0).
            MakeDisc(root, "ApexDisc", new Vector2(0f, 5f), 0.5f);
        }

        // ---------------------------------------------------------------------------------------------------
        // (3) Non-uniform capsule floor. Scale (2.5, 1.2) on a horizontal capsule — caps re-derived from scaled size.

        public static void P12NonUniformCapsule(GameObject root)
        {
            // A horizontal CapsuleCollider2D (a wide flat lozenge). Size (4, 2), direction Horizontal. Under scale
            // (2.5, 1.2): scaled size (10, 2.4) → radius = 2.4/2 = 1.2, caps at x=±(5-1.2)=±3.8. Place center at
            // y = -1.2 so the scaled top is at y=0; disc rests at center y ≈ 0.5.
            var floor = NewChild(root, "NonUniformCapsuleFloor", new Vector3(0f, -1.2f, 0f));
            floor.transform.localScale = new Vector3(2.5f, 1.2f, 1f);
            var capsule = floor.AddComponent<CapsuleCollider2D>();
            capsule.size = new Vector2(4f, 2f);
            capsule.direction = CapsuleDirection2D.Horizontal;

            MakeDisc(root, "CapsuleDisc", new Vector2(0f, 5f), 0.5f);
        }

        // ---------------------------------------------------------------------------------------------------
        // (4) Negative-X-scaled asymmetric polygon floor — a winding-flip probe.

        public static void P12NegativePolygon(GameObject root)
        {
            // An ASYMMETRIC convex polygon (a right trapezoid) authored CCW, with a FLAT TOP at y=0. Under Scale
            // X = -2 the shape mirrors about x=0; the baker must reverse winding on the flip or PolygonGeometry
            // .Create rejects the inside-out hull and the disc falls through.
            var floor = NewChild(root, "NegativePolygonFloor", new Vector3(0f, 0f, 0f));
            floor.transform.localScale = new Vector3(-2f, 1f, 1f);
            var poly = floor.AddComponent<PolygonCollider2D>();
            poly.points = new[]
            {
                new Vector2(-2f, -1f), // bottom-left
                new Vector2(3f, -1f), // bottom-right (wider on the right — the asymmetry)
                new Vector2(2f, 0f), // top-right
                new Vector2(-1f, 0f), // top-left (narrower)
            };

            // Disc dropped at x=0 (inside the flat top span both before and after the mirror) rests at y ≈ 0.5.
            MakeDisc(root, "PolyDisc", new Vector2(0f, 5f), 0.5f);
        }

        // ---------------------------------------------------------------------------------------------------
        // (5) Negative-X-scaled asymmetric edge (chain) floor — the solid-side-is-winding probe.

        public static void P12NegativeEdge(GameObject root)
        {
            // A wide edge chain whose winding faces solid-side UP (the proven Phase-1A right-to-left dished
            // winding). Under Scale X = -2 the chain mirrors; a baker that did not reverse the point order would
            // flip the solid side DOWN and the disc would fall through.
            var edgeGo = NewChild(root, "NegativeEdgeFloor", new Vector3(0f, 0f, 0f));
            edgeGo.transform.localScale = new Vector3(-2f, 1f, 1f);
            var edge = edgeGo.AddComponent<EdgeCollider2D>();
            edge.points = new[]
            {
                new Vector2(8f, 1f),
                new Vector2(3f, 0f),
                new Vector2(-3f, 0f),
                new Vector2(-8f, 1f),
            };

            MakeDisc(root, "EdgeDisc", new Vector2(0f, 5f), 0.5f);
        }

        // ---------------------------------------------------------------------------------------------------
        // (6) Scaled-offset box floor — the offset must scale per-axis (signed).

        public static void P12ScaledOffset(GameObject root)
        {
            // A 1×1 BoxCollider2D with Offset (4, 0) on Scale (3, 1). The collider sits at offset*scale = (12, 0)
            // in world space (the GameObject is at origin). A disc dropped at x=12 must rest on the offset-shifted,
            // scaled box (width 3, spanning x ∈ [10.5, 13.5]).
            var floor = NewChild(root, "ScaledOffsetFloor", new Vector3(0f, -0.5f, 0f)); // box top (offset y=0) at y=0
            floor.transform.localScale = new Vector3(3f, 1f, 1f);
            var box = floor.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);
            box.offset = new Vector2(4f, 0f); // scaled to world +12 in x

            // Disc over the offset-shifted box top.
            MakeDisc(root, "OffsetDisc", new Vector2(12f, 5f), 0.5f);
        }

        // ---------------------------------------------------------------------------------------------------
        // (7) Scaled composite floor.

        static GameObject MakeComposite(
            GameObject root,
            string name,
            Vector3 worldScale,
            Vector2[] childBoxCenters,
            Vector2 childBoxSize
        )
        {
            var compositeRoot = NewChild(root, name, Vector3.zero);
            compositeRoot.transform.localScale = worldScale;
            var rb = compositeRoot.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var composite = compositeRoot.AddComponent<CompositeCollider2D>();
            composite.geometryType = CompositeCollider2D.GeometryType.Polygons;
            composite.generationType = CompositeCollider2D.GenerationType.Synchronous;

            for (var i = 0; i < childBoxCenters.Length; i++)
            {
                var childGo = new GameObject($"{name}_Box{i}");
                childGo.transform.SetParent(compositeRoot.transform, worldPositionStays: false);
                childGo.transform.localPosition = childBoxCenters[i];
                var box = childGo.AddComponent<BoxCollider2D>();
                box.size = childBoxSize;
                box.compositeOperation = Collider2D.CompositeOperation.Merge;
            }

            composite.GenerateGeometry();
            return compositeRoot;
        }

        public static void P12ScaledComposite(GameObject root)
        {
            // A flat merged bar of 3 unit boxes (local span x ∈ [-1.5, 1.5], y ∈ [-1, 0], top at local y=0) on a
            // NON-UNIT scale (4, 1.5): the scaled bar spans x ∈ [-6, 6], top at world y=0. A disc dropped at x=5
            // (well outside the UNSCALED bar's x ∈ [-1.5, 1.5]) rests on the scaled bar only if the merged path
            // was scaled.
            MakeComposite(
                root,
                "ScaledCompositeBar",
                new Vector3(4f, 1.5f, 1f),
                new[] { new Vector2(-1f, -0.5f), new Vector2(0f, -0.5f), new Vector2(1f, -0.5f) },
                new Vector2(1f, 1f)
            );
            MakeDisc(root, "CompositeDisc", new Vector2(5f, 5f), 0.5f);
        }

        // ---------------------------------------------------------------------------------------------------
        // (8) Scaled custom-collider floor.

        public static void P12ScaledCustom(GameObject root)
        {
            // A static CustomCollider2D carrying a single wide polygon base quad (local span x ∈ [-2, 2],
            // y ∈ [-1, 0], top at local y=0) on a NON-UNIT scale (3.5, 1.5): the scaled quad spans x ∈ [-7, 7],
            // top at world y=0. A disc at x=6 (outside the UNSCALED quad's x ∈ [-2, 2]) rests only if the group
            // vertices were scaled.
            var customRoot = NewChild(root, "ScaledCustomBody", Vector3.zero);
            customRoot.transform.localScale = new Vector3(3.5f, 1.5f, 1f);
            var rb = customRoot.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var custom = customRoot.AddComponent<CustomCollider2D>();

            var group = new PhysicsShapeGroup2D();
            group.AddPolygon(
                new List<Vector2>
                {
                    new Vector2(-2f, -1f),
                    new Vector2(2f, -1f),
                    new Vector2(2f, 0f),
                    new Vector2(-2f, 0f),
                }
            );
            custom.SetCustomShapes(group);

            MakeDisc(root, "CustomDisc", new Vector2(6f, 5f), 0.5f);
        }

        // ---------------------------------------------------------------------------------------------------
        // (9) Scaled DYNAMIC faller for the rendering-scale-preserved + smoothing pin.

        public static void P12ScaledDynamic(GameObject root)
        {
            // An unscaled wide static floor (a plain box) to catch the faller.
            var floor = NewChild(root, "DynFloor", new Vector3(0f, -0.5f, 0f));
            var box = floor.AddComponent<BoxCollider2D>();
            box.size = new Vector2(40f, 1f);

            // A NON-UNIFORMLY scaled dynamic disc: a Rigidbody2D+CircleCollider2D on Scale (2, 3). Its baked
            // LocalToWorld must carry (2, 3) as the column lengths (rendering-scale-preserved), and the smoothing
            // system must preserve the scale through interpolation. The disc carries Interpolate so the smoothing
            // system processes it.
            var disc = NewChild(root, "ScaledDynDisc", new Vector3(0f, 5f, 0f));
            disc.transform.localScale = new Vector3(2f, 3f, 1f);
            var rb = disc.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.interpolation = RigidbodyInterpolation2D.Interpolate;
            var circle = disc.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
        }
    }
}
