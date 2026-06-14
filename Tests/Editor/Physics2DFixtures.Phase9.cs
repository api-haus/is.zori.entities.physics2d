using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Phase-9 fixtures (composite Polygons + Outlines, custom group, edge material/layer) authored as
    /// populate methods for the EditMode harness. The GameObjects here are reproduced VERBATIM from
    /// <c>Phase9CompositeCustomFixtureBuilder</c> — the same <see cref="CompositeCollider2D"/> children +
    /// geometryType, the same <see cref="CustomCollider2D"/> <c>PhysicsShapeGroup2D</c>, and the same bouncy
    /// edge material + layers — dropping only the builder's separate child/parent scene save and the
    /// build-settings registration (the harness's <c>BuildScene</c> owns scene wiring). Material assets are
    /// persisted into <see cref="CurrentFolder"/> so they are deleted with the temp fixture.
    /// </summary>
    public static partial class Physics2DFixtures
    {
        // The non-default surface on the edge: high bounce + low friction (verbatim from the builder).
        const float Phase9EdgeBounce = 0.85f;
        const float Phase9EdgeFriction = 0.1f;

        // The edge sits on layer 8; the pass-through disc on layer 9, which the gate's matrix disables vs 8.
        const int Phase9EdgeLayer = 8;
        const int Phase9PassThroughDiscLayer = 9;

        // ---- Composite Polygons (concave L of merged boxes) -----------------------------------------------

        public static void CompositePolygons(GameObject root)
        {
            // An L-shaped concave merged surface from 3 unit boxes: a horizontal arm of two boxes plus one box
            // stacked at the left to form the corner. Top of the horizontal arm at y=0; the vertical box rises
            // to y=1 at x in [-2.5,-1.5]. A disc dropped at x=0 rests on the horizontal arm top (y=0).
            // The concavity is real (an L is non-convex), so CreatePolygons must decompose into >= 2 fragments.
            MakeComposite(
                root,
                "CompositeL",
                CompositeCollider2D.GeometryType.Polygons,
                new[]
                {
                    new Vector2(-2f, -0.5f), // left base box  -> spans [-2.5,-1.5]x[-1,0]
                    new Vector2(-1f, -0.5f), // mid base box   -> spans [-1.5,-0.5]x[-1,0]
                    new Vector2(-2f, 0.5f), // left riser box  -> spans [-2.5,-1.5]x[0,1]
                },
                new Vector2(1f, 1f)
            );
            // Disc rests on the base arm top (y=0) at x=-1 (over the mid box, clear of the riser) -> center ~0.5.
            MakeDisc(root, "Disc", new Vector2(-1f, 5f), 0.5f);
        }

        // ---- Composite Outlines (closed ring of merged boxes) --------------------------------------------

        public static void CompositeOutlines(GameObject root)
        {
            // A SOLID bar of 3 adjacent unit boxes — the merged Outlines geometry is a SINGLE closed outer
            // outline (one path), the cleanest count witness (a hollow frame would merge to outer + inner = >1
            // path). As Outlines this bakes to one closed loop chain. The bar top sits at y=0; a disc dropped
            // from above rests on the top, exercising the loop chain's solid-side winding vs the GameObject
            // oracle. Bar spans [-1.5,1.5]x[-1,0].
            MakeComposite(
                root,
                "CompositeBar",
                CompositeCollider2D.GeometryType.Outlines,
                new[]
                {
                    new Vector2(-1f, -0.5f), // -> [-1.5,-0.5]x[-1,0]
                    new Vector2(0f, -0.5f), //  -> [-0.5,0.5]x[-1,0]
                    new Vector2(1f, -0.5f), //  -> [0.5,1.5]x[-1,0]
                },
                new Vector2(1f, 1f)
            );
            // Disc rests on the bar's top edge (y=0) -> center ~0.5.
            MakeDisc(root, "Disc", new Vector2(0f, 5f), 0.5f);
        }

        // ---- Custom collider (known mixed-kind PhysicsShapeGroup2D) --------------------------------------

        public static void CustomGroup(GameObject root)
        {
            // A static CustomCollider2D carrying a known PhysicsShapeGroup2D of THREE shapes of distinct kinds:
            //  - a Polygon (a wide base quad, top at y=0) — the resting surface
            //  - a Circle (a bump off to the right, clear of the drop)
            //  - a Capsule (a bump off to the left, clear of the drop)
            // The gate asserts the baked shape count == 3 and the per-shape kinds match this group.
            var bodyGo = NewChild(root, "CustomBody", Vector3.zero);
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var custom = bodyGo.AddComponent<CustomCollider2D>();

            var group = new PhysicsShapeGroup2D();
            // Polygon base quad spanning [-2,2]x[-1,0] (top at y=0). AddPolygon takes a convex vertex list.
            group.AddPolygon(
                new List<Vector2>
                {
                    new Vector2(-2f, -1f),
                    new Vector2(2f, -1f),
                    new Vector2(2f, 0f),
                    new Vector2(-2f, 0f),
                }
            );
            // Circle bump centered at (3, 0.5), radius 0.5 — off to the right, away from the drop column.
            group.AddCircle(new Vector2(3f, 0.5f), 0.5f);
            // Capsule bump: two end-cap centers at (-3.5,0.25) and (-2.5,0.25), radius 0.25 — off to the left.
            group.AddCapsule(new Vector2(-3.5f, 0.25f), new Vector2(-2.5f, 0.25f), 0.25f);
            custom.SetCustomShapes(group);

            // Disc dropped over the polygon base (x=0) rests on its top at y=0 -> center ~0.5.
            MakeDisc(root, "Disc", new Vector2(0f, 5f), 0.5f);
        }

        // ---- EdgeCollider2D material + layer regression --------------------------------------------------

        public static void EdgeMaterial(GameObject root)
        {
            // A bouncy, low-friction PhysicsMaterial2D persisted as an asset so the SAME material feeds both the
            // ECS bake (Collider2DBaking.ReadSurface) and the GameObject reference (sharedMaterial). Before
            // Phase 9 the EdgeCollider2D baker silently dropped the material; this fixture makes the bounce
            // observable so a regression re-drops it loudly. Persisted into the temp folder so it is deleted
            // with the fixture.
            var material = new PhysicsMaterial2D("P9EdgeBounceMaterial")
            {
                friction = Phase9EdgeFriction,
                bounciness = Phase9EdgeBounce,
            };
            AssetDatabase.CreateAsset(material, CurrentFolder + "/P9EdgeBounceMaterial.physicsMaterial2D");

            // Static edge surface: a wide flat chain at y=0, wound right-to-left so its solid side faces up
            // (the Phase-1A edge winding finding). Carries the bouncy material AND sits on the filtered layer.
            var edgeGo = NewChild(root, "BouncyEdge", Vector3.zero);
            edgeGo.layer = Phase9EdgeLayer;
            var edge = edgeGo.AddComponent<EdgeCollider2D>();
            // The PROVEN solid-side-up winding from the Phase-1A EdgeOnFloor fixture (right-to-left, slightly
            // dished up at the ends): a Box2D chain is one-sided, and this vertex order faces the collidable
            // side UP toward the falling discs. A flat right-to-left chain faced solid-side DOWN (the discs fell
            // through, observed y=-74), so the dished proven winding is reused. >= 3 vertices is also the Box2D
            // ChainGeometry minimum (a 2-vertex edge is legal built-in authoring but cannot become a chain — a
            // known package boundary, noted in the gate).
            edge.points = new[]
            {
                new Vector2(10f, 1f),
                new Vector2(3f, 0f),
                new Vector2(-3f, 0f),
                new Vector2(-10f, 1f),
            };
            edge.sharedMaterial = material;

            // The bouncing disc: layer 0 (collides with the edge layer under the default matrix), dropped onto
            // the edge, carrying NO material (default friction 0.4, bounciness 0). This is load-bearing: Box2D-v3
            // mixes a contact pair's bounciness with the MAXIMUM rule by default, so the rebound here can only
            // come from the EDGE's baked bounciness — a material-less disc on a correctly-baked 0.85 edge bounces
            // (max(0, 0.85)=0.85), while a material-less disc on a dropped-material edge does NOT (max(0,0)=0).
            // Giving the disc its own bouncy material would mask a dropped edge material (the disc would supply
            // the bounce), so the disc is deliberately material-less — the edge is the sole restitution source.
            MakeDisc(root, "BouncingDisc", new Vector2(0f, 4f), 0.5f, layer: 0, material: null);

            // The pass-through disc: on PassThroughDiscLayer (9), which the gate's matrix sets NOT to collide
            // with EdgeLayer (8). It must fall THROUGH the edge — proving the chain honours the baked filter.
            MakeDisc(root, "PassThroughDisc", new Vector2(5f, 4f), 0.5f, layer: Phase9PassThroughDiscLayer);
        }

        // ---- composite / disc authoring helpers (verbatim port of the builder's MakeComposite/MakeDisc) ---

        // A composite of merged BoxCollider2D children. The composite GameObject carries the Rigidbody2D
        // (Static for a level-geometry surface) + the CompositeCollider2D; each child box is a child GameObject
        // with compositeOperation = Merge so the engine merges it.
        static GameObject MakeComposite(
            GameObject root,
            string name,
            CompositeCollider2D.GeometryType geometryType,
            Vector2[] childBoxCenters,
            Vector2 childBoxSize
        )
        {
            var compositeGo = NewChild(root, name, Vector3.zero);
            var rb = compositeGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var composite = compositeGo.AddComponent<CompositeCollider2D>();
            composite.geometryType = geometryType;
            // Synchronous so the merged geometry regenerates as children are added; the baker also calls
            // GenerateGeometry, but Synchronous keeps the authoring asset's paths current at save time.
            composite.generationType = CompositeCollider2D.GenerationType.Synchronous;

            // Children: box colliders on child GameObjects, each merged into the composite. A child must share
            // the composite's Rigidbody2D — being a child of the composite GameObject achieves that.
            for (var i = 0; i < childBoxCenters.Length; i++)
            {
                var childGo = new GameObject($"{name}_Box{i}");
                childGo.transform.SetParent(compositeGo.transform, worldPositionStays: false);
                childGo.transform.localPosition = childBoxCenters[i];
                var box = childGo.AddComponent<BoxCollider2D>();
                box.size = childBoxSize;
                box.compositeOperation = Collider2D.CompositeOperation.Merge;
            }

            composite.GenerateGeometry();
            return compositeGo;
        }

        static GameObject MakeDisc(
            GameObject root,
            string name,
            Vector2 pos,
            float radius,
            int layer = 0,
            PhysicsMaterial2D material = null
        )
        {
            var go = NewChild(root, name, pos);
            go.layer = layer;
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = go.AddComponent<CircleCollider2D>();
            circle.radius = radius;
            if (material != null)
                circle.sharedMaterial = material;
            return go;
        }
    }
}
