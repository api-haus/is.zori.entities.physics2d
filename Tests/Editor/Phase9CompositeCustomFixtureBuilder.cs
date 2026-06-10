using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free authoring of the Phase-9 GameObject-parity fixtures: real <see cref="CompositeCollider2D"/> and
    /// <see cref="CustomCollider2D"/> GameObjects in SubScenes, baked through the actual Phase-9 bakers, plus a
    /// regression fixture for the <see cref="EdgeCollider2D"/> material/layer fix. The validating gate
    /// (<c>Phase9CompositeCustomGate</c>) loads these and counts baked shapes — the witness the Phase-9 smoke
    /// (which authored the runtime multi-shape channel directly) never produced.
    /// </summary>
    /// <remarks>
    /// <para><b>Why these are REAL components, not runtime-authored buffers.</b> The Phase-9 smoke
    /// (<c>CompositeCustomColliderSmoke</c>) hand-builds a <c>PhysicsShape2D</c> + <c>DynamicBuffer&lt;PhysicsShape2DElement&gt;</c>
    /// to prove the runtime CREATION path, but it never runs <c>CompositeCollider2DBaker</c>/<c>CustomCollider2DBaker</c>
    /// against an actual built-in component. The merged-path geometry (<c>GetPath</c>) and the custom shape group
    /// (<c>GetCustomShapes</c>) only exist when the engine bakes a real GameObject, so the bakers' most error-prone
    /// surface — the GameObject API reads and the <c>compositeOperation</c> exclusion — ships unexecuted without
    /// these. Each fixture is a child SubScene (the authored components) + a parent scene carrying a
    /// <c>SubScene</c> that auto-loads and bakes it on PlayMode enter, both registered in build settings.</para>
    ///
    /// <para><b>Composite authoring.</b> A <c>CompositeCollider2D</c> requires a <c>Rigidbody2D</c> on its own
    /// GameObject; the merged children are sibling/child <c>BoxCollider2D</c>s whose
    /// <c>compositeOperation = Merge</c> attaches them to the composite. A merged-level-geometry surface is a
    /// STATIC rigidbody (the falling disc is the dynamic compared body). <c>geometryType</c> selects Polygons
    /// (decomposed convex fragments) vs Outlines (a closed chain loop).</para>
    ///
    /// <para>Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.Phase9CompositeCustomFixtureBuilder.BuildAll</c>
    /// before the PlayMode gate.</para>
    /// </remarks>
    public static class Phase9CompositeCustomFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        // --- Composite Polygons (concave L of merged boxes) ---
        public const string CompositePolygonsParent = FixtureRoot + "/P9CompositePolygons.unity";
        public const string CompositePolygonsChild = FixtureRoot + "/P9CompositePolygons_Sub.unity";

        // --- Composite Outlines (closed ring of merged boxes) ---
        public const string CompositeOutlinesParent = FixtureRoot + "/P9CompositeOutlines.unity";
        public const string CompositeOutlinesChild = FixtureRoot + "/P9CompositeOutlines_Sub.unity";

        // --- Custom collider (known mixed-kind PhysicsShapeGroup2D) ---
        public const string CustomGroupParent = FixtureRoot + "/P9CustomGroup.unity";
        public const string CustomGroupChild = FixtureRoot + "/P9CustomGroup_Sub.unity";

        // --- EdgeCollider2D material + layer regression ---
        public const string EdgeMaterialParent = FixtureRoot + "/P9EdgeMaterial.unity";
        public const string EdgeMaterialChild = FixtureRoot + "/P9EdgeMaterial_Sub.unity";

        // The non-default surface on the edge: high bounce + low friction, so a bouncing disc proves the
        // material was NOT silently dropped (the latent Phase-9 bug). A material-less edge would settle without
        // a bounce.
        public const string EdgeMaterialPath = FixtureRoot + "/P9EdgeBounceMaterial.physicsMaterial2D";
        public const float EdgeBounce = 0.85f;
        public const float EdgeFriction = 0.1f;

        // The filtered-layer pin: the edge sits on a layer the disc's layer is set NOT to collide with, so the
        // disc passes through (the Phase-5 filter is respected through the chain's contactFilter). Built-in
        // layers 8 and 9 are the project's first user layers; the gate's matrix disables the 8<->9 pair.
        public const int EdgeLayer = 8;
        public const int PassThroughDiscLayer = 9;

        // The number of merged child boxes in each composite, so the gate can assert the baked shape count is
        // NOT the child count (the merge collapsed them) and NOT zero.
        public const int PolygonsChildBoxCount = 3; // an L: two arms
        public const int OutlinesChildBoxCount = 3; // a solid bar (one closed outer outline)

        [MenuItem("Tools/Zori/Build Entities Physics2D Phase-9 Composite+Custom Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildEdgeMaterial();
            BuildCompositePolygons();
            BuildCompositeOutlines();
            BuildCustomGroup();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                "Entities Physics2D Phase-9 fixtures built (composite Polygons + Outlines, custom group, edge "
                    + "material/layer)."
            );
        }

        // ---------------------------------------------------------------------------------------------------

        // A composite of merged BoxCollider2D children. The composite GameObject carries the Rigidbody2D
        // (Static for a level-geometry surface) + the CompositeCollider2D; each child box sits on the SAME
        // GameObject hierarchy with compositeOperation = Merge so the engine merges it.
        static GameObject MakeComposite(
            UnityEngine.SceneManagement.Scene scene,
            string name,
            CompositeCollider2D.GeometryType geometryType,
            Vector2[] childBoxCenters,
            Vector2 childBoxSize
        )
        {
            var root = new GameObject(name);
            root.transform.position = Vector3.zero;
            var rb = root.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var composite = root.AddComponent<CompositeCollider2D>();
            composite.geometryType = geometryType;
            // Synchronous so the merged geometry regenerates as children are added; the baker also calls
            // GenerateGeometry, but Synchronous keeps the authoring asset's paths current at save time.
            composite.generationType = CompositeCollider2D.GenerationType.Synchronous;

            // Children: box colliders on child GameObjects, each merged into the composite. A child must share
            // the composite's Rigidbody2D — being a child of the composite GameObject achieves that.
            for (var i = 0; i < childBoxCenters.Length; i++)
            {
                var childGo = new GameObject($"{name}_Box{i}");
                childGo.transform.SetParent(root.transform, worldPositionStays: false);
                childGo.transform.localPosition = childBoxCenters[i];
                var box = childGo.AddComponent<BoxCollider2D>();
                box.size = childBoxSize;
                box.compositeOperation = Collider2D.CompositeOperation.Merge;
            }

            composite.GenerateGeometry();
            UnityEditor.SceneManagement.EditorSceneManager.MoveGameObjectToScene(root, scene);
            return root;
        }

        static GameObject MakeDisc(
            UnityEngine.SceneManagement.Scene scene,
            string name,
            Vector2 pos,
            float radius,
            int layer = 0,
            PhysicsMaterial2D material = null
        )
        {
            var go = new GameObject(name) { layer = layer };
            go.transform.position = pos;
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = go.AddComponent<CircleCollider2D>();
            circle.radius = radius;
            if (material != null)
                circle.sharedMaterial = material;
            UnityEditor.SceneManagement.EditorSceneManager.MoveGameObjectToScene(go, scene);
            return go;
        }

        static void BuildCompositePolygons()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // An L-shaped concave merged surface from 3 unit boxes: a horizontal arm of two boxes plus one box
            // stacked at the left to form the corner. Top of the horizontal arm at y=0; the vertical box rises
            // to y=1 at x in [-2.5,-1.5]. A disc dropped at x=0 rests on the horizontal arm top (y=0).
            // The concavity is real (an L is non-convex), so CreatePolygons must decompose into >= 2 fragments.
            MakeComposite(
                child,
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
            MakeDisc(child, "Disc", new Vector2(-1f, 5f), 0.5f);

            SaveChildAndParent(child, CompositePolygonsChild, CompositePolygonsParent, "P9CompositePolygons");
        }

        static void BuildCompositeOutlines()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A SOLID bar of 3 adjacent unit boxes — the merged Outlines geometry is a SINGLE closed outer
            // outline (one path), the cleanest count witness (a hollow frame would merge to outer + inner = >1
            // path). As Outlines this bakes to one closed loop chain. The bar top sits at y=0; a disc dropped
            // from above rests on the top, exercising the loop chain's solid-side winding vs the GameObject
            // oracle. Bar spans [-1.5,1.5]x[-1,0].
            MakeComposite(
                child,
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
            MakeDisc(child, "Disc", new Vector2(0f, 5f), 0.5f);

            SaveChildAndParent(child, CompositeOutlinesChild, CompositeOutlinesParent, "P9CompositeOutlines");
        }

        static void BuildCustomGroup()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A static CustomCollider2D carrying a known PhysicsShapeGroup2D of THREE shapes of distinct kinds:
            //  - a Polygon (a wide base quad, top at y=0) — the resting surface
            //  - a Circle (a bump off to the right, clear of the drop)
            //  - a Capsule (a bump off to the left, clear of the drop)
            // The gate asserts the baked shape count == 3 and the per-shape kinds match this group.
            var root = new GameObject("CustomBody");
            root.transform.position = Vector3.zero;
            var rb = root.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var custom = root.AddComponent<CustomCollider2D>();

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

            UnityEditor.SceneManagement.EditorSceneManager.MoveGameObjectToScene(root, child);

            // Disc dropped over the polygon base (x=0) rests on its top at y=0 -> center ~0.5.
            MakeDisc(child, "Disc", new Vector2(0f, 5f), 0.5f);

            SaveChildAndParent(child, CustomGroupChild, CustomGroupParent, "P9CustomGroup");
        }

        static void BuildEdgeMaterial()
        {
            // A bouncy, low-friction PhysicsMaterial2D authored as an asset so the SAME material feeds both the
            // ECS bake (Collider2DBaking.ReadSurface) and the GameObject reference (sharedMaterial). Before
            // Phase 9 the EdgeCollider2D baker silently dropped the material; this fixture makes the bounce
            // observable so a regression re-drops it loudly.
            var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial2D>(EdgeMaterialPath);
            if (material == null)
            {
                material = new PhysicsMaterial2D("P9EdgeBounceMaterial")
                {
                    friction = EdgeFriction,
                    bounciness = EdgeBounce,
                };
                AssetDatabase.CreateAsset(material, EdgeMaterialPath);
            }
            else
            {
                material.friction = EdgeFriction;
                material.bounciness = EdgeBounce;
            }

            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Static edge surface: a wide flat chain at y=0, wound right-to-left so its solid side faces up
            // (the Phase-1A edge winding finding). Carries the bouncy material AND sits on the filtered layer.
            var edgeGo = new GameObject("BouncyEdge") { layer = EdgeLayer };
            edgeGo.transform.position = Vector3.zero;
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
            UnityEditor.SceneManagement.EditorSceneManager.MoveGameObjectToScene(edgeGo, child);

            // The bouncing disc: layer 0 (collides with the edge layer under the default matrix), dropped onto
            // the edge, carrying NO material (default friction 0.4, bounciness 0). This is load-bearing: Box2D-v3
            // mixes a contact pair's bounciness with the MAXIMUM rule by default, so the rebound here can only
            // come from the EDGE's baked bounciness — a material-less disc on a correctly-baked 0.85 edge bounces
            // (max(0, 0.85)=0.85), while a material-less disc on a dropped-material edge does NOT (max(0,0)=0).
            // Giving the disc its own bouncy material would mask a dropped edge material (the disc would supply
            // the bounce), so the disc is deliberately material-less — the edge is the sole restitution source.
            MakeDisc(child, "BouncingDisc", new Vector2(0f, 4f), 0.5f, layer: 0, material: null);

            // The pass-through disc: on PassThroughDiscLayer (9), which the gate's matrix sets NOT to collide
            // with EdgeLayer (8). It must fall THROUGH the edge — proving the chain honours the baked filter.
            MakeDisc(child, "PassThroughDisc", new Vector2(5f, 4f), 0.5f, layer: PassThroughDiscLayer);

            SaveChildAndParent(child, EdgeMaterialChild, EdgeMaterialParent, "P9EdgeMaterial");
        }

        // ---------------------------------------------------------------------------------------------------

        static void SaveChildAndParent(
            UnityEngine.SceneManagement.Scene child,
            string childPath,
            string parentPath,
            string subSceneName
        )
        {
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(subSceneName + " SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(childPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, parentPath);

            RegisterSceneInBuildSettings(parentPath);
            RegisterSceneInBuildSettings(childPath);
        }

        static void RegisterSceneInBuildSettings(string scenePath)
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (scenes.Exists(s => s.path == scenePath))
                return;
            scenes.Add(new EditorBuildSettingsScene(scenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }
    }
}
