using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the Phase-1A collider-shape parity fixtures: for each of the
    /// four remaining built-in 2D collider shapes (Box / Capsule / Polygon / Edge), one child SubScene
    /// holding a dynamic body carrying that shape <em>resting on a static floor</em>, plus a parent scene
    /// carrying the SubScene. Both child and parent are registered in build settings so the runtime parity
    /// harness can load the parent (ECS bake) and additively load the child (the GameObject reference) by
    /// name. Single authoring: the same child scene feeds both backends.
    /// </summary>
    /// <remarks>
    /// The fixture differs from the free-fall circle: it carries a <em>static floor</em> so the shaped body
    /// settles, which is what makes a shaped collider's contact behaviour (the thing the shape mapping must
    /// get right) observable. The floor is a collider-only static body (no <see cref="Rigidbody2D"/>), which
    /// exercises the collider bakers' static-body fallback. The two bodies have distinct initial Y (floor at
    /// 0, shaped body at 5), so the harness's initial-pose matching key is stable across both backends.
    ///
    /// <para>Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.ColliderShapeFixtureBuilder.BuildAll</c>
    /// before the PlayMode parity tests.</para>
    /// </remarks>
    public static class ColliderShapeFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        // Floor: a wide, thin static box centred at the origin. Collider-only (no Rigidbody2D) → the
        // collider bakers' static-body fallback makes it a static body.
        const float FloorY = 0f;
        static readonly Vector2 FloorSize = new(40f, 1f);

        // Shaped dynamic body start height — well above the floor so it falls a few metres then rests.
        const float ShapeStartY = 5f;

        public const string BoxParent = FixtureRoot + "/BoxOnFloor.unity";
        public const string BoxChild = FixtureRoot + "/BoxOnFloor_Sub.unity";
        public const string CapsuleParent = FixtureRoot + "/CapsuleOnFloor.unity";
        public const string CapsuleChild = FixtureRoot + "/CapsuleOnFloor_Sub.unity";
        public const string PolygonParent = FixtureRoot + "/PolygonOnFloor.unity";
        public const string PolygonChild = FixtureRoot + "/PolygonOnFloor_Sub.unity";
        public const string EdgeParent = FixtureRoot + "/EdgeOnFloor.unity";
        public const string EdgeChild = FixtureRoot + "/EdgeOnFloor_Sub.unity";

        [MenuItem("Tools/Zori/Build Entities Physics2D Collider-Shape Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildBox();
            BuildCapsule();
            BuildPolygon();
            BuildEdge();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Entities Physics2D collider-shape fixtures built (Box/Capsule/Polygon/Edge on floor).");
        }

        static void BuildBox()
        {
            BuildShapeFixture(
                BoxChild,
                BoxParent,
                "BoxBody",
                go =>
                {
                    var box = go.AddComponent<BoxCollider2D>();
                    box.size = new Vector2(1f, 1f);
                }
            );
        }

        static void BuildCapsule()
        {
            BuildShapeFixture(
                CapsuleChild,
                CapsuleParent,
                "CapsuleBody",
                go =>
                {
                    var cap = go.AddComponent<CapsuleCollider2D>();
                    cap.size = new Vector2(1f, 2f);
                    cap.direction = CapsuleDirection2D.Vertical;
                }
            );
        }

        static void BuildPolygon()
        {
            BuildShapeFixture(
                PolygonChild,
                PolygonParent,
                "PolygonBody",
                go =>
                {
                    var poly = go.AddComponent<PolygonCollider2D>();
                    // A convex pentagon (PolygonGeometry.Create requires a convex 3..8-vertex hull).
                    poly.SetPath(
                        0,
                        new[]
                        {
                            new Vector2(0f, 0.6f),
                            new Vector2(-0.6f, 0.2f),
                            new Vector2(-0.35f, -0.5f),
                            new Vector2(0.35f, -0.5f),
                            new Vector2(0.6f, 0.2f),
                        }
                    );
                }
            );
        }

        static void BuildEdge()
        {
            // The Edge fixture is authored differently: the EdgeCollider2D is the STATIC surface, and a
            // dynamic CircleCollider2D body falls onto it. This is the faithful EdgeCollider2D parity
            // scenario — an edge/chain is a one-sided non-solid surface, the form built-in EdgeCollider2D and
            // Box2D chains are both designed for (static ground/walls). Attaching a chain to a DYNAMIC body
            // and expecting it to rest on a floor fights that design: a Box2D chain is one-sided, so a dynamic
            // chain body falls through a floor on its non-solid side (observed: it sank to y≈−40). So the
            // compared dynamic body carries a circle (already a proven shape), and the edge surface it lands
            // on is the thing under test — its baker, chain geometry, offset folding, and static-body path.
            // This is a scene-specific gate choice under the phase's fail-soft rule, not a relaxed envelope.
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Static edge ground: a wide, slightly-dished open chain centred at the origin. Collider-only
            // (no Rigidbody2D) → baked as a static body via the edge baker's static fallback.
            var edgeGo = new GameObject("EdgeGround");
            edgeGo.transform.position = new Vector3(0f, FloorY, 0f);
            var edge = edgeGo.AddComponent<EdgeCollider2D>();
            // Right-to-left winding: a Box2D chain is one-sided, and its solid side is determined by vertex
            // order. A left-to-right dish let the falling circle pass through (solid side faced down, observed
            // sink to y≈−40), so the points are wound right-to-left to face the solid (collidable) side
            // upward, toward the falling body.
            edge.points = new[]
            {
                new Vector2(10f, 1f),
                new Vector2(3f, 0f),
                new Vector2(-3f, 0f),
                new Vector2(-10f, 1f),
            };

            // Dynamic body that falls onto the edge surface and rests in the dish.
            var bodyGo = new GameObject("EdgeRestingBody");
            bodyGo.transform.position = new Vector3(0f, ShapeStartY, 0f);
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = bodyGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, EdgeChild);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject("Edge SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(EdgeChild);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, EdgeParent);

            RegisterSceneInBuildSettings(EdgeParent);
            RegisterSceneInBuildSettings(EdgeChild);
        }

        /// <summary>
        /// Author one shape fixture: a child scene with a static floor (collider-only) + one dynamic body
        /// carrying the shape produced by <paramref name="addShape"/>, and a parent scene carrying the child
        /// as an auto-loaded SubScene. Registers both in build settings.
        /// </summary>
        static void BuildShapeFixture(
            string childPath,
            string parentPath,
            string bodyName,
            System.Action<GameObject> addShape
        )
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Static floor: a collider-only Box (no Rigidbody2D) → baked as a static body.
            var floor = new GameObject("Floor");
            floor.transform.position = new Vector3(0f, FloorY, 0f);
            var floorBox = floor.AddComponent<BoxCollider2D>();
            floorBox.size = FloorSize;

            // Dynamic shaped body, raised above the floor.
            var bodyGo = new GameObject(bodyName);
            bodyGo.transform.position = new Vector3(0f, ShapeStartY, 0f);
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            addShape(bodyGo);

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(bodyName + " SubScene");
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
