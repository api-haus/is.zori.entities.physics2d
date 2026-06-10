using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the falling-body PlayMode fixture: a child SubScene holding
    /// one GameObject with a built-in <see cref="Rigidbody2D"/> (Dynamic) and <see cref="CircleCollider2D"/>,
    /// referenced by a parent scene's <see cref="SubScene"/> component, with the parent scene registered
    /// in build settings so the runtime PlayMode test can load it via <c>SceneManager.LoadScene</c>.
    /// </summary>
    /// <remarks>
    /// SubScene authoring is an edit-time operation, so it cannot live in the runtime/PlayMode test
    /// assembly — this is the editor entry point the slice's gate depends on. Run it via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.FallingBodyFixtureBuilder.Build</c> before
    /// running the PlayMode test (the editor bakes the SubScene's authoring components through the
    /// package's <c>Rigidbody2DBaker</c>/<c>CircleCollider2DBaker</c>).
    /// </remarks>
    public static class FallingBodyFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";
        public const string ParentScenePath = FixtureRoot + "/FallingBody.unity";

        // The child (authoring) scene is also registered in build settings, so the parity harness can
        // additively load it at runtime by name and instantiate the SAME built-in-authored bodies as the
        // GameObject reference. This is what makes the oracle single-authoring: one authored child scene
        // feeds the ECS bake (via the parent's SubScene) AND the GameObject reference (via this additive
        // load) — there is no second hand-authored reference scene to drift out of sync.
        public const string ChildScenePath = FixtureRoot + "/FallingBody_Sub.unity";

        // The authored start height. The runtime test asserts world-Y strictly decreases from here.
        public const float StartY = 10f;

        [MenuItem("Tools/Zori/Build Entities Physics2D Falling-Body Fixture")]
        public static void Build()
        {
            Directory.CreateDirectory(FixtureRoot);

            // 1) Child scene = the SubScene contents: one Dynamic body + circle collider, raised so it
            //    has room to fall under gravity.
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var bodyGo = new GameObject("FallingBody");
            bodyGo.transform.position = new Vector3(0f, StartY, 0f);
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = bodyGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, ChildScenePath);

            // 2) Parent scene = an empty scene carrying a SubScene component that references the child as
            //    an authoring SubScene (AutoLoadScene so it streams + bakes on PlayMode enter).
            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject("FallingBody SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(ChildScenePath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, ParentScenePath);

            RegisterSceneInBuildSettings(ParentScenePath);
            // Register the child too: the parity harness loads it additively at runtime (by name) to build
            // the GameObject reference from the identical authored bodies. Runtime SceneManager loads by
            // name only resolve against build-settings-registered scenes, so an unregistered child cannot
            // be reached from the all-platforms PlayMode test assembly.
            RegisterSceneInBuildSettings(ChildScenePath);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log($"Entities Physics2D falling-body fixture built. PlayMode-load: {ParentScenePath}");
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
