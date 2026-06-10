using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free authoring of the filter-BAKE parity fixture: a child SubScene holding four static
    /// collider-only circle bodies on layers {0, 8, 9, 11}, referenced by an auto-loaded parent
    /// <c>SubScene</c>. This is the geometry the bake gate (<c>FilterBakeParityGate</c>) loads to exercise
    /// <c>Collider2DBaking.ReadFilter</c> — the baker's read of the project layer-collision-matrix into the
    /// baked <c>categoryBits</c>/<c>contactBits</c>.
    /// </summary>
    /// <remarks>
    /// <para><b>This builder does NOT set the matrix.</b> In this project a SubScene bakes LIVE on
    /// PlayMode-load (no closed-bake entity binary is produced at edit time), so a matrix mutated here and
    /// restored before the bake runs has no effect on the baked bits — the live bake reads the project matrix
    /// as it is at load. The bake gate therefore OWNS the matrix in its own <c>[SetUp]</c>, before
    /// <c>LoadScene</c> streams the SubScene, and this builder only authors the matrix-independent geometry
    /// (the four bodies on their four layers).</para>
    ///
    /// Run before the PlayMode test via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.FilterBakeFixtureBuilder.Build</c>.
    /// </remarks>
    public static class FilterBakeFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";
        public const string ParentScenePath = FixtureRoot + "/FilterBake.unity";
        public const string ChildScenePath = FixtureRoot + "/FilterBake_Sub.unity";

        // The four layers each body sits on. The bake gate sets a matrix in which layer 8 ignores 9 and layer
        // 0 ignores 11, then asserts the baked rows track it.
        public const int LA = 8;
        public const int LB = 9;
        public const int LDefault = 0;
        public const int LX = 11;

        // Baked body authored positions (y), each on a layer, so the PlayMode test maps a baked entity to its
        // intended layer by its baked initialPosition.
        public const float YA = 2f; // layer 8
        public const float YB = 4f; // layer 9
        public const float YDefault = 6f; // layer 0
        public const float YX = 8f; // layer 11

        [MenuItem("Tools/Zori/Build Entities Physics2D Filter-Bake Fixture")]
        public static void Build()
        {
            Directory.CreateDirectory(FixtureRoot);

            // Child scene = the SubScene contents: four static collider-only circle bodies on the four layers.
            // Collider-only (no Rigidbody2D) → the baker emits a static PhysicsBody2DDefinition.
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            MakeBaked(child, "BodyA", new Vector3(0f, YA, 0f), LA);
            MakeBaked(child, "BodyB", new Vector3(0f, YB, 0f), LB);
            MakeBaked(child, "BodyDefault", new Vector3(0f, YDefault, 0f), LDefault);
            MakeBaked(child, "BodyX", new Vector3(0f, YX, 0f), LX);
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, ChildScenePath);

            // Parent scene = a SubScene component referencing the child, auto-loaded so it streams + bakes
            // live on PlayMode enter under whatever matrix the bake gate set.
            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject("FilterBake SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(ChildScenePath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, ParentScenePath);

            RegisterSceneInBuildSettings(ParentScenePath);
            RegisterSceneInBuildSettings(ChildScenePath);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                $"Entities Physics2D filter-bake fixture built (4 bodies on layers {LDefault},{LA},{LB},{LX}). "
                    + $"PlayMode-load: {ParentScenePath}"
            );
        }

        static void MakeBaked(UnityEngine.SceneManagement.Scene scene, string name, Vector3 pos, int layer)
        {
            var go = new GameObject(name) { layer = layer };
            go.transform.position = pos;
            var c = go.AddComponent<CircleCollider2D>();
            c.radius = 0.5f;
            UnityEditor.SceneManagement.EditorSceneManager.MoveGameObjectToScene(go, scene);
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
