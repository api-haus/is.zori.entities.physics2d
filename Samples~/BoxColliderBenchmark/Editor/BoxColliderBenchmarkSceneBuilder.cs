using System.Collections.Generic;
using System.IO;
using Unity.Mathematics;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Samples.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the openable Box-Collider-Benchmark demo scene, mirroring the
    /// package's <c>CustomAuthoringFixtureBuilder</c> (a child SubScene authoring the workload + a parent host
    /// carrying the SubScene reference + a 2D camera, both registered in build settings). The result is a scene a
    /// user opens and presses Play: the spawner sprays box-collider quads that fall, render off the physics
    /// <c>LocalToWorld</c> via Unity.Entities.Graphics, and pile on a static floor, while the timing instrument
    /// logs a <c>[BoxColliderBenchmark]</c> line per creation frame. The dedup control
    /// (<c>PhysicsStep2DAuthoring.CacheIdenticalBodies</c> / <c>IdenticalBodyThreshold</c>) is authored into the
    /// SubScene so a user can toggle it and re-bake between runs.
    /// </summary>
    /// <remarks>
    /// The scene is authored programmatically rather than hand-written as YAML: scene YAML is GUID-sensitive and
    /// fragile, whereas <c>EditorSceneManager</c> + <c>AddComponent</c> produce correct MonoBehaviour script
    /// references and a correct <c>SubScene</c> binding every time. Re-run it any time to regenerate the scene
    /// from scratch (it overwrites the saved <c>.unity</c> assets). The demo is a <em>watch-it-work</em> artifact:
    /// the desktop timing numbers it logs are NOT a benchmark result (benchmarks run on the Steam Deck only).
    ///
    /// <para>Run via the menu (<c>Tools/Zori/Build Box-Collider Benchmark Demo Scene</c>) or
    /// <c>-executeMethod Zori.Entities.Physics2D.Samples.Editor.BoxColliderBenchmarkSceneBuilder.Build</c>.</para>
    /// </remarks>
    public static class BoxColliderBenchmarkSceneBuilder
    {
        public const string SceneRoot = "Assets/EntitiesPhysics2DBenchmark/Scenes";

        public const string HostScenePath = SceneRoot + "/BoxColliderBenchmarkDemo.unity";
        public const string SubScenePath = SceneRoot + "/BoxColliderBenchmarkDemo_Sub.unity";

        // Authored workload defaults — sensible, visible, watchable. A few thousand boxes total at a modest
        // per-second rate so the spray spans a few seconds; a per-frame ceiling well above the rate × dt so the
        // rate is the binding pace. The fields are freely editable in the inspector up to the ~1M stress ceiling;
        // these are just the watchable demo defaults. A spawn AABB above the camera centre so quads fall into frame.
        const int SpawnedTotalLimit = 4096;
        const float SpawnedPerSecondTarget = 1000f;
        const int SpawnedPerFrameMax = 256;
        static readonly float2 BoxSize = new float2(0.4f, 0.4f);

        // Static floor the sprayed boxes pile up on, a touch below the camera centre. 40 wide centred at x=0, so
        // its collidable extent is x in [-20, 20].
        static readonly Vector3 FloorPos = new Vector3(0f, -3f, 0f);
        static readonly float2 FloorSize = new float2(40f, 1f);

        // Spawn band: scattered across (nearly) the full floor width so the boxes land ACROSS the floor rather than
        // piling in a narrow centre. Inset ~2 m from the floor's [-20, 20] collidable edges so a box never spawns
        // hanging past the collider. The boxes fall straight down (no horizontal velocity seed), so the landing
        // spread is exactly this spawn band — matching it to the floor is what fills the floor.
        static readonly float2 SpawnMin = new float2(-18f, 5f);
        static readonly float2 SpawnMax = new float2(18f, 12f);

        // Dedup control (the optimisation under test) — authored ON at the package-default threshold so a user sees
        // it engaged out of the box and can toggle it off / re-bake to compare.
        const bool CacheIdenticalBodies = true;
        const int IdenticalBodyThreshold = 8;

        // 2D camera framing: orthographic, centred so the spawn band (y 5..12) and the pile (y ~ -3) are both in
        // frame. Size 9 → vertical view y ~ -5..13; width scales with the game-view aspect.
        static readonly Vector3 CameraPos = new Vector3(0f, 4f, -10f);
        const float OrthoSize = 9f;

        [MenuItem("Tools/Zori/Build Box-Collider Benchmark Demo Scene")]
        public static void Build()
        {
            Directory.CreateDirectory(SceneRoot);

            BuildSubScene();
            BuildHostScene();

            RegisterSceneInBuildSettings(HostScenePath);
            RegisterSceneInBuildSettings(SubScenePath);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                "[BoxColliderBenchmark] Built demo scene: open "
                    + HostScenePath
                    + " and press Play (sprays box-collider quads that fall, render, and pile; logs timing). "
                    + "Toggle the dedup on the PhysicsStep2D GameObject in the SubScene and re-bake to compare."
            );
        }

        // The SubScene authors the workload: the benchmark spray config, the physics-step + dedup control, and a
        // static floor. Its bake emits the ECS singletons that arm the spawner and configure the world.
        static void BuildSubScene()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // The spray config — a MonoBehaviour whose baker emits BoxColliderBenchmarkConfig. Its serialized
            // fields are private (no public setters), so author them through SerializedObject — GUID-free and
            // robust to field renames the inspector already tracks.
            var benchGo = new GameObject("BoxColliderBenchmark");
            var bench = benchGo.AddComponent<BoxColliderBenchmarkAuthoring>();
            var so = new SerializedObject(bench);
            SetInt(so, "m_SpawnedTotalLimit", SpawnedTotalLimit);
            SetFloat(so, "m_SpawnedPerSecondTarget", SpawnedPerSecondTarget);
            SetInt(so, "m_SpawnedPerFrameMax", SpawnedPerFrameMax);
            SetVector2(so, "m_BoxSize", BoxSize);
            SetVector2(so, "m_SpawnMin", SpawnMin);
            SetVector2(so, "m_SpawnMax", SpawnMax);
            so.ApplyModifiedPropertiesWithoutUndo();

            // The dedup control surface — the package's PhysicsStep2DAuthoring. Authored ON at the default
            // threshold; a user toggles CacheIdenticalBodies (or changes IdenticalBodyThreshold) here and re-bakes
            // the SubScene to sweep the optimisation. Public setters exist, so set them directly.
            var stepGo = new GameObject("PhysicsStep2D");
            var step = stepGo.AddComponent<PhysicsStep2DAuthoring>();
            step.CacheIdenticalBodies = CacheIdenticalBodies;
            step.IdenticalBodyThreshold = IdenticalBodyThreshold;

            // A static box floor so the sprayed dynamic boxes pile up visibly. A shape with no
            // PhysicsBody2DAuthoring would fall back to static, but author the body explicitly as Static for
            // clarity in the inspector.
            var floorGo = new GameObject("Floor");
            floorGo.transform.position = FloorPos;
            var floorBody = floorGo.AddComponent<PhysicsBody2DAuthoring>();
            floorBody.BodyType = PhysicsBody2DMotionType.Static;
            var floorShape = floorGo.AddComponent<PhysicsShape2DAuthoring>();
            floorShape.Kind = PhysicsShape2DKind.Box;
            floorShape.BoxSize = FloorSize;

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, SubScenePath);
        }

        // The host scene a user opens: a 2D camera framed on the spawn/pile region plus the SubScene reference.
        static void BuildHostScene()
        {
            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var cameraGo = new GameObject("Main Camera");
            cameraGo.tag = "MainCamera";
            cameraGo.transform.position = CameraPos;
            var camera = cameraGo.AddComponent<Camera>();
            camera.orthographic = true;
            camera.orthographicSize = OrthoSize;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.06f, 0.06f, 0.09f, 1f);
            // URP attaches its per-camera data component (UniversalAdditionalCameraData) automatically when the
            // camera first renders, so the builder does not reference the URP runtime assembly itself.

            var subSceneGo = new GameObject("BoxColliderBenchmark SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(SubScenePath);
            subScene.AutoLoadScene = true;

            // The benchmark HUD — a plain-MonoBehaviour IMGUI overlay (spawned-box counter + rolling-window FPS
            // stats), in the host SCENE (not the SubScene / ECS), so it is present and drawing on Play.
            var hudGo = new GameObject("BenchmarkGUI");
            hudGo.AddComponent<BenchmarkGUI>();

            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, HostScenePath);
        }

        static void SetInt(SerializedObject so, string field, int value)
        {
            var prop = so.FindProperty(field);
            if (prop != null)
                prop.intValue = value;
            else
                Debug.LogWarning($"[BoxColliderBenchmark] serialized field '{field}' not found");
        }

        static void SetFloat(SerializedObject so, string field, float value)
        {
            var prop = so.FindProperty(field);
            if (prop != null)
                prop.floatValue = value;
            else
                Debug.LogWarning($"[BoxColliderBenchmark] serialized field '{field}' not found");
        }

        static void SetVector2(SerializedObject so, string field, float2 value)
        {
            var prop = so.FindProperty(field);
            if (prop != null)
                prop.vector2Value = new Vector2(value.x, value.y);
            else
                Debug.LogWarning($"[BoxColliderBenchmark] serialized field '{field}' not found");
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
