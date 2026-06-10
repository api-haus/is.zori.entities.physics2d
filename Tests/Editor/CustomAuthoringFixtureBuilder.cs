using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the Phase-3 custom-authoring parity fixtures, mirroring the
    /// existing fixture builders (a child SubScene + a parent carrying it, both registered in build settings).
    /// Two fixtures:
    /// <list type="bullet">
    /// <item><b>CustomVsBuiltIn</b> — one child scene authoring a circle body via
    /// <see cref="PhysicsBody2DAuthoring"/>/<see cref="PhysicsShape2DAuthoring"/> at X = −2 AND an equivalent
    /// circle body via built-in <c>Rigidbody2D</c>/<c>CircleCollider2D</c> at X = +2, identical save for X.
    /// Both bake into one ECS world for the tight same-solver comparison.</item>
    /// <item><b>CustomCircleOnFloor</b> — a custom-authored circle body falling onto a custom-authored static
    /// box floor, for the broad GameObject-oracle gate run against a custom-authored scene.</item>
    /// </list>
    /// Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.CustomAuthoringFixtureBuilder.BuildAll</c>
    /// before the PlayMode custom-authoring tests.
    /// </summary>
    public static class CustomAuthoringFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        public const string CustomVsBuiltInParent = FixtureRoot + "/CustomVsBuiltIn.unity";
        public const string CustomVsBuiltInChild = FixtureRoot + "/CustomVsBuiltIn_Sub.unity";
        public const string CustomFloorParent = FixtureRoot + "/CustomCircleOnFloor.unity";
        public const string CustomFloorChild = FixtureRoot + "/CustomCircleOnFloor_Sub.unity";

        const float StartY = 10f;
        const float CircleRadius = 0.5f;

        [MenuItem("Tools/Zori/Build Entities Physics2D Custom-Authoring Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildCustomVsBuiltIn();
            BuildCustomCircleOnFloor();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Entities Physics2D custom-authoring fixtures built (CustomVsBuiltIn, CustomCircleOnFloor).");
        }

        static void BuildCustomVsBuiltIn()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Custom-authored body at the more-negative X (index 0 in the X-sorted compare).
            var customGo = new GameObject("CustomBody");
            customGo.transform.position = new Vector3(-2f, StartY, 0f);
            var body = customGo.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.GravityScale = 1f;
            body.UseAutoMass = true; // density-derived mass — matches the built-in default-collider path
            var shape = customGo.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = CircleRadius;
            shape.Density = 1f;

            // Built-in body at the more-positive X (index 1), authored to match the custom body's params.
            var builtinGo = new GameObject("BuiltInBody");
            builtinGo.transform.position = new Vector3(2f, StartY, 0f);
            var rb = builtinGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.useAutoMass = true;
            var circle = builtinGo.AddComponent<CircleCollider2D>();
            circle.radius = CircleRadius;
            circle.density = 1f;

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, CustomVsBuiltInChild);

            SaveParentWithSubScene(CustomVsBuiltInParent, CustomVsBuiltInChild, "CustomVsBuiltIn SubScene");

            RegisterSceneInBuildSettings(CustomVsBuiltInParent);
            RegisterSceneInBuildSettings(CustomVsBuiltInChild);
        }

        static void BuildCustomCircleOnFloor()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Custom-authored static box floor (no PhysicsBody2DAuthoring → the shape baker's static fallback).
            var floor = new GameObject("CustomFloor");
            floor.transform.position = new Vector3(0f, 0f, 0f);
            var floorShape = floor.AddComponent<PhysicsShape2DAuthoring>();
            floorShape.Kind = PhysicsShape2DKind.Box;
            floorShape.BoxSize = new Unity.Mathematics.float2(40f, 1f);

            // Custom-authored dynamic circle body that falls onto the floor and rests.
            var bodyGo = new GameObject("CustomFallingBody");
            bodyGo.transform.position = new Vector3(0f, 5f, 0f);
            var body = bodyGo.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.GravityScale = 1f;
            body.UseAutoMass = true;
            var shape = bodyGo.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = CircleRadius;
            shape.Density = 1f;

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, CustomFloorChild);

            SaveParentWithSubScene(CustomFloorParent, CustomFloorChild, "CustomCircleOnFloor SubScene");

            RegisterSceneInBuildSettings(CustomFloorParent);
            RegisterSceneInBuildSettings(CustomFloorChild);
        }

        static void SaveParentWithSubScene(string parentPath, string childPath, string subSceneName)
        {
            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(subSceneName);
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(childPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, parentPath);
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
