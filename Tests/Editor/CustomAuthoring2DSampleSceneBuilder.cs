using System.Collections.Generic;
using System.IO;
using Unity.Mathematics;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the <c>CustomAuthoring2D</c> sample scene — the Phase-E deliverable
    /// that demonstrates the complete body+shape custom-authoring surface (Phases A–D). It builds one child
    /// SubScene holding nine authored GameObjects (the five shape kinds, a material-template + per-field override
    /// body, a filtered pair, a static box floor and a static edge wall) and a parent scene carrying that SubScene,
    /// into mara's fixture folder so the bake smoke runs in place. The committed shippable copy under
    /// <c>Samples~/CustomAuthoring2D/Scenes/</c> is the same content copied out after the bake check (the
    /// <c>Samples~</c> folder is not imported, so it cannot be baked in place).
    ///
    /// Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.CustomAuthoring2DSampleSceneBuilder.BuildSampleScene</c>.
    /// </summary>
    public static class CustomAuthoring2DSampleSceneBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        public const string SampleParent = FixtureRoot + "/CustomAuthoring2DSample.unity";
        public const string SampleChild = FixtureRoot + "/CustomAuthoring2DSample_Sub.unity";
        public const string SampleMaterial = FixtureRoot + "/BouncyTemplate.physicsMaterial2D";

        const float FloorTop = 0f;
        const int FilterCategory = 8; // an arbitrary explicit category bit for the filtered pair

        [MenuItem("Tools/Zori/Build CustomAuthoring2D Sample Scene")]
        public static void BuildSampleScene()
        {
            Directory.CreateDirectory(FixtureRoot);

            // The material template MaterialBody inherits its bounciness from.
            var template = new PhysicsMaterial2D("BouncyTemplate") { friction = 0.3f, bounciness = 0.8f };
            AssetDatabase.CreateAsset(template, SampleMaterial);

            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A static box floor (no PhysicsBody2DAuthoring → the shape baker's static fallback). Radius 0 so the
            // box has a sharp top edge (a non-zero corner radius would raise the effective floor surface).
            var floor = MakeShape("Floor", new Vector3(0f, FloorTop, 0f));
            floor.Kind = PhysicsShape2DKind.Box;
            floor.BoxSize = new float2(40f, 1f);
            floor.Radius = 0f;

            // The five shape kinds as dynamic bodies that fall and settle on the floor. (Render-rate
            // Interpolation is a documented authoring field but is intentionally NOT set on a scene body: it is
            // invisible in a still settling scene and, loaded into the persistent default test world via a
            // SubScene, its PhysicsBody2DSmoothing entities leak the smoothing job across tests. It is exercised
            // in isolation by Phase8InterpCcdJointBreakGate and documented in custom-authoring.md.)
            var circle = MakeBody("CircleBody", new Vector3(-9f, 6f, 0f), out var circleShape);
            circleShape.Kind = PhysicsShape2DKind.Circle;
            circleShape.Radius = 0.5f;

            var box = MakeBody("BoxBody", new Vector3(-6f, 6f, 0f), out var boxShape);
            boxShape.Kind = PhysicsShape2DKind.Box;
            boxShape.BoxSize = new float2(1f, 1f);
            boxShape.BoxAngle = 20f; // the free box orientation a built-in BoxCollider2D cannot author

            var capsule = MakeBody("CapsuleBody", new Vector3(-3f, 6f, 0f), out var capsuleShape);
            capsuleShape.Kind = PhysicsShape2DKind.Capsule;
            capsuleShape.CapsuleSize = new float2(0.8f, 1.6f);
            capsuleShape.CapsuleVertical = true;
            capsuleShape.CapsuleAngle = 15f; // the free capsule orientation

            var polygon = MakeBody("PolygonBody", new Vector3(0f, 6f, 0f), out var polygonShape);
            polygonShape.Kind = PhysicsShape2DKind.Polygon;
            polygonShape.Vertices = new[]
            {
                new Vector2(-0.5f, -0.5f),
                new Vector2(0.5f, -0.5f),
                new Vector2(0.7f, 0.2f),
                new Vector2(0f, 0.7f),
                new Vector2(-0.7f, 0.2f),
            }; // a 5-vertex convex hull — the single-hull (PolygonDecompose off) path
            polygonShape.PolygonDecompose = false;

            // A static edge/chain wall (the 2D analogue of the 3D "plane"; an open chain, no enclosing body).
            var edge = MakeShape("EdgeWall", new Vector3(12f, 1f, 0f));
            edge.Kind = PhysicsShape2DKind.Edge;
            edge.EdgeIsLoop = false;
            edge.Vertices = new[] { new Vector2(-2f, 0f), new Vector2(0f, 1.5f), new Vector2(2f, 0f) };

            // A body driven by a PhysicsMaterial2D template (bounciness inherited) WITH a per-field override
            // (friction overridden inline) — the Phase-B inheritance + override model.
            var material = MakeBody("MaterialBody", new Vector3(3f, 7f, 0f), out var materialShape);
            materialShape.Kind = PhysicsShape2DKind.Box;
            materialShape.BoxSize = new float2(1f, 1f);
            materialShape.MaterialTemplate = template; // inherits bounciness 0.8
            materialShape.OverrideFriction = true;
            materialShape.Friction = 0.1f; // overrides the template's friction 0.3 with a slippery 0.1

            // A filtered pair: two circles that share an explicit category bit and collide only with that bit, so
            // they collide with each other but not with the default-filter bodies. OverrideFilterBits bypasses the
            // project layer-collision matrix entirely, so the demonstration is project-independent.
            var filterA = MakeBody("FilteredBodyA", new Vector3(7f, 6f, 0f), out var filterAShape);
            ConfigureFilteredCircle(filterAShape);
            var filterB = MakeBody("FilteredBodyB", new Vector3(7f, 9f, 0f), out var filterBShape);
            ConfigureFilteredCircle(filterBShape);

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, SampleChild);

            SaveParentWithSubScene(SampleParent, SampleChild, "CustomAuthoring2D Sample SubScene");

            RegisterSceneInBuildSettings(SampleParent);
            RegisterSceneInBuildSettings(SampleChild);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                "[CustomAuthoring2DSampleSceneBuilder] built the sample scene "
                    + "(9 GameObjects: 5 shape kinds + material-template/override + filtered pair + floor + edge)."
            );
        }

        static void ConfigureFilteredCircle(PhysicsShape2DAuthoring shape)
        {
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = 0.4f;
            shape.OverrideFilterBits = true;
            shape.CategoryBits = 1 << FilterCategory;
            shape.ContactBits = 1 << FilterCategory;
        }

        static PhysicsBody2DAuthoring MakeBody(string name, Vector3 position, out PhysicsShape2DAuthoring shape)
        {
            var go = new GameObject(name);
            go.transform.position = position;
            var body = go.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.GravityScale = 1f;
            body.UseAutoMass = true; // density-derived mass — no per-body mass tuning
            shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Density = 1f;
            return body;
        }

        static PhysicsShape2DAuthoring MakeShape(string name, Vector3 position)
        {
            var go = new GameObject(name);
            go.transform.position = position;
            return go.AddComponent<PhysicsShape2DAuthoring>();
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
