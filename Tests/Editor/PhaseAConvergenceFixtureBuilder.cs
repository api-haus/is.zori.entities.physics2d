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
    /// Click-free authoring of the Phase-A per-field CONVERGENCE fixtures: for each new authored field that has
    /// a built-in equivalent, one child SubScene authors a custom-authored body carrying the field AND an
    /// equivalent built-in-authored body that reaches the SAME world configuration through the built-in
    /// mechanism, distinct only in start X. Both bake into one ECS world and run the SAME Box2D-v3 solver, so
    /// their start-relative trajectories must agree to the tight near-exact envelope (the dual-surface
    /// convergence the standing gate proves for the circle, extended here to each new field). A field that
    /// silently fails to bake splits the two trajectories.
    /// </summary>
    /// <remarks>
    /// The built-in equivalent per field:
    /// <list type="bullet">
    /// <item><b>BoxOrient</b> — a custom box with <see cref="PhysicsShape2DAuthoring.BoxAngle"/> = 25° on an
    /// un-rotated Transform vs a built-in <c>BoxCollider2D</c> (axis-aligned) on a Transform rotated 25° about
    /// z. Both reach a box oriented 25° in world space — the custom field folds the angle into the shape
    /// geometry, the built-in carries it on the Transform.</item>
    /// <item><b>CapsuleOrient</b> — a custom vertical capsule with
    /// <see cref="PhysicsShape2DAuthoring.CapsuleAngle"/> = 25° vs a built-in vertical <c>CapsuleCollider2D</c>
    /// on a Transform rotated 25°.</item>
    /// <item><b>Interp</b> — a custom body with <see cref="PhysicsBody2DAuthoring.Interpolation"/> = Interpolate
    /// vs a built-in <c>Rigidbody2D</c> with <c>interpolation = Interpolate</c>. Both free-fall; the physics
    /// pose converges (interpolation is a render-rate overlay, inert under the fixed-step compare).</item>
    /// <item><b>FilterLayer</b> — a custom shape with <see cref="PhysicsShape2DAuthoring.Layer"/> = 8 vs a
    /// built-in collider whose <c>gameObject.layer</c> = 8, both dropped onto a shared floor on the same layer.
    /// Both resolve the identical category/contact bits, so they collide with and settle on the floor
    /// identically — a mismapped Layer would change the bits and the body would fall through.</item>
    /// </list>
    /// Each child scene also carries a custom-authored static box floor so the contact fixtures (BoxOrient,
    /// CapsuleOrient, FilterLayer) have a surface to settle on; Interp is a pure free-fall, no floor. Run via
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.PhaseAConvergenceFixtureBuilder.BuildAll</c>.
    /// </remarks>
    public static class PhaseAConvergenceFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        public const string BoxOrientParent = FixtureRoot + "/PhaseABoxOrient.unity";
        public const string BoxOrientChild = FixtureRoot + "/PhaseABoxOrient_Sub.unity";
        public const string CapsuleOrientParent = FixtureRoot + "/PhaseACapsuleOrient.unity";
        public const string CapsuleOrientChild = FixtureRoot + "/PhaseACapsuleOrient_Sub.unity";
        public const string InterpParent = FixtureRoot + "/PhaseAInterp.unity";
        public const string InterpChild = FixtureRoot + "/PhaseAInterp_Sub.unity";
        public const string FilterLayerParent = FixtureRoot + "/PhaseAFilterLayer.unity";
        public const string FilterLayerChild = FixtureRoot + "/PhaseAFilterLayer_Sub.unity";

        const float CustomX = -2f;
        const float BuiltInX = 2f;
        const float OrientAngle = 25f;
        const int FilterLayer = 8;

        [MenuItem("Tools/Zori/Build Entities Physics2D Phase-A Convergence Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildBoxOrient();
            BuildCapsuleOrient();
            BuildInterp();
            BuildFilterLayer();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                "Entities Physics2D Phase-A convergence fixtures built "
                    + "(PhaseABoxOrient, PhaseACapsuleOrient, PhaseAInterp, PhaseAFilterLayer)."
            );
        }

        // ----- BoxOrient: custom BoxAngle vs built-in Transform-rotated BoxCollider2D, both onto a floor. -----
        static void BuildBoxOrient()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            AddCustomFloor(new Vector3(0f, 0f, 0f), new Unity.Mathematics.float2(40f, 1f));

            // Custom box at X=-2, BoxAngle 25°, Transform un-rotated.
            var customGo = new GameObject("CustomBoxOrient");
            customGo.transform.position = new Vector3(CustomX, 5f, 0f);
            var cBody = customGo.AddComponent<PhysicsBody2DAuthoring>();
            cBody.BodyType = PhysicsBody2DMotionType.Dynamic;
            cBody.UseAutoMass = true;
            var cShape = customGo.AddComponent<PhysicsShape2DAuthoring>();
            cShape.Kind = PhysicsShape2DKind.Box;
            cShape.BoxSize = new Unity.Mathematics.float2(1f, 1f);
            cShape.BoxAngle = OrientAngle;
            cShape.Density = 1f;

            // Built-in box at X=+2, axis-aligned collider on a Transform rotated 25° about z.
            var builtinGo = new GameObject("BuiltInBoxOrient");
            builtinGo.transform.position = new Vector3(BuiltInX, 5f, 0f);
            builtinGo.transform.rotation = Quaternion.Euler(0f, 0f, OrientAngle);
            var rb = builtinGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            var box = builtinGo.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);
            box.density = 1f;

            SaveChildAndParent(child, BoxOrientChild, BoxOrientParent, "PhaseABoxOrient SubScene");
        }

        // ----- CapsuleOrient: custom CapsuleAngle vs built-in Transform-rotated CapsuleCollider2D. -----
        static void BuildCapsuleOrient()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            AddCustomFloor(new Vector3(0f, 0f, 0f), new Unity.Mathematics.float2(40f, 1f));

            var customGo = new GameObject("CustomCapsuleOrient");
            customGo.transform.position = new Vector3(CustomX, 5f, 0f);
            var cBody = customGo.AddComponent<PhysicsBody2DAuthoring>();
            cBody.BodyType = PhysicsBody2DMotionType.Dynamic;
            cBody.UseAutoMass = true;
            var cShape = customGo.AddComponent<PhysicsShape2DAuthoring>();
            cShape.Kind = PhysicsShape2DKind.Capsule;
            cShape.CapsuleSize = new Unity.Mathematics.float2(1f, 2f);
            cShape.CapsuleVertical = true;
            cShape.CapsuleAngle = OrientAngle;
            cShape.Density = 1f;

            var builtinGo = new GameObject("BuiltInCapsuleOrient");
            builtinGo.transform.position = new Vector3(BuiltInX, 5f, 0f);
            builtinGo.transform.rotation = Quaternion.Euler(0f, 0f, OrientAngle);
            var rb = builtinGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            var cap = builtinGo.AddComponent<CapsuleCollider2D>();
            cap.size = new Vector2(1f, 2f);
            cap.direction = CapsuleDirection2D.Vertical;
            cap.density = 1f;

            SaveChildAndParent(child, CapsuleOrientChild, CapsuleOrientParent, "PhaseACapsuleOrient SubScene");
        }

        // ----- Interp: custom Interpolation vs built-in Rigidbody2D.interpolation, pure free-fall. -----
        static void BuildInterp()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var customGo = new GameObject("CustomInterp");
            customGo.transform.position = new Vector3(CustomX, 10f, 0f);
            var cBody = customGo.AddComponent<PhysicsBody2DAuthoring>();
            cBody.BodyType = PhysicsBody2DMotionType.Dynamic;
            cBody.UseAutoMass = true;
            cBody.Interpolation = PhysicsBody2DInterpolation.Interpolate;
            var cShape = customGo.AddComponent<PhysicsShape2DAuthoring>();
            cShape.Kind = PhysicsShape2DKind.Circle;
            cShape.Radius = 0.5f;
            cShape.Density = 1f;

            var builtinGo = new GameObject("BuiltInInterp");
            builtinGo.transform.position = new Vector3(BuiltInX, 10f, 0f);
            var rb = builtinGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            rb.interpolation = RigidbodyInterpolation2D.Interpolate;
            var circle = builtinGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            circle.density = 1f;

            SaveChildAndParent(child, InterpChild, InterpParent, "PhaseAInterp SubScene");
        }

        // ----- FilterLayer: custom Layer vs built-in gameObject.layer, both onto a same-layer floor. -----
        static void BuildFilterLayer()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A custom-authored static floor on the filter layer so the layer-collision matrix governs the
            // contact for both bodies; the floor sits under both drop columns.
            var floor = new GameObject("CustomFilterFloor") { layer = FilterLayer };
            floor.transform.position = new Vector3(0f, 0f, 0f);
            var floorShape = floor.AddComponent<PhysicsShape2DAuthoring>();
            floorShape.Kind = PhysicsShape2DKind.Box;
            floorShape.BoxSize = new Unity.Mathematics.float2(40f, 1f);
            floorShape.Layer = FilterLayer;

            // Custom shape with Layer = 8 at X=-2.
            var customGo = new GameObject("CustomFilterBody") { layer = FilterLayer };
            customGo.transform.position = new Vector3(CustomX, 5f, 0f);
            var cBody = customGo.AddComponent<PhysicsBody2DAuthoring>();
            cBody.BodyType = PhysicsBody2DMotionType.Dynamic;
            cBody.UseAutoMass = true;
            var cShape = customGo.AddComponent<PhysicsShape2DAuthoring>();
            cShape.Kind = PhysicsShape2DKind.Circle;
            cShape.Radius = 0.5f;
            cShape.Density = 1f;
            cShape.Layer = FilterLayer;

            // Built-in collider whose gameObject.layer = 8 at X=+2.
            var builtinGo = new GameObject("BuiltInFilterBody") { layer = FilterLayer };
            builtinGo.transform.position = new Vector3(BuiltInX, 5f, 0f);
            var rb = builtinGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            var circle = builtinGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            circle.density = 1f;

            SaveChildAndParent(child, FilterLayerChild, FilterLayerParent, "PhaseAFilterLayer SubScene");
        }

        // A custom-authored static box floor in the CURRENT open child scene (no PhysicsBody2DAuthoring →
        // collider-only static body via the shape baker's static fallback).
        static void AddCustomFloor(Vector3 pos, Unity.Mathematics.float2 size)
        {
            var floor = new GameObject("CustomFloor");
            floor.transform.position = pos;
            var shape = floor.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = size;
        }

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
            var subSceneGo = new GameObject(subSceneName);
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
