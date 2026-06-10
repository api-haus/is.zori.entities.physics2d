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
    /// Click-free authoring of the Phase-11 step-config BAKE fixtures: SubScenes carrying a REAL
    /// <see cref="PhysicsStep2DAuthoring"/> on a GameObject, plus a built-in <see cref="Rigidbody2D"/> +
    /// <see cref="CircleCollider2D"/> faller, baked through the actual <c>PhysicsStep2DAuthoringBaker</c> and the
    /// collider/body bakers. The validating gate (<c>Phase11StepConfigGate</c>) loads these and proves the
    /// AUTHORED config values reach the world (read back off the live <c>PhysicsWorld</c> and the baked
    /// <c>PhysicsWorld2DConfig</c> singleton) and change behaviour — the witness the Phase-11 smoke, which
    /// CONSTRUCTS the singleton in code (<c>EntityManager.CreateSingleton</c>), never produced.
    /// </summary>
    /// <remarks>
    /// <para><b>Why these are REAL components, not a code-constructed singleton.</b> The smoke
    /// (<c>StepConfigSmoke</c>) proves the runtime READ of a <c>PhysicsWorld2DConfig</c> it builds directly; it
    /// never runs <c>PhysicsStep2DAuthoringBaker.Bake</c> against an authored <c>PhysicsStep2DAuthoring</c>. A
    /// baker that silently dropped a field (e.g. forgot to copy <c>gravity</c> into <c>AsConfig</c>, or used
    /// <c>TransformUsageFlags</c> that suppressed the component) would pass the code-constructed smoke and fail a
    /// real bake. Each fixture is a child SubScene (the authored components) + a parent scene carrying a
    /// <c>SubScene</c> that auto-loads and bakes it on PlayMode enter, both registered in build settings — the
    /// Phase-9 / FilterBake fixture-builder pattern.</para>
    ///
    /// <para><b>The faller is a built-in <c>Rigidbody2D</c>+<c>CircleCollider2D</c>, not a direct-authored
    /// entity.</b> The gate's GameObject-parity oracle instantiates the SAME authored faller live and steps it
    /// with <c>Physics2D.Simulate</c> under a matched <c>Physics2D.gravity</c>, so the faller must be a built-in
    /// authored body the reference side can collect. The config GRAVITY is authored into the
    /// <c>PhysicsStep2DAuthoring</c> component itself (a serialized field), so it bakes from the SubScene with no
    /// dependence on project-settings state (unlike the layer matrix, which the FilterBake doc notes is read
    /// from persisted project settings at import — gravity has no such confound).</para>
    ///
    /// <para>Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.StepConfigFixtureBuilder.BuildAll</c> before the
    /// PlayMode gate.</para>
    /// </remarks>
    public static class StepConfigFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        // --- Configured gravity: a NON-default gravity (a horizontal drift + a weaker-than-default fall). ---
        public const string ConfiguredGravityParent = FixtureRoot + "/P11ConfiguredGravity.unity";
        public const string ConfiguredGravityChild = FixtureRoot + "/P11ConfiguredGravity_Sub.unity";

        // --- Default fallback: NO PhysicsStep2DAuthoring at all (the backward-compat path). ---
        public const string DefaultFallbackParent = FixtureRoot + "/P11DefaultFallback.unity";
        public const string DefaultFallbackChild = FixtureRoot + "/P11DefaultFallback_Sub.unity";

        // --- Substeps: author simulationSubSteps at a distinct value (the read-back witness). ---
        public const string SubstepsParent = FixtureRoot + "/P11Substeps.unity";
        public const string SubstepsChild = FixtureRoot + "/P11Substeps_Sub.unity";

        // --- More fields: sleepingAllowed=false + maximumLinearSpeed + bounceThreshold + contactSpeed. ---
        public const string MoreFieldsParent = FixtureRoot + "/P11MoreFields.unity";
        public const string MoreFieldsChild = FixtureRoot + "/P11MoreFields_Sub.unity";

        // --- Multiplicity: TWO PhysicsStep2DAuthoring on two GameObjects in one SubScene. ---
        public const string MultiplicityParent = FixtureRoot + "/P11Multiplicity.unity";
        public const string MultiplicityChild = FixtureRoot + "/P11Multiplicity_Sub.unity";

        // The authored NON-default gravity: a horizontal drift (5,0) plus a weaker-than-default fall (0,-20 is
        // STRONGER, so pick (5,-20) — distinct in BOTH axes from the (0,-9.81) default so a dropped-field baker
        // that left gravity at default is caught on either component). The gate matches Physics2D.gravity to
        // this for the GameObject-parity oracle.
        public static readonly float2 ConfiguredGravity = new float2(5f, -20f);

        // The faller start: high enough that a body drifting/falling under the configured gravity travels well
        // within the scene without contacting anything (these fixtures author no floor — the faller free-falls).
        public static readonly Vector2 FallerStart = new Vector2(0f, 50f);
        public const float FallerRadius = 0.5f;

        // Authored distinct simulationSubSteps (default is 4) — read back off the live world. A distinct value
        // 1 (the minimum the OnValidate clamp allows) is unambiguously not the default.
        public const int SubstepsValue = 1;

        // Authored distinct non-default values for the "more fields" fixture, each read back off the live world.
        public const bool SleepingAllowedValue = false; // default true
        public const float MaximumLinearSpeedValue = 7.5f; // default 400
        public const float BounceThresholdValue = 13.5f; // default 1
        public const float ContactSpeedValue = 9.25f; // default 3

        [MenuItem("Tools/Zori/Build Entities Physics2D Phase-11 Step-Config Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildConfiguredGravity();
            BuildDefaultFallback();
            BuildSubsteps();
            BuildMoreFields();
            BuildMultiplicity();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                "Entities Physics2D Phase-11 step-config fixtures built (configured gravity, default fallback, "
                    + "substeps, more fields, multiplicity)."
            );
        }

        // ---------------------------------------------------------------------------------------------------

        // A dynamic faller authored as a built-in Rigidbody2D + CircleCollider2D, so the GameObject-parity oracle
        // can instantiate the SAME body live. gravityScale = 1 so the world gravity is the sole driver.
        static GameObject MakeFaller(UnityEngine.SceneManagement.Scene scene, string name, Vector2 pos)
        {
            var go = new GameObject(name);
            go.transform.position = pos;
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = go.AddComponent<CircleCollider2D>();
            circle.radius = FallerRadius;
            UnityEditor.SceneManagement.EditorSceneManager.MoveGameObjectToScene(go, scene);
            return go;
        }

        // A GameObject carrying a real PhysicsStep2DAuthoring, configured via the public setters so the baked
        // PhysicsWorld2DConfig carries exactly these values. mutate is the per-fixture knob.
        static GameObject MakeStepConfig(
            UnityEngine.SceneManagement.Scene scene,
            string name,
            System.Action<PhysicsStep2DAuthoring> mutate
        )
        {
            var go = new GameObject(name);
            var step = go.AddComponent<PhysicsStep2DAuthoring>();
            mutate(step);
            UnityEditor.SceneManagement.EditorSceneManager.MoveGameObjectToScene(go, scene);
            return go;
        }

        static void BuildConfiguredGravity()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            MakeStepConfig(child, "StepConfig", s => s.Gravity = ConfiguredGravity);
            MakeFaller(child, "Faller", FallerStart);
            SaveChildAndParent(child, ConfiguredGravityChild, ConfiguredGravityParent, "P11ConfiguredGravity");
        }

        static void BuildDefaultFallback()
        {
            // NO PhysicsStep2DAuthoring — the backward-compat path. Only the faller. The world must use the
            // Box2D defaultDefinition (g = -9.81), unchanged from before the config surface existed.
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            MakeFaller(child, "Faller", FallerStart);
            SaveChildAndParent(child, DefaultFallbackChild, DefaultFallbackParent, "P11DefaultFallback");
        }

        static void BuildSubsteps()
        {
            // A config that differs from the default ONLY in simulationSubSteps (gravity stays the default so the
            // fall is the familiar -9.81 and the read-back of substeps is the isolated witness).
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            MakeStepConfig(child, "StepConfig", s => s.SimulationSubSteps = SubstepsValue);
            MakeFaller(child, "Faller", FallerStart);
            SaveChildAndParent(child, SubstepsChild, SubstepsParent, "P11Substeps");
        }

        static void BuildMoreFields()
        {
            // Four more fields at non-default values, each read back off the live world: sleepingAllowed (bool),
            // maximumLinearSpeed (float), bounceThreshold (float), contactSpeed (float). Gravity stays default.
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            MakeStepConfig(
                child,
                "StepConfig",
                s =>
                {
                    s.SleepingAllowed = SleepingAllowedValue;
                    s.MaximumLinearSpeed = MaximumLinearSpeedValue;
                    s.BounceThreshold = BounceThresholdValue;
                    s.ContactSpeed = ContactSpeedValue;
                }
            );
            MakeFaller(child, "Faller", FallerStart);
            SaveChildAndParent(child, MoreFieldsChild, MoreFieldsParent, "P11MoreFields");
        }

        static void BuildMultiplicity()
        {
            // TWO PhysicsStep2DAuthoring on TWO GameObjects in one SubScene. [DisallowMultipleComponent] only
            // prevents two on ONE GameObject; this is the residual case the design's TryGetSingleton throw
            // catches at world creation. The gate asserts the documented behaviour (a singleton-multiplicity
            // throw), pinning that two-config is not silently last-wins-without-notice.
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            MakeStepConfig(child, "StepConfigA", s => s.Gravity = new float2(0f, -3f));
            MakeStepConfig(child, "StepConfigB", s => s.Gravity = new float2(0f, -7f));
            MakeFaller(child, "Faller", FallerStart);
            SaveChildAndParent(child, MultiplicityChild, MultiplicityParent, "P11Multiplicity");
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
