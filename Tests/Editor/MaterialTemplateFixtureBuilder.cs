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
    /// Click-free authoring of the Phase-B material-TEMPLATE bake fixture: one child SubScene holding four
    /// collider-only static bodies that exercise every arm of the shape baker's
    /// <c>ResolveSurface</c> precedence (<c>override &gt; template &gt; inline-default</c>) through a REAL bake
    /// against a REAL <see cref="PhysicsMaterial2D"/> asset, plus the built-in convergence oracle. The
    /// Phase-B gate (<c>MaterialTemplateBakeGate</c>) loads it and asserts each baked
    /// <c>PhysicsShape2D</c>'s surface (friction / bounciness / frictionMixing / bouncinessMixing) against the
    /// branch the body authored.
    /// </summary>
    /// <remarks>
    /// The four bodies (identified at test time by their distinct authored X, mirrored as a constant in the
    /// gate because the runtime Tests asmdef cannot reference this Editor-platform builder):
    /// <list type="bullet">
    /// <item><b>TemplateCustom</b> (X=-6) — a custom <see cref="PhysicsShape2DAuthoring"/> whose
    /// <see cref="PhysicsShape2DAuthoring.MaterialTemplate"/> is the asset and NO override flag is set. Its
    /// baked surface must equal the asset's friction / bounciness / combine (the TEMPLATE arm), and must be
    /// bit-identical to the built-in oracle below (the convergence the dual surface relies on, extended to a
    /// template-driven shape).</item>
    /// <item><b>TemplateBuiltIn</b> (X=-4) — a built-in <c>BoxCollider2D</c> whose <c>sharedMaterial</c> is the
    /// SAME asset. It bakes through <c>Collider2DBaking.ReadSurface</c> (the trusted oracle), so its baked
    /// surface is the convergence target for TemplateCustom: if <c>ResolveSurface</c> mis-resolved the template
    /// the two would split.</item>
    /// <item><b>OverrideCustom</b> (X=-2) — a custom shape WITH the template assigned but
    /// <see cref="PhysicsShape2DAuthoring.OverrideFriction"/> on (inline <see cref="PhysicsShape2DAuthoring.Friction"/>
    /// = a distinct value) and <see cref="PhysicsShape2DAuthoring.OverrideBouncinessCombine"/> on (inline combine
    /// = a distinct mode). Its friction + bouncinessMixing must be the INLINE override values (override beats
    /// template), while its un-overridden bounciness + frictionMixing still inherit the template (the override is
    /// per-field, not all-or-nothing).</item>
    /// <item><b>DefaultCustom</b> (X=0) — a custom shape with NO template and no override. Its surface must be the
    /// inline defaults (friction 0.4, bounciness 0, both combines Average) — the pre-Phase-B bake, unchanged.</item>
    /// </list>
    /// The asset's values are deliberately distinct from the inline defaults (friction 0.123 != 0.4, bounciness
    /// 0.456 != 0, frictionCombine Maximum != Average, bounceCombine Minimum != Average), so a baker that ignored
    /// the template and fell to the inline default would bake a DIFFERENT number and trip the gate. The asset is
    /// persisted (<c>AssetDatabase.CreateAsset</c>) so the SubScene reference is a real persistent-asset
    /// dependency — the same reference the baker's <c>DependsOn(MaterialTemplate)</c> registers, and the
    /// from-scratch read the importer performs against it. Run via
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.MaterialTemplateFixtureBuilder.Build</c>.
    /// </remarks>
    public static class MaterialTemplateFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";
        public const string ParentScenePath = FixtureRoot + "/MaterialTemplate.unity";
        public const string ChildScenePath = FixtureRoot + "/MaterialTemplate_Sub.unity";
        public const string MaterialAssetPath = FixtureRoot + "/PhaseBTemplate.physicsMaterial2D";

        // The template asset's distinctive values — each clearly distinct from the inline default so an
        // inheriting body bakes a number the default path never would.
        public const float TemplateFriction = 0.123f;
        public const float TemplateBounciness = 0.456f;
        public const PhysicsMaterialCombine2D TemplateFrictionCombine = PhysicsMaterialCombine2D.Maximum;
        public const PhysicsMaterialCombine2D TemplateBounceCombine = PhysicsMaterialCombine2D.Minimum;

        // The OverrideCustom body's inline override values — distinct from BOTH the template and the default, so
        // observing them at bake proves the override won (not the template, not the default).
        public const float OverrideFriction = 0.777f;
        public const PhysicsSurfaceMixing2D OverrideBouncinessCombine = PhysicsSurfaceMixing2D.Multiply;

        // The inline defaults the DefaultCustom (no-template) body must bake — the pre-Phase-B values.
        public const float DefaultFriction = 0.4f;
        public const float DefaultBounciness = 0f;

        // Authored X per body — the gate maps a baked entity to its branch by its baked initialPosition.x.
        public const float XTemplateCustom = -6f;
        public const float XTemplateBuiltIn = -4f;
        public const float XOverrideCustom = -2f;
        public const float XDefaultCustom = 0f;

        [MenuItem("Tools/Zori/Build Entities Physics2D Material-Template Fixture")]
        public static void Build()
        {
            Directory.CreateDirectory(FixtureRoot);

            // Create + persist the template material asset FIRST, so the scene references a real persistent
            // asset (the dependency the baker takes via DependsOn, and the import-time read source).
            var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial2D>(MaterialAssetPath);
            if (material == null)
            {
                material = new PhysicsMaterial2D("PhaseBTemplate");
                AssetDatabase.CreateAsset(material, MaterialAssetPath);
            }
            material.friction = TemplateFriction;
            material.bounciness = TemplateBounciness;
            material.frictionCombine = TemplateFrictionCombine;
            material.bounceCombine = TemplateBounceCombine;
            EditorUtility.SetDirty(material);
            AssetDatabase.SaveAssetIfDirty(material);

            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            BuildTemplateCustom(material);
            BuildTemplateBuiltIn(material);
            BuildOverrideCustom(material);
            BuildDefaultCustom();

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, ChildScenePath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject("MaterialTemplate SubScene");
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
                "Entities Physics2D material-template fixture built (TemplateCustom, TemplateBuiltIn, "
                    + $"OverrideCustom, DefaultCustom). Material asset: {MaterialAssetPath}. "
                    + $"PlayMode-load: {ParentScenePath}"
            );
        }

        // TemplateCustom: inherit every coefficient from the template (no override). A collider-only static body
        // (no PhysicsBody2DAuthoring → the shape baker's static fallback), since the gate inspects the baked
        // surface, not a simulation.
        static void BuildTemplateCustom(PhysicsMaterial2D material)
        {
            var go = new GameObject("TemplateCustom");
            go.transform.position = new Vector3(XTemplateCustom, 0f, 0f);
            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = new Unity.Mathematics.float2(1f, 1f);
            shape.MaterialTemplate = material;
            shape.Density = 1f;
        }

        // TemplateBuiltIn: the convergence oracle. A built-in BoxCollider2D carrying the SAME material as
        // sharedMaterial bakes through Collider2DBaking.ReadSurface — the trusted path TemplateCustom must match.
        static void BuildTemplateBuiltIn(PhysicsMaterial2D material)
        {
            var go = new GameObject("TemplateBuiltIn");
            go.transform.position = new Vector3(XTemplateBuiltIn, 0f, 0f);
            var box = go.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);
            box.density = 1f;
            box.sharedMaterial = material;
        }

        // OverrideCustom: template assigned, but friction overridden inline AND bounciness-combine overridden
        // inline. The two overridden fields take the inline value; the two un-overridden fields still inherit the
        // template — proving the override is per-field.
        static void BuildOverrideCustom(PhysicsMaterial2D material)
        {
            var go = new GameObject("OverrideCustom");
            go.transform.position = new Vector3(XOverrideCustom, 0f, 0f);
            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = new Unity.Mathematics.float2(1f, 1f);
            shape.MaterialTemplate = material;
            shape.Density = 1f;

            shape.OverrideFriction = true;
            shape.Friction = OverrideFriction;
            shape.OverrideBouncinessCombine = true;
            shape.BouncinessCombine = OverrideBouncinessCombine;
            // friction-combine and bounciness left un-overridden → they inherit the template.
        }

        // DefaultCustom: no template, no override → the inline Phase-A defaults (the pre-Phase-B bake).
        static void BuildDefaultCustom()
        {
            var go = new GameObject("DefaultCustom");
            go.transform.position = new Vector3(XDefaultCustom, 0f, 0f);
            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = new Unity.Mathematics.float2(1f, 1f);
            shape.Density = 1f;
            // MaterialTemplate left null; all Override flags left false → inline defaults bake.
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
