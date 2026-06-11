using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-B adversarial BAKE gate for the material-TEMPLATE inheritance model
    /// (<c>PhysicsShape2DAuthoringBaker.ResolveSurface</c>: <c>override &gt; template &gt; inline-default</c>).
    /// It loads a SubScene baked by <c>MaterialTemplateFixtureBuilder</c> against a real
    /// <see cref="PhysicsMaterial2D"/> asset and pins each arm of the precedence on the baked
    /// <c>PhysicsShape2D</c> surface — through the REAL baker, against the REAL asset, not a hand-written shape.
    /// </summary>
    /// <remarks>
    /// <para><b>What each body pins.</b> Four collider-only bodies, matched by their baked
    /// <c>initialPosition.x</c> (the runtime Tests asmdef cannot reference the Editor-platform builder, so the X
    /// keys and expected values are duplicated as constants — the package's established pattern, see
    /// <see cref="FilterBakeParityGate"/>):</para>
    /// <list type="bullet">
    /// <item><b>TemplateCustom</b> — the TEMPLATE arm: a custom shape inheriting the asset with no override bakes
    /// the asset's friction / bounciness / combine. Asserted equal to the asset values AND bit-identical to
    /// TemplateBuiltIn (convergence: <c>ResolveSurface</c> ≡ <c>ReadSurface</c> for the same material).</item>
    /// <item><b>TemplateBuiltIn</b> — the convergence ORACLE: a built-in collider with the same
    /// <c>sharedMaterial</c>, baked via the trusted <c>ReadSurface</c>.</item>
    /// <item><b>OverrideCustom</b> — the OVERRIDE arm: friction + bounciness-combine overridden inline beat the
    /// template; the un-overridden bounciness + friction-combine still inherit the template (per-field override).</item>
    /// <item><b>DefaultCustom</b> — the DEFAULT arm: no template, no override bakes the inline Phase-A defaults
    /// (friction 0.4, bounciness 0, both combines Average) — the pre-Phase-B bake, unchanged.</item>
    /// </list>
    /// <para><b>The <c>DependsOn(MaterialTemplate)</c> from-scratch read.</b> The asset's values are distinct from
    /// the inline defaults, so observing them in the baked TemplateCustom / OverrideCustom surface proves the
    /// importer actually READ the referenced asset at bake (the value follows the asset, not the inline default).
    /// An incremental re-bake on a LIVE asset edit is not reachable in-process under batchmode (the SubScene
    /// importer keys its cached bake on the scene asset and re-bakes only on an
    /// <c>AssetDatabase.GlobalArtifactDependencyVersion</c> bump driven by the asset-import pipeline; the
    /// in-process <c>BakingUtility.BakeGameObjects</c> is <c>internal</c> to <c>Unity.Entities.Hybrid</c> and
    /// visible only to that package's own test assemblies), so the incremental arm is documented as
    /// harness-limited in <c>03-phaseB-material-filter.md</c>; this gate pins the from-scratch asset read, which
    /// is the dependency's observable effect.</para>
    /// </remarks>
    public sealed class MaterialTemplateBakeGate
    {
        const int LoadTimeoutFrames = 600;
        const string ParentScenePath = "Assets/EntitiesPhysics2DFixture/MaterialTemplate.unity";

        // Mirror of MaterialTemplateFixtureBuilder constants (the runtime asmdef cannot reference the Editor
        // builder; duplicating the load-bearing constants is the package's pattern, FilterBakeParityGate).
        const float XTemplateCustom = -6f;
        const float XTemplateBuiltIn = -4f;
        const float XOverrideCustom = -2f;
        const float XDefaultCustom = 0f;

        const float TemplateFriction = 0.123f;
        const float TemplateBounciness = 0.456f;

        // PhysicsMaterialCombine2D.Maximum maps (by-name, Collider2DBaking.MapCombine) to Mixing Maximum.
        const PhysicsSurfaceMixing2D TemplateFrictionMixing = PhysicsSurfaceMixing2D.Maximum;

        // PhysicsMaterialCombine2D.Minimum maps to Mixing Minimum.
        const PhysicsSurfaceMixing2D TemplateBounceMixing = PhysicsSurfaceMixing2D.Minimum;

        const float OverrideFriction = 0.777f;
        const PhysicsSurfaceMixing2D OverrideBounceMixing = PhysicsSurfaceMixing2D.Multiply;

        const float DefaultFriction = 0.4f;
        const float DefaultBounciness = 0f;
        const PhysicsSurfaceMixing2D DefaultMixing = PhysicsSurfaceMixing2D.Average;

        const float Eps = 1e-5f;

        [UnityTest]
        public IEnumerator MaterialTemplate_Override_Default_ResolvePrecedence_AndConvergeWithBuiltIn()
        {
            SceneManager.LoadScene(ParentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");
            var em = world.EntityManager;

            var query = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );

            var framesWaited = 0;
            while (query.CalculateEntityCount() < 4 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            var count = query.CalculateEntityCount();
            Assert.GreaterOrEqual(
                count,
                4,
                $"Only {count} baked bodies appeared after {framesWaited} frames — build the fixture first via "
                    + "-executeMethod Zori.Entities.Physics2D.Tests.Editor.MaterialTemplateFixtureBuilder.Build."
            );

            using var shapes = query.ToComponentDataArray<PhysicsShape2D>(Allocator.Temp);
            using var defs = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);

            var haveTemplateCustom = false;
            var haveTemplateBuiltIn = false;
            var haveOverrideCustom = false;
            var haveDefaultCustom = false;
            PhysicsShape2D templateCustom = default;
            PhysicsShape2D templateBuiltIn = default;
            PhysicsShape2D overrideCustom = default;
            PhysicsShape2D defaultCustom = default;

            for (var i = 0; i < shapes.Length; i++)
            {
                var x = defs[i].initialPosition.x;
                if (abs(x - XTemplateCustom) < 0.25f)
                {
                    templateCustom = shapes[i];
                    haveTemplateCustom = true;
                }
                else if (abs(x - XTemplateBuiltIn) < 0.25f)
                {
                    templateBuiltIn = shapes[i];
                    haveTemplateBuiltIn = true;
                }
                else if (abs(x - XOverrideCustom) < 0.25f)
                {
                    overrideCustom = shapes[i];
                    haveOverrideCustom = true;
                }
                else if (abs(x - XDefaultCustom) < 0.25f)
                {
                    defaultCustom = shapes[i];
                    haveDefaultCustom = true;
                }
            }

            Assert.IsTrue(
                haveTemplateCustom
                    && haveTemplateBuiltIn
                    && haveOverrideCustom
                    && haveDefaultCustom,
                $"Missing one of the four baked bodies (templateCustom={haveTemplateCustom}, "
                    + $"templateBuiltIn={haveTemplateBuiltIn}, overrideCustom={haveOverrideCustom}, "
                    + $"defaultCustom={haveDefaultCustom})."
            );

            Debug.Log(
                "[PHYSICS2D-PHASEB-TEMPLATE] "
                    + $"templateCustom(f={templateCustom.friction:F4} b={templateCustom.bounciness:F4} "
                    + $"fMix={templateCustom.frictionMixing} bMix={templateCustom.bouncinessMixing}) | "
                    + $"templateBuiltIn(f={templateBuiltIn.friction:F4} b={templateBuiltIn.bounciness:F4} "
                    + $"fMix={templateBuiltIn.frictionMixing} bMix={templateBuiltIn.bouncinessMixing}) | "
                    + $"overrideCustom(f={overrideCustom.friction:F4} b={overrideCustom.bounciness:F4} "
                    + $"fMix={overrideCustom.frictionMixing} bMix={overrideCustom.bouncinessMixing}) | "
                    + $"defaultCustom(f={defaultCustom.friction:F4} b={defaultCustom.bounciness:F4} "
                    + $"fMix={defaultCustom.frictionMixing} bMix={defaultCustom.bouncinessMixing})"
            );

            // ---- TEMPLATE arm: a non-overriding custom shape bakes the asset's values. ----
            Assert.AreEqual(
                TemplateFriction,
                templateCustom.friction,
                Eps,
                "TemplateCustom did not inherit the template friction — ResolveSurface ignored the assigned "
                    + "PhysicsMaterial2D (fell to the inline default 0.4 instead of the asset's 0.123). "
                    + "Either the template arm is broken or DependsOn did not read the asset at bake."
            );
            Assert.AreEqual(
                TemplateBounciness,
                templateCustom.bounciness,
                Eps,
                "TemplateCustom did not inherit the template bounciness — fell to the inline default 0 instead "
                    + "of the asset's 0.456."
            );
            Assert.AreEqual(
                TemplateFrictionMixing,
                templateCustom.frictionMixing,
                "TemplateCustom did not inherit the template frictionCombine (Maximum) — MapCombine/ResolveSurface "
                    + "did not route the template's combine through the friction-mixing arm."
            );
            Assert.AreEqual(
                TemplateBounceMixing,
                templateCustom.bouncinessMixing,
                "TemplateCustom did not inherit the template bounceCombine (Minimum)."
            );

            // ---- CONVERGENCE: the template-driven custom shape bakes bit-identical to the built-in oracle. ----
            Assert.AreEqual(
                templateBuiltIn.friction,
                templateCustom.friction,
                0f,
                $"Convergence broke on friction: custom template-driven {templateCustom.friction} != built-in "
                    + $"sharedMaterial-driven {templateBuiltIn.friction}. ResolveSurface and ReadSurface diverge "
                    + "for the same PhysicsMaterial2D."
            );
            Assert.AreEqual(
                templateBuiltIn.bounciness,
                templateCustom.bounciness,
                0f,
                $"Convergence broke on bounciness: custom {templateCustom.bounciness} != built-in "
                    + $"{templateBuiltIn.bounciness}."
            );
            Assert.AreEqual(
                templateBuiltIn.frictionMixing,
                templateCustom.frictionMixing,
                "Convergence broke on frictionMixing: custom != built-in for the same material's frictionCombine."
            );
            Assert.AreEqual(
                templateBuiltIn.bouncinessMixing,
                templateCustom.bouncinessMixing,
                "Convergence broke on bouncinessMixing: custom != built-in for the same material's bounceCombine."
            );

            // ---- OVERRIDE arm: overridden fields take the inline value; un-overridden fields inherit. ----
            Assert.AreEqual(
                OverrideFriction,
                overrideCustom.friction,
                Eps,
                $"OverrideCustom friction = {overrideCustom.friction}, expected the inline override "
                    + $"{OverrideFriction}. The override did NOT beat the template (got "
                    + $"{(abs(overrideCustom.friction - TemplateFriction) < Eps ? "the template value" : "neither")})."
            );
            Assert.AreEqual(
                OverrideBounceMixing,
                overrideCustom.bouncinessMixing,
                $"OverrideCustom bouncinessMixing = {overrideCustom.bouncinessMixing}, expected the inline "
                    + $"override {OverrideBounceMixing}. The combine override did not beat the template."
            );
            // The two fields left un-overridden on the override body STILL inherit the template — per-field.
            Assert.AreEqual(
                TemplateBounciness,
                overrideCustom.bounciness,
                Eps,
                "OverrideCustom bounciness should still inherit the template (it was NOT overridden) — a "
                    + "per-field override leaked into bounciness, or the whole surface fell to the default."
            );
            Assert.AreEqual(
                TemplateFrictionMixing,
                overrideCustom.frictionMixing,
                "OverrideCustom frictionMixing should still inherit the template (friction-combine was NOT "
                    + "overridden) — the friction-value override wrongly captured the friction-combine."
            );

            // ---- DEFAULT arm: no template, no override → the inline Phase-A defaults (pre-Phase-B bake). ----
            Assert.AreEqual(
                DefaultFriction,
                defaultCustom.friction,
                Eps,
                $"DefaultCustom friction = {defaultCustom.friction}, expected the inline default {DefaultFriction}."
            );
            Assert.AreEqual(
                DefaultBounciness,
                defaultCustom.bounciness,
                Eps,
                $"DefaultCustom bounciness = {defaultCustom.bounciness}, expected the inline default "
                    + $"{DefaultBounciness}."
            );
            Assert.AreEqual(
                DefaultMixing,
                defaultCustom.frictionMixing,
                "DefaultCustom frictionMixing should be the inline default Average."
            );
            Assert.AreEqual(
                DefaultMixing,
                defaultCustom.bouncinessMixing,
                "DefaultCustom bouncinessMixing should be the inline default Average."
            );

            // Disqualifier: the template value must actually DIFFER from the default, or "inherited the template"
            // is vacuously satisfied by the default. (A regression that made the template equal the default would
            // pass the template arm but fail here.)
            Assert.AreNotEqual(
                DefaultFriction,
                TemplateFriction,
                "Test self-check: the template friction must differ from the inline default, or the template arm "
                    + "proves nothing."
            );
            yield break;
        }
    }
}
