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

        // The four baked shapes, discovered once per test by LoadAndDiscoverShapes from the SubScene.
        PhysicsShape2D m_TemplateCustom;
        PhysicsShape2D m_TemplateBuiltIn;
        PhysicsShape2D m_OverrideCustom;
        PhysicsShape2D m_DefaultCustom;

        [UnityTest]
        public IEnumerator TemplateArm_NonOverridingCustomShape_InheritsTheAssetValues()
        {
            yield return LoadAndDiscoverShapes();

            Assert.AreEqual(
                TemplateFriction,
                m_TemplateCustom.friction,
                Eps,
                "TemplateCustom did not inherit the template friction — ResolveSurface ignored the assigned "
                    + "PhysicsMaterial2D (fell to the inline default 0.4 instead of the asset's 0.123). "
                    + "Either the template arm is broken or DependsOn did not read the asset at bake."
            );
            Assert.AreEqual(
                TemplateBounciness,
                m_TemplateCustom.bounciness,
                Eps,
                "TemplateCustom did not inherit the template bounciness — fell to the inline default 0 instead "
                    + "of the asset's 0.456."
            );
            Assert.AreEqual(
                TemplateFrictionMixing,
                m_TemplateCustom.frictionMixing,
                "TemplateCustom did not inherit the template frictionCombine (Maximum) — MapCombine/ResolveSurface "
                    + "did not route the template's combine through the friction-mixing arm."
            );
            Assert.AreEqual(
                TemplateBounceMixing,
                m_TemplateCustom.bouncinessMixing,
                "TemplateCustom did not inherit the template bounceCombine (Minimum)."
            );
        }

        [UnityTest]
        public IEnumerator Convergence_TemplateDrivenCustomShape_BakesBitIdenticalToTheBuiltInOracle()
        {
            yield return LoadAndDiscoverShapes();

            Assert.AreEqual(
                m_TemplateBuiltIn.friction,
                m_TemplateCustom.friction,
                0f,
                $"Convergence broke on friction: custom template-driven {m_TemplateCustom.friction} != built-in "
                    + $"sharedMaterial-driven {m_TemplateBuiltIn.friction}. ResolveSurface and ReadSurface diverge "
                    + "for the same PhysicsMaterial2D."
            );
            Assert.AreEqual(
                m_TemplateBuiltIn.bounciness,
                m_TemplateCustom.bounciness,
                0f,
                $"Convergence broke on bounciness: custom {m_TemplateCustom.bounciness} != built-in "
                    + $"{m_TemplateBuiltIn.bounciness}."
            );
            Assert.AreEqual(
                m_TemplateBuiltIn.frictionMixing,
                m_TemplateCustom.frictionMixing,
                "Convergence broke on frictionMixing: custom != built-in for the same material's frictionCombine."
            );
            Assert.AreEqual(
                m_TemplateBuiltIn.bouncinessMixing,
                m_TemplateCustom.bouncinessMixing,
                "Convergence broke on bouncinessMixing: custom != built-in for the same material's bounceCombine."
            );
        }

        [UnityTest]
        public IEnumerator OverrideArm_OverriddenFieldsTakeInlineValue_UnOverriddenStillInherit()
        {
            yield return LoadAndDiscoverShapes();

            Assert.AreEqual(
                OverrideFriction,
                m_OverrideCustom.friction,
                Eps,
                $"OverrideCustom friction = {m_OverrideCustom.friction}, expected the inline override "
                    + $"{OverrideFriction}. The override did NOT beat the template (got "
                    + $"{(abs(m_OverrideCustom.friction - TemplateFriction) < Eps ? "the template value" : "neither")})."
            );
            Assert.AreEqual(
                OverrideBounceMixing,
                m_OverrideCustom.bouncinessMixing,
                $"OverrideCustom bouncinessMixing = {m_OverrideCustom.bouncinessMixing}, expected the inline "
                    + $"override {OverrideBounceMixing}. The combine override did not beat the template."
            );
            // The two fields left un-overridden on the override body STILL inherit the template — per-field.
            Assert.AreEqual(
                TemplateBounciness,
                m_OverrideCustom.bounciness,
                Eps,
                "OverrideCustom bounciness should still inherit the template (it was NOT overridden) — a "
                    + "per-field override leaked into bounciness, or the whole surface fell to the default."
            );
            Assert.AreEqual(
                TemplateFrictionMixing,
                m_OverrideCustom.frictionMixing,
                "OverrideCustom frictionMixing should still inherit the template (friction-combine was NOT "
                    + "overridden) — the friction-value override wrongly captured the friction-combine."
            );
        }

        [UnityTest]
        public IEnumerator DefaultArm_NoTemplateNoOverride_BakesTheInlinePhaseADefaults()
        {
            yield return LoadAndDiscoverShapes();

            Assert.AreEqual(
                DefaultFriction,
                m_DefaultCustom.friction,
                Eps,
                $"DefaultCustom friction = {m_DefaultCustom.friction}, expected the inline default {DefaultFriction}."
            );
            Assert.AreEqual(
                DefaultBounciness,
                m_DefaultCustom.bounciness,
                Eps,
                $"DefaultCustom bounciness = {m_DefaultCustom.bounciness}, expected the inline default "
                    + $"{DefaultBounciness}."
            );
            Assert.AreEqual(
                DefaultMixing,
                m_DefaultCustom.frictionMixing,
                "DefaultCustom frictionMixing should be the inline default Average."
            );
            Assert.AreEqual(
                DefaultMixing,
                m_DefaultCustom.bouncinessMixing,
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
        }

        // Load the baked SubScene and resolve the four collider-only bodies into m_TemplateCustom / m_TemplateBuiltIn
        // / m_OverrideCustom / m_DefaultCustom, keyed by baked initialPosition.x. The bodies carry immutable baked
        // PhysicsShape2D snapshots, so each precedence arm above reads its own shape independently of the others.
        IEnumerator LoadAndDiscoverShapes()
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
            m_TemplateCustom = default;
            m_TemplateBuiltIn = default;
            m_OverrideCustom = default;
            m_DefaultCustom = default;

            for (var i = 0; i < shapes.Length; i++)
            {
                var x = defs[i].initialPosition.x;
                if (abs(x - XTemplateCustom) < 0.25f)
                {
                    m_TemplateCustom = shapes[i];
                    haveTemplateCustom = true;
                }
                else if (abs(x - XTemplateBuiltIn) < 0.25f)
                {
                    m_TemplateBuiltIn = shapes[i];
                    haveTemplateBuiltIn = true;
                }
                else if (abs(x - XOverrideCustom) < 0.25f)
                {
                    m_OverrideCustom = shapes[i];
                    haveOverrideCustom = true;
                }
                else if (abs(x - XDefaultCustom) < 0.25f)
                {
                    m_DefaultCustom = shapes[i];
                    haveDefaultCustom = true;
                }
            }

            Assert.IsTrue(
                haveTemplateCustom && haveTemplateBuiltIn && haveOverrideCustom && haveDefaultCustom,
                $"Missing one of the four baked bodies (templateCustom={haveTemplateCustom}, "
                    + $"templateBuiltIn={haveTemplateBuiltIn}, overrideCustom={haveOverrideCustom}, "
                    + $"defaultCustom={haveDefaultCustom})."
            );

            Debug.Log(
                "[PHYSICS2D-PHASEB-TEMPLATE] "
                    + $"templateCustom(f={m_TemplateCustom.friction:F4} b={m_TemplateCustom.bounciness:F4} "
                    + $"fMix={m_TemplateCustom.frictionMixing} bMix={m_TemplateCustom.bouncinessMixing}) | "
                    + $"templateBuiltIn(f={m_TemplateBuiltIn.friction:F4} b={m_TemplateBuiltIn.bounciness:F4} "
                    + $"fMix={m_TemplateBuiltIn.frictionMixing} bMix={m_TemplateBuiltIn.bouncinessMixing}) | "
                    + $"overrideCustom(f={m_OverrideCustom.friction:F4} b={m_OverrideCustom.bounciness:F4} "
                    + $"fMix={m_OverrideCustom.frictionMixing} bMix={m_OverrideCustom.bouncinessMixing}) | "
                    + $"defaultCustom(f={m_DefaultCustom.friction:F4} b={m_DefaultCustom.bounciness:F4} "
                    + $"fMix={m_DefaultCustom.frictionMixing} bMix={m_DefaultCustom.bouncinessMixing})"
            );
        }
    }
}
