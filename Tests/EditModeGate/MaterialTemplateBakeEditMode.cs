using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using UnityEngine;
using Zori.Entities.Physics2D.Tests.Editor;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>MaterialTemplateBakeGate</c>: pins each arm of the shape baker's
    /// <c>ResolveSurface</c> precedence (override &gt; template &gt; inline-default) on the baked
    /// <c>PhysicsShape2D</c> surface, against a real <c>PhysicsMaterial2D</c> asset, through the real baker. The
    /// four bodies are keyed by baked <c>initialPosition.x</c>; assertions and expected values copied verbatim.
    /// </summary>
    public sealed class MaterialTemplateBakeEditMode : Physics2DEditModeHarness
    {
        const float XTemplateCustom = Physics2DFixtures.MtXTemplateCustom;
        const float XTemplateBuiltIn = Physics2DFixtures.MtXTemplateBuiltIn;
        const float XOverrideCustom = Physics2DFixtures.MtXOverrideCustom;
        const float XDefaultCustom = Physics2DFixtures.MtXDefaultCustom;

        const float TemplateFriction = Physics2DFixtures.MtTemplateFriction;
        const float TemplateBounciness = Physics2DFixtures.MtTemplateBounciness;
        const PhysicsSurfaceMixing2D TemplateFrictionMixing = PhysicsSurfaceMixing2D.Maximum;
        const PhysicsSurfaceMixing2D TemplateBounceMixing = PhysicsSurfaceMixing2D.Minimum;
        const float OverrideFriction = Physics2DFixtures.MtOverrideFriction;
        const PhysicsSurfaceMixing2D OverrideBounceMixing = PhysicsSurfaceMixing2D.Multiply;
        const float DefaultFriction = Physics2DFixtures.MtDefaultFriction;
        const float DefaultBounciness = Physics2DFixtures.MtDefaultBounciness;
        const PhysicsSurfaceMixing2D DefaultMixing = PhysicsSurfaceMixing2D.Average;
        const float Eps = 1e-5f;

        PhysicsShape2D m_TemplateCustom;
        PhysicsShape2D m_TemplateBuiltIn;
        PhysicsShape2D m_OverrideCustom;
        PhysicsShape2D m_DefaultCustom;

        void Discover()
        {
            LoadSubScene(Physics2DFixtures.MaterialTemplate, "MaterialTemplate");

            var query = Query(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );
            Assert.GreaterOrEqual(query.CalculateEntityCount(), 4, "Expected >= 4 baked bodies.");

            using var shapes = query.ToComponentDataArray<PhysicsShape2D>(Allocator.Temp);
            using var defs = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);

            var haveTc = false;
            var haveTb = false;
            var haveOc = false;
            var haveDc = false;
            for (var i = 0; i < shapes.Length; i++)
            {
                var x = defs[i].initialPosition.x;
                if (abs(x - XTemplateCustom) < 0.25f)
                {
                    m_TemplateCustom = shapes[i];
                    haveTc = true;
                }
                else if (abs(x - XTemplateBuiltIn) < 0.25f)
                {
                    m_TemplateBuiltIn = shapes[i];
                    haveTb = true;
                }
                else if (abs(x - XOverrideCustom) < 0.25f)
                {
                    m_OverrideCustom = shapes[i];
                    haveOc = true;
                }
                else if (abs(x - XDefaultCustom) < 0.25f)
                {
                    m_DefaultCustom = shapes[i];
                    haveDc = true;
                }
            }
            Assert.IsTrue(
                haveTc && haveTb && haveOc && haveDc,
                $"Missing a baked body (tc={haveTc}, tb={haveTb}, oc={haveOc}, dc={haveDc})."
            );
        }

        [Test]
        public void TemplateArm_NonOverridingCustomShape_InheritsTheAssetValues()
        {
            Discover();
            Assert.AreEqual(TemplateFriction, m_TemplateCustom.friction, Eps, "TemplateCustom friction != asset.");
            Assert.AreEqual(
                TemplateBounciness,
                m_TemplateCustom.bounciness,
                Eps,
                "TemplateCustom bounciness != asset."
            );
            Assert.AreEqual(
                TemplateFrictionMixing,
                m_TemplateCustom.frictionMixing,
                "TemplateCustom frictionMixing != asset (Maximum)."
            );
            Assert.AreEqual(
                TemplateBounceMixing,
                m_TemplateCustom.bouncinessMixing,
                "TemplateCustom bouncinessMixing != asset (Minimum)."
            );
        }

        [Test]
        public void Convergence_TemplateDrivenCustomShape_BakesBitIdenticalToTheBuiltInOracle()
        {
            Discover();
            Assert.AreEqual(
                m_TemplateBuiltIn.friction,
                m_TemplateCustom.friction,
                0f,
                "Convergence broke on friction."
            );
            Assert.AreEqual(
                m_TemplateBuiltIn.bounciness,
                m_TemplateCustom.bounciness,
                0f,
                "Convergence broke on bounciness."
            );
            Assert.AreEqual(
                m_TemplateBuiltIn.frictionMixing,
                m_TemplateCustom.frictionMixing,
                "Convergence broke on frictionMixing."
            );
            Assert.AreEqual(
                m_TemplateBuiltIn.bouncinessMixing,
                m_TemplateCustom.bouncinessMixing,
                "Convergence broke on bouncinessMixing."
            );
        }

        [Test]
        public void OverrideArm_OverriddenFieldsTakeInlineValue_UnOverriddenStillInherit()
        {
            Discover();
            Assert.AreEqual(
                OverrideFriction,
                m_OverrideCustom.friction,
                Eps,
                "OverrideCustom friction != inline override."
            );
            Assert.AreEqual(
                OverrideBounceMixing,
                m_OverrideCustom.bouncinessMixing,
                "OverrideCustom bouncinessMixing != inline override."
            );
            Assert.AreEqual(
                TemplateBounciness,
                m_OverrideCustom.bounciness,
                Eps,
                "OverrideCustom bounciness should still inherit the template."
            );
            Assert.AreEqual(
                TemplateFrictionMixing,
                m_OverrideCustom.frictionMixing,
                "OverrideCustom frictionMixing should still inherit the template."
            );
        }

        [Test]
        public void DefaultArm_NoTemplateNoOverride_BakesTheInlinePhaseADefaults()
        {
            Discover();
            Assert.AreEqual(
                DefaultFriction,
                m_DefaultCustom.friction,
                Eps,
                "DefaultCustom friction != inline default."
            );
            Assert.AreEqual(
                DefaultBounciness,
                m_DefaultCustom.bounciness,
                Eps,
                "DefaultCustom bounciness != inline default."
            );
            Assert.AreEqual(
                DefaultMixing,
                m_DefaultCustom.frictionMixing,
                "DefaultCustom frictionMixing should be Average."
            );
            Assert.AreEqual(
                DefaultMixing,
                m_DefaultCustom.bouncinessMixing,
                "DefaultCustom bouncinessMixing should be Average."
            );
            Assert.AreNotEqual(
                DefaultFriction,
                TemplateFriction,
                "Self-check: template friction must differ from default."
            );
        }
    }
}
