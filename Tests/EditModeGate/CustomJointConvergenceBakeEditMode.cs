using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using UnityEngine;
using Zori.Entities.Physics2D.Tests.Editor;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>CustomJointConvergenceBakeGate</c> (BAKE level): for each of the nine joint kinds, a
    /// custom <c>PhysicsJoint2DAuthoring</c> body and the equivalent built-in <c>*Joint2D</c> body of identical
    /// parameters bake — through the REAL bakers, in one SubScene — to a <see cref="PhysicsJoint2DDefinition"/>, and
    /// this gate asserts the two baked definitions are FIELD-EQUAL. Loads the fixture through
    /// <c>Physics2DFixtures.CustomJointConvergence</c> into a private Game world (no <c>SceneManager.LoadScene</c>,
    /// no build-settings registration) and reads the baked defs only — no body creation needed. X-keys,
    /// <c>Debug.Log</c>, and the <c>Pick</c>/<c>AssertConverges</c>/<c>AssertEq</c> compare are copied verbatim.
    /// </summary>
    public sealed class CustomJointConvergenceBakeEditMode : Physics2DEditModeHarness
    {
        // Mirror of CustomJointConvergenceFixtureBuilder X-keys. ALL NINE kinds are pinned (the four original
        // representative kinds at X ∈ [−10, 10] plus the five the validation gate added at X ≥ 20).
        const float XHingeCustom = -10f;
        const float XHingeBuiltIn = -8f;
        const float XWheelCustom = -4f;
        const float XWheelBuiltIn = -2f;
        const float XRelativeCustom = 2f;
        const float XRelativeBuiltIn = 4f;
        const float XTargetCustom = 8f;
        const float XTargetBuiltIn = 10f;
        const float XSliderCustom = 20f;
        const float XSliderBuiltIn = 22f;
        const float XDistanceCustom = 26f;
        const float XDistanceBuiltIn = 28f;
        const float XSpringCustom = 32f;
        const float XSpringBuiltIn = 34f;
        const float XFixedCustom = 38f;
        const float XFixedBuiltIn = 40f;
        const float XFrictionCustom = 44f;
        const float XFrictionBuiltIn = 46f;

        const int ExpectedJointCount = 18; // 9 kinds × (custom + built-in)

        [Test]
        public void CustomJoint_BakesSameDefinition_AsBuiltInJoint()
        {
            LoadSubScene(Physics2DFixtures.CustomJointConvergence, "CustomJointConvergence");

            var query = Query(
                ComponentType.ReadOnly<PhysicsJoint2DDefinition>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );

            var count = query.CalculateEntityCount();
            Assert.GreaterOrEqual(
                count,
                ExpectedJointCount,
                $"Only {count} baked joint owners appeared (expected {ExpectedJointCount}) — the "
                    + "CustomJointConvergence fixture did not bake all nine custom-vs-built-in pairs."
            );

            using var joints = query.ToComponentDataArray<PhysicsJoint2DDefinition>(Allocator.Temp);
            using var bodies = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);

            var hingeCustom = Pick(joints, bodies, XHingeCustom, "Hinge custom");
            var hingeBuiltIn = Pick(joints, bodies, XHingeBuiltIn, "Hinge built-in");
            var wheelCustom = Pick(joints, bodies, XWheelCustom, "Wheel custom");
            var wheelBuiltIn = Pick(joints, bodies, XWheelBuiltIn, "Wheel built-in");
            var relativeCustom = Pick(joints, bodies, XRelativeCustom, "Relative custom");
            var relativeBuiltIn = Pick(joints, bodies, XRelativeBuiltIn, "Relative built-in");
            var targetCustom = Pick(joints, bodies, XTargetCustom, "Target custom");
            var targetBuiltIn = Pick(joints, bodies, XTargetBuiltIn, "Target built-in");
            var sliderCustom = Pick(joints, bodies, XSliderCustom, "Slider custom");
            var sliderBuiltIn = Pick(joints, bodies, XSliderBuiltIn, "Slider built-in");
            var distanceCustom = Pick(joints, bodies, XDistanceCustom, "Distance custom");
            var distanceBuiltIn = Pick(joints, bodies, XDistanceBuiltIn, "Distance built-in");
            var springCustom = Pick(joints, bodies, XSpringCustom, "Spring custom");
            var springBuiltIn = Pick(joints, bodies, XSpringBuiltIn, "Spring built-in");
            var fixedCustom = Pick(joints, bodies, XFixedCustom, "Fixed custom");
            var fixedBuiltIn = Pick(joints, bodies, XFixedBuiltIn, "Fixed built-in");
            var frictionCustom = Pick(joints, bodies, XFrictionCustom, "Friction custom");
            var frictionBuiltIn = Pick(joints, bodies, XFrictionBuiltIn, "Friction built-in");

            AssertConverges(hingeCustom, hingeBuiltIn, "Hinge", compareConnectedBody: false);
            AssertConverges(wheelCustom, wheelBuiltIn, "Wheel", compareConnectedBody: false);
            AssertConverges(relativeCustom, relativeBuiltIn, "Relative", compareConnectedBody: false);
            // Target uses a null connected body on BOTH sides (Entity.Null), so the connected body matches too.
            AssertConverges(targetCustom, targetBuiltIn, "Target", compareConnectedBody: true);
            AssertConverges(sliderCustom, sliderBuiltIn, "Slider", compareConnectedBody: false);
            AssertConverges(distanceCustom, distanceBuiltIn, "Distance", compareConnectedBody: false);
            AssertConverges(springCustom, springBuiltIn, "Spring", compareConnectedBody: false);
            AssertConverges(fixedCustom, fixedBuiltIn, "Fixed", compareConnectedBody: false);
            AssertConverges(frictionCustom, frictionBuiltIn, "Friction", compareConnectedBody: false);
            Assert.AreEqual(
                Entity.Null,
                targetCustom.connectedBody,
                "Target custom connectedBody should be Entity.Null (the static world-anchor path)."
            );
            Assert.AreEqual(
                Entity.Null,
                targetBuiltIn.connectedBody,
                "Target built-in connectedBody should be Entity.Null (TargetJoint2D is single-body)."
            );
        }

        static PhysicsJoint2DDefinition Pick(
            NativeArray<PhysicsJoint2DDefinition> joints,
            NativeArray<PhysicsBody2DDefinition> bodies,
            float x,
            string label
        )
        {
            for (var i = 0; i < joints.Length; i++)
            {
                if (abs(bodies[i].initialPosition.x - x) < 0.25f)
                    return joints[i];
            }
            Assert.Fail($"No baked joint owner found at X={x} ({label}).");
            return default;
        }

        // Field-by-field convergence of the two baked joint definitions. connectedBody is compared only for the
        // null-anchor (Target) case; everywhere else the two joints reference different anchor bodies by design.
        static void AssertConverges(
            PhysicsJoint2DDefinition custom,
            PhysicsJoint2DDefinition builtIn,
            string label,
            bool compareConnectedBody
        )
        {
            Debug.Log(
                $"[PHYSICS2D-PHASEF-CONVERGE] {label} custom(kind={custom.kind} anchor={custom.anchor} "
                    + $"connAnchor={custom.connectedAnchor} axis={custom.axisAngleDegrees} mot={custom.enableMotor}/"
                    + $"{custom.motorSpeed}/{custom.maxMotorEffort} lim={custom.enableLimit}/{custom.lowerLimit}/"
                    + $"{custom.upperLimit} spr={custom.enableSpring}/{custom.springFrequency}/{custom.springDamping} "
                    + $"rest={custom.restLength} off={custom.linearOffset}/{custom.angularOffsetDegrees} "
                    + $"maxF={custom.maxForce} maxT={custom.maxTorque} coll={custom.collideConnected} "
                    + $"brk={custom.breakForce}/{custom.breakTorque}/{custom.breakAction}) | built-in(kind="
                    + $"{builtIn.kind} anchor={builtIn.anchor} connAnchor={builtIn.connectedAnchor} "
                    + $"axis={builtIn.axisAngleDegrees} mot={builtIn.enableMotor}/{builtIn.motorSpeed}/"
                    + $"{builtIn.maxMotorEffort} lim={builtIn.enableLimit}/{builtIn.lowerLimit}/{builtIn.upperLimit} "
                    + $"spr={builtIn.enableSpring}/{builtIn.springFrequency}/{builtIn.springDamping} "
                    + $"rest={builtIn.restLength} off={builtIn.linearOffset}/{builtIn.angularOffsetDegrees} "
                    + $"maxF={builtIn.maxForce} maxT={builtIn.maxTorque} coll={builtIn.collideConnected} "
                    + $"brk={builtIn.breakForce}/{builtIn.breakTorque}/{builtIn.breakAction})"
            );

            Assert.AreEqual(builtIn.kind, custom.kind, $"{label}: kind diverged.");
            AssertEq(builtIn.anchor.x, custom.anchor.x, $"{label}: anchor.x");
            AssertEq(builtIn.anchor.y, custom.anchor.y, $"{label}: anchor.y");
            AssertEq(builtIn.connectedAnchor.x, custom.connectedAnchor.x, $"{label}: connectedAnchor.x");
            AssertEq(builtIn.connectedAnchor.y, custom.connectedAnchor.y, $"{label}: connectedAnchor.y");
            AssertEq(builtIn.axisAngleDegrees, custom.axisAngleDegrees, $"{label}: axisAngleDegrees");
            Assert.AreEqual(builtIn.enableMotor, custom.enableMotor, $"{label}: enableMotor");
            AssertEq(builtIn.motorSpeed, custom.motorSpeed, $"{label}: motorSpeed");
            AssertEq(builtIn.maxMotorEffort, custom.maxMotorEffort, $"{label}: maxMotorEffort");
            Assert.AreEqual(builtIn.enableLimit, custom.enableLimit, $"{label}: enableLimit");
            AssertEq(builtIn.lowerLimit, custom.lowerLimit, $"{label}: lowerLimit");
            AssertEq(builtIn.upperLimit, custom.upperLimit, $"{label}: upperLimit");
            Assert.AreEqual(builtIn.enableSpring, custom.enableSpring, $"{label}: enableSpring");
            AssertEq(builtIn.springFrequency, custom.springFrequency, $"{label}: springFrequency");
            AssertEq(builtIn.springDamping, custom.springDamping, $"{label}: springDamping");
            AssertEq(builtIn.restLength, custom.restLength, $"{label}: restLength");
            AssertEq(builtIn.linearOffset.x, custom.linearOffset.x, $"{label}: linearOffset.x");
            AssertEq(builtIn.linearOffset.y, custom.linearOffset.y, $"{label}: linearOffset.y");
            AssertEq(builtIn.angularOffsetDegrees, custom.angularOffsetDegrees, $"{label}: angularOffsetDegrees");
            AssertEq(builtIn.maxForce, custom.maxForce, $"{label}: maxForce");
            AssertEq(builtIn.maxTorque, custom.maxTorque, $"{label}: maxTorque");
            Assert.AreEqual(builtIn.collideConnected, custom.collideConnected, $"{label}: collideConnected");
            AssertEq(builtIn.breakForce, custom.breakForce, $"{label}: breakForce");
            AssertEq(builtIn.breakTorque, custom.breakTorque, $"{label}: breakTorque");
            Assert.AreEqual(builtIn.breakAction, custom.breakAction, $"{label}: breakAction");

            if (compareConnectedBody)
                Assert.AreEqual(builtIn.connectedBody, custom.connectedBody, $"{label}: connectedBody");
        }

        // Near-exact float compare. Most fields are bit-identical (both bakers write the same authored value),
        // but a few built-in properties round-trip through radians internally (RelativeJoint2D.angularOffset
        // bakes 15° as 15.00000095°), so the band is a tiny epsilon rather than 0 — still adversarially tight
        // (a mismapped field diverges by whole units / degrees, orders of magnitude past this). Infinity (the
        // never-break default) compares equal under Assert.AreEqual.
        const float ConvergenceEps = 1e-4f;

        static void AssertEq(float expected, float actual, string field)
        {
            if (float.IsInfinity(expected) || float.IsInfinity(actual))
            {
                Assert.AreEqual(
                    expected,
                    actual,
                    $"{field} diverged: built-in {expected} != custom {actual} (infinite-threshold mismatch)."
                );
                return;
            }
            Assert.AreEqual(
                expected,
                actual,
                ConvergenceEps,
                $"{field} diverged: built-in {expected} != custom {actual} — the custom baker did not "
                    + "reproduce the built-in baker's field for this kind."
            );
        }
    }
}
