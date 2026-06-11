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
    /// Phase-F custom-joint CONVERGENCE smoke (BAKE level): for each representative joint kind, a custom
    /// <c>PhysicsJoint2DAuthoring</c> body and the equivalent built-in <c>*Joint2D</c> body of identical
    /// parameters bake — through the REAL bakers, in one SubScene — to a <see cref="PhysicsJoint2DDefinition"/>,
    /// and this gate asserts the two baked definitions are FIELD-EQUAL. That is the standing convergence
    /// property for joints: a custom-authored joint bakes the same runtime joint as the built-in
    /// <c>*Joint2D</c> of the same params, so it creates the same Box2D constraint and simulates identically —
    /// exactly as the body/shape custom authoring converges with the built-in surface.
    /// </summary>
    /// <remarks>
    /// <para>The owner entities are keyed by their baked <c>PhysicsBody2DDefinition.initialPosition.x</c> (the
    /// runtime Tests asmdef cannot reference the Editor-platform builder, so the X-keys are duplicated as
    /// constants — the package's established pattern, see <see cref="MaterialTemplateBakeGate"/> /
    /// <see cref="FilterBakeParityGate"/>). Each joint owner carries BOTH a <c>PhysicsBody2DDefinition</c> (the
    /// body) and a <c>PhysicsJoint2DDefinition</c> (the joint), so the X-key picks the right joint def.</para>
    ///
    /// <para>The compare excludes the <c>connectedBody</c> Entity: the custom joint connects to a custom-authored
    /// static anchor and the built-in joint to a <see cref="Rigidbody2D"/> anchor, so the two connected entities
    /// differ by construction (different GameObjects). Convergence is about the joint GEOMETRY/motor/limit/
    /// spring/break fields, not the entity identity of the connected body — and for the Target pair (null
    /// connected → <c>Entity.Null</c> on both) even that matches, asserted separately.</para>
    ///
    /// <para>Build the fixture first via <c>-executeMethod
    /// Zori.Entities.Physics2D.Tests.Editor.CustomJointConvergenceFixtureBuilder.Build</c>.</para>
    /// </remarks>
    public sealed class CustomJointConvergenceBakeGate
    {
        const int LoadTimeoutFrames = 600;
        const string ParentScenePath = "Assets/EntitiesPhysics2DFixture/CustomJointConvergence.unity";

        // Mirror of CustomJointConvergenceFixtureBuilder X-keys.
        const float XHingeCustom = -10f;
        const float XHingeBuiltIn = -8f;
        const float XWheelCustom = -4f;
        const float XWheelBuiltIn = -2f;
        const float XRelativeCustom = 2f;
        const float XRelativeBuiltIn = 4f;
        const float XTargetCustom = 8f;
        const float XTargetBuiltIn = 10f;

        const int ExpectedJointCount = 8; // 4 kinds × (custom + built-in)

        [UnityTest]
        public IEnumerator CustomJoint_BakesSameDefinition_AsBuiltInJoint()
        {
            SceneManager.LoadScene(ParentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");
            var em = world.EntityManager;

            var query = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsJoint2DDefinition>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );

            var framesWaited = 0;
            while (query.CalculateEntityCount() < ExpectedJointCount && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            var count = query.CalculateEntityCount();
            Assert.GreaterOrEqual(
                count,
                ExpectedJointCount,
                $"Only {count} baked joint owners appeared after {framesWaited} frames (expected "
                    + $"{ExpectedJointCount}) — build the fixture first via -executeMethod "
                    + "Zori.Entities.Physics2D.Tests.Editor.CustomJointConvergenceFixtureBuilder.Build."
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

            AssertConverges(hingeCustom, hingeBuiltIn, "Hinge", compareConnectedBody: false);
            AssertConverges(wheelCustom, wheelBuiltIn, "Wheel", compareConnectedBody: false);
            AssertConverges(relativeCustom, relativeBuiltIn, "Relative", compareConnectedBody: false);
            // Target uses a null connected body on BOTH sides (Entity.Null), so the connected body matches too.
            AssertConverges(targetCustom, targetBuiltIn, "Target", compareConnectedBody: true);
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

            yield break;
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
