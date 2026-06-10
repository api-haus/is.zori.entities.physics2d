using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="HingeJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Hinge"/>. The hinge pins two bodies at a shared anchor and lets them
    /// rotate about it, optionally driven by a motor and clamped by an angle limit — the Box2D
    /// <c>PhysicsHingeJoint</c>. Editor-only assembly, so this never reaches a player build.
    /// </summary>
    /// <remarks>
    /// The owner GameObject's <c>Rigidbody2D</c> is Box2D <c>bodyB</c>; <c>connectedBody</c> is <c>bodyA</c>.
    /// <c>anchor</c>/<c>connectedAnchor</c> are each body's local space (the built-in convention), folded
    /// directly into the runtime definition. Angle limits and motor speed are degrees on BOTH sides — the
    /// built-in <c>JointAngleLimits2D</c>/<c>JointMotor2D.motorSpeed</c> are degrees, and the Box2D
    /// <c>lower/upperAngleLimit</c> and <c>motorSpeed</c> are documented "in degrees" / "degrees per second"
    /// (module XML), so both bake 1:1 with no radians conversion. A hinge has no built-in spring field, so
    /// the runtime spring stays disabled.
    /// </remarks>
    public sealed class HingeJoint2DBaker : Baker<HingeJoint2D>
    {
        public override void Bake(HingeJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var motor = authoring.motor;
            var limits = authoring.limits;

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Hinge,
                    connectedBody = Joint2DBaking.ResolveConnectedBody(this, authoring),
                    anchor = (float2)authoring.anchor,
                    connectedAnchor = (float2)authoring.connectedAnchor,
                    // A hinge constrains rotation only — no translation axis.
                    axisAngleDegrees = 0f,
                    enableMotor = authoring.useMotor,
                    // JointMotor2D.motorSpeed is deg/sec; the Box2D hinge motorSpeed is also deg/sec, so 1:1.
                    motorSpeed = motor.motorSpeed,
                    maxMotorEffort = motor.maxMotorTorque,
                    enableLimit = authoring.useLimits,
                    // JointAngleLimits2D min/max are degrees; Box2D lower/upperAngleLimit are degrees too — 1:1.
                    lowerLimit = limits.min,
                    upperLimit = limits.max,
                    enableSpring = false,
                    springFrequency = 0f,
                    springDamping = 0f,
                    collideConnected = authoring.enableCollision,
                    // Break: shared Joint2D.breakForce/breakTorque/breakAction. Passed through; the creation
                    // system arms the native threshold only when finite (default Infinity = never break).
                    breakForce = authoring.breakForce,
                    breakTorque = authoring.breakTorque,
                    breakAction = Joint2DBaking.MapBreakAction(authoring.breakAction),
                }
            );
        }
    }
}
