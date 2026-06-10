using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="WheelJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Wheel"/>. The wheel joint is a slider with a suspension spring along its
    /// axis plus a rotational motor about the wheel — the Box2D <c>PhysicsWheelJoint</c>. It is the joint
    /// behind a vehicle's sprung, driven wheel. Editor-only assembly.
    /// </summary>
    /// <remarks>
    /// The suspension lives in <c>WheelJoint2D.suspension</c> (<c>JointSuspension2D</c>): <c>angle</c>
    /// (degrees) is the suspension axis → <c>localAnchorA</c>'s rotation; <c>frequency</c> (Hz) →
    /// <c>springFrequency</c>; <c>dampingRatio</c> (non-dimensional) → <c>springDamping</c>. A wheel always
    /// has a spring (it is the joint's defining feature), so the runtime spring is enabled. The motor is
    /// rotational: <c>JointMotor2D.maxMotorTorque</c> → <c>maxMotorTorque</c>, <c>motorSpeed</c> (deg/sec) →
    /// <c>motorSpeed</c>. The built-in <c>WheelJoint2D</c> exposes no translation limit, so the runtime limit
    /// stays disabled (the suspension spring, not a hard limit, bounds travel).
    /// </remarks>
    public sealed class WheelJoint2DBaker : Baker<WheelJoint2D>
    {
        public override void Bake(WheelJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var motor = authoring.motor;
            var suspension = authoring.suspension;

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Wheel,
                    connectedBody = Joint2DBaking.ResolveConnectedBody(this, authoring),
                    anchor = (float2)authoring.anchor,
                    connectedAnchor = (float2)authoring.connectedAnchor,
                    // JointSuspension2D.angle is the suspension axis angle in degrees.
                    axisAngleDegrees = suspension.angle,
                    enableMotor = authoring.useMotor,
                    motorSpeed = motor.motorSpeed,
                    maxMotorEffort = motor.maxMotorTorque,
                    // WheelJoint2D has no translation limit field.
                    enableLimit = false,
                    lowerLimit = 0f,
                    upperLimit = 0f,
                    // A wheel's suspension is always a spring.
                    enableSpring = true,
                    springFrequency = suspension.frequency,
                    springDamping = suspension.dampingRatio,
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
