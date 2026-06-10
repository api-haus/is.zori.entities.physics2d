using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="SliderJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Slider"/>. The slider constrains a body to translate along a fixed axis
    /// relative to the connected body, optionally driven by a linear motor and clamped by translation
    /// limits — the Box2D <c>PhysicsSliderJoint</c>. Editor-only assembly.
    /// </summary>
    /// <remarks>
    /// The slide axis is <c>SliderJoint2D.angle</c> (degrees), carried to the runtime definition and folded
    /// into <c>localAnchorA</c>'s rotation at creation so the frame's local X is the slide direction. Motor
    /// speed is m/s and translation limits are meters on both sides (built-in
    /// <c>JointMotor2D.motorSpeed</c>/<c>JointTranslationLimits2D</c> ↔ Box2D <c>motorSpeed</c>/<c>lower/
    /// upperTranslationLimit</c>), so both bake 1:1. The built-in slider exposes no spring, so the runtime
    /// spring stays disabled.
    /// </remarks>
    public sealed class SliderJoint2DBaker : Baker<SliderJoint2D>
    {
        public override void Bake(SliderJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var motor = authoring.motor;
            var limits = authoring.limits;

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Slider,
                    connectedBody = Joint2DBaking.ResolveConnectedBody(this, authoring),
                    anchor = (float2)authoring.anchor,
                    connectedAnchor = (float2)authoring.connectedAnchor,
                    // SliderJoint2D.angle is the slide-line angle in degrees.
                    axisAngleDegrees = authoring.angle,
                    enableMotor = authoring.useMotor,
                    // JointMotor2D.motorSpeed is m/s for a slider; Box2D slider motorSpeed is m/s — 1:1.
                    motorSpeed = motor.motorSpeed,
                    maxMotorEffort = motor.maxMotorTorque, // → maxMotorForce for the slider
                    enableLimit = authoring.useLimits,
                    // JointTranslationLimits2D min/max are meters; Box2D lower/upperTranslationLimit are too.
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
