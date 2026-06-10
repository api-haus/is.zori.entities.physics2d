using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="RelativeJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Relative"/>. A relative joint keeps one body at a target linear and
    /// angular offset relative to another, correcting toward that offset with capped force/torque — the Box2D
    /// <c>PhysicsRelativeJoint</c>. Editor-only assembly.
    /// </summary>
    /// <remarks>
    /// <see cref="RelativeJoint2D"/> derives from <c>Joint2D</c>, not <c>AnchoredJoint2D</c>, so it carries no
    /// <c>anchor</c>/<c>connectedAnchor</c> — the constraint is on the bodies' relative origins. The maintained
    /// offset is <c>linearOffset</c> (m) and <c>angularOffset</c> (degrees), which the creation system folds
    /// into the Box2D relative-joint anchor frames so that <c>bodyB</c> is held at that offset from
    /// <c>bodyA</c>. <c>maxForce</c>/<c>maxTorque</c> (newtons / newton-metres) cap the correction effort (zero
    /// turns a cap off, per the module XML). <c>correctionScale</c> is a built-in solver-tuning knob with no
    /// Box2D equivalent and is not baked (documented no-op).
    /// </remarks>
    public sealed class RelativeJoint2DBaker : Baker<RelativeJoint2D>
    {
        public override void Bake(RelativeJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Relative,
                    connectedBody = Joint2DBaking.ResolveConnectedBody(this, authoring),
                    // RelativeJoint2D has no anchors of its own — the constraint is on the body origins.
                    anchor = float2.zero,
                    connectedAnchor = float2.zero,
                    // The maintained relative pose of bodyB w.r.t. bodyA.
                    linearOffset = (float2)authoring.linearOffset,
                    angularOffsetDegrees = authoring.angularOffset,
                    maxForce = authoring.maxForce,
                    maxTorque = authoring.maxTorque,
                    // A relative joint REACHES the offset via the Box2D relative joint's position spring (the
                    // creation system enables it when enableSpring is set). The built-in RelativeJoint2D has no
                    // frequency knob — it is a stiff constraint capped by maxForce/maxTorque — so we drive a
                    // high-frequency, critically-damped spring (capped by those forces) to approximate that
                    // rigid pull toward the offset.
                    enableSpring = true,
                    springFrequency = 8f, // stiff position controller (the built-in has no frequency to map)
                    springDamping = 1f, // critically damped → converges to the offset without overshoot
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
