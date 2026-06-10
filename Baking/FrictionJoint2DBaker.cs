using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="FrictionJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Friction"/>. A friction joint is a relative joint with a ZERO target
    /// offset: it does not move the bodies toward a pose, it only resists their relative linear and angular
    /// motion up to a force/torque cap — sliding/rotational friction between two bodies — the Box2D
    /// <c>PhysicsRelativeJoint</c>. Editor-only assembly.
    /// </summary>
    /// <remarks>
    /// <see cref="FrictionJoint2D"/> derives from <c>Joint2D</c>, so it carries no <c>anchor</c>/
    /// <c>connectedAnchor</c> and no target offset — the relative joint bakes with a zero
    /// <c>linearOffset</c>/<c>angularOffset</c>, which makes it hold the bodies' CURRENT relative pose and
    /// damp any change to it. <c>maxForce</c>/<c>maxTorque</c> (newtons / newton-metres) are the friction caps
    /// → Box2D <c>maxForce</c>/<c>maxTorque</c>: relative motion that needs less than the cap is fully
    /// resisted (the bodies move together), motion needing more slips. This is the same Box2D kind as
    /// <see cref="RelativeJoint2DBaker"/>, distinguished by the zero offset.
    /// </remarks>
    public sealed class FrictionJoint2DBaker : Baker<FrictionJoint2D>
    {
        public override void Bake(FrictionJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Friction,
                    connectedBody = Joint2DBaking.ResolveConnectedBody(this, authoring),
                    anchor = float2.zero,
                    connectedAnchor = float2.zero,
                    // Friction has no target offset — it resists relative motion from the current pose.
                    linearOffset = float2.zero,
                    angularOffsetDegrees = 0f,
                    maxForce = authoring.maxForce,
                    maxTorque = authoring.maxTorque,
                    // No position spring — a friction joint only DAMPS relative motion (the Box2D relative
                    // joint's velocity control), it never drives toward a pose. enableSpring false makes the
                    // creation system set maxForce/maxTorque as the velocity-control friction caps.
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
