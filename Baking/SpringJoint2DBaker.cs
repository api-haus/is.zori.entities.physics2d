using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="SpringJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Spring"/>. A spring joint is a distance joint whose constraint behaves
    /// like a spring: the two anchors oscillate toward a rest length rather than holding it rigidly — the
    /// Box2D <c>PhysicsDistanceJoint</c> with its spring enabled. Editor-only assembly.
    /// </summary>
    /// <remarks>
    /// Same Box2D kind as <see cref="DistanceJoint2DBaker"/> (the distance joint), distinguished by
    /// <c>enableSpring</c> = true. <c>SpringJoint2D.distance</c> → the runtime <c>restLength</c> the spring
    /// pulls toward; <c>frequency</c> (Hz) → <c>springFrequency</c>; <c>dampingRatio</c> (non-dimensional) →
    /// <c>springDamping</c>. <c>anchor</c>/<c>connectedAnchor</c> are each body's local space, folded directly
    /// into the runtime definition. With the spring enabled the constraint oscillates around <c>restLength</c>
    /// at the authored frequency, settling as the damping bleeds the oscillation out.
    /// </remarks>
    public sealed class SpringJoint2DBaker : Baker<SpringJoint2D>
    {
        public override void Bake(SpringJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Spring,
                    connectedBody = Joint2DBaking.ResolveConnectedBody(this, authoring),
                    anchor = (float2)authoring.anchor,
                    connectedAnchor = (float2)authoring.connectedAnchor,
                    // SpringJoint2D.distance is the rest length the spring oscillates toward.
                    restLength = authoring.distance,
                    // A spring joint's defining feature is its spring.
                    enableSpring = true,
                    springFrequency = authoring.frequency,
                    springDamping = authoring.dampingRatio,
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
