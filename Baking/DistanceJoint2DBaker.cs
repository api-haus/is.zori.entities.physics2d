using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="DistanceJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Distance"/>. A distance joint holds the two anchors a fixed distance
    /// apart — a rigid rod (or rope, if <c>maxDistanceOnly</c>) — the Box2D <c>PhysicsDistanceJoint</c> with
    /// its spring disabled. Editor-only assembly, so this never reaches a player build.
    /// </summary>
    /// <remarks>
    /// <c>DistanceJoint2D.distance</c> → the runtime <c>restLength</c> the Box2D distance joint keeps between
    /// <c>localAnchorA</c> and <c>localAnchorB</c>. With <c>enableSpring</c> false the constraint is rigid (it
    /// overrides limit and motor, per the module XML), which is the plain distance joint's behaviour — the
    /// anchors stay exactly <c>distance</c> apart. <c>anchor</c>/<c>connectedAnchor</c> are each body's local
    /// space (the <c>AnchoredJoint2D</c> convention), folded directly into the runtime definition. A distance
    /// joint has no motor or angle in its built-in surface, so those runtime fields stay zero/disabled.
    /// </remarks>
    public sealed class DistanceJoint2DBaker : Baker<DistanceJoint2D>
    {
        public override void Bake(DistanceJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Distance,
                    connectedBody = Joint2DBaking.ResolveConnectedBody(this, authoring),
                    anchor = (float2)authoring.anchor,
                    connectedAnchor = (float2)authoring.connectedAnchor,
                    // DistanceJoint2D.distance is the fixed separation the joint maintains.
                    restLength = authoring.distance,
                    // A plain distance joint is rigid — no spring.
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
