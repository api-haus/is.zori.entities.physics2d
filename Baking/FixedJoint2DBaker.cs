using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="FixedJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Fixed"/>. A fixed joint locks two bodies in their relative pose (position
    /// and orientation) — the Box2D <c>PhysicsFixedJoint</c>. With a zero frequency it is a rigid weld; a
    /// non-zero frequency makes it a stiff spring around that pose. Editor-only assembly.
    /// </summary>
    /// <remarks>
    /// The built-in <see cref="FixedJoint2D"/> exposes a single <c>frequency</c>/<c>dampingRatio</c> pair that
    /// governs the whole joint; Box2D's <c>PhysicsFixedJointDefinition</c> splits the stiffness into a linear
    /// and an angular term, so the one built-in frequency feeds BOTH (<c>frequency</c> →
    /// <c>linearFrequency</c> + <c>angularFrequency</c>, <c>dampingRatio</c> → <c>linearDamping</c> +
    /// <c>angularDamping</c>) at creation. Frequency zero is the maximum-stiffness rigid lock (module XML:
    /// "Use zero for maximum stiffness"), which is the default fixed joint. The frequency/damping ride in the
    /// shared <c>springFrequency</c>/<c>springDamping</c> fields. <c>anchor</c>/<c>connectedAnchor</c> are each
    /// body's local space, folded into the runtime definition; the relative pose the joint locks is implied by
    /// the bodies' authored transforms at bake (the same way the built-in joint captures it).
    /// </remarks>
    public sealed class FixedJoint2DBaker : Baker<FixedJoint2D>
    {
        public override void Bake(FixedJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Fixed,
                    connectedBody = Joint2DBaking.ResolveConnectedBody(this, authoring),
                    anchor = (float2)authoring.anchor,
                    connectedAnchor = (float2)authoring.connectedAnchor,
                    // FixedJoint2D.frequency/dampingRatio feed both the linear and angular Box2D stiffness;
                    // zero frequency is the rigid weld. Carried in the shared spring fields, applied to both
                    // axes at creation (see PhysicsJoint2DCreationSystem's Fixed arm).
                    enableSpring = false,
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
