using Unity.Entities;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Shared baking helpers for the built-in 2D joints. The per-joint bakers (<c>HingeJoint2DBaker</c>,
    /// <c>SliderJoint2DBaker</c>, <c>WheelJoint2DBaker</c>) read their type-specific motor/limit/spring
    /// fields and call <see cref="ResolveConnectedBody"/> to turn the built-in <c>Joint2D.connectedBody</c>
    /// into the <see cref="Entity"/> the runtime joint definition references. Factored out so the
    /// connected-body resolution and its bake-dependency registration live in exactly one place across the
    /// joint bakers.
    /// </summary>
    public static class Joint2DBaking
    {
        /// <summary>
        /// Resolve a built-in joint's <c>connectedBody</c> (a <c>Rigidbody2D</c>) to the entity the package
        /// bakes it to, registering the bake dependency so a change to the connected body re-bakes the joint.
        /// Returns <see cref="Entity.Null"/> when the built-in joint has no connected body (an implicit
        /// static world anchor), which the creation system resolves to its shared static anchor body.
        /// </summary>
        /// <remarks>
        /// The baker's <c>this</c> is passed in because <c>GetEntity</c>/<c>DependsOn</c> are instance
        /// methods on the <c>Baker&lt;T&gt;</c> — a static helper cannot call them, so the per-joint baker
        /// hands itself in. <c>TransformUsageFlags.Dynamic</c> matches the body bakers, so the connected body
        /// resolves to the same entity the <c>Rigidbody2DBaker</c> produced.
        /// </remarks>
        public static Entity ResolveConnectedBody<T>(Baker<T> baker, Joint2D authoring)
            where T : Joint2D
        {
            var connected = authoring.connectedBody;
            // Register a dependency on the connected Rigidbody2D so an edit to it re-bakes this joint.
            baker.DependsOn(connected);
            if (connected == null)
                return Entity.Null;
            return baker.GetEntity(connected, TransformUsageFlags.Dynamic);
        }

        /// <summary>
        /// Map the built-in <c>JointBreakAction2D</c> to the package's <see cref="PhysicsJointBreakAction2D"/>
        /// (1:1). Each of the nine joint bakers folds the shared <c>Joint2D.breakForce</c>/<c>breakTorque</c>
        /// (passed through verbatim — the creation system arms the native threshold only when finite, the
        /// built-in default being <c>Infinity</c> = never break) and this mapped action into its runtime
        /// definition.
        /// </summary>
        public static PhysicsJointBreakAction2D MapBreakAction(JointBreakAction2D action)
        {
            return action switch
            {
                JointBreakAction2D.CallbackOnly => PhysicsJointBreakAction2D.CallbackOnly,
                JointBreakAction2D.Destroy => PhysicsJointBreakAction2D.Destroy,
                JointBreakAction2D.Disable => PhysicsJointBreakAction2D.Disable,
                _ => PhysicsJointBreakAction2D.Ignore,
            };
        }
    }
}
