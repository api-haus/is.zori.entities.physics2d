using Unity.Entities;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// One joint-break event from the most recent simulation step — the package's analogue of
    /// <c>Joint2D.OnJointBreak2D</c>. Produced when a joint's reaction force/torque exceeds its baked
    /// <c>breakForce</c>/<c>breakTorque</c> (Box2D's native <c>jointThresholdEvents</c>). Lives in a
    /// <c>DynamicBuffer&lt;PhysicsJointBreakEvent2D&gt;</c> on the <see cref="PhysicsWorldSingleton2D"/> entity,
    /// cleared and refilled by <c>PhysicsWorld2DSystem</c> each step.
    /// </summary>
    /// <remarks>
    /// <b>Collect / apply split.</b> The producer (<c>PhysicsWorld2DSystem</c>) only COLLECTS these from the
    /// volatile post-step joint-threshold span (resolving each joint to its owner entity via the packed
    /// <c>joint.userData</c>); the structural reaction — destroying the Box2D joint and removing
    /// <c>PhysicsJoint2D</c> for a <see cref="PhysicsJointBreakAction2D.Destroy"/>/<c>Disable</c> action — is
    /// done by <c>PhysicsJoint2DBreakSystem</c>, which runs <c>[UpdateAfter]</c> the world system where a
    /// structural change is legal. A <see cref="PhysicsJointBreakAction2D.CallbackOnly"/> event leaves the joint
    /// in place. The <see cref="breakAction"/> is carried so a consumer can mirror the built-in
    /// destroy-the-entity vs disable-the-component distinction Box2D itself cannot express.
    ///
    /// <b>Validity window.</b> Same as the contact/trigger buffers: valid for any system that runs after
    /// <c>PhysicsWorld2DSystem</c> within the same fixed tick (and before the next tick clears it). Read with
    /// <c>SystemAPI.GetSingletonBuffer&lt;PhysicsJointBreakEvent2D&gt;(isReadOnly: true)</c>.
    /// </remarks>
    public struct PhysicsJointBreakEvent2D : IBufferElementData
    {
        /// <summary>The owning entity of the broken joint (resolved from <c>joint.userData</c>), or
        /// <see cref="Entity.Null"/> if unresolved.</summary>
        public Entity jointEntity;

        /// <summary>The raw broken joint handle. Valid only this frame (it may be destroyed this tick by
        /// <c>PhysicsJoint2DBreakSystem</c>).</summary>
        public PhysicsJoint joint;

        /// <summary>The action the joint was baked with — what the package does (and what a consumer should
        /// mirror).</summary>
        public PhysicsJointBreakAction2D breakAction;
    }

    /// <summary>
    /// A zero-size tag added to an entity whose joint has broken (action <see cref="PhysicsJointBreakAction2D.Destroy"/>
    /// or <c>Disable</c>), so the joint-creation query never re-creates the joint. The creation query keys on
    /// <c>WithAll&lt;PhysicsJoint2DDefinition&gt; WithNone&lt;PhysicsJoint2D&gt;</c>; a broken entity still carries
    /// its definition, so without this tag it would re-form next update. With the tag the broken state is sticky,
    /// matching GameObject (a destroyed/disabled joint does not silently re-form).
    /// </summary>
    public struct PhysicsJoint2DBroken : IComponentData { }
}
