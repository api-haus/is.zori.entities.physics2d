using Unity.Entities;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The retained copy of an entity's low-level <see cref="PhysicsBody"/> handle that survives the entity's
    /// destruction, so its Box2D body can be freed after the entity is gone. Added in the same step as
    /// <see cref="PhysicsBody2D"/> at body creation; because this is an <see cref="ICleanupComponentData"/>,
    /// ECS strips the regular <see cref="PhysicsBody2D"/> on <c>DestroyEntity</c> but <em>retains</em> this one,
    /// leaving a "ghost" entity (cleanup component present, <see cref="PhysicsBody2D"/> absent) that
    /// <c>PhysicsBody2DCleanupSystem</c> finds, uses to destroy the body, and then removes — which finally lets
    /// ECS reclaim the entity.
    /// </summary>
    /// <remarks>
    /// Kept a separate component from <see cref="PhysicsBody2D"/> on purpose: <see cref="PhysicsBody2D"/>'s
    /// presence/absence is the "already created" / "live body" marker three existing queries depend on
    /// (<c>WithNone&lt;PhysicsBody2D&gt;</c> in the creation loop, <c>WithAll&lt;PhysicsBody2D&gt;</c> in
    /// write-back, the joint-readiness <c>ComponentLookup&lt;PhysicsBody2D&gt;</c>), so it must keep being
    /// stripped on destroy exactly as before. The cleanup component carries the handle witness without
    /// touching those semantics — the same split <c>com.unity.physics</c> uses (regular <c>PhysicsCollider</c>
    /// + separate cleanup <c>ColliderBlobCleanupData</c>). <see cref="PhysicsBody"/> is a blittable 64-bit-ID
    /// struct, so this is a managed-free <see cref="ICleanupComponentData"/>.
    /// </remarks>
    public struct PhysicsBody2DCleanup : ICleanupComponentData
    {
        public PhysicsBody body;
    }
}
