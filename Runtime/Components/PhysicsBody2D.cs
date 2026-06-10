using Unity.Entities;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The live association between an entity and its low-level <see cref="PhysicsBody"/> handle, added
    /// by <c>PhysicsWorld2DSystem</c> at body creation (not by a baker). Its presence is also the
    /// "already created" marker the creation query filters on with <c>WithNone&lt;PhysicsBody2D&gt;</c>.
    /// A copy of the handle is also stored in the <see cref="PhysicsBody2DCleanup"/> cleanup component so the
    /// body can be freed when the entity is destroyed (this regular component is stripped on destroy, the
    /// cleanup copy is retained — see <c>PhysicsBody2DCleanupSystem</c>).
    /// </summary>
    /// <remarks>
    /// <see cref="PhysicsBody"/> is a 64-bit-ID struct — blittable and storable in an
    /// <see cref="IComponentData"/>, and usable inside a Burst job (the POC stores a
    /// <c>NativeArray&lt;PhysicsBody&gt;</c> and reads it from a job).
    /// </remarks>
    public struct PhysicsBody2D : IComponentData
    {
        public PhysicsBody body;
    }
}
