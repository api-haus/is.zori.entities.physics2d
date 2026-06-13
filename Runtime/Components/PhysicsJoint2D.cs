using Unity.Entities;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The live association between an entity and its low-level <see cref="PhysicsJoint"/> handle, added by
    /// <c>PhysicsJoint2DCreationSystem</c> once the joint exists (not by a baker). Its presence is also the
    /// "already created" marker the creation query filters on with <c>WithNone&lt;PhysicsJoint2D&gt;</c> —
    /// exactly the role <see cref="PhysicsBody2D"/> plays for bodies.
    /// </summary>
    /// <remarks>
    /// <see cref="PhysicsJoint"/> is a 64-bit-ID struct — blittable and storable in an
    /// <see cref="IComponentData"/>. The handle is what the creation system collects into the
    /// <c>DestroyJointBatch</c> span at world teardown, and what per-joint break/runtime-tune reads
    /// to mutate or destroy a single joint.
    /// </remarks>
    public struct PhysicsJoint2D : IComponentData
    {
        public PhysicsJoint joint;
    }
}
