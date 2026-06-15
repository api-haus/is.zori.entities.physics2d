using Unity.Entities;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Holds the one <see cref="PhysicsWorld"/> this ECS <c>World</c> owns, on a singleton entity created
    /// by <c>PhysicsWorld2DSystem.OnCreate</c>. Other systems reach the world through this singleton.
    /// </summary>
    /// <remarks>
    /// <see cref="PhysicsWorld"/> is a 64-bit-ID struct, so the singleton is managed-free. Destroying the
    /// world (in <c>OnDestroy</c>) destroys every body and shape it contains, so there is no per-body
    /// teardown <em>at world end</em>. Per-entity body teardown during a session (a body freed when its
    /// entity is despawned) is a separate concern handled by <c>PhysicsBody2DCleanupSystem</c>.
    /// </remarks>
    public struct PhysicsWorldSingleton2D : IComponentData
    {
        public PhysicsWorld world;
    }
}
