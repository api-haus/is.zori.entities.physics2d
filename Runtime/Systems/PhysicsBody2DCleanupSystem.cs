using Unity.Collections;
using Unity.Entities;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Frees the Box2D body of every destroyed physics entity, so a body's lifetime tracks its entity's. When
    /// an entity carrying a physics body is destroyed, ECS strips its regular components (including the
    /// <see cref="PhysicsBody2D"/> handle) but retains the <see cref="PhysicsBody2DCleanup"/> cleanup
    /// component — the surviving witness of the handle. This system finds those "ghost" entities
    /// (<c>WithAll&lt;PhysicsBody2DCleanup&gt;</c> and <c>WithNone&lt;PhysicsBody2D&gt;</c>), destroys their
    /// bodies in one bulk native call, and removes the cleanup component so ECS reclaims the entities.
    /// </summary>
    /// <remarks>
    /// <b>Mechanism.</b> The cleanup-component pattern, mirroring <c>com.unity.physics</c>'s
    /// <c>ColliderBlobCleanupSystem</c>: a destroyed entity becomes a ghost addressable through its retained
    /// cleanup component until the last cleanup component is removed. The ghost query is the destroyed-entity
    /// set; a still-live entity has both <see cref="PhysicsBody2D"/> and <see cref="PhysicsBody2DCleanup"/> and
    /// is excluded by the <c>WithNone&lt;PhysicsBody2D&gt;</c> clause.
    ///
    /// <b>Bulk destroy + cascade.</b> <c>PhysicsBody.DestroyBatch(ReadOnlySpan&lt;PhysicsBody&gt;)</c> (static)
    /// destroys every body in one call and, per the module XML, destroys all shapes and joints attached to each
    /// body — so a destroyed body's lone shape and any joint it participates in (as owner or as a connected
    /// body) are freed by the same call, with no separate shape/joint teardown needed. Invalid handles
    /// (already freed, or a handle invalidated by a physics-module reset that recreated the world) are ignored
    /// by the API, so the call is safe even when the world is gone.
    ///
    /// <b>Ordering — before the step.</b> Runs <c>[UpdateBefore(PhysicsWorld2DSystem)]</c> in
    /// <see cref="FixedStepSimulationSystemGroup"/> so a body destroyed on any frame is freed at the top of the
    /// next fixed step, before <see cref="PhysicsWorld2DSystem"/>'s <c>Simulate</c> would integrate it — a dead
    /// entity is therefore stepped zero further times once cleanup runs. (Write-back already never touches a
    /// ghost: it queries <c>WithAll&lt;PhysicsBody2D, LocalToWorld&gt;</c> and the ghost has neither.)
    ///
    /// <b>Not <c>[BurstCompile]</c>.</b> <c>PhysicsBody.DestroyBatch</c> is a managed <c>Unity.U2D.Physics</c>
    /// call on the main thread, exactly like the body/shape/joint creation in the sibling systems — Burst is
    /// confined to the write-back job. <c>OnDestroy</c> drains any ghosts still pending at world teardown the
    /// same way, the orderly no-warning path (harmless if <see cref="PhysicsWorld2DSystem"/> tore the world
    /// down first, since <c>DestroyBatch</c> ignores the now-invalid handles).
    /// </remarks>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(PhysicsWorld2DSystem))]
    public partial struct PhysicsBody2DCleanupSystem : ISystem
    {
        EntityQuery _ghostQuery;

        public void OnCreate(ref SystemState state)
        {
            _ghostQuery = SystemAPI
                .QueryBuilder()
                .WithAll<PhysicsBody2DCleanup>()
                .WithNone<PhysicsBody2D>()
                .Build();
            state.RequireForUpdate(_ghostQuery);
        }

        public void OnUpdate(ref SystemState state)
        {
            DestroyGhostBodies(ref state, _ghostQuery);
            // Remove the last cleanup component from every ghost so ECS can reclaim the entities.
            state.EntityManager.RemoveComponent<PhysicsBody2DCleanup>(_ghostQuery);
        }

        public void OnDestroy(ref SystemState state)
        {
            // World teardown: free any bodies whose entity was destroyed but not yet cleaned up. The world is
            // also torn down by PhysicsWorld2DSystem.OnDestroy (which frees everything); this is the orderly
            // path and a no-op if the world went first (DestroyBatch ignores the invalid handles).
            var query = SystemAPI
                .QueryBuilder()
                .WithAll<PhysicsBody2DCleanup>()
                .WithNone<PhysicsBody2D>()
                .Build();
            if (query.IsEmpty)
                return;
            DestroyGhostBodies(ref state, query);
            state.EntityManager.RemoveComponent<PhysicsBody2DCleanup>(query);
        }

        // Collect the ghost bodies' handles and free them in one bulk PhysicsBody.DestroyBatch call (which
        // cascades to each body's shapes and joints). Managed Unity.U2D.Physics call, main thread — not Burst.
        static void DestroyGhostBodies(ref SystemState state, EntityQuery ghostQuery)
        {
            var count = ghostQuery.CalculateEntityCount();
            if (count == 0)
                return;

            using var cleanups = ghostQuery.ToComponentDataArray<PhysicsBody2DCleanup>(Allocator.Temp);
            var bodies = new NativeArray<PhysicsBody>(count, Allocator.Temp);
            for (var i = 0; i < count; i++)
                bodies[i] = cleanups[i].body;
            // DestroyBatch is a static method on PhysicsBody (the world is implied by the handles); it destroys
            // every body's attached shapes and joints too, and ignores any handle that is already invalid.
            PhysicsBody.DestroyBatch(bodies.AsReadOnlySpan());
            bodies.Dispose();
        }
    }
}
