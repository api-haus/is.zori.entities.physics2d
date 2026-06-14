using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The low-level direct-authoring surface: set the package's runtime components
    /// (<see cref="PhysicsBody2DDefinition"/> + <see cref="PhysicsShape2D"/>) on an entity straight from
    /// code — no MonoBehaviour, no <c>Baker&lt;T&gt;</c>, no built-in <c>Rigidbody2D</c>/<c>Collider2D</c>.
    /// An entity authored this way flows through the SAME <c>PhysicsWorld2DSystem</c> creation loop as a
    /// baked one (the loop keys on those two components plus <c>WithNone&lt;PhysicsBody2D&gt;</c>), so it
    /// gets a Box2D body, steps, and writes <see cref="LocalToWorld"/> identically. This is the
    /// dual-surface convergence at the code level: bake and direct-author produce one runtime archetype.
    /// </summary>
    /// <remarks>
    /// The helper adds <see cref="LocalToWorld"/> too, with the matrix at the authored pose — a baked entity
    /// gets it from <c>TransformUsageFlags.Dynamic</c>, but a from-scratch entity has no transform baking, so
    /// the direct surface must add it (the write-back system requires it and overwrites it each step). Use
    /// the <see cref="EntityManager"/> overload for immediate structural changes (a bootstrap system or a
    /// test), or the <see cref="EntityCommandBuffer"/> overload to author inside a job/query where structural
    /// change must be deferred.
    /// </remarks>
    public static class DirectPhysics2DAuthoring
    {
        /// <summary>
        /// Author the body+shape components on an existing entity via the <see cref="EntityManager"/>
        /// (immediate). The entity is left without a <see cref="PhysicsBody2D"/>, so the next
        /// <c>PhysicsWorld2DSystem</c> update creates the Box2D body for it.
        /// </summary>
        public static void Author(
            EntityManager entityManager,
            Entity entity,
            in PhysicsBody2DDefinition body,
            in PhysicsShape2D shape
        )
        {
            entityManager.AddComponentData(entity, body);
            entityManager.AddComponentData(entity, shape);
            entityManager.AddComponentData(entity, LocalToWorldAt(body));
        }

        /// <summary>Create a new entity and author the body+shape components on it via the
        /// <see cref="EntityManager"/> (immediate). Returns the new entity.</summary>
        public static Entity Create(
            EntityManager entityManager,
            in PhysicsBody2DDefinition body,
            in PhysicsShape2D shape
        )
        {
            var entity = entityManager.CreateEntity();
            Author(entityManager, entity, body, shape);
            return entity;
        }

        /// <summary>
        /// Author the body+shape components on an entity via an <see cref="EntityCommandBuffer"/> (deferred),
        /// for use inside a job/query where an immediate structural change is illegal.
        /// </summary>
        public static void Author(
            EntityCommandBuffer ecb,
            Entity entity,
            in PhysicsBody2DDefinition body,
            in PhysicsShape2D shape
        )
        {
            ecb.AddComponent(entity, body);
            ecb.AddComponent(entity, shape);
            ecb.AddComponent(entity, LocalToWorldAt(body));
        }

        /// <summary>
        /// The flat <see cref="LocalToWorld"/> at a body's authored pose — column-major, Z-rotation only,
        /// matching the matrix the write-back job builds from a <c>PhysicsBody.BatchTransform</c>. Seeds the
        /// component so the entity has a valid transform before its first step writes the post-step pose.
        /// </summary>
        static LocalToWorld LocalToWorldAt(in PhysicsBody2DDefinition body)
        {
            math.sincos(body.initialRotationRadians, out var s, out var c);
            var p = body.initialPosition;
            return new LocalToWorld
            {
                Value = new float4x4(c, -s, 0f, p.x, s, c, 0f, p.y, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f),
            };
        }
    }
}
