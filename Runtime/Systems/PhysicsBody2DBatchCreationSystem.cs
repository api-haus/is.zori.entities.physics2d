using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Consumes <see cref="PhysicsBody2DBatchRequest"/>s by bulk-creating N identical circle bodies in one
    /// <c>PhysicsWorld.CreateBodyBatch</c> native call (the low-level optimization), scattering their start
    /// positions in one <c>SetBatchTransform</c> call, and spawning N entities — each carrying a live
    /// <see cref="PhysicsBody2D"/> + <see cref="LocalToWorld"/> — that the step + write-back systems already
    /// drive. The created entities deliberately do NOT carry <see cref="PhysicsBody2DDefinition"/> /
    /// <see cref="PhysicsShape2D"/>, so the per-entity creation loop in <see cref="PhysicsWorld2DSystem"/>
    /// never double-creates a body for them.
    /// </summary>
    /// <remarks>
    /// Ordered after <see cref="PhysicsWorld2DSystem"/> (so the world singleton exists and is valid) and
    /// before <see cref="PhysicsBody2DWriteBackSystem"/> (so the new bodies' poses flow out the same frame
    /// they are created — though their <see cref="LocalToWorld"/> is also seeded at the spawn pose, so a
    /// frame's delay would be harmless). Not <c>[BurstCompile]</c>: the batch calls are managed
    /// <c>Unity.U2D.Physics</c> methods on the main thread.
    ///
    /// <para><b>Why the per-body <c>CreateShape</c> loop, not <c>CreateShapeBatch</c>.</b>
    /// <c>PhysicsBody.CreateShapeBatch(span, def, allocator)</c> creates N shapes on ONE body (a compound
    /// collider), which is the wrong primitive for "N bodies, one shape each." So the bulk win is the single
    /// <c>CreateBodyBatch</c> body-creation call (replacing N <c>CreateBody</c> calls) and the single
    /// <c>SetBatchTransform</c> scatter; each body's lone shape is attached in a tight loop, exactly as the
    /// POC does (<c>SpriteAccelerator.CreateBodies</c>).</para>
    /// </remarks>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsWorld2DSystem))]
    [UpdateBefore(typeof(PhysicsBody2DWriteBackSystem))]
    public partial struct PhysicsBody2DBatchCreationSystem : ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.TryGetSingleton<PhysicsWorldSingleton2D>(out var singleton))
                return;
            var world = singleton.world;
            if (!world.isValid)
                return;

            // Snapshot the pending requests into a temp array, then consume + clear them in one ECB before
            // creating any batch entities. The batch entities are created with the EntityManager directly
            // (not deferred via the ECB) AFTER this query is fully drained, because each body's userData must
            // pack its REAL owning entity for shape→entity query resolution — a deferred ECB entity has a
            // placeholder index/version that would not match the post-playback entity.
            var requestQuery = SystemAPI.QueryBuilder().WithAll<PhysicsBody2DBatchRequest>().Build();
            if (requestQuery.IsEmpty)
                return;

            var requests = requestQuery.ToComponentDataArray<PhysicsBody2DBatchRequest>(Allocator.Temp);

            // Consume every request entity regardless of outcome so a malformed request cannot loop forever.
            state.EntityManager.DestroyEntity(requestQuery);

            for (var r = 0; r < requests.Length; r++)
            {
                if (requests[r].count <= 0)
                    continue;
                CreateBatch(ref state, world, requests[r]);
            }

            requests.Dispose();
        }

        static void CreateBatch(
            ref SystemState state,
            PhysicsWorld world,
            in PhysicsBody2DBatchRequest request
        )
        {
            // 1) One native call creates every body in the batch from the single shared definition.
            var bodyDef = PhysicsBodyDefinition.defaultDefinition;
            bodyDef.type = request.bodyType;
            bodyDef.gravityScale = request.gravityScale;
            bodyDef.linearDamping = request.linearDamping;
            bodyDef.angularDamping = request.angularDamping;
            // Locked DOTS posture: poses move via GetBatchTransform, never a managed Transform write.
            bodyDef.transformWriteMode = PhysicsBody.TransformWriteMode.Off;
            var bodies = world.CreateBodyBatch(bodyDef, request.count, Allocator.Persistent);

            // 2) Attach the one shared circle shape to each body (CreateShapeBatch is N-shapes-on-one-body,
            //    the wrong primitive here — see remarks).
            var shapeDef = PhysicsShapeDefinition.defaultDefinition;
            if (request.density > 0f)
                shapeDef.density = request.density;
            // Shared contact filter from the request's layer-matrix bits. A zero category keeps the
            // everything-default (the batch path's historical behaviour), so a request that sets no bits
            // collides with everything exactly as before.
            if (request.categoryBits != 0ul)
            {
                var filter = PhysicsShape.ContactFilter.defaultFilter;
                filter.categories = new PhysicsMask { bitMask = request.categoryBits };
                filter.contacts = new PhysicsMask { bitMask = request.contactBits };
                shapeDef.contactFilter = filter;
            }
            // Trigger + event reporting, identical to the per-entity path in PhysicsWorld2DSystem: isTrigger from
            // the request, contact + trigger events on for every shape, startStaticContacts on (a no-op for the
            // batch's Dynamic bodies, but kept uniform with the per-entity path).
            shapeDef.isTrigger = request.isTrigger;
            shapeDef.contactEvents = true;
            shapeDef.triggerEvents = true;
            shapeDef.startStaticContacts = true;
            var geometry = new CircleGeometry { radius = request.radius };
            for (var i = 0; i < bodies.Length; i++)
                bodies[i].CreateShape(geometry, shapeDef);

            // 3) One batched transform write scatters the bodies across the spawn AABB so identical bodies do
            //    not stack at a single point. Deterministic from the request seed.
            var rng = new Unity.Mathematics.Random(request.seed == 0u ? 0x9E3779B9u : request.seed);
            var transforms = new NativeArray<PhysicsBody.BatchTransform>(
                bodies.Length,
                Allocator.Temp
            );
            for (var i = 0; i < bodies.Length; i++)
            {
                var p = rng.NextFloat2(request.spawnMin, request.spawnMax);
                transforms[i] = new PhysicsBody.BatchTransform(bodies[i]) { position = (Vector2)p };
            }
            PhysicsBody.SetBatchTransform(transforms.AsReadOnlySpan());

            // 4) Spawn one entity per body carrying the live handle + a LocalToWorld at the scatter pose, so
            //    the write-back system finds it and the step advances it. These entities have no
            //    PhysicsBody2DDefinition/PhysicsShape2D, so PhysicsWorld2DSystem's creation loop skips them.
            //    Created with the EntityManager (immediate) so the entity is real now — its packed identity
            //    must match what a query resolves from the body's userData.
            var em = state.EntityManager;
            for (var i = 0; i < bodies.Length; i++)
            {
                // Local copy because PhysicsBody is a 64-bit-ID handle struct: a NativeArray indexer returns a
                // copy, so setting userData on `body` still routes to the same native body the handle names.
                var body = bodies[i];
                var entity = em.CreateEntity();
                em.AddComponentData(entity, new PhysicsBody2D { body = body });
                // The retained handle witness that survives the entity's destruction, so
                // PhysicsBody2DCleanupSystem can free this body when the entity is despawned.
                em.AddComponentData(entity, new PhysicsBody2DCleanup { body = body });
                var p = (float2)(Vector2)transforms[i].position;
                em.AddComponentData(entity, LocalToWorldAt(p));
                // Pack the real owning entity into the body's userData for shape→entity query resolution.
                body.userData = PhysicsQueries2D.PackEntity(entity);
            }

            transforms.Dispose();
            bodies.Dispose();
        }

        static LocalToWorld LocalToWorldAt(float2 p)
        {
            return new LocalToWorld
            {
                Value = new float4x4(
                    1f,
                    0f,
                    0f,
                    p.x,
                    0f,
                    1f,
                    0f,
                    p.y,
                    0f,
                    0f,
                    1f,
                    0f,
                    0f,
                    0f,
                    0f,
                    1f
                ),
            };
        }
    }
}
