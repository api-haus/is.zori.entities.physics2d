using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Computes the <see cref="PhysicsBody2DFormHash"/> for every baked body entity, as a baking-world system
    /// ordered AFTER the per-component bakers. It folds the body-form fields of <see cref="PhysicsBody2DDefinition"/>
    /// and the shape fields of <see cref="PhysicsShape2D"/> (plus every extra <see cref="PhysicsShape2DElement"/>
    /// and, for Polygon/Edge, the vertex blob content) into a stable 128-bit content hash, with POSE and the
    /// initial-velocity seed excluded, and adds the hash component so it replicates to every
    /// <c>ecb.Instantiate(prefab)</c> instance for free.
    /// </summary>
    /// <remarks>
    /// <para><b>Why a system, not a baker.</b> <c>Rigidbody2DBaker</c> produces the body definition and the
    /// collider bakers produce the shape; no single <c>Baker&lt;T&gt;</c> sees both on one entity. Reading the
    /// already-baked runtime components here means the built-in, custom-authoring, and direct-baker paths all
    /// produce the same hash for the same form without re-reading authoring data.</para>
    ///
    /// <para>Placed in <c>PostBakingSystemGroup</c> (which carries the <c>BakingSystem</c> world filter), so it
    /// runs in the baking world after the component bakers have written the body+shape components. It is idempotent
    /// — re-baking recomputes the same hash from the same components.</para>
    ///
    /// <para>Not <c>[BurstCompile]</c>: it folds a managed loop over the (possibly multi-shape) entity and reads a
    /// blob, on the main thread, like the rest of the baking code. The scalar form fields are packed into one
    /// <c>unmanaged</c> struct and hashed via <c>xxHash3.Hash128(in struct)</c> — that method is <c>unsafe</c> but
    /// takes a managed <c>in T</c>, so calling it needs no <c>allowUnsafeCode</c> here. Polygon/Edge vertices are
    /// mixed in one at a time so two forms differing only in their outline hash differently.</para>
    /// </remarks>
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    [UpdateInGroup(typeof(PostBakingSystemGroup))]
    public partial struct PhysicsBody2DFormHashBakingSystem : ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            foreach (
                var (defRO, shapeRO, entity) in SystemAPI
                    .Query<RefRO<PhysicsBody2DDefinition>, RefRO<PhysicsShape2D>>()
                    .WithEntityAccess()
                    .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)
            )
            {
                var value = ComputeFormHash(in defRO.ValueRO, in shapeRO.ValueRO);

                // Multi-shape bodies: fold every extra shape's fields into the same hash, in buffer order, so a
                // body with a different shape-1 is a different form. A single-shape body has no buffer (no-op).
                if (SystemAPI.HasBuffer<PhysicsShape2DElement>(entity))
                {
                    var extra = SystemAPI.GetBuffer<PhysicsShape2DElement>(entity);
                    for (var i = 0; i < extra.Length; i++)
                    {
                        var element = extra[i].shape;
                        value = MixShape(value, in element);
                    }
                }

                ecb.AddComponent(entity, new PhysicsBody2DFormHash { value = value });
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        // The hash of a body+primary-shape form: pack the pose-free scalar fields into one unmanaged struct,
        // hash it, then mix in any variable-length (Polygon/Edge) vertex blob.
        static uint4 ComputeFormHash(in PhysicsBody2DDefinition d, in PhysicsShape2D sh)
        {
            var fields = new FormFields
            {
                bodyType = (int)d.bodyType,
                gravityScale = d.gravityScale,
                linearDamping = d.linearDamping,
                angularDamping = d.angularDamping,
                constraints = (int)d.constraints,
                mass = d.mass,
                useAutoMass = d.useAutoMass ? 1 : 0,
                fastCollisions = d.fastCollisions ? 1 : 0,
                interpolation = (int)d.interpolation,
                overrideMassDistribution = d.overrideMassDistribution ? 1 : 0,
                centerOfMass = d.centerOfMass,
                rotationalInertia = d.rotationalInertia,
            };
            var value = xxHash3.Hash128(in fields);
            return MixShape(value, in sh);
        }

        // Fold one shape's fields (and, for Polygon/Edge, its vertex blob) into a running hash. The primary shape
        // and every buffer element pass through here, so identical handling and offset folding apply uniformly.
        static uint4 MixShape(uint4 seed, in PhysicsShape2D sh)
        {
            var fields = new ShapeFields
            {
                seed = seed,
                kind = (int)sh.kind,
                offset = sh.offset,
                radius = sh.radius,
                size = sh.size,
                boxAngleRadians = sh.boxAngleRadians,
                capsuleCenter1 = sh.capsuleCenter1,
                capsuleCenter2 = sh.capsuleCenter2,
                edgeIsLoop = sh.edgeIsLoop ? 1 : 0,
                polygonDecompose = sh.polygonDecompose ? 1 : 0,
                friction = sh.friction,
                bounciness = sh.bounciness,
                density = sh.density,
                frictionMixing = (int)sh.frictionMixing,
                bouncinessMixing = (int)sh.bouncinessMixing,
                categoryBits = sh.categoryBits,
                contactBits = sh.contactBits,
                isTrigger = sh.isTrigger ? 1 : 0,
            };
            var value = xxHash3.Hash128(in fields);

            // Variable-length kinds: fold each vertex into the hash so two outlines that differ produce different
            // forms (and so the inline kinds, which never created the blob, are unaffected — IsCreated is false).
            if (
                (sh.kind == PhysicsShape2DKind.Polygon || sh.kind == PhysicsShape2DKind.Edge)
                && sh.vertices.IsCreated
            )
            {
                ref var points = ref sh.vertices.Value.points;
                for (var i = 0; i < points.Length; i++)
                {
                    var vtx = new VertexFold { seed = value, vertex = points[i] };
                    value = xxHash3.Hash128(in vtx);
                }
            }

            return value;
        }

        // The pose-free body-form scalar fields, packed for one hash call. Excludes initialPosition,
        // initialRotationRadians (per-instance pose) and the PhysicsBody2DInitialVelocity seed (per-instance
        // launch), exactly as the design's §1.1 prescribes.
        struct FormFields
        {
            public int bodyType;
            public float gravityScale;
            public float linearDamping;
            public float angularDamping;
            public int constraints;
            public float mass;
            public int useAutoMass;
            public int fastCollisions;
            public int interpolation;
            public int overrideMassDistribution;
            public float2 centerOfMass;
            public float rotationalInertia;
        }

        // One shape's fields plus the running seed, packed for one hash call. offset IS folded (it changes the
        // created geometry, so two shapes differing only in offset are genuinely different forms).
        struct ShapeFields
        {
            public uint4 seed;
            public int kind;
            public float2 offset;
            public float radius;
            public float2 size;
            public float boxAngleRadians;
            public float2 capsuleCenter1;
            public float2 capsuleCenter2;
            public int edgeIsLoop;
            public int polygonDecompose;
            public float friction;
            public float bounciness;
            public float density;
            public int frictionMixing;
            public int bouncinessMixing;
            public ulong categoryBits;
            public ulong contactBits;
            public int isTrigger;
        }

        // One vertex mixed into the running hash for a Polygon/Edge outline.
        struct VertexFold
        {
            public uint4 seed;
            public float2 vertex;
        }
    }
}
