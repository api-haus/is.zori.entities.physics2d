using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// A one-shot request to bulk-create many <em>identical</em> dynamic circle bodies in a single native
    /// <c>PhysicsWorld.CreateBodyBatch</c> call — the low-level optimization opportunity the per-entity bake
    /// path is deliberately not optimized for. The per-entity creation loop in <c>PhysicsWorld2DSystem</c>
    /// issues one <c>CreateBody</c> + <c>CreateShape</c> per entity, which is correct when baked entities
    /// arrive heterogeneously through a query but wasteful when a workload spawns N copies of one body in
    /// one frame (a sand-grain burst, a particle volley). This request carries ONE shared definition + one
    /// shape + a count; <c>PhysicsBody2DBatchCreationSystem</c> turns it into N live bodies created in one
    /// batch call and N entities the step + write-back already consume.
    /// </summary>
    /// <remarks>
    /// Scoped to the identical-body case on purpose: <c>CreateBodyBatch(def, count, allocator)</c> takes a
    /// single <c>PhysicsBodyDefinition</c>, so every body in a batch shares type/damping/gravity. The shape
    /// is a circle (the common burst primitive and the POC's batched shape); a heterogeneous spawn uses the
    /// per-entity bake/direct path instead. Start positions are scattered deterministically across the
    /// AABB <c>[spawnMin, spawnMax]</c> via one <c>SetBatchTransform</c> call seeded by <see cref="seed"/>,
    /// so a batch of identical bodies does not stack at one point.
    /// </remarks>
    public struct PhysicsBody2DBatchRequest : IComponentData
    {
        /// <summary>How many identical bodies to create in the one batch call.</summary>
        public int count;

        /// <summary>Shared body type for every body in the batch (typically Dynamic).</summary>
        public PhysicsBody.BodyType bodyType;

        /// <summary>Shared gravity scale.</summary>
        public float gravityScale;

        /// <summary>Shared linear damping.</summary>
        public float linearDamping;

        /// <summary>Shared angular damping.</summary>
        public float angularDamping;

        /// <summary>Shared circle radius for every body's single shape.</summary>
        public float radius;

        /// <summary>Shared per-shape density (drives auto mass). A value &lt;= 0 keeps Box2D's default.</summary>
        public float density;

        /// <summary>Deterministic-scatter AABB minimum corner (world space).</summary>
        public float2 spawnMin;

        /// <summary>Deterministic-scatter AABB maximum corner (world space).</summary>
        public float2 spawnMax;

        /// <summary>RNG seed for the deterministic scatter, so a batch reproduces across runs.</summary>
        public uint seed;

        /// <summary>
        /// Shared Box2D contact-filter category bits for every body's shape (which categories the batch is in).
        /// A value of <c>0</c> (the default) keeps the everything-default filter, so a request that sets no
        /// filter collides with everything exactly as the batch path historically did. To make a batch filter
        /// by layer, set this to <c>1 &lt;&lt; layer</c> and <see cref="contactBits"/> to the matrix row.
        /// </summary>
        public ulong categoryBits;

        /// <summary>
        /// Shared Box2D contact-filter contacts bits (which categories the batch contacts with). Used only when
        /// <see cref="categoryBits"/> is non-zero; otherwise the everything-default is applied. Typically
        /// <c>Physics2D.GetLayerCollisionMask(layer)</c> captured at spawn time.
        /// </summary>
        public ulong contactBits;

        /// <summary>
        /// Whether every body's shape in the batch is a sensor (a trigger). Default false (solid, responding
        /// shapes). A trigger batch overlaps without a collision response and produces
        /// <see cref="PhysicsTriggerEvent2D"/>s rather than <see cref="PhysicsContactEvent2D"/>s.
        /// </summary>
        public bool isTrigger;
    }
}
