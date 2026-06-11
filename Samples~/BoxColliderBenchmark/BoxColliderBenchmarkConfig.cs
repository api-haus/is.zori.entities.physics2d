using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D.Samples
{
    /// <summary>
    /// Scene-authored opt-in for the box-collider creation benchmark. Add this singleton to a scene (from a
    /// bootstrap system, a tiny authoring MonoBehaviour + baker, or a directly-authored entity) and
    /// <see cref="BoxColliderBenchmarkSpawnerSystem"/> spray-spawns quads with box colliders rendered by
    /// Unity.Entities.Graphics, while <see cref="BodyCreationTimingSystem"/> times the per-entity creation cost.
    /// </summary>
    /// <remarks>
    /// The spray is paced by three independent workload knobs (all inspector-editable on
    /// <c>BoxColliderBenchmarkAuthoring</c>):
    /// <list type="bullet">
    /// <item><see cref="spawnedPerSecondTarget"/> — the target spray RATE. Each frame the spawner spawns
    /// <c>round(spawnedPerSecondTarget * dt)</c> (carrying the fractional remainder across frames so a low rate
    /// still spawns exactly), so this sets the pace independent of frame rate.</item>
    /// <item><see cref="spawnedPerFrameMax"/> — a hard CEILING on instances created in one frame, bounding the
    /// per-frame structural-change cost regardless of how high the rate or how long a frame stalls.</item>
    /// <item><see cref="spawnedTotalLimit"/> — the run STOPS once this many have been created. Supports up to
    /// ~1,000,000 for an on-screen stress test (deck-only for real timing).</item>
    /// </list>
    /// Per frame the spawner creates <c>min(spawnedPerFrameMax, round(spawnedPerSecondTarget * dt) + carry)</c>,
    /// clamped so the running total never exceeds <see cref="spawnedTotalLimit"/>.
    ///
    /// <para>The dedup on/off + threshold control is NOT here — it lives on the scene's
    /// <c>PhysicsStep2DAuthoring</c> (<c>CacheIdenticalBodies</c> / <c>IdenticalBodyThreshold</c>, baked into
    /// <c>PhysicsWorld2DConfig</c>), the package's existing control surface. This config carries only the workload
    /// shape. To sweep the optimisation, toggle / re-author the <c>PhysicsStep2DAuthoring</c> between runs and
    /// re-read the timing log; the spray + timing are identical across the sweep so only the creation path
    /// differs.</para>
    /// </remarks>
    public struct BoxColliderBenchmarkConfig : IComponentData
    {
        /// <summary>The run stops once this many quads have been created. Supports up to
        /// <see cref="MaxTotalLimit"/> (~1M) for an on-screen stress test. <c>&lt;= 0</c> defaults to
        /// <see cref="DefaultTotalLimit"/>.</summary>
        public int spawnedTotalLimit;

        /// <summary>Target spray rate, quads per second — the pace. The spawner spawns
        /// <c>round(spawnedPerSecondTarget * dt)</c> per frame (with a carried fractional remainder), so the rate
        /// is frame-rate independent. <c>&lt;= 0</c> defaults to <see cref="DefaultPerSecondTarget"/>.</summary>
        public float spawnedPerSecondTarget;

        /// <summary>Hard ceiling on quads instantiated in one frame — bounds the per-frame structural-change cost.
        /// The spawner never creates more than this in a single frame even if the rate × dt asks for more.
        /// <c>&lt;= 0</c> defaults to <see cref="DefaultPerFrameMax"/>.</summary>
        public int spawnedPerFrameMax;

        /// <summary>Full extents of the box collider + quad mesh (the package's Box kind). <c>&lt;= 0</c>
        /// defaults to <see cref="DefaultBoxSize"/>.</summary>
        public float2 boxSize;

        /// <summary>The spawn-AABB minimum corner (each instance is scattered to a distinct pose across it).</summary>
        public float2 spawnMin;

        /// <summary>The spawn-AABB maximum corner.</summary>
        public float2 spawnMax;

        /// <summary>The supported upper bound for <see cref="spawnedTotalLimit"/> — the on-screen stress ceiling
        /// the workload is built to reach (deck-only for real timing).</summary>
        public const int MaxTotalLimit = 1_000_000;

        /// <summary>The default <see cref="spawnedPerFrameMax"/> when the field is left <c>&lt;= 0</c>.</summary>
        public const int DefaultPerFrameMax = 256;

        /// <summary>The default <see cref="spawnedPerSecondTarget"/> when the field is left <c>&lt;= 0</c>.</summary>
        public const float DefaultPerSecondTarget = 1000f;

        /// <summary>The default <see cref="spawnedTotalLimit"/> when the field is left <c>&lt;= 0</c>.</summary>
        public const int DefaultTotalLimit = 4096;

        /// <summary>The default box full-extent when <see cref="boxSize"/> is left <c>&lt;= 0</c>.</summary>
        public static readonly float2 DefaultBoxSize = new float2(0.4f, 0.4f);
    }
}
