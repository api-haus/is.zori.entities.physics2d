using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D.Samples
{
    /// <summary>
    /// Scene-authored opt-in for the box-collider creation benchmark. Add this singleton to a scene (from a
    /// bootstrap system, a tiny authoring MonoBehaviour + baker, or a directly-authored entity) and
    /// <see cref="BoxColliderBenchmarkSpawnerSystem"/> spray-spawns <see cref="count"/> quads with box colliders
    /// (<see cref="perFrame"/> per frame, a cross-frame spray) rendered by Unity.Entities.Graphics, while
    /// <see cref="BodyCreationTimingSystem"/> times the per-entity creation cost.
    /// </summary>
    /// <remarks>
    /// The on/off + threshold control is NOT here — it lives on the scene's <c>PhysicsStep2DAuthoring</c>
    /// (<c>CacheIdenticalBodies</c> / <c>IdenticalBodyThreshold</c>, baked into <c>PhysicsWorld2DConfig</c>),
    /// the package's existing control surface. This config carries only the workload shape. To sweep the
    /// optimisation, toggle / re-author the <c>PhysicsStep2DAuthoring</c> between runs and re-read the timing
    /// log; the spray + timing are identical across the sweep so only the creation path differs.
    /// </remarks>
    public struct BoxColliderBenchmarkConfig : IComponentData
    {
        /// <summary>Total quads to spray over the run. A swept N (1k / 4k / 16k for the deck benchmark).</summary>
        public int count;

        /// <summary>Quads instantiated per frame — a cross-frame spray, not a single bulk burst. The mechanism
        /// under test is the cross-frame cached template, so this stays small (e.g. 8–64) and the spray spans
        /// many frames. <c>&lt;= 0</c> defaults to <see cref="DefaultPerFrame"/>.</summary>
        public int perFrame;

        /// <summary>Full extents of the box collider + quad mesh (the package's Box kind). <c>&lt;= 0</c>
        /// defaults to <see cref="DefaultBoxSize"/>.</summary>
        public float2 boxSize;

        /// <summary>The spawn-AABB minimum corner (each instance is scattered to a distinct pose across it).</summary>
        public float2 spawnMin;

        /// <summary>The spawn-AABB maximum corner.</summary>
        public float2 spawnMax;

        /// <summary>The default <see cref="perFrame"/> when the field is left <c>&lt;= 0</c>.</summary>
        public const int DefaultPerFrame = 16;

        /// <summary>The default total <see cref="count"/> when the field is left <c>&lt;= 0</c>.</summary>
        public const int DefaultCount = 1024;

        /// <summary>The default box full-extent when <see cref="boxSize"/> is left <c>&lt;= 0</c>.</summary>
        public static readonly float2 DefaultBoxSize = new float2(0.4f, 0.4f);
    }
}
