using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Samples
{
    /// <summary>
    /// mara dev/test authoring for the box-collider benchmark: drop ONE on a GameObject in a SubScene and its
    /// baker (<c>BoxColliderBenchmarkBaker</c>, in the Editor-only Baking assembly) emits the
    /// <see cref="BoxColliderBenchmarkConfig"/> singleton that arms <see cref="BoxColliderBenchmarkSpawnerSystem"/>.
    /// This is the mara-side fixture for the package sample (which ships in <c>Samples~/BoxColliderBenchmark</c>),
    /// mirroring how <c>Assets/EntitiesPhysics2DFixture/</c> is the mara fixture for the CustomAuthoring2D sample.
    /// Pair it in the scene with a <c>PhysicsStep2DAuthoring</c> whose <c>Cache Identical Bodies</c> / <c>Identical
    /// Body Threshold</c> drive the optimisation under test.
    /// </summary>
    /// <remarks>
    /// The MonoBehaviour lives in an all-platforms assembly (so it is addable to a scene GameObject); the baker
    /// lives in a separate Editor-only assembly that references <c>Unity.Entities.Hybrid</c>, mirroring the
    /// package's own <c>Authoring</c> / <c>Baking</c> split. The baker reads the authored values through
    /// <see cref="ToConfig"/> rather than the private serialized fields directly, the way
    /// <c>PhysicsStep2DAuthoring.AsConfig</c> exposes its config.
    /// </remarks>
    [AddComponentMenu("Zori/Entities Physics 2D/Box Collider Benchmark")]
    [DisallowMultipleComponent]
    public sealed class BoxColliderBenchmarkAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Min(1)]
        [Tooltip(
            "Spawned total (limit): the run stops once this many quads have been created. Freely editable up to "
                + "~1,000,000 for an on-screen stress test (1M is deck-only for real timing; the demo default is a "
                + "watchable few thousand)."
        )]
        int m_SpawnedTotalLimit = BoxColliderBenchmarkConfig.DefaultTotalLimit;

        [SerializeField]
        [Min(0f)]
        [Tooltip(
            "Spawned per second (target): the spray RATE. Each frame the spawner creates "
                + "round(perSecondTarget * dt) quads (carrying the fractional remainder), so this sets the pace "
                + "independent of frame rate. Raise to tens of thousands/sec to reach the 1M limit quickly."
        )]
        float m_SpawnedPerSecondTarget = BoxColliderBenchmarkConfig.DefaultPerSecondTarget;

        [SerializeField]
        [Min(1)]
        [Tooltip(
            "Spawned per frame (max): a hard CEILING on how many quads are instantiated in one frame, bounding "
                + "the per-frame structural-change cost regardless of the rate × dt."
        )]
        int m_SpawnedPerFrameMax = BoxColliderBenchmarkConfig.DefaultPerFrameMax;

        [SerializeField]
        [Tooltip("Full extents of the box collider + quad mesh.")]
        Vector2 m_BoxSize = new Vector2(0.4f, 0.4f);

        [SerializeField]
        [Tooltip("Spawn-AABB minimum corner.")]
        Vector2 m_SpawnMin = new Vector2(-8f, 4f);

        [SerializeField]
        [Tooltip("Spawn-AABB maximum corner.")]
        Vector2 m_SpawnMax = new Vector2(8f, 12f);

        /// <summary>The runtime config this component bakes to. Read by <c>BoxColliderBenchmarkBaker</c>.</summary>
        public BoxColliderBenchmarkConfig ToConfig() =>
            new BoxColliderBenchmarkConfig
            {
                spawnedTotalLimit = m_SpawnedTotalLimit,
                spawnedPerSecondTarget = m_SpawnedPerSecondTarget,
                spawnedPerFrameMax = m_SpawnedPerFrameMax,
                boxSize = new float2(m_BoxSize.x, m_BoxSize.y),
                spawnMin = new float2(m_SpawnMin.x, m_SpawnMin.y),
                spawnMax = new float2(m_SpawnMax.x, m_SpawnMax.y),
            };

        void OnValidate()
        {
            // Clamp the total limit to the supported on-screen stress ceiling; keep the rate / per-frame ceiling
            // non-negative. The fields are freely editable up to MaxTotalLimit (~1M) — no slider, just a typed int.
            m_SpawnedTotalLimit = math.clamp(
                m_SpawnedTotalLimit,
                1,
                BoxColliderBenchmarkConfig.MaxTotalLimit
            );
            m_SpawnedPerSecondTarget = math.max(0f, m_SpawnedPerSecondTarget);
            m_SpawnedPerFrameMax = math.max(1, m_SpawnedPerFrameMax);
        }
    }
}
