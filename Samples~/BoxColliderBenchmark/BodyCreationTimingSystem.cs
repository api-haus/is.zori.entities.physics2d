using System.Diagnostics;
using Unity.Entities;
using Debug = UnityEngine.Debug;

namespace Zori.Entities.Physics2D.Samples
{
    /// <summary>
    /// The timing instrument that makes this sample the measurement vehicle for the cached-body-template
    /// optimisation. It brackets <c>PhysicsWorld2DSystem</c>'s update with two cheap systems in
    /// <see cref="FixedStepSimulationSystemGroup"/>: <see cref="BodyCreationTimingBeginSystem"/> (before) starts a
    /// wall-clock timer and snapshots how many entities still await a body, and
    /// <see cref="BodyCreationTimingEndSystem"/> (after) stops the timer, derives how many bodies were created this
    /// frame, and on a creation frame logs the per-frame timing + a running per-created-body mean.
    /// </summary>
    /// <remarks>
    /// <b>What this brackets, stated plainly.</b> <c>PhysicsWorld2DSystem.OnUpdate</c> does body creation AND the
    /// per-step <c>Simulate(dt)</c> in one update; they are not separable from outside the package. So this
    /// measures creation PLUS the same-frame simulate, not creation alone. The number that isolates the
    /// creation-path saving is the design's intended one: the <b>on/off delta</b> at matched body counts — run the
    /// same scene/spray once with <c>CacheIdenticalBodies</c> ON and once OFF (toggle it on the scene's
    /// <c>PhysicsStep2DAuthoring</c>), and compare the per-frame times at equal live-body counts. Simulate cost is
    /// identical across the two arms for the same live-body count, so the difference is the creation-path saving.
    /// The per-frame log (not just a total) exposes the warm-up tax below the threshold.
    ///
    /// <para><b>Honesty.</b> A box is the cheapest-but-one form to build, so the on/off delta is real but bounded —
    /// the cache removes the per-entity C# definition construction + mass arithmetic, NOT the irreducible per-body
    /// <c>CreateBody</c>/<c>CreateShape</c> native calls. The log reports the measured delta as it is; if it is
    /// small, the readout says so rather than overselling. Meaningful timing is deck-only (CPU/managed-marshalling
    /// cost) — desktop numbers are for local verification, not the benchmark result.</para>
    /// </remarks>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(PhysicsWorld2DSystem))]
    public partial struct BodyCreationTimingBeginSystem : ISystem
    {
        EntityQuery m_AwaitingBodyQuery;

        public void OnCreate(ref SystemState state)
        {
            // Entities that have the body+shape but no live PhysicsBody2D yet — exactly the creation query
            // PhysicsWorld2DSystem drains this frame.
            m_AwaitingBodyQuery = SystemAPI
                .QueryBuilder()
                .WithAll<PhysicsBody2DDefinition, PhysicsShape2D>()
                .WithNone<PhysicsBody2D>()
                .Build();
            state.RequireForUpdate<BoxColliderBenchmarkConfig>();
            if (!SystemAPI.HasSingleton<BodyCreationTimingState>())
                state.EntityManager.CreateSingleton(new BodyCreationTimingState());
        }

        public void OnUpdate(ref SystemState state)
        {
            var s = SystemAPI.GetSingleton<BodyCreationTimingState>();
            s.awaitingBeforeUpdate = m_AwaitingBodyQuery.CalculateEntityCount();
            s.startTimestamp = Stopwatch.GetTimestamp();
            SystemAPI.SetSingleton(s);
        }
    }

    /// <summary>The post-creation half of the timing instrument (see
    /// <see cref="BodyCreationTimingBeginSystem"/>).</summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsWorld2DSystem))]
    public partial struct BodyCreationTimingEndSystem : ISystem
    {
        EntityQuery m_AwaitingBodyQuery;
        EntityQuery m_LiveBodyQuery;

        public void OnCreate(ref SystemState state)
        {
            m_AwaitingBodyQuery = SystemAPI
                .QueryBuilder()
                .WithAll<PhysicsBody2DDefinition, PhysicsShape2D>()
                .WithNone<PhysicsBody2D>()
                .Build();
            m_LiveBodyQuery = SystemAPI.QueryBuilder().WithAll<PhysicsBody2D>().Build();
            state.RequireForUpdate<BodyCreationTimingState>();
            state.RequireForUpdate<BoxColliderBenchmarkConfig>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var endTimestamp = Stopwatch.GetTimestamp();
            var s = SystemAPI.GetSingleton<BodyCreationTimingState>();

            var awaitingAfter = m_AwaitingBodyQuery.CalculateEntityCount();
            var created = s.awaitingBeforeUpdate - awaitingAfter;
            if (created <= 0)
                return; // No creation this frame (pure-step or idle frame) — nothing to attribute.

            var elapsedMicros = (endTimestamp - s.startTimestamp) * 1_000_000.0 / Stopwatch.Frequency;
            var liveBodies = m_LiveBodyQuery.CalculateEntityCount();

            s.totalCreated += created;
            s.totalMicros += elapsedMicros;
            s.creationFrames += 1;
            SystemAPI.SetSingleton(s);

            var perBodyThisFrame = elapsedMicros / created;
            var perBodyRunning = s.totalMicros / s.totalCreated;
            // One line per creation frame: the per-frame creation+simulate window, the bodies created in it, the
            // per-created-body cost this frame, and the running per-body mean across the spray so far. ManagedLog
            // because this is a sample diagnostic, not Burst-path code (the whole instrument is main-thread managed).
            BodyCreationTimingLog.Frame(
                created,
                liveBodies,
                elapsedMicros,
                perBodyThisFrame,
                s.totalCreated,
                perBodyRunning
            );
        }
    }

    /// <summary>The cross-system timing handoff: the begin system writes the pre-update awaiting-count + start
    /// timestamp, the end system reads them. A singleton component (not a static) so it lives per-world and is
    /// reset cleanly on world teardown.</summary>
    public struct BodyCreationTimingState : IComponentData
    {
        /// <summary>Count of body-awaiting entities snapshotted by the begin system before
        /// <c>PhysicsWorld2DSystem</c> runs.</summary>
        public int awaitingBeforeUpdate;

        /// <summary><c>Stopwatch.GetTimestamp()</c> captured by the begin system.</summary>
        public long startTimestamp;

        /// <summary>Running total of bodies created across all creation frames (for the per-body mean).</summary>
        public long totalCreated;

        /// <summary>Running total of bracketed microseconds across all creation frames.</summary>
        public double totalMicros;

        /// <summary>How many frames actually created at least one body.</summary>
        public int creationFrames;
    }

    /// <summary>The managed logging sink, factored out so the timing systems stay free of the
    /// <c>Debug.Log</c> string formatting (which is not Burst-compatible) and a consumer can swap it for a CSV
    /// writer or a profiler counter without touching the timing logic. Deliberately NOT <c>[BurstCompile]</c> —
    /// the whole instrument is main-thread managed (it logs a formatted string), matching
    /// <c>PhysicsWorld2DSystem</c>'s non-Burst posture.</summary>
    static class BodyCreationTimingLog
    {
        public static void Frame(
            int created,
            int liveBodies,
            double elapsedMicros,
            double perBodyMicros,
            long totalCreated,
            double perBodyRunningMicros
        )
        {
            Debug.Log(
                $"[BoxColliderBenchmark] created={created} live={liveBodies} "
                    + $"frame={elapsedMicros:F1}us perBody={perBodyMicros:F3}us "
                    + $"(running: total={totalCreated} perBody={perBodyRunningMicros:F3}us) "
                    + "[creation+simulate window; compare ON vs OFF at matched live count]"
            );
        }
    }
}
