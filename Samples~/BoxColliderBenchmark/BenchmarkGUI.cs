using Unity.Entities;
using Unity.Rendering;
using UnityEngine;

namespace Zori.Entities.Physics2D.Samples
{
    /// <summary>
    /// A plain-MonoBehaviour IMGUI overlay for the box-collider benchmark — a SCENE GameObject (NOT in the
    /// SubScene / ECS), drawn via <see cref="OnGUI"/>. Top-left shows the running spawned-box count read from the
    /// default ECS world; top-right shows a rolling-window FPS distribution (p50 / p95 / p99 / avg / mean / min /
    /// max) computed from per-frame <see cref="Time.unscaledDeltaTime"/> samples.
    /// </summary>
    /// <remarks>
    /// IMGUI (<c>OnGUI</c>) is the right tool for a benchmark overlay — zero asset setup, no Canvas, no ECS
    /// coupling. The spawned count is read directly from the world each frame: the sprayed quads carry
    /// <see cref="PhysicsBody2D"/> + a <see cref="MaterialMeshInfo"/> (the Entities.Graphics render component) and
    /// are not <c>Prefab</c>-tagged, while the code-built prefab template is <c>Prefab</c>-tagged and the static
    /// floor body carries no <see cref="MaterialMeshInfo"/> — so a query of (<see cref="PhysicsBody2D"/> +
    /// <see cref="MaterialMeshInfo"/>, excluding <c>Prefab</c>) counts exactly the spawned quads. The boxes never
    /// despawn, so that live count equals spawned-so-far.
    ///
    /// <para>The FPS window is a fixed <see cref="WindowSize"/>-entry ring buffer of frame times; the percentiles
    /// come from a sorted copy of the filled portion. "Avg" and "Mean" are both the arithmetic mean (shown under
    /// both labels as requested); min / max / p50 / p95 / p99 are distinct order statistics.</para>
    /// </remarks>
    [AddComponentMenu("Zori/Entities Physics 2D/Benchmark GUI")]
    [DisallowMultipleComponent]
    public sealed class BenchmarkGUI : MonoBehaviour
    {
        /// <summary>The rolling FPS window length, in frames.</summary>
        public const int WindowSize = 1024;

        readonly float[] m_FrameMs = new float[WindowSize];
        readonly float[] m_Sorted = new float[WindowSize];
        int m_Head; // next write index into the ring
        int m_Filled; // how many entries are valid (ramps up to WindowSize, then stays)

        EntityQuery m_SpawnedQuery;
        World m_World;

        GUIStyle m_Style;

        void Update()
        {
            // Record this frame's wall-clock time (unscaled — independent of Time.timeScale) into the ring.
            m_FrameMs[m_Head] = Time.unscaledDeltaTime * 1000f;
            m_Head = (m_Head + 1) % WindowSize;
            if (m_Filled < WindowSize)
                m_Filled++;
        }

        int SpawnedCount()
        {
            // (Re)acquire the world + query lazily: the default world is created on Play, after this MonoBehaviour
            // may already exist in the loaded scene.
            if (m_World == null || !m_World.IsCreated)
            {
                m_World = World.DefaultGameObjectInjectionWorld;
                if (m_World == null || !m_World.IsCreated)
                    return 0;
                m_SpawnedQuery = m_World.EntityManager.CreateEntityQuery(
                    new EntityQueryDesc
                    {
                        All = new[]
                        {
                            ComponentType.ReadOnly<PhysicsBody2D>(),
                            ComponentType.ReadOnly<MaterialMeshInfo>(),
                        },
                        None = new[] { ComponentType.ReadOnly<Prefab>() },
                    }
                );
            }

            return m_SpawnedQuery.CalculateEntityCount();
        }

        void OnGUI()
        {
            m_Style ??= new GUIStyle(GUI.skin.label)
            {
                fontSize = 16,
                normal = { textColor = Color.white },
            };

            // Top-left: the spawned-box counter.
            GUI.Label(
                new Rect(12f, 10f, 360f, 28f),
                $"Spawned: {SpawnedCount()}",
                m_Style
            );

            // Top-right: the rolling-window FPS distribution.
            if (m_Filled == 0)
                return;

            var n = m_Filled;
            System.Array.Copy(m_FrameMs, m_Sorted, n);
            System.Array.Sort(m_Sorted, 0, n);

            // Percentiles over the sorted frame-time window (low ms = high FPS; p95/p99 of frame time are the
            // worst frames, i.e. the LOW FPS tail, which is what a percentile readout should surface).
            var minMs = m_Sorted[0];
            var maxMs = m_Sorted[n - 1];
            var p50Ms = Percentile(m_Sorted, n, 0.50f);
            var p95Ms = Percentile(m_Sorted, n, 0.95f);
            var p99Ms = Percentile(m_Sorted, n, 0.99f);

            var sum = 0f;
            for (var i = 0; i < n; i++)
                sum += m_Sorted[i];
            var meanMs = sum / n;

            const float w = 300f;
            const float lh = 22f;
            var right = Screen.width - w - 12f;
            var y = 10f;

            GUI.Label(new Rect(right, y, w, lh), $"FPS over {n}-frame window:", m_Style);
            y += lh;
            // p50 / p95 / p99 are frame-time percentiles, reported as FPS (the worst frames are the low-FPS tail).
            GUI.Label(new Rect(right, y, w, lh), $"  p50: {Fps(p50Ms):0.0} ({p50Ms:0.00} ms)", m_Style);
            y += lh;
            GUI.Label(new Rect(right, y, w, lh), $"  p95: {Fps(p95Ms):0.0} ({p95Ms:0.00} ms)", m_Style);
            y += lh;
            GUI.Label(new Rect(right, y, w, lh), $"  p99: {Fps(p99Ms):0.0} ({p99Ms:0.00} ms)", m_Style);
            y += lh;
            GUI.Label(new Rect(right, y, w, lh), $"  avg: {Fps(meanMs):0.0}", m_Style);
            y += lh;
            GUI.Label(new Rect(right, y, w, lh), $"  mean: {Fps(meanMs):0.0}", m_Style);
            y += lh;
            // min/max FRAME TIME → max/min FPS; report each as the FPS it corresponds to under its own label.
            GUI.Label(new Rect(right, y, w, lh), $"  min: {Fps(maxMs):0.0} (worst frame {maxMs:0.00} ms)", m_Style);
            y += lh;
            GUI.Label(new Rect(right, y, w, lh), $"  max: {Fps(minMs):0.0} (best frame {minMs:0.00} ms)", m_Style);
        }

        // Nearest-rank percentile over the sorted [0, n) window. q in [0, 1]. Internal so the sample's EditMode
        // test can validate the order-statistic math directly without driving the player loop.
        internal static float Percentile(float[] sorted, int n, float q)
        {
            if (n <= 1)
                return sorted[0];
            var rank = Mathf.Clamp(Mathf.CeilToInt(q * n) - 1, 0, n - 1);
            return sorted[rank];
        }

        internal static float Fps(float ms) => ms > 0f ? 1000f / ms : 0f;
    }
}
