using NUnit.Framework;

namespace Zori.Entities.Physics2D.Samples.Tests
{
    /// <summary>
    /// Validates <see cref="BenchmarkGUI"/>'s pure stats helpers — the nearest-rank percentile and the
    /// frame-time→FPS conversion — directly, without driving the player loop. The spawned-count read and the
    /// per-frame ring accumulation are exercised live by the scene-verify PlayMode run; this covers the one piece
    /// of non-trivial arithmetic (the order statistics the FPS widget reports).
    /// </summary>
    public sealed class BenchmarkGUIStatsTest
    {
        // A known sorted window 1..100 ms: nearest-rank percentile p of n is sorted[ceil(p*n)-1].
        static float[] SortedOneToHundred()
        {
            var a = new float[100];
            for (var i = 0; i < 100; i++)
                a[i] = i + 1; // 1..100, already sorted
            return a;
        }

        [Test]
        public void Percentile_NearestRank_PicksExpectedOrderStatistic()
        {
            var s = SortedOneToHundred();
            // p50 → ceil(0.50*100)-1 = index 49 → 50ms; p95 → index 94 → 95ms; p99 → index 98 → 99ms.
            Assert.AreEqual(50f, BenchmarkGUI.Percentile(s, 100, 0.50f), 1e-4f);
            Assert.AreEqual(95f, BenchmarkGUI.Percentile(s, 100, 0.95f), 1e-4f);
            Assert.AreEqual(99f, BenchmarkGUI.Percentile(s, 100, 0.99f), 1e-4f);
            // Bounds: q=0 → first, q=1 → last.
            Assert.AreEqual(1f, BenchmarkGUI.Percentile(s, 100, 0f), 1e-4f);
            Assert.AreEqual(100f, BenchmarkGUI.Percentile(s, 100, 1f), 1e-4f);
        }

        [Test]
        public void Percentile_PartialWindow_UsesOnlyFilledPortion()
        {
            // Ring not yet full: only the first n entries are valid. With n=10 (1..10), p50 → index 4 → 5ms.
            var s = SortedOneToHundred();
            Assert.AreEqual(5f, BenchmarkGUI.Percentile(s, 10, 0.50f), 1e-4f);
            Assert.AreEqual(10f, BenchmarkGUI.Percentile(s, 10, 1f), 1e-4f);
            // n=1 returns the single sample regardless of q.
            Assert.AreEqual(1f, BenchmarkGUI.Percentile(s, 1, 0.95f), 1e-4f);
        }

        [Test]
        public void Fps_IsReciprocalOfFrameMs()
        {
            Assert.AreEqual(60f, BenchmarkGUI.Fps(1000f / 60f), 1e-3f); // 16.67ms → 60 FPS
            Assert.AreEqual(100f, BenchmarkGUI.Fps(10f), 1e-3f); // 10ms → 100 FPS
            Assert.AreEqual(0f, BenchmarkGUI.Fps(0f), 1e-6f); // guard: 0ms → 0, not div-by-zero
        }
    }
}
