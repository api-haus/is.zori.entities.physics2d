using System.Runtime.CompilerServices;

// Exposes the sample's internal helpers (e.g. BenchmarkGUI's percentile / FPS math) to the sample's test
// assembly so the order-statistic computation can be validated directly, without driving the player loop.
[assembly: InternalsVisibleTo("Zori.Entities.Physics2D.BoxColliderBenchmark.Tests")]
