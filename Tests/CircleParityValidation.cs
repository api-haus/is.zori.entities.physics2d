using System.Collections;
using Unity.Mathematics;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The first scene to validate the reusable <see cref="PhysicsParityHarness"/>: the Circle falling-body
    /// fixture (one Dynamic <c>Rigidbody2D</c> + <c>CircleCollider2D</c>, authored at world-Y 10, no ground)
    /// run two ways — ECS-baked vs the GameObject reference — and compared. This proves the single-authoring
    /// oracle end to end on the same case the vertical slice already covers
    /// (<see cref="FallingBodyValidation"/>), so a green here means later phases can author a fixture and
    /// get a parity assertion for free.
    /// </summary>
    /// <remarks>
    /// Free fall only: at g = 9.81 over 120 steps of 1/60 s (2.0 s) the body drops ~19.6–20 m to y ≈ -10.
    /// Because there is no contact, the only parity error is the v2-vs-v3 free-fall convention offset, which
    /// the probe measured at ~1.47e-3 m/step, exactly linear, with exactly-zero angle error (00d). The
    /// envelope is therefore a linear-growth position band (no flat per-step cap — free-fall error grows
    /// linearly) and a tiny flat angle cap. The settle region brackets the expected end-of-fall position.
    /// </remarks>
    public sealed class CircleParityValidation
    {
        const string ParentSceneName = "FallingBody";
        const string ChildSceneName = "FallingBody_Sub";
        const float Dt = 1f / 60f;
        const int Steps = 120;

        [UnityTest]
        public IEnumerator Circle_FallingBody_AgreesWithGameObjectReference()
        {
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // The v2-vs-v3 free-fall convention offset is exactly linear. MEASURED on this fixture
                // (one circle, dt=1/60): ~1.72e-3 m/step, ~0.206 m worst at step 119 (the probe's box-stack
                // at dt=0.02 measured ~1.47e-3 m/step — the slope is fixture/dt-specific). The band is set
                // ~1.15x the measured slope plus a base margin, so the worst step clears it by ~17% rather
                // than tracking it. Over 120 steps this is a ~0.25 m band at the end — generous for v2/v3
                // free-fall noise, far below a real bake/integration regression (which diverges by metres).
                positionBaseMeters = 1e-2f,
                positionGrowthPerStep = 2.0e-3f,
                // Free-fall angle error is exactly zero on both solvers; 1e-2 rad is the design's standing cap.
                angleCapRadians = 1e-2f,
                // The body starts at y=10 and free-falls ~20 m; it ends near (0, -10) with no lateral drift.
                // A coarse region that brackets the end of fall without pinning the exact landing.
                settleRegionMin = new float2(-0.5f, -12f),
                settleRegionMax = new float2(0.5f, -7f),
                // It must drop many metres; 0.5 m disqualifies a silently-no-op bake.
                minTravelMeters = 0.5f,
            };

            yield return PhysicsParityHarness.RunParity(ParentSceneName, ChildSceneName, Dt, Steps, envelope);
        }
    }
}
