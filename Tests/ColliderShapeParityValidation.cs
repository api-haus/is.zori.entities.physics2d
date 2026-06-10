using System.Collections;
using Unity.Mathematics;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-1A collider-shape parity: for each remaining built-in 2D collider shape (Box / Capsule /
    /// Polygon / Edge), a dynamic body carrying that shape falls onto a static floor and settles, run two
    /// ways — ECS-baked through the package's bakers vs the GameObject <c>Physics2D.Simulate</c> reference —
    /// from one single-authored child scene, and compared via <see cref="PhysicsParityHarness"/>.
    /// </summary>
    /// <remarks>
    /// These are <em>contact</em> fixtures, unlike the free-fall circle: the shaped body drops a few metres,
    /// hits the floor, and rests. Two Box2D-lineage solvers (GameObject = v2 iteration, package = v3
    /// sub-stepping — <c>00d</c>) diverge faster through a contact/settle phase than in pure free-fall, so the
    /// envelope is widened from the circle's free-fall band toward the design's measured contact-phase worst
    /// (~7e-2 m / ~9e-3 rad on a small contact scene) plus margin. The disqualifiers do the load-bearing work
    /// here: the body must travel several metres (it fell), settle in a region just above the floor, and
    /// never produce NaN/Inf. The floor is a collider-only static body (no <c>Rigidbody2D</c>), so it is
    /// excluded from the compared set on both backends — the harness compares only the dynamic shaped body.
    ///
    /// <para>Build the fixtures first via
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.ColliderShapeFixtureBuilder.BuildAll</c>.</para>
    /// </remarks>
    public sealed class ColliderShapeParityValidation
    {
        const float Dt = 1f / 60f;

        // 180 steps (3.0 s): the body falls ~4 m (≈0.9 s) then has >2 s to settle on the floor.
        const int Steps = 180;

        // Contact-phase envelope. The body free-falls ~55 steps, hits the floor, and during the landing
        // transient the v2-iteration (GameObject) and v3-substep (package) solvers resolve the impact at
        // different rates — the body momentarily sits at a different penetration depth between the two before
        // both settle. That LANDING TRANSIENT, not free fall, is the worst-case: a convex polygon landing on
        // an edge/face was measured to spike ~0.26 m at the contact step on this fixture (Phase-1A run),
        // decaying as both solvers settle. So the band carries a larger flat base (0.15 m) to absorb the
        // one-time landing spike, plus a free-fall slope (3e-3 m/step, above the circle's measured ~1.72e-3)
        // for the descent. This is wider than the free-fall circle band by design — the design (02-design.md
        // §parity observable) explicitly directs widening the band for contact/restitution/tumbling scenes
        // from the probe's measured contact-phase worst. The disqualifiers (travelled several metres, settled
        // in a coarse region just above the surface, no NaN/Inf) do the load-bearing correctness work; the
        // band fails loudly only on a metres-scale divergence or a body that never settles.
        static PhysicsParityHarness.ParityEnvelope ContactEnvelope(float2 settleMin, float2 settleMax)
        {
            return new PhysicsParityHarness.ParityEnvelope
            {
                positionBaseMeters = 1.5e-1f,
                positionGrowthPerStep = 3.0e-3f,
                angleCapRadians = 9e-2f,
                settleRegionMin = settleMin,
                settleRegionMax = settleMax,
                // Falls from y=5 to rest near y≈1; ≥2 m disqualifies a silently-no-op bake.
                minTravelMeters = 2f,
            };
        }

        [UnityTest]
        public IEnumerator Box_OnFloor_AgreesWithGameObjectReference()
        {
            // Box (1×1) rests with its centre at floor-top (0.5) + half-height (0.5) = 1.0.
            yield return PhysicsParityHarness.RunParity(
                "BoxOnFloor",
                "BoxOnFloor_Sub",
                Dt,
                Steps,
                ContactEnvelope(new float2(-1.5f, 0.3f), new float2(1.5f, 2.0f))
            );
        }

        [UnityTest]
        public IEnumerator Capsule_OnFloor_AgreesWithGameObjectReference()
        {
            // Vertical capsule (1×2) rests with its centre at floor-top (0.5) + half-height (1.0) = 1.5.
            yield return PhysicsParityHarness.RunParity(
                "CapsuleOnFloor",
                "CapsuleOnFloor_Sub",
                Dt,
                Steps,
                ContactEnvelope(new float2(-1.5f, 0.8f), new float2(1.5f, 2.5f))
            );
        }

        [UnityTest]
        public IEnumerator Polygon_OnFloor_AgreesWithGameObjectReference()
        {
            // Convex pentagon (lowest verts at y=-0.5) rests with its centre at ~0.5 + 0.5 = 1.0.
            yield return PhysicsParityHarness.RunParity(
                "PolygonOnFloor",
                "PolygonOnFloor_Sub",
                Dt,
                Steps,
                ContactEnvelope(new float2(-1.5f, 0.3f), new float2(1.5f, 2.0f))
            );
        }

        [UnityTest]
        public IEnumerator Edge_OnFloor_AgreesWithGameObjectReference()
        {
            // The Edge collider is the STATIC ground surface (a wide dished open chain), and the compared
            // dynamic body is a circle that falls onto it and rests in the dish — the faithful EdgeCollider2D
            // scenario (a Box2D chain is a one-sided non-solid surface, designed for static geometry; a
            // dynamic chain body falls through a floor on its non-solid side). The dynamic circle rests on the
            // flat middle of the chain at centre y ≈ 0 + 0.5 = 0.5. This is a scene-specific gate (the edge as
            // surface, not as the falling body) chosen under the phase's fail-soft rule, not a loosened band.
            yield return PhysicsParityHarness.RunParity(
                "EdgeOnFloor",
                "EdgeOnFloor_Sub",
                Dt,
                Steps,
                ContactEnvelope(new float2(-2.0f, 0.0f), new float2(2.0f, 2.0f))
            );
        }
    }
}
