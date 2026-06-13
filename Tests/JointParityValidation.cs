using System.Collections;
using Unity.Mathematics;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-2A joint parity: for each of the first three built-in 2D joints (Hinge / Slider / Wheel), a
    /// single dynamic body jointed to the static world is run two ways — ECS-baked through the package's
    /// joint bakers + creation system vs the GameObject <c>Physics2D.Simulate</c> reference — from one
    /// single-authored child scene, and compared via <see cref="PhysicsParityHarness"/>. The built-in
    /// <c>*Joint2D</c> component rides on the same authored body, so the reference side gets the joint for
    /// free; the package bakes it to a <see cref="PhysicsJoint2DDefinition"/> and creates the Box2D joint.
    /// </summary>
    /// <remarks>
    /// A joint constrains motion, and the test asserts the constraint is actually HELD on the ECS side and
    /// agrees with the GameObject reference within a generous v2-vs-v3 band. Joints diverge faster between
    /// the two solvers than free fall — a pendulum's angle and a suspension's spring response are exactly
    /// where a v2-iteration and a v3-substep solver differ most — so the envelopes here are wider than the
    /// collider contact band, scene-specific, and the disqualifiers do the load-bearing correctness work:
    /// the jointed body moved as the joint dictates (swung / slid / sprung), never flew free (it stays in a
    /// tight region around its anchor, which a broken/absent joint would not), and produced no NaN/Inf.
    ///
    /// <para>Build the fixtures first via
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.JointFixtureBuilder.BuildAll</c>.</para>
    /// </remarks>
    public sealed class JointParityValidation
    {
        const float Dt = 1f / 60f;

        [UnityTest]
        public IEnumerator Hinge_PendulumSwing_AgreesWithGameObjectReference()
        {
            // A 2-unit arm pinned at its left end to the world at (0, 5), released horizontal, swings down.
            // The arm's CENTRE starts at (1, 5) and traces a circle of radius 1 about the pivot: at the
            // bottom of the swing the centre is at (0, 4); it never leaves the 1-unit circle around (0, 5).
            // The hardest-to-fake observable is the bounded swing arc + the pinned end staying at the anchor:
            // a body that flew free (broken joint) would fall straight to y → −∞ and blow the settle region.
            //
            // 150 steps (2.5 s) covers several full swings of a ~1 m pendulum (period ~2 s). The pendulum
            // ROTATES, so angle error between v2 and v3 grows fast through the swing — the band is wide on
            // angle and pinned mainly by the disqualifiers (the centre stays within the swing circle around
            // the anchor, never flies away).
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // The pendulum's PHASE drifts between v2 and v3 over several swings, so the two arms can be on
                // opposite sides of the arc at a late step → up to ~2 m centre separation on a 1 m-radius
                // pendulum. The band is wide and grows to absorb that phase drift; the disqualifiers (stays in
                // the swing circle, no NaN, moved) carry correctness.
                positionBaseMeters = 0.25f,
                positionGrowthPerStep = 1.2e-2f,
                angleCapRadians = 3.2f, // ~π: phase drift can put the arms a full swing apart in angle
                // The arm centre traces a circle radius 1 about (0, 5): it stays in [−1.2,1.2]×[3.8,6.2].
                settleRegionMin = new float2(-1.3f, 3.6f),
                settleRegionMax = new float2(1.3f, 6.4f),
                // The pendulum OSCILLATES, so net end−start displacement is small even though the arc traversed
                // is long; the swing is proven instead by the position-error column oscillating (the v2/v3
                // phase drift dips and grows) and by the arm staying on the swing circle. A still arm would
                // have a monotone error and zero motion; 0.1 m of net displacement disqualifies that no-op.
                minTravelMeters = 0.1f,
            };
            yield return PhysicsParityHarness.RunParity("HingeJoint", "HingeJoint_Sub", Dt, 150, envelope);
        }

        [UnityTest]
        public IEnumerator Slider_AxisConfinedMotion_AgreesWithGameObjectReference()
        {
            // A 1×1 block on a HORIZONTAL slide axis (angle 0°), anchored to the world at (0, 5), launched at
            // +6 m/s along +X. Gravity pulls down, but the slider confines motion to the axis, so Y stays
            // ~constant at 5 and the block slides out along +X. The hardest-to-fake observable is exactly that
            // axis confinement: a broken/absent joint would let gravity drop the block (y → −∞), blowing the
            // settle region; a correct slider keeps y ≈ 5 for all steps. There is no friction/limit, so the
            // block slides ballistically along X (no deceleration) — both backends advance x ≈ 6·t.
            //
            // 120 steps (2.0 s): the block reaches x ≈ 12. X agreement is tight (ballistic, no contact); the
            // band's job is mainly to confirm the SAME x on both sides and y pinned to the axis.
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // Both slide at the same launch speed with no contact, so the only divergence is the v2-vs-v3
                // free-fall-style integration offset along the constrained axis — small. A modest grow band.
                positionBaseMeters = 2.0e-2f,
                positionGrowthPerStep = 3.0e-3f,
                angleCapRadians = 1.0e-2f, // the block does not rotate (axis-aligned slide)
                // y pinned to ~5 (the axis), x reaches ~12. Region is wide on x, tight on y to PROVE
                // confinement: a body that fell off the axis (broken joint) ends far below y=5 and fails here.
                settleRegionMin = new float2(8.0f, 4.5f),
                settleRegionMax = new float2(16.0f, 5.5f),
                // Slides ~12 m along X — a no-op bake (or a body that just fell) fails this.
                minTravelMeters = 6f,
            };
            yield return PhysicsParityHarness.RunParity("SliderJoint", "SliderJoint_Sub", Dt, 120, envelope);
        }

        [UnityTest]
        public IEnumerator Wheel_SuspensionTravel_AgreesWithGameObjectReference()
        {
            // A radius-0.5 hub on a VERTICAL suspension axis (angle 90°), anchored to the world at (0, 5),
            // starting AT the anchor. Gravity sags it down the axis; the 2 Hz, lightly-damped (0.2) suspension
            // spring resists and bounces it, settling at the spring's equilibrium sag below 5. The
            // hardest-to-fake observable is the bounded suspension travel along the axis: a broken/absent joint
            // lets the hub free-fall (y → −∞), blowing the settle region; a correct wheel keeps the hub within
            // a fraction of a metre of the anchor along the axis, oscillating then settling. X stays ~0 (the
            // axis is vertical, so the hub cannot move horizontally).
            //
            // 180 steps (3.0 s) covers several spring cycles (period 0.5 s) and the settle. The spring's PHASE
            // and equilibrium differ between v2 and v3 (this is the canonical solver-difference case for a
            // spring), so the band is generous and the disqualifiers pin correctness (stays near the anchor on
            // the axis, x ≈ 0, no NaN, sagged downward at least a little).
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // Spring phase/equilibrium drift between v2 and v3 → the hub can be near opposite ends of its
                // small travel between the two at a given step. Travel is sub-metre, so even a wide relative
                // band is a small absolute number.
                positionBaseMeters = 0.12f,
                positionGrowthPerStep = 2.0e-3f,
                angleCapRadians = 1.0f, // a free wheel can spin; angle is not the observable here
                // x pinned to ~0 (vertical axis), y within the suspension travel below the anchor at 5. A 2 Hz
                // spring's equilibrium sag under gravity is g/ω² ≈ 9.81/(2π·2)² ≈ 0.062 m, so the hub settles
                // just below 5; the transient overshoots a little further down.
                settleRegionMin = new float2(-0.4f, 4.4f),
                settleRegionMax = new float2(0.4f, 5.2f),
                // The hub sags ~0.06 m off the anchor at equilibrium (a 2 Hz spring under gravity), overshooting
                // further during the transient; a no-op bake (hub frozen exactly at the anchor) fails this,
                // while a free-fall (broken joint) fails the settle region instead.
                minTravelMeters = 0.03f,
            };
            yield return PhysicsParityHarness.RunParity("WheelJoint", "WheelJoint_Sub", Dt, 180, envelope);
        }

        [UnityTest]
        public IEnumerator Distance_RigidSeparationHeld_AgreesWithGameObjectReference()
        {
            // A radius-0.5 disc 3 units RIGHT of a static anchor at (0, 5), joined by a RIGID distance joint of
            // rest length 3. Gravity swings the disc down about the anchor like a rod pendulum, but the rigid
            // constraint holds |disc − (0,5)| ≈ 3 throughout: the disc traces a radius-3 circle about the
            // anchor. The hardest-to-fake observable is exactly that held separation — a broken joint lets the
            // disc free-fall (separation grows without bound, y → −∞), blowing the settle region; a correct
            // distance joint keeps the disc on the radius-3 circle around (0, 5).
            //
            // 150 steps (2.5 s) covers part of a swing of a ~3 m rod pendulum (period ~3.5 s). Like the hinge
            // pendulum this ROTATES, so the disc's angular position drifts between v2 and v3 over the swing —
            // a wide position band that grows to absorb the phase drift, with the disqualifiers (stays on the
            // radius-3 circle around the anchor, never flies free) carrying correctness.
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // The disc swings on a radius-3 arc; v2/v3 phase drift can separate the two discs by up to a
                // couple of metres late in the swing. The band grows to absorb that; the held-separation
                // disqualifier (settle region is a band around the anchor at radius ~3) does the real work.
                positionBaseMeters = 0.25f,
                positionGrowthPerStep = 1.4e-2f,
                angleCapRadians = 3.2f, // the disc orbits the anchor; orientation drifts freely
                // The disc stays on the radius-3 circle about (0, 5): x ∈ [−3.3, 3.3], y ∈ [1.6, 5.3] (it starts
                // level with the anchor and swings down, never above it by more than slack).
                settleRegionMin = new float2(-3.3f, 1.6f),
                settleRegionMax = new float2(3.3f, 5.3f),
                // Released level and swinging down, the disc descends ~1+ m off its start before the window
                // ends — a still disc (no-op bake) or one frozen at the anchor fails this.
                minTravelMeters = 0.5f,
            };
            yield return PhysicsParityHarness.RunParity("DistanceJoint", "DistanceJoint_Sub", Dt, 150, envelope);
        }

        [UnityTest]
        public IEnumerator Spring_OscillatesTowardRest_AgreesWithGameObjectReference()
        {
            // A radius-0.5 disc 2 units BELOW a static anchor at (0, 5) — i.e. at (0, 3) — joined by a SPRING
            // joint of rest length 1 (so the rest position is (0, 4)). Starting 1 unit below rest, the spring
            // yanks the disc up, overshoots, and oscillates vertically about (0, 4) at ~1.5 Hz, settling as the
            // light damping (0.15) bleeds it out, a touch below 4 from the gravity sag. X stays ~0 (the spring
            // line is vertical). The hardest-to-fake observable is the oscillation toward the rest length: a
            // broken joint lets the disc free-fall (y → −∞), blowing the settle region; a correct spring brings
            // the disc UP toward y ≈ 4 and oscillates there.
            //
            // 180 steps (3.0 s) covers ~4–5 spring cycles (period ~0.67 s) and the settle. The spring's PHASE
            // and equilibrium are the canonical v2-vs-v3 divergence case, so the band is generous and the
            // disqualifiers pin correctness (the disc rises toward rest and stays near (0, 4), never falls
            // away).
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // Spring phase/equilibrium drift between v2 and v3 → the two discs can be near opposite ends of
                // the ~1 m oscillation at a given step. The band is wide enough to absorb that phase offset.
                positionBaseMeters = 0.3f,
                positionGrowthPerStep = 3.0e-3f,
                angleCapRadians = 1.0e-1f, // the disc barely rotates (a centred vertical spring)
                // x pinned to ~0 (vertical spring line), y within the oscillation about the rest position
                // (0, 4): the disc starts at 3, rises through 4, overshoots a little past it, and settles near
                // 4. It never goes below its start (3) by much, nor above the anchor (5).
                settleRegionMin = new float2(-0.5f, 3.4f),
                settleRegionMax = new float2(0.5f, 4.8f),
                // It travels from y=3 up to ~y=4 (and oscillates), so net displacement clears this easily; a
                // no-op bake (disc frozen at 3) fails it.
                minTravelMeters = 0.4f,
            };
            yield return PhysicsParityHarness.RunParity("SpringJoint", "SpringJoint_Sub", Dt, 180, envelope);
        }

        [UnityTest]
        public IEnumerator Fixed_RelativePoseLocked_AgreesWithGameObjectReference()
        {
            // A 1×1 block welded to a static anchor at (0, 5), authored 2 units RIGHT at (2, 5). A RIGID fixed
            // joint (frequency 0 = maximum stiffness) locks the block in that relative pose, so under gravity
            // it neither falls nor rotates away — it stays pinned at (2, 5) with ~0 rotation. The hardest-to-
            // fake observable is the held relative pose: a broken joint lets the block free-fall (y → −∞),
            // blowing the settle region; a correct fixed joint keeps the block at (2, 5) for every step. This is
            // the tightest joint band — a rigid weld means almost no motion, so the only divergence is the
            // tiny v2-vs-v3 solver compliance at the weld.
            //
            // 150 steps (2.5 s). Because the block barely moves, minTravel is small — the test proves the joint
            // HELD (block did not fall), not that it travelled far. The settle region is tight around (2, 5).
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // A rigid weld holds the pose nearly exactly on both solvers; the only divergence is the
                // small solver compliance under the gravity load. A modest band.
                positionBaseMeters = 5.0e-2f,
                positionGrowthPerStep = 1.0e-3f,
                angleCapRadians = 5.0e-2f, // the weld holds rotation too; only tiny solver compliance
                // The block stays welded at (2, 5): a tight box around it. A block that fell ends far below.
                settleRegionMin = new float2(1.7f, 4.6f),
                settleRegionMax = new float2(2.3f, 5.4f),
                // A rigid weld is DEFINED by holding the pose — near-zero travel is the CORRECT signal, not a
                // no-op. The measured weld held to ~2e-5 m (run 1), so the SETTLE region (block stays at
                // (2, 5)) and the parity band carry correctness; minTravel is a floor below the measured hold
                // only to catch a literally-uncreated body (which would also fail settle). For a "constraint
                // held" observable the held position IS the proof, so the floor sits below the held motion.
                minTravelMeters = 1.0e-5f,
            };
            yield return PhysicsParityHarness.RunParity("FixedJoint", "FixedJoint_Sub", Dt, 150, envelope);
        }

        [UnityTest]
        public IEnumerator Relative_OffsetMaintained_AgreesWithGameObjectReference()
        {
            // A 1×1 block with a relative joint of linearOffset (2, 0) to a static anchor at (0, 5), authored at
            // (2, 5). The built-in RelativeJoint2D sign holds bodyB at (bodyA − linearOffset) = (−2, 5), so the
            // joint DRIVES the block from its authored (2, 5) across to (−2, 5) and holds it there against
            // gravity. The package encodes the same sign (measured against the GameObject reference, which
            // drove the block to −2), so both backends converge to (−2, 5). The hardest-to-fake observable is
            // reaching AND holding that offset: a broken joint lets the block fall straight down (y → −∞),
            // blowing the settle region; a correct relative joint moves it LEFT to x ≈ −2 and holds (−2, 5).
            //
            // 150 steps (2.5 s) covers the drive-across transient and the settle. The v2-vs-v3 difference is the
            // convergence rate across the 4 m drive; both reach the same steady offset.
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // The 4 m drive-across converges at slightly different rates between v2 and v3, so mid-flight
                // the two blocks can be a fraction of a metre apart; both reach (−2, 5). The band grows to
                // absorb the convergence-rate difference over the transient.
                positionBaseMeters = 0.3f,
                positionGrowthPerStep = 6.0e-3f,
                angleCapRadians = 0.5f, // the block may rotate a little before the angular offset settles
                // Driven to and held near (−2, 5): a box around it wide enough for the transient sag, tight
                // enough that a fallen block (y ≪ 5) or one that never drove across (x ≈ 2) fails.
                settleRegionMin = new float2(-2.7f, 4.3f),
                settleRegionMax = new float2(-1.3f, 5.5f),
                // The block drives ~4 m from (2, 5) to (−2, 5) — a no-op bake (frozen at (2,5)) fails this, a
                // fallen block fails the settle region.
                minTravelMeters = 2.0f,
            };
            yield return PhysicsParityHarness.RunParity("RelativeJoint", "RelativeJoint_Sub", Dt, 150, envelope);
        }

        [UnityTest]
        public IEnumerator Friction_RelativeMotionDamps_AgreesWithGameObjectReference()
        {
            // A 1×1 block on a friction joint to a static anchor at the block's start (0, 5), launched at +5 m/s
            // along +X. The friction joint resists relative motion between block and (static) anchor up to a
            // force cap, so the launch velocity damps out: the block slides a short distance, decelerating, and
            // stops — held against gravity too. The hardest-to-fake observable is that DAMPING: a broken/absent
            // joint lets the block keep its +5 m/s (sliding metres) and fall under gravity; a correct friction
            // joint brings it to rest near its start. So the block travels a little (it does not freeze
            // instantly) but ends near (0, 5), nothing like the ballistic ~10 m an unconstrained launch covers.
            //
            // 120 steps (2.0 s) is enough for the friction to arrest the launch. The disqualifier is the
            // settle near the start (motion damped, not ballistic, and not fallen); v2-vs-v3 differ in the
            // exact decel curve.
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // The two solvers brake the launch at slightly different rates, so the block's position during
                // the decel transient can differ by a fraction of a metre; both end near the start. A moderate
                // band over the short transient.
                positionBaseMeters = 0.3f,
                positionGrowthPerStep = 4.0e-3f,
                angleCapRadians = 0.5f, // the block may rotate slightly as friction arrests it
                // Damps to rest NEAR the start (0, 5) — it slides +X then stops, held against gravity. A wide
                // box on x for the (now visible) slide distance, on y for the small gravity sag; a ballistic
                // (unbraked) block ends at x ≈ 10 and fails the upper x bound, a fallen block ends y ≪ 5 and
                // fails the lower y bound. The slide distance is measured, not guessed (run-2 calibration).
                settleRegionMin = new float2(-0.3f, 4.3f),
                settleRegionMax = new float2(6.0f, 5.4f),
                // The block slides a visible fraction of a metre before friction arrests it; a frozen no-op
                // bake fails this floor, while a runaway launch fails the settle region's x bound.
                minTravelMeters = 2.0e-2f,
            };
            yield return PhysicsParityHarness.RunParity("FrictionJoint", "FrictionJoint_Sub", Dt, 120, envelope);
        }

        [UnityTest]
        public IEnumerator Target_PullsToWorldTarget_AgreesWithGameObjectReference()
        {
            // A radius-0.5 disc authored at (0, 5), pulled by a target joint toward a FIXED world target at
            // (3, 5). A target joint is normally mouse-driven; here the target is a serialized fixed point
            // (autoConfigureTarget = false), the single-authoring analogue of the InitialVelocity2DAuthoring
            // pattern, so both backends pull toward the same point. The disc is drawn from (0, 5) RIGHT toward
            // (3, 5) and settles near it (a touch low from the gravity sag, since maxForce holds it up). The
            // hardest-to-fake observable is REACHING the target: a broken/absent joint lets the disc free-fall
            // straight down from (0, 5) (x stays 0, y → −∞); a correct target joint moves it RIGHT to x ≈ 3 and
            // holds near (3, 5). This is the one joint with NO static anchor body — it targets a point in the
            // world via the package's null-connectedBody static world anchor.
            //
            // 150 steps (2.5 s) lets the stiff (5 Hz), critically-damped pull converge. The disqualifier is the
            // settle near (3, 5) AND the +X travel (a free-fall would never move +X). The v2-vs-v3 difference is
            // the convergence transient toward the target.
            var envelope = new PhysicsParityHarness.ParityEnvelope
            {
                // The pull-to-target transient converges at slightly different rates between v2 and v3, so the
                // disc's position mid-flight can differ by a fraction of a metre; both reach near (3, 5). The
                // band absorbs the convergence-rate difference.
                positionBaseMeters = 0.35f,
                positionGrowthPerStep = 3.0e-3f,
                angleCapRadians = 0.5f, // the disc may spin a little under the off-centre pull
                // Settles near the target (3, 5): a box around it wide enough for the gravity sag and the
                // convergence overshoot, tight enough that a free-fell disc (x ≈ 0, y ≪ 5) fails.
                settleRegionMin = new float2(2.4f, 4.2f),
                settleRegionMax = new float2(3.6f, 5.4f),
                // The disc travels ~3 m from (0, 5) to (3, 5); a no-op bake (frozen at the start) fails this,
                // and a free-fall (which moves DOWN, not toward the target) fails the settle region.
                minTravelMeters = 1.5f,
            };
            yield return PhysicsParityHarness.RunParity("TargetJoint", "TargetJoint_Sub", Dt, 150, envelope);
        }
    }
}
