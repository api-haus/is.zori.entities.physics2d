using System.Collections;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-8 smoke for continuous collision detection, joint break, and interpolation. Three minimal
    /// mechanism witnesses (the hard GameObject-parity e2e gate is a separate validating agent's deliverable):
    /// <list type="bullet">
    /// <item><b>CCD</b> — a fast Continuous dynamic body fired at a thin static wall STOPS at the wall instead
    /// of tunnelling through it, while an identical Discrete body tunnels. Proves <c>fastCollisions</c> →
    /// <c>PhysicsBodyDefinition.fastCollisionsAllowed</c> bakes/creates the bullet body. (Dynamic-vs-Static CCD
    /// is the world default; the contrast is the Continuous body not tunnelling where the Discrete one does at
    /// the same speed.)</item>
    /// <item><b>Joint break</b> — a dynamic body hung from the world by a distance joint with a finite
    /// <c>breakForce</c>, loaded by gravity past that threshold, has its joint GONE (the body no longer carries
    /// <see cref="PhysicsJoint2D"/>, a break event fired) and the body then SEPARATES (free-falls away). A
    /// joint with infinite breakForce holds forever. Proves the native threshold arm + the
    /// collect/apply break path.</item>
    /// <item><b>Interpolation</b> — an internal-invariant test (see the class remark): driving the render-rate
    /// smoothing system with a known prev/cur pose and a known sub-step fraction yields a <c>LocalToWorld</c>
    /// pose that is the correct lerp of the bracketing physics poses. Proves <c>PhysicsBody2DSmoothing</c> capture
    /// + the smoothing math, without depending on a batchmode render-rate-vs-fixed-rate timing the harness cannot
    /// produce deterministically.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/>. The CCD and joint tests use the fixed-step
    /// systems driven one step per <c>group.Update()</c> (like the Phase-6/7 smokes), adding
    /// <see cref="PhysicsJoint2DCreationSystem"/> and <see cref="PhysicsJoint2DBreakSystem"/> for the joint test.
    /// The interpolation test drives <see cref="PhysicsBody2DSmoothingSystem"/> directly with a hand-set
    /// <see cref="PhysicsBody2DSmoothing"/> and <see cref="PhysicsFixedStepTime2D"/> + world time, because the
    /// smoothing is a render-rate visual: a batchmode fixed-step loop has no sub-step time to interpolate over
    /// (<c>timeAhead ≈ 0</c>), so the gate is the smoothing MATH (interpolated pose == lerp of the bracketing
    /// poses), the framing the brief's scope note recommends for the validating agent.
    /// </remarks>
    public sealed class InterpCcdJointBreakSmoke
    {
        const float Dt = 1f / 60f;

        static World MakeFixedWorld(out FixedStepSimulationSystemGroup group, bool withJoints)
        {
            var world = new World("Physics2DPhase8SmokeWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DBatchCreationSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            if (withJoints)
            {
                fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsJoint2DCreationSystem>());
                fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsJoint2DBreakSystem>());
            }
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }

        static Entity SpawnHorizontalBullet(EntityManager em, float2 pos, float vx, bool continuous)
        {
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f, // pure horizontal flight — isolate the wall hit from gravity
                    initialPosition = pos,
                    useAutoMass = true,
                    fastCollisions = continuous,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.25f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddComponentData(
                entity,
                new PhysicsBody2DInitialVelocity { linearVelocity = new float2(vx, 0f) }
            );
            return entity;
        }

        static void SpawnThinWall(EntityManager em, float2 center)
        {
            DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = center,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(0.05f, 4f), // a THIN tall wall — easy to tunnel at speed
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
        }

        static float BodyX(EntityManager em, Entity e) =>
            em.GetComponentData<Unity.Transforms.LocalToWorld>(e).Position.x;

        [UnityTest]
        public IEnumerator ContinuousBody_DoesNotTunnelThinWall_WhileDiscreteDoes()
        {
            // A thin static wall at x=0; a fast circle fired from x=-3 at +120 m/s. In one 1/60 s step the body
            // would travel 2 m — far past the 0.05 m-thick wall — so a Discrete body tunnels (ends x > 0) while
            // a Continuous (fast-collision) body is caught by CCD and stops on the wall's left side (x < 0).
            var world = MakeFixedWorld(out var group, withJoints: false);
            var em = world.EntityManager;

            SpawnThinWall(em, new float2(0f, 0f));
            var continuousBody = SpawnHorizontalBullet(em, new float2(-3f, 0f), 120f, continuous: true);
            var discreteBody = SpawnHorizontalBullet(em, new float2(-3f, 2.5f), 120f, continuous: false);

            // First update creates the bodies (no step). Then step enough to carry both past the wall plane.
            group.Update();
            for (var f = 0; f < 30; f++)
                group.Update();

            var continuousX = BodyX(em, continuousBody);
            var discreteX = BodyX(em, discreteBody);

            Assert.Less(
                continuousX,
                0.2f,
                $"A Continuous (fast-collision) body tunnelled the thin wall: ended at x={continuousX:F3} "
                    + "(expected to be stopped at/left of the wall at x≈0). fastCollisionsAllowed did not take "
                    + "effect, or the wall was not solid."
            );
            Assert.Greater(
                discreteX,
                0.2f,
                $"A Discrete body did NOT tunnel the thin wall: ended at x={discreteX:F3} (expected to pass "
                    + "through to x>0 at 120 m/s through a 0.05 m wall in one step). If it stopped, the wall is "
                    + "catching even discrete bodies and the CCD contrast is not isolated."
            );

            Debug.Log(
                $"[PHYSICS2D-CCD] continuousX={continuousX:F3} (stopped at wall), "
                    + $"discreteX={discreteX:F3} (tunnelled)."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator JointLoadedPastBreakForce_BreaksAndBodiesSeparate()
        {
            // A dynamic disc hung from the static world by a rigid distance joint of rest length 1, with a
            // breakForce small enough that the disc's weight (m·g) exceeds the joint's reaction immediately. The
            // joint must break: the disc loses its PhysicsJoint2D, a break event fires, and the disc then
            // free-falls (y drops far below where the joint would have held it, ~anchorY − restLength).
            var world = MakeFixedWorld(out var group, withJoints: true);
            var em = world.EntityManager;

            // The hung disc (owner = bodyB). connectedBody Entity.Null → the static world anchor at the origin;
            // connectedAnchor is the world-space pin point (0, 5). The disc starts 1 unit below the pin.
            var disc = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = new float2(0f, 4f),
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddComponentData(
                disc,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Distance,
                    connectedBody = Entity.Null, // pinned to the world
                    anchor = Unity.Mathematics.float2.zero,
                    connectedAnchor = new float2(0f, 5f),
                    restLength = 1f,
                    enableSpring = false,
                    // A tiny break force: the disc's weight (m·g ≈ 0.785·9.81 ≈ 7.7 N) far exceeds it, so the
                    // distance joint's reaction breaks it on the first loaded step.
                    breakForce = 0.5f,
                    breakTorque = float.PositiveInfinity,
                    breakAction = PhysicsJointBreakAction2D.Destroy,
                }
            );

            // Update 1 creates the body (no step). Update 2 creates the joint (no step — joint creation runs the
            // update after the body exists; PhysicsWorld2DSystem skips Simulate on a body-creation frame). From
            // then on each Update steps. Drive enough steps to break and then free-fall.
            group.Update();

            var brokeObserved = false;
            for (var f = 0; f < 120; f++)
            {
                group.Update();
                if (em.HasComponent<PhysicsJoint2DBroken>(disc))
                    brokeObserved = true;
            }

            Assert.IsTrue(
                brokeObserved,
                "The over-loaded joint never broke: the disc still has no PhysicsJoint2DBroken tag after 120 "
                    + "steps. The native forceThreshold was not armed, no jointThresholdEvent fired, or the "
                    + "collect/apply break path did not destroy the joint."
            );
            Assert.IsFalse(
                em.HasComponent<PhysicsJoint2D>(disc),
                "The joint handle (PhysicsJoint2D) was not removed after the break — the apply system did not "
                    + "strip it, so a consumer would still think the joint is live."
            );

            // After the break the disc free-falls: with the joint gone it drops well below where a rest-length-1
            // joint would have held it (the pin is at y=5, the held disc would sit near y=4). A free-fall over
            // the remaining steps puts it far below.
            var finalY = em.GetComponentData<Unity.Transforms.LocalToWorld>(disc).Position.y;
            Assert.Less(
                finalY,
                2f,
                $"The disc did not separate after the break: ended at y={finalY:F3} (a held joint keeps it near "
                    + "y=4; a broken joint lets it free-fall well below y=2)."
            );

            Debug.Log($"[PHYSICS2D-JOINTBREAK] disc broke and separated; finalY={finalY:F3}.");

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator UnbreakableJoint_HoldsForever()
        {
            // The negative control: the SAME hung disc with an INFINITE breakForce never breaks and stays held
            // near the rest position (pin at y=5, rest length 1 → held near y=4), proving the threshold is armed
            // only when finite (the default Infinity = never break).
            var world = MakeFixedWorld(out var group, withJoints: true);
            var em = world.EntityManager;

            var disc = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = new float2(0f, 4f),
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddComponentData(
                disc,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Distance,
                    connectedBody = Entity.Null,
                    anchor = Unity.Mathematics.float2.zero,
                    connectedAnchor = new float2(0f, 5f),
                    restLength = 1f,
                    enableSpring = false,
                    breakForce = float.PositiveInfinity,
                    breakTorque = float.PositiveInfinity,
                    breakAction = PhysicsJointBreakAction2D.Destroy,
                }
            );

            group.Update();
            for (var f = 0; f < 120; f++)
                group.Update();

            Assert.IsFalse(
                em.HasComponent<PhysicsJoint2DBroken>(disc),
                "An infinite-breakForce joint broke — the threshold was armed despite the value being Infinity "
                    + "(should never break)."
            );
            Assert.IsTrue(
                em.HasComponent<PhysicsJoint2D>(disc),
                "An infinite-breakForce joint lost its handle — it must stay live."
            );
            var finalY = em.GetComponentData<Unity.Transforms.LocalToWorld>(disc).Position.y;
            Assert.Greater(
                finalY,
                3f,
                $"The held disc fell to y={finalY:F3} — a rest-length-1 joint pinned at y=5 should hold it near "
                    + "y=4, not let it drop below y=3."
            );

            Debug.Log($"[PHYSICS2D-JOINTBREAK] unbreakable joint held; finalY={finalY:F3}.");

            world.Dispose();
            yield break;
        }

        [Test]
        public void InterpolatedPose_IsTheLerpOfBracketingPhysicsPoses()
        {
            // Internal-invariant test: drive PhysicsBody2DSmoothingSystem with a known previous pose, a known
            // current pose, and a known sub-step fraction, and assert the LocalToWorld it writes is the correct
            // lerp of the two bracketing poses. This is the gate framing the brief's scope note recommends:
            // GameObject interpolation is a render-time visual that a batchmode fixed loop cannot sample (no
            // sub-step time → timeAhead ≈ 0), so the package's interpolation is proven by the SMOOTHING MATH
            // (interpolated pose == lerp/extrapolate of the bracketing physics states), not a frame capture.
            var world = new World("Physics2DInterpSmoothMathWorld");
            var em = world.EntityManager;

            // A presentation group running the smoothing system, driven explicitly. The system is normally in
            // TransformSystemGroup at render rate; here it is created standalone and Update()d once.
            var smoothing = world.GetOrCreateSystem<PhysicsBody2DSmoothingSystem>();

            // The fixed-step time singleton the system reads: last step at elapsed=1.0 s, dt=1/60. Set the world
            // clock to 1.0 + half a step so timeAhead is exactly half the fixed step → normalizedTimeAhead=0.5.
            var timeSingleton = em.CreateEntity(typeof(PhysicsFixedStepTime2D));
            em.SetComponentData(
                timeSingleton,
                new PhysicsFixedStepTime2D { elapsedTime = 1.0, deltaTime = Dt }
            );
            world.SetTime(new Unity.Core.TimeData(elapsedTime: 1.0 + 0.5 * Dt, deltaTime: Dt));

            // A body whose previous pose is (0,0)/angle 0 and current pose is (2,4)/angle 90°. At t=0.5 the
            // interpolated position is the midpoint (1,2) and the angle is the nlerp of 0° and 90°.
            var prevPos = new float2(0f, 0f);
            sincos(0f, out var ps, out var pc);
            var curPos = new float2(2f, 4f);
            sincos(radians(90f), out var cs2, out var cc2);

            var body = em.CreateEntity(
                typeof(PhysicsBody2DSmoothing),
                typeof(Unity.Transforms.LocalToWorld)
            );
            em.SetComponentData(
                body,
                new PhysicsBody2DSmoothing
                {
                    prevPos = prevPos,
                    prevCosSin = new float2(pc, ps),
                    curPos = curPos,
                    curCosSin = new float2(cc2, cs2),
                    linearVel = Unity.Mathematics.float2.zero,
                    angularVelRad = 0f,
                    mode = (byte)PhysicsBody2DInterpolation.Interpolate,
                    hasPrev = 1,
                }
            );

            // Update the smoothing system once (it ScheduleParallels its job), then complete the job so the
            // LocalToWorld write is visible to the assertions below.
            smoothing.Update(world.Unmanaged);
            world.EntityManager.CompleteAllTrackedJobs();

            var m = em.GetComponentData<Unity.Transforms.LocalToWorld>(body).Value;
            var smoothedPos = new float2(m.c3.x, m.c3.y);
            // c0 = (R00, R10) = (cos, sin) of the smoothed angle.
            var smoothedAngle = atan2(m.c0.y, m.c0.x);

            // Expected: position is the exact midpoint; angle is the normalized lerp of (1,0) and (0,1) at 0.5,
            // which is (cos45°, sin45°) → 45°.
            var expectedPos = lerp(prevPos, curPos, 0.5f);
            var expectedAngle = radians(45f);

            Assert.Less(
                length(smoothedPos - expectedPos),
                1e-4f,
                $"Interpolated position {smoothedPos} is not the midpoint {expectedPos} of the bracketing poses "
                    + "at the half-step fraction."
            );
            Assert.Less(
                abs(smoothedAngle - expectedAngle),
                1e-3f,
                $"Interpolated angle {smoothedAngle} rad is not the nlerp ({expectedAngle} rad ≈ 45°) of the "
                    + "bracketing angles at the half-step fraction."
            );

            Debug.Log(
                $"[PHYSICS2D-INTERP] smoothedPos={smoothedPos} (expected {expectedPos}); "
                    + $"smoothedAngle={smoothedAngle:F4} rad (expected {expectedAngle:F4})."
            );

            world.Dispose();
        }
    }
}
