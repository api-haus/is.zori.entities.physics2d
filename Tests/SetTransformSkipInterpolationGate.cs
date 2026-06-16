using System.Collections;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif
using UnityEngine;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The e2e gate for the instantaneous-teleport surface added alongside the swept <c>Move*</c> commands:
    /// <see cref="PhysicsBody2DCommands.SetTransform"/> (a hard, NON-swept pose set — the native
    /// <c>b2Body_SetTransform</c> via the <c>PhysicsBody.transform</c> setter) and
    /// <see cref="PhysicsBody2DCommands.SkipInterpolation"/> (the 2D analogue of the 3D
    /// <c>CharacterInterpolation.SkipNextInterpolation()</c>). Built to falsify the two load-bearing claims from the
    /// surface's decision points, not the happy path:
    /// <list type="bullet">
    /// <item><b>No speed clamp.</b> A <c>SetTransform</c> moves a body to an arbitrarily far destination in ONE step,
    /// well past the swept move's per-step ceiling of <c>maximumLinearSpeed · dt</c> (≈400·dt ≈ 6.7 m at 60 Hz). A
    /// destination far beyond that ceiling proves the instantaneous set bypasses the velocity-based clamp the swept
    /// <c>SetTransformTarget</c> obeys.</item>
    /// <item><b>Streak suppression.</b> After a teleport, <c>SkipInterpolation</c> collapses the body's render-rate
    /// smoothing so prev == cur == the new pose with <c>hasPrev = 0</c> — the state the smoothing system reads as
    /// "write the current pose, do not interpolate," so no one-step slide from the old location to the new is drawn.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/> holding the package's four FixedStep systems,
    /// driven one fixed step per <c>group.Update()</c>: the first <c>group.Update()</c> creates the body (no step);
    /// each later one drains the command buffer onto the body and steps once. Bodies are authored directly via
    /// <see cref="DirectPhysics2DAuthoring"/>. The coroutines yield <c>null</c> only (never <c>WaitForEndOfFrame</c>,
    /// which does not tick in batchmode). No Burst/Jobs code is authored — every probe drives <c>group.Update()</c>
    /// and reads native poses on the main thread.
    /// </remarks>
    public sealed class SetTransformSkipInterpolationGate
    {
        const float Dt = 1f / 60f;

        // The default world maximumLinearSpeed (m/s) the swept SetTransformTarget is clamped to (the bake-contract
        // default; PhysicsWorldDefinition.maximumLinearSpeed). The per-step swept ceiling is this × dt.
        const float MaxLinearSpeed = 400f;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DSetTransformGateWorld", out group, Dt);

        static Entity SpawnDrivable(
            EntityManager em,
            float2 pos,
            PhysicsBody.BodyType bodyType,
            PhysicsBody2DInterpolation interpolation
        )
        {
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = bodyType,
                    gravityScale = 0f,
                    initialPosition = pos,
                    useAutoMass = true,
                    interpolation = interpolation,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddBuffer<PhysicsBody2DCommand>(entity);
            return entity;
        }

        static PhysicsBody BodyOf(EntityManager em, Entity e) => em.GetComponentData<PhysicsBody2D>(e).body;

        static DynamicBuffer<PhysicsBody2DCommand> CommandsOf(EntityManager em, Entity e) =>
            em.GetBuffer<PhysicsBody2DCommand>(e);

        static float2 PosOf(PhysicsBody b) => (float2)(Vector2)b.position;

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 1 — SetTransform is INSTANTANEOUS and UNCLAMPED: a body teleports to an arbitrarily far
        // destination in ONE step. Decision point: kind SetTransform writes body.transform directly (native
        // b2Body_SetTransform), with no deltaTime and no velocity, so the world maximumLinearSpeed clamp (which
        // gates only the swept SetTransformTarget) never applies. Falsification: a destination far beyond the swept
        // per-step ceiling (maximumLinearSpeed · dt ≈ 6.7 m). A swept MovePosition to the same target would clamp to
        // ≤ that ceiling this step; SetTransform must land EXACTLY at the destination regardless of distance.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator SetTransform_ReachesFarDestinationInOneStep_NoSpeedClamp()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A DYNAMIC body so the clamp would bite a swept move: maximumLinearSpeed gates dynamic/kinematic
            // velocity alike, and the instantaneous set bypasses it for either.
            var entity = SpawnDrivable(
                em,
                new float2(0f, 0f),
                PhysicsBody.BodyType.Dynamic,
                PhysicsBody2DInterpolation.None
            );

            group.Update(); // create (no step)
            var body = BodyOf(em, entity);
            Assert.IsTrue(body.isValid, "Body was not created on the first update.");

            // A destination far beyond the swept per-step ceiling. The swept SetTransformTarget could move at most
            // maximumLinearSpeed · dt ≈ 6.67 m in one step; this target is two orders of magnitude past it.
            var sweptCeiling = MaxLinearSpeed * Dt;
            var target = new float2(1000f, -750f); // |target| = 1250 m, far past the ~6.7 m swept ceiling
            Assert.Greater(
                length(target),
                sweptCeiling * 10f,
                "Test target is not far enough past the swept ceiling to distinguish the instantaneous set."
            );

            PhysicsBody2DCommands.SetTransform(CommandsOf(em, entity), target, 0f);

            // One step drains the SetTransform onto the body and steps once.
            group.Update();

            var landed = PosOf(body);
            // EXACT (within FP noise): a hard transform set lands at the destination, no swept tolerance.
            Assert.Less(
                length(landed - target),
                1e-3f,
                $"SetTransform did not land the body at the far destination in one step. landed={landed}, "
                    + $"target={target} (|target|={length(target):F1} m, swept ceiling≈{sweptCeiling:F2} m). The "
                    + "instantaneous set either did not bypass the speed clamp or did not write body.transform."
            );
            // The displacement this step vastly exceeds the swept ceiling — the clamp was bypassed.
            Assert.Greater(
                length(landed),
                sweptCeiling * 10f,
                $"Body moved only {length(landed):F2} m — within the swept clamp ceiling ({sweptCeiling:F2} m). "
                    + "SetTransform behaved like a clamped swept move, not an instantaneous set."
            );

            Debug.Log(
                $"[PHYSICS2D-TELEPORT] SetTransform target={target} (|t|={length(target):F1} m, swept "
                    + $"ceiling≈{sweptCeiling:F2} m) → landed={landed} in ONE step (unclamped)."
            );

            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 1b — SetPosition keeps rotation; SetRotation keeps position (the per-axis flag). Decision point:
        // the worldPoint.x flag bits select which axis the instantaneous set writes; the unchanged axis keeps the
        // body's CURRENT value. Falsification: a SetPosition that also snapped rotation (or vice versa) would move
        // the axis it must leave alone.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator SetPosition_KeepsRotation_SetRotation_KeepsPosition()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var entity = SpawnDrivable(
                em,
                new float2(0f, 0f),
                PhysicsBody.BodyType.Kinematic,
                PhysicsBody2DInterpolation.None
            );
            group.Update();
            var body = BodyOf(em, entity);

            // Position-only set: move far, rotation must stay 0.
            var farPos = new float2(500f, 200f);
            PhysicsBody2DCommands.SetPosition(CommandsOf(em, entity), farPos);
            group.Update();
            Assert.Less(length(PosOf(body) - farPos), 1e-3f, "SetPosition did not land at the far position.");
            Assert.Less(
                abs(body.rotation.radians),
                1e-3f,
                $"SetPosition snapped the rotation to {body.rotation.radians} rad — it must keep the current 0."
            );

            // Rotation-only set: spin to π/2, position must stay at farPos.
            var targetAngle = math.radians(90f);
            PhysicsBody2DCommands.SetRotation(CommandsOf(em, entity), targetAngle);
            group.Update();
            Assert.Less(
                abs(body.rotation.radians - targetAngle),
                1e-3f,
                $"SetRotation did not land the exact angle. got={body.rotation.radians}, target={targetAngle}."
            );
            Assert.Less(
                length(PosOf(body) - farPos),
                1e-3f,
                $"SetRotation moved the position to {PosOf(body)} — it must keep the current {farPos}."
            );

            Debug.Log(
                $"[PHYSICS2D-TELEPORT] SetPosition→{farPos} kept rot 0; SetRotation→{targetAngle:F4} rad kept "
                    + $"pos {farPos}."
            );

            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 2 — SkipInterpolation suppresses the smoothing streak. Decision point: kind SkipInterpolation
        // resets the body's PhysicsBody2DSmoothing to prev == cur == the live pose with hasPrev = 0 (the state the
        // smoothing system reads as "write current, do not interpolate"). Falsification: BEFORE the skip, a body that
        // has stepped while moving carries prev != cur and hasPrev = 1 (a streak WOULD render); a SetTransform alone
        // leaves the stale prev pointing at the old location, so the smoothing system would slide from old to new
        // over one render step. After SetTransform + SkipInterpolation, prev == cur == the destination and hasPrev
        // == 0 — no streak. Pins that the reset reads the POST-SetTransform pose (it must drain after it).
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator SkipInterpolation_CollapsesSmoothingToCurrentPose_NoStreak()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // An INTERPOLATE body so it carries PhysicsBody2DSmoothing (a None body has nothing to reset).
            var entity = SpawnDrivable(
                em,
                new float2(0f, 0f),
                PhysicsBody.BodyType.Kinematic,
                PhysicsBody2DInterpolation.Interpolate
            );
            group.Update(); // create (no step)
            var body = BodyOf(em, entity);
            Assert.IsTrue(
                em.HasComponent<PhysicsBody2DSmoothing>(entity),
                "Interpolate body did not get a PhysicsBody2DSmoothing component."
            );

            // Drive the body so the write-back captures two DISTINCT poses (prev != cur, hasPrev = 1) — the state a
            // streak would render from. Use a swept MovePosition each step to a moving target near the origin.
            PhysicsBody2DCommands.MovePosition(CommandsOf(em, entity), new float2(1f, 0f));
            group.Update();
            PhysicsBody2DCommands.MovePosition(CommandsOf(em, entity), new float2(2f, 0f));
            group.Update();

            var preSkip = em.GetComponentData<PhysicsBody2DSmoothing>(entity);
            Assert.AreEqual(
                (byte)1,
                preSkip.hasPrev,
                "Pre-skip: the body has not captured two poses yet — the test cannot prove a streak is suppressed."
            );
            Assert.Greater(
                length(preSkip.curPos - preSkip.prevPos),
                1e-3f,
                $"Pre-skip: prev ({preSkip.prevPos}) == cur ({preSkip.curPos}) — no streak exists to suppress, so "
                    + "the test's premise is not met (the body must have moved between the two captured steps)."
            );

            // Respawn-style teleport in ONE frame: zero the kinematic velocity (so the swept MovePosition residual
            // does not integrate the body off the destination during this step), hard-set the transform, then skip
            // interpolation. The SkipInterpolation reset reads the body's pose AFTER the SetTransform drains (both in
            // buffer order), so it collapses the smoothing to the NEW pose.
            var teleportTarget = new float2(300f, -120f);
            PhysicsBody2DCommands.SetLinearVelocity(CommandsOf(em, entity), new float2(0f, 0f));
            PhysicsBody2DCommands.SetTransform(CommandsOf(em, entity), teleportTarget, 0f);
            PhysicsBody2DCommands.SkipInterpolation(CommandsOf(em, entity));
            group.Update();

            var landed = PosOf(body);
            Assert.Less(
                length(landed - teleportTarget),
                1e-3f,
                $"Body did not teleport to {teleportTarget} (landed {landed}) — SetTransform did not run."
            );

            // After the write-back ran this step, CaptureSmoothing shifts cur→prev and writes the new cur. The
            // critical pin is at DRAIN time (pre-step): SkipInterpolation set prev == cur == teleportTarget and
            // hasPrev = 0. The write-back then captured the SAME teleportTarget pose into cur (the body did not move
            // after the instantaneous set, since a kinematic body with no velocity does not integrate), so prev
            // (== teleportTarget, from the reset) == cur (== teleportTarget, just captured) — still no streak. We
            // assert the no-streak END STATE: prev == cur == teleportTarget. (Had SkipInterpolation NOT run, prev
            // would be the pre-teleport ≈(2,0) pose and cur the teleportTarget, a 300+ m streak.)
            var postSkip = em.GetComponentData<PhysicsBody2DSmoothing>(entity);
            Assert.Less(
                length(postSkip.curPos - postSkip.prevPos),
                1e-3f,
                $"Post-skip: prev ({postSkip.prevPos}) != cur ({postSkip.curPos}) — a {length(postSkip.curPos - postSkip.prevPos):F1} m "
                    + "streak survives. SkipInterpolation did not collapse the smoothing to the teleported pose."
            );
            Assert.Less(
                length(postSkip.curPos - teleportTarget),
                1e-3f,
                $"Post-skip: cur ({postSkip.curPos}) is not the teleport destination ({teleportTarget})."
            );

            Debug.Log(
                $"[PHYSICS2D-TELEPORT] Pre-skip streak prev={preSkip.prevPos}→cur={preSkip.curPos} (hasPrev=1); "
                    + $"after SetTransform({teleportTarget})+SkipInterpolation: prev={postSkip.prevPos} == "
                    + $"cur={postSkip.curPos} (no streak)."
            );

            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 2b — SkipInterpolation on a NON-interpolated body is a safe no-op. Decision point: a body with
        // no PhysicsBody2DSmoothing (interpolation None) has no streak to suppress; the reset is a HasComponent miss.
        // Falsification: the command must not throw or add a component to a None body.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator SkipInterpolation_OnNonInterpolatedBody_IsNoOp()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var entity = SpawnDrivable(
                em,
                new float2(0f, 0f),
                PhysicsBody.BodyType.Kinematic,
                PhysicsBody2DInterpolation.None
            );
            group.Update();
            Assert.IsFalse(
                em.HasComponent<PhysicsBody2DSmoothing>(entity),
                "A None-interpolation body must not carry a smoothing component."
            );

            PhysicsBody2DCommands.SetTransform(CommandsOf(em, entity), new float2(50f, 50f), 0f);
            PhysicsBody2DCommands.SkipInterpolation(CommandsOf(em, entity));
            // Must drain without throwing and without adding a smoothing component.
            Assert.DoesNotThrow(() => group.Update());
            Assert.IsFalse(
                em.HasComponent<PhysicsBody2DSmoothing>(entity),
                "SkipInterpolation added a smoothing component to a None-interpolation body (must be a no-op)."
            );
            Assert.Less(
                length(PosOf(BodyOf(em, entity)) - new float2(50f, 50f)),
                1e-3f,
                "The SetTransform alongside the no-op SkipInterpolation still teleported the body."
            );

            world.Dispose();
            yield break;
        }
    }
}
