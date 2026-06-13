using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The independent adversarial behavioural gate for the Phase-A center-of-mass + rotational-inertia
    /// override (<see cref="Authoring.PhysicsBody2DAuthoring.OverrideMassDistribution"/> /
    /// <see cref="Authoring.PhysicsBody2DAuthoring.CenterOfMass"/> /
    /// <see cref="Authoring.PhysicsBody2DAuthoring.RotationalInertia"/>, baked to
    /// <see cref="PhysicsBody2DDefinition"/> and applied through <c>PhysicsBody.massConfiguration</c> in
    /// <c>PhysicsWorld2DSystem.ApplyMassDistributionOverride</c>). The override was only code-path-verified by
    /// the implementor; this gate proves the override changes a body's ROTATIONAL DYNAMICS the way the physics
    /// demands, against the GameObject <c>Rigidbody2D.centerOfMass</c>/<c>.inertia</c> oracle and the analytic
    /// expectation.
    /// </summary>
    /// <remarks>
    /// <para><b>Why a SHORT, near-impulse horizon.</b> The probes measure the rotational response right after a
    /// single off-centre impulse / one torque step, not a long settle. A long-horizon spin under a continuous
    /// torque diverges across the two backends for a reason unrelated to the override: the package
    /// <see cref="PhysicsBody2DDefinition"/> exposes no per-body sleep opt-out, so a slowly-spinning package body
    /// falls asleep mid-run and freezes while the <c>NeverSleep</c> GameObject reference keeps accelerating — a
    /// SLEEP confound, not a mass-distribution one. (A single-step probe confirmed the two backends' torque→
    /// angular-velocity response is bit-close: a unit box gains 11.459°/s on the package vs 11.450°/s on the
    /// GameObject under the same torque, both at the analytic inertia 0.16667.) So the gate reads the angular
    /// velocity at the impulse/first step, before sleep can act, where the override's effect is clean.</para>
    ///
    /// <para><b>Why an off-centre impulse is the falsifying probe for the COM override.</b> An impulse applied at
    /// the body's geometric origin produces a torque about the centre of mass equal to <c>r × J</c> where
    /// <c>r</c> is the COM→application-point vector. With a centred COM (the default) that cross product is zero,
    /// so the body translates without spin; with the COM offset, the same impulse spins the body. A baker that
    /// dropped the COM field leaves the body centred and it will NOT spin — the override fails open and this gate
    /// catches it. The induced angular velocity's sign and magnitude are asserted against a GameObject
    /// <c>Rigidbody2D</c> whose <c>centerOfMass</c> is the same offset, and the sign cross-checked analytically.</para>
    ///
    /// <para><b>Why equal torque under different inertia is the falsifying probe for the inertia override.</b>
    /// Angular acceleration is <c>torque / rotationalInertia</c>, so over one step two bodies given the identical
    /// torque gain angular velocity at the inverse ratio of their inertias. A baker that dropped the inertia
    /// field leaves both on the shape inertia and they gain ω identically — the gate catches that by requiring
    /// the non-default-inertia body to gain ω SLOWER by the inertia ratio, cross-checked against a GameObject
    /// body with the same <c>Rigidbody2D.inertia</c>.</para>
    ///
    /// <para><b>World isolation.</b> Each test runs in its OWN disposable <see cref="World"/>; the GameObject
    /// references and global <c>Physics2D</c> knobs are torn down/restored in <c>[SetUp]</c>/<c>[TearDown]</c>.
    /// The dynamics are deterministic, so two runs produce the identical witnesses.</para>
    /// </remarks>
    public sealed class PhaseAMassDistributionGate
    {
        const float Dt = 1f / 60f;
        static readonly Vector2 NoGravity = Vector2.zero;

        SimulationMode2D _prevMode;
        Vector2 _prevGravity;

        [SetUp]
        public void SetUp()
        {
            _prevMode = UnityEngine.Physics2D.simulationMode;
            _prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = NoGravity;
        }

        [TearDown]
        public void TearDown()
        {
            UnityEngine.Physics2D.gravity = _prevGravity;
            UnityEngine.Physics2D.simulationMode = _prevMode;
        }

        static World MakePackageWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("PhaseAMassGateWorld", out group, Dt);

        // =====================================================================================================
        // INVARIANT — COM override induces spin under a centre-origin impulse. A box body authored with an
        // OFFSET centre of mass, struck by an impulse through its geometric origin, gains angular velocity
        // (torque about the COM = r × J ≠ 0); a centred (default) body under the same impulse does not. The
        // induced angular velocity's SIGN and magnitude match a GameObject Rigidbody2D with the same offset
        // centerOfMass, and the resolved live COM equals the authored override.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator ComOverride_OffCenterImpulse_InducesSpin_MatchesGameObjectCenterOfMass()
        {
            var size = new float2(1f, 1f); // unit box, mass 1, shape inertia ≈ 0.16667 about the centroid
            var com = new float2(0.4f, 0f); // COM offset along +X
            var impulse = new float2(0f, 3f); // applied at the world origin (the geometric centre)

            // ---- package: COM-overridden body + a centred control, struck at the origin ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var overridden = SpawnPkgBox(
                em,
                Unity.Mathematics.float2.zero,
                size,
                overrideMass: true,
                com: com,
                inertia: 0f
            );
            var control = SpawnPkgBox(
                em,
                new float2(6f, 0f),
                size,
                overrideMass: false,
                com: Unity.Mathematics.float2.zero,
                inertia: 0f
            );
            em.AddBuffer<PhysicsBody2DCommand>(overridden);
            em.AddBuffer<PhysicsBody2DCommand>(control);

            group.Update(); // create

            // The impulse at each body's geometric origin (lever arm = the COM offset). One-shot: append, step
            // once; the angular velocity right after the impulse is the clean, sleep-free observable.
            PhysicsBody2DCommands.AddForceAtPosition(
                em.GetBuffer<PhysicsBody2DCommand>(overridden),
                impulse,
                em.GetComponentData<LocalToWorld>(overridden).Value.c3.xy,
                PhysicsForceMode2D.Impulse
            );
            PhysicsBody2DCommands.AddForceAtPosition(
                em.GetBuffer<PhysicsBody2DCommand>(control),
                impulse,
                em.GetComponentData<LocalToWorld>(control).Value.c3.xy,
                PhysicsForceMode2D.Impulse
            );
            group.Update(); // the impulse step

            var overBody = em.GetComponentData<PhysicsBody2D>(overridden).body;
            var ctrlBody = em.GetComponentData<PhysicsBody2D>(control).body;
            var pkgOverOmega = radians(overBody.angularVelocity); // rad/s (engine angularVelocity is deg/s)
            var pkgCtrlOmega = radians(ctrlBody.angularVelocity);
            var pkgResolvedCom = (float2)(Vector2)overBody.massConfiguration.center;
            world.Dispose();

            // ---- GameObject oracle: a Rigidbody2D with the same offset centerOfMass, struck the same way ----
            var goOmega = RunGoComImpulse(size, com, impulse, out var goResolvedCom);

            // Analytic torque sign about the COM: r = origin − COM = (−0.4, 0); J = (0, +3); cross_z < 0.
            var analyticCrossZ = (-com.x) * impulse.y - (-com.y) * impulse.x;

            Debug.Log(
                $"[PHYSICS2D-PHASEA-COM] pkgOverOmega={pkgOverOmega:F5} rad/s pkgCtrlOmega={pkgCtrlOmega:E3} rad/s "
                    + $"goOmega={goOmega:F5} rad/s | pkgResolvedCOM={pkgResolvedCom} goResolvedCOM={goResolvedCom} "
                    + $"| analyticCrossZ={analyticCrossZ:F3}."
            );

            // 1. The centred control gains no spin from a centre impulse.
            Assert.Less(
                abs(pkgCtrlOmega),
                1e-3f,
                $"A centred-COM control gained angular velocity {pkgCtrlOmega} rad/s from a centre-origin impulse "
                    + "— the default mass distribution is not centred or the override leaked onto the control."
            );
            // 2. The overridden body DID spin (the COM override reached the live body).
            Assert.Greater(
                abs(pkgOverOmega),
                0.5f,
                $"The COM-overridden body barely spun ({pkgOverOmega} rad/s) under an off-centre-relative-to-COM "
                    + "impulse — OverrideMassDistribution.CenterOfMass did not reach the live body."
            );
            // 3. The resolved live COM equals the authored override (the direct bake→apply witness).
            Assert.Less(
                length(pkgResolvedCom - com),
                1e-4f,
                $"The live body's massConfiguration.center={pkgResolvedCom} != the authored COM={com}."
            );
            // 4. The induced spin sign matches the analytic torque sign.
            Assert.AreEqual(
                sign(analyticCrossZ),
                sign(pkgOverOmega),
                $"The induced spin sign ({sign(pkgOverOmega)}) disagrees with the analytic torque sign "
                    + $"({sign(analyticCrossZ)}) — the COM offset is applied with the wrong sign/axis."
            );
            // 5. GameObject parity: same sign and a magnitude within a tight band (the single-step torque→ω
            //    response is bit-close across backends, so the off-centre-impulse Δω matches well).
            Assert.AreEqual(
                sign(goOmega),
                sign(pkgOverOmega),
                $"Package spin sign ({sign(pkgOverOmega)}) disagrees with the GameObject centerOfMass oracle "
                    + $"sign ({sign(goOmega)})."
            );
            Assert.Less(
                length(goResolvedCom - com),
                1e-3f,
                $"GameObject oracle's resolved centerOfMass {goResolvedCom} != the authored offset {com}."
            );
            var ratio = abs(pkgOverOmega) / max(abs(goOmega), 1e-6f);
            Assert.That(
                ratio,
                Is.InRange(0.7f, 1.4f),
                $"Package-vs-GameObject induced angular-velocity ratio {ratio} is outside [0.7, 1.4] — the COM "
                    + "lever arm or inertia magnitude diverges from the GameObject oracle."
            );
            yield break;
        }

        // =====================================================================================================
        // INVARIANT — rotational-inertia override scales the per-step angular-velocity gain. Over one torque
        // step, a body authored with a NON-default (larger) rotational inertia gains LESS angular velocity than
        // a default-inertia control by the inverse inertia ratio (Δω = (τ/I)·dt). A dropped inertia field leaves
        // both on the shape inertia → identical Δω, which this gate falsifies. Cross-checked against a GameObject
        // Rigidbody2D.inertia oracle.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator InertiaOverride_EqualTorque_SlowsSpin_MatchesGameObjectInertia()
        {
            var size = new float2(1f, 1f); // mass 1, shape inertia ≈ 0.16667
            const float torque = 2f;
            const float bigInertia = 1.0f; // ~6× the shape inertia

            // ---- package: default-inertia control + inertia-overridden body, one torque step ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var control = SpawnPkgBox(
                em,
                Unity.Mathematics.float2.zero,
                size,
                overrideMass: false,
                com: Unity.Mathematics.float2.zero,
                inertia: 0f
            );
            var overridden = SpawnPkgBox(
                em,
                new float2(6f, 0f),
                size,
                overrideMass: true,
                com: Unity.Mathematics.float2.zero,
                inertia: bigInertia
            );
            em.AddBuffer<PhysicsBody2DCommand>(control);
            em.AddBuffer<PhysicsBody2DCommand>(overridden);

            group.Update(); // create

            PhysicsBody2DCommands.AddTorque(
                em.GetBuffer<PhysicsBody2DCommand>(control),
                torque,
                PhysicsForceMode2D.Force
            );
            PhysicsBody2DCommands.AddTorque(
                em.GetBuffer<PhysicsBody2DCommand>(overridden),
                torque,
                PhysicsForceMode2D.Force
            );
            group.Update(); // the torque step

            var ctrlBody = em.GetComponentData<PhysicsBody2D>(control).body;
            var overBody = em.GetComponentData<PhysicsBody2D>(overridden).body;
            var ctrlOmega = radians(ctrlBody.angularVelocity);
            var overOmega = radians(overBody.angularVelocity);
            var pkgCtrlInertia = ctrlBody.massConfiguration.rotationalInertia;
            var pkgOverInertia = overBody.massConfiguration.rotationalInertia;
            world.Dispose();

            // ---- GameObject oracle: control vs Rigidbody2D.inertia = bigInertia, one torque step each ----
            RunGoInertiaTorque(
                size,
                torque,
                bigInertia,
                out var goCtrlOmega,
                out var goOverOmega,
                out var goOverInertia
            );

            var pkgOmegaRatio = abs(overOmega) / max(abs(ctrlOmega), 1e-6f);
            var expectedRatio = pkgCtrlInertia / pkgOverInertia; // ≈ 0.16667 / 1.0

            Debug.Log(
                $"[PHYSICS2D-PHASEA-INERTIA] pkg ctrlOmega={ctrlOmega:F5} overOmega={overOmega:F5} ratio={pkgOmegaRatio:F4} "
                    + $"| pkg inertias ctrl={pkgCtrlInertia:F5} over={pkgOverInertia:F5} expectedRatio={expectedRatio:F4} "
                    + $"| GO ctrlOmega={goCtrlOmega:F5} overOmega={goOverOmega:F5} GO overInertia={goOverInertia:F5}."
            );

            // 1. The override reached the live body: its resolved inertia is the authored value.
            Assert.Less(
                abs(pkgOverInertia - bigInertia),
                1e-3f,
                $"The overridden body's rotationalInertia={pkgOverInertia} != the authored {bigInertia}."
            );
            // 2. The control kept its (distinctly smaller) shape inertia, so the probe is non-degenerate.
            Assert.Less(
                pkgCtrlInertia,
                bigInertia * 0.6f,
                $"The control body's shape inertia {pkgCtrlInertia} is not clearly below the override {bigInertia}."
            );
            // 3. The overridden body gained LESS angular velocity (larger inertia ⇒ smaller Δω under equal torque).
            Assert.Less(
                abs(overOmega),
                abs(ctrlOmega),
                $"The inertia-overridden body ({overOmega} rad/s) did not gain less angular velocity than the "
                    + $"default-inertia control ({ctrlOmega} rad/s) under equal torque — the override had no effect."
            );
            // 4. The Δω ratio matches the inverse inertia ratio (Δω = (τ/I)·dt) within a tight band.
            Assert.That(
                pkgOmegaRatio / expectedRatio,
                Is.InRange(0.85f, 1.15f),
                $"The angular-velocity ratio {pkgOmegaRatio} does not track the inverse inertia ratio "
                    + $"{expectedRatio} within [0.85, 1.15]× — the inertia magnitude is wrong."
            );
            // 5. GameObject parity: the oracle with Rigidbody2D.inertia = bigInertia gains ω at the same ratio.
            var goOmegaRatio = abs(goOverOmega) / max(abs(goCtrlOmega), 1e-6f);
            Assert.That(
                pkgOmegaRatio / max(goOmegaRatio, 1e-6f),
                Is.InRange(0.85f, 1.18f),
                $"Package Δω-ratio {pkgOmegaRatio} vs GameObject Δω-ratio {goOmegaRatio} diverge beyond "
                    + "[0.85, 1.18]× — the package inertia override does not track the GameObject Rigidbody2D.inertia."
            );
            yield break;
        }

        // -----------------------------------------------------------------------------------------------------
        // A dynamic box with explicit (non-auto) mass 1 and the optional COM/inertia override. useAutoMass=false
        // + default mass 1 hits ApplyMass's default-mass no-perturb branch (it leaves the shape's auto mass when
        // it is already positive), then ApplyMassDistributionOverride applies the override on top.
        // -----------------------------------------------------------------------------------------------------
        static Entity SpawnPkgBox(
            EntityManager em,
            float2 pos,
            float2 size,
            bool overrideMass,
            float2 com,
            float inertia
        )
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f,
                    initialPosition = pos,
                    useAutoMass = false,
                    mass = 1f,
                    overrideMassDistribution = overrideMass,
                    centerOfMass = com,
                    rotationalInertia = inertia,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = size,
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
        }

        // GameObject oracle for the COM probe: a Rigidbody2D box with centerOfMass = com, struck by the same
        // impulse at its world origin; returns the angular velocity (rad/s) right after one Simulate.
        static float RunGoComImpulse(float2 size, float2 com, float2 impulse, out float2 resolvedCom)
        {
            var track = new List<GameObject>();
            var go = new GameObject("GoComBody") { layer = 0 };
            go.transform.position = Vector3.zero;
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 0f;
            rb.useAutoMass = false;
            rb.mass = 1f;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            rb.centerOfMass = (Vector2)com;
            go.AddComponent<BoxCollider2D>().size = (Vector2)size;
            track.Add(go);
            UnityEngine.Physics2D.SyncTransforms();
            resolvedCom = new float2(rb.centerOfMass.x, rb.centerOfMass.y);

            rb.AddForceAtPosition((Vector2)impulse, go.transform.position, ForceMode2D.Impulse);
            UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            var omega = radians(rb.angularVelocity);
            foreach (var g in track)
                Object.Destroy(g);
            return omega;
        }

        // GameObject oracle for the inertia probe: a default-inertia control and an inertia-overridden body, each
        // given the same torque for ONE step; returns each angular velocity (rad/s) and the override's inertia.
        static void RunGoInertiaTorque(
            float2 size,
            float torque,
            float bigInertia,
            out float ctrlOmega,
            out float overOmega,
            out float overInertia
        )
        {
            var track = new List<GameObject>();

            var ctrlGo = new GameObject("GoInertiaCtrl") { layer = 0 };
            ctrlGo.transform.position = Vector3.zero;
            var ctrlRb = ctrlGo.AddComponent<Rigidbody2D>();
            ctrlRb.bodyType = RigidbodyType2D.Dynamic;
            ctrlRb.gravityScale = 0f;
            ctrlRb.useAutoMass = false;
            ctrlRb.mass = 1f;
            ctrlRb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            ctrlGo.AddComponent<BoxCollider2D>().size = (Vector2)size;
            track.Add(ctrlGo);

            var overGo = new GameObject("GoInertiaOver") { layer = 0 };
            overGo.transform.position = new Vector3(6f, 0f, 0f);
            var overRb = overGo.AddComponent<Rigidbody2D>();
            overRb.bodyType = RigidbodyType2D.Dynamic;
            overRb.gravityScale = 0f;
            overRb.useAutoMass = false;
            overRb.mass = 1f;
            overRb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            overGo.AddComponent<BoxCollider2D>().size = (Vector2)size;
            overRb.inertia = bigInertia; // explicit inertia override (set after the collider so it is not recomputed)
            track.Add(overGo);

            UnityEngine.Physics2D.SyncTransforms();
            overInertia = overRb.inertia;

            ctrlRb.AddTorque(torque, ForceMode2D.Force);
            overRb.AddTorque(torque, ForceMode2D.Force);
            UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            ctrlOmega = radians(ctrlRb.angularVelocity);
            overOmega = radians(overRb.angularVelocity);
            foreach (var g in track)
                Object.Destroy(g);
        }
    }
}
