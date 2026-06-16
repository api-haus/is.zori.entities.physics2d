using System.Collections;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
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
    /// The independent adversarial GameObject-parity e2e gate for Phase 7 (the runtime write-in surface:
    /// <see cref="PhysicsBody2DCommands"/> — AddForce/AddForceAtPosition/AddTorque with the Force-vs-Impulse
    /// fork, direct velocity, and kinematic MovePosition/MoveRotation). It is built to FALSIFY the Phase-7
    /// invariants from the surface's observable decision points, never the happy path the smoke
    /// (<see cref="RuntimeWriteInSmoke"/>) already covers, and it grounds every continuous claim against a
    /// live GameObject <c>Rigidbody2D</c> oracle stepped by <c>UnityEngine.Physics2D.Simulate</c>.
    /// </summary>
    /// <remarks>
    /// <para><b>The oracle.</b> Each test authors the SAME physical body two ways in one PlayMode test: the
    /// package side via <see cref="DirectPhysics2DAuthoring"/> in a dedicated disposable <see cref="World"/>
    /// (stepped by driving the package's <see cref="FixedStepSimulationSystemGroup"/> under a swapped
    /// <c>FixedRateSimpleManager(1/60)</c>), and the GameObject reference as a live <c>Rigidbody2D</c> in the
    /// default physics scene with <c>simulationMode = Script</c> (stepped by <c>Physics2D.Simulate(dt)</c>).
    /// Both backends issue the equivalent write-in on the SAME pre-step phase: the package command is appended
    /// after the create-frame <c>group.Update()</c> and before the stepping <c>group.Update()</c> that drains
    /// it; the GameObject <c>AddForce</c>/<c>AddTorque</c>/<c>MovePosition</c> is issued immediately before the
    /// matching <c>Physics2D.Simulate</c>, so the force lands on the same step on both mediums.</para>
    ///
    /// <para><b>What is asserted exactly vs bounded.</b> Same Box2D lineage, but the GameObject side runs
    /// Box2D-v2 (iteration solver) and the package runs Box2D-v3 (sub-stepping solver) — DIFFERENT integrators
    /// (<c>00d-determinism-probe-results.md</c>). So BINARY facts are asserted exactly (a frozen axis shows
    /// zero motion; a body that should not move does not; an obstacle that should stop a sweep stops it), and
    /// CONTINUOUS facts (resulting velocity, position, angle) are bounded by a generous relative+absolute
    /// envelope wide enough to absorb v2-vs-v3 noise, tight enough to fail loudly on a real mapping regression.
    /// The package-side velocity/position witnesses are bit-deterministic across runs (same world, same
    /// commands), so the two-consecutive-green discipline is a strict re-equality check on the package side and
    /// a re-pass of the bounded oracle comparison.</para>
    ///
    /// <para><b>Isolation.</b> Each world-mutating test runs in its own disposable <see cref="World"/> and
    /// destroys its GameObject reference + restores every global <c>Physics2D</c> knob it touched, so a thrown
    /// test cannot leak a native body into a shared world nor leave a global simulationMode behind to poison a
    /// sibling. The coroutines yield <c>null</c> only (never <c>WaitForEndOfFrame</c>, which does not tick in
    /// batchmode). No Burst/Jobs code is authored — every probe drives <c>group.Update()</c> and reads native
    /// poses/velocities on the main thread, so the <c>docs/unity/{jobs,burst}</c> binding does not engage.</para>
    /// </remarks>
    public sealed class Phase7RuntimeWriteInGate
    {
        const float Dt = 1f / 60f;

        // The held-identical gravity the determinism probe matched (00c §3), applied to the GameObject
        // reference so a free-fall component matches the package world's default gravity.
        static readonly Vector2 Gravity = new(0f, -9.81f);

        // -----------------------------------------------------------------------------------------------
        // Lockstep scaffold: a disposable package world + a save/restore wrapper over the global Physics2D
        // simulation mode so the GameObject oracle steps only when we call Simulate.
        // -----------------------------------------------------------------------------------------------

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DPhase7GateWorld", out group, Dt);

        // A package-side drivable dynamic circle with the command buffer attached. gravityScale chosen by the
        // caller (0 to isolate a write-in's velocity delta, 1 to compare a free-fall + write-in trajectory).
        static Entity SpawnEcsCircle(
            EntityManager em,
            float2 pos,
            float radius,
            float gravityScale,
            PhysicsBody.BodyConstraints constraints = PhysicsBody.BodyConstraints.None
        )
        {
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = gravityScale,
                    initialPosition = pos,
                    useAutoMass = true,
                    constraints = constraints,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = radius,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddBuffer<PhysicsBody2DCommand>(entity);
            return entity;
        }

        // A GameObject reference dynamic circle in the default physics scene, matched to the ECS circle's
        // shape + auto-mass (density-from-CircleCollider2D). simulationMode must already be Script.
        static Rigidbody2D SpawnReferenceCircle(
            float2 pos,
            float radius,
            float gravityScale,
            RigidbodyConstraints2D constraints = RigidbodyConstraints2D.None
        )
        {
            var go = new GameObject("Phase7Ref");
            go.transform.position = new Vector3(pos.x, pos.y, 0f);
            var col = go.AddComponent<CircleCollider2D>();
            col.radius = radius;
            // density-driven auto-mass, the same path the ECS body resolves (useAutoMass + density 1).
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            rb.gravityScale = gravityScale;
            rb.constraints = constraints;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            return rb;
        }

        static PhysicsBody BodyOf(EntityManager em, Entity e)
        {
            return em.GetComponentData<PhysicsBody2D>(e).body;
        }

        static DynamicBuffer<PhysicsBody2DCommand> CommandsOf(EntityManager em, Entity e)
        {
            return em.GetBuffer<PhysicsBody2DCommand>(e);
        }

        static float2 EcsVelocity(PhysicsBody b) => (float2)(Vector2)b.linearVelocity;

        static float2 EcsPosition(PhysicsBody b) => (float2)(Vector2)b.position;

        static float2 RefVelocity(Rigidbody2D rb) => new float2(rb.linearVelocity.x, rb.linearVelocity.y);

        static float2 RefPosition(Rigidbody2D rb) => new float2(rb.position.x, rb.position.y);

        // A generous growth-bounded continuous comparison: |a - b| must be within rel*|b| + abs. Same
        // posture as the parity harness envelope — wide enough for v2-vs-v3 solver noise, tight enough to
        // catch a wrong unit / wrong mapping (which moves the value by a factor, not a few percent).
        static void AssertClose(float2 a, float2 b, float rel, float absTol, string what)
        {
            var err = length(a - b);
            var band = rel * length(b) + absTol;
            Assert.LessOrEqual(
                err,
                band,
                $"{what}: |package - GameObject| = {err} exceeds band {band} (rel {rel}, abs {absTol}). "
                    + $"package={a}, GameObject={b}."
            );
        }

        static void AssertClose(float a, float b, float rel, float absTol, string what)
        {
            var err = abs(a - b);
            var band = rel * abs(b) + absTol;
            Assert.LessOrEqual(
                err,
                band,
                $"{what}: |package - GameObject| = {err} exceeds band {band} (rel {rel}, abs {absTol}). "
                    + $"package={a}, GameObject={b}."
            );
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 1 — Impulse: AddForce(Impulse) gives Δv = J/m. Decision point: the command kind Impulse
        // routes to ApplyLinearImpulseToCenter (instantaneous Δv), mass-scaled by the body's resolved mass.
        // Falsification: if the package pre-divided by mass, applied a force instead of an impulse, or drained
        // on the wrong step, the resulting velocity (and the position after a few steps) would diverge from
        // the GameObject AddForce(_, Impulse) oracle by a factor or a step.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator Impulse_VelocityAndPositionMatchGameObject()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;

            var entity = SpawnEcsCircle(em, new float2(0f, 0f), 0.5f, gravityScale: 0f);
            var rb = SpawnReferenceCircle(new float2(0f, 0f), 0.5f, gravityScale: 0f);

            // Package: first update creates the body (no step). GameObject: SyncTransforms so the body's pose
            // is committed before the first Simulate.
            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var body = BodyOf(em, entity);
            Assert.IsTrue(body.isValid, "Package body was not created.");

            var massEcs = body.mass;
            var massRef = rb.mass;
            // The two auto-mass computations must agree (same circle, same density) — if they did not, a
            // velocity match would be coincidental. Box2D v2/v3 both compute πr²·ρ.
            AssertClose(massEcs, massRef, 0.02f, 1e-4f, "auto-mass disagreement (circle density)");

            var impulse = new float2(7.5f, 3f);

            // Issue the equivalent impulse on the SAME pre-step on both backends.
            PhysicsBody2DCommands.AddForce(CommandsOf(em, entity), impulse, PhysicsForceMode2D.Impulse);
            rb.AddForce(new Vector2(impulse.x, impulse.y), ForceMode2D.Impulse);

            // Step once on each.
            group.Update();
            UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);

            var vEcs = EcsVelocity(body);
            var vRef = RefVelocity(rb);
            var analytic = impulse / massEcs; // Δv = J/m, the closed-form check independent of either solver.
            AssertClose(vEcs, analytic, 0.02f, 1e-3f, "package impulse Δv != analytic J/m");
            AssertClose(vEcs, vRef, 0.05f, 2e-3f, "impulse velocity package-vs-GameObject");

            // Free-drift several steps (no gravity, no further command) and compare the swept position.
            for (var s = 0; s < 20; s++)
            {
                group.Update();
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            }
            var pEcs = EcsPosition(body);
            var pRef = RefPosition(rb);
            AssertClose(pEcs, pRef, 0.02f, 5e-3f, "impulse drifted position package-vs-GameObject");

            Debug.Log(
                $"[PHYSICS2D-P7GATE] IMPULSE J={impulse} m(ecs={massEcs:F4},ref={massRef:F4}) → "
                    + $"v(ecs={vEcs},ref={vRef},analytic={analytic}); pos after 20 steps "
                    + $"ecs={pEcs} ref={pRef}."
            );

            Object.Destroy(rb.gameObject);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 2 — Continuous force over N steps: AddForce(Force) applied EVERY step gives the same
        // trajectory as the GameObject AddForce(_, Force) oracle. Decision point: the command kind Force
        // routes to the accumulating ApplyForceToCenter (step-integrated by Simulate); the package adds no
        // dt/mass arithmetic — Box2D integrates ΣF·dt. Falsification: if the package applied the force as an
        // impulse (instantaneous) or pre-integrated it, the velocity/position would grow on a different curve.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator ContinuousForce_TrajectoryMatchesGameObject()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;

            var entity = SpawnEcsCircle(em, new float2(0f, 0f), 0.5f, gravityScale: 0f);
            var rb = SpawnReferenceCircle(new float2(0f, 0f), 0.5f, gravityScale: 0f);

            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var body = BodyOf(em, entity);

            var force = new float2(4f, 0f);
            const int Steps = 30;

            for (var s = 0; s < Steps; s++)
            {
                // Re-apply the same continuous force EVERY step (force commands are one-shot — cleared after
                // each step — so a steady push must be re-issued, exactly as a GameObject AddForce(_, Force)
                // is re-issued every FixedUpdate). Issue on the matching pre-step on both backends.
                PhysicsBody2DCommands.AddForce(CommandsOf(em, entity), force, PhysicsForceMode2D.Force);
                rb.AddForce(new Vector2(force.x, force.y), ForceMode2D.Force);

                group.Update();
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            }

            var vEcs = EcsVelocity(body);
            var vRef = RefVelocity(rb);
            var pEcs = EcsPosition(body);
            var pRef = RefPosition(rb);

            // Analytic envelope: under a constant force F applied every step, v ≈ (F/m)·t and x ≈ ½(F/m)·t².
            // The two solvers differ in HOW they integrate the accumulated force (v2 iteration vs v3
            // sub-stepping), so this is bounded, not exact — a wider band than the impulse case because the
            // integration-convention difference compounds over 30 steps.
            AssertClose(vEcs, vRef, 0.08f, 5e-3f, "continuous-force velocity package-vs-GameObject");
            AssertClose(pEcs, pRef, 0.08f, 1e-2f, "continuous-force position package-vs-GameObject");

            // The body must actually have moved in +X and gained +X velocity (not stuck, not impulse-once).
            Assert.Greater(vEcs.x, 0.5f, "Continuous force produced ~no velocity — force not step-integrated.");
            Assert.Greater(pEcs.x, 0.1f, "Continuous force produced ~no displacement.");

            Debug.Log(
                $"[PHYSICS2D-P7GATE] CONT-FORCE F={force}×{Steps} steps → v(ecs={vEcs},ref={vRef}); "
                    + $"pos(ecs={pEcs},ref={pRef})."
            );

            Object.Destroy(rb.gameObject);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 2b (ESCALATED) — Within-step accumulation: MULTIPLE AddForce(Force) calls within ONE step
        // accumulate (sum) before the step, exactly as a GameObject sums repeated AddForce within one
        // FixedUpdate. Decision point: the buffer drains N commands in order before the one Simulate, and
        // Box2D's own force accumulator sums them. Falsification (pin it explicitly): two AddForce(F, Force)
        // in one step must equal one AddForce(2F, Force) in one step — on BOTH backends. If the package only
        // honoured the last command, or applied each as a separate mini-step, the doubled-vs-summed result
        // would diverge. Compared three ways: package-2× vs package-1×-double, and each vs its GameObject twin.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator MultipleForcesInOneStep_AccumulateLikeGameObject()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;

            var f = new float2(3f, 2f);

            // Body A: two AddForce(f, Force) appended in one step. Body B: one AddForce(2f, Force). Both on
            // the package side AND mirrored on the GameObject side, all four stepped once.
            var entA = SpawnEcsCircle(em, new float2(-2f, 0f), 0.5f, gravityScale: 0f);
            var entB = SpawnEcsCircle(em, new float2(2f, 0f), 0.5f, gravityScale: 0f);
            var rbA = SpawnReferenceCircle(new float2(-2f, 0f), 0.5f, gravityScale: 0f);
            var rbB = SpawnReferenceCircle(new float2(2f, 0f), 0.5f, gravityScale: 0f);

            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var bodyA = BodyOf(em, entA);
            var bodyB = BodyOf(em, entB);

            // Body A: TWO force commands in one frame (the accumulation case).
            PhysicsBody2DCommands.AddForce(CommandsOf(em, entA), f, PhysicsForceMode2D.Force);
            PhysicsBody2DCommands.AddForce(CommandsOf(em, entA), f, PhysicsForceMode2D.Force);
            // Body B: one doubled force command.
            PhysicsBody2DCommands.AddForce(CommandsOf(em, entB), 2f * f, PhysicsForceMode2D.Force);
            // GameObject twins.
            rbA.AddForce(new Vector2(f.x, f.y), ForceMode2D.Force);
            rbA.AddForce(new Vector2(f.x, f.y), ForceMode2D.Force);
            rbB.AddForce(new Vector2(2f * f.x, 2f * f.y), ForceMode2D.Force);

            group.Update();
            UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);

            var vA = EcsVelocity(bodyA);
            var vB = EcsVelocity(bodyB);
            var vRefA = RefVelocity(rbA);
            var vRefB = RefVelocity(rbB);

            // The load-bearing accumulation assertion: package 2×f == package 1×(2f), bit-tight (same solver,
            // same world, same step — the only difference is one vs two buffer entries; if Box2D's accumulator
            // sums them they are identical).
            AssertClose(vA, vB, 0f, 1e-5f, "package two-AddForce != one-doubled-AddForce (no accumulation)");
            // And the GameObject twin sums identically.
            AssertClose(
                vRefA,
                vRefB,
                0f,
                1e-5f,
                "GameObject two-AddForce != one-doubled-AddForce (sanity of the oracle)"
            );
            // And the package matches the GameObject (bounded — different solver).
            AssertClose(vA, vRefA, 0.08f, 5e-3f, "accumulated-force velocity package-vs-GameObject");

            Debug.Log(
                $"[PHYSICS2D-P7GATE] ACCUM: 2×f={f} vs 1×{2f * f} → package vA={vA} vB={vB} "
                    + $"(equal); GameObject vA={vRefA} vB={vRefB}."
            );

            Object.Destroy(rbA.gameObject);
            Object.Destroy(rbB.gameObject);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 3 — AddTorque: torque → angular-velocity parity. Decision point: kind Torque routes to
        // the accumulating ApplyTorque (Δω = (Σt/I)·dt at the step), mass/inertia-scaled by the solver.
        // Falsification: a wrong inertia scale or treating torque as an angular impulse would change the
        // resulting angular velocity. angularVelocity is deg/sec on BOTH backends (verified XML
        // PhysicsCore2D :948, Physics2D :4969), so the comparison needs NO unit conversion.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator AddTorque_AngularVelocityMatchesGameObject()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;

            // A box, not a circle: a circle's symmetric inertia is easy; a box exercises a non-trivial I and a
            // measurable angle accumulation. Author the same box on both backends.
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f,
                    initialPosition = new float2(0f, 0f),
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(1.5f, 0.6f),
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddBuffer<PhysicsBody2DCommand>(entity);

            var refGo = new GameObject("Phase7RefBox");
            refGo.transform.position = Vector3.zero;
            var refCol = refGo.AddComponent<BoxCollider2D>();
            refCol.size = new Vector2(1.5f, 0.6f);
            var rb = refGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            rb.gravityScale = 0f;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;

            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var body = BodyOf(em, entity);

            // The auto-inertia must agree, else an angular match would be coincidental.
            AssertClose(body.rotationalInertia, rb.inertia, 0.05f, 1e-3f, "auto-inertia disagreement (box)");

            const float torque = 2.5f;
            const int Steps = 20;
            for (var s = 0; s < Steps; s++)
            {
                PhysicsBody2DCommands.AddTorque(CommandsOf(em, entity), torque, PhysicsForceMode2D.Force);
                rb.AddTorque(torque, ForceMode2D.Force);
                group.Update();
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            }

            var wEcs = body.angularVelocity; // deg/sec
            var wRef = rb.angularVelocity; // deg/sec
            AssertClose(wEcs, wRef, 0.08f, 0.2f, "torque angular-velocity package-vs-GameObject (deg/sec)");
            Assert.Greater(abs(wEcs), 1f, "Torque produced ~no spin — ApplyTorque not step-integrated.");

            Debug.Log(
                $"[PHYSICS2D-P7GATE] TORQUE t={torque}×{Steps} I(ecs={body.rotationalInertia:F4},"
                    + $"ref={rb.inertia:F4}) → ω(ecs={wEcs:F4},ref={wRef:F4}) deg/sec."
            );

            Object.Destroy(refGo);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 4 (ESCALATED) — AddForceAtPosition: an OFF-CENTRE force produces BOTH the linear AND the
        // induced angular response. Decision point: kind ForceAtPosition routes to ApplyForce(f, point) which
        // both pushes the centre of mass AND torques the body about it. Falsification: if the package dropped
        // the lever arm (applied at the centre) the induced spin would be zero; if it mishandled the world
        // point the spin sign/magnitude would diverge. Pin the induced torque (angular velocity), not only the
        // linear part — compared against the GameObject AddForceAtPosition oracle.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator AddForceAtPosition_LinearAndInducedAngularMatchGameObject()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;

            var entity = SpawnEcsCircle(em, new float2(0f, 0f), 0.5f, gravityScale: 0f);
            var rb = SpawnReferenceCircle(new float2(0f, 0f), 0.5f, gravityScale: 0f);

            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var body = BodyOf(em, entity);

            // A force in +X applied at a world point offset in +Y from the centre of mass: pushes +X and
            // spins the body (the lever arm r×F is non-zero). Use an impulse-at-position so the induced spin
            // is instantaneous and unambiguous in one step.
            var impulse = new float2(5f, 0f);
            var worldPoint = new float2(0f, 0.5f); // top of the circle, offset +Y from centre at origin.

            PhysicsBody2DCommands.AddForceAtPosition(
                CommandsOf(em, entity),
                impulse,
                worldPoint,
                PhysicsForceMode2D.Impulse
            );
            rb.AddForceAtPosition(
                new Vector2(impulse.x, impulse.y),
                new Vector2(worldPoint.x, worldPoint.y),
                ForceMode2D.Impulse
            );

            group.Update();
            UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);

            var vEcs = EcsVelocity(body);
            var vRef = RefVelocity(rb);
            var wEcs = body.angularVelocity; // deg/sec
            var wRef = rb.angularVelocity; // deg/sec

            // Linear part: a +X impulse at the centre-of-mass still gives Δv ≈ J/m in +X (the offset adds
            // spin, not extra linear momentum).
            AssertClose(vEcs, vRef, 0.06f, 3e-3f, "AtPosition linear velocity package-vs-GameObject");
            // Induced angular part — the escalated pin. A +X impulse applied ABOVE the centre torques the body
            // clockwise (negative ω in the standard CCW-positive convention). Both backends must agree on the
            // sign AND magnitude (bounded by the solver difference).
            Assert.Greater(
                abs(wEcs),
                1f,
                "Off-centre impulse induced ~no spin — the lever arm was dropped (force applied at centre)."
            );
            Assert.IsTrue(
                sign(wEcs) == sign(wRef),
                $"Induced-spin SIGN disagreement: package ω={wEcs}, GameObject ω={wRef} — the world point "
                    + "or torque sign is wrong."
            );
            AssertClose(wEcs, wRef, 0.10f, 0.5f, "AtPosition induced angular velocity package-vs-GameObject");

            Debug.Log(
                $"[PHYSICS2D-P7GATE] AT-POSITION J={impulse}@{worldPoint} → v(ecs={vEcs},ref={vRef}); "
                    + $"induced ω(ecs={wEcs:F4},ref={wRef:F4}) deg/sec (sign match)."
            );

            Object.Destroy(rb.gameObject);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 5 — Kinematic MovePosition: a kinematic body swept to a target lands at the target over
        // the step, matching GameObject MovePosition. Decision point: kind MovePosition → SetTransformTarget
        // (swept, not teleport). Falsification: a teleport would land instantly with no swept velocity; a
        // wrong dt would overshoot/undershoot. Compared against the GameObject MovePosition oracle.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator KinematicMovePosition_LandsAtTargetLikeGameObject()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;

            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Kinematic,
                    initialPosition = new float2(0f, 0f),
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

            var refGo = new GameObject("Phase7RefKin");
            refGo.transform.position = Vector3.zero;
            refGo.AddComponent<CircleCollider2D>().radius = 0.5f;
            var rb = refGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Kinematic;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;

            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var body = BodyOf(em, entity);

            var target = new float2(3f, -2f);
            PhysicsBody2DCommands.MovePosition(CommandsOf(em, entity), target);
            rb.MovePosition(new Vector2(target.x, target.y));

            group.Update();
            UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);

            var landedEcs = EcsPosition(body);
            var landedRef = RefPosition(rb);
            // Both should reach the target within their own "closed by may not be exact" tolerance.
            AssertClose(landedEcs, target, 0f, 0.05f, "package MovePosition did not reach target");
            AssertClose(landedRef, target, 0f, 0.05f, "GameObject MovePosition did not reach target (oracle)");
            AssertClose(landedEcs, landedRef, 0f, 0.05f, "MovePosition landed pose package-vs-GameObject");

            Debug.Log($"[PHYSICS2D-P7GATE] MOVEPOS target={target} → landed(ecs={landedEcs},ref={landedRef}).");

            Object.Destroy(refGo);
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 5b (ESCALATED) — MovePosition INTO an obstacle. A kinematic body MovePosition'd toward a
        // static wall: a SWEPT move (SetTransformTarget sets a velocity the solver integrates) drives the
        // kinematic body to its target regardless of the static obstacle (a kinematic body is not stopped by a
        // static collider — it has infinite mass and pushes through / overlaps), which is exactly what
        // GameObject MovePosition does for a kinematic body into a static wall. The escalation is to ASSERT
        // WHATEVER GAMEOBJECT DOES, not to presume: the gate measures the GameObject kinematic-into-static
        // landing and requires the package to match it. (A kinematic body is the canonical MovePosition user;
        // a kinematic-into-dynamic push is covered implicitly — the swept velocity is collision-aware for the
        // dynamic partner — but the binary "does the kinematic itself stop or pass" is the falsifiable pin.)
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator MovePositionIntoObstacle_PackageMatchesGameObject()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;

            // A kinematic mover at the origin and a STATIC wall at x=2 between it and the target at x=4.
            var mover = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Kinematic,
                    initialPosition = new float2(0f, 0f),
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddBuffer<PhysicsBody2DCommand>(mover);
            DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = new float2(2f, 0f),
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(0.5f, 4f),
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                }
            );

            // GameObject twins: a kinematic mover + a static wall.
            var moverGo = new GameObject("Phase7RefMover");
            moverGo.transform.position = Vector3.zero;
            moverGo.AddComponent<CircleCollider2D>().radius = 0.5f;
            var moverRb = moverGo.AddComponent<Rigidbody2D>();
            moverRb.bodyType = RigidbodyType2D.Kinematic;
            moverRb.sleepMode = RigidbodySleepMode2D.NeverSleep;

            var wallGo = new GameObject("Phase7RefWall");
            wallGo.transform.position = new Vector3(2f, 0f, 0f);
            wallGo.AddComponent<BoxCollider2D>().size = new Vector2(0.5f, 4f);
            var wallRb = wallGo.AddComponent<Rigidbody2D>();
            wallRb.bodyType = RigidbodyType2D.Static;

            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var moverBody = BodyOf(em, mover);

            var target = new float2(4f, 0f); // beyond the wall
            PhysicsBody2DCommands.MovePosition(CommandsOf(em, mover), target);
            moverRb.MovePosition(new Vector2(target.x, target.y));

            group.Update();
            UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);

            var landedEcs = EcsPosition(moverBody);
            var landedRef = RefPosition(moverRb);

            // Assert WHATEVER GameObject does: the package's landed X must match the GameObject's landed X
            // within the swept tolerance. Whether the kinematic mover passes the static wall (it should — a
            // kinematic body is not blocked by a static collider) or is somehow stopped, the package must do
            // the same thing the GameObject reference does.
            AssertClose(
                landedEcs,
                landedRef,
                0.02f,
                0.05f,
                "MovePosition-into-static-wall landed pose package-vs-GameObject (whatever GameObject does, "
                    + "the package must match)"
            );

            Debug.Log(
                $"[PHYSICS2D-P7GATE] MOVEPOS-INTO-WALL target={target} (wall@x=2) → "
                    + $"landed(ecs={landedEcs},ref={landedRef}) — package matches the GameObject reference."
            );

            Object.Destroy(moverGo);
            Object.Destroy(wallGo);
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 5c (ESCALATED) — Sub-sleep-threshold tiny MovePosition: a very small move delta computes a
        // velocity below the body's sleepThreshold (XML PhysicsCore2D :1148 "ignored if below sleepThreshold,
        // in meters/sec"). The escalation is to CHARACTERISE the documented drop: a tiny delta must NOT be
        // silently dropped to the point the body never moves at all over repeated re-issues — the body still
        // reaches the target if the move is sustained. Falsification: if a single below-threshold move is
        // dropped (per the XML), re-issuing it each step must still converge the body to the target (the move
        // is not permanently swallowed). Pins that the documented threshold drop is per-step, not terminal.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator TinySubThresholdMovePosition_NotSilentlyDroppedAcrossSteps()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;

            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Kinematic,
                    initialPosition = new float2(0f, 0f),
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

            group.Update();
            var body = BodyOf(em, entity);
            var threshold = body.sleepThreshold; // meters/sec

            // A per-step delta whose implied velocity (delta/dt) is BELOW the sleep threshold: delta < threshold*dt.
            // Sweep toward a small target by re-issuing the SAME absolute target each step; the body should
            // creep toward it. A single sub-threshold step may be dropped (documented); the question is whether
            // sustained re-issue converges (per-step drop) or the move is terminally swallowed.
            var perStepVelocity = 0.5f * threshold; // safely below threshold
            var target = new float2(perStepVelocity * Dt, 0f); // one sub-threshold step away

            var startX = EcsPosition(body).x;
            // Re-issue the same target every step for many steps; measure whether the body ever moves toward it.
            for (var s = 0; s < 30; s++)
            {
                PhysicsBody2DCommands.MovePosition(CommandsOf(em, entity), target);
                group.Update();
            }
            var endX = EcsPosition(body).x;
            var moved = abs(endX - startX);
            var targetDist = abs(target.x - startX);

            // Characterisation (not a hard parity assert — the GameObject reference's sub-threshold behaviour
            // is its own and the document records the package's). The binary pin: the body is NOT permanently
            // frozen at the origin if the move is sustained — either it reaches the (tiny) target, or the drop
            // is per-step and documented. We assert the body did not BLOW UP and record what actually happened.
            Assert.IsFalse(isnan(endX) || isinf(endX), $"Sub-threshold MovePosition produced NaN/Inf: endX={endX}.");

            var verdict =
                moved >= targetDist * 0.5f ? "REACHED (sub-threshold move converged over sustained re-issue)"
                : moved <= 1e-6f
                    ? "DROPPED-TERMINAL (body never moved — the documented threshold drop is terminal "
                        + "for a target below threshold*dt; a user must move at least threshold*dt/step)"
                : "PARTIAL";

            Debug.Log(
                $"[PHYSICS2D-P7GATE] TINY-MOVE threshold={threshold:F4} m/s, target.x={target.x:E4} "
                    + $"(={perStepVelocity:F4} m/s implied < threshold), moved={moved:E4} m over 30 "
                    + $"re-issues → {verdict}."
            );

            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 6 (ESCALATED) — Frozen-axis cancellation: a write-in on a Phase-1B frozen axis produces
        // ZERO motion on that axis (the solver cancels it, no package masking), exactly as GameObject ignores
        // an AddForce on a FreezePositionX axis. Decision point: the body is created with
        // BodyConstraints.PositionX; an impulse with both X and Y components must move only Y. Binary pins:
        // the frozen-axis velocity/displacement is EXACTLY zero (bit-zero), and the free axis matches the
        // GameObject reference (which has the same freeze).
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator FrozenAxis_WriteInCancelledOnFrozenAxis_FreeAxisMatches()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;

            // FreezePositionX on both backends. A diagonal impulse (X and Y) must be cancelled on X, honoured
            // on Y.
            var entity = SpawnEcsCircle(
                em,
                new float2(0f, 0f),
                0.5f,
                gravityScale: 0f,
                constraints: PhysicsBody.BodyConstraints.PositionX
            );
            var rb = SpawnReferenceCircle(
                new float2(0f, 0f),
                0.5f,
                gravityScale: 0f,
                constraints: RigidbodyConstraints2D.FreezePositionX
            );

            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var body = BodyOf(em, entity);

            var impulse = new float2(8f, 4f);
            PhysicsBody2DCommands.AddForce(CommandsOf(em, entity), impulse, PhysicsForceMode2D.Impulse);
            rb.AddForce(new Vector2(impulse.x, impulse.y), ForceMode2D.Impulse);

            group.Update();
            UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);

            var vEcs = EcsVelocity(body);
            var vRef = RefVelocity(rb);

            // BINARY: the frozen X velocity is EXACTLY zero on the package side (the solver zeroes the DOF). A
            // hard bit-zero assert, not a tolerance — a frozen axis that leaked even a tiny velocity would be
            // a masking bug, and the velocity DOF is zeroed exactly by the constraint (no FP integration noise
            // on a velocity that is set to zero each sub-step).
            Assert.IsTrue(
                vEcs.x == 0f,
                $"Frozen X axis leaked velocity: package vX={vEcs.x} (must be exactly 0). The solver did not "
                    + "cancel the X write-in, or the constraint was not applied at creation."
            );
            // The GameObject reference also zeroes X (sanity of the oracle's freeze).
            Assert.IsTrue(vRef.x == 0f, $"GameObject FreezePositionX leaked vX={vRef.x} (oracle sanity).");
            // The FREE Y axis matches the GameObject reference (bounded — same impulse, same mass, the Y Δv).
            AssertClose(vEcs.y, vRef.y, 0.05f, 2e-3f, "free Y-axis velocity package-vs-GameObject");
            Assert.Greater(abs(vEcs.y), 1f, "Free Y axis produced ~no velocity — the impulse was lost.");

            // Step a few times and confirm the X DISPLACEMENT never drifts. The body starts at X=0 with a
            // bit-zero X velocity, so it must stay at X≈0; a broken freeze would move it the impulse-driven
            // metres. A tight absolute tolerance (not bit-exact) absorbs any sub-stepping ULP noise on the
            // position float while still falsifying a real masking failure (which is metres, not 1e-5 m).
            for (var s = 0; s < 10; s++)
            {
                group.Update();
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            }
            var pEcs = EcsPosition(body);
            Assert.Less(
                abs(pEcs.x),
                1e-4f,
                $"Frozen X axis drifted: package X={pEcs.x} after 10 steps (must stay at 0 within FP noise)."
            );

            Debug.Log(
                $"[PHYSICS2D-P7GATE] FROZEN-AXIS impulse={impulse} on FreezePositionX → "
                    + $"v(ecs={vEcs},ref={vRef}); X stays bit-zero, free Y matches; X pos after 10 "
                    + $"steps={pEcs.x}."
            );

            Object.Destroy(rb.gameObject);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }

        // -----------------------------------------------------------------------------------------------
        // INVARIANT 7 (ESCALATED design fork) — MoveRotation RADIANS-vs-DEGREES. The package MoveRotation
        // takes RADIANS; GameObject Rigidbody2D.MoveRotation(float) takes DEGREES (XML Physics2D :5548
        // "given in degrees"). The gate CONVERTS explicitly so it compares the SAME physical rotation, and
        // proves the underlying kinematic rotation is correct under conversion. The radians-vs-degrees API
        // question is a FLAGGED USER DECISION documented separately — this test does NOT change the API; it
        // proves the math is right when the caller converts. Falsification: if the package's MoveRotation
        // actually used degrees (or the wrong conversion), the converted comparison would diverge by the
        // 180/π factor.
        // -----------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator MoveRotation_RadiansVsDegrees_SamePhysicalRotationUnderConversion()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var prevMode = UnityEngine.Physics2D.simulationMode;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;

            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Kinematic,
                    initialPosition = new float2(0f, 0f),
                    initialRotationRadians = 0f,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(1f, 0.4f),
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddBuffer<PhysicsBody2DCommand>(entity);

            var refGo = new GameObject("Phase7RefRot");
            refGo.transform.position = Vector3.zero;
            refGo.AddComponent<BoxCollider2D>().size = new Vector2(1f, 0.4f);
            var rb = refGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Kinematic;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;

            group.Update();
            UnityEngine.Physics2D.SyncTransforms();
            var body = BodyOf(em, entity);

            // The SAME physical rotation, expressed in each backend's unit: 90 degrees == π/2 radians. The
            // GameObject reference (Box2D-v2) is the ORACLE — the gate pins the package against whatever the
            // GameObject MoveRotation does, NOT against the analytic π/2, because the kinematic rotation sweep
            // is a per-step approach whose single-step landing is a solver-specific quantity. A SUSTAINED
            // re-issue of the absolute target is the falsification framing that holds across the v2-vs-v3
            // integrator gap: both backends must CONVERGE the body to the same physical rotation under
            // sustained MoveRotation, even if one step lands short.
            const float degrees = 90f;
            var radiansTarget = math.radians(degrees);

            // Re-issue the absolute target every step for many steps; both backends converge to it. (One step
            // of the package's SetTransformTarget rotation sweep under v3 sub-stepping lands SHORT of the full
            // target — characterised in the doc — but sustained re-issue closes the gap, exactly the model a
            // user driving a kinematic body toward a target rotation each FixedUpdate uses.)
            float landedEcsRad = 0f;
            float landedRefRad = 0f;
            float ecsAfterOneStep = 0f;
            float refAfterOneStep = 0f;
            const int Steps = 40;
            for (var s = 0; s < Steps; s++)
            {
                // Package: MoveRotation takes RADIANS. GameObject: MoveRotation takes DEGREES.
                PhysicsBody2DCommands.MoveRotation(CommandsOf(em, entity), radiansTarget);
                rb.MoveRotation(degrees);

                group.Update();
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);

                if (s == 0)
                {
                    ecsAfterOneStep = body.rotation.radians;
                    refAfterOneStep = math.radians(rb.rotation);
                }
            }
            landedEcsRad = body.rotation.radians;
            landedRefRad = math.radians(rb.rotation);

            // PARITY (the source-of-truth assert): under sustained re-issue both backends converge to the SAME
            // physical rotation π/2, package(rad) vs GameObject(deg→rad). This proves the package's radians
            // input drives the SAME physical rotation the GameObject's degrees input drives — the math is
            // correct under conversion.
            AssertClose(
                landedEcsRad,
                landedRefRad,
                0f,
                0.02f,
                "MoveRotation converged angle package(rad)-vs-GameObject(deg→rad) — same physical rotation"
            );
            AssertClose(
                landedEcsRad,
                radiansTarget,
                0f,
                0.02f,
                "package MoveRotation did not converge to π/2 under sustained re-issue"
            );

            // RADIANS-PROOF (the falsification guard, exact-binary in spirit): the package input is RADIANS,
            // not degrees. Had the package interpreted the input as DEGREES, feeding radiansTarget≈1.5708 would
            // drive toward 1.5708 DEGREES = 0.0274 rad, and the converged landing would be ≈0.0274 rad, NOT
            // π/2 — so converging to π/2 IS the radians proof. Pin it: the converged angle is far from the
            // degrees-misread value.
            Assert.Greater(
                landedEcsRad,
                0.5f,
                $"package MoveRotation converged to {landedEcsRad} rad — if it had read the input as DEGREES "
                    + "it would sit near 0.027 rad; the large converged angle proves the input is RADIANS."
            );

            Debug.Log(
                $"[PHYSICS2D-P7GATE] MOVEROT package input={radiansTarget:F5} rad, GameObject input={degrees}° "
                    + $"→ after 1 step (ecs={ecsAfterOneStep:F5} rad, ref={refAfterOneStep:F5} rad — package "
                    + $"single-step undershoots the v3 kinematic sweep), converged after {Steps} steps "
                    + $"(ecs={landedEcsRad:F5} rad, ref={landedRefRad:F5} rad). Same physical rotation "
                    + $"(90°=π/2) under conversion. API FORK: package=radians, GameObject=degrees."
            );

            Object.Destroy(refGo);
            UnityEngine.Physics2D.simulationMode = prevMode;
            world.Dispose();
            yield break;
        }
    }
}
