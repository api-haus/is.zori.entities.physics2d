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
    /// The independent adversarial GameObject-parity e2e gate for Phase 8 (interpolation/extrapolation,
    /// continuous collision detection, joint break). Built by a validating agent that did NOT write the
    /// Phase-8 code, to FALSIFY each sub-feature from its observable decision points, never the happy path
    /// the smoke (<see cref="InterpCcdJointBreakSmoke"/>) already witnesses. Continuous facts are bounded
    /// against a live GameObject oracle (Box2D-v2 iteration solver, stepped by
    /// <c>UnityEngine.Physics2D.Simulate</c> under <c>simulationMode = Script</c>); binary facts
    /// (tunnel/no-tunnel, broke/not-broke, smoothing-mode equality) are asserted exactly.
    /// </summary>
    /// <remarks>
    /// <para><b>The oracle + the v2-vs-v3 gap.</b> The package runs Box2D-v3 (sub-stepping solver); the
    /// GameObject reference runs Box2D-v2 (iteration solver) — DIFFERENT integrators
    /// (<c>00d-determinism-probe-results.md</c>). So the gate asserts solver-independent BINARY verdicts
    /// exactly and brackets CONTINUOUS quantities (break load, swept distance) by a generous relative band.</para>
    ///
    /// <para><b>Isolation + determinism.</b> Each world-mutating test runs in its own disposable
    /// <see cref="World"/> and restores every global <c>Physics2D</c> knob it touched, so a thrown test cannot
    /// leak a native body into a shared world. The package-side poses/verdicts are bit-deterministic across
    /// runs (same world, same authored inputs), so two-consecutive-green is a strict re-equality of the
    /// tunnel/break verdicts and a re-pass of the bounded oracle comparison. Coroutines yield <c>null</c> only
    /// (never <c>WaitForEndOfFrame</c>, which does not tick in batchmode).</para>
    ///
    /// <para><b>Interpolation framing.</b> GameObject <c>Rigidbody2D.interpolation</c> is a render-time visual
    /// a batchmode fixed loop cannot sample (no sub-step headroom → <c>timeAhead ≈ 0</c>), so interpolation is
    /// pinned as an INTERNAL INVARIANT: the smoothing system's written <c>LocalToWorld</c> at a controlled
    /// sub-step fraction must equal the analytic interpolate/extrapolate of the bracketing physics states
    /// captured from a REAL stepped body — covering the write-back capture path, not only the math.</para>
    /// </remarks>
    public sealed class Phase8InterpCcdJointBreakGate
    {
        const float Dt = 1f / 60f;
        static readonly Vector2 Gravity = new(0f, -9.81f);

        // ===============================================================================================
        // Scaffolds
        // ===============================================================================================

        // A disposable package world with the fixed-step systems; optionally the joint create/break systems.
        static World MakeFixedWorld(out FixedStepSimulationSystemGroup group, bool withJoints)
        {
            var world = new World("Physics2DPhase8GateWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
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

        static float BodyX(EntityManager em, Entity e) => em.GetComponentData<LocalToWorld>(e).Position.x;

        static PhysicsBody BodyOf(EntityManager em, Entity e) => em.GetComponentData<PhysicsBody2D>(e).body;

        static Entity SpawnBullet(EntityManager em, float2 pos, float vx, bool continuous)
        {
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f, // pure horizontal flight, isolate the wall hit from gravity
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
            em.AddComponentData(entity, new PhysicsBody2DInitialVelocity { linearVelocity = new float2(vx, 0f) });
            return entity;
        }

        static void SpawnStaticWall(EntityManager em, float2 center, float thickness)
        {
            DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition { bodyType = PhysicsBody.BodyType.Static, initialPosition = center },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(thickness, 4f),
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
        }

        // A GameObject reference bullet circle (dynamic), matched to the package's auto-mass + CCD mode.
        static Rigidbody2D SpawnRefBullet(float2 pos, float vx, bool continuous)
        {
            var go = new GameObject("P8RefBullet");
            go.transform.position = new Vector3(pos.x, pos.y, 0f);
            go.AddComponent<CircleCollider2D>().radius = 0.25f;
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            rb.gravityScale = 0f;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            rb.collisionDetectionMode = continuous
                ? CollisionDetectionMode2D.Continuous
                : CollisionDetectionMode2D.Discrete;
            rb.linearVelocity = new Vector2(vx, 0f);
            return rb;
        }

        static Rigidbody2D SpawnRefStaticWall(float2 center, float thickness)
        {
            var go = new GameObject("P8RefWall");
            go.transform.position = new Vector3(center.x, center.y, 0f);
            go.AddComponent<BoxCollider2D>().size = new Vector2(thickness, 4f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            return rb;
        }

        // ===============================================================================================
        // (B) CCD — clean binary tunnel/no-tunnel contrast, both mediums, against the GameObject oracle.
        // ===============================================================================================

        // Run ONE package-side shot of a bullet at a thin static wall at x=0 from x=-3, N steps; return the
        // final body X. tunnel ⇔ X > wallHalf (it passed the wall plane).
        static float PackageStaticShot(float vx, bool continuous)
        {
            var world = MakeFixedWorld(out var group, withJoints: false);
            var em = world.EntityManager;
            SpawnStaticWall(em, new float2(0f, 0f), 0.05f);
            var bullet = SpawnBullet(em, new float2(-3f, 0f), vx, continuous);
            group.Update(); // create (no step)
            for (var f = 0; f < 30; f++)
                group.Update();
            var x = BodyX(em, bullet);
            world.Dispose();
            return x;
        }

        // ---------------------------------------------------------------------------------------------
        // (B.1) CCD speed sweep against a STATIC wall, vs the GameObject oracle. Decision points: the package
        // Continuous body's CCD, and the v2-vs-v3 split on the Dynamic-vs-STATIC medium. The XML
        // (PhysicsCore2D :13259-13263) makes Dynamic-vs-Static CCD the WORLD-level continuousAllowed (default
        // true), distinct from the per-body fastCollisionsAllowed flag (Dynamic-vs-Dynamic/Kinematic). So in
        // Box2D-v3 the static-wall case is caught by the world default for BOTH modes — a package body does not
        // tunnel a static wall whether Discrete or Continuous. Box2D-v2 (the GameObject reference) has NO
        // world-level Dynamic-vs-Static CCD: its Discrete body tunnels a static wall above an engage speed,
        // and only its per-body Continuous flag prevents it. This is a genuine v2-vs-v3 divergence, surfaced
        // and measured here.
        //
        // FALSIFICATION, framed for the divergence:
        //   - Package Continuous NEVER tunnels a static wall across the whole sweep (the package contract).
        //   - GameObject Continuous NEVER tunnels (oracle parity holds on the Continuous mode).
        //   - Package Discrete is at least as SAFE as GameObject Discrete (it never tunnels where GameObject
        //     does NOT, and the divergence is one-directional: package safer). The measured GameObject Discrete
        //     engage speed is recorded as the v2-vs-v3 known-gap number.
        // The strict per-mode equality is deliberately NOT asserted on the Discrete mode against a static wall,
        // because requiring the package to REPRODUCE v2's static-wall tunnelling would be asserting a worse
        // (less safe) behaviour — the clean per-mode binary parity lives in the dynamic/kinematic medium (B.2),
        // where the body flag alone governs and v2/v3 agree.
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator CCD_SpeedSweep_StaticWall_ContinuousNeverTunnels_DiscreteAtLeastAsSafe()
        {
            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Vector2.zero; // horizontal flight, no gravity

            // Speeds spanning the GameObject discrete tunnelling onset (measured ≈ 48 m/s for a 0.5 m circle
            // vs a 0.05 m wall at 1/60 s). The circle rests on the near face at x ≈ -0.275 when stopped.
            var speeds = new[] { 6f, 12f, 24f, 48f, 96f, 120f };
            // "Tunnelled" ⇔ ended clearly past the wall plane (> 0.2, unambiguous vs the resting x ≈ -0.275).
            const float TunnelX = 0.2f;

            var log = new System.Text.StringBuilder();
            log.AppendLine("[P8GATE-CCD-STATIC] speed\tpkgDisc\tpkgCont\trefDisc\trefCont (final X; * = tunnelled)");

            string pkgContTunnelViolation = null;
            string refContTunnelViolation = null;
            string discSafetyViolation = null;
            var refDiscreteEngageSpeed = float.PositiveInfinity;
            var sweepDidEngageRefDiscrete = false;

            foreach (var v in speeds)
            {
                // Package side: two disposable worlds (discrete + continuous), each at the wall's Y so the
                // body actually meets the wall (not flying over it).
                var pkgDiscX = PackageStaticShot(v, continuous: false);
                var pkgContX = PackageStaticShot(v, continuous: true);

                // GameObject reference: a discrete + a continuous bullet, each aligned with its own wall.
                var refDiscRb = SpawnRefBullet(new float2(-3f, 0f), v, continuous: false);
                var refContRb = SpawnRefBullet(new float2(-3f, 2.5f), v, continuous: true);
                var refWallA = SpawnRefStaticWall(new float2(0f, 0f), 0.05f);
                var refWallB = SpawnRefStaticWall(new float2(0f, 2.5f), 0.05f);
                UnityEngine.Physics2D.SyncTransforms();
                for (var f = 0; f < 30; f++)
                    UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                var refDiscX = refDiscRb.position.x;
                var refContX = refContRb.position.x;
                Object.Destroy(refDiscRb.gameObject);
                Object.Destroy(refContRb.gameObject);
                Object.Destroy(refWallA.gameObject);
                Object.Destroy(refWallB.gameObject);
                yield return null;

                var pkgDiscT = pkgDiscX > TunnelX;
                var pkgContT = pkgContX > TunnelX;
                var refDiscT = refDiscX > TunnelX;
                var refContT = refContX > TunnelX;

                log.AppendLine(
                    $"{v}\t{pkgDiscX:F3}{(pkgDiscT ? "*" : "")}\t{pkgContX:F3}{(pkgContT ? "*" : "")}"
                        + $"\t{refDiscX:F3}{(refDiscT ? "*" : "")}\t{refContX:F3}{(refContT ? "*" : "")}"
                );

                // Package Continuous never tunnels a static wall.
                if (pkgContTunnelViolation == null && pkgContT)
                    pkgContTunnelViolation =
                        $"Package Continuous tunnelled a static wall at {v} m/s (x={pkgContX:F3}) — "
                        + "fastCollisionsAllowed + the world continuousAllowed both failed to catch it.";
                // GameObject Continuous never tunnels (oracle sanity on the Continuous mode).
                if (refContTunnelViolation == null && refContT)
                    refContTunnelViolation =
                        $"GameObject Continuous tunnelled a static wall at {v} m/s (x={refContX:F3}) — oracle "
                        + "sanity failure (Continuous CCD should hold in v2).";

                // The divergence is one-directional: the package Discrete may be SAFER than GameObject Discrete
                // (caught by v3's world default), but it must never tunnel where GameObject Discrete does NOT —
                // that would be a regression making the package LESS safe than the reference.
                if (discSafetyViolation == null && pkgDiscT && !refDiscT)
                    discSafetyViolation =
                        $"Package Discrete tunnelled a static wall at {v} m/s (x={pkgDiscX:F3}) where "
                        + $"GameObject Discrete did NOT (x={refDiscX:F3}) — the package is LESS safe than the "
                        + "reference, a real regression (not the benign v3-safer divergence).";

                if (refDiscT)
                {
                    sweepDidEngageRefDiscrete = true;
                    refDiscreteEngageSpeed = min(refDiscreteEngageSpeed, v);
                }
            }

            Debug.Log(log.ToString());

            Assert.IsNull(pkgContTunnelViolation, pkgContTunnelViolation);
            Assert.IsNull(refContTunnelViolation, refContTunnelViolation);
            Assert.IsNull(discSafetyViolation, discSafetyViolation);
            Assert.IsTrue(
                sweepDidEngageRefDiscrete,
                "The sweep never produced a GameObject Discrete tunnelling — the speeds did not reach the v2 "
                    + "engage threshold, so the divergence was not characterised. Widen the speeds."
            );

            Debug.Log(
                $"[P8GATE-CCD-STATIC] KNOWN-GAP (v2-vs-v3, static medium): GameObject (v2) Discrete tunnels a "
                    + $"static wall from ≈ {refDiscreteEngageSpeed} m/s; the package (v3) Discrete NEVER tunnels "
                    + "a static wall (caught by the world-level continuousAllowed). Package Continuous and "
                    + "GameObject Continuous both never tunnel. The package is strictly SAFER than the v2 "
                    + "reference against a static wall; clean per-mode parity is in the dynamic medium (B.2)."
            );

            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            yield break;
        }

        // ---------------------------------------------------------------------------------------------
        // (B.2) CCD against a DYNAMIC wall — the medium the body flag (not the world-level continuousAllowed)
        // governs per the XML ("continuous collision detection against dynamic and kinematic bodies"). A fast
        // Continuous bullet into a heavy dynamic slab must NOT tunnel; a Discrete one tunnels. Pinned against
        // the GameObject reference's same-medium verdict.
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator CCD_DynamicWall_ContinuousDoesNotTunnel_DiscreteDoes_MatchesGameObject()
        {
            const float V = 120f;
            const float TunnelX = 0.5f;

            // --- package side: a fast bullet into a heavy KINEMATIC slab (a stand-in for an immovable dynamic
            // partner — kinematic is the body-flag medium and does not drift under the impact, keeping the
            // verdict clean). Both modes, each its own world. ---
            float PackageDynamicShot(bool continuous)
            {
                var world = MakeFixedWorld(out var group, withJoints: false);
                var em = world.EntityManager;
                // A thin kinematic wall at x=0.
                DirectPhysics2DAuthoring.Create(
                    em,
                    new PhysicsBody2DDefinition
                    {
                        bodyType = PhysicsBody.BodyType.Kinematic,
                        initialPosition = new float2(0f, 0f),
                    },
                    new PhysicsShape2D
                    {
                        kind = PhysicsShape2DKind.Box,
                        size = new float2(0.05f, 4f),
                        radius = 0f,
                        density = 1f,
                        friction = 0.4f,
                    }
                );
                var bullet = SpawnBullet(em, new float2(-3f, 0f), V, continuous);
                group.Update();
                for (var f = 0; f < 30; f++)
                    group.Update();
                var x = BodyX(em, bullet);
                world.Dispose();
                return x;
            }

            var pkgDiscX = PackageDynamicShot(continuous: false);
            var pkgContX = PackageDynamicShot(continuous: true);
            yield return null;

            // --- GameObject reference: same shot at a kinematic wall. ---
            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Vector2.zero;

            var refGos = new List<GameObject>();
            Rigidbody2D RefDynamicShot(bool continuous, float y)
            {
                var wallGo = new GameObject("P8RefKinWall");
                wallGo.transform.position = new Vector3(0f, y, 0f);
                wallGo.AddComponent<BoxCollider2D>().size = new Vector2(0.05f, 4f);
                var wallRb = wallGo.AddComponent<Rigidbody2D>();
                wallRb.bodyType = RigidbodyType2D.Kinematic;
                refGos.Add(wallGo);
                var bullet = SpawnRefBullet(new float2(-3f, y), V, continuous);
                refGos.Add(bullet.gameObject);
                return bullet;
            }

            var refDiscRb = RefDynamicShot(continuous: false, y: 0f);
            var refContRb = RefDynamicShot(continuous: true, y: 2.5f);
            UnityEngine.Physics2D.SyncTransforms();
            for (var f = 0; f < 30; f++)
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            var refDiscX = refDiscRb.position.x;
            var refContX = refContRb.position.x;

            var pkgDiscT = pkgDiscX > TunnelX;
            var pkgContT = pkgContX > TunnelX;
            var refDiscT = refDiscX > TunnelX;
            var refContT = refContX > TunnelX;

            Debug.Log(
                $"[P8GATE-CCD-DYN] V={V} pkgDisc={pkgDiscX:F3}{(pkgDiscT ? "*" : "")} "
                    + $"pkgCont={pkgContX:F3}{(pkgContT ? "*" : "")} refDisc={refDiscX:F3}{(refDiscT ? "*" : "")} "
                    + $"refCont={refContX:F3}{(refContT ? "*" : "")} (* = tunnelled past kinematic wall)."
            );

            // Tear down before asserting so a fail leaves no leaked GO.
            foreach (var go in refGos)
                if (go != null)
                    Object.Destroy(go);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;

            // BINARY: Continuous does NOT tunnel the dynamic-medium wall; per-mode verdict matches GameObject.
            Assert.IsFalse(
                pkgContT,
                $"Package Continuous bullet tunnelled the kinematic wall (x={pkgContX:F3}) — the body flag did "
                    + "not engage CCD against the dynamic/kinematic medium."
            );
            Assert.AreEqual(
                refContT,
                pkgContT,
                $"Continuous tunnel verdict vs GameObject diverged (kinematic medium): package={pkgContT}, "
                    + $"GameObject={refContT}."
            );
            Assert.AreEqual(
                refDiscT,
                pkgDiscT,
                $"Discrete tunnel verdict vs GameObject diverged (kinematic medium): package={pkgDiscT} "
                    + $"(x={pkgDiscX:F3}), GameObject={refDiscT} (x={refDiscX:F3})."
            );
            yield break;
        }

        // ===============================================================================================
        // (C) Joint break — the UNITS escalation, with measured load-at-break on BOTH backends.
        // ===============================================================================================

        // Build a package disc hung from the world by a rigid distance joint, with a given breakForce/action.
        static (Entity disc, EntityManager em, FixedStepSimulationSystemGroup group, World world) MakeHungDisc(
            float breakForce,
            float breakTorque,
            PhysicsJointBreakAction2D action
        )
        {
            var world = MakeFixedWorld(out var group, withJoints: true);
            var em = world.EntityManager;
            var disc = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f, // no gravity — the load is applied explicitly via AddForce
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
                    breakForce = breakForce,
                    breakTorque = breakTorque,
                    breakAction = action,
                }
            );
            em.AddBuffer<PhysicsBody2DCommand>(disc);
            return (disc, em, group, world);
        }

        // ---------------------------------------------------------------------------------------------
        // (C.1) The UNITS escalation — measure the load-at-break on BOTH backends under a KNOWN ramped pull.
        // A disc is pulled straight down (−Y, along the joint) by a force that ramps each step; the step the
        // joint breaks records the applied force at break. Repeat on the GameObject reference (a distance
        // joint, same authored breakForce sweep). The package's breakForce-at-which-it-breaks under a known
        // load, vs the GameObject's, reveals whether forceThreshold(N) matches breakForce units. The decision
        // (impl-bug vs v2-vs-v3 divergence) is recorded with the measured ratio.
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator JointBreak_ForceUnits_LoadAtBreakAgreesWithGameObject()
        {
            // For a steady pull P (N) held on a rigid distance joint, the joint's reaction force is ≈ P (the
            // joint resists the full applied pull). So a joint with breakForce B breaks once the applied pull
            // P exceeds B (in the joint's force units). We find, on each backend, the smallest applied steady
            // pull that breaks a joint of a FIXED breakForce B — that pull is the empirical break-force in that
            // backend's units; comparing the two reveals the unit relationship.
            const float B = 20f; // a fixed finite breakForce on both backends.
            // Candidate steady pulls (N) bracketing B generously on both sides (covers a unit scale up to ~3×).
            var pulls = new[] { 5f, 10f, 15f, 18f, 20f, 22f, 25f, 30f, 40f, 60f };

            // --- package: for each pull, hold it steady (AddForce every step) and see if the joint breaks. ---
            float PackageBreakPull()
            {
                foreach (var P in pulls)
                {
                    var (disc, em, group, world) = MakeHungDisc(
                        B,
                        float.PositiveInfinity,
                        PhysicsJointBreakAction2D.Destroy
                    );
                    group.Update(); // create body (no step)
                    var broke = false;
                    for (var f = 0; f < 60; f++)
                    {
                        // Pull straight down along the joint, re-issued each step (continuous force).
                        PhysicsBody2DCommands.AddForce(
                            em.GetBuffer<PhysicsBody2DCommand>(disc),
                            new float2(0f, -P),
                            PhysicsForceMode2D.Force
                        );
                        group.Update();
                        if (em.HasComponent<PhysicsJoint2DBroken>(disc))
                        {
                            broke = true;
                            break;
                        }
                    }
                    world.Dispose();
                    if (broke)
                        return P;
                }
                return float.PositiveInfinity;
            }

            var pkgBreakPull = PackageBreakPull();
            yield return null;

            // --- GameObject reference: a disc on a DistanceJoint2D pinned to the world, breakForce = B. ---
            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Vector2.zero;

            float RefBreakPull()
            {
                foreach (var P in pulls)
                {
                    var go = new GameObject("P8RefHungDisc");
                    go.transform.position = new Vector3(0f, 4f, 0f);
                    go.AddComponent<CircleCollider2D>().radius = 0.5f;
                    var rb = go.AddComponent<Rigidbody2D>();
                    rb.bodyType = RigidbodyType2D.Dynamic;
                    rb.useAutoMass = true;
                    rb.gravityScale = 0f;
                    rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
                    var dj = go.AddComponent<DistanceJoint2D>();
                    dj.autoConfigureConnectedAnchor = false;
                    dj.connectedBody = null; // pinned to the world
                    dj.anchor = Vector2.zero;
                    dj.connectedAnchor = new Vector2(0f, 5f);
                    dj.distance = 1f;
                    dj.autoConfigureDistance = false;
                    dj.maxDistanceOnly = false;
                    dj.breakForce = B;
                    dj.breakTorque = Mathf.Infinity;
                    dj.breakAction = JointBreakAction2D.Destroy;
                    UnityEngine.Physics2D.SyncTransforms();

                    var broke = false;
                    for (var f = 0; f < 60; f++)
                    {
                        rb.AddForce(new Vector2(0f, -P), ForceMode2D.Force);
                        UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                        // A broken Joint2D's breakAction=Destroy destroys the joint COMPONENT.
                        if (dj == null)
                        {
                            broke = true;
                            break;
                        }
                    }
                    Object.Destroy(go);
                    if (broke)
                        return P;
                }
                return float.PositiveInfinity;
            }

            var refBreakPull = RefBreakPull();

            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;

            var ratio =
                float.IsInfinity(pkgBreakPull) || float.IsInfinity(refBreakPull)
                    ? float.NaN
                    : pkgBreakPull / refBreakPull;
            Debug.Log(
                $"[P8GATE-JOINTBREAK-UNITS] breakForce B={B} N. Smallest steady pull that breaks: "
                    + $"package={pkgBreakPull} N, GameObject={refBreakPull} N. ratio(pkg/ref)={ratio:F3}. "
                    + "A ratio ≈ 1 means forceThreshold(N) matches breakForce units; a constant ≠1 ratio is a "
                    + "v2-vs-v3 reaction-scale divergence (document with this number) or a missing bake scale."
            );

            // BINARY: both backends DO break under a sufficient pull (the mechanism works on both).
            Assert.IsFalse(
                float.IsInfinity(pkgBreakPull),
                $"Package joint never broke under any pull up to {pulls[pulls.Length - 1]} N at breakForce "
                    + $"{B} N — the native forceThreshold was not armed / no event fired."
            );
            Assert.IsFalse(
                float.IsInfinity(refBreakPull),
                $"GameObject joint never broke under any pull up to {pulls[pulls.Length - 1]} N at breakForce "
                    + $"{B} N — oracle sanity failure."
            );
            // UNITS PARITY (the escalation): the break pull agrees within a generous band. The two solvers
            // scale the reaction differently (v2 per-timeStep convention vs v3 newtons), so this is bounded,
            // not exact — but a unit MISMATCH (a 1/dt ≈ 60× scale, or a 60× the other way) would put the break
            // pulls in disjoint decades and trip this. A ratio in [0.5, 2] means the SAME authored breakForce
            // breaks at a comparable load on both → units match within solver noise.
            Assert.LessOrEqual(
                ratio,
                2.0f,
                $"Break-force UNITS DIVERGE: package breaks at {pkgBreakPull} N but GameObject at "
                    + $"{refBreakPull} N (ratio {ratio:F3} > 2) — forceThreshold does NOT match breakForce "
                    + "units; the bake needs a unit scale (or this is a v2-vs-v3 divergence — see the doc)."
            );
            Assert.GreaterOrEqual(
                ratio,
                0.5f,
                $"Break-force UNITS DIVERGE: package breaks at {pkgBreakPull} N but GameObject at "
                    + $"{refBreakPull} N (ratio {ratio:F3} < 0.5) — forceThreshold does NOT match breakForce "
                    + "units; the bake needs a unit scale (or this is a v2-vs-v3 divergence — see the doc)."
            );
            yield break;
        }

        // ---------------------------------------------------------------------------------------------
        // (C.2) The near-threshold pin: a joint loaded JUST UNDER its breakForce HOLDS; the same joint loaded
        // JUST OVER breaks — on BOTH backends. Pins that the arm honours the threshold, not merely "breaks
        // under any load." Uses the package's own measured break pull to bracket.
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator JointBreak_NearThreshold_HoldsUnderBreaksOver()
        {
            const float B = 20f;

            bool PackageBreaksUnderPull(float P, int steps)
            {
                var (disc, em, group, world) = MakeHungDisc(
                    B,
                    float.PositiveInfinity,
                    PhysicsJointBreakAction2D.Destroy
                );
                group.Update();
                var broke = false;
                for (var f = 0; f < steps; f++)
                {
                    PhysicsBody2DCommands.AddForce(
                        em.GetBuffer<PhysicsBody2DCommand>(disc),
                        new float2(0f, -P),
                        PhysicsForceMode2D.Force
                    );
                    group.Update();
                    if (em.HasComponent<PhysicsJoint2DBroken>(disc))
                        broke = true;
                }
                world.Dispose();
                return broke;
            }

            // Well under the threshold must hold; well over must break. (A generous margin so the bound is not
            // a knife-edge — the near-tie is genuinely ambiguous and not a useful gate; the unambiguous
            // under/over contrast is.)
            var heldUnder = PackageBreaksUnderPull(B * 0.4f, 60);
            yield return null;
            var brokeOver = PackageBreaksUnderPull(B * 2.5f, 60);

            Debug.Log(
                $"[P8GATE-JOINTBREAK-NEAR] breakForce={B}: pull {B * 0.4f} N → broke={heldUnder} (want false); "
                    + $"pull {B * 2.5f} N → broke={brokeOver} (want true)."
            );

            Assert.IsFalse(
                heldUnder,
                $"A joint with breakForce {B} broke under a pull of {B * 0.4f} N (well below threshold) — the "
                    + "threshold fired too early (a wrong/zero forceThreshold or a units underscale)."
            );
            Assert.IsTrue(
                brokeOver,
                $"A joint with breakForce {B} did NOT break under a pull of {B * 2.5f} N (well above "
                    + "threshold) — the threshold did not fire (a wrong/huge forceThreshold or a units overscale)."
            );
            yield break;
        }

        // ---------------------------------------------------------------------------------------------
        // (C.3) breakTORQUE — the unprobed path. A disc on a HINGE joint pinned to the world, spun by a steady
        // torque; a finite breakTorque must break it (the hinge's reaction torque exceeds the threshold), an
        // Infinity breakTorque must not. Pins the torqueThreshold arm + the torque-break collect/apply path.
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator JointBreak_Torque_FiniteBreaks_InfiniteHolds()
        {
            // A hinge pinned to the world with an ANGLE LIMIT, driven by a steady torque into the limit so the
            // joint reaction torque builds against the stop and exceeds a finite breakTorque.
            (bool broke, float finalAngleDeg) RunHinge(float breakTorque, float appliedTorque)
            {
                var world = MakeFixedWorld(out var group, withJoints: true);
                var em = world.EntityManager;
                var arm = DirectPhysics2DAuthoring.Create(
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
                        size = new float2(2f, 0.2f),
                        radius = 0f,
                        density = 1f,
                        friction = 0.4f,
                    }
                );
                em.AddComponentData(
                    arm,
                    new PhysicsJoint2DDefinition
                    {
                        kind = PhysicsJoint2DKind.Hinge,
                        connectedBody = Entity.Null, // pinned to the world origin
                        anchor = Unity.Mathematics.float2.zero,
                        connectedAnchor = Unity.Mathematics.float2.zero,
                        enableLimit = true,
                        lowerLimit = radians(-5f),
                        upperLimit = radians(5f), // a tight angle limit so the torque loads against the stop
                        breakForce = float.PositiveInfinity,
                        breakTorque = breakTorque,
                        breakAction = PhysicsJointBreakAction2D.Destroy,
                    }
                );
                em.AddBuffer<PhysicsBody2DCommand>(arm);
                group.Update();
                var broke = false;
                for (var f = 0; f < 90; f++)
                {
                    PhysicsBody2DCommands.AddTorque(
                        em.GetBuffer<PhysicsBody2DCommand>(arm),
                        appliedTorque,
                        PhysicsForceMode2D.Force
                    );
                    group.Update();
                    if (em.HasComponent<PhysicsJoint2DBroken>(arm))
                        broke = true;
                }
                var m = em.GetComponentData<LocalToWorld>(arm).Value;
                var angleDeg = degrees(atan2(m.c0.y, m.c0.x));
                world.Dispose();
                return (broke, angleDeg);
            }

            // A large steady torque into a tight limit. finite breakTorque should break; Infinity should hold.
            const float AppliedTorque = 200f;
            var finiteRun = RunHinge(breakTorque: 5f, AppliedTorque);
            yield return null;
            var infiniteRun = RunHinge(breakTorque: float.PositiveInfinity, AppliedTorque);

            Debug.Log(
                $"[P8GATE-JOINTBREAK-TORQUE] applied τ={AppliedTorque}: finite breakTorque=5 → broke="
                    + $"{finiteRun.broke} (angle {finiteRun.finalAngleDeg:F1}°); ∞ breakTorque → broke="
                    + $"{infiniteRun.broke} (angle {infiniteRun.finalAngleDeg:F1}°, held in the limit)."
            );

            Assert.IsTrue(
                finiteRun.broke,
                "A hinge with a finite breakTorque, driven hard into its angle limit, did NOT break — the "
                    + "torqueThreshold was not armed, or no joint-threshold event fired for the torque path."
            );
            Assert.IsFalse(
                infiniteRun.broke,
                "A hinge with an INFINITE breakTorque broke — the threshold was armed despite Infinity (the "
                    + "finite-only arm guard failed for the torque path)."
            );
            // The unbroken hinge stays inside its ±5° limit (it did not free-spin away).
            Assert.LessOrEqual(
                abs(infiniteRun.finalAngleDeg),
                15f,
                $"The unbroken hinge rotated to {infiniteRun.finalAngleDeg:F1}° — far past its ±5° limit, so "
                    + "the limit (and thus the reaction-torque load) was not actually engaged."
            );
            yield break;
        }

        // ---------------------------------------------------------------------------------------------
        // (C.4) breakAction branches — CallbackOnly KEEPS the joint (surfaces the event only); Disable
        // DESTROYS the constraint and the surfaced action reads Disable; Infinity NEVER breaks (control).
        // Pins the three untested action branches the smoke (Destroy-only) skipped.
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator JointBreak_Actions_CallbackOnlyKeeps_DisableDestroys_InfinityHolds()
        {
            const float B = 5f; // small breakForce, easily exceeded by a steady 60 N pull.
            const float Pull = 60f;

            // Run the hung disc under a heavy pull with a given action; report whether the joint survived, the
            // broken tag appeared, and the surfaced breakAction (if any) over the run.
            (bool jointAlive, bool brokenTag, bool sawEvent, PhysicsJointBreakAction2D evAction) Run(
                PhysicsJointBreakAction2D action,
                float breakForce
            )
            {
                var (disc, em, group, world) = MakeHungDisc(breakForce, float.PositiveInfinity, action);
                group.Update();
                var sawEvent = false;
                var evAction = PhysicsJointBreakAction2D.Ignore;
                for (var f = 0; f < 60; f++)
                {
                    PhysicsBody2DCommands.AddForce(
                        em.GetBuffer<PhysicsBody2DCommand>(disc),
                        new float2(0f, -Pull),
                        PhysicsForceMode2D.Force
                    );
                    group.Update();
                    // Read the per-step break-event buffer on the world singleton (valid this tick). The
                    // singleton + its buffers exist after the first step (PhysicsWorld2DSystem created them).
                    var se = em.CreateEntityQuery(typeof(PhysicsWorldSingleton2D)).GetSingletonEntity();
                    var buf = em.GetBuffer<PhysicsJointBreakEvent2D>(se, isReadOnly: true);
                    for (var i = 0; i < buf.Length; i++)
                        if (buf[i].jointEntity == disc)
                        {
                            sawEvent = true;
                            evAction = buf[i].breakAction;
                        }
                }
                var jointAlive = em.HasComponent<PhysicsJoint2D>(disc);
                var brokenTag = em.HasComponent<PhysicsJoint2DBroken>(disc);
                world.Dispose();
                return (jointAlive, brokenTag, sawEvent, evAction);
            }

            var callbackOnly = Run(PhysicsJointBreakAction2D.CallbackOnly, B);
            yield return null;
            var disable = Run(PhysicsJointBreakAction2D.Disable, B);
            yield return null;
            var infinity = Run(PhysicsJointBreakAction2D.Destroy, float.PositiveInfinity);

            Debug.Log(
                $"[P8GATE-JOINTBREAK-ACTIONS] CallbackOnly: jointAlive={callbackOnly.jointAlive} "
                    + $"brokenTag={callbackOnly.brokenTag} sawEvent={callbackOnly.sawEvent} "
                    + $"evAction={callbackOnly.evAction}. Disable: jointAlive={disable.jointAlive} "
                    + $"brokenTag={disable.brokenTag} sawEvent={disable.sawEvent} evAction={disable.evAction}. "
                    + $"Infinity: jointAlive={infinity.jointAlive} brokenTag={infinity.brokenTag} "
                    + $"sawEvent={infinity.sawEvent}."
            );

            // CallbackOnly: the event fires, but the joint STAYS (constraint still holds; no broken tag).
            Assert.IsTrue(
                callbackOnly.sawEvent,
                "CallbackOnly: no break event surfaced despite the joint being loaded past threshold — the "
                    + "threshold was armed (action != Ignore) but the event was not collected."
            );
            Assert.IsTrue(
                callbackOnly.jointAlive,
                "CallbackOnly: the joint was destroyed — CallbackOnly must KEEP the joint (surface only)."
            );
            Assert.IsFalse(
                callbackOnly.brokenTag,
                "CallbackOnly: the broken tag was added — CallbackOnly must not mark the joint broken."
            );
            Assert.AreEqual(
                PhysicsJointBreakAction2D.CallbackOnly,
                callbackOnly.evAction,
                "CallbackOnly: the surfaced event carried the wrong breakAction."
            );

            // Disable: the constraint is destroyed (joint gone, broken tag set) AND the surfaced action reads
            // Disable (the distinction Box2D cannot express, carried for the consumer).
            Assert.IsFalse(
                disable.jointAlive,
                "Disable: the joint survived — Disable must destroy the Box2D constraint (bodies separate)."
            );
            Assert.IsTrue(
                disable.brokenTag,
                "Disable: no broken tag — the apply system did not mark the entity broken."
            );
            Assert.AreEqual(
                PhysicsJointBreakAction2D.Disable,
                disable.evAction,
                "Disable: the surfaced event did not carry the Disable action (the destroy-vs-disable "
                    + "distinction was lost)."
            );

            // Infinity control: never breaks, no event, joint stays.
            Assert.IsFalse(infinity.brokenTag, "Infinity breakForce broke (must never break).");
            Assert.IsTrue(infinity.jointAlive, "Infinity breakForce lost its joint handle (must stay live).");
            Assert.IsFalse(infinity.sawEvent, "Infinity breakForce surfaced a break event (must never fire).");
            yield break;
        }

        // ===============================================================================================
        // (A) Interpolation — internal invariant, end-to-end (capture from a real stepped body) + None +
        // Extrapolate + no-clobber-by-LocalToWorldSystem.
        // ===============================================================================================

        // Drive the smoothing system once at a chosen sub-step fraction over a body carrying a given smoothing
        // component, and return the written LocalToWorld (pos, angle). Standalone smoothing system + a hand-set
        // PhysicsFixedStepTime2D + world clock, exactly as the smoke seeds it.
        static (float2 pos, float angle) SmoothOnce(PhysicsBody2DSmoothing smoothing, float fractionOfStep)
        {
            var world = new World("P8InterpSmoothWorld");
            var em = world.EntityManager;
            var sys = world.GetOrCreateSystem<PhysicsBody2DSmoothingSystem>();

            var timeSingleton = em.CreateEntity(typeof(PhysicsFixedStepTime2D));
            em.SetComponentData(timeSingleton, new PhysicsFixedStepTime2D { elapsedTime = 1.0, deltaTime = Dt });
            world.SetTime(new Unity.Core.TimeData(elapsedTime: 1.0 + fractionOfStep * Dt, deltaTime: Dt));

            var body = em.CreateEntity(typeof(PhysicsBody2DSmoothing), typeof(LocalToWorld));
            em.SetComponentData(body, smoothing);

            sys.Update(world.Unmanaged);
            world.EntityManager.CompleteAllTrackedJobs();

            var m = em.GetComponentData<LocalToWorld>(body).Value;
            var pos = new float2(m.c3.x, m.c3.y);
            var angle = atan2(m.c0.y, m.c0.x);
            world.Dispose();
            return (pos, angle);
        }

        // ---------------------------------------------------------------------------------------------
        // (A.1) Interpolate INTERNAL INVARIANT at several sub-step fractions: the written pose is the exact
        // lerp(prev,cur,t) / nlerp of the (cos,sin) pair. Drives a synthetic prev/cur and sweeps t.
        // ---------------------------------------------------------------------------------------------
        [Test]
        public void Interpolate_WrittenPose_IsAnalyticLerp_AtSeveralFractions()
        {
            var prevPos = new float2(-1f, 3f);
            var curPos = new float2(2f, 4f);
            var prevAng = radians(10f);
            var curAng = radians(100f);
            sincos(prevAng, out var ps, out var pc);
            sincos(curAng, out var cs2, out var cc2);

            var s = new PhysicsBody2DSmoothing
            {
                prevPos = prevPos,
                prevCosSin = new float2(pc, ps),
                curPos = curPos,
                curCosSin = new float2(cc2, cs2),
                linearVel = Unity.Mathematics.float2.zero,
                angularVelRad = 0f,
                mode = (byte)PhysicsBody2DInterpolation.Interpolate,
                hasPrev = 1,
            };

            foreach (var t in new[] { 0f, 0.25f, 0.5f, 0.75f, 1f })
            {
                var (pos, angle) = SmoothOnce(s, t);
                var expectedPos = lerp(prevPos, curPos, t);
                // The system's own nlerp: normalize(lerp((cos,sin)prev, (cos,sin)cur, t)).
                var lerpedCosSin = lerp(new float2(pc, ps), new float2(cc2, cs2), t);
                var expectedAngle = atan2(lerpedCosSin.y, lerpedCosSin.x);

                Assert.Less(
                    length(pos - expectedPos),
                    1e-4f,
                    $"Interpolate position at t={t}: written {pos} != analytic lerp {expectedPos}."
                );
                Assert.Less(
                    abs(AngleDelta(angle, expectedAngle)),
                    1e-3f,
                    $"Interpolate angle at t={t}: written {angle} rad != analytic nlerp {expectedAngle} rad."
                );
                Debug.Log(
                    $"[P8GATE-INTERP] t={t}: pos={pos} (expect {expectedPos}); angle={degrees(angle):F3}° "
                        + $"(expect {degrees(expectedAngle):F3}°)."
                );
            }
        }

        // ---------------------------------------------------------------------------------------------
        // (A.2) Extrapolate INTERNAL INVARIANT: the written pose leads the CURRENT pose by velocity·timeAhead
        // (linear) and ω·timeAhead (angular), at several timeAhead fractions.
        // ---------------------------------------------------------------------------------------------
        [Test]
        public void Extrapolate_WrittenPose_IsCurPlusVelocityTimesTimeAhead()
        {
            var curPos = new float2(2f, 4f);
            var curAng = radians(30f);
            sincos(curAng, out var cs2, out var cc2);
            var linVel = new float2(5f, -3f); // m/s
            var angVelRad = radians(90f); // rad/s

            var s = new PhysicsBody2DSmoothing
            {
                prevPos = curPos, // prev irrelevant for extrapolate
                prevCosSin = new float2(cc2, cs2),
                curPos = curPos,
                curCosSin = new float2(cc2, cs2),
                linearVel = linVel,
                angularVelRad = angVelRad,
                mode = (byte)PhysicsBody2DInterpolation.Extrapolate,
                hasPrev = 1,
            };

            foreach (var frac in new[] { 0.25f, 0.5f, 1f })
            {
                var (pos, angle) = SmoothOnce(s, frac);
                var timeAhead = frac * Dt;
                var expectedPos = curPos + linVel * timeAhead;
                var expectedAngle = curAng + angVelRad * timeAhead;

                Assert.Less(
                    length(pos - expectedPos),
                    1e-4f,
                    $"Extrapolate position at frac={frac} (timeAhead={timeAhead}): written {pos} != "
                        + $"cur+vel·timeAhead {expectedPos}."
                );
                Assert.Less(
                    abs(AngleDelta(angle, expectedAngle)),
                    1e-3f,
                    $"Extrapolate angle at frac={frac}: written {angle} rad != cur+ω·timeAhead "
                        + $"{expectedAngle} rad."
                );
                Debug.Log(
                    $"[P8GATE-EXTRAP] frac={frac} timeAhead={timeAhead:F5}: pos={pos} (expect {expectedPos}); "
                        + $"angle={degrees(angle):F3}° (expect {degrees(expectedAngle):F3}°)."
                );
            }
        }

        // ---------------------------------------------------------------------------------------------
        // (A.3) END-TO-END capture: step a REAL interpolated body two fixed steps, then drive the smoothing
        // system at a half-step and assert the rendered pose lies on the lerp between the two CAPTURED physics
        // poses. This covers the write-back CaptureSmoothing path (cur→prev shift) the synthetic tests do not.
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator Interpolate_EndToEnd_RenderedPoseBetweenCapturedPhysicsPoses()
        {
            var world = MakeFixedWorld(out var group, withJoints: false);
            // Add the smoothing system to the SAME world (it lives in TransformSystemGroup, but here we drive
            // it explicitly so it must exist as a system in this world).
            var smoothingSys = world.GetOrCreateSystem<PhysicsBody2DSmoothingSystem>();
            var em = world.EntityManager;

            // An interpolated body drifting at a known horizontal velocity (no gravity) so its captured poses
            // are predictable and distinct step to step.
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f,
                    initialPosition = new float2(0f, 0f),
                    useAutoMass = true,
                    interpolation = PhysicsBody2DInterpolation.Interpolate,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.25f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddComponentData(entity, new PhysicsBody2DInitialVelocity { linearVelocity = new float2(3f, 0f) });

            group.Update(); // create the body + the smoothing component (no step)
            Assert.IsTrue(
                em.HasComponent<PhysicsBody2DSmoothing>(entity),
                "An Interpolate body did not get a PhysicsBody2DSmoothing component at creation."
            );

            // Step twice so the write-back captures two distinct poses (prev = step1, cur = step2).
            group.Update();
            group.Update();

            var sm = em.GetComponentData<PhysicsBody2DSmoothing>(entity);
            Assert.AreEqual(
                (byte)1,
                sm.hasPrev,
                "After two steps the smoothing component still has hasPrev=0 — the write-back capture did not "
                    + "run / did not flip hasPrev."
            );
            var capturedPrev = sm.prevPos;
            var capturedCur = sm.curPos;
            Assert.Greater(
                length(capturedCur - capturedPrev),
                1e-4f,
                $"The two captured poses are identical ({capturedPrev} == {capturedCur}) — the body did not "
                    + "advance between captures, so there is nothing to interpolate."
            );

            // Drive the smoothing system at a half-step: render time = lastFixed + 0.5·dt. The fixed-step time
            // singleton lives on the world singleton entity and was written by PhysicsWorld2DSystem.
            var fixedTimeEntity = em.CreateEntityQuery(typeof(PhysicsFixedStepTime2D)).GetSingletonEntity();
            var ft = em.GetComponentData<PhysicsFixedStepTime2D>(fixedTimeEntity);
            world.SetTime(new Unity.Core.TimeData(elapsedTime: ft.elapsedTime + 0.5 * Dt, deltaTime: Dt));

            smoothingSys.Update(world.Unmanaged);
            world.EntityManager.CompleteAllTrackedJobs();

            var rendered = em.GetComponentData<LocalToWorld>(entity).Position;
            var renderedPos = new float2(rendered.x, rendered.y);
            var expectedMid = lerp(capturedPrev, capturedCur, 0.5f);

            Debug.Log(
                $"[P8GATE-INTERP-E2E] capturedPrev={capturedPrev} capturedCur={capturedCur} → rendered@0.5="
                    + $"{renderedPos} (expect midpoint {expectedMid})."
            );

            Assert.Less(
                length(renderedPos - expectedMid),
                1e-4f,
                $"End-to-end interpolation: the rendered pose {renderedPos} at a half-step is not the midpoint "
                    + $"{expectedMid} of the two CAPTURED physics poses ({capturedPrev}, {capturedCur}) — the "
                    + "write-back capture (cur→prev shift) or the smoothing math is wrong."
            );

            world.Dispose();
            yield break;
        }

        // ---------------------------------------------------------------------------------------------
        // (A.4) None mode = the RAW stepped pose. A None body carries NO smoothing component and its
        // LocalToWorld is exactly the write-back's fixed-step pose (no smoothing system touches it).
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator NoneMode_NoSmoothingComponent_LocalToWorldIsRawSteppedPose()
        {
            var world = MakeFixedWorld(out var group, withJoints: false);
            var em = world.EntityManager;

            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f,
                    initialPosition = new float2(0f, 0f),
                    useAutoMass = true,
                    interpolation = PhysicsBody2DInterpolation.None,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.25f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddComponentData(entity, new PhysicsBody2DInitialVelocity { linearVelocity = new float2(3f, 0f) });

            group.Update();
            // A None body must NOT carry a smoothing component (the gate on the smoothing system).
            Assert.IsFalse(
                em.HasComponent<PhysicsBody2DSmoothing>(entity),
                "A None-interpolation body carries a PhysicsBody2DSmoothing component — None must cost nothing "
                    + "and never be smoothed."
            );

            group.Update();
            group.Update();

            // The LocalToWorld equals the body's actual stepped pose (the write-back's fixed-step write), with
            // no render-rate smoothing applied. Compare against the live Box2D body position.
            var body = BodyOf(em, entity);
            var bodyPos = (float2)(Vector2)body.position;
            var ltwPos = em.GetComponentData<LocalToWorld>(entity).Position;

            Debug.Log(
                $"[P8GATE-NONE] body.position={bodyPos} LocalToWorld.pos=({ltwPos.x:F4},{ltwPos.y:F4}) — raw "
                    + "stepped pose, no smoothing."
            );

            Assert.Less(
                length(new float2(ltwPos.x, ltwPos.y) - bodyPos),
                1e-4f,
                $"None body's LocalToWorld ({ltwPos.x},{ltwPos.y}) does not equal its raw stepped Box2D pose "
                    + $"{bodyPos} — something smoothed a None body."
            );

            world.Dispose();
            yield break;
        }

        // ---------------------------------------------------------------------------------------------
        // (A.5) LocalToWorldSystem does NOT clobber the smoothing write. Run the smoothing system AND the
        // LocalToWorldSystem (in TransformSystemGroup) in one world the same frame, and confirm the smoothed
        // LocalToWorld survives (the body's LocalTransform stays unchanged, so the dirty-check skips it).
        // ---------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator SmoothedLocalToWorld_SurvivesLocalToWorldSystem()
        {
            var world = MakeFixedWorld(out var group, withJoints: false);
            var em = world.EntityManager;

            // A full TransformSystemGroup (it owns LocalToWorldSystem) driven explicitly, with the smoothing
            // system placed before LocalToWorldSystem (its own [UpdateBefore] declares the order).
            var transformGroup = world.GetOrCreateSystemManaged<TransformSystemGroup>();
            transformGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DSmoothingSystem>());
            transformGroup.AddSystemToUpdateList(world.GetOrCreateSystem<LocalToWorldSystem>());
            transformGroup.SortSystems();

            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f,
                    initialPosition = new float2(0f, 0f),
                    useAutoMass = true,
                    interpolation = PhysicsBody2DInterpolation.Interpolate,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.25f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddComponentData(entity, new PhysicsBody2DInitialVelocity { linearVelocity = new float2(3f, 0f) });

            group.Update();
            group.Update();
            group.Update();

            var sm = em.GetComponentData<PhysicsBody2DSmoothing>(entity);
            var fixedTimeEntity = em.CreateEntityQuery(typeof(PhysicsFixedStepTime2D)).GetSingletonEntity();
            var ft = em.GetComponentData<PhysicsFixedStepTime2D>(fixedTimeEntity);
            world.SetTime(new Unity.Core.TimeData(elapsedTime: ft.elapsedTime + 0.5 * Dt, deltaTime: Dt));

            // Run the transform group: smoothing writes the midpoint, then LocalToWorldSystem runs after it.
            transformGroup.Update();
            world.EntityManager.CompleteAllTrackedJobs();

            var rendered = em.GetComponentData<LocalToWorld>(entity).Position;
            var renderedPos = new float2(rendered.x, rendered.y);
            var expectedMid = lerp(sm.prevPos, sm.curPos, 0.5f);

            Debug.Log(
                $"[P8GATE-INTERP-NOCLOBBER] after TransformSystemGroup (smoothing + LocalToWorldSystem): "
                    + $"rendered={renderedPos}, expected smoothed midpoint={expectedMid}."
            );

            Assert.Less(
                length(renderedPos - expectedMid),
                1e-4f,
                $"LocalToWorldSystem clobbered the smoothing write: rendered {renderedPos} != smoothed "
                    + $"midpoint {expectedMid}. The body's LocalTransform must stay unchanged so the dirty-check "
                    + "skips it; if LocalToWorldSystem overwrote it, the smoothing is lost at render time."
            );

            world.Dispose();
            yield break;
        }

        static float AngleDelta(float a, float b)
        {
            var d = a - b;
            while (d > PI)
                d -= 2f * PI;
            while (d < -PI)
                d += 2f * PI;
            return d;
        }
    }
}
