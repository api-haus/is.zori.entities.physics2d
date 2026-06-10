using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The reusable GameObject-vs-ECS parity oracle. One built-in-authored source (a SubScene's child
    /// scene) is run two ways from a single PlayMode session and compared: the ECS side bakes it (the
    /// parent scene's <c>SubScene</c> streams + bakes the authoring <c>Rigidbody2D</c>/<c>Collider2D</c>
    /// through the package's bakers), and the GameObject reference instantiates the SAME authored bodies
    /// live and steps them with <c>Physics2D.Simulate</c>. There is no second hand-authored reference
    /// scene — the same child scene feeds both backends, so the two cannot drift apart by authoring.
    /// </summary>
    /// <remarks>
    /// <para><b>Why this is the e2e methodology for every later phase.</b> A phase ships a new
    /// baker/shape/joint by authoring one built-in-component fixture (a child SubScene + a parent carrying
    /// the SubScene, both registered in build settings by an editor fixture builder), then calls
    /// <see cref="RunParity"/> with that fixture's scene names, the step parameters, and an
    /// <see cref="ParityEnvelope"/>. The harness returns a parity assertion; the phase supplies only the
    /// scene and the tolerance.</para>
    ///
    /// <para><b>Why the gate is forgiving, not tight.</b> The determinism probe
    /// (<c>docs/orchestrate/entities-physics2d/00d-determinism-probe-results.md</c>) measured the two paths
    /// to be <em>different solvers</em> in editor 6000.6.0a6: the GameObject <c>Physics2D.Simulate</c> runs
    /// the Box2D-v2 iteration solver (its <c>velocityIterations</c>/<c>positionIterations</c> move the
    /// trajectory; the v3 <c>useSubStepping</c>/<c>maxSubStepCount</c> are inert), while the package's
    /// <c>PhysicsWorld.Simulate</c> runs the Box2D-v3 sub-stepping solver. Two Box2D-lineage solvers
    /// converging on the same physics agree to a <em>bounded</em> band, never bit-identity. So the gate
    /// asserts disqualifiers (solver-independent correctness) ALWAYS, plus a generous growth-bounded
    /// envelope on per-step position/angle error — wide enough to tolerate v2-vs-v3 noise, tight enough to
    /// fail loudly on a real bake/mapping regression.</para>
    ///
    /// <para><b>Body↔entity matching.</b> Both backends start every body from the identical authored
    /// transform (single authoring), so the stable correspondence key is the initial pose: each side's
    /// trajectory is sorted by initial (y, then x). This is creation-order-equivalent whenever authored
    /// initial positions are distinct — which a parity fixture must guarantee (coincident start poses have
    /// no stable cross-backend identity). The Circle case has one body, so matching is trivial.</para>
    /// </remarks>
    public static class PhysicsParityHarness
    {
        const int LoadTimeoutFrames = 600;
        const float FixedDt = 1f / 60f;

        // Held-identical determinism preconditions (00c §3), applied to the GameObject reference so it
        // matches the package's world definition. These are the probe's matched values.
        static readonly Vector2 Gravity = new(0f, -9.81f);
        const float ReferenceFriction = 0.4f;
        const float ReferenceRestitution = 0f;

        /// <summary>
        /// One sample of a body's pose at one step: world-space (x, y) and the in-plane rotation in
        /// radians. The element a parity comparison operates on, identical in shape for both backends.
        /// </summary>
        public struct Pose
        {
            public float2 position;
            public float angleRadians;
        }

        /// <summary>
        /// The generous, growth-bounded tolerance a scene supplies. Position error is allowed to grow
        /// linearly with step index (the free-fall convention offset between the v2 and v3 integrators is
        /// exactly linear — <c>00d</c> — so a flat per-step position epsilon is wrong); angle error is a
        /// flat cap (free-fall angle error is exactly zero). The settle region is the coarse disqualifier
        /// bound: after the run, every body must rest inside it.
        /// </summary>
        public struct ParityEnvelope
        {
            /// <summary>Flat base position error allowed at step 0 (m). Small — measurement margin.</summary>
            public float positionBaseMeters;

            /// <summary>
            /// Per-step linear growth of the position error band (m/step). The allowed position error at
            /// step <c>s</c> is <c>positionBaseMeters + positionGrowthPerStep * (s + 1)</c>. Start from the
            /// probe's measured ~1.47e-3 m/step free-fall offset times a small margin (~1.7e-3); widen per
            /// scene as contacts/tumbling demand and document the widening.
            /// </summary>
            public float positionGrowthPerStep;

            /// <summary>Flat angle error cap (rad), applied at every step. Start ~1e-2.</summary>
            public float angleCapRadians;

            /// <summary>Coarse expected rest region (world AABB) every body must end inside.</summary>
            public float2 settleRegionMin;
            public float2 settleRegionMax;

            /// <summary>
            /// Minimum world-space distance each body must travel from its start, so a silently-no-op bake
            /// (body never moved) is disqualified. For a free-fall scene this is comfortably large.
            /// </summary>
            public float minTravelMeters;
        }

        /// <summary>
        /// Run the full parity gate for one fixture. Loads the parent scene (ECS bake), builds the
        /// GameObject reference from the additively-loaded child authoring scene, steps both
        /// <paramref name="stepCount"/> times at <paramref name="dt"/>, and asserts disqualifiers +
        /// envelope. The coroutine drives the ECS <see cref="FixedStepSimulationSystemGroup"/> explicitly
        /// (rate manager swapped) and <see cref="Physics2D.Simulate"/> directly, so it produces
        /// deterministic steps under PlayMode-in-batchmode where frame-pumping does not. Yields
        /// <c>null</c> only while waiting for the SubScene to stream (never <c>WaitForEndOfFrame</c>, which
        /// does not tick in batchmode).
        /// </summary>
        public static IEnumerator RunParity(
            string parentSceneName,
            string childSceneName,
            float dt,
            int stepCount,
            ParityEnvelope envelope
        )
        {
            // --- ECS side: load the parent, wait for the SubScene to stream + bake. ---
            SceneManager.LoadScene(parentSceneName, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");

            // The compared bodies are the NON-STATIC ones only: the GameObject reference side collects live
            // Rigidbody2D bodies (CollectReferenceBodies), and a static floor in a fixture is collider-only
            // (no Rigidbody2D), so it never enters the reference set. The ECS side must match that — a baked
            // static floor carries a (default) PhysicsBody2DDefinition + PhysicsShape2D + LocalToWorld and so
            // would otherwise inflate the ECS count and have no reference counterpart. Both queries therefore
            // also read PhysicsBody2DDefinition so the harness can filter to non-static bodies. A static body
            // does not move, so excluding it from the comparison loses no parity signal; its contact surface
            // still exists on both backends (it is baked/loaded), it is just not a compared body.
            var bakedQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var liveQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            // Disable the FixedStepSimulationSystemGroup for the whole bake-wait and reference-load phase. The
            // wait below yields `null` to let the SubScene stream, and on each yield the default world updates;
            // if this group is live it creates the baked bodies the instant they appear and then steps them a
            // wall-clock-dependent number of times before the wait exits — all before the reference scene is
            // even loaded. That uncontrolled pre-stepping is what made the ECS trajectory lead the reference by
            // a NONDETERMINISTIC number of steps (a single pre-loop reference Simulate cannot cancel a count
            // that varies with streaming timing). With the group disabled, the ECS bodies bake into existence
            // but do not integrate until the lockstep loop drives the group explicitly, so both backends start
            // the loop at their identical authored pose regardless of how long streaming took.
            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            fixedGroup.Enabled = false;

            var framesWaited = 0;
            while (
                CountNonStatic(bakedQuery) == 0 && framesWaited < LoadTimeoutFrames
            )
            {
                framesWaited++;
                yield return null;
            }
            var bakedCount = CountNonStatic(bakedQuery);
            Assert.Greater(
                bakedCount,
                0,
                $"No baked dynamic body appeared after {framesWaited} frames — the SubScene "
                    + $"'{parentSceneName}' did not stream/bake. Build the fixture first via its editor "
                    + "fixture builder."
            );

            // --- GameObject reference: instantiate the SAME authored bodies, live, stepped manually. ---
            // Set Script stepping BEFORE the additive load so the authored bodies never auto-step while we
            // wait. Save/restore every global Physics2D knob the harness touches, so it leaves no global
            // state behind (mirrors the probe's discipline).
            // Fully qualified: this code lives in namespace Zori.Entities.Physics2D.Tests, so a bare
            // `Physics2D.` is parsed as the enclosing-namespace segment, not UnityEngine.Physics2D.
            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;

            var referenceMaterial = new PhysicsMaterial2D("ParityReference")
            {
                friction = ReferenceFriction,
                bounciness = ReferenceRestitution,
            };

            var refLoad = SceneManager.LoadSceneAsync(childSceneName, LoadSceneMode.Additive);
            Assert.IsNotNull(
                refLoad,
                $"Child authoring scene '{childSceneName}' is not loadable by name — it must be registered "
                    + "in build settings by the fixture builder."
            );
            while (!refLoad.isDone)
                yield return null;

            var childScene = SceneManager.GetSceneByName(childSceneName);
            var bodies = CollectReferenceBodies(childScene, referenceMaterial);
            Assert.AreEqual(
                bakedCount,
                bodies.Count,
                $"Authored body count mismatch: the SubScene baked {bakedCount} bodies but the child scene "
                    + $"'{childSceneName}' instantiated {bodies.Count} live Rigidbody2D bodies. Single "
                    + "authoring is violated — both sides must come from the identical authored source."
            );
            UnityEngine.Physics2D.SyncTransforms();

            // --- Step both sides in lockstep, capturing per-step trajectories in body order. ---
            // Re-enable the group (disabled through the bake-wait) and swap its rate manager so each
            // fixedGroup.Update() runs exactly one fixed step at the test's dt.
            var savedRateManager = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(dt);
            fixedGroup.Enabled = true;

            // First group Update runs body creation + write-back so the live PhysicsBody2D entities (and their
            // LocalToWorld) exist before any pose is read; PhysicsWorld2DSystem skips its Simulate on the
            // creation frame, so this Update does NOT integrate. With the group having been disabled for the
            // whole bake-wait, the ECS bodies have taken zero steps — they sit at their authored pose, exactly
            // like the just-loaded, un-stepped reference. Both backends then advance one step per loop iteration
            // from that shared authored start, so capture s reflects (s + 1) integrations on each, deterministic
            // regardless of streaming timing.
            fixedGroup.Update();
            var liveCount = CountNonStatic(liveQuery);
            Assert.AreEqual(
                bakedCount,
                liveCount,
                $"Body creation did not run for every baked body: {bakedCount} baked, {liveCount} live "
                    + "PhysicsBody2D after the first fixed step. PhysicsWorld2DSystem did not create them."
            );

            var ecsTraj = new Pose[stepCount][]; // [step][bodyIndex]
            var refTraj = new Pose[stepCount][];

            for (var s = 0; s < stepCount; s++)
            {
                // Advance both backends one fixed step per iteration from the shared authored start.
                fixedGroup.Update();
                ecsTraj[s] = CaptureEcsPoses(liveQuery, world);

                UnityEngine.Physics2D.Simulate(dt, UnityEngine.Physics2D.AllLayers);
                refTraj[s] = CaptureReferencePoses(bodies);
            }

            fixedGroup.RateManager = savedRateManager;

            // --- Tear down the GameObject reference + restore global state, then assert. ---
            foreach (var rb in bodies)
                if (rb != null)
                    Object.Destroy(rb.gameObject);
            Object.Destroy(referenceMaterial);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            var refUnload = SceneManager.UnloadSceneAsync(childScene);
            if (refUnload != null)
                while (!refUnload.isDone)
                    yield return null;

            AssertParity(ecsTraj, refTraj, dt, envelope, framesWaited);
        }

        // -----------------------------------------------------------------------------------------------

        /// <summary>
        /// Collect every live <see cref="Rigidbody2D"/> in the additively-loaded child authoring scene, in
        /// the scene's root order, applying the matched determinism preconditions (00c §3): NeverSleep so a
        /// body parking early cannot mask divergence, and a shared friction/restitution
        /// <em>fallback</em> material on any collider that has no authored <c>sharedMaterial</c>.
        /// </summary>
        /// <remarks>
        /// The fallback is applied <em>only when the collider carries no material of its own</em>, so a
        /// fixture that authors a <c>PhysicsMaterial2D</c> (the bounce/friction slices) keeps it on the
        /// reference body — and the ECS bake reads that same authored material (<c>Collider2DBaking.ReadSurface</c>),
        /// so both backends use identical surface properties. When the collider has no material both sides use
        /// the same default (the harness fallback here; the engine's 0.4/0 in the baker), so the free-fall and
        /// contact fixtures that author no material stay matched. The fallback values match the baker's
        /// material-less default (friction 0.4, bounciness 0).
        /// </remarks>
        static List<Rigidbody2D> CollectReferenceBodies(Scene childScene, PhysicsMaterial2D fallbackMaterial)
        {
            var bodies = new List<Rigidbody2D>();
            foreach (var root in childScene.GetRootGameObjects())
            {
                foreach (var rb in root.GetComponentsInChildren<Rigidbody2D>(includeInactive: true))
                {
                    // A STATIC Rigidbody2D is an anchor (a joint's connected body), not a compared body: the
                    // ECS side already excludes static bodies from the compared set (CountNonStatic /
                    // CaptureEcsPoses), so the reference side must too, or the body-count assertion mismatches
                    // and the static anchor would be matched against a non-existent ECS counterpart. A static
                    // body does not move, so dropping it loses no parity signal — its contact/anchor role
                    // still exists on both backends (it is loaded/baked). This is the static-vs-dynamic filter
                    // the harness doc flagged as latent until a fixture carried a static-bodied anchor; the
                    // Phase-2A joint fixtures (a dynamic body jointed to a static anchor) are that fixture.
                    if (rb.bodyType == RigidbodyType2D.Static)
                        continue;
                    rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
                    foreach (var col in rb.GetComponents<Collider2D>())
                        if (col.sharedMaterial == null)
                            col.sharedMaterial = fallbackMaterial;
                    // Apply the serialized velocity seed to the live reference body, from the SAME authored
                    // source the package's InitialVelocity2DBaker reads. Rigidbody2D.linearVelocity is
                    // runtime-only, so the seed cannot live on the Rigidbody2D itself — single authoring is
                    // preserved by both backends reading InitialVelocity2DAuthoring. simulationMode is Script
                    // (set before this load), so no step has run and the velocity persists to the first
                    // Simulate.
                    var seed = rb.GetComponent<InitialVelocity2DAuthoring>();
                    if (seed != null)
                    {
                        rb.linearVelocity = seed.linearVelocity;
                        rb.angularVelocity = seed.angularVelocity;
                    }
                    bodies.Add(rb);
                }
            }
            return bodies;
        }

        static Pose[] CaptureReferencePoses(List<Rigidbody2D> bodies)
        {
            var poses = new Pose[bodies.Count];
            for (var i = 0; i < bodies.Count; i++)
            {
                var rb = bodies[i];
                poses[i] = new Pose
                {
                    position = new float2(rb.position.x, rb.position.y),
                    angleRadians = radians(rb.rotation),
                };
            }
            return poses;
        }

        static Pose[] CaptureEcsPoses(EntityQuery liveQuery, World world)
        {
            // Capture poses for the non-static bodies only, in query order — symmetric with the GameObject
            // reference, which collects only live Rigidbody2D bodies. A baked static floor has body type
            // Static and is skipped, so it is never matched against a (non-existent) reference counterpart.
            using var ltws = liveQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            using var defs = liveQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var poses = new List<Pose>(ltws.Length);
            for (var i = 0; i < ltws.Length; i++)
            {
                if (defs[i].bodyType == Unity.U2D.Physics.PhysicsBody.BodyType.Static)
                    continue;
                var m = ltws[i].Value;
                poses.Add(
                    new Pose
                    {
                        // c3 is the translation column; the upper-left 2x2 is the in-plane rotation.
                        position = new float2(m.c3.x, m.c3.y),
                        // atan2(R10, R00): column-major float4x4, so R00 = c0.x, R10 = c0.y.
                        angleRadians = atan2(m.c0.y, m.c0.x),
                    }
                );
            }
            return poses.ToArray();
        }

        /// <summary>
        /// Count the non-static bodies a query matches. A static body (the floor in a contact fixture) is
        /// excluded from the parity comparison because the GameObject reference, built from live
        /// <see cref="Rigidbody2D"/> bodies, never includes a collider-only static floor.
        /// </summary>
        static int CountNonStatic(EntityQuery query)
        {
            using var defs = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var n = 0;
            for (var i = 0; i < defs.Length; i++)
                if (defs[i].bodyType != Unity.U2D.Physics.PhysicsBody.BodyType.Static)
                    n++;
            return n;
        }

        // -----------------------------------------------------------------------------------------------

        /// <summary>
        /// Match bodies↔entities by initial pose (single-authoring identity), assert the disqualifiers
        /// ALWAYS and the growth-bounded envelope per step, and emit a per-step error table to the log
        /// (the reliable channel under batchmode).
        /// </summary>
        internal static void AssertParity(
            Pose[][] ecsTraj,
            Pose[][] refTraj,
            float dt,
            ParityEnvelope envelope,
            int loadFrames
        )
        {
            var stepCount = ecsTraj.Length;
            var bodyCount = ecsTraj[0].Length;

            // Order both sides by their first-step pose so index i refers to the same authored body on both
            // backends (single authoring => identical initial poses => a stable key).
            var ecsOrder = OrderByInitialPose(ecsTraj[0]);
            var refOrder = OrderByInitialPose(refTraj[0]);

            var log = new System.Text.StringBuilder();
            log.AppendLine(
                $"[PHYSICS2D-PARITY] bodies={bodyCount} steps={stepCount} dt={dt} loadFrames={loadFrames}"
            );
            log.AppendLine(
                $"[PHYSICS2D-PARITY] envelope: posBase={envelope.positionBaseMeters} "
                    + $"posGrowth/step={envelope.positionGrowthPerStep} angCap={envelope.angleCapRadians}"
            );
            log.AppendLine("step\tmaxPosErr\tmeanPosErr\tmaxAngErr\tposBand");

            // Compute the full per-step table FIRST and collect the first violations, then log the table, then
            // assert. A mid-loop Assert.* throws and the trajectory table never reaches the log — exactly the
            // disaggregating-diagnostic gap 04-validation.md hit. Deferring the assertion means a failing run
            // still prints where the chain diverged (free-fall band? contact phase? settle region?).
            var worstPos = 0f;
            var worstAng = 0f;
            string posViolation = null;
            string angViolation = null;
            string nanViolation = null;
            for (var s = 0; s < stepCount; s++)
            {
                var posBand = envelope.positionBaseMeters + envelope.positionGrowthPerStep * (s + 1);
                var maxPos = 0f;
                var sumPos = 0f;
                var maxAng = 0f;
                for (var b = 0; b < bodyCount; b++)
                {
                    var e = ecsTraj[s][ecsOrder[b]];
                    var r = refTraj[s][refOrder[b]];

                    if (
                        nanViolation == null
                        && (
                            isnan(e.position.x) || isnan(e.position.y) || isinf(e.position.x)
                            || isinf(e.position.y) || isnan(e.angleRadians) || isinf(e.angleRadians)
                        )
                    )
                        nanViolation =
                            $"ECS body {b} produced NaN/Inf at step {s}: pos={e.position}, "
                            + $"ang={e.angleRadians}.";

                    var dp = length(e.position - r.position);
                    var da = abs(AngleDelta(e.angleRadians, r.angleRadians));
                    maxPos = max(maxPos, dp);
                    sumPos += dp;
                    maxAng = max(maxAng, da);

                    if (posViolation == null && dp > posBand)
                        posViolation =
                            $"Position parity broke at step {s}, body {b}: |ECS - GameObject| = {dp} m "
                            + $"exceeds the growth-bounded band {posBand} m. ECS={e.position}, "
                            + $"GameObject={r.position}.";
                    if (angViolation == null && da > envelope.angleCapRadians)
                        angViolation =
                            $"Angle parity broke at step {s}, body {b}: |ECS - GameObject| = {da} rad "
                            + $"exceeds the cap {envelope.angleCapRadians} rad. ECS={e.angleRadians}, "
                            + $"GameObject={r.angleRadians}.";
                }
                worstPos = max(worstPos, maxPos);
                worstAng = max(worstAng, maxAng);
                log.AppendLine(
                    $"{s}\t{maxPos:E6}\t{(sumPos / bodyCount):E6}\t{maxAng:E6}\t{posBand:E6}"
                );
            }

            // Disqualifiers on the final state: each body moved, settles in the coarse region.
            string travelViolation = null;
            string settleViolation = null;
            for (var b = 0; b < bodyCount; b++)
            {
                var start = ecsTraj[0][ecsOrder[b]].position;
                var end = ecsTraj[stepCount - 1][ecsOrder[b]].position;
                var travel = length(end - start);
                if (travelViolation == null && travel < envelope.minTravelMeters)
                    travelViolation =
                        $"Body {b} barely moved ({travel} m < {envelope.minTravelMeters} m) — a silently "
                        + "no-op bake/create (the body exists but never integrated).";
                if (
                    settleViolation == null
                    && !(
                        end.x >= envelope.settleRegionMin.x && end.x <= envelope.settleRegionMax.x
                        && end.y >= envelope.settleRegionMin.y && end.y <= envelope.settleRegionMax.y
                    )
                )
                    settleViolation =
                        $"Body {b} ended outside the expected region: end={end}, "
                        + $"region=[{envelope.settleRegionMin}..{envelope.settleRegionMax}].";
            }

            log.AppendLine($"[PHYSICS2D-PARITY] WORST_POS_ERR={worstPos:E6} WORST_ANG_ERR={worstAng:E6}");
            if (nanViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY] NAN: {nanViolation}");
            if (posViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY] POS: {posViolation}");
            if (angViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY] ANG: {angViolation}");
            if (travelViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY] TRAVEL: {travelViolation}");
            if (settleViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY] SETTLE: {settleViolation}");
            Debug.Log(log.ToString());

            // Disqualifiers first (solver-independent correctness), then the envelope band.
            Assert.IsNull(nanViolation, nanViolation);
            Assert.IsNull(travelViolation, travelViolation);
            Assert.IsNull(settleViolation, settleViolation);
            Assert.IsNull(posViolation, posViolation);
            Assert.IsNull(angViolation, angViolation);
        }

        /// <summary>
        /// Index permutation sorting body slots by initial pose: ascending y, then ascending x. Stable,
        /// backend-independent because both sides share the authored initial transform.
        /// </summary>
        static int[] OrderByInitialPose(Pose[] firstStep)
        {
            var order = new int[firstStep.Length];
            for (var i = 0; i < order.Length; i++)
                order[i] = i;
            System.Array.Sort(
                order,
                (a, b) =>
                {
                    var pa = firstStep[a].position;
                    var pb = firstStep[b].position;
                    var cy = pa.y.CompareTo(pb.y);
                    return cy != 0 ? cy : pa.x.CompareTo(pb.x);
                }
            );
            return order;
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
