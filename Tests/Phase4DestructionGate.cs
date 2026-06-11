using System.Collections;
using NUnit.Framework;
using Unity.Collections;
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
    /// The independent adversarial e2e gate for Phase 4 (per-entity Box2D body teardown on despawn). It is
    /// built to FALSIFY the Phase-4 invariant — that destroying an entity frees its Box2D body, that body's
    /// shapes, and any joint attached to it, so native bodies stay bounded under churn and a dead entity is
    /// neither simulated nor written back — by probing the system's observable decision points rather than the
    /// happy path the smoke (<see cref="BodyDestructionValidation"/>) already covers. The witnesses are the
    /// authoritative NATIVE counts: <c>PhysicsWorld.GetBodies</c> (XML
    /// <c>UnityEngine.PhysicsCore2DModule.xml:15311</c>, "all the active PhysicsBody in the specified world")
    /// and <c>PhysicsWorld.GetJoints</c> (XML <c>:15370</c>, "all the active PhysicsJoint in the specified
    /// world") — not an ECS-side proxy, because the leak is native resources that ECS already forgot about.
    /// </summary>
    /// <remarks>
    /// Each test runs in its OWN dedicated, disposable <see cref="World"/> (never the default injection world),
    /// holding the package's five FixedStep systems — including <see cref="PhysicsJoint2DCreationSystem"/>,
    /// which the smoke's helper omits — inside a <see cref="FixedStepSimulationSystemGroup"/> swapped to a
    /// <c>FixedRateSimpleManager(1/60)</c> so one <c>group.Update()</c> is exactly one fixed step. A test that
    /// threw mid-run would otherwise leak native bodies into a shared world and poison its siblings; the
    /// per-test disposable world plus <c>world.Dispose()</c> (which destroys the owning <see cref="PhysicsWorld"/>
    /// and everything in it) makes a failure self-contained. The coroutines yield <c>null</c> only (never
    /// <c>WaitForEndOfFrame</c>, which does not tick in batchmode). No Burst/Jobs code is authored here — every
    /// probe drives <c>group.Update()</c> and reads native counts/poses on the main thread, so the
    /// <c>docs/unity/{jobs,burst}</c> binding (which gates Burst/Jobs authoring) does not apply to this file;
    /// the only Burst job in the path is the runtime's own write-back job, exercised through the systems.
    /// </remarks>
    public sealed class Phase4DestructionGate
    {
        const float Dt = 1f / 60f;

        // A fresh world holding ALL FIVE package FixedStep systems (the smoke helper omits the joint creation
        // system; the jointed-despawn probes need it). Declared-order sorting via the [UpdateInGroup]/
        // [UpdateBefore]/[UpdateAfter] attributes.
        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("Physics2DPhase4GateWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsJoint2DCreationSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }

        // The native PhysicsWorld this ECS world owns (or default if not yet created).
        static PhysicsWorld GetWorld(EntityManager em)
        {
            var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            if (q.IsEmpty)
                return default;
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        // Live native body count straight from the Box2D world — the authoritative leak witness.
        static int LiveBodyCount(EntityManager em)
        {
            var world = GetWorld(em);
            if (!world.isValid)
                return 0;
            using var bodies = world.GetBodies(Allocator.Temp);
            return bodies.Length;
        }

        // Live native joint count straight from the Box2D world — the authoritative dangling-joint witness.
        static int LiveJointCount(EntityManager em)
        {
            var world = GetWorld(em);
            if (!world.isValid)
                return 0;
            using var joints = world.GetJoints(Allocator.Temp);
            return joints.Length;
        }

        static Entity AuthorDynamicCircle(EntityManager em, float2 pos)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = pos,
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
        }

        // ---------------------------------------------------------------------------------------------------
        // DECISION POINT 1 — Churn / monotonic growth. Spawn N dynamic bodies, destroy ALL, repeat over many
        // cycles; the native live body count must return to a constant baseline (0) each cycle and never grow.
        // A one-body-per-cycle leak over Cycles cycles would be unmistakable in the final count. This is the
        // input the implementer's single-body smoke did NOT exercise: sustained churn, where a contiguous /
        // index-aligned body-array assumption or a per-cycle off-by-one would accumulate.
        // ---------------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator Churn_ManyCyclesOfSpawnDestroyAll_BodyCountReturnsToBaseline_NeverGrows()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            const int Cycles = 50;
            const int N = 20;

            // The world is created lazily on the first update; baseline is zero bodies.
            group.Update();
            var baseline = LiveBodyCount(em);
            Assert.AreEqual(0, baseline, $"Pre-churn baseline body count was {baseline}, expected 0.");

            var peak = 0;
            for (var c = 0; c < Cycles; c++)
            {
                var entities = new Entity[N];
                for (var i = 0; i < N; i++)
                    entities[i] = AuthorDynamicCircle(em, new float2((i - N / 2) * 1.5f, 10f + c));

                // Frame A: bodies created (no step on the creation frame).
                group.Update();
                var afterCreate = LiveBodyCount(em);
                Assert.AreEqual(
                    baseline + N,
                    afterCreate,
                    $"Cycle {c}: expected {baseline + N} live bodies after creating {N}, found "
                        + $"{afterCreate}. Either a previous cycle leaked or creation under-produced."
                );
                peak = max(peak, afterCreate);

                // Step a couple of times so the bodies are genuinely simulated, then destroy them all.
                group.Update();
                group.Update();
                for (var i = 0; i < N; i++)
                    em.DestroyEntity(entities[i]);

                // Frame B: cleanup runs before the step and frees every ghost body.
                group.Update();
                var afterDestroy = LiveBodyCount(em);
                Assert.AreEqual(
                    baseline,
                    afterDestroy,
                    $"Cycle {c}: body count did not return to baseline {baseline} after destroying all "
                        + $"{N}; found {afterDestroy}. A per-cycle leak grows linearly — this is the leak."
                );
            }

            var finalCount = LiveBodyCount(em);
            Assert.AreEqual(
                baseline,
                finalCount,
                $"After {Cycles} churn cycles of {N} bodies the live body count is {finalCount}, not the "
                    + $"baseline {baseline}. Monotonic growth = the per-entity body leak Phase 4 must close."
            );

            Debug.Log(
                $"[PHYSICS2D-P4GATE] CHURN: {Cycles} cycles x {N} bodies, peak live={peak}, "
                    + $"final live={finalCount} (baseline {baseline}). Bounded — no growth."
            );

            world.Dispose();
            yield break;
        }

        // ---------------------------------------------------------------------------------------------------
        // DECISION POINT 2a — Jointed-body despawn, OWNER side (Box2D bodyB, the entity carrying the
        // PhysicsJoint2D handle). Build a hinge between two concrete dynamic bodies (NOT the null-connected
        // world-anchor form, so the body baseline is exactly the two bodies with no implicit anchor),
        // destroy the joint OWNER, and assert: native joint count returns to baseline (the body cascade freed
        // the joint), the owner's body is freed, no exception. Connected body survives.
        // ---------------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator JointedDespawn_DestroyOwnerBody_FreesJointAndBody_NoDangle()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // bodyA (connected) above, bodyB (owner) below — a concrete two-body hinge, no world anchor.
            var connected = AuthorDynamicCircle(em, new float2(0f, 8f));
            var owner = AuthorDynamicCircle(em, new float2(0f, 6f));
            var jointEntity = MakeHingeOwner(em, owner, connected, anchorOnOwner: new float2(0f, 1f));

            // Update 1: both bodies created (creation frame, no step), no joint yet (joint creation defers
            // until both have a live PhysicsBody2D, which appears only after this update's ECB playback).
            group.Update();
            // Update 2: joint creation now sees both bodies and creates the hinge; the step honours it.
            group.Update();
            // Update 3: settle, confirm the joint actually exists natively.
            group.Update();

            Assert.AreEqual(
                2,
                LiveBodyCount(em),
                "Expected exactly 2 live bodies (concrete two-body hinge, no world anchor)."
            );
            Assert.AreEqual(
                1,
                LiveJointCount(em),
                "The hinge joint was not created natively — joint-creation deferral never resolved."
            );
            Assert.IsTrue(
                em.HasComponent<PhysicsJoint2D>(jointEntity),
                "The joint-owner entity never gained its PhysicsJoint2D handle."
            );
            var jointHandle = em.GetComponentData<PhysicsJoint2D>(jointEntity).joint;
            Assert.IsTrue(jointHandle.isValid, "The created joint handle reads invalid.");

            // Destroy the joint OWNER (which also carries the PhysicsJoint2D). Its body's cascade must free
            // the joint. (The owner entity == the joint entity here: a hinge owner IS bodyB.)
            em.DestroyEntity(owner);

            // One update: cleanup frees the owner's ghost body, cascading to the joint.
            group.Update();

            Assert.IsFalse(jointHandle.isValid, "The joint survived the owner body's destruction (dangling).");
            Assert.AreEqual(
                0,
                LiveJointCount(em),
                "Native joint count did not return to baseline after destroying a jointed body — a stale "
                    + "joint remains in the world."
            );
            Assert.AreEqual(
                1,
                LiveBodyCount(em),
                "Expected exactly 1 live body (the connected survivor) after destroying the owner."
            );
            Assert.IsTrue(
                em.Exists(connected) && em.HasComponent<PhysicsBody2D>(connected),
                "The connected (surviving) body was wrongly destroyed by the cascade."
            );

            // The survivor must keep simulating sanely (no NaN), proving no broken-constraint poison. Read the
            // pose through LocalToWorld (the write-back path the runtime already exercises), not the native
            // PhysicsBody.position accessor — keeping every API call to ones shipping code already uses.
            for (var f = 0; f < 30; f++)
                group.Update();
            var survPos = em.GetComponentData<LocalToWorld>(connected).Position;
            Assert.IsFalse(
                isnan(survPos.x) || isnan(survPos.y) || isinf(survPos.x) || isinf(survPos.y),
                $"Surviving body produced NaN/Inf after the joint partner was despawned: {survPos}."
            );
            Assert.AreEqual(
                0,
                LiveJointCount(em),
                "A joint reappeared after the owner despawn — the dangling joint was re-stepped."
            );

            Debug.Log(
                "[PHYSICS2D-P4GATE] JOINT-OWNER DESPAWN: 2 bodies+1 joint → destroy owner → "
                    + "joint freed (invalid), live joints 1→0, live bodies 2→1, survivor sane."
            );

            world.Dispose();
            yield break;
        }

        // ---------------------------------------------------------------------------------------------------
        // DECISION POINT 2b — Jointed-body despawn, CONNECTED side (Box2D bodyA, the entity that carries NO
        // PhysicsJoint2D handle at all — the joint reference lives only on the OWNER). This is the case the
        // impl explicitly escalated as XML-asserted-but-not-probed: does destroying a body that participates
        // in a joint only as the *connected* partner still cascade-free the joint? The connected entity's
        // cleanup component holds only ITS OWN body handle; the joint handle is nowhere on it. If the cascade
        // is real, GetJoints() drops to baseline; if the XML over-promised, a stale joint lingers on the
        // surviving owner. Also probes the dangling ECS PhysicsJoint2D the owner keeps (an invalid handle):
        // the owner must keep simulating with no exception.
        // ---------------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator JointedDespawn_DestroyConnectedBody_CascadeFreesJoint_OwnerSurvivesSane()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var connected = AuthorDynamicCircle(em, new float2(0f, 8f));
            var owner = AuthorDynamicCircle(em, new float2(0f, 6f));
            var jointEntity = MakeHingeOwner(em, owner, connected, anchorOnOwner: new float2(0f, 1f));

            group.Update(); // create bodies
            group.Update(); // create joint + first step
            group.Update(); // settle

            Assert.AreEqual(2, LiveBodyCount(em), "Expected 2 live bodies before the connected despawn.");
            Assert.AreEqual(1, LiveJointCount(em), "Expected 1 live joint before the connected despawn.");
            var jointHandle = em.GetComponentData<PhysicsJoint2D>(jointEntity).joint;
            Assert.IsTrue(jointHandle.isValid, "Joint handle invalid before the connected despawn.");

            // Destroy the CONNECTED body (bodyA). Its entity has no joint reference; only its own body handle
            // rides in its cleanup component. The joint must die via the body-cascade from the bodyA side.
            em.DestroyEntity(connected);

            group.Update(); // cleanup frees the connected ghost body; cascade must take the joint with it

            Assert.IsFalse(
                jointHandle.isValid,
                "The joint survived destruction of its CONNECTED body — the PhysicsBody.DestroyBatch cascade "
                    + "does NOT free a joint from the bodyA side, contradicting the XML cascade claim. This is "
                    + "the escalated dangling-joint risk."
            );
            Assert.AreEqual(
                0,
                LiveJointCount(em),
                "Native joint count did not return to baseline after destroying the connected body — a "
                    + "dangling joint remains attached to the surviving owner."
            );
            Assert.AreEqual(
                1,
                LiveBodyCount(em),
                "Expected exactly 1 live body (the surviving owner) after destroying the connected body."
            );

            // The OWNER survives, still carrying a now-INVALID PhysicsJoint2D handle (its joint reference was
            // never cleaned up — the cleanup component only tracks bodies). It must keep simulating: now
            // unconstrained, it free-falls. No exception, no NaN, no joint resurrection.
            Assert.IsTrue(
                em.Exists(owner) && em.HasComponent<PhysicsBody2D>(owner),
                "The surviving owner body was wrongly destroyed."
            );
            var ownerYBefore = em.GetComponentData<LocalToWorld>(owner).Position.y;
            for (var f = 0; f < 60; f++)
                group.Update();
            var ownerPos = em.GetComponentData<LocalToWorld>(owner).Position;
            Assert.IsFalse(
                isnan(ownerPos.x) || isnan(ownerPos.y) || isinf(ownerPos.x) || isinf(ownerPos.y),
                $"Surviving owner produced NaN/Inf after its joint partner was despawned: {ownerPos}."
            );
            Assert.Less(
                ownerPos.y,
                ownerYBefore - 0.2f,
                $"Surviving owner did not free-fall after its joint partner (and joint) were freed — it is "
                    + $"still constrained by a ghost joint. yBefore={ownerYBefore}, yAfter={ownerPos.y}."
            );
            Assert.AreEqual(
                0,
                LiveJointCount(em),
                "A joint reappeared after the connected despawn — the dangling joint was re-stepped."
            );

            Debug.Log(
                "[PHYSICS2D-P4GATE] JOINT-CONNECTED DESPAWN: destroy bodyA (connected, no joint ref) → "
                    + "cascade freed joint (invalid), live joints 1→0, owner survives & free-falls (now "
                    + "unconstrained), no exception."
            );

            world.Dispose();
            yield break;
        }

        // ---------------------------------------------------------------------------------------------------
        // DECISION POINT 3 — Partial despawn / index fragmentation. Spawn a batch of N bodies via the bulk
        // CreateBodyBatch path (the one the impl flagged for a possible contiguous/index-aligned body-array
        // assumption), destroy a SUBSET (every other body), and assert: the survivors keep simulating with
        // correct, finite, distinct poses that track each survivor's own body handle, and the live body count
        // equals the survivor count. Reading each survivor's pose straight from its native PhysicsBody.position
        // (not the ECS LocalToWorld) pins that destruction did not shuffle the body↔entity correspondence
        // (which a fragmented index-aligned array would do — a survivor would read a freed/other body's pose).
        // ---------------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator PartialDespawn_DestroySubsetOfBatch_SurvivorsKeepCorrectPoses_NoFragmentation()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // Spray N IDENTICAL-form dynamic circle bodies in one frame. With the cache default on (threshold 8),
            // the form crosses the threshold and the same-frame run is created through the in-frame CreateBodyBatch
            // collapse — the path that replaced the removed PhysicsBody2DBatchRequest. This exercises exactly the
            // possible contiguous/index-aligned body-array assumption the destruction probe is here to pin.
            const int N = 24;
            var formKey = new Unity.Mathematics.uint4(0xDEADu, 0xBEEFu, 0xC0FFu, 0xEE00u);
            for (var i = 0; i < N; i++)
            {
                var x = -8f + (16f * i / (N - 1));
                var e = DirectPhysics2DAuthoring.Create(
                    em,
                    new PhysicsBody2DDefinition
                    {
                        bodyType = PhysicsBody.BodyType.Dynamic,
                        gravityScale = 1f,
                        initialPosition = new float2(x, 20f + (i % 5)),
                        useAutoMass = true,
                    },
                    new PhysicsShape2D
                    {
                        kind = PhysicsShape2DKind.Circle,
                        radius = 0.25f,
                        density = 1f,
                        friction = 0.4f,
                    }
                );
                em.AddComponentData(e, new PhysicsBody2DFormHash { value = formKey });
            }

            group.Update(); // creates N bodies (the same-frame collapse fires for the run past the threshold)
            Assert.AreEqual(N, LiveBodyCount(em), $"Spray did not create {N} live bodies.");

            // Step a few times so the bodies have moved apart from their spawn poses (genuinely simulated).
            for (var f = 0; f < 10; f++)
                group.Update();

            // Collect every sprayed body entity (PhysicsBody2D + LocalToWorld), pair each with its native handle.
            var batchQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var entities = batchQuery.ToEntityArray(Allocator.Temp);
            Assert.AreEqual(N, entities.Length, "Sprayed entity count mismatch before partial despawn.");

            // Destroy every OTHER entity (the odd indices). The survivors are the even indices.
            var survivorEntities = new System.Collections.Generic.List<Entity>();
            var survivorHandles = new System.Collections.Generic.List<PhysicsBody>();
            var destroyedHandles = new System.Collections.Generic.List<PhysicsBody>();
            for (var i = 0; i < entities.Length; i++)
            {
                var h = em.GetComponentData<PhysicsBody2D>(entities[i]).body;
                if ((i & 1) == 1)
                {
                    destroyedHandles.Add(h);
                    em.DestroyEntity(entities[i]);
                }
                else
                {
                    survivorEntities.Add(entities[i]);
                    survivorHandles.Add(h);
                }
            }
            entities.Dispose();
            var expectedSurvivors = survivorEntities.Count;

            // One update: cleanup frees exactly the destroyed subset's bodies before the step.
            group.Update();

            Assert.AreEqual(
                expectedSurvivors,
                LiveBodyCount(em),
                $"After destroying a subset, live body count is {LiveBodyCount(em)}, expected "
                    + $"{expectedSurvivors}. Destruction freed the wrong count (over- or under-free)."
            );

            // Every destroyed body's handle is now invalid; every survivor's handle is still valid — the
            // destruction freed exactly the chosen subset, not a fragmented slot range.
            foreach (var h in destroyedHandles)
                Assert.IsFalse(h.isValid, "A destroyed batch body's handle is still valid (not freed).");
            foreach (var h in survivorHandles)
                Assert.IsTrue(h.isValid, "A survivor batch body's handle went invalid (wrongly freed).");

            // Snapshot each survivor's pose immediately after the partial despawn (via LocalToWorld — the
            // write-back path the runtime already drives), then step ONE fixed step. The body↔entity
            // correspondence must hold: a survivor's pose may move only the small physically-plausible amount
            // one gravity step produces. A fragmented index-aligned write-back array would scatter a survivor's
            // LocalToWorld onto a different (or freed) body's pose — a large, non-physical jump this catches.
            // float2[] must be fully qualified: `using static …math` makes the bare token `float2` resolve to
            // the math.float2(...) method, so `new float2[N]` is ambiguous (CS0119). The type is unambiguous
            // fully qualified.
            var survivorPoseAfterDespawn = new Unity.Mathematics.float2[survivorEntities.Count];
            for (var s = 0; s < survivorEntities.Count; s++)
                survivorPoseAfterDespawn[s] = em.GetComponentData<LocalToWorld>(survivorEntities[s]).Position.xy;

            group.Update();
            for (var s = 0; s < survivorEntities.Count; s++)
            {
                var e = survivorEntities[s];
                Assert.IsTrue(em.Exists(e), $"Survivor entity {s} was wrongly reclaimed.");
                Assert.IsTrue(
                    em.GetComponentData<PhysicsBody2D>(e).body.isValid,
                    $"Survivor {s}'s body handle is invalid after stepping (wrongly freed)."
                );
                var p = em.GetComponentData<LocalToWorld>(e).Position.xy;
                Assert.IsFalse(
                    isnan(p.x) || isnan(p.y) || isinf(p.x) || isinf(p.y),
                    $"Survivor {s} produced NaN/Inf after the partial despawn: {p}."
                );
                // One 1/60 s gravity step moves a free body well under 0.5 m; a fragmented alias to another
                // body would jump it metres. The continuity bound pins the correspondence held.
                var jump = length(p - survivorPoseAfterDespawn[s]);
                Assert.Less(
                    jump,
                    0.5f,
                    $"Survivor {s} teleported {jump} m in one step after the partial despawn — its "
                        + "LocalToWorld was scattered onto a different body (index fragmentation)."
                );
            }

            // Continue simulating; the live count and survivor validity must stay stable.
            for (var f = 0; f < 40; f++)
                group.Update();
            for (var s = 0; s < survivorEntities.Count; s++)
            {
                Assert.IsTrue(em.Exists(survivorEntities[s]), $"Survivor {s} lost after extended stepping.");
                var p = em.GetComponentData<LocalToWorld>(survivorEntities[s]).Position;
                Assert.IsFalse(
                    isnan(p.x) || isnan(p.y) || isinf(p.x) || isinf(p.y),
                    $"Survivor {s} produced NaN/Inf after extended stepping: {p}."
                );
            }

            Assert.AreEqual(
                expectedSurvivors,
                LiveBodyCount(em),
                "Live body count drifted after stepping the survivors — destruction is not stable."
            );

            Debug.Log(
                $"[PHYSICS2D-P4GATE] PARTIAL DESPAWN: batch {N} → destroyed {destroyedHandles.Count}, "
                    + $"{expectedSurvivors} survivors keep their own valid handles + finite poses, live "
                    + $"count = {expectedSurvivors}. No fragmentation."
            );

            world.Dispose();
            yield break;
        }

        // ---------------------------------------------------------------------------------------------------
        // DECISION POINT 4 — Destroy-same-step-as-create. An entity authored and then destroyed within the
        // SAME group-update window. Two sub-cases:
        //   (i) destroyed BEFORE its body is ever created (no PhysicsBody2D / no cleanup component yet): must
        //       leave zero bodies and never create one for a dead entity.
        //   (ii) created on update A, destroyed before update B: the standard one-step teardown, but interleaved
        //       with a fresh spawn on the same update so a created-and-freed pair cannot net-leak.
        // The world's live body count must equal only the genuinely-alive entities at each point.
        // ---------------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator DestroySameStepAsCreate_NoGhostBodyEverSimulated()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            group.Update();
            Assert.AreEqual(0, LiveBodyCount(em), "Baseline not zero.");

            // (i) Author then immediately destroy BEFORE any update creates a body. No PhysicsBody2D and no
            // cleanup component were ever added, so there is nothing to free and no body must appear.
            var stillborn = AuthorDynamicCircle(em, new float2(0f, 10f));
            Assert.IsFalse(em.HasComponent<PhysicsBody2D>(stillborn), "Body created too eagerly (before step).");
            em.DestroyEntity(stillborn);
            group.Update();
            Assert.AreEqual(
                0,
                LiveBodyCount(em),
                "A body was created and/or leaked for an entity destroyed before its body-creation step."
            );

            // (ii) Author a body that lives, plus on the SAME update destroy one created the previous update.
            var keep = AuthorDynamicCircle(em, new float2(-2f, 10f));
            var transient = AuthorDynamicCircle(em, new float2(2f, 10f));
            group.Update(); // both created
            Assert.AreEqual(2, LiveBodyCount(em), "Expected 2 live bodies after creating keep + transient.");

            // Destroy the transient and spawn a brand-new one in the SAME window before the next update, so the
            // cleanup of the transient and the creation of the newcomer are processed in one group.Update().
            em.DestroyEntity(transient);
            var newcomer = AuthorDynamicCircle(em, new float2(4f, 10f));
            group.Update(); // transient ghost freed (cleanup) AND newcomer created, in one update

            Assert.AreEqual(
                2,
                LiveBodyCount(em),
                "Interleaved destroy+create in one update did not net to 2 live bodies (keep + newcomer) — "
                    + "either the transient leaked or the newcomer was not created."
            );
            Assert.IsTrue(em.Exists(keep) && em.HasComponent<PhysicsBody2D>(keep), "keep was lost.");
            Assert.IsTrue(em.Exists(newcomer) && em.HasComponent<PhysicsBody2D>(newcomer), "newcomer was lost.");
            Assert.IsFalse(em.Exists(transient), "transient ghost was not reclaimed.");

            // A few more steps must keep the count steady at 2.
            for (var f = 0; f < 20; f++)
                group.Update();
            Assert.AreEqual(2, LiveBodyCount(em), "Live count drifted after interleaved churn.");

            Debug.Log(
                "[PHYSICS2D-P4GATE] SAME-STEP CREATE/DESTROY: stillborn left 0 bodies; interleaved "
                    + "destroy+create netted to 2 live (keep + newcomer), transient reclaimed."
            );

            world.Dispose();
            yield break;
        }

        // ---------------------------------------------------------------------------------------------------
        // DECISION POINT 5 — Ghost stepping & write-back. After an entity is destroyed, its LocalToWorld must
        // STOP being updated (a ghost is never written back) and its body must not be integrated. Capture a
        // survivor's and the soon-dead entity's LocalToWorld, destroy one, and confirm: (a) the dead entity is
        // gone (no lingering LocalToWorld write because the entity itself is reclaimed), (b) the survivor keeps
        // being written back (its LocalToWorld advances), (c) the live body count drops by exactly one and the
        // dead body's native pose handle is invalid (cannot be stepped).
        // ---------------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator GhostStepping_DestroyedEntityNotWrittenBackNorStepped()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var survivor = AuthorDynamicCircle(em, new float2(-3f, 12f));
            var doomed = AuthorDynamicCircle(em, new float2(3f, 12f));

            group.Update(); // create
            group.Update(); // step once so write-back has run on live bodies
            group.Update();

            Assert.AreEqual(2, LiveBodyCount(em), "Expected 2 live bodies pre-destroy.");
            var doomedHandle = em.GetComponentData<PhysicsBody2D>(doomed).body;
            var survivorLtwBefore = em.GetComponentData<LocalToWorld>(survivor).Position;

            em.DestroyEntity(doomed);
            group.Update(); // cleanup frees the doomed body before the step

            // The doomed entity is fully reclaimed — there is no entity left to write a LocalToWorld onto, so a
            // ghost cannot keep updating a transform; and its native body is freed (cannot be stepped).
            Assert.IsFalse(em.Exists(doomed), "The doomed entity was not reclaimed (ghost lingers).");
            Assert.IsFalse(doomedHandle.isValid, "The doomed body is still valid — it would keep being stepped.");
            Assert.AreEqual(1, LiveBodyCount(em), "Live body count did not drop to exactly 1 after one destroy.");

            // The survivor IS still written back: its LocalToWorld advances (falls) across further steps, so the
            // write-back system kept running on the live set after the ghost was removed from it.
            for (var f = 0; f < 30; f++)
                group.Update();
            var survivorLtwAfter = em.GetComponentData<LocalToWorld>(survivor).Position;
            Assert.Less(
                survivorLtwAfter.y,
                survivorLtwBefore.y - 0.2f,
                $"The survivor's LocalToWorld did not advance after the ghost despawn — write-back stalled. "
                    + $"yBefore={survivorLtwBefore.y}, yAfter={survivorLtwAfter.y}."
            );
            Assert.IsFalse(
                isnan(survivorLtwAfter.x) || isnan(survivorLtwAfter.y),
                $"Survivor LocalToWorld went NaN after the ghost despawn: {survivorLtwAfter}."
            );
            Assert.AreEqual(1, LiveBodyCount(em), "A body reappeared after the ghost despawn (ghost re-stepped).");

            Debug.Log(
                "[PHYSICS2D-P4GATE] GHOST STEP: destroyed entity reclaimed + body freed (not stepped); "
                    + "survivor still written back (LocalToWorld advanced), live bodies 2→1."
            );

            world.Dispose();
            yield break;
        }

        // ---------------------------------------------------------------------------------------------------
        // DECISION POINT 6 — Null-connected (world-anchor) jointed despawn. A joint whose connectedBody is
        // Entity.Null creates a shared STATIC world-anchor body (PhysicsJoint2DCreationSystem.EnsureWorldAnchor),
        // so the live body count includes that anchor. Destroying the single dynamic owner must free the
        // dynamic body and the joint, leaving ONLY the persistent static anchor (which the package keeps for the
        // world's lifetime). This probes that the anchor is correctly NOT swept by per-entity cleanup (it has no
        // entity) and the joint count returns to zero.
        // ---------------------------------------------------------------------------------------------------
        [UnityTest]
        public IEnumerator NullConnectedJointDespawn_FreesOwnerAndJoint_AnchorPersists()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var owner = AuthorDynamicCircle(em, new float2(1f, 5f));
            // connectedBody = Entity.Null → joint to a world point; the system supplies a static anchor.
            var jointEntity = MakeHingeOwner(em, owner, Entity.Null, anchorOnOwner: new float2(0f, 0f));

            group.Update(); // create owner body
            group.Update(); // joint creation makes the world anchor + the hinge, then steps
            group.Update();

            var bodiesWithAnchor = LiveBodyCount(em);
            // 1 dynamic owner + 1 static world anchor = 2.
            Assert.AreEqual(
                2,
                bodiesWithAnchor,
                $"Expected 2 live bodies (owner + world anchor) for a null-connected joint, found "
                    + $"{bodiesWithAnchor}."
            );
            Assert.AreEqual(1, LiveJointCount(em), "The null-connected hinge was not created.");
            var jointHandle = em.GetComponentData<PhysicsJoint2D>(jointEntity).joint;
            Assert.IsTrue(jointHandle.isValid, "Null-connected joint handle invalid.");

            em.DestroyEntity(owner);
            group.Update(); // cleanup frees the owner body, cascading to the joint

            Assert.IsFalse(jointHandle.isValid, "The joint survived the owner's destruction (dangling).");
            Assert.AreEqual(0, LiveJointCount(em), "Native joint count did not return to baseline.");
            // The static world anchor PERSISTS (it has no entity, is not swept by per-entity cleanup).
            Assert.AreEqual(
                1,
                LiveBodyCount(em),
                "Expected exactly 1 live body (the persistent static world anchor) after destroying the "
                    + "owner — the anchor must not be freed, and the owner must be."
            );

            // Stepping must not resurrect a joint nor grow the body count beyond the lone anchor.
            for (var f = 0; f < 20; f++)
                group.Update();
            Assert.AreEqual(0, LiveJointCount(em), "A joint reappeared after the null-connected owner despawn.");
            Assert.AreEqual(1, LiveBodyCount(em), "Body count changed after the owner despawn (anchor unstable).");

            Debug.Log(
                "[PHYSICS2D-P4GATE] NULL-CONNECTED JOINT DESPAWN: owner+anchor+joint → destroy owner → "
                    + "joint freed, owner freed, static world anchor persists (live bodies 2→1, joints 1→0)."
            );

            world.Dispose();
            yield break;
        }

        // Author a hinge-joint owner entity (bodyB) connected to `connected` (bodyA; Entity.Null = world).
        static Entity MakeHingeOwner(EntityManager em, Entity owner, Entity connected, float2 anchorOnOwner)
        {
            em.AddComponentData(
                owner,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Hinge,
                    connectedBody = connected,
                    anchor = anchorOnOwner,
                    connectedAnchor = new float2(0f, 0f),
                    collideConnected = false,
                }
            );
            return owner;
        }
    }
}
