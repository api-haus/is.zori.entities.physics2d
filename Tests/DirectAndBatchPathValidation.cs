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
    /// The low-level direct surface and the cached-body-template creation optimisation, exercised with NO
    /// MonoBehaviour and NO subscene bake — bodies are authored straight onto entities from code and driven through
    /// the same step + write-back systems. Three concerns:
    /// <list type="bullet">
    /// <item><b>Direct authoring</b> (<see cref="DirectPhysics2DAuthoring"/>): set
    /// <see cref="PhysicsBody2DDefinition"/> + <see cref="PhysicsShape2D"/> on entities; the per-entity creation
    /// loop turns each into a live body.</item>
    /// <item><b>Transparency</b>: a body created through the cached-template path (optimisation ON, past the
    /// threshold) is BIT-IDENTICAL in simulated pose to one created through the per-entity path (optimisation
    /// OFF). Enabling vs disabling the optimisation, and any threshold N, yields the same simulation.</item>
    /// <item><b>Cross-frame spray</b>: instantiating one identical body per frame (the falling-sand pattern, the
    /// workload the cross-frame cache is built for) creates each body correctly and immediately physical.</item>
    /// </list>
    /// The optimisation only changes HOW bodies are created, never the RESULT — that is what these assert.
    /// </summary>
    /// <remarks>
    /// Each test runs in a DEDICATED, disposable <see cref="World"/> — not the default injection world — so the
    /// live Box2D bodies and the owning <c>PhysicsWorld</c> are torn down on <c>world.Dispose()</c> and leave zero
    /// residue for the next PlayMode test. The isolated world holds the package's three FixedStep systems plus a
    /// <see cref="FixedStepSimulationSystemGroup"/> with a per-step rate manager, so one <c>group.Update()</c> is
    /// exactly one fixed step. The <see cref="PhysicsWorld2DConfig"/> singleton, when present, is created BEFORE the
    /// first update so <c>PhysicsWorld2DSystem</c> reads it at world creation; it carries the
    /// <c>cacheIdenticalBodies</c> / <c>identicalBodyThreshold</c> knobs under test. The form hash is set directly
    /// on the authored entities (a baked prefab would carry it from <c>PhysicsBody2DFormHashBakingSystem</c>); its
    /// value only needs to be equal across identical forms — the runtime rebuilds the template from the donor's
    /// real components, so any equal key is a valid grouping key. No <c>WaitForEndOfFrame</c> (it does not tick in
    /// batchmode); coroutines yield <c>null</c> only.
    /// </remarks>
    public sealed class DirectAndBatchPathValidation
    {
        const float Dt = 1f / 60f;
        const int Steps = 90;

        // A fixed, arbitrary form hash stamped on every identical authored body in a test. Its value is irrelevant
        // to correctness (the runtime rebuilds the template from the donor's components) — it only has to be EQUAL
        // across bodies the test means to be one form, and DIFFERENT across forms.
        static readonly uint4 FormKeyA = new uint4(0x1111_1111u, 0x2222_2222u, 0x3333_3333u, 0x4444_4444u);

        // Build a fresh world holding only what the physics step needs, optionally seeding a config singleton with
        // the cache knobs. Without a config the world uses the defaults (cache ON, threshold 8).
        static World MakePhysicsWorld(
            out FixedStepSimulationSystemGroup group,
            bool? cacheEnabled = null,
            int threshold = 8
        )
        {
            var world = new World("Physics2DDirectTestWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            fixedGroup.SortSystems();

            if (cacheEnabled.HasValue)
            {
                var cfg = PhysicsWorld2DConfig.Default;
                cfg.cacheIdenticalBodies = cacheEnabled.Value;
                cfg.identicalBodyThreshold = threshold;
                world.EntityManager.CreateSingleton(cfg);
            }

            group = fixedGroup;
            return world;
        }

        static PhysicsBody2DDefinition DynamicCircleBody(float2 pos) =>
            new PhysicsBody2DDefinition
            {
                bodyType = PhysicsBody.BodyType.Dynamic,
                gravityScale = 1f,
                initialPosition = pos,
                useAutoMass = true,
            };

        static PhysicsShape2D Circle(float radius) =>
            new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Circle,
                radius = radius,
                density = 1f,
                friction = 0.4f,
            };

        // Direct-author a dynamic circle at a pose, stamped with the shared form hash so the runtime sees it as one
        // of a form. The entity flows through the same creation loop as a baked one.
        static Entity AuthorFormBody(EntityManager em, float2 pos, float radius)
        {
            var entity = DirectPhysics2DAuthoring.Create(em, DynamicCircleBody(pos), Circle(radius));
            em.AddComponentData(entity, new PhysicsBody2DFormHash { value = FormKeyA });
            return entity;
        }

        // Same, but with an initial velocity seed. Velocity is EXCLUDED from the form hash, so a velocity body and a
        // plain body of the same circle form share a template — the template path overwrites only the per-instance
        // pose + velocity. This exercises the velocity overwrite through both the single-body and the collapse path.
        static Entity AuthorFormBodyWithVelocity(
            EntityManager em,
            float2 pos,
            float radius,
            float2 linVel,
            float angVel
        )
        {
            var entity = AuthorFormBody(em, pos, radius);
            em.AddComponentData(
                entity,
                new PhysicsBody2DInitialVelocity { linearVelocity = linVel, angularVelocity = angVel }
            );
            return entity;
        }

        [UnityTest]
        public IEnumerator DirectAuthoring_BodiesGetCreatedAndFall_NoNaN()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // Author 8 dynamic circle bodies directly — no MonoBehaviour, no Baker, no Rigidbody2D, no form hash.
            const int N = 8;
            var entities = new Entity[N];
            var startY = new float[N];
            for (var i = 0; i < N; i++)
            {
                startY[i] = 10f + i;
                entities[i] = DirectPhysics2DAuthoring.Create(
                    em,
                    DynamicCircleBody(new float2(i * 1.5f, startY[i])),
                    Circle(0.5f)
                );
            }

            // First update creates the bodies (no step), so the live PhysicsBody2D appears.
            group.Update();
            for (var i = 0; i < N; i++)
                Assert.IsTrue(
                    em.HasComponent<PhysicsBody2D>(entities[i]),
                    $"Direct-authored entity {i} did not gain a live PhysicsBody2D — the creation loop "
                        + "did not pick up a from-code-authored body."
                );

            for (var f = 0; f < Steps; f++)
                group.Update();

            for (var i = 0; i < N; i++)
            {
                var ltw = em.GetComponentData<LocalToWorld>(entities[i]);
                var y = ltw.Position.y;
                Assert.IsFalse(
                    isnan(y) || isinf(y) || isnan(ltw.Position.x) || isinf(ltw.Position.x),
                    $"Direct-authored body {i} produced NaN/Inf: pos={ltw.Position}."
                );
                Assert.Less(y, startY[i] - 0.5f, $"Direct-authored body {i} did not fall: startY={startY[i]}, y={y}.");
            }

            Debug.Log($"[PHYSICS2D-DIRECT] {N} direct-authored bodies created + fell, no NaN.");

            world.Dispose();
            yield break;
        }

        // The binding correctness gate: a body created via the cached-template path (optimisation ON, threshold
        // crossed) lands at the BIT-IDENTICAL simulated pose as one created via the per-entity path (optimisation
        // OFF). Both worlds author one identical form at the identical pose; the ON world additionally spawns
        // throw-away warm-up bodies of the SAME form first so the measured body is created from the built template,
        // not as the donor. After equal stepping the two measured bodies must match to the bit.
        [UnityTest]
        public IEnumerator CachedTemplatePath_IsBitIdenticalToPerEntityPath()
        {
            var probePos = new float2(3.5f, 25f);

            // OFF world: optimisation disabled — every body is the per-entity path. This is the oracle.
            var offWorld = MakePhysicsWorld(out var offGroup, cacheEnabled: false);
            var offEm = offWorld.EntityManager;
            var offProbe = AuthorFormBody(offEm, probePos, 0.5f);

            // ON world: optimisation enabled, threshold 2. Spawn 4 warm-up bodies of the form first (so the form's
            // template is built and IN USE), then the probe — which is therefore created from the template.
            var onWorld = MakePhysicsWorld(out var onGroup, cacheEnabled: true, threshold: 2);
            var onEm = onWorld.EntityManager;
            for (var i = 0; i < 4; i++)
                AuthorFormBody(onEm, new float2(-10f + i, 40f), 0.5f);
            var onProbe = AuthorFormBody(onEm, probePos, 0.5f);

            // First update creates the bodies on both (no step).
            offGroup.Update();
            onGroup.Update();

            var offBody = offEm.GetComponentData<PhysicsBody2D>(offProbe).body;
            var onBody = onEm.GetComponentData<PhysicsBody2D>(onProbe).body;
            Assert.IsTrue(offBody.isValid && onBody.isValid, "A probe body was not created.");

            // The mass configuration the two paths produced must be bit-identical BEFORE any stepping (the cache
            // replays the donor's resolved mass; the per-entity path resolves it directly).
            AssertMassIdentical(offBody, onBody, "at creation");

            // Step both the same number of times; the simulated poses must stay bit-identical step for step.
            for (var f = 0; f < Steps; f++)
            {
                offGroup.Update();
                onGroup.Update();
            }

            var offPos = (float2)(Vector2)offBody.position;
            var onPos = (float2)(Vector2)onBody.position;
            Assert.AreEqual(
                offPos.x,
                onPos.x,
                $"Template-path body X diverged from per-entity body: off={offPos}, on={onPos}. The optimisation "
                    + "is NOT transparent."
            );
            Assert.AreEqual(
                offPos.y,
                onPos.y,
                $"Template-path body Y diverged from per-entity body: off={offPos}, on={onPos}. The optimisation "
                    + "is NOT transparent."
            );
            var offRot = offBody.rotation;
            var onRot = onBody.rotation;
            Assert.AreEqual(offRot.cos, onRot.cos, "Template-path body rotation diverged (cos).");
            Assert.AreEqual(offRot.sin, onRot.sin, "Template-path body rotation diverged (sin).");
            AssertMassIdentical(offBody, onBody, "after stepping");

            Debug.Log(
                $"[PHYSICS2D-TRANSPARENT] cached-template body == per-entity body after {Steps} steps "
                    + $"(pos={onPos}, identical to off-path {offPos})."
            );

            offWorld.Dispose();
            onWorld.Dispose();
            yield break;
        }

        // A body carrying an initial-velocity seed must be bit-identical through the cached path too. The template
        // path and the in-frame collapse overwrite the per-instance velocity (the per-entity path sets it in the
        // body definition; the collapse sets it on the body after CreateBodyBatch) — this pins those agree. The ON
        // world spawns warm-up bodies of the form so the velocity probe lands on the in-frame collapse path.
        [UnityTest]
        public IEnumerator CachedTemplatePath_WithInitialVelocity_IsBitIdentical()
        {
            var probePos = new float2(-2f, 28f);
            var linVel = new float2(3f, 6f);
            const float angVel = 45f; // deg/s

            var offWorld = MakePhysicsWorld(out var offGroup, cacheEnabled: false);
            var offProbe = AuthorFormBodyWithVelocity(offWorld.EntityManager, probePos, 0.5f, linVel, angVel);

            var onWorld = MakePhysicsWorld(out var onGroup, cacheEnabled: true, threshold: 2);
            var onEm = onWorld.EntityManager;
            for (var i = 0; i < 4; i++)
                AuthorFormBody(onEm, new float2(-12f + i, 44f), 0.5f); // warm-up so the probe is template/collapse path
            var onProbe = AuthorFormBodyWithVelocity(onEm, probePos, 0.5f, linVel, angVel);

            offGroup.Update();
            onGroup.Update();

            var offBody = offWorld.EntityManager.GetComponentData<PhysicsBody2D>(offProbe).body;
            var onBody = onEm.GetComponentData<PhysicsBody2D>(onProbe).body;

            // Before any step, the seeded velocity must match exactly (both paths applied the same seed).
            Assert.AreEqual(
                ((float2)(Vector2)offBody.linearVelocity).x,
                ((float2)(Vector2)onBody.linearVelocity).x,
                "Seeded linear velocity X differs between per-entity and template paths."
            );
            Assert.AreEqual(
                ((float2)(Vector2)offBody.linearVelocity).y,
                ((float2)(Vector2)onBody.linearVelocity).y,
                "Seeded linear velocity Y differs between per-entity and template paths."
            );
            Assert.AreEqual(
                offBody.angularVelocity,
                onBody.angularVelocity,
                "Seeded angular velocity differs between per-entity and template paths."
            );

            for (var f = 0; f < Steps; f++)
            {
                offGroup.Update();
                onGroup.Update();
            }

            var offPos = (float2)(Vector2)offBody.position;
            var onPos = (float2)(Vector2)onBody.position;
            Assert.AreEqual(offPos.x, onPos.x, $"Velocity body X diverged: off={offPos}, on={onPos}.");
            Assert.AreEqual(offPos.y, onPos.y, $"Velocity body Y diverged: off={offPos}, on={onPos}.");

            Debug.Log($"[PHYSICS2D-TRANSPARENT-VEL] velocity body identical through the cached path (pos={onPos}).");

            offWorld.Dispose();
            onWorld.Dispose();
            yield break;
        }

        // On vs off, and a sweep of thresholds, must all produce the same final pose for the same authored spray —
        // the threshold N changes only WHEN the template starts being used, never the result.
        [UnityTest]
        public IEnumerator OnOff_AndThresholdSweep_ProduceSameSimulation()
        {
            const int N = 16;
            var startPos = new float2[N];
            for (var i = 0; i < N; i++)
                startPos[i] = new float2(i * 0.8f, 30f);

            var baseline = RunSpawnAndStep(N, startPos, cacheEnabled: false, threshold: 8);

            foreach (var thr in new[] { 1, 2, 4, 8, 16, 32 })
            {
                var withCache = RunSpawnAndStep(N, startPos, cacheEnabled: true, threshold: thr);
                for (var i = 0; i < N; i++)
                {
                    Assert.AreEqual(
                        baseline[i].x,
                        withCache[i].x,
                        $"Body {i} X differs between cache-off and cache-on(N={thr}): "
                            + $"{baseline[i]} vs {withCache[i]}. Threshold changed the simulation."
                    );
                    Assert.AreEqual(
                        baseline[i].y,
                        withCache[i].y,
                        $"Body {i} Y differs between cache-off and cache-on(N={thr}): "
                            + $"{baseline[i]} vs {withCache[i]}. Threshold changed the simulation."
                    );
                }
            }

            Debug.Log($"[PHYSICS2D-ONOFF] {N} bodies identical across cache off and N in {{1,2,4,8,16,32}}.");
            yield break;
        }

        // Spawn N identical form bodies at the given poses in ONE frame, step, and return each body's final
        // position. Used as the on/off + threshold equivalence oracle.
        static float2[] RunSpawnAndStep(int n, float2[] startPos, bool cacheEnabled, int threshold)
        {
            var world = MakePhysicsWorld(out var group, cacheEnabled: cacheEnabled, threshold: threshold);
            var em = world.EntityManager;
            var entities = new Entity[n];
            for (var i = 0; i < n; i++)
                entities[i] = AuthorFormBody(em, startPos[i], 0.5f);

            group.Update(); // create
            for (var f = 0; f < Steps; f++)
                group.Update();

            var result = new float2[n];
            for (var i = 0; i < n; i++)
                result[i] = (float2)(Vector2)em.GetComponentData<PhysicsBody2D>(entities[i]).body.position;

            world.Dispose();
            return result;
        }

        // The cross-frame spray — the workload the cache exists for. One identical form body is instantiated each
        // frame over many frames (a 1/frame spray, K = 1, so the in-frame collapse never fires and every body is the
        // cross-frame template path past the threshold). Every body must become physical immediately and fall, with
        // no NaN — the cache must not drop, duplicate, or stall a body in the spray.
        [UnityTest]
        public IEnumerator CrossFrameSpray_OnePerFrame_EachBodyCreatedAndFalls()
        {
            var world = MakePhysicsWorld(out var group, cacheEnabled: true, threshold: 4);
            var em = world.EntityManager;

            const int Frames = 40;
            var entities = new System.Collections.Generic.List<Entity>(Frames);
            var startY = new System.Collections.Generic.List<float>(Frames);

            for (var f = 0; f < Frames; f++)
            {
                var y = 50f + f; // each grain spawns a little higher, like a stream from a nozzle
                var e = AuthorFormBody(em, new float2(f * 0.1f, y), 0.25f);
                entities.Add(e);
                startY.Add(y);

                // One update per frame: this frame's lone new body is created (no step on a creation frame), and
                // every previously-created body steps once. So body f is created on frame f and physical at once.
                group.Update();
                Assert.IsTrue(
                    em.HasComponent<PhysicsBody2D>(e),
                    $"Sprayed body {f} did not become physical on the frame it spawned — the cross-frame cache "
                        + "dropped or stalled it."
                );
                Assert.IsTrue(
                    em.GetComponentData<PhysicsBody2D>(e).body.isValid,
                    $"Sprayed body {f}'s Box2D handle is invalid immediately after creation."
                );
            }

            // Settle: step enough that every grain has fallen measurably.
            for (var f = 0; f < Steps; f++)
                group.Update();

            for (var i = 0; i < entities.Count; i++)
            {
                var p = em.GetComponentData<LocalToWorld>(entities[i]).Position;
                Assert.IsFalse(
                    isnan(p.x) || isnan(p.y) || isinf(p.x) || isinf(p.y),
                    $"Sprayed body {i} produced NaN/Inf: {p}."
                );
                Assert.Less(
                    p.y,
                    startY[i] - 0.5f,
                    $"Sprayed body {i} did not fall: startY={startY[i]}, y={p.y}. A cached-template body is not "
                        + "simulating."
                );
            }

            Debug.Log($"[PHYSICS2D-SPRAY] {entities.Count} bodies sprayed 1/frame, each created + fell, no NaN.");

            world.Dispose();
            yield break;
        }

        // The deferred-simulation regression gate. The world step must run for the bodies ALREADY live even on a
        // frame that ALSO creates new bodies — i.e. an early-sprayed body keeps falling WHILE later bodies are
        // still being created, not frozen until the spray ends. (A prior per-frame "skip the step on any
        // creation frame" gate froze the whole population for the entire cross-frame spray: every frame created,
        // so no frame ever stepped until the spray finished, then all bodies dropped at once.) This asserts the
        // MID-spray displacement the older spray tests never checked — they only asserted the final settled pose
        // after a post-spray settle phase, which is exactly why the freeze-until-spray-ends bug passed them.
        [UnityTest]
        public IEnumerator CrossFrameSpray_EarlyBodiesFall_WhileLaterStillSpawning()
        {
            var world = MakePhysicsWorld(out var group, cacheEnabled: true, threshold: 4);
            var em = world.EntityManager;

            // Body 0 is created on frame 0; it must be falling well before the spray finishes.
            const float Body0StartY = 50f;
            var body0 = AuthorFormBody(em, new float2(0f, Body0StartY), 0.25f);

            // Frame 0: body0 is created (created AFTER this frame's step, so it does not integrate yet).
            group.Update();
            Assert.IsTrue(em.HasComponent<PhysicsBody2D>(body0), "body 0 should be physical on the frame it spawned.");
            var y0AtCreation = em.GetComponentData<PhysicsBody2D>(body0).body.position.y;

            // Keep creating a NEW body every frame for many frames. On each of these frames a new body is created,
            // so the old per-frame gate would skip the step entirely and body0 would never move. With the fix, the
            // step runs for the already-live population every frame, so body0 falls continuously during the spray.
            const int SprayFrames = 30;
            for (var f = 0; f < SprayFrames; f++)
            {
                AuthorFormBody(em, new float2(1f + f, 60f + f), 0.25f);
                group.Update(); // creates a new body this frame AND must step the already-live body0
            }

            // Body0 was created on frame 0 and has been stepped on every subsequent creation frame, so by now —
            // still mid-spray (a new body was created on the very last frame) — it must have fallen measurably.
            var y0MidSpray = em.GetComponentData<PhysicsBody2D>(body0).body.position.y;
            Assert.Less(
                y0MidSpray,
                y0AtCreation - 1f,
                $"body 0 did not fall during the spray: y at creation={y0AtCreation}, y mid-spray={y0MidSpray}. "
                    + "The world step is being deferred until the spray completes — bodies are frozen while "
                    + "later bodies are still being created."
            );

            Debug.Log(
                $"[PHYSICS2D-SPRAY-MIDFALL] body0 fell from {y0AtCreation} to {y0MidSpray} mid-spray "
                    + $"(over {SprayFrames} creation frames), concurrent with the ongoing spray."
            );

            world.Dispose();
            yield break;
        }

        static void AssertMassIdentical(PhysicsBody a, PhysicsBody b, string when)
        {
            var ma = a.massConfiguration;
            var mb = b.massConfiguration;
            Assert.AreEqual(ma.mass, mb.mass, $"mass differs {when} (per-entity {ma.mass} vs template {mb.mass}).");
            Assert.AreEqual(
                ma.rotationalInertia,
                mb.rotationalInertia,
                $"rotationalInertia differs {when} ({ma.rotationalInertia} vs {mb.rotationalInertia})."
            );
            Assert.AreEqual(
                ((float2)(Vector2)ma.center).x,
                ((float2)(Vector2)mb.center).x,
                $"center.x differs {when}."
            );
            Assert.AreEqual(
                ((float2)(Vector2)ma.center).y,
                ((float2)(Vector2)mb.center).y,
                $"center.y differs {when}."
            );
        }
    }
}
