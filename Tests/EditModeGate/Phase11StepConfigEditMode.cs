using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.TestTools;
using Zori.Entities.Physics2D.Tests.Editor;
using static Unity.Mathematics.math;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <see cref="Zori.Entities.Physics2D.Tests.Phase11StepConfigGate"/>: the simulation-config
    /// surface (<c>PhysicsStep2DAuthoring</c> → <c>PhysicsWorld2DConfig</c> singleton → world creation), authored
    /// as REAL components in a SubScene and baked through the actual <c>PhysicsStep2DAuthoringBaker</c>. Proves
    /// the AUTHORED values reach the world two ways — the baked <c>PhysicsWorld2DConfig</c> singleton carries
    /// them AND the live <c>PhysicsWorld</c> the system created reads them back — and that they change behaviour
    /// (trajectory under the configured gravity, in parity with a GameObject oracle whose
    /// <c>Physics2D.gravity</c> is matched to the authored value). Assertions, tolerances, step counts and log
    /// messages copied verbatim from the PlayMode gate; the <c>[UnityTest]</c> coroutines become plain
    /// <c>[Test]</c> over the synchronous EditMode SubScene harness.
    /// </summary>
    public sealed class Phase11StepConfigEditMode : Physics2DEditModeHarness
    {
        const float Dt = 1f / 60f;

        // The authored non-default gravity (mirrors Physics2DFixtures.P11ConfiguredGravity — duplicated as gate
        // constants, the source's pattern).
        static readonly float2 ConfiguredGravity = new float2(5f, -20f);
        static readonly float2 DefaultGravity = new float2(0f, -9.81f);
        const int SubstepsValue = 1;
        const bool SleepingAllowedValue = false;
        const float MaximumLinearSpeedValue = 7.5f;
        const float BounceThresholdValue = 13.5f;
        const float ContactSpeedValue = 9.25f;

        // -----------------------------------------------------------------------------------------------
        // Shared: author + load a SubScene, find the baked faller, create the Box2D world+body with ONE group
        // Update (PhysicsWorld2DSystem skips its Simulate on the creation frame), and hand the EntityManager + the
        // faller entity to the caller WITHOUT having stepped. The EditMode analogue of the source's
        // LoadBakeAndCreate — LoadSubScene already holds the FixedStep group off through streaming/baking and
        // re-enables it, so CreateBodies() is the deterministic creation frame.

        void LoadBakeAndCreate(Action<GameObject> populate, string sceneName, Action<EntityManager, Entity> onReady)
        {
            LoadSubScene(populate, sceneName);

            var fallerQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            Assert.Greater(
                fallerQuery.CalculateEntityCount(),
                0,
                $"No baked faller appeared in '{sceneName}' — the SubScene did not stream/bake."
            );
            using var fallers = fallerQuery.ToEntityArray(Allocator.Temp);
            var faller = fallers[0];

            // One group Update creates the world + the Box2D body (PhysicsWorld2DSystem skips its step on the
            // creation frame), so PhysicsWorldSingleton2D + PhysicsBody2D exist for the read-back, with no step.
            CreateBodies();

            onReady(EntityManager, faller);
        }

        // Read the live PhysicsWorld the system created from the published singleton.
        PhysicsWorld GetWorld(World world, EntityManager em)
        {
            var query = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            Assert.AreEqual(
                1,
                query.CalculateEntityCount(),
                "Expected exactly one PhysicsWorldSingleton2D after the creation update."
            );
            var pw = query.GetSingleton<PhysicsWorldSingleton2D>().world;
            Assert.IsTrue(pw.isValid, "The created PhysicsWorld is not valid.");
            return pw;
        }

        // The baked config singleton, or null if none baked. count > 1 is a multiplicity bug surfaced by the
        // caller; this helper returns the first and the count.
        static bool TryGetBakedConfig(EntityManager em, out PhysicsWorld2DConfig cfg, out int count)
        {
            var query = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorld2DConfig>());
            count = query.CalculateEntityCount();
            if (count == 0)
            {
                cfg = default;
                return false;
            }
            using var arr = query.ToComponentDataArray<PhysicsWorld2DConfig>(Allocator.Temp);
            cfg = arr[0];
            return true;
        }

        static float2 BodyPos(EntityManager em, Entity e) =>
            ((float2)(Vector2)em.GetComponentData<PhysicsBody2D>(e).body.position);

        // -----------------------------------------------------------------------------------------------
        // (1) Authored gravity reaches the world AND drives motion — the headline.

        [Test]
        public void ConfiguredGravity_ReachesWorldAndSingleton_AndDrivesMotion()
        {
            float2 worldGravity = default;
            var sawConfig = false;
            var configCount = 0;
            PhysicsWorld2DConfig bakedCfg = default;
            float2 startPos = default;

            LoadBakeAndCreate(
                Physics2DFixtures.P11ConfiguredGravityFixture,
                "P11ConfiguredGravity",
                (em, faller) =>
                {
                    var pw = GetWorld(World, em);
                    worldGravity = (float2)(Vector2)pw.gravity;
                    sawConfig = TryGetBakedConfig(em, out bakedCfg, out configCount);
                    startPos = BodyPos(em, faller);
                }
            );

            Debug.Log(
                $"[PHYSICS2D-P11-GRAV] baked-singleton-present={sawConfig} count={configCount} "
                    + $"bakedGravity={(sawConfig ? bakedCfg.gravity.ToString() : "—")} "
                    + $"liveWorldGravity={worldGravity} authored={ConfiguredGravity} startPos={startPos}"
            );

            // The baker produced exactly ONE config singleton from the authored component.
            Assert.IsTrue(
                sawConfig,
                "No PhysicsWorld2DConfig singleton baked from the authored PhysicsStep2DAuthoring — the baker "
                    + "did not emit the config. The authoring→bake chain is broken."
            );
            Assert.AreEqual(1, configCount, "Expected exactly one baked config singleton.");

            // The baked singleton carries the AUTHORED gravity (a dropped gravity field in AsConfig/Bake fails
            // here) — exact, this is a serialized value round-tripped through the baker.
            Assert.AreEqual(
                ConfiguredGravity.x,
                bakedCfg.gravity.x,
                1e-4f,
                $"Baked config gravity.x = {bakedCfg.gravity.x} != authored {ConfiguredGravity.x}. The baker "
                    + "dropped or mangled the gravity field."
            );
            Assert.AreEqual(
                ConfiguredGravity.y,
                bakedCfg.gravity.y,
                1e-4f,
                $"Baked config gravity.y = {bakedCfg.gravity.y} != authored {ConfiguredGravity.y}."
            );

            // The LIVE world the system created reads back the authored gravity (proves the system applied the
            // singleton at CreateWorld, not just that the singleton exists). The strongest binary witness.
            Assert.AreEqual(
                ConfiguredGravity.x,
                worldGravity.x,
                1e-3f,
                $"Live world gravity.x = {worldGravity.x} != authored {ConfiguredGravity.x}. The system did NOT "
                    + "apply the baked config to the world at creation."
            );
            Assert.AreEqual(
                ConfiguredGravity.y,
                worldGravity.y,
                1e-3f,
                $"Live world gravity.y = {worldGravity.y} != authored {ConfiguredGravity.y}."
            );
        }

        [Test]
        public void ConfiguredGravity_FallerTrajectory_ParityWithGameObjectOracle()
        {
            // Drive the baked faller and a GameObject oracle (Physics2D.gravity matched to the authored (5,-20))
            // in lockstep, comparing per-step positions within a growth-bounded envelope. This is BOTH the
            // gravity-drives-motion witness AND the no-double-step witness: the oracle integrates ONCE per
            // Physics2D.Simulate, so if the ECS side double-stepped it would diverge by ~2x immediately.
            const int Steps = 90; // 1.5 s — the faller drifts +x and falls -y under (5,-20).
            RunParityWithGravity(
                Physics2DFixtures.P11ConfiguredGravityFixture,
                "P11ConfiguredGravity",
                ConfiguredGravity,
                Steps,
                positionBaseMeters: 0.05f,
                positionGrowthPerStep: 0.01f
            );
        }

        [Test]
        public void ConfiguredGravity_NoDoubleStep_FallMatchesSingleStepIntegration()
        {
            // Absolute no-double-step pin: a body under gravity g, started at rest, displaces ½·g·t² after t
            // seconds of ONE-step-per-update integration. A double-step (engine auto-step + package Simulate)
            // would roughly quadruple the displacement (≈2x the steps → ≈4x the distance). Assert both axes of
            // the configured (5,-20) gravity land within a generous band around ½·g·t², which a 4x double-step
            // cannot satisfy.
            const int Steps = 90;
            float2 start = default;
            float2 end = default;

            LoadBakeAndCreate(
                Physics2DFixtures.P11ConfiguredGravityFixture,
                "P11ConfiguredGravity",
                (em, faller) =>
                {
                    start = BodyPos(em, faller);
                    for (var i = 0; i < Steps; i++)
                        FixedGroup.Update();
                    end = BodyPos(em, faller);
                }
            );

            var t = Steps * Dt;
            var expected = 0.5f * ConfiguredGravity * (t * t); // (½·gx·t², ½·gy·t²)
            var displacement = end - start;

            Debug.Log(
                $"[PHYSICS2D-P11-NODOUBLE] start={start} end={end} displacement={displacement} "
                    + $"expected≈{expected} (½·g·t², t={t:F3}s). A double-step would be ≈4x."
            );

            // 20% band around the single-step expectation — comfortably excludes the ≈4x a double-step gives,
            // while absorbing the v3 sub-stepping integrator's small free-fall convention offset.
            Assert.AreEqual(
                expected.x,
                displacement.x,
                0.2f * abs(expected.x) + 0.05f,
                $"Horizontal drift {displacement.x} m != single-step ½·gx·t² ≈ {expected.x} m. A value near "
                    + $"{4f * expected.x} would indicate a double-step (engine auto-step + package Simulate)."
            );
            Assert.AreEqual(
                expected.y,
                displacement.y,
                0.2f * abs(expected.y) + 0.05f,
                $"Vertical fall {displacement.y} m != single-step ½·gy·t² ≈ {expected.y} m. A value near "
                    + $"{4f * expected.y} would indicate a double-step."
            );
        }

        // -----------------------------------------------------------------------------------------------
        // (2) Backward-compat fallback: NO PhysicsStep2DAuthoring → default world, behaviour unchanged.

        [Test]
        public void DefaultFallback_NoConfigBaked_WorldKeepsDefaultGravity()
        {
            float2 worldGravity = default;
            var sawConfig = false;
            var configCount = 0;

            LoadBakeAndCreate(
                Physics2DFixtures.P11DefaultFallbackFixture,
                "P11DefaultFallback",
                (em, faller) =>
                {
                    var pw = GetWorld(World, em);
                    worldGravity = (float2)(Vector2)pw.gravity;
                    sawConfig = TryGetBakedConfig(em, out _, out configCount);
                }
            );

            Debug.Log(
                $"[PHYSICS2D-P11-FALLBACK] config-baked={sawConfig} count={configCount} "
                    + $"liveWorldGravity={worldGravity} default={DefaultGravity}"
            );

            // No PhysicsStep2DAuthoring authored → NO config singleton baked. The if (cfg.HasValue) guard must
            // see no config and take the defaultDefinition path.
            Assert.IsFalse(
                sawConfig,
                $"A PhysicsWorld2DConfig singleton ({configCount}) was baked into a SubScene with NO "
                    + "PhysicsStep2DAuthoring — something is emitting a config where none was authored, which "
                    + "would shift the backward-compat default path."
            );

            // The world keeps the Box2D defaultDefinition gravity — byte-identical to before the config surface.
            Assert.AreEqual(
                DefaultGravity.x,
                worldGravity.x,
                1e-3f,
                $"No-config world gravity.x = {worldGravity.x} != Box2D default {DefaultGravity.x}."
            );
            Assert.AreEqual(
                DefaultGravity.y,
                worldGravity.y,
                1e-2f,
                $"No-config world gravity.y = {worldGravity.y} != Box2D default {DefaultGravity.y}. The "
                    + "backward-compat defaultDefinition fallback is not intact."
            );
        }

        [Test]
        public void DefaultFallback_FallerTrajectory_ParityWithDefaultGravityOracle()
        {
            // The no-config faller must fall as today: parity with a GameObject oracle at the Box2D default
            // gravity (-9.81). This pins the default PATH behaviour, not just the read-back value.
            const int Steps = 90;
            RunParityWithGravity(
                Physics2DFixtures.P11DefaultFallbackFixture,
                "P11DefaultFallback",
                DefaultGravity,
                Steps,
                positionBaseMeters: 0.05f,
                positionGrowthPerStep: 0.01f
            );
        }

        // -----------------------------------------------------------------------------------------------
        // (3) Substep count takes effect (read-back).

        [Test]
        public void Substeps_AuthoredValueReachesWorldAndSingleton()
        {
            var worldSubSteps = 0;
            var sawConfig = false;
            PhysicsWorld2DConfig bakedCfg = default;

            LoadBakeAndCreate(
                Physics2DFixtures.P11SubstepsFixture,
                "P11Substeps",
                (em, faller) =>
                {
                    var pw = GetWorld(World, em);
                    worldSubSteps = pw.simulationSubSteps;
                    sawConfig = TryGetBakedConfig(em, out bakedCfg, out _);
                }
            );

            Debug.Log(
                $"[PHYSICS2D-P11-SUBSTEPS] baked-config={sawConfig} bakedSubSteps="
                    + $"{(sawConfig ? bakedCfg.simulationSubSteps.ToString() : "—")} liveWorldSubSteps="
                    + $"{worldSubSteps} authored={SubstepsValue} (default would be 4)"
            );

            Assert.IsTrue(sawConfig, "No config singleton baked for the substeps fixture.");
            Assert.AreEqual(
                SubstepsValue,
                bakedCfg.simulationSubSteps,
                $"Baked config simulationSubSteps = {bakedCfg.simulationSubSteps} != authored {SubstepsValue}. "
                    + "The baker dropped the simulationSubSteps field."
            );
            // The live world reads back the authored substep count, NOT the default 4. The behavioural effect of
            // 1 vs 4 substeps on a free-falling body is sub-perceptible (free fall is exact under any substep
            // count), so the read-back IS the deterministic witness — documented in the gate's doc as the
            // reason a behavioural difference is not asserted for this field.
            Assert.AreEqual(
                SubstepsValue,
                worldSubSteps,
                $"Live world simulationSubSteps = {worldSubSteps} != authored {SubstepsValue}. The system did "
                    + "NOT apply the configured substep count (it would read 4, the default, if it ignored the "
                    + "config field)."
            );
            Assert.AreNotEqual(
                4,
                worldSubSteps,
                "Live world substeps is the default 4 — the authored value 1 was not applied."
            );
        }

        // -----------------------------------------------------------------------------------------------
        // (4) ≥2 more fields reach the world: sleepingAllowed, maximumLinearSpeed, bounceThreshold, contactSpeed.

        [Test]
        public void MoreFields_AuthoredValuesReachWorldAndSingleton()
        {
            var sawConfig = false;
            PhysicsWorld2DConfig bakedCfg = default;
            var worldSleeping = true;
            var worldMaxSpeed = 0f;
            var worldBounce = 0f;
            var worldContactSpeed = 0f;

            LoadBakeAndCreate(
                Physics2DFixtures.P11MoreFieldsFixture,
                "P11MoreFields",
                (em, faller) =>
                {
                    var pw = GetWorld(World, em);
                    sawConfig = TryGetBakedConfig(em, out bakedCfg, out _);
                    worldSleeping = pw.sleepingAllowed;
                    worldMaxSpeed = pw.maximumLinearSpeed;
                    worldBounce = pw.bounceThreshold;
                    worldContactSpeed = pw.contactSpeed;
                }
            );

            Debug.Log(
                $"[PHYSICS2D-P11-FIELDS] baked-config={sawConfig} live: sleeping={worldSleeping} "
                    + $"maxSpeed={worldMaxSpeed} bounce={worldBounce} contactSpeed={worldContactSpeed} | "
                    + $"authored: sleeping={SleepingAllowedValue} maxSpeed={MaximumLinearSpeedValue} "
                    + $"bounce={BounceThresholdValue} contactSpeed={ContactSpeedValue}"
            );

            Assert.IsTrue(sawConfig, "No config singleton baked for the more-fields fixture.");

            // Baked-singleton round-trip (catches a baker dropping any of these four fields).
            Assert.AreEqual(SleepingAllowedValue, bakedCfg.sleepingAllowed, "Baker dropped sleepingAllowed.");
            Assert.AreEqual(
                MaximumLinearSpeedValue,
                bakedCfg.maximumLinearSpeed,
                1e-4f,
                "Baker dropped maximumLinearSpeed."
            );
            Assert.AreEqual(BounceThresholdValue, bakedCfg.bounceThreshold, 1e-4f, "Baker dropped bounceThreshold.");
            Assert.AreEqual(ContactSpeedValue, bakedCfg.contactSpeed, 1e-4f, "Baker dropped contactSpeed.");

            // Live-world read-back (proves the system applied each field at CreateWorld, not just baked it).
            Assert.AreEqual(
                SleepingAllowedValue,
                worldSleeping,
                $"Live world sleepingAllowed = {worldSleeping} != authored {SleepingAllowedValue}. The system "
                    + "did not apply the authored sleepingAllowed (it would be true, the default, if ignored)."
            );
            Assert.AreEqual(
                MaximumLinearSpeedValue,
                worldMaxSpeed,
                1e-3f,
                $"Live world maximumLinearSpeed = {worldMaxSpeed} != authored {MaximumLinearSpeedValue} "
                    + "(default 400)."
            );
            Assert.AreEqual(
                BounceThresholdValue,
                worldBounce,
                1e-3f,
                $"Live world bounceThreshold = {worldBounce} != authored {BounceThresholdValue} (default 1)."
            );
            Assert.AreEqual(
                ContactSpeedValue,
                worldContactSpeed,
                1e-3f,
                $"Live world contactSpeed = {worldContactSpeed} != authored {ContactSpeedValue} (default 3)."
            );
        }

        [Test]
        public void MoreFields_MaximumLinearSpeed_ClampsFastFaller()
        {
            // Behavioural witness for maximumLinearSpeed: the more-fields fixture authors maximumLinearSpeed=7.5
            // m/s under the DEFAULT gravity (-9.81). After enough free-fall steps a body would exceed 7.5 m/s
            // (terminal-less free fall), so the clamp holds its speed at ~7.5. We step long enough that an
            // UNCLAMPED faller would be well past 7.5 m/s (after ~1.5 s, v ≈ 9.81·1.5 ≈ 14.7 m/s), then assert
            // the body's speed never materially exceeds the configured clamp.
            const int Steps = 120; // 2.0 s — unclamped v would reach ~19.6 m/s.
            var maxSpeedSeen = 0f;

            LoadBakeAndCreate(
                Physics2DFixtures.P11MoreFieldsFixture,
                "P11MoreFields",
                (em, faller) =>
                {
                    // The creation update already ran (no step). Step and watch the speed.
                    for (var i = 0; i < Steps; i++)
                    {
                        FixedGroup.Update();
                        var v = (float2)(Vector2)em.GetComponentData<PhysicsBody2D>(faller).body.linearVelocity;
                        maxSpeedSeen = max(maxSpeedSeen, length(v));
                    }
                }
            );

            Debug.Log(
                $"[PHYSICS2D-P11-CLAMP] maxSpeedSeen={maxSpeedSeen:F3} m/s over {Steps} steps "
                    + $"(authored maximumLinearSpeed={MaximumLinearSpeedValue}; unclamped free fall would reach "
                    + $"~{9.81f * Steps * Dt:F1} m/s)."
            );

            // The clamp is the authored 7.5 m/s. Allow a small margin for the solver's per-step clamp ordering,
            // but the body must NOT approach the ~19.6 m/s an unclamped faller reaches. A clamp that was the
            // default 400 would let the speed run to ~19.6 here.
            Assert.LessOrEqual(
                maxSpeedSeen,
                MaximumLinearSpeedValue + 1.0f,
                $"Faller speed reached {maxSpeedSeen:F3} m/s, above the authored maximumLinearSpeed clamp "
                    + $"{MaximumLinearSpeedValue} m/s. The clamp was not applied to the world (a default-400 "
                    + "clamp would let free fall reach ~19.6 m/s here)."
            );
            // And it must have actually built up to near the clamp (so the test isn't trivially passing on a
            // body that never moved).
            Assert.Greater(
                maxSpeedSeen,
                0.5f * MaximumLinearSpeedValue,
                $"Faller speed peaked at only {maxSpeedSeen:F3} m/s — it never built up toward the clamp, so the "
                    + "clamp witness is inconclusive (the body may not be falling)."
            );
        }

        // -----------------------------------------------------------------------------------------------
        // (6) Singleton multiplicity: two PhysicsStep2DAuthoring in one SubScene throw at world creation.

        [Test]
        public void Multiplicity_TwoConfigs_ThrowAtWorldCreation()
        {
            // LoadSubScene streams + bakes the SubScene synchronously (the FixedStep group held off through the
            // streaming update), so both config singletons exist before any creation step. The creation update
            // (CreateBodies) is then the world-creation frame whose config resolution throws.
            LoadSubScene(Physics2DFixtures.P11MultiplicityFixture, "P11Multiplicity");

            var cfgQuery = Query(ComponentType.ReadOnly<PhysicsWorld2DConfig>());
            var fallerQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>()
            );
            Assert.Greater(fallerQuery.CalculateEntityCount(), 0, "No baked faller in the multiplicity fixture.");
            var bakedConfigs = cfgQuery.CalculateEntityCount();
            Debug.Log(
                $"[PHYSICS2D-P11-MULTI] baked config singletons={bakedConfigs} "
                    + "(two PhysicsStep2DAuthoring on two GameObjects in one SubScene)."
            );
            Assert.AreEqual(
                2,
                bakedConfigs,
                $"Expected 2 baked PhysicsWorld2DConfig singletons (the multiplicity fixture authors two "
                    + $"PhysicsStep2DAuthoring on two GameObjects); saw {bakedConfigs}. [DisallowMultipleComponent] "
                    + "only blocks two on ONE GameObject, so two on two GameObjects must both bake."
            );

            // The documented behaviour: PhysicsWorld2DSystem.OnUpdate resolves the config via
            // SystemAPI.TryGetSingleton<PhysicsWorld2DConfig>, which THROWS on more than one — surfacing the
            // "one PhysicsStep2D per world" rule loudly at world creation rather than silently last-wins. The
            // throw originates inside the Burst-direct-call codegen of OnUpdate, where Entities catches it and
            // routes it to Debug.LogException rather than propagating it up through group.Update() — so a
            // try/catch around the Update does NOT see it, and the throw fires at config resolution (:309)
            // BEFORE the world-(re)create block, so no new world is published this step (a PhysicsWorldSingleton2D
            // that survives a LoadSceneMode.Single swap from an earlier test would falsely read as "created").
            // The framing-correct witness is therefore the LOGGED exception itself: it must be an
            // InvalidOperationException naming PhysicsWorld2DConfig and the "zero or one" singleton rule. A silent
            // last-wins pick (the bug class) would log NO such exception and step cleanly. LogAssert.Expect makes
            // the absence of that log a test failure, and consumes the (otherwise unexpected) exception log so it
            // does not itself fail the run.
            LogAssert.Expect(
                LogType.Exception,
                new System.Text.RegularExpressions.Regex(
                    "InvalidOperationException.*(HasSingleton|TryGetSingleton).*PhysicsWorld2DConfig.*"
                        + "(zero or one|only.*one)"
                )
            );

            CreateBodies(); // config resolution throws on the two configs; logged, not propagated

            Debug.Log(
                "[PHYSICS2D-P11-MULTI] expected the logged InvalidOperationException naming PhysicsWorld2DConfig "
                    + "and the zero-or-one singleton rule (the documented multiplicity throw at world creation). "
                    + "If absent, the LogAssert.Expect above flushes as a failure at test teardown — that IS the "
                    + "assertion; a silent last-wins pick would log no such exception."
            );
        }
    }
}
