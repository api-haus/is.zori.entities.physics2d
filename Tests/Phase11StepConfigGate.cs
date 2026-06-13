using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-11 GameObject-parity / read-back gate for the simulation-config surface
    /// (<see cref="PhysicsStep2DAuthoring"/> → <see cref="PhysicsWorld2DConfig"/> singleton → world creation),
    /// authored as REAL components in a SubScene and baked through the actual
    /// <c>PhysicsStep2DAuthoringBaker</c>. The Phase-11 smoke (<c>StepConfigSmoke</c>) CONSTRUCTS the singleton
    /// in code (<c>EntityManager.CreateSingleton</c>); it never runs the baker, so a baker that silently dropped
    /// an authored field would pass the smoke and fail a real bake. This gate closes that escalated gap: it
    /// bakes a real authoring component and proves the AUTHORED values reach the world two ways — the baked
    /// <see cref="PhysicsWorld2DConfig"/> singleton carries them, AND the live <c>PhysicsWorld</c> the system
    /// created reads them back (its <c>gravity</c>/<c>simulationSubSteps</c>/<c>sleepingAllowed</c>/… getters) —
    /// and that they change behaviour (trajectory under the configured gravity, in parity with a GameObject
    /// oracle whose <c>Physics2D.gravity</c> is matched to the authored value).
    /// </summary>
    /// <remarks>
    /// <para><b>Decision points pinned.</b> (1) Authored gravity reaches the world AND drives motion — the baked
    /// singleton + the live world both carry (5,-20), the baked faller drifts/falls at that gravity, and a
    /// GameObject body under a matched <c>Physics2D.gravity</c> agrees within a growth-bounded envelope.
    /// (2) Backward-compat fallback — a SubScene with NO <c>PhysicsStep2DAuthoring</c> bakes NO config singleton,
    /// the world keeps the Box2D <c>defaultDefinition</c> gravity (-9.81), and the faller falls as today (parity
    /// with a -9.81 oracle). (3) Substep count takes effect — an authored <c>simulationSubSteps=1</c> reads back
    /// off the live world as 1, not the default 4. (4) ≥2 more fields reach the world — authored
    /// <c>sleepingAllowed=false</c>, <c>maximumLinearSpeed=7.5</c>, <c>bounceThreshold=13.5</c>,
    /// <c>contactSpeed=9.25</c> all read back off the live world, plus a behavioural witness that
    /// <c>maximumLinearSpeed</c> clamps a fast faller's speed. (5) No double-step — the configured-gravity faller
    /// advances by exactly ONE step's worth per ECS step (½·g·t², matching the once-per-update Simulate under
    /// <c>simulationType=Script</c>); a double-step would show ~4× the displacement and blow both the absolute
    /// check and the GameObject-parity envelope. (6) Singleton multiplicity — two authored components in one
    /// SubScene throw at world creation (the <c>TryGetSingleton</c> contract), not silently last-wins.</para>
    ///
    /// <para><b>World isolation.</b> Each test loads its parent scene with <c>LoadSceneMode.Single</c> (tearing
    /// the prior scene's entities out of the default world) and disables the
    /// <c>FixedStepSimulationSystemGroup</c> through the bake-wait (the <c>PhysicsParityHarness</c> discipline),
    /// so a thrown test cannot leak stepped bodies into a later one. The group is driven explicitly with a
    /// swapped <c>FixedRateSimpleManager</c> so each <c>Update()</c> is exactly one fixed step.</para>
    /// </remarks>
    public sealed class Phase11StepConfigGate
    {
        const int LoadTimeoutFrames = 600;
        const float Dt = 1f / 60f;

        // The authored non-default gravity (mirrors StepConfigFixtureBuilder.ConfiguredGravity — the runtime
        // Tests asmdef cannot reference the Editor-platform fixture builder, so the load-bearing constant is
        // duplicated, the package's pattern).
        static readonly float2 ConfiguredGravity = new float2(5f, -20f);
        static readonly float2 DefaultGravity = new float2(0f, -9.81f);
        const int SubstepsValue = 1;
        const bool SleepingAllowedValue = false;
        const float MaximumLinearSpeedValue = 7.5f;
        const float BounceThresholdValue = 13.5f;
        const float ContactSpeedValue = 9.25f;

        // -----------------------------------------------------------------------------------------------
        // Shared: load a parent SubScene, wait for the faller to bake, create the Box2D world+body with ONE
        // group Update (PhysicsWorld2DSystem skips its Simulate on the creation frame), and hand the world +
        // EntityManager + the faller entity to the caller WITHOUT having stepped.

        static IEnumerator LoadBakeAndCreate(
            string parentScenePath,
            System.Action<World, EntityManager, FixedStepSimulationSystemGroup, Entity> onReady
        )
        {
            SceneManager.LoadScene(parentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");
            var em = world.EntityManager;

            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            // Hold the step off through the bake-wait so the faller never integrates before we read back.
            fixedGroup.Enabled = false;

            // Wait for the baked dynamic faller (a Rigidbody2D+CircleCollider2D) to stream + bake.
            var fallerQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var framesWaited = 0;
            while (fallerQuery.CalculateEntityCount() < 1 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.Greater(
                fallerQuery.CalculateEntityCount(),
                0,
                $"No baked faller appeared after {framesWaited} frames — the SubScene '{parentScenePath}' did "
                    + "not stream/bake. Build the fixtures first via "
                    + "-executeMethod Zori.Entities.Physics2D.Tests.Editor.StepConfigFixtureBuilder.BuildAll."
            );

            using var fallers = fallerQuery.ToEntityArray(Allocator.Temp);
            var faller = fallers[0];

            // One group Update creates the world + the Box2D body (PhysicsWorld2DSystem skips its step on the
            // creation frame), so PhysicsWorldSingleton2D + PhysicsBody2D exist for the read-back, with no step.
            var savedRate = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
            fixedGroup.Enabled = true;
            fixedGroup.Update();

            onReady(world, em, fixedGroup, faller);

            fixedGroup.RateManager = savedRate;
            fixedGroup.Enabled = false;
        }

        // Read the live PhysicsWorld the system created from the published singleton.
        static PhysicsWorld GetWorld(World world, EntityManager em)
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

        [UnityTest]
        public IEnumerator ConfiguredGravity_ReachesWorldAndSingleton_AndDrivesMotion()
        {
            float2 worldGravity = default;
            var sawConfig = false;
            var configCount = 0;
            PhysicsWorld2DConfig bakedCfg = default;
            float2 startPos = default;

            yield return LoadBakeAndCreate(
                "Assets/EntitiesPhysics2DFixture/P11ConfiguredGravity.unity",
                (world, em, group, faller) =>
                {
                    var pw = GetWorld(world, em);
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

        [UnityTest]
        public IEnumerator ConfiguredGravity_FallerTrajectory_ParityWithGameObjectOracle()
        {
            // Drive the baked faller and a GameObject oracle (Physics2D.gravity matched to the authored (5,-20))
            // in lockstep, comparing per-step positions within a growth-bounded envelope. This is BOTH the
            // gravity-drives-motion witness AND the no-double-step witness: the oracle integrates ONCE per
            // Physics2D.Simulate, so if the ECS side double-stepped it would diverge by ~2x immediately.
            const int Steps = 90; // 1.5 s — the faller drifts +x and falls -y under (5,-20).
            yield return RunParityAgainstOracle(
                "Assets/EntitiesPhysics2DFixture/P11ConfiguredGravity.unity",
                ConfiguredGravity,
                Steps,
                positionBaseMeters: 0.05f,
                positionGrowthPerStep: 0.01f
            );
        }

        [UnityTest]
        public IEnumerator ConfiguredGravity_NoDoubleStep_FallMatchesSingleStepIntegration()
        {
            // Absolute no-double-step pin: a body under gravity g, started at rest, displaces ½·g·t² after t
            // seconds of ONE-step-per-update integration. A double-step (engine auto-step + package Simulate)
            // would roughly quadruple the displacement (≈2x the steps → ≈4x the distance). Assert both axes of
            // the configured (5,-20) gravity land within a generous band around ½·g·t², which a 4x double-step
            // cannot satisfy.
            const int Steps = 90;
            float2 start = default;
            float2 end = default;

            yield return LoadBakeAndCreateThenStep(
                "Assets/EntitiesPhysics2DFixture/P11ConfiguredGravity.unity",
                Steps,
                (em, faller) => start = BodyPos(em, faller),
                (em, faller) => end = BodyPos(em, faller)
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

        [UnityTest]
        public IEnumerator DefaultFallback_NoConfigBaked_WorldKeepsDefaultGravity()
        {
            float2 worldGravity = default;
            var sawConfig = false;
            var configCount = 0;

            yield return LoadBakeAndCreate(
                "Assets/EntitiesPhysics2DFixture/P11DefaultFallback.unity",
                (world, em, group, faller) =>
                {
                    var pw = GetWorld(world, em);
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

        [UnityTest]
        public IEnumerator DefaultFallback_FallerTrajectory_ParityWithDefaultGravityOracle()
        {
            // The no-config faller must fall as today: parity with a GameObject oracle at the Box2D default
            // gravity (-9.81). This pins the default PATH behaviour, not just the read-back value.
            const int Steps = 90;
            yield return RunParityAgainstOracle(
                "Assets/EntitiesPhysics2DFixture/P11DefaultFallback.unity",
                DefaultGravity,
                Steps,
                positionBaseMeters: 0.05f,
                positionGrowthPerStep: 0.01f
            );
        }

        // -----------------------------------------------------------------------------------------------
        // (3) Substep count takes effect (read-back).

        [UnityTest]
        public IEnumerator Substeps_AuthoredValueReachesWorldAndSingleton()
        {
            var worldSubSteps = 0;
            var sawConfig = false;
            PhysicsWorld2DConfig bakedCfg = default;

            yield return LoadBakeAndCreate(
                "Assets/EntitiesPhysics2DFixture/P11Substeps.unity",
                (world, em, group, faller) =>
                {
                    var pw = GetWorld(world, em);
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

        [UnityTest]
        public IEnumerator MoreFields_AuthoredValuesReachWorldAndSingleton()
        {
            var sawConfig = false;
            PhysicsWorld2DConfig bakedCfg = default;
            var worldSleeping = true;
            var worldMaxSpeed = 0f;
            var worldBounce = 0f;
            var worldContactSpeed = 0f;

            yield return LoadBakeAndCreate(
                "Assets/EntitiesPhysics2DFixture/P11MoreFields.unity",
                (world, em, group, faller) =>
                {
                    var pw = GetWorld(world, em);
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

        [UnityTest]
        public IEnumerator MoreFields_MaximumLinearSpeed_ClampsFastFaller()
        {
            // Behavioural witness for maximumLinearSpeed: the more-fields fixture authors maximumLinearSpeed=7.5
            // m/s under the DEFAULT gravity (-9.81). After enough free-fall steps a body would exceed 7.5 m/s
            // (terminal-less free fall), so the clamp holds its speed at ~7.5. We step long enough that an
            // UNCLAMPED faller would be well past 7.5 m/s (after ~1.5 s, v ≈ 9.81·1.5 ≈ 14.7 m/s), then assert
            // the body's speed never materially exceeds the configured clamp.
            const int Steps = 120; // 2.0 s — unclamped v would reach ~19.6 m/s.
            var maxSpeedSeen = 0f;

            yield return LoadBakeAndCreate(
                "Assets/EntitiesPhysics2DFixture/P11MoreFields.unity",
                (world, em, group, faller) =>
                {
                    // The creation update already ran (no step). Step and watch the speed.
                    for (var i = 0; i < Steps; i++)
                    {
                        group.Update();
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

        [UnityTest]
        public IEnumerator Multiplicity_TwoConfigs_ThrowAtWorldCreation()
        {
            SceneManager.LoadScene("Assets/EntitiesPhysics2DFixture/P11Multiplicity.unity", LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world.");
            var em = world.EntityManager;

            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.Enabled = false;

            // Wait for BOTH config singletons to bake (the residual two-on-two-GameObjects case).
            var cfgQuery = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorld2DConfig>());
            var fallerQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>()
            );
            var framesWaited = 0;
            while (
                (cfgQuery.CalculateEntityCount() < 2 || fallerQuery.CalculateEntityCount() < 1)
                && framesWaited < LoadTimeoutFrames
            )
            {
                framesWaited++;
                yield return null;
            }
            var bakedConfigs = cfgQuery.CalculateEntityCount();
            Debug.Log(
                $"[PHYSICS2D-P11-MULTI] baked config singletons={bakedConfigs} after {framesWaited} frames "
                    + "(two PhysicsStep2DAuthoring on two GameObjects in one SubScene)."
            );
            Assert.AreEqual(
                2,
                bakedConfigs,
                $"Expected 2 baked PhysicsWorld2DConfig singletons (the multiplicity fixture authors two "
                    + $"PhysicsStep2DAuthoring on two GameObjects); saw {bakedConfigs}. [DisallowMultipleComponent] "
                    + "only blocks two on ONE GameObject, so two on two GameObjects must both bake. Build the "
                    + "fixtures first if this is 0."
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

            var savedRate = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
            fixedGroup.Enabled = true;
            fixedGroup.Update(); // config resolution throws on the two configs; logged, not propagated
            fixedGroup.RateManager = savedRate;
            fixedGroup.Enabled = false;

            Debug.Log(
                "[PHYSICS2D-P11-MULTI] expected the logged InvalidOperationException naming PhysicsWorld2DConfig "
                    + "and the zero-or-one singleton rule (the documented multiplicity throw at world creation). "
                    + "If absent, the LogAssert.Expect above flushes as a failure at test teardown — that IS the "
                    + "assertion; a silent last-wins pick would log no such exception."
            );
        }

        // -----------------------------------------------------------------------------------------------
        // Parity oracle: drive the baked faller and a GameObject Rigidbody2D built from the SAME child scene in
        // lockstep, Physics2D.gravity matched to `gravity`, comparing per-step positions within a growth-bounded
        // envelope. A specialised RunParity (the harness hardcodes -9.81; here gravity is per-fixture). One
        // faller per fixture, so body↔body matching is trivial.

        IEnumerator RunParityAgainstOracle(
            string parentScenePath,
            float2 gravity,
            int stepCount,
            float positionBaseMeters,
            float positionGrowthPerStep
        )
        {
            // Derive the child scene name from the parent (the fixture builder names them <name> / <name>_Sub).
            var parentName = System.IO.Path.GetFileNameWithoutExtension(parentScenePath);
            var childName = parentName + "_Sub";

            // --- ECS side: load parent, wait for the faller to bake, hold the step off through the wait. ---
            SceneManager.LoadScene(parentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world.");
            var em = world.EntityManager;
            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup.");
            fixedGroup.Enabled = false;

            var liveQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var bakedQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var framesWaited = 0;
            while (bakedQuery.CalculateEntityCount() < 1 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.Greater(
                bakedQuery.CalculateEntityCount(),
                0,
                $"No baked faller in '{parentScenePath}' after {framesWaited} frames. Build the fixtures first."
            );

            // --- GameObject reference: matched Physics2D.gravity + Script mode, same authored faller, live. ---
            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = new Vector2(gravity.x, gravity.y);

            var refLoad = SceneManager.LoadSceneAsync(childName, LoadSceneMode.Additive);
            Assert.IsNotNull(
                refLoad,
                $"Child authoring scene '{childName}' is not loadable by name — it must be registered in build "
                    + "settings by the fixture builder."
            );
            while (!refLoad.isDone)
                yield return null;

            var childScene = SceneManager.GetSceneByName(childName);
            var refBodies = new List<Rigidbody2D>();
            foreach (var root in childScene.GetRootGameObjects())
            foreach (var rb in root.GetComponentsInChildren<Rigidbody2D>(includeInactive: true))
            {
                if (rb.bodyType == RigidbodyType2D.Static)
                    continue;
                rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
                refBodies.Add(rb);
            }
            Assert.AreEqual(
                bakedQuery.CalculateEntityCount(),
                refBodies.Count,
                $"Body count mismatch: baked {bakedQuery.CalculateEntityCount()} vs reference {refBodies.Count}."
            );
            UnityEngine.Physics2D.SyncTransforms();

            // --- Step both in lockstep. First ECS Update creates the body (no step); the oracle is un-stepped. ---
            var savedRate = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
            fixedGroup.Enabled = true;
            fixedGroup.Update(); // create, no step

            var ecsTraj = new float2[stepCount];
            var refTraj = new float2[stepCount];
            for (var s = 0; s < stepCount; s++)
            {
                fixedGroup.Update();
                using (var ltws = liveQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp))
                    ecsTraj[s] = new float2(ltws[0].Value.c3.x, ltws[0].Value.c3.y);

                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                refTraj[s] = new float2(refBodies[0].position.x, refBodies[0].position.y);
            }

            fixedGroup.RateManager = savedRate;
            fixedGroup.Enabled = false;

            // --- Tear down + restore global state. ---
            foreach (var rb in refBodies)
                if (rb != null)
                    Object.Destroy(rb.gameObject);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            var refUnload = SceneManager.UnloadSceneAsync(childScene);
            if (refUnload != null)
                while (!refUnload.isDone)
                    yield return null;

            // --- Assert: growth-bounded position parity + a travel disqualifier. ---
            var worst = 0f;
            string violation = null;
            var log = new System.Text.StringBuilder();
            log.AppendLine(
                $"[PHYSICS2D-P11-PARITY] scene={parentName} gravity={gravity} steps={stepCount} "
                    + $"loadFrames={framesWaited}"
            );
            log.AppendLine("step\tposErr\tband\tecs\tref");
            for (var s = 0; s < stepCount; s++)
            {
                var band = positionBaseMeters + positionGrowthPerStep * (s + 1);
                var dp = length(ecsTraj[s] - refTraj[s]);
                worst = max(worst, dp);
                if (violation == null && dp > band)
                    violation =
                        $"Position parity broke at step {s}: |ECS - GameObject| = {dp} m exceeds the band "
                        + $"{band} m. ECS={ecsTraj[s]}, GameObject={refTraj[s]} (gravity {gravity}).";
                if (s % 10 == 0 || s == stepCount - 1)
                    log.AppendLine($"{s}\t{dp:E4}\t{band:E4}\t{ecsTraj[s]}\t{refTraj[s]}");
            }
            var travel = length(ecsTraj[stepCount - 1] - ecsTraj[0]);
            log.AppendLine($"[PHYSICS2D-P11-PARITY] WORST_POS_ERR={worst:E6} travel={travel:F3} m");
            if (violation != null)
                log.AppendLine($"[PHYSICS2D-P11-PARITY] VIOLATION: {violation}");
            Debug.Log(log.ToString());

            Assert.Greater(
                travel,
                1.0f,
                $"The baked faller barely moved ({travel:F3} m) — a silently no-op bake (the body exists but "
                    + "never integrated under the configured gravity)."
            );
            Assert.IsNull(violation, violation);
        }

        // Variant of LoadBakeAndCreate that steps the faller `stepCount` times, calling `onStart` after the
        // creation update (pre-step) and `onEnd` after the last step. For the absolute no-double-step pin.
        IEnumerator LoadBakeAndCreateThenStep(
            string parentScenePath,
            int stepCount,
            System.Action<EntityManager, Entity> onStart,
            System.Action<EntityManager, Entity> onEnd
        )
        {
            yield return LoadBakeAndCreate(
                parentScenePath,
                (world, em, group, faller) =>
                {
                    onStart(em, faller);
                    for (var i = 0; i < stepCount; i++)
                        group.Update();
                    onEnd(em, faller);
                }
            );
        }
    }
}
