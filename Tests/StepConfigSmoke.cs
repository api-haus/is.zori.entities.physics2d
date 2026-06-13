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
    /// Phase-11 smoke for the simulation-config surface (<see cref="PhysicsWorld2DConfig"/> baked from
    /// <c>PhysicsStep2DAuthoring</c>). Three minimal mechanism witnesses (the hard GameObject-parity e2e gate
    /// is a separate validating agent's deliverable):
    /// <list type="bullet">
    /// <item><b>No config → default gravity.</b> A world with NO <see cref="PhysicsWorld2DConfig"/> singleton
    /// drops a Dynamic body at the Box2D default gravity (-9.81 m/s²) — proves the backward-compatible
    /// <c>defaultDefinition</c> fallback is intact.</item>
    /// <item><b>Config → its gravity.</b> A world with a <see cref="PhysicsWorld2DConfig"/> carrying a
    /// non-default gravity drops the same body at THAT gravity, not the default — proves the system reads the
    /// singleton and applies it at world creation.</item>
    /// <item><b>Proportionality.</b> The two falls are in proportion to their gravities (½·g·t²), so the
    /// configured value is the one in force, not a coincidence.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/> holding the package's FixedStep systems,
    /// driven one fixed step per <c>group.Update()</c> at a deterministic 1/60 s (the rate-manager swap from
    /// <c>FallingBodyValidation</c>, so the step count is exact rather than wall-clock-gated). The config
    /// singleton is created BEFORE the first <c>group.Update()</c>, so it exists when
    /// <c>PhysicsWorld2DSystem</c> creates the world. Bodies are authored directly via
    /// <see cref="DirectPhysics2DAuthoring"/>; the first <c>group.Update()</c> creates the body (no step), each
    /// later one steps once.
    /// </remarks>
    public sealed class StepConfigSmoke
    {
        const float Dt = 1f / 60f;
        const int FallSteps = 60; // 1.0 s of fall.
        const float StartY = 100f; // raised so a slow-gravity body still falls measurably without hitting nothing.

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("Physics2DStepConfigSmokeWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }

        static Entity SpawnFaller(EntityManager em, float startY)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = new float2(0f, startY),
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

        // Drop a faller for FallSteps fixed steps and return how far it fell on the Y axis.
        static float MeasureFall(World world, FixedStepSimulationSystemGroup group, Entity entity)
        {
            var em = world.EntityManager;
            group.Update(); // creates the body (no step)
            var body = em.GetComponentData<PhysicsBody2D>(entity).body;
            Assert.IsTrue(body.isValid, "Body was not created on the first update.");
            var y0 = ((float2)(Vector2)body.position).y;
            for (var i = 0; i < FallSteps; i++)
                group.Update();
            var y1 = ((float2)(Vector2)body.position).y;
            return y0 - y1; // positive = fell down
        }

        [UnityTest]
        public IEnumerator NoConfig_FallsAtDefaultGravity()
        {
            var world = MakePhysicsWorld(out var group);
            var entity = SpawnFaller(world.EntityManager, StartY);

            // No PhysicsWorld2DConfig singleton authored — must use the Box2D defaultDefinition (g = -9.81).
            var fell = MeasureFall(world, group, entity);

            var t = FallSteps * Dt;
            var expected = 0.5f * 9.81f * t * t; // ≈ 4.905 m over 1.0 s
            Assert.That(
                fell,
                Is.EqualTo(expected).Within(0.15f * expected),
                $"No-config world did not fall at the Box2D default gravity. fell={fell:F3} m, expected≈{expected:F3} m "
                    + "(½·9.81·t²). The defaultDefinition fallback is not intact."
            );

            Debug.Log(
                $"[PHYSICS2D-STEPCFG] no-config fall over {t:F2}s = {fell:F4} m (expected≈{expected:F3} at g=-9.81)."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator Config_FallsAtConfiguredGravity_NotDefault()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // Author a config singleton with a clearly non-default gravity BEFORE the first update.
            const float ConfigG = -2.5f; // distinctly weaker than the -9.81 default.
            var cfg = PhysicsWorld2DConfig.Default;
            cfg.gravity = new float2(0f, ConfigG);
            em.CreateSingleton(cfg);

            var entity = SpawnFaller(em, StartY);
            var fell = MeasureFall(world, group, entity);

            var t = FallSteps * Dt;
            var expectedConfig = 0.5f * 2.5f * t * t; // ≈ 1.25 m
            var expectedDefault = 0.5f * 9.81f * t * t; // ≈ 4.905 m — must NOT be this

            Assert.That(
                fell,
                Is.EqualTo(expectedConfig).Within(0.15f * expectedConfig),
                $"Configured-gravity world did not fall at g={ConfigG}. fell={fell:F3} m, expected≈{expectedConfig:F3} m. "
                    + "PhysicsWorld2DSystem did not read the PhysicsWorld2DConfig singleton at world creation."
            );
            Assert.Less(
                fell,
                0.5f * expectedDefault,
                $"Configured-gravity world fell {fell:F3} m — too close to the default-gravity {expectedDefault:F3} m. "
                    + "The config gravity did not override the Box2D default."
            );

            Debug.Log(
                $"[PHYSICS2D-STEPCFG] config g={ConfigG} fall over {t:F2}s = {fell:F4} m (expected≈{expectedConfig:F3}, default would be {expectedDefault:F3})."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator Config_FallScalesWithGravity()
        {
            // Two configured worlds at different gravities fall in proportion (½·g·t² ⇒ ratio = g-ratio),
            // confirming the configured value is the operative one rather than a fixed default.
            const float Ga = -2.0f;
            const float Gb = -8.0f; // 4× Ga

            var worldA = MakePhysicsWorld(out var groupA);
            var cfgA = PhysicsWorld2DConfig.Default;
            cfgA.gravity = new float2(0f, Ga);
            worldA.EntityManager.CreateSingleton(cfgA);
            var fellA = MeasureFall(worldA, groupA, SpawnFaller(worldA.EntityManager, StartY));

            var worldB = MakePhysicsWorld(out var groupB);
            var cfgB = PhysicsWorld2DConfig.Default;
            cfgB.gravity = new float2(0f, Gb);
            worldB.EntityManager.CreateSingleton(cfgB);
            var fellB = MeasureFall(worldB, groupB, SpawnFaller(worldB.EntityManager, StartY));

            var ratio = fellB / fellA;
            Assert.That(
                ratio,
                Is.EqualTo(4f).Within(0.4f),
                $"Fall did not scale with configured gravity. fellA(g={Ga})={fellA:F3} m, fellB(g={Gb})={fellB:F3} m, "
                    + $"ratio={ratio:F2} (expected ≈4). The two configs are not each in force."
            );

            Debug.Log(
                $"[PHYSICS2D-STEPCFG] scaling: g={Ga}→{fellA:F4} m, g={Gb}→{fellB:F4} m, ratio={ratio:F2} (expected≈4)."
            );

            worldA.Dispose();
            worldB.Dispose();
            yield break;
        }
    }
}
