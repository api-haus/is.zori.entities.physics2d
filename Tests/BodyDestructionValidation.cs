using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif
using UnityEngine;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-4 per-entity body-destruction smoke. Proves a body's lifetime tracks its entity's: destroying a
    /// physics entity frees its Box2D body (and the body's shape) from the <see cref="PhysicsWorld"/> on the
    /// next fixed step, the world's live body count returns to baseline after churn, and no stale body for a
    /// dead entity is simulated. The hard adversarial e2e gate (high churn, many bodies, joints, baseline
    /// distribution) is a separate validating agent's deliverable; this is the minimal mechanism witness.
    /// </summary>
    /// <remarks>
    /// Runs in a DEDICATED disposable <see cref="World"/> (not the default injection world), holding the
    /// package's three FixedStep systems — <see cref="PhysicsWorld2DSystem"/>,
    /// <see cref="PhysicsBody2DCleanupSystem"/>, <see cref="PhysicsBody2DWriteBackSystem"/> — in a
    /// <see cref="FixedStepSimulationSystemGroup"/> driven
    /// one step per <c>group.Update()</c>. The live body count is read directly from the Box2D world via
    /// <c>PhysicsWorld.GetBodies</c>, the authoritative witness of what the simulation actually holds (not an
    /// ECS-side proxy).
    /// </remarks>
    public sealed class BodyDestructionValidation
    {
        const float Dt = 1f / 60f;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DDestructionTestWorld", out group, Dt);

        // Live body count straight from the Box2D world the package owns.
        static int LiveBodyCount(EntityManager em)
        {
            var singletonQuery = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            var world = singletonQuery.GetSingleton<PhysicsWorldSingleton2D>().world;
            if (!world.isValid)
                return 0;
            using var bodies = world.GetBodies(Allocator.Temp);
            return bodies.Length;
        }

        [UnityTest]
        public IEnumerator DestroyingEntity_FreesItsBody_AndCountReturnsToBaseline()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // Author one dynamic circle directly (no MonoBehaviour, no subscene), then create the body.
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = new float2(0f, 10f),
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

            // First update ensures the world + creates the body (no step on the creation frame).
            group.Update();
            Assert.IsTrue(
                em.HasComponent<PhysicsBody2D>(entity),
                "Entity did not gain a live PhysicsBody2D — body creation did not run."
            );
            Assert.IsTrue(
                em.HasComponent<PhysicsBody2DCleanup>(entity),
                "Entity did not gain a PhysicsBody2DCleanup — the creation site did not add the cleanup "
                    + "component, so its body would leak on destroy."
            );

            var handle = em.GetComponentData<PhysicsBody2D>(entity).body;
            Assert.IsTrue(handle.isValid, "The created PhysicsBody handle is invalid.");

            var baseline = LiveBodyCount(em); // bodies present before the churned entity existed = 0
            Assert.AreEqual(1, baseline, $"Expected exactly one live Box2D body after creation, found {baseline}.");

            // Step a few times so the body is genuinely being simulated, then destroy the entity.
            for (var f = 0; f < 5; f++)
                group.Update();
            Assert.IsTrue(handle.isValid, "The body went invalid mid-simulation before any destroy.");

            em.DestroyEntity(entity);

            // The regular PhysicsBody2D is stripped immediately; the cleanup ghost retains the handle.
            Assert.IsFalse(
                em.HasComponent<PhysicsBody2D>(entity) && em.Exists(entity),
                "Destroyed entity still carries the regular PhysicsBody2D."
            );

            // One more update: PhysicsBody2DCleanupSystem runs before the step, frees the ghost body, and
            // removes the cleanup component — reclaiming the entity.
            group.Update();

            Assert.IsFalse(
                handle.isValid,
                "The despawned entity's Box2D body is still valid — the body was not freed (the leak)."
            );
            Assert.IsFalse(
                em.Exists(entity),
                "The ghost entity was not reclaimed — PhysicsBody2DCleanupSystem did not remove the "
                    + "cleanup component."
            );

            var after = LiveBodyCount(em);
            Assert.AreEqual(
                0,
                after,
                $"Live body count did not return to baseline after churn: expected 0, found {after} "
                    + "(a stale body for the dead entity is still in the world)."
            );

            // Further steps must not resurrect, NaN, or re-simulate the dead body.
            for (var f = 0; f < 30; f++)
                group.Update();
            Assert.AreEqual(
                0,
                LiveBodyCount(em),
                "A body reappeared in the world after the despawned entity was cleaned up."
            );

            Debug.Log(
                "[PHYSICS2D-DESTROY] created 1 body, stepped, destroyed entity → body freed on next step, "
                    + "live count 1 → 0, handle invalid, ghost reclaimed."
            );

            world.Dispose();
            yield break;
        }
    }
}
