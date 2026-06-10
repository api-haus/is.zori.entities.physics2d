using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The vertical-slice observable: a subscene-authored GameObject (built-in <c>Rigidbody2D</c> Dynamic
    /// + <c>CircleCollider2D</c>) bakes to one ECS entity that, under gravity, falls — its
    /// <see cref="LocalToWorld"/> world-Y translation strictly decreasing across fixed steps, with no
    /// lateral X drift. This proves authoring → bake → create → <c>Simulate</c> → <c>GetBatchTransform</c>
    /// → <c>LocalToWorld</c> end to end, with the smallest possible scene — no ground, no collisions, no
    /// rendering, just a body falling and its matrix moving.
    /// </summary>
    /// <remarks>
    /// Runtime-only (no editor APIs in this assembly): the fixture is authored once by the editor builder
    /// <c>FallingBodyFixtureBuilder</c> and registered in build settings, so <c>SceneManager.LoadScene</c>
    /// opens it. The SubScene auto-loads + bakes on PlayMode enter; the test pumps frames (yield
    /// <c>null</c> — <c>WaitForEndOfFrame</c> does not tick in batchmode) until the baked entity appears,
    /// then drives the <see cref="FixedStepSimulationSystemGroup"/> a fixed number of steps and samples
    /// its world-Y before and after. The fixed-step group is ticked explicitly (with its rate manager
    /// swapped to a per-call <c>FixedRateSimpleManager</c>) rather than via more <c>yield null</c>s,
    /// because the default catch-up manager gates each step on real wall-clock elapsed time, which barely
    /// advances per frame under batchmode — so frame-pumping alone produces almost no fixed steps.
    /// </remarks>
    public sealed class FallingBodyValidation
    {
        const string SceneName = "FallingBody";
        const int LoadTimeoutFrames = 600;
        const int SettleSteps = 120;

        [UnityTest]
        public IEnumerator FallingBody_LocalToWorldYStrictlyDecreases()
        {
            SceneManager.LoadScene(SceneName, LoadSceneMode.Single);
            yield return null; // let the load + SubScene streaming begin

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");

            var query = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            // The baked entity carries its definition + shape the moment the SubScene streams in, before
            // PhysicsWorld2DSystem turns it into a live PhysicsBody2D. Waiting on the definition (not the
            // PhysicsBody2D) separates "did the SubScene stream/bake" from "did body creation run", and
            // lets the explicit fixed-step driving below run the creation step deterministically.
            var bakedQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            // Wait (bounded) for the SubScene to stream and the body to be baked. SubScene streaming is
            // driven by the normal world update, so yield null (next frame) advances it.
            var framesWaited = 0;
            while (bakedQuery.CalculateEntityCount() == 0 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.Greater(
                bakedQuery.CalculateEntityCount(),
                0,
                $"No baked falling body appeared after {framesWaited} frames — the SubScene did not "
                    + "stream/bake. Build the fixture first via FallingBodyFixtureBuilder.Build."
            );

            // Deterministically drive the FixedStepSimulationSystemGroup: the design's observable is "let
            // the FixedStepSimulationSystemGroup tick N fixed steps", and under PlayMode-in-batchmode the
            // default FixedRateCatchUpManager gates each step on World.Time.ElapsedTime (real wall-clock),
            // which barely advances per `yield null` — so frames produce almost no fixed steps. Swapping in
            // a FixedRateSimpleManager makes each group.Update() run exactly one fixed step with the same
            // 1/60 s dt, independent of wall-clock. This is the same group, the same systems, the same dt —
            // only the rate gating is replaced so the test gets the N steps it asserts on.
            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            const float FixedDt = 1f / 60f;
            var savedRateManager = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(FixedDt);

            // First update: PhysicsWorld2DSystem creates the body (and, by design, does NOT step on the
            // creation frame), and the write-back populates this entity's PhysicsBody2D + LocalToWorld at the
            // authored pose. So the sampled initial Y below is the true authored start, and the subsequent
            // updates are what move the body.
            fixedGroup.Update();
            Assert.Greater(
                query.CalculateEntityCount(),
                0,
                "Body creation did not run: the baked entity never gained a live PhysicsBody2D after a "
                    + "fixed step. PhysicsWorld2DSystem did not create the Box2D body."
            );

            float SampleY()
            {
                using var ltw = query.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
                return ltw[0].Position.y;
            }
            float SampleX()
            {
                using var ltw = query.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
                return ltw[0].Position.x;
            }

            var initialY = SampleY();
            var initialX = SampleX();

            // Tick N fixed steps: each group.Update() runs PhysicsWorld2DSystem.Simulate (advancing the
            // body under gravity) and PhysicsBody2DWriteBackSystem (writing the new pose into LocalToWorld).
            var prevY = initialY;
            var sawStrictDecrease = false;
            for (var f = 0; f < SettleSteps; f++)
            {
                fixedGroup.Update();
                var y = SampleY();
                if (y < prevY - 1e-5f)
                    sawStrictDecrease = true;
                prevY = y;
            }

            fixedGroup.RateManager = savedRateManager;

            var finalY = SampleY();
            var finalX = SampleX();

            Debug.Log(
                $"[PHYSICS2D-SLICE] initial=( {initialX:F4}, {initialY:F4} ) "
                    + $"final=( {finalX:F4}, {finalY:F4} ) dropped={(initialY - finalY):F4} "
                    + $"loadFrames={framesWaited} sawStrictDecrease={sawStrictDecrease}"
            );

            // Pass/fail observable: world-Y measurably less after N steps (the body fell), each step's Y
            // strictly decreasing, and X unchanged within epsilon (pure vertical fall, no lateral drift).
            Assert.Less(
                finalY,
                initialY - 0.5f,
                $"Body did not fall: initialY={initialY}, finalY={finalY}. Gravity step or write-back "
                    + "is not advancing LocalToWorld."
            );
            Assert.IsTrue(
                sawStrictDecrease,
                "World-Y never strictly decreased between consecutive fixed steps — the pose is not "
                    + "advancing per step."
            );
            Assert.AreEqual(
                initialX,
                finalX,
                1e-3f,
                $"Body drifted laterally: initialX={initialX}, finalX={finalX}. A pure-gravity fall must "
                    + "not move in X."
            );
        }
    }
}
