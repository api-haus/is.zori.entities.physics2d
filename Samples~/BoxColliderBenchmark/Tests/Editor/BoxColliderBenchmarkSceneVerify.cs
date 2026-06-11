using System.Collections;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Samples.Editor
{
    /// <summary>
    /// One-shot PlayMode verification that the AUTHORED demo scene (not the hand-built test world) runs: it loads
    /// <c>BoxColliderBenchmarkDemo.unity</c>, enters Play, lets the SubScene stream + bake, ticks a few seconds,
    /// and asserts the bake armed the config and the spawner sprayed live, renderable physics bodies. This drives
    /// the real scene path the user opens, complementing <c>BoxColliderBenchmarkSmoke</c> (which exercises the
    /// systems in a disposable world). Run via
    /// <c>-runTests -testFilter Zori.Entities.Physics2D.Samples.Editor.BoxColliderBenchmarkSceneVerify</c>.
    /// </summary>
    public sealed class BoxColliderBenchmarkSceneVerify
    {
        [UnityTest]
        public IEnumerator DemoScene_Plays_SpraysRenderableBodies()
        {
            EditorSceneManager.OpenScene(
                BoxColliderBenchmarkSceneBuilder.HostScenePath,
                OpenSceneMode.Single
            );

            yield return new EnterPlayMode();

            // The benchmark HUD is a host-scene MonoBehaviour (not in the SubScene / ECS) — confirm it loaded and
            // is live, so the overlay is present and drawing on Play.
            var hud = Object.FindFirstObjectByType<BenchmarkGUI>();
            Assert.IsNotNull(hud, "the host scene should carry a live BenchmarkGUI overlay");

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "default world should exist in Play mode");
            var em = world.EntityManager;

            // Wait for the SubScene to stream + bake the config singleton (bounded — fail loudly if it never bakes).
            var configQuery = em.CreateEntityQuery(typeof(BoxColliderBenchmarkConfig));
            var waited = 0;
            while (configQuery.CalculateEntityCount() == 0 && waited++ < 600)
                yield return null;
            Assert.AreEqual(
                1,
                configQuery.CalculateEntityCount(),
                "the SubScene bake should have emitted exactly one BoxColliderBenchmarkConfig singleton"
            );
            var totalLimit = configQuery.GetSingleton<BoxColliderBenchmarkConfig>().spawnedTotalLimit;

            // Every sprayed quad carries the Entities.Graphics render components off the physics LocalToWorld; the
            // static floor (no render components) does not. The spray is RATE-paced (quads/second against the
            // frame dt), so under headless batchmode the wall-clock is too fast to drain the full demo limit in a
            // bounded tick budget — and draining the limit is not what proves the scene runs. Tick until a
            // meaningful batch of renderable quads exists (well past one frame's worth), which proves the bake
            // armed the spray, the spawner is actively instantiating, and the instances are renderable off the
            // physics pose. Bounded so a genuine stall (zero progress) still fails loudly.
            const int ProgressTarget = 512; // > the demo's per-frame ceiling, < the full demo limit
            var renderableQuery = em.CreateEntityQuery(
                new EntityQueryDesc
                {
                    All = new[]
                    {
                        ComponentType.ReadOnly<PhysicsBody2D>(),
                        ComponentType.ReadOnly<Unity.Rendering.MaterialMeshInfo>(),
                        ComponentType.ReadOnly<Unity.Transforms.LocalToWorld>(),
                    },
                    None = new[] { ComponentType.ReadOnly<Prefab>() },
                }
            );
            var target = math.min(ProgressTarget, totalLimit);
            var ticks = 0;
            while (renderableQuery.CalculateEntityCount() < target && ticks++ < 4000)
                yield return null;
            var renderable = renderableQuery.CalculateEntityCount();
            Assert.GreaterOrEqual(
                renderable,
                target,
                "the rate-paced spray should produce a batch of renderable quads "
                    + "(MaterialMeshInfo + LocalToWorld off the physics pose)"
            );
            Assert.LessOrEqual(
                renderable,
                totalLimit,
                "the spray must never exceed the authored spawnedTotalLimit"
            );

            // Each renderable quad's LocalToWorld (the transform the renderer reads) is finite — the physics
            // write-back drives a real pose, not NaN.
            using (
                var l2ws = renderableQuery.ToComponentDataArray<Unity.Transforms.LocalToWorld>(
                    Unity.Collections.Allocator.Temp
                )
            )
            {
                for (var i = 0; i < l2ws.Length; i++)
                    Assert.IsTrue(
                        math.all(math.isfinite(l2ws[i].Position)),
                        "every renderable quad's LocalToWorld position must be finite"
                    );
            }

            // The live, non-prefab bodies are the sprayed quads PLUS the authored static floor (the code-built
            // prefab template also carries PhysicsBody2D but is Prefab-tagged, so excluded). So live > renderable.
            var liveBodies = em.CreateEntityQuery(
                    new EntityQueryDesc
                    {
                        All = new[] { ComponentType.ReadOnly<PhysicsBody2D>() },
                        None = new[] { ComponentType.ReadOnly<Prefab>() },
                    }
                )
                .CalculateEntityCount();
            Assert.Greater(
                liveBodies,
                renderable,
                "live bodies should be the sprayed quads PLUS the static floor body"
            );
            Debug.Log(
                $"[BoxColliderBenchmark] demo-scene play check: totalLimit={totalLimit} live={liveBodies} renderable={renderable}"
            );

            yield return new ExitPlayMode();
        }
    }
}
