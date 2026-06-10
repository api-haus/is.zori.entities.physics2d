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
    /// Phase-3 low-level direct surface, exercised with NO MonoBehaviour and NO subscene bake — bodies are
    /// authored straight onto entities from code and driven through the same step + write-back systems. Two
    /// paths:
    /// <list type="bullet">
    /// <item><b>Direct authoring</b> (<see cref="DirectPhysics2DAuthoring"/>): set
    /// <see cref="PhysicsBody2DDefinition"/> + <see cref="PhysicsShape2D"/> on entities; the per-entity
    /// creation loop turns each into a live body.</item>
    /// <item><b>Bulk creation</b> (<see cref="PhysicsBody2DBatchRequest"/> +
    /// <see cref="PhysicsBody2DBatchCreationSystem"/>): one request creates N identical bodies in a single
    /// <c>CreateBodyBatch</c> native call.</item>
    /// </list>
    /// Both assert the entities gain a <see cref="PhysicsBody2D"/> and actually simulate (fall under gravity),
    /// with no NaN/Inf — the disqualifiers the design names for the direct/bulk path.
    /// </summary>
    /// <remarks>
    /// Each test runs in a DEDICATED, disposable <see cref="World"/> — not the default injection world — so
    /// the live Box2D bodies and the owning <c>PhysicsWorld</c> are torn down on <c>world.Dispose()</c> and
    /// leave zero residue for the next PlayMode test. Driving the default world here would leak bodies (an
    /// entity destroy does not destroy its Box2D body) and desync the write-back's bulk read against the
    /// next test's query. The isolated world holds the package's three FixedStep systems plus a
    /// <see cref="FixedStepSimulationSystemGroup"/> with a per-step rate manager, so one <c>group.Update()</c>
    /// is exactly one fixed step.
    /// </remarks>
    public sealed class DirectAndBatchPathValidation
    {
        const float Dt = 1f / 60f;
        const int Steps = 90;

        // Build a fresh world holding only what the physics step needs: the world/step system, the batch
        // creation system, and the write-back system, inside a FixedStepSimulationSystemGroup driven one
        // step per Update.
        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("Physics2DDirectTestWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            // Create the three package systems and slot them into the fixed-step group in their declared
            // order (the [UpdateInGroup]/[UpdateAfter]/[UpdateBefore] attributes drive sorting).
            var worldSys = world.GetOrCreateSystem<PhysicsWorld2DSystem>();
            var batchSys = world.GetOrCreateSystem<PhysicsBody2DBatchCreationSystem>();
            var writeBackSys = world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>();
            fixedGroup.AddSystemToUpdateList(worldSys);
            fixedGroup.AddSystemToUpdateList(batchSys);
            fixedGroup.AddSystemToUpdateList(writeBackSys);
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }

        [UnityTest]
        public IEnumerator DirectAuthoring_BodiesGetCreatedAndFall_NoNaN()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // Author 8 dynamic circle bodies directly — no MonoBehaviour, no Baker, no Rigidbody2D.
            const int N = 8;
            var entities = new Entity[N];
            var startY = new float[N];
            for (var i = 0; i < N; i++)
            {
                startY[i] = 10f + i;
                var body = new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = new float2(i * 1.5f, startY[i]),
                    useAutoMass = true,
                };
                var shape = new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                };
                entities[i] = DirectPhysics2DAuthoring.Create(em, body, shape);
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
                Assert.Less(
                    y,
                    startY[i] - 0.5f,
                    $"Direct-authored body {i} did not fall: startY={startY[i]}, y={y}."
                );
            }

            Debug.Log($"[PHYSICS2D-DIRECT] {N} direct-authored bodies created + fell, no NaN.");

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator BatchCreation_NIdenticalBodiesGetCreatedAndFall_NoNaN()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // One request → N identical dynamic circle bodies in a single CreateBodyBatch native call.
            const int N = 64;
            var requestEntity = em.CreateEntity();
            em.AddComponentData(
                requestEntity,
                new PhysicsBody2DBatchRequest
                {
                    count = N,
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    radius = 0.25f,
                    density = 1f,
                    spawnMin = new float2(-5f, 20f),
                    spawnMax = new float2(5f, 30f),
                    seed = 0x12345u,
                }
            );

            // The batch entities carry PhysicsBody2D + LocalToWorld; query both so the pose read is legal.
            var batchQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            // First update: PhysicsWorld2DSystem ensures the world, then the batch system consumes the
            // request and bulk-creates the N bodies in one CreateBodyBatch call.
            group.Update();

            Assert.IsFalse(
                em.Exists(requestEntity),
                "The batch request entity was not consumed — PhysicsBody2DBatchCreationSystem did not run."
            );
            Assert.AreEqual(
                N,
                batchQuery.CalculateEntityCount(),
                $"Batch creation made {batchQuery.CalculateEntityCount()} bodies, expected {N}. "
                    + "CreateBodyBatch did not produce one entity per body."
            );

            // Pre-step: every body spawned at a finite pose inside the scatter AABB; record the highest Y.
            var preMaxY = float.NegativeInfinity;
            using (var ltws = batchQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp))
                for (var i = 0; i < ltws.Length; i++)
                {
                    var p = ltws[i].Position;
                    Assert.IsFalse(
                        isnan(p.x) || isnan(p.y) || isinf(p.x) || isinf(p.y),
                        $"Batch body {i} spawned at NaN/Inf: {p}."
                    );
                    preMaxY = max(preMaxY, p.y);
                }

            for (var f = 0; f < Steps; f++)
                group.Update();

            var postMaxY = float.NegativeInfinity;
            var anyNaN = false;
            using (var ltws = batchQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp))
                for (var i = 0; i < ltws.Length; i++)
                {
                    var p = ltws[i].Position;
                    if (isnan(p.x) || isnan(p.y) || isinf(p.x) || isinf(p.y))
                        anyNaN = true;
                    postMaxY = max(postMaxY, p.y);
                }

            Assert.IsFalse(anyNaN, "A batch-created body produced NaN/Inf after stepping.");
            Assert.Less(
                postMaxY,
                preMaxY,
                $"Batch bodies did not fall: preMaxY={preMaxY}, postMaxY={postMaxY}."
            );

            Debug.Log(
                $"[PHYSICS2D-BATCH] {N} bodies bulk-created via CreateBodyBatch + fell "
                    + $"(preMaxY={preMaxY:F2} → postMaxY={postMaxY:F2}), no NaN."
            );

            world.Dispose();
            yield break;
        }
    }
}
