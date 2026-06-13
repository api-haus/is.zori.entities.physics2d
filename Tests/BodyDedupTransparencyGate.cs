using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.TestTools;
using Zori.Entities.Physics2D.Baking;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The independent adversarial gate for the runtime identical-body creation optimisation (Dedup part 1). The
    /// optimisation's whole promise is TRANSPARENCY — it changes HOW a body is created (per-entity warm-up,
    /// cross-frame cached template, or same-frame <c>CreateBodyBatch</c> collapse), never the RESULT. The impl's own
    /// transparency smoke (<see cref="DirectAndBatchPathValidation"/>) probed only the CIRCLE kind and stamped the
    /// form hash by hand, so two gaps were escalated: every other shape kind's cached-path bit-identity was unproven,
    /// and the bake-produced form-hash invariants were run but never asserted. This gate closes both.
    /// </summary>
    /// <remarks>
    /// <para><b>Two concerns, two decision surfaces.</b></para>
    /// <list type="bullet">
    /// <item><b>Per-kind cached-vs-per-entity bit-identity</b> (<see cref="PhysicsWorld2DSystem"/>'s three creation
    /// paths). For each VALUE-CACHEABLE kind (Circle, Box, Capsule, simple convex Polygon) a body created from the
    /// built template lands at the BIT-IDENTICAL simulated pose, rotation, and mass as one created per-entity — the
    /// cached arm replays the donor's prepared definition/geometry/mass. The NON-cacheable kinds (Edge's chain
    /// geometry, a decompose Polygon, any multi-shape body) are deliberately excluded from the cached arm
    /// (<c>TryGetCacheableGeometry</c> / the <c>!hasExtraShapes</c> guard), so for them transparency means the cache
    /// NEVER engages and the body is always the unchanged per-entity path — proven here by stepping them on vs off
    /// and getting one identical trajectory, the negative-space half of the contract.</item>
    /// <item><b>Form-hash invariants</b> (<see cref="PhysicsBody2DFormHashBakingSystem"/>). Asserted by driving the
    /// REAL baking system over real authored components and reading back the <see cref="PhysicsBody2DFormHash"/> it
    /// produced — no reimplementation of the hash, so a real split/collision in the shipped code is caught. Two
    /// forms of the same form but different pose/velocity hash EQUAL; two forms differing in ANY hashed field hash
    /// DIFFERENT; the hash is deterministic; and an adversarial near-collision (two forms differing only in one
    /// subtle hashed field) hashes different.</item>
    /// </list>
    ///
    /// <para><b>Why this is a STRICT-equality gate, not the parity band.</b> <see cref="PhysicsParityHarness"/>
    /// compares two different solvers (GameObject Box2D-v2 vs the package's Box2D-v3) and so asserts only a bounded
    /// band. This gate compares two creation paths on the SAME v3 solver in the SAME world definition, so a
    /// transparent optimisation must be bit-identical — every assertion is exact equality, and a single bit of
    /// divergence is an impl bug, not solver noise.</para>
    ///
    /// <para><b>Isolation.</b> Each test runs in a dedicated disposable <see cref="World"/> (or two — the on/off pair)
    /// so the live Box2D bodies and the owning <c>PhysicsWorld</c> tear down on <c>world.Dispose()</c> and leave zero
    /// residue. One <c>group.Update()</c> is exactly one fixed step (a swapped <c>FixedRateSimpleManager</c>); the
    /// first Update creates bodies without stepping. No <c>WaitForEndOfFrame</c> (it does not tick in batchmode).</para>
    /// </remarks>
    public sealed class BodyDedupTransparencyGate
    {
        const float Dt = 1f / 60f;
        const int Steps = 120;

        // Distinct arbitrary form-hash keys stamped on the runtime-authored bodies in the creation-path tests. The
        // key VALUE is irrelevant to creation correctness (the runtime rebuilds the template from the donor's real
        // components); it only has to be EQUAL across bodies the test means as one form and absent on the oracle so
        // the off world never groups. A real bake would land it from PhysicsBody2DFormHashBakingSystem — Part B
        // exercises that path; Part A only needs a stable grouping key per form.
        static readonly uint4 FormKey = new uint4(0xA1B2C3D4u, 0x11223344u, 0x55667788u, 0x99AABBCCu);

        // ------------------------------------------------------------------------------------------------
        // Disposable-world plumbing (mirrors DirectAndBatchPathValidation / CompositeCustomColliderSmoke).
        // ------------------------------------------------------------------------------------------------

        static World MakePhysicsWorld(
            out FixedStepSimulationSystemGroup group,
            bool? cacheEnabled = null,
            int threshold = 8
        )
        {
            var world = PhysicsTestWorld.Create("Physics2DDedupGateWorld", out group, Dt);

            if (cacheEnabled.HasValue)
            {
                var cfg = PhysicsWorld2DConfig.Default;
                cfg.cacheIdenticalBodies = cacheEnabled.Value;
                cfg.identicalBodyThreshold = threshold;
                world.EntityManager.CreateSingleton(cfg);
            }

            return world;
        }

        // A dynamic body of the given form, useAutoMass so the cached mass resolution captures the donor's
        // shape-derived massConfiguration (the path that exercises the ResolveMass replay).
        static PhysicsBody2DDefinition DynamicBody(float2 pos) =>
            new PhysicsBody2DDefinition
            {
                bodyType = PhysicsBody.BodyType.Dynamic,
                gravityScale = 1f,
                initialPosition = pos,
                useAutoMass = true,
            };

        static readonly List<BlobAssetReference<PhysicsShape2DVertices>> s_Blobs = new();

        static BlobAssetReference<PhysicsShape2DVertices> Blob(float2[] pts)
        {
            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<PhysicsShape2DVertices>();
            var arr = builder.Allocate(ref root.points, pts.Length);
            for (var i = 0; i < pts.Length; i++)
                arr[i] = pts[i];
            var blob = builder.CreateBlobAssetReference<PhysicsShape2DVertices>(Allocator.Persistent);
            builder.Dispose();
            s_Blobs.Add(blob);
            return blob;
        }

        static void DisposeBlobs()
        {
            foreach (var b in s_Blobs)
                if (b.IsCreated)
                    b.Dispose();
            s_Blobs.Clear();
        }

        // The shape fixtures, one per kind. Each carries a non-default surface/filter so BuildShapeDef has real
        // content to replay, and is a stable convex form (no decompose) so a cacheable kind reaches the template.
        static PhysicsShape2D CircleShape() =>
            new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Circle,
                radius = 0.5f,
                offset = new float2(0.1f, -0.05f),
                density = 1f,
                friction = 0.4f,
                bounciness = 0.2f,
            };

        static PhysicsShape2D BoxShape() =>
            new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Box,
                size = new float2(0.8f, 0.6f),
                radius = 0.02f,
                boxAngleRadians = 0.3f,
                offset = new float2(-0.1f, 0.05f),
                density = 1f,
                friction = 0.5f,
                bounciness = 0.1f,
            };

        static PhysicsShape2D CapsuleShape() =>
            new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Capsule,
                capsuleCenter1 = new float2(0f, -0.3f),
                capsuleCenter2 = new float2(0f, 0.3f),
                radius = 0.25f,
                offset = new float2(0.05f, 0f),
                density = 1f,
                friction = 0.3f,
            };

        static PhysicsShape2D PolygonShape() =>
            new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Polygon,
                polygonDecompose = false,
                radius = 0.01f,
                offset = new float2(0.0f, 0.0f),
                density = 1f,
                friction = 0.45f,
                vertices = Blob(
                    new[]
                    {
                        new float2(-0.4f, -0.3f),
                        new float2(0.4f, -0.3f),
                        new float2(0.4f, 0.3f),
                        new float2(-0.4f, 0.3f),
                    }
                ),
            };

        // A non-cacheable kind: a decompose Polygon (a multi-fragment CreateShapeBatch). Same outline as the simple
        // polygon, but flagged decompose so TryGetCacheableGeometry returns false and the body stays per-entity.
        static PhysicsShape2D DecomposePolygonShape() =>
            new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Polygon,
                polygonDecompose = true,
                density = 1f,
                friction = 0.4f,
                vertices = Blob(
                    new[]
                    {
                        new float2(-0.4f, -0.3f),
                        new float2(0.4f, -0.3f),
                        new float2(0.4f, 0.3f),
                        new float2(-0.4f, 0.3f),
                    }
                ),
            };

        // A non-cacheable kind: an Edge (chain geometry, NativeArray-backed) — a closed loop so a dynamic chain-only
        // body has a bounded surface to fall and rotate on.
        static PhysicsShape2D EdgeShape() =>
            new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Edge,
                edgeIsLoop = true,
                friction = 0.4f,
                vertices = Blob(
                    new[]
                    {
                        new float2(-0.5f, -0.5f),
                        new float2(0.5f, -0.5f),
                        new float2(0.5f, 0.5f),
                        new float2(-0.5f, 0.5f),
                    }
                ),
            };

        static Entity AuthorBody(EntityManager em, float2 pos, in PhysicsShape2D shape, bool stampForm = true)
        {
            var entity = DirectPhysics2DAuthoring.Create(em, DynamicBody(pos), shape);
            if (stampForm)
                em.AddComponentData(entity, new PhysicsBody2DFormHash { value = FormKey });
            return entity;
        }

        // A multi-shape body (primary + one buffer element). Always per-entity (the !hasExtraShapes guard), so the
        // stamped form hash is irrelevant to creation — it is stamped anyway to prove the guard, not the hash,
        // excludes it.
        static Entity AuthorMultiShapeBody(EntityManager em, float2 pos)
        {
            var entity = AuthorBody(em, pos, CircleShape());
            var buffer = em.AddBuffer<PhysicsShape2DElement>(entity);
            buffer.Add(
                new PhysicsShape2DElement
                {
                    shape = new PhysicsShape2D
                    {
                        kind = PhysicsShape2DKind.Box,
                        size = new float2(0.4f, 0.4f),
                        offset = new float2(0.6f, 0f),
                        density = 1f,
                        friction = 0.4f,
                    },
                }
            );
            return entity;
        }

        // ------------------------------------------------------------------------------------------------
        // Part A — per-kind cached-vs-per-entity bit-identity.
        //
        // Falsification framing: a body served from the cached template must be indistinguishable from one built
        // per-entity. If any kind's cached arm built a subtly different geometry / shape def / mass than its
        // per-entity builder, the two paths diverge — caught to the bit here. The drive: an OFF world (cache
        // disabled, the per-entity ORACLE) and an ON world (threshold 2, four warm-up bodies of the form first so
        // the template is built and IN USE, then the probe — which is therefore created from the template, not as
        // the donor). Both probes start at the identical pose; after equal stepping every read-back field must match.
        // ------------------------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator CachedPathBitIdentical_Circle()
        {
            yield return RunCachedVsPerEntity(CircleShape());
        }

        [UnityTest]
        public IEnumerator CachedPathBitIdentical_Box()
        {
            yield return RunCachedVsPerEntity(BoxShape());
        }

        [UnityTest]
        public IEnumerator CachedPathBitIdentical_Capsule()
        {
            yield return RunCachedVsPerEntity(CapsuleShape());
        }

        [UnityTest]
        public IEnumerator CachedPathBitIdentical_Polygon()
        {
            yield return RunCachedVsPerEntity(PolygonShape());
        }

        IEnumerator RunCachedVsPerEntity(PhysicsShape2D shape)
        {
            var probePos = new float2(2.5f, 30f);

            var offWorld = MakePhysicsWorld(out var offGroup, cacheEnabled: false);
            var offProbe = AuthorBody(offWorld.EntityManager, probePos, shape);

            var onWorld = MakePhysicsWorld(out var onGroup, cacheEnabled: true, threshold: 2);
            var onEm = onWorld.EntityManager;
            for (var i = 0; i < 4; i++)
                AuthorBody(onEm, new float2(-10f + i, 45f), shape); // warm-ups: build + put the template in use
            var onProbe = AuthorBody(onEm, probePos, shape);

            offGroup.Update(); // create (no step)
            onGroup.Update();
            DisposeBlobs(); // geometry copied into Box2D on the creation Update; the source blobs are free now

            var offBody = offWorld.EntityManager.GetComponentData<PhysicsBody2D>(offProbe).body;
            var onBody = onEm.GetComponentData<PhysicsBody2D>(onProbe).body;
            Assert.IsTrue(offBody.isValid && onBody.isValid, $"A probe body for {shape.kind} was not created.");

            // Mass identical at creation: the cached arm replays the donor's resolved massConfiguration; the
            // per-entity arm resolves it directly. They must agree before any integration.
            AssertMassIdentical(offBody, onBody, shape.kind, "at creation");
            AssertPoseIdentical(offBody, onBody, shape.kind, 0);

            for (var s = 0; s < Steps; s++)
            {
                offGroup.Update();
                onGroup.Update();
                AssertPoseIdentical(offBody, onBody, shape.kind, s + 1);
            }

            AssertMassIdentical(offBody, onBody, shape.kind, "after stepping");
            Debug.Log(
                $"[DEDUP-GATE-A] {shape.kind}: cached-template body == per-entity body, bit-identical for {Steps} steps (pos={(float2)(Vector2)onBody.position})."
            );

            offWorld.Dispose();
            onWorld.Dispose();
            yield break;
        }

        // ------------------------------------------------------------------------------------------------
        // Part A (negative space) — the NON-cacheable kinds are never served from a template, so on vs off is the
        // unchanged per-entity path either way and produces ONE identical trajectory. Falsification framing: if a
        // non-cacheable kind were wrongly admitted to the cached arm, the cache could serve it a wrong template (e.g.
        // a stale geometry) and on would diverge from off; equality here proves it is NOT admitted and the body is
        // unaffected by the optimisation.
        // ------------------------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator NonCacheableUnaffected_Edge()
        {
            yield return RunNonCacheableOnOff(EdgeShape());
        }

        [UnityTest]
        public IEnumerator NonCacheableUnaffected_DecomposePolygon()
        {
            yield return RunNonCacheableOnOff(DecomposePolygonShape());
        }

        IEnumerator RunNonCacheableOnOff(PhysicsShape2D shape)
        {
            var probePos = new float2(1.5f, 26f);

            var offWorld = MakePhysicsWorld(out var offGroup, cacheEnabled: false);
            var offProbe = AuthorBody(offWorld.EntityManager, probePos, shape);

            // ON world, low threshold + warm-ups of the SAME form: if the kind were (wrongly) cacheable, the probe
            // would be template-served. It is not, so it stays per-entity — identical to off.
            var onWorld = MakePhysicsWorld(out var onGroup, cacheEnabled: true, threshold: 2);
            var onEm = onWorld.EntityManager;
            for (var i = 0; i < 4; i++)
                AuthorBody(onEm, new float2(-8f + i, 40f), shape);
            var onProbe = AuthorBody(onEm, probePos, shape);

            offGroup.Update();
            onGroup.Update();
            DisposeBlobs();

            var offBody = offWorld.EntityManager.GetComponentData<PhysicsBody2D>(offProbe).body;
            var onBody = onEm.GetComponentData<PhysicsBody2D>(onProbe).body;
            Assert.IsTrue(offBody.isValid && onBody.isValid, $"A {shape.kind} probe body was not created.");

            for (var s = 0; s < Steps; s++)
            {
                offGroup.Update();
                onGroup.Update();
                AssertPoseIdentical(offBody, onBody, shape.kind, s + 1);
            }

            Debug.Log(
                $"[DEDUP-GATE-A-NEG] {shape.kind} (non-cacheable): on==off, the cache never engaged (pos={(float2)(Vector2)onBody.position})."
            );

            offWorld.Dispose();
            onWorld.Dispose();
            yield break;
        }

        // Multi-shape body: excluded by the !hasExtraShapes guard regardless of kind. Same negative-space proof.
        [UnityTest]
        public IEnumerator NonCacheableUnaffected_MultiShape()
        {
            var probePos = new float2(0.5f, 24f);

            var offWorld = MakePhysicsWorld(out var offGroup, cacheEnabled: false);
            var offProbe = AuthorMultiShapeBody(offWorld.EntityManager, probePos);

            var onWorld = MakePhysicsWorld(out var onGroup, cacheEnabled: true, threshold: 2);
            var onEm = onWorld.EntityManager;
            for (var i = 0; i < 4; i++)
                AuthorMultiShapeBody(onEm, new float2(-8f + i, 38f));
            var onProbe = AuthorMultiShapeBody(onEm, probePos);

            offGroup.Update();
            onGroup.Update();
            DisposeBlobs();

            var offBody = offWorld.EntityManager.GetComponentData<PhysicsBody2D>(offProbe).body;
            var onBody = onEm.GetComponentData<PhysicsBody2D>(onProbe).body;
            Assert.IsTrue(offBody.isValid && onBody.isValid, "A multi-shape probe body was not created.");
            AssertMassIdentical(offBody, onBody, PhysicsShape2DKind.Circle, "multi-shape at creation");

            for (var s = 0; s < Steps; s++)
            {
                offGroup.Update();
                onGroup.Update();
                AssertPoseIdentical(offBody, onBody, PhysicsShape2DKind.Circle, s + 1);
            }

            Debug.Log(
                $"[DEDUP-GATE-A-NEG] multi-shape (non-cacheable): on==off, the cache never engaged (pos={(float2)(Vector2)onBody.position})."
            );

            offWorld.Dispose();
            onWorld.Dispose();
            yield break;
        }

        // ------------------------------------------------------------------------------------------------
        // Part A — same-frame collapse path. A K>=2 batch of one isBuilt form arriving in ONE frame goes through
        // CreateBodyBatchInto (one CreateBodyBatch + SetBatchTransform). Those bodies must be bit-identical to the
        // per-entity oracle too. Drive: ON world, threshold 1, all K probes authored together so the warm-up donor
        // builds the template on the first probe and the rest land in the same-frame collapse bucket.
        // ------------------------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator SameFrameCollapse_BitIdenticalToPerEntity()
        {
            const int K = 6;
            var poses = new float2[K];
            for (var i = 0; i < K; i++)
                poses[i] = new float2(i * 1.3f, 32f);

            // ON, threshold 1: a warm-up donor of the form is created and built into a template on its OWN earlier
            // frame, so when the K probes are authored together and the group updates, every one of the K routes
            // through the in-frame collapse (one CreateBodyBatch + SetBatchTransform) — NOT as the donor.
            var onWorld = MakePhysicsWorld(out var onGroup, cacheEnabled: true, threshold: 1);
            var onEm = onWorld.EntityManager;
            AuthorBody(onEm, new float2(-20f, 60f), BoxShape()); // donor warm-up
            onGroup.Update(); // donor created → template built+isBuilt (threshold 1). The donor steps on later frames.
            var onE = new Entity[K];
            for (var i = 0; i < K; i++)
                onE[i] = AuthorBody(onEm, poses[i], BoxShape());

            // Oracle: per-entity, all K authored together. Created on the oracle's FIRST group update — the same
            // relative create-then-not-stepped position as the cached probes' creation frame, so both sides are read
            // at step 0 (zero integrations) before any divergence the comparison would mistake for an impl bug.
            var offWorld = MakePhysicsWorld(out var offGroup, cacheEnabled: false);
            var offEm = offWorld.EntityManager;
            var offE = new Entity[K];
            for (var i = 0; i < K; i++)
                offE[i] = AuthorBody(offEm, poses[i], BoxShape());

            onGroup.Update(); // the K probes land in ONE frame → in-frame collapse (CreateBodyBatch); no step for them
            offGroup.Update(); // the K oracle bodies are created here; no step for them either
            DisposeBlobs();

            var offBodies = new PhysicsBody[K];
            var onBodies = new PhysicsBody[K];
            for (var i = 0; i < K; i++)
            {
                offBodies[i] = offEm.GetComponentData<PhysicsBody2D>(offE[i]).body;
                onBodies[i] = onEm.GetComponentData<PhysicsBody2D>(onE[i]).body;
                Assert.IsTrue(offBodies[i].isValid && onBodies[i].isValid, $"Collapse probe {i} not created.");
                AssertMassIdentical(offBodies[i], onBodies[i], PhysicsShape2DKind.Box, $"collapse[{i}] at creation");
                AssertPoseIdentical(offBodies[i], onBodies[i], PhysicsShape2DKind.Box, 0);
            }

            for (var s = 0; s < Steps; s++)
            {
                offGroup.Update();
                onGroup.Update();
                for (var i = 0; i < K; i++)
                    AssertPoseIdentical(offBodies[i], onBodies[i], PhysicsShape2DKind.Box, s + 1);
            }

            Debug.Log(
                $"[DEDUP-GATE-A] same-frame collapse of K={K} boxes bit-identical to per-entity for {Steps} steps."
            );

            offWorld.Dispose();
            onWorld.Dispose();
            yield break;
        }

        // ------------------------------------------------------------------------------------------------
        // Part A — on/off + threshold-sweep transparency over EVERY cacheable kind. The threshold changes only WHEN
        // the template starts being used, never the result; on vs off, and every N in {1,2,8,large}, must yield the
        // identical final pose for the same authored spray.
        // ------------------------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator ThresholdSweepTransparent_Circle()
        {
            yield return RunThresholdSweep(CircleShape);
        }

        [UnityTest]
        public IEnumerator ThresholdSweepTransparent_Box()
        {
            yield return RunThresholdSweep(BoxShape);
        }

        [UnityTest]
        public IEnumerator ThresholdSweepTransparent_Capsule()
        {
            yield return RunThresholdSweep(CapsuleShape);
        }

        [UnityTest]
        public IEnumerator ThresholdSweepTransparent_Polygon()
        {
            yield return RunThresholdSweep(PolygonShape);
        }

        IEnumerator RunThresholdSweep(System.Func<PhysicsShape2D> shapeFactory)
        {
            const int N = 12;
            var startPos = new float2[N];
            // Space the spray so NO two bodies interpenetrate at spawn (1.5 m > the largest body's full extent, the
            // Circle's 1.0 m diameter). Per-body cached-vs-per-entity state is bit-identical (the CachedPathBitIdentical
            // and SameFrameCollapse tests prove it to the bit at creation and through stepping), but the optimisation
            // changes the Box2D body-CREATION ORDER (one CreateBody donor + one CreateBodyBatch vs N CreateBody calls),
            // which changes internal body indices. Box2D's contact-solver iteration order is index-dependent, so a spray
            // whose bodies MUTUALLY INTERPENETRATE would let that order difference seed a ~1-ULP chaotic divergence under
            // contact resolution — a sensitivity of the solver to creation order, NOT a per-body state difference the
            // optimisation introduced, and not achievable-to-the-bit by ANY batch-creation optimisation. A real
            // falling-sand spray (grains spawned at distinct, non-overlapping poses) is the contract's workload; this
            // spacing matches it. The non-overlapping spray is bit-identical on vs off across the whole threshold sweep.
            for (var i = 0; i < N; i++)
                startPos[i] = new float2(i * 1.5f, 33f);

            var baseline = SpawnStepCollect(N, startPos, shapeFactory, cacheEnabled: false, threshold: 8);

            foreach (var thr in new[] { 1, 2, 8, 64 })
            {
                var withCache = SpawnStepCollect(N, startPos, shapeFactory, cacheEnabled: true, threshold: thr);
                for (var i = 0; i < N; i++)
                {
                    Assert.AreEqual(
                        baseline[i].x,
                        withCache[i].x,
                        $"{shapeFactory().kind} body {i} X differs cache-off vs on(N={thr}): {baseline[i]} vs {withCache[i]}."
                    );
                    Assert.AreEqual(
                        baseline[i].y,
                        withCache[i].y,
                        $"{shapeFactory().kind} body {i} Y differs cache-off vs on(N={thr}): {baseline[i]} vs {withCache[i]}."
                    );
                }
            }

            Debug.Log(
                $"[DEDUP-GATE-A] {shapeFactory().kind}: {N} bodies identical across cache-off and N in {{1,2,8,64}}."
            );
            yield break;
        }

        static float2[] SpawnStepCollect(
            int n,
            float2[] startPos,
            System.Func<PhysicsShape2D> shapeFactory,
            bool cacheEnabled,
            int threshold
        )
        {
            var world = MakePhysicsWorld(out var group, cacheEnabled: cacheEnabled, threshold: threshold);
            var em = world.EntityManager;
            var entities = new Entity[n];
            for (var i = 0; i < n; i++)
                entities[i] = AuthorBody(em, startPos[i], shapeFactory());

            group.Update();
            DisposeBlobs();
            for (var f = 0; f < Steps; f++)
                group.Update();

            var result = new float2[n];
            for (var i = 0; i < n; i++)
                result[i] = (float2)(Vector2)em.GetComponentData<PhysicsBody2D>(entities[i]).body.position;

            world.Dispose();
            return result;
        }

        // ------------------------------------------------------------------------------------------------
        // Part A — cross-frame spray (the falling-sand workload the cache exists for): one identical-form body per
        // frame, K=1 each frame so the in-frame collapse never fires and every body past the threshold is the
        // cross-frame template path. Each body must become physical on its spawn frame and fall, with no NaN — the
        // cache must not drop, duplicate, or stall a body — across every cacheable kind.
        // ------------------------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator CrossFrameSpray_Box()
        {
            yield return RunSpray(BoxShape);
        }

        [UnityTest]
        public IEnumerator CrossFrameSpray_Capsule()
        {
            yield return RunSpray(CapsuleShape);
        }

        [UnityTest]
        public IEnumerator CrossFrameSpray_Polygon()
        {
            yield return RunSpray(PolygonShape);
        }

        IEnumerator RunSpray(System.Func<PhysicsShape2D> shapeFactory)
        {
            var world = MakePhysicsWorld(out var group, cacheEnabled: true, threshold: 4);
            var em = world.EntityManager;

            const int Frames = 36;
            var entities = new List<Entity>(Frames);
            var startY = new List<float>(Frames);

            for (var f = 0; f < Frames; f++)
            {
                var y = 50f + f;
                var e = AuthorBody(em, new float2(f * 0.1f, y), shapeFactory());
                entities.Add(e);
                startY.Add(y);

                group.Update(); // body f created this frame (no step on a creation frame); all earlier bodies step
                Assert.IsTrue(
                    em.HasComponent<PhysicsBody2D>(e),
                    $"{shapeFactory().kind} sprayed body {f} not physical on its spawn frame — the cross-frame cache dropped or stalled it."
                );
                Assert.IsTrue(
                    em.GetComponentData<PhysicsBody2D>(e).body.isValid,
                    $"{shapeFactory().kind} sprayed body {f} has an invalid Box2D handle immediately after creation."
                );
            }
            DisposeBlobs();

            for (var f = 0; f < Steps; f++)
                group.Update();

            for (var i = 0; i < entities.Count; i++)
            {
                var p = em.GetComponentData<LocalToWorld>(entities[i]).Position;
                Assert.IsFalse(
                    isnan(p.x) || isnan(p.y) || isinf(p.x) || isinf(p.y),
                    $"{shapeFactory().kind} sprayed body {i} produced NaN/Inf: {p}."
                );
                Assert.Less(
                    p.y,
                    startY[i] - 0.5f,
                    $"{shapeFactory().kind} sprayed body {i} did not fall: startY={startY[i]}, y={p.y}."
                );
            }

            Debug.Log(
                $"[DEDUP-GATE-A] {shapeFactory().kind}: {entities.Count} bodies sprayed 1/frame, each created + fell, no NaN."
            );

            world.Dispose();
            yield break;
        }

        // ================================================================================================
        // Part B — form-hash invariants, asserted through the REAL PhysicsBody2DFormHashBakingSystem.
        //
        // The system is a PostBakingSystemGroup ISystem reading the already-baked PhysicsBody2DDefinition +
        // PhysicsShape2D (+ any PhysicsShape2DElement buffer) and writing the PhysicsBody2DFormHash. Its WorldSystem
        // filter selects which world auto-runs it, but GetOrCreateSystem + manual Update drives it in any world — so
        // these assertions exercise the SHIPPED hash code, not a reimplementation. Each helper authors entities,
        // runs the system once, and reads back the produced hash.
        // ================================================================================================

        static uint4 BakeHash(
            World world,
            in PhysicsBody2DDefinition body,
            in PhysicsShape2D shape,
            PhysicsShape2D? extra = null
        )
        {
            var em = world.EntityManager;
            var e = em.CreateEntity();
            em.AddComponentData(e, body);
            em.AddComponentData(e, shape);
            if (extra.HasValue)
            {
                var buf = em.AddBuffer<PhysicsShape2DElement>(e);
                buf.Add(new PhysicsShape2DElement { shape = extra.Value });
            }

            var sys = world.GetOrCreateSystem<PhysicsBody2DFormHashBakingSystem>();
            sys.Update(world.Unmanaged);

            Assert.IsTrue(
                em.HasComponent<PhysicsBody2DFormHash>(e),
                "PhysicsBody2DFormHashBakingSystem did not add the form hash — it did not match the baked entity."
            );
            var hash = em.GetComponentData<PhysicsBody2DFormHash>(e).value;
            em.DestroyEntity(e);
            return hash;
        }

        static World MakeHashWorld() => new World("Physics2DHashGateWorld");

        static readonly PhysicsBody2DDefinition BaseBody = new PhysicsBody2DDefinition
        {
            bodyType = PhysicsBody.BodyType.Dynamic,
            gravityScale = 1f,
            linearDamping = 0.1f,
            angularDamping = 0.05f,
            constraints = PhysicsBody.BodyConstraints.None,
            mass = 1f,
            useAutoMass = true,
            fastCollisions = false,
            interpolation = PhysicsBody2DInterpolation.None,
            overrideMassDistribution = false,
            centerOfMass = new float2(0f, 0f),
            rotationalInertia = 0f,
            // pose / velocity excluded from the hash — varied freely below.
            initialPosition = new float2(0f, 0f),
            initialRotationRadians = 0f,
        };

        static PhysicsShape2D BaseShape() =>
            new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Box,
                size = new float2(1f, 1f),
                radius = 0.05f,
                boxAngleRadians = 0f,
                offset = new float2(0f, 0f),
                density = 1f,
                friction = 0.4f,
                bounciness = 0f,
                frictionMixing = PhysicsSurfaceMixing2D.Maximum,
                bouncinessMixing = PhysicsSurfaceMixing2D.Maximum,
                categoryBits = 1ul,
                contactBits = ~0ul,
                isTrigger = false,
            };

        // INVARIANT 1: same form, different pose AND different velocity → SAME hash. Pose/velocity are excluded from
        // the hash so a spray of one form at scattered poses with scattered launch velocities is ONE form (the whole
        // mechanism depends on this — folding pose in would split every spray into singletons).
        [UnityTest]
        public IEnumerator FormHash_SameFormDifferentPose_HashesEqual()
        {
            var world = MakeHashWorld();

            var bodyA = BaseBody;
            bodyA.initialPosition = new float2(0f, 0f);
            bodyA.initialRotationRadians = 0f;

            var bodyB = BaseBody;
            bodyB.initialPosition = new float2(123.5f, -47.25f);
            bodyB.initialRotationRadians = 2.1f;

            var hashA = BakeHash(world, bodyA, BaseShape());
            var hashB = BakeHash(world, bodyB, BaseShape());

            Assert.IsTrue(
                all(hashA == hashB),
                $"Two bodies of one form with different POSE hashed DIFFERENT (A={hashA}, B={hashB}) — pose leaked into the form hash; a spray would split into singleton forms and the optimisation never engages."
            );
            Debug.Log($"[DEDUP-GATE-B] same form, different pose → identical hash {hashA}.");

            world.Dispose();
            yield break;
        }

        // INVARIANT 2: every hashed field, varied alone from the base form, produces a DIFFERENT hash. A field that
        // does NOT change the hash is a collision risk: two genuinely different forms would share a template and one
        // would be served the other's geometry/def/mass. This is the broad falsification sweep — one mutation per
        // hashed field, body-side and shape-side.
        [UnityTest]
        public IEnumerator FormHash_EveryHashedFieldChangesHash()
        {
            var world = MakeHashWorld();
            var baseHash = BakeHash(world, BaseBody, BaseShape());

            // Body-side hashed fields (FormFields).
            AssertBodyFieldChanges(world, baseHash, "bodyType", b => b.bodyType = PhysicsBody.BodyType.Kinematic);
            AssertBodyFieldChanges(world, baseHash, "gravityScale", b => b.gravityScale = 0.5f);
            AssertBodyFieldChanges(world, baseHash, "linearDamping", b => b.linearDamping = 0.9f);
            AssertBodyFieldChanges(world, baseHash, "angularDamping", b => b.angularDamping = 0.9f);
            AssertBodyFieldChanges(
                world,
                baseHash,
                "constraints",
                b => b.constraints = PhysicsBody.BodyConstraints.Rotation
            );
            AssertBodyFieldChanges(world, baseHash, "mass", b => b.mass = 7.5f);
            AssertBodyFieldChanges(world, baseHash, "useAutoMass", b => b.useAutoMass = !b.useAutoMass);
            AssertBodyFieldChanges(world, baseHash, "fastCollisions", b => b.fastCollisions = !b.fastCollisions);
            AssertBodyFieldChanges(
                world,
                baseHash,
                "interpolation",
                b => b.interpolation = PhysicsBody2DInterpolation.Interpolate
            );
            AssertBodyFieldChanges(
                world,
                baseHash,
                "overrideMassDistribution",
                b => b.overrideMassDistribution = !b.overrideMassDistribution
            );
            AssertBodyFieldChanges(world, baseHash, "centerOfMass", b => b.centerOfMass = new float2(0.3f, -0.2f));
            AssertBodyFieldChanges(world, baseHash, "rotationalInertia", b => b.rotationalInertia = 2.7f);

            // Shape-side hashed fields (ShapeFields).
            AssertShapeFieldChanges(world, baseHash, "kind", s => s.kind = PhysicsShape2DKind.Circle);
            AssertShapeFieldChanges(world, baseHash, "offset", s => s.offset = new float2(0.25f, -0.1f));
            AssertShapeFieldChanges(world, baseHash, "radius", s => s.radius = 0.2f);
            AssertShapeFieldChanges(world, baseHash, "size", s => s.size = new float2(1.5f, 0.5f));
            AssertShapeFieldChanges(world, baseHash, "boxAngleRadians", s => s.boxAngleRadians = 0.4f);
            AssertShapeFieldChanges(world, baseHash, "capsuleCenter1", s => s.capsuleCenter1 = new float2(0f, 0.4f));
            AssertShapeFieldChanges(world, baseHash, "capsuleCenter2", s => s.capsuleCenter2 = new float2(0f, -0.4f));
            AssertShapeFieldChanges(world, baseHash, "edgeIsLoop", s => s.edgeIsLoop = !s.edgeIsLoop);
            AssertShapeFieldChanges(world, baseHash, "polygonDecompose", s => s.polygonDecompose = !s.polygonDecompose);
            AssertShapeFieldChanges(world, baseHash, "friction", s => s.friction = 0.8f);
            AssertShapeFieldChanges(world, baseHash, "bounciness", s => s.bounciness = 0.6f);
            AssertShapeFieldChanges(world, baseHash, "density", s => s.density = 2.3f);
            AssertShapeFieldChanges(
                world,
                baseHash,
                "frictionMixing",
                s => s.frictionMixing = PhysicsSurfaceMixing2D.Minimum
            );
            AssertShapeFieldChanges(
                world,
                baseHash,
                "bouncinessMixing",
                s => s.bouncinessMixing = PhysicsSurfaceMixing2D.Multiply
            );
            AssertShapeFieldChanges(world, baseHash, "categoryBits", s => s.categoryBits = 1ul << 5);
            AssertShapeFieldChanges(world, baseHash, "contactBits", s => s.contactBits = 0xF0F0ul);
            AssertShapeFieldChanges(world, baseHash, "isTrigger", s => s.isTrigger = !s.isTrigger);

            Debug.Log(
                "[DEDUP-GATE-B] every hashed body-side and shape-side field changes the form hash when varied alone."
            );

            world.Dispose();
            yield break;
        }

        void AssertBodyFieldChanges(World world, uint4 baseHash, string field, System.Action<RefBody> mutate)
        {
            var b = BaseBody;
            var rb = new RefBody(ref b);
            mutate(rb);
            var h = BakeHash(world, rb.Value, BaseShape());
            Assert.IsFalse(
                all(h == baseHash),
                $"Varying body field '{field}' did NOT change the form hash ({h}) — two forms differing only in {field} would share a template; the cache could serve a wrong definition."
            );
        }

        void AssertShapeFieldChanges(World world, uint4 baseHash, string field, System.Action<RefShape> mutate)
        {
            var s = BaseShape();
            var rs = new RefShape(ref s);
            mutate(rs);
            var h = BakeHash(world, BaseBody, rs.Value);
            Assert.IsFalse(
                all(h == baseHash),
                $"Varying shape field '{field}' did NOT change the form hash ({h}) — two forms differing only in {field} would share a template; the cache could serve a wrong geometry/surface."
            );
        }

        // Tiny by-ref mutator shims so the per-field lambdas above read naturally (a struct passed to a lambda needs
        // a reference holder).
        sealed class RefBody
        {
            public PhysicsBody2DDefinition Value;

            public RefBody(ref PhysicsBody2DDefinition v)
            {
                Value = v;
            }

            public PhysicsBody.BodyType bodyType
            {
                set => Value.bodyType = value;
            }
            public float gravityScale
            {
                set => Value.gravityScale = value;
            }
            public float linearDamping
            {
                set => Value.linearDamping = value;
            }
            public float angularDamping
            {
                set => Value.angularDamping = value;
            }
            public PhysicsBody.BodyConstraints constraints
            {
                set => Value.constraints = value;
            }
            public float mass
            {
                set => Value.mass = value;
            }
            public bool useAutoMass
            {
                get => Value.useAutoMass;
                set => Value.useAutoMass = value;
            }
            public bool fastCollisions
            {
                get => Value.fastCollisions;
                set => Value.fastCollisions = value;
            }
            public PhysicsBody2DInterpolation interpolation
            {
                set => Value.interpolation = value;
            }
            public bool overrideMassDistribution
            {
                get => Value.overrideMassDistribution;
                set => Value.overrideMassDistribution = value;
            }
            public float2 centerOfMass
            {
                set => Value.centerOfMass = value;
            }
            public float rotationalInertia
            {
                set => Value.rotationalInertia = value;
            }
        }

        sealed class RefShape
        {
            public PhysicsShape2D Value;

            public RefShape(ref PhysicsShape2D v)
            {
                Value = v;
            }

            public PhysicsShape2DKind kind
            {
                set => Value.kind = value;
            }
            public float2 offset
            {
                set => Value.offset = value;
            }
            public float radius
            {
                set => Value.radius = value;
            }
            public float2 size
            {
                set => Value.size = value;
            }
            public float boxAngleRadians
            {
                set => Value.boxAngleRadians = value;
            }
            public float2 capsuleCenter1
            {
                set => Value.capsuleCenter1 = value;
            }
            public float2 capsuleCenter2
            {
                set => Value.capsuleCenter2 = value;
            }
            public bool edgeIsLoop
            {
                get => Value.edgeIsLoop;
                set => Value.edgeIsLoop = value;
            }
            public bool polygonDecompose
            {
                get => Value.polygonDecompose;
                set => Value.polygonDecompose = value;
            }
            public float friction
            {
                set => Value.friction = value;
            }
            public float bounciness
            {
                set => Value.bounciness = value;
            }
            public float density
            {
                set => Value.density = value;
            }
            public PhysicsSurfaceMixing2D frictionMixing
            {
                set => Value.frictionMixing = value;
            }
            public PhysicsSurfaceMixing2D bouncinessMixing
            {
                set => Value.bouncinessMixing = value;
            }
            public ulong categoryBits
            {
                set => Value.categoryBits = value;
            }
            public ulong contactBits
            {
                set => Value.contactBits = value;
            }
            public bool isTrigger
            {
                get => Value.isTrigger;
                set => Value.isTrigger = value;
            }
        }

        // INVARIANT 3: determinism — the same form baked twice (in two entities, two system runs) hashes identical.
        [UnityTest]
        public IEnumerator FormHash_Deterministic()
        {
            var world = MakeHashWorld();
            var h1 = BakeHash(world, BaseBody, BaseShape());
            var h2 = BakeHash(world, BaseBody, BaseShape());
            var world2 = MakeHashWorld();
            var h3 = BakeHash(world2, BaseBody, BaseShape());

            Assert.IsTrue(all(h1 == h2), $"The form hash is not deterministic within one world ({h1} vs {h2}).");
            Assert.IsTrue(
                all(h1 == h3),
                $"The form hash is not deterministic across worlds ({h1} vs {h3}) — uninitialised struct padding may be leaking into the hash input."
            );
            Debug.Log($"[DEDUP-GATE-B] form hash deterministic: {h1} thrice.");

            world.Dispose();
            world2.Dispose();
            yield break;
        }

        // INVARIANT 4 (adversarial near-collision): two forms that differ ONLY in one subtle hashed field must hash
        // different. Built from the artifact's own decision points (the exact ShapeFields/FormFields members), not
        // from inputs the author imagined: a box rotated by a hair (boxAngleRadians, a custom-authoring-only field
        // the built-in box baker leaves 0), a surface whose ONLY difference is the bounciness MIXING MODE (same
        // friction/bounciness values), and a polygon whose only difference is one vertex moved a hair (the
        // variable-length blob fold). Each is a pair a naive "hash the obvious scalars" implementation could collide.
        [UnityTest]
        public IEnumerator FormHash_AdversarialNearCollisions_HashDifferent()
        {
            var world = MakeHashWorld();

            // (a) Box angle: two boxes identical but for a 0.001 rad rotation.
            var boxA = BaseShape();
            var boxB = BaseShape();
            boxB.boxAngleRadians = 0.001f;
            var ha = BakeHash(world, BaseBody, boxA);
            var hb = BakeHash(world, BaseBody, boxB);
            Assert.IsFalse(
                all(ha == hb),
                $"Two boxes differing only by a 0.001 rad rotation COLLIDED ({ha}) — a hair-rotated box would be served an axis-aligned template."
            );

            // (b) Mixing mode only: identical friction/bounciness scalars, different bouncinessMixing enum. A pure-
            // scalar hash that forgot to fold the mixing enums would collide these.
            var mixA = BaseShape();
            var mixB = BaseShape();
            mixA.bouncinessMixing = PhysicsSurfaceMixing2D.Maximum;
            mixB.bouncinessMixing = PhysicsSurfaceMixing2D.Minimum;
            var hma = BakeHash(world, BaseBody, mixA);
            var hmb = BakeHash(world, BaseBody, mixB);
            Assert.IsFalse(
                all(hma == hmb),
                $"Two surfaces differing only in bouncinessMixing COLLIDED ({hma}) — different contact-combine behaviour would share a template."
            );

            // (c) Polygon vertex: two convex quads identical but for one corner moved 0.01 m. The blob fold must make
            // them different forms.
            var polyA = new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Polygon,
                density = 1f,
                friction = 0.4f,
                vertices = Blob(
                    new[]
                    {
                        new float2(-0.5f, -0.5f),
                        new float2(0.5f, -0.5f),
                        new float2(0.5f, 0.5f),
                        new float2(-0.5f, 0.5f),
                    }
                ),
            };
            var polyB = new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Polygon,
                density = 1f,
                friction = 0.4f,
                vertices = Blob(
                    new[]
                    {
                        new float2(-0.5f, -0.5f),
                        new float2(0.5f, -0.5f),
                        new float2(0.51f, 0.5f),
                        new float2(-0.5f, 0.5f),
                    }
                ),
            };
            var hpa = BakeHash(world, BaseBody, polyA);
            var hpb = BakeHash(world, BaseBody, polyB);
            Assert.IsFalse(
                all(hpa == hpb),
                $"Two polygons differing only in one vertex (0.01 m) COLLIDED ({hpa}) — a different outline would be served a stale template."
            );

            // (d) Multi-shape order: a body whose extra (buffer) shape differs must hash different, proving the
            // buffer fold participates. Same primary, different extra-shape size.
            var extraA = new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Box,
                size = new float2(0.4f, 0.4f),
                density = 1f,
                friction = 0.4f,
            };
            var extraB = new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Box,
                size = new float2(0.9f, 0.4f),
                density = 1f,
                friction = 0.4f,
            };
            var hxa = BakeHash(world, BaseBody, BaseShape(), extraA);
            var hxb = BakeHash(world, BaseBody, BaseShape(), extraB);
            Assert.IsFalse(
                all(hxa == hxb),
                $"Two multi-shape bodies differing only in the EXTRA shape COLLIDED ({hxa}) — the buffer fold did not participate in the hash."
            );
            // And the multi-shape body must differ from the single-shape primary alone (the extra shape adds form).
            var hSingle = BakeHash(world, BaseBody, BaseShape());
            Assert.IsFalse(
                all(hxa == hSingle),
                $"A multi-shape body hashed the SAME as its primary-only single-shape form ({hxa}) — the extra shape was not folded in."
            );

            DisposeBlobs();
            Debug.Log(
                "[DEDUP-GATE-B] adversarial near-collisions all hash different (box angle, mixing mode, polygon vertex, multi-shape extra)."
            );

            world.Dispose();
            yield break;
        }

        // ------------------------------------------------------------------------------------------------
        // Strict-equality read-back assertions (NOT the parity band). Two creation paths on one v3 solver →
        // bit-identical. Box2D float reads compared with Assert.AreEqual (exact).
        // ------------------------------------------------------------------------------------------------

        static void AssertPoseIdentical(PhysicsBody a, PhysicsBody b, PhysicsShape2DKind kind, int step)
        {
            var pa = (float2)(Vector2)a.position;
            var pb = (float2)(Vector2)b.position;
            Assert.AreEqual(
                pa.x,
                pb.x,
                $"{kind} step {step}: position X diverged per-entity={pa.x} vs cached={pb.x}. NOT transparent."
            );
            Assert.AreEqual(
                pa.y,
                pb.y,
                $"{kind} step {step}: position Y diverged per-entity={pa.y} vs cached={pb.y}. NOT transparent."
            );
            var ra = a.rotation;
            var rb = b.rotation;
            Assert.AreEqual(ra.cos, rb.cos, $"{kind} step {step}: rotation cos diverged {ra.cos} vs {rb.cos}.");
            Assert.AreEqual(ra.sin, rb.sin, $"{kind} step {step}: rotation sin diverged {ra.sin} vs {rb.sin}.");
            var va = (float2)(Vector2)a.linearVelocity;
            var vb = (float2)(Vector2)b.linearVelocity;
            Assert.AreEqual(va.x, vb.x, $"{kind} step {step}: linear velocity X diverged {va.x} vs {vb.x}.");
            Assert.AreEqual(va.y, vb.y, $"{kind} step {step}: linear velocity Y diverged {va.y} vs {vb.y}.");
            Assert.AreEqual(
                a.angularVelocity,
                b.angularVelocity,
                $"{kind} step {step}: angular velocity diverged {a.angularVelocity} vs {b.angularVelocity}."
            );
        }

        static void AssertMassIdentical(PhysicsBody a, PhysicsBody b, PhysicsShape2DKind kind, string when)
        {
            var ma = a.massConfiguration;
            var mb = b.massConfiguration;
            Assert.AreEqual(ma.mass, mb.mass, $"{kind} mass differs {when}: per-entity {ma.mass} vs cached {mb.mass}.");
            Assert.AreEqual(
                ma.rotationalInertia,
                mb.rotationalInertia,
                $"{kind} rotationalInertia differs {when}: {ma.rotationalInertia} vs {mb.rotationalInertia}."
            );
            Assert.AreEqual(
                ((float2)(Vector2)ma.center).x,
                ((float2)(Vector2)mb.center).x,
                $"{kind} center.x differs {when}."
            );
            Assert.AreEqual(
                ((float2)(Vector2)ma.center).y,
                ((float2)(Vector2)mb.center).y,
                $"{kind} center.y differs {when}."
            );
        }
    }
}
