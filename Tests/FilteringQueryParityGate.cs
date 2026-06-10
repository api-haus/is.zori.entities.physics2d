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
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-5 adversarial GameObject-parity gate for the two surfaces the phase shipped: (A) collision
    /// filtering by GameObject layer + the project layer-collision-matrix, and (B) the spatial-query API
    /// (raycast / overlap / cast). The oracle is the established cross-backend pattern (single authoring →
    /// run the SAME scene two ways and compare): the package's Box2D-v3 world is built from code, the
    /// GameObject reference is the editor's Box2D-v2 world stepped with <c>Physics2D.Simulate</c> and probed
    /// with <c>UnityEngine.Physics2D.Raycast</c>/<c>Overlap*</c>/<c>*Cast</c>, and both are authored from one
    /// set of poses/layers so they cannot drift by authoring.
    /// </summary>
    /// <remarks>
    /// <para><b>Binary facts asserted EXACTLY; continuous facts bounded.</b> The two backends are different
    /// integrators (v2 vs v3), so this gate asserts SET/BINARY facts exactly — does a layer pair interact or
    /// pass through; is a body in the query hit-set or not — and bounds CONTINUOUS facts (settled positions,
    /// hit point/normal/fraction) with a generous growth-bounded envelope, exactly as
    /// <see cref="PhysicsParityHarness"/> argues.</para>
    ///
    /// <para><b>Why the package side computes its filter bits at runtime.</b> The brief requires varying the
    /// project matrix across SEVERAL configurations (same-layer self, A-vs-B colliding, A-vs-B ignored, and a
    /// layer-0-ignores-X case). A SubScene bakes the matrix at editor-bake time and freezes it, so it cannot
    /// vary per PlayMode run. The package side therefore authors <see cref="PhysicsShape2D.categoryBits"/> /
    /// <see cref="PhysicsShape2D.contactBits"/> directly from <c>1 &lt;&lt; layer</c> /
    /// <c>(uint)UnityEngine.Physics2D.GetLayerCollisionMask(layer)</c> — the IDENTICAL formula
    /// <c>Collider2DBaking.ReadFilter</c> uses — read AFTER the test mutates the matrix. This pins the runtime
    /// filter mechanism (<c>CreateShapeForBody</c> applying the bits) and the bit-math contract against the
    /// live GameObject matrix across configs. The separate <see cref="FilterBakeParityGate"/> pins the baker
    /// itself (that <c>ReadFilter</c> consults the matrix, not a hardcoded All) on a fixed non-default matrix.</para>
    ///
    /// <para><b>Global matrix state.</b> The layer-collision matrix is global project state. Every test that
    /// mutates it saves the affected pairs in <c>[SetUp]</c> snapshot, mutates via
    /// <c>Physics2D.IgnoreLayerCollision</c>, and restores in <c>[TearDown]</c>, so configs never bleed across
    /// tests or runs.</para>
    ///
    /// <para><b>World isolation.</b> Each test runs in its own disposable <see cref="World"/> (a thrown test
    /// leaks native bodies into a shared world and poisons later tests), and the GameObject reference bodies +
    /// global <c>Physics2D</c> knobs are torn down/restored in the same test.</para>
    /// </remarks>
    public sealed class FilteringQueryParityGate
    {
        const float Dt = 1f / 60f;
        static readonly Vector2 Gravity = new(0f, -9.81f);

        // The layers this gate touches. Picked in the user-layer range (8..31, the built-in 0..7 are reserved
        // names but layer 0 = Default is deliberately included to pin the layer-0 case).
        const int LDefault = 0;
        const int LA = 8;
        const int LB = 9;
        const int LFloor = 10;
        const int LX = 11; // the layer paired against Default in the layer-0-ignores-X config

        // --- global Physics2D state save/restore -----------------------------------------------------------

        SimulationMode2D _prevMode;
        Vector2 _prevGravity;
        bool _prevQueriesStartInColliders;
        bool _prevQueriesHitTriggers;
        readonly List<(int a, int b, bool ignored)> _savedPairs = new();

        // All ordered layer pairs the gate's configs ever touch — snapshot + restore exactly these.
        static readonly (int a, int b)[] TouchedPairs =
        {
            (LA, LA), (LB, LB), (LA, LB), (LA, LFloor), (LB, LFloor),
            (LDefault, LDefault), (LDefault, LX), (LDefault, LFloor), (LX, LFloor), (LX, LX),
            (LDefault, LA), (LDefault, LB),
        };

        [SetUp]
        public void SetUp()
        {
            _prevMode = UnityEngine.Physics2D.simulationMode;
            _prevGravity = UnityEngine.Physics2D.gravity;
            _prevQueriesStartInColliders = UnityEngine.Physics2D.queriesStartInColliders;
            _prevQueriesHitTriggers = UnityEngine.Physics2D.queriesHitTriggers;

            _savedPairs.Clear();
            foreach (var (a, b) in TouchedPairs)
                _savedPairs.Add((a, b, UnityEngine.Physics2D.GetIgnoreLayerCollision(a, b)));

            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;
            // A query that starts inside a collider should still report it (the package query has no
            // "skip start-overlapped" notion), and triggers are irrelevant here (no triggers authored).
            UnityEngine.Physics2D.queriesStartInColliders = true;
        }

        [TearDown]
        public void TearDown()
        {
            // Restore every pair to its snapshot state so no config bleeds into the next test or run.
            foreach (var (a, b, ignored) in _savedPairs)
                UnityEngine.Physics2D.IgnoreLayerCollision(a, b, ignored);
            UnityEngine.Physics2D.queriesHitTriggers = _prevQueriesHitTriggers;
            UnityEngine.Physics2D.queriesStartInColliders = _prevQueriesStartInColliders;
            UnityEngine.Physics2D.gravity = _prevGravity;
            UnityEngine.Physics2D.simulationMode = _prevMode;
        }

        // --- package world ---------------------------------------------------------------------------------

        static World MakePackageWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("Physics2DParityGateWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DBatchCreationSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            fixedGroup.SortSystems();
            group = fixedGroup;
            return world;
        }

        static PhysicsWorld GetWorld(EntityManager em)
        {
            var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        // Resolve the (category, contacts) bits for a layer from the LIVE project matrix — the identical
        // formula Collider2DBaking.ReadFilter bakes, evaluated after the test mutates the matrix.
        static void BitsForLayer(int layer, out ulong categoryBits, out ulong contactBits)
        {
            categoryBits = 1ul << layer;
            contactBits = unchecked((uint)UnityEngine.Physics2D.GetLayerCollisionMask(layer));
        }

        static Entity SpawnPackageCircle(EntityManager em, float2 pos, float r, int layer, bool dynamic)
        {
            BitsForLayer(layer, out var cat, out var con);
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = dynamic ? PhysicsBody.BodyType.Dynamic : PhysicsBody.BodyType.Static,
                    gravityScale = dynamic ? 1f : 0f,
                    initialPosition = pos,
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = r,
                    density = 1f,
                    friction = 0.4f,
                    categoryBits = cat,
                    contactBits = con,
                }
            );
        }

        static Entity SpawnPackageBox(EntityManager em, float2 center, float2 size, int layer, bool dynamic)
        {
            BitsForLayer(layer, out var cat, out var con);
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = dynamic ? PhysicsBody.BodyType.Dynamic : PhysicsBody.BodyType.Static,
                    gravityScale = dynamic ? 1f : 0f,
                    initialPosition = center,
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = size,
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                    categoryBits = cat,
                    contactBits = con,
                }
            );
        }

        // --- GameObject reference ---------------------------------------------------------------------------

        // A live GameObject body (Rigidbody2D + collider) on a layer, NeverSleep so it cannot park early and
        // mask a divergence. Tracked for teardown.
        Rigidbody2D MakeRefCircle(float2 pos, float r, int layer, bool dynamic, List<GameObject> track)
        {
            var go = new GameObject($"RefCircle_{layer}") { layer = layer };
            go.transform.position = new Vector3(pos.x, pos.y, 0f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = dynamic ? RigidbodyType2D.Dynamic : RigidbodyType2D.Static;
            rb.gravityScale = dynamic ? 1f : 0f;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var c = go.AddComponent<CircleCollider2D>();
            c.radius = r;
            track.Add(go);
            return rb;
        }

        Rigidbody2D MakeRefBox(float2 center, float2 size, int layer, bool dynamic, List<GameObject> track)
        {
            var go = new GameObject($"RefBox_{layer}") { layer = layer };
            go.transform.position = new Vector3(center.x, center.y, 0f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = dynamic ? RigidbodyType2D.Dynamic : RigidbodyType2D.Static;
            rb.gravityScale = dynamic ? 1f : 0f;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var c = go.AddComponent<BoxCollider2D>();
            c.size = (Vector2)size;
            track.Add(go);
            return rb;
        }

        static void DestroyAll(List<GameObject> track)
        {
            foreach (var go in track)
                if (go != null)
                    Object.Destroy(go);
            track.Clear();
        }

        // =====================================================================================================
        // (A) FILTERING PARITY over multiple matrix configurations.
        // =====================================================================================================

        /// <summary>
        /// The discriminating geometry, run in BOTH backends for one matrix config: a static floor (on a
        /// third layer both bodies collide with), a bottom dynamic circle resting on it, and a top dynamic
        /// circle dropped from high. The binary outcome is whether the top stacks on the bottom (the pair
        /// collides) or passes through and rests beside it on the floor (the pair is ignored). Returns the
        /// top's settled Y MINUS the bottom's settled Y for each backend — a large gap (~2r) means stacked, a
        /// near-zero gap means passed through. The floor layer collides with both top and bottom in every
        /// config, so the body-floor contact is held constant and only the body-body decision varies.
        /// </summary>
        IEnumerator RunFilterConfig(
            string label,
            int topLayer,
            int botLayer,
            bool expectCollide,
            System.Action applyMatrix
        )
        {
            applyMatrix();
            UnityEngine.Physics2D.SyncTransforms();

            const float r = 0.5f;
            const float floorTopY = 0f;
            var floorCenter = new float2(0f, floorTopY - 0.5f);
            var floorSize = new float2(6f, 1f);
            var botPos = new float2(0f, floorTopY + r); // resting on the floor
            var topPos = new float2(0f, floorTopY + 5f); // dropped from high

            // The floor must collide with BOTH body layers in every config, so put it on LFloor and ensure
            // LFloor is not ignored against either body layer (SetUp's matrix is all-on by default; the
            // configs only ignore body-body pairs, never a body-floor pair — but a layer-0 config could touch
            // LDefault, so assert the invariant rather than assume it).
            Assert.IsFalse(
                UnityEngine.Physics2D.GetIgnoreLayerCollision(topLayer, LFloor)
                    || UnityEngine.Physics2D.GetIgnoreLayerCollision(botLayer, LFloor),
                $"[{label}] floor layer {LFloor} must collide with both body layers — body-floor contact is "
                    + "the held-constant confound; a config that ignores it would break the discriminator."
            );

            // ---- package backend ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            SpawnPackageBox(em, floorCenter, floorSize, LFloor, dynamic: false);
            var pkgBot = SpawnPackageCircle(em, botPos, r, botLayer, dynamic: true);
            var pkgTop = SpawnPackageCircle(em, topPos, r, topLayer, dynamic: true);
            group.Update(); // create, no step
            for (var f = 0; f < 240; f++)
                group.Update();
            float PkgY(Entity e) => em.GetComponentData<LocalToWorld>(e).Position.y;
            var pkgTopY = PkgY(pkgTop);
            var pkgBotY = PkgY(pkgBot);
            var pkgGap = pkgTopY - pkgBotY;
            world.Dispose();

            // ---- GameObject backend (same poses, same layers, live matrix) ----
            var track = new List<GameObject>();
            MakeRefBox(floorCenter, floorSize, LFloor, dynamic: false, track);
            var refBot = MakeRefCircle(botPos, r, botLayer, dynamic: true, track);
            var refTop = MakeRefCircle(topPos, r, topLayer, dynamic: true, track);
            UnityEngine.Physics2D.SyncTransforms();
            for (var f = 0; f < 240; f++)
            {
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                yield return null;
            }
            var refTopY = refTop.position.y;
            var refBotY = refBot.position.y;
            var refGap = refTopY - refBotY;
            DestroyAll(track);

            // ---- verdicts ----
            // BINARY: both backends agree on stacked-vs-passed-through. Use a 0.6 m threshold (well between a
            // ~1.0 m stacked gap and a ~0 m pass-through gap).
            const float gapThreshold = 0.6f;
            var pkgStacked = pkgGap > gapThreshold;
            var refStacked = refGap > gapThreshold;
            Debug.Log(
                $"[PHYSICS2D-FILTER-PARITY] {label}: expectCollide={expectCollide} | "
                    + $"package gap={pkgGap:F3} (stacked={pkgStacked}) topY={pkgTopY:F3} botY={pkgBotY:F3} | "
                    + $"GameObject gap={refGap:F3} (stacked={refStacked}) topY={refTopY:F3} botY={refBotY:F3} | "
                    + $"matrixRow(top {topLayer})=0x{UnityEngine.Physics2D.GetLayerCollisionMask(topLayer):X8}"
            );
            Assert.AreEqual(
                expectCollide,
                refStacked,
                $"[{label}] GameObject ORACLE disagrees with the expected outcome: expectCollide="
                    + $"{expectCollide} but GameObject stacked={refStacked} (gap={refGap:F3}). The oracle "
                    + "itself is wrong for this config — the matrix mutation or geometry is off, not the package."
            );
            Assert.AreEqual(
                refStacked,
                pkgStacked,
                $"[{label}] FILTER PARITY BROKE: GameObject stacked={refStacked} (gap={refGap:F3}) but "
                    + $"package stacked={pkgStacked} (gap={pkgGap:F3}). The baked/applied contact filter does "
                    + "not reproduce the GameObject's layer-matrix collide/ignore decision for this config."
            );
            // Continuous bound: the two settled gaps agree to a generous band (different integrators).
            Assert.Less(
                abs(pkgGap - refGap),
                0.4f,
                $"[{label}] settled gap envelope exceeded: package gap={pkgGap:F3}, GameObject gap={refGap:F3}."
            );
        }

        [UnityTest]
        public IEnumerator Filter_SameLayerSelfCollision_StacksInBoth()
        {
            // Same-layer self-collision: both on layer A, A self-collides by default → the pair interacts.
            // Pins that a body does NOT pass through another body on its own layer.
            yield return RunFilterConfig(
                "same-layer-self (A,A collide)",
                topLayer: LA,
                botLayer: LA,
                expectCollide: true,
                applyMatrix: () => { } // default all-on matrix; LA self-collides
            );
        }

        [UnityTest]
        public IEnumerator Filter_DistinctLayersColliding_StacksInBoth()
        {
            // A-vs-B, matrix leaves them colliding → the pair interacts.
            yield return RunFilterConfig(
                "A-vs-B colliding (8,9 on)",
                topLayer: LA,
                botLayer: LB,
                expectCollide: true,
                applyMatrix: () => UnityEngine.Physics2D.IgnoreLayerCollision(LA, LB, false)
            );
        }

        [UnityTest]
        public IEnumerator Filter_DistinctLayersIgnored_PassesThroughInBoth()
        {
            // A-vs-B ignored → the pair passes through, but both still land on the floor (LFloor collides
            // with both).
            yield return RunFilterConfig(
                "A-vs-B ignored (8,9 off)",
                topLayer: LA,
                botLayer: LB,
                expectCollide: false,
                applyMatrix: () => UnityEngine.Physics2D.IgnoreLayerCollision(LA, LB, true)
            );
        }

        [UnityTest]
        public IEnumerator Filter_Layer0IgnoresX_PassesThroughInBoth_NotHardcodedAll()
        {
            // THE escalated layer-0 pin: layer 0 (Default) is NOT "collide with everything". With layer 0
            // ignoring layer X, a top on layer 0 and a bottom on layer X must PASS THROUGH in BOTH mediums.
            // A package that hardcoded layer 0 to "All" would wrongly STACK — this config falsifies that.
            yield return RunFilterConfig(
                "layer-0 ignores X (0,11 off)",
                topLayer: LDefault,
                botLayer: LX,
                expectCollide: false,
                applyMatrix: () =>
                {
                    UnityEngine.Physics2D.IgnoreLayerCollision(LDefault, LX, true);
                    // Layer 0 must still collide with the floor layer so the top lands rather than falling
                    // forever — assert via SetUp's all-on default (only the 0,X pair is ignored here).
                }
            );
        }

        [UnityTest]
        public IEnumerator Filter_Layer0CollidesX_StacksInBoth()
        {
            // The control for the layer-0 pin: with layer 0 NOT ignoring layer X, the same scene STACKS in
            // both — so the pass-through above is the matrix decision, not an unconditional layer-0 quirk.
            yield return RunFilterConfig(
                "layer-0 collides X (0,11 on)",
                topLayer: LDefault,
                botLayer: LX,
                expectCollide: true,
                applyMatrix: () => UnityEngine.Physics2D.IgnoreLayerCollision(LDefault, LX, false)
            );
        }

        // =====================================================================================================
        // (B) QUERY PARITY — ray / overlap / cast hit-SET equals the GameObject collider set; mask honored;
        //     a hit on a BATCH-created body resolves the correct entity.
        // =====================================================================================================

        // A query scene: four static circle targets at distinct positions on distinct layers, authored in
        // BOTH backends. Two of the package targets come through the BULK BATCH path (the batch-body pin),
        // two through direct authoring. The GameObject side authors all four as plain colliders. Bodies are
        // matched across backends by authored position (single-authoring identity key).
        struct QueryTarget
        {
            public float2 pos;
            public float radius;
            public int layer;
            public bool viaBatch; // package side: created through PhysicsBody2DBatchRequest
        }

        static readonly QueryTarget[] Targets =
        {
            new() { pos = new float2(0f, 5f), radius = 1f, layer = LA, viaBatch = true },
            new() { pos = new float2(0f, 9f), radius = 1f, layer = LB, viaBatch = true },
            new() { pos = new float2(4f, 5f), radius = 1f, layer = LA, viaBatch = false },
            new() { pos = new float2(-4f, 5f), radius = 1f, layer = LB, viaBatch = false },
        };

        // Build the package query scene. Returns the world + a position→entity map for hit-set identity.
        World BuildPackageQueryScene(out FixedStepSimulationSystemGroup group, out Dictionary<Entity, float2> entityPos)
        {
            var world = MakePackageWorld(out group);
            var em = world.EntityManager;
            entityPos = new Dictionary<Entity, float2>();

            // Batch targets: one request per batch target so each lands at a known position (spawnMin==
            // spawnMax pins the scatter to a single point). This exercises the CreateBodyBatch path AND the
            // per-body userData packing the batch system does.
            foreach (var t in Targets)
            {
                if (!t.viaBatch)
                    continue;
                BitsForLayer(t.layer, out var cat, out var con);
                var req = em.CreateEntity();
                em.AddComponentData(
                    req,
                    new PhysicsBody2DBatchRequest
                    {
                        count = 1,
                        bodyType = PhysicsBody.BodyType.Static,
                        gravityScale = 0f,
                        radius = t.radius,
                        density = 1f,
                        spawnMin = t.pos,
                        spawnMax = t.pos,
                        seed = 0xABCDu,
                        categoryBits = cat,
                        contactBits = con,
                    }
                );
            }

            // Direct targets.
            foreach (var t in Targets)
            {
                if (t.viaBatch)
                    continue;
                SpawnPackageCircle(em, t.pos, t.radius, t.layer, dynamic: false);
            }

            group.Update(); // creates per-entity bodies AND consumes batch requests (batch system runs after world)

            // Map every live body entity to its authored position (read LocalToWorld, which the batch path
            // seeds at the scatter pose and the direct path seeds at the authored pose).
            var q = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            using var ents = q.ToEntityArray(Allocator.Temp);
            using var ltws = q.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            for (var i = 0; i < ents.Length; i++)
                entityPos[ents[i]] = ((float3)ltws[i].Position).xy;

            return world;
        }

        List<Rigidbody2D> BuildReferenceQueryScene(List<GameObject> track)
        {
            var bodies = new List<Rigidbody2D>();
            foreach (var t in Targets)
                bodies.Add(MakeRefCircle(t.pos, t.radius, t.layer, dynamic: false, track));
            UnityEngine.Physics2D.SyncTransforms();
            return bodies;
        }

        // Collect the set of authored positions a GameObject query hit, from the hit colliders' transforms.
        static HashSet<int> RefHitKeys(IEnumerable<Collider2D> cols)
        {
            var keys = new HashSet<int>();
            foreach (var c in cols)
                if (c != null)
                    keys.Add(PosKey(((float3)(Vector3)c.transform.position).xy));
            return keys;
        }

        // Collect the set of authored positions a package query hit, from the resolved entities' map. Asserts
        // every hit resolved to a real package entity (Entity.Null would be a userData-packing failure).
        static HashSet<int> PkgHitKeys(NativeList<PhysicsQueryHit2D> hits, Dictionary<Entity, float2> map, string label)
        {
            var keys = new HashSet<int>();
            for (var i = 0; i < hits.Length; i++)
            {
                var e = hits[i].entity;
                Assert.AreNotEqual(
                    Entity.Null,
                    e,
                    $"[{label}] package query hit #{i} resolved to Entity.Null — the shape→body→entity "
                        + "userData packing failed for a body the package created (batch or direct)."
                );
                Assert.IsTrue(
                    map.ContainsKey(e),
                    $"[{label}] package query hit #{i} resolved to entity {e} which is not in the scene's "
                        + "body map — userData unpacked to a stale/wrong Entity."
                );
                keys.Add(PosKey(map[e]));
            }
            return keys;
        }

        // Quantize a world position to a stable integer key (0.05 m grid) for cross-backend set identity.
        static int PosKey(float2 p) => ((int)round(p.x * 20f)) * 100000 + (int)round(p.y * 20f);

        [UnityTest]
        public IEnumerator Query_Raycast_HitSetMatchesGameObject_BatchBodyResolves()
        {
            var world = BuildPackageQueryScene(out _, out var map);
            var em = world.EntityManager;
            var pw = GetWorld(em);

            // A vertical ray through the column of targets at x=0 (the two BATCH targets at y=5 and y=9).
            var origin = new float2(0f, 0f);
            var dir = new float2(0f, 1f);
            const float dist = 12f;

            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            PhysicsQueries2D.Raycast(pw, origin, dir, dist, 0ul, hits);
            var pkgKeys = PkgHitKeys(hits, map, "Raycast");

            // Capture the closest entity too (the batch-body pin: a batch body must resolve correctly).
            PhysicsQueries2D.RaycastClosest(pw, origin, dir, dist, 0ul, out var closest);
            world.Dispose();

            // GameObject oracle: Raycast with a ContactFilter2D over all layers, array-filling for the full
            // hit set.
            var track = new List<GameObject>();
            BuildReferenceQueryScene(track);
            var cf = new ContactFilter2D();
            cf.useTriggers = false;
            cf.ClearLayerMask();
            var refHitsArr = new RaycastHit2D[16];
            var n = UnityEngine.Physics2D.Raycast(
                (Vector2)origin,
                (Vector2)dir,
                cf,
                refHitsArr,
                dist
            );
            var refCols = new List<Collider2D>();
            for (var i = 0; i < n; i++)
                refCols.Add(refHitsArr[i].collider);
            var refKeys = RefHitKeys(refCols);
            DestroyAll(track);

            Debug.Log(
                $"[PHYSICS2D-QUERY-PARITY] Raycast: package hits={pkgKeys.Count} GameObject hits={refKeys.Count}; "
                    + $"closest batch entity={closest.entity} at {(map.ContainsKey(closest.entity) ? map[closest.entity].ToString() : "n/a")} "
                    + $"point={closest.point} fraction={closest.fraction:F3}."
            );
            // The vertical ray hits exactly the two x=0 targets (both BATCH-created). Equal hit SET.
            Assert.IsTrue(
                pkgKeys.SetEquals(refKeys),
                $"[Raycast] hit-SET mismatch: package={KeySet(pkgKeys)} GameObject={KeySet(refKeys)}."
            );
            Assert.AreEqual(2, refKeys.Count, "[Raycast] oracle should hit exactly the two x=0 column targets.");
            // Batch-body resolution: the closest hit is the y=5 batch body; it resolved to a real entity at y~5.
            Assert.IsTrue(
                map.ContainsKey(closest.entity) && abs(map[closest.entity].y - 5f) < 0.5f,
                $"[Raycast] closest hit did not resolve to the batch-created y=5 body: entity={closest.entity}."
            );
            yield break;
        }

        [UnityTest]
        public IEnumerator Query_OverlapCircle_And_OverlapBox_HitSetsMatchGameObject()
        {
            var world = BuildPackageQueryScene(out _, out var map);
            var em = world.EntityManager;
            var pw = GetWorld(em);

            // Overlap circle around the y=5 row: should catch the three y=5 targets (x=0 batch, x=4, x=-4)
            // and miss the y=9 target. A big radius so all three y=5 are inside.
            var center = new float2(0f, 5f);
            const float radius = 5f;
            using var ocHits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            PhysicsQueries2D.OverlapCircle(pw, center, radius, 0ul, ocHits);
            var pkgOcKeys = PkgHitKeys(ocHits, map, "OverlapCircle");

            // Overlap box covering just the x=0 column (the two batch targets at y=5 and y=9).
            var boxCenter = new float2(0f, 7f);
            var boxSize = new float2(1.5f, 6f);
            using var obHits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            PhysicsQueries2D.OverlapBox(pw, boxCenter, boxSize, 0f, 0ul, obHits);
            var pkgObKeys = PkgHitKeys(obHits, map, "OverlapBox");
            world.Dispose();

            var track = new List<GameObject>();
            BuildReferenceQueryScene(track);
            var cf = new ContactFilter2D();
            cf.useTriggers = false;
            cf.ClearLayerMask();
            var ocCols = new List<Collider2D>();
            UnityEngine.Physics2D.OverlapCircle((Vector2)center, radius, cf, ocCols);
            var refOcKeys = RefHitKeys(ocCols);
            var obCols = new List<Collider2D>();
            UnityEngine.Physics2D.OverlapBox((Vector2)boxCenter, (Vector2)boxSize, 0f, cf, obCols);
            var refObKeys = RefHitKeys(obCols);
            DestroyAll(track);

            Debug.Log(
                $"[PHYSICS2D-QUERY-PARITY] OverlapCircle: package={pkgOcKeys.Count} GameObject={refOcKeys.Count}; "
                    + $"OverlapBox: package={pkgObKeys.Count} GameObject={refObKeys.Count}."
            );
            Assert.IsTrue(
                pkgOcKeys.SetEquals(refOcKeys),
                $"[OverlapCircle] hit-SET mismatch: package={KeySet(pkgOcKeys)} GameObject={KeySet(refOcKeys)}."
            );
            Assert.IsTrue(
                pkgObKeys.SetEquals(refObKeys),
                $"[OverlapBox] hit-SET mismatch: package={KeySet(pkgObKeys)} GameObject={KeySet(refObKeys)}."
            );
            yield break;
        }

        [UnityTest]
        public IEnumerator Query_CircleCast_And_BoxCast_HitSetsMatchGameObject()
        {
            var world = BuildPackageQueryScene(out _, out var map);
            var em = world.EntityManager;
            var pw = GetWorld(em);

            // Circle cast straight up through the x=0 column.
            var origin = new float2(0f, 0f);
            var dir = new float2(0f, 1f);
            const float dist = 12f;
            const float castRadius = 0.25f;
            using var ccHits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            PhysicsQueries2D.CircleCast(pw, origin, castRadius, dir, dist, 0ul, ccHits);
            var pkgCcKeys = PkgHitKeys(ccHits, map, "CircleCast");

            using var bcHits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            PhysicsQueries2D.BoxCast(pw, origin, new float2(0.5f, 0.5f), 0f, dir, dist, 0ul, bcHits);
            var pkgBcKeys = PkgHitKeys(bcHits, map, "BoxCast");
            world.Dispose();

            var track = new List<GameObject>();
            BuildReferenceQueryScene(track);
            var cf = new ContactFilter2D();
            cf.useTriggers = false;
            cf.ClearLayerMask();
            var ccArr = new RaycastHit2D[16];
            var nc = UnityEngine.Physics2D.CircleCast((Vector2)origin, castRadius, (Vector2)dir, cf, ccArr, dist);
            var ccCols = new List<Collider2D>();
            for (var i = 0; i < nc; i++)
                ccCols.Add(ccArr[i].collider);
            var refCcKeys = RefHitKeys(ccCols);

            var bcArr = new RaycastHit2D[16];
            var nb = UnityEngine.Physics2D.BoxCast((Vector2)origin, new Vector2(0.5f, 0.5f), 0f, (Vector2)dir, cf, bcArr, dist);
            var bcCols = new List<Collider2D>();
            for (var i = 0; i < nb; i++)
                bcCols.Add(bcArr[i].collider);
            var refBcKeys = RefHitKeys(bcCols);
            DestroyAll(track);

            Debug.Log(
                $"[PHYSICS2D-QUERY-PARITY] CircleCast: package={pkgCcKeys.Count} GameObject={refCcKeys.Count}; "
                    + $"BoxCast: package={pkgBcKeys.Count} GameObject={refBcKeys.Count}."
            );
            Assert.IsTrue(
                pkgCcKeys.SetEquals(refCcKeys),
                $"[CircleCast] hit-SET mismatch: package={KeySet(pkgCcKeys)} GameObject={KeySet(refCcKeys)}."
            );
            Assert.IsTrue(
                pkgBcKeys.SetEquals(refBcKeys),
                $"[BoxCast] hit-SET mismatch: package={KeySet(pkgBcKeys)} GameObject={KeySet(refBcKeys)}."
            );
            yield break;
        }

        [UnityTest]
        public IEnumerator Query_LayerMask_MaskedOutBodyHitByNeither()
        {
            // Mask the query to layer A only. The y=5 batch target (layer A) is in-set; the y=9 batch target
            // (layer B) is masked out and must be hit by NEITHER backend.
            var world = BuildPackageQueryScene(out _, out var map);
            var em = world.EntityManager;
            var pw = GetWorld(em);

            var maskA = 1ul << LA;
            var origin = new float2(0f, 0f);
            var dir = new float2(0f, 1f);
            const float dist = 12f;
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            PhysicsQueries2D.Raycast(pw, origin, dir, dist, maskA, hits);
            var pkgKeys = PkgHitKeys(hits, map, "Raycast(maskA)");
            world.Dispose();

            var track = new List<GameObject>();
            BuildReferenceQueryScene(track);
            var cf = new ContactFilter2D();
            cf.useTriggers = false;
            cf.SetLayerMask(1 << LA); // hit layer A only
            var refArr = new RaycastHit2D[16];
            var n = UnityEngine.Physics2D.Raycast((Vector2)origin, (Vector2)dir, cf, refArr, dist);
            var refCols = new List<Collider2D>();
            for (var i = 0; i < n; i++)
                refCols.Add(refArr[i].collider);
            var refKeys = RefHitKeys(refCols);
            // The y=9 layer-B target's authored position key — must be absent from BOTH hit sets.
            var maskedKey = PosKey(new float2(0f, 9f));
            DestroyAll(track);

            Debug.Log(
                $"[PHYSICS2D-QUERY-PARITY] Raycast(maskA={maskA:X}): package hits={pkgKeys.Count} "
                    + $"GameObject hits={refKeys.Count}; masked y=9 key={maskedKey} "
                    + $"in package={pkgKeys.Contains(maskedKey)} in GameObject={refKeys.Contains(maskedKey)}."
            );
            Assert.IsTrue(
                pkgKeys.SetEquals(refKeys),
                $"[Raycast(maskA)] masked hit-SET mismatch: package={KeySet(pkgKeys)} GameObject={KeySet(refKeys)}."
            );
            Assert.IsFalse(
                pkgKeys.Contains(maskedKey),
                "[Raycast(maskA)] package HIT the masked-out layer-B y=9 body — layer mask not honored."
            );
            Assert.IsFalse(
                refKeys.Contains(maskedKey),
                "[Raycast(maskA)] GameObject oracle hit the masked-out body — oracle mask setup is wrong."
            );
            // The in-set y=5 layer-A batch body must be present in both.
            Assert.IsTrue(
                pkgKeys.Contains(PosKey(new float2(0f, 5f))),
                "[Raycast(maskA)] package missed the in-mask layer-A y=5 batch body."
            );
            yield break;
        }

        static string KeySet(HashSet<int> keys)
        {
            var sb = new System.Text.StringBuilder("{");
            foreach (var k in keys)
                sb.Append(k).Append(',');
            return sb.Append('}').ToString();
        }
    }
}
