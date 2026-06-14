using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Scenes;
using Unity.Transforms;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Zori.Entities.Physics2D.Authoring;
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
    /// Shared EditMode harness that mirrors <c>PhysicsParityHarness</c>/<c>CustomAuthoringParityHarness</c> but
    /// runs entirely in batchmode EditMode without <c>SceneManager.LoadScene</c> or build-settings registration.
    /// A gate authors its fixture through one of <c>Physics2DFixtures</c>' populate methods; this base class owns
    /// the temp <c>Assets/</c> folder, loads the SubScene child synchronously by GUID into a private
    /// <c>WorldFlags.Game</c> world, swaps a <c>FixedRateSimpleManager(1/60)</c>, and (for the parity gates)
    /// opens the same authoring child scene additively as live GameObjects stepped by <c>Physics2D.Simulate</c>.
    /// </summary>
    /// <remarks>
    /// A Game world is required because the EditMode live-bake "Editor World" only instantiates systems flagged
    /// <c>WorldSystemFilterFlags.Editor</c>; the package's FixedStep systems carry the default flags, so they
    /// exist in a Game world but not the editor world (the cc2d recipe documents this with engine cites).
    /// <c>SceneLoadFlags.BlockOnImport | BlockOnStreamIn</c> runs the real SubScene baker on demand and completes
    /// streaming within one <c>world.Update()</c>, so gates run as plain <c>[Test]</c> — no <c>[UnityTest]</c>
    /// coroutine, no frame-pumping. No mocks: the real bakers, the real FixedStep systems, a real Box2D world,
    /// and (reference side) the real built-in 2D physics scene stepped manually.
    /// <para>The GameObject-reference side is proven by <c>Physics2DSimulateInEditModeProbe</c>: a body opened
    /// additively in EditMode and stepped with <c>Physics2D.Simulate(Script)</c> falls deterministically in
    /// batchmode.</para>
    /// </remarks>
    public abstract class Physics2DEditModeHarness
    {
        protected const float FixedDt = 1f / 60f;
        protected static readonly Vector2 Gravity = new(0f, -9.81f);

        // The matched determinism preconditions the PlayMode harness applied to the GameObject reference.
        const float ReferenceFriction = 0.4f;
        const float ReferenceRestitution = 0f;

        protected World World;
        protected EntityManager EntityManager => World.EntityManager;
        protected FixedStepSimulationSystemGroup FixedGroup;

        string _folder;
        string _childPath;
        Scene _refScene;
        bool _refSceneOpen;
        SimulationMode2D _prevMode;
        Vector2 _prevGravity;
        bool _physicsGlobalsCaptured;
        readonly List<UnityEngine.Object> _refDestroyables = new();

        // ---- setup: tolerate unrelated editor log noise during the fixture's AssetDatabase.Refresh ----------

        // Authoring a fixture calls AssetDatabase.Refresh, which fires every registered asset postprocessor in the
        // project — including a vendor MicroSplat texture-array preprocessor that logs an [Error] about a malformed
        // .meta on an unrelated package's demo texture as it imports. The Unity Test Framework turns any unexpected
        // editor log into a test failure, so that third-party noise (observed only on zority_6_3, a streaming-timing
        // race) failed the first parity gate whose Refresh happened to catch the import. Ignoring failing log
        // messages here drops that conversion-of-log-to-failure; it does NOT relax any physics check — the gates
        // assert through Assert.*, which throws regardless of the log policy. Reset in TearDown so the policy never
        // leaks past one test.
        [SetUp]
        public void SetUp() => LogAssert.ignoreFailingMessages = true;

        // ---- ECS side: author + synchronously load the SubScene into a private Game world ------------------

        /// <summary>
        /// Author <paramref name="populate"/>'s world into a temp folder and load its SubScene synchronously into
        /// a fresh Game world with all default systems. After this the FixedStep group exists with a
        /// FixedRateSimpleManager swapped in; the caller decides whether to run the creation step via
        /// <see cref="CreateBodies"/>. The child SubScene asset path is returned (also kept for the parity
        /// reference side).
        /// </summary>
        protected string LoadSubScene(Action<GameObject> populate, string sceneName)
        {
            _folder = "Assets/__p2d_fixture_" + Guid.NewGuid().ToString("N") + "__";
            _childPath = Physics2DFixtures.BuildScene(_folder, sceneName, populate);
            var guid = new Unity.Entities.Hash128(AssetDatabase.AssetPathToGUID(_childPath));

            World = new World("Physics2D EditMode", WorldFlags.Game);
            DefaultWorldInitialization.AddSystemsToRootLevelSystemGroups(
                World,
                DefaultWorldInitialization.GetAllSystems(WorldSystemFilterFlags.Default)
            );

            FixedGroup = World.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(FixedGroup, "No FixedStepSimulationSystemGroup in the Game world.");
            FixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(FixedDt);

            // Disable the FixedStep group across the streaming World.Update so the SubScene streams + bakes without
            // the physics systems creating/stepping the baked bodies inside that update. With the group live, the
            // streaming update would run the body-creation frame, and the caller's explicit CreateBodies() would
            // then INTEGRATE one step instead of creating — putting the ECS body one step ahead of the reference
            // (the lockstep parity bug). Re-enabled below so CreateBodies() is the deterministic creation frame
            // (mirrors PhysicsParityHarness disabling the group through the bake-wait).
            FixedGroup.Enabled = false;
            var sceneEntity = SceneSystem.LoadSceneAsync(
                World.Unmanaged,
                guid,
                new SceneSystem.LoadParameters { Flags = SceneLoadFlags.BlockOnImport | SceneLoadFlags.BlockOnStreamIn }
            );
            World.Update();
            Assert.IsTrue(
                SceneSystem.IsSceneLoaded(World.Unmanaged, sceneEntity),
                "SubScene did not import+stream synchronously — BlockOnImport|BlockOnStreamIn should land the "
                    + "section within one world update in EditMode."
            );
            FixedGroup.Enabled = true;
            return _childPath;
        }

        /// <summary>
        /// Run the body-creation fixed step: PhysicsWorld2DSystem creates the Box2D bodies and (by design) does
        /// not integrate on this frame, so every baked body sits at its authored pose afterward.
        /// </summary>
        protected void CreateBodies() => FixedGroup.Update();

        protected void Step(int count)
        {
            for (var i = 0; i < count; i++)
                FixedGroup.Update();
        }

        protected EntityQuery Query(params ComponentType[] types) => EntityManager.CreateEntityQuery(types);

        // ---- GameObject reference side: open the SAME child scene additively as live built-in bodies ---------

        /// <summary>
        /// Enter Script-stepping mode (capturing the global 2D physics knobs for teardown restore) and open the
        /// authoring child scene additively so its GameObjects come up as live <see cref="Rigidbody2D"/>/collider
        /// instances in the built-in 2D physics scene. Must be called after <see cref="LoadSubScene"/>.
        /// </summary>
        protected Scene OpenReferenceScene()
        {
            CaptureAndEnterScriptMode();
            _refScene = EditorSceneManager.OpenScene(_childPath, OpenSceneMode.Additive);
            _refSceneOpen = true;
            return _refScene;
        }

        void CaptureAndEnterScriptMode()
        {
            _prevMode = UnityEngine.Physics2D.simulationMode;
            _prevGravity = UnityEngine.Physics2D.gravity;
            _physicsGlobalsCaptured = true;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;
        }

        protected PhysicsMaterial2D NewReferenceMaterial(string name)
        {
            var m = new PhysicsMaterial2D(name) { friction = ReferenceFriction, bounciness = ReferenceRestitution };
            _refDestroyables.Add(m);
            return m;
        }

        // Collect every live Rigidbody2D in the additively-opened child scene (the built-in-authored reference),
        // applying the matched determinism preconditions: NeverSleep, a fallback material on un-materialed
        // colliders, the static-body filter, and the serialized InitialVelocity2DAuthoring seed. Verbatim port of
        // PhysicsParityHarness.CollectReferenceBodies.
        protected List<Rigidbody2D> CollectReferenceBodies(Scene childScene, PhysicsMaterial2D fallbackMaterial)
        {
            var bodies = new List<Rigidbody2D>();
            foreach (var root in childScene.GetRootGameObjects())
            {
                foreach (var rb in root.GetComponentsInChildren<Rigidbody2D>(includeInactive: true))
                {
                    if (rb.bodyType == RigidbodyType2D.Static)
                        continue;
                    rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
                    foreach (var col in rb.GetComponents<Collider2D>())
                        if (col.sharedMaterial == null)
                            col.sharedMaterial = fallbackMaterial;
                    var seed = rb.GetComponent<InitialVelocity2DAuthoring>();
                    if (seed != null)
                    {
                        rb.linearVelocity = seed.linearVelocity;
                        rb.angularVelocity = seed.angularVelocity;
                    }
                    bodies.Add(rb);
                }
            }
            return bodies;
        }

        // Build live Rigidbody2D + collider reference bodies from the custom authoring components in the child
        // scene — verbatim port of CustomAuthoringParityHarness.BuildReferenceFromCustomAuthoring.
        protected List<Rigidbody2D> BuildReferenceFromCustomAuthoring(
            Scene childScene,
            PhysicsMaterial2D fallbackMaterial
        )
        {
            var bodies = new List<Rigidbody2D>();
            foreach (var root in childScene.GetRootGameObjects())
            {
                foreach (var bodyAuthoring in root.GetComponentsInChildren<PhysicsBody2DAuthoring>(true))
                {
                    if (bodyAuthoring.BodyType == PhysicsBody2DMotionType.Static)
                        continue;

                    var go = bodyAuthoring.gameObject;
                    var rb = go.AddComponent<Rigidbody2D>();
                    rb.bodyType =
                        bodyAuthoring.BodyType == PhysicsBody2DMotionType.Dynamic
                            ? RigidbodyType2D.Dynamic
                            : RigidbodyType2D.Kinematic;
                    rb.gravityScale = bodyAuthoring.GravityScale;
                    rb.linearDamping = bodyAuthoring.LinearDamping;
                    rb.angularDamping = bodyAuthoring.AngularDamping;
                    rb.useAutoMass = bodyAuthoring.UseAutoMass;
                    if (!bodyAuthoring.UseAutoMass && bodyAuthoring.Mass > 0f)
                        rb.mass = bodyAuthoring.Mass;
                    rb.sleepMode = RigidbodySleepMode2D.NeverSleep;

                    var shapeAuthoring = go.GetComponent<PhysicsShape2DAuthoring>();
                    if (shapeAuthoring != null)
                        AddColliderFor(go, shapeAuthoring, fallbackMaterial);

                    if (bodyAuthoring.HasInitialVelocity)
                    {
                        rb.linearVelocity = bodyAuthoring.InitialLinearVelocity;
                        rb.angularVelocity = bodyAuthoring.InitialAngularVelocity;
                    }
                    bodies.Add(rb);
                }

                foreach (var shapeAuthoring in root.GetComponentsInChildren<PhysicsShape2DAuthoring>(true))
                {
                    if (shapeAuthoring.GetComponent<PhysicsBody2DAuthoring>() != null)
                        continue;
                    AddColliderFor(shapeAuthoring.gameObject, shapeAuthoring, fallbackMaterial);
                }
            }
            return bodies;
        }

        static void AddColliderFor(GameObject go, PhysicsShape2DAuthoring shape, PhysicsMaterial2D fallbackMaterial)
        {
            Collider2D col = shape.Kind switch
            {
                PhysicsShape2DKind.Circle => MakeCircle(go, shape),
                PhysicsShape2DKind.Box => MakeBox(go, shape),
                PhysicsShape2DKind.Capsule => MakeCapsule(go, shape),
                _ => MakeCircle(go, shape),
            };
            col.offset = shape.Offset;
            col.density = shape.Density > 0f ? shape.Density : 1f;
            col.sharedMaterial = fallbackMaterial;
        }

        static Collider2D MakeCircle(GameObject go, PhysicsShape2DAuthoring shape)
        {
            var c = go.AddComponent<CircleCollider2D>();
            c.radius = shape.Radius;
            return c;
        }

        static Collider2D MakeBox(GameObject go, PhysicsShape2DAuthoring shape)
        {
            var c = go.AddComponent<BoxCollider2D>();
            c.size = shape.BoxSize;
            c.edgeRadius = shape.BoxCornerRadius;
            return c;
        }

        static Collider2D MakeCapsule(GameObject go, PhysicsShape2DAuthoring shape)
        {
            var c = go.AddComponent<CapsuleCollider2D>();
            c.size = shape.CapsuleSize;
            c.direction = shape.CapsuleVertical ? CapsuleDirection2D.Vertical : CapsuleDirection2D.Horizontal;
            return c;
        }

        protected static void SimulateReference(float dt) =>
            UnityEngine.Physics2D.Simulate(dt, UnityEngine.Physics2D.AllLayers);

        protected static void SyncReferenceTransforms() => UnityEngine.Physics2D.SyncTransforms();

        // ---- teardown: dispose the world, restore the global 2D physics state, delete the temp fixture --------

        [TearDown]
        public void TearDown()
        {
            LogAssert.ignoreFailingMessages = false;

            foreach (var rb in _refDestroyables)
                if (rb != null)
                    UnityEngine.Object.DestroyImmediate(rb);
            _refDestroyables.Clear();

            if (World is { IsCreated: true })
                World.Dispose();
            World = null;
            FixedGroup = null;

            if (_physicsGlobalsCaptured)
            {
                UnityEngine.Physics2D.gravity = _prevGravity;
                UnityEngine.Physics2D.simulationMode = _prevMode;
                _physicsGlobalsCaptured = false;
            }

            if (_refSceneOpen && _refScene.IsValid())
                EditorSceneManager.CloseScene(_refScene, removeScene: true);
            _refSceneOpen = false;

            if (!string.IsNullOrEmpty(_folder) && AssetDatabase.IsValidFolder(_folder))
            {
                AssetDatabase.DeleteAsset(_folder);
                AssetDatabase.Refresh();
            }
            _folder = null;
            _childPath = null;
        }

        // ---- parity orchestration (the EditMode analogue of PhysicsParityHarness.RunParity) ---------------

        /// <summary>
        /// Full GameObject-vs-ECS parity gate for one built-in-authored fixture. Authors the fixture, bakes it in
        /// the ECS Game world AND opens the same child scene additively as live built-in bodies, steps both
        /// <paramref name="stepCount"/> times in lockstep, and asserts the disqualifiers + growth-bounded
        /// envelope. The EditMode analogue of <c>PhysicsParityHarness.RunParity</c>; the gate supplies the
        /// fixture populate + child name + step params + envelope.
        /// </summary>
        protected void RunParity(
            Action<GameObject> populate,
            string sceneName,
            float dt,
            int stepCount,
            ParityEnvelope envelope
        )
        {
            RunParityCore(populate, sceneName, dt, stepCount, envelope, customAuthoredReference: false);
        }

        /// <summary>
        /// Like <see cref="RunParity"/>, but the child authoring scene carries
        /// <c>PhysicsBody2DAuthoring</c>/<c>PhysicsShape2DAuthoring</c> (not built-in components), so the
        /// GameObject reference is built live from those custom fields. The EditMode analogue of
        /// <c>CustomAuthoringParityHarness.RunCustomAuthoredGameObjectParity</c>.
        /// </summary>
        protected void RunCustomAuthoredGameObjectParity(
            Action<GameObject> populate,
            string sceneName,
            float dt,
            int stepCount,
            ParityEnvelope envelope
        )
        {
            RunParityCore(populate, sceneName, dt, stepCount, envelope, customAuthoredReference: true);
        }

        /// <summary>
        /// Single-faller parity against a GameObject oracle whose <c>Physics2D.gravity</c> is set to
        /// <paramref name="gravity"/> (the configurable-gravity gates need a per-fixture gravity; the base parity
        /// hardcodes -9.81). Loads the fixture, builds the reference at <paramref name="gravity"/>, steps both in
        /// lockstep, and asserts a growth-bounded position band + a >1 m travel disqualifier. Mirror of
        /// <c>Phase11StepConfigGate.RunParityAgainstOracle</c>.
        /// </summary>
        protected void RunParityWithGravity(
            Action<GameObject> populate,
            string sceneName,
            float2 gravity,
            int stepCount,
            float positionBaseMeters,
            float positionGrowthPerStep
        )
        {
            LoadSubScene(populate, sceneName);

            var liveQuery = Query(ComponentType.ReadOnly<PhysicsBody2D>(), ComponentType.ReadOnly<LocalToWorld>());
            var bakedQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            Assert.Greater(bakedQuery.CalculateEntityCount(), 0, $"No baked faller in '{sceneName}'.");

            // Reference at the per-fixture gravity (override the harness's -9.81 default before opening the scene).
            _prevMode = UnityEngine.Physics2D.simulationMode;
            _prevGravity = UnityEngine.Physics2D.gravity;
            _physicsGlobalsCaptured = true;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = new Vector2(gravity.x, gravity.y);
            var childScene = OpenReferenceSceneNoCapture();

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
                $"Body count mismatch in '{sceneName}': baked {bakedQuery.CalculateEntityCount()} vs reference {refBodies.Count}."
            );
            SyncReferenceTransforms();

            CreateBodies();

            var ecsTraj = new float2[stepCount];
            var refTraj = new float2[stepCount];
            for (var s = 0; s < stepCount; s++)
            {
                FixedGroup.Update();
                using (var ltws = liveQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp))
                    ecsTraj[s] = new float2(ltws[0].Value.c3.x, ltws[0].Value.c3.y);
                SimulateReference(FixedDt);
                refTraj[s] = new float2(refBodies[0].position.x, refBodies[0].position.y);
            }

            var worst = 0f;
            string violation = null;
            for (var s = 0; s < stepCount; s++)
            {
                var band = positionBaseMeters + positionGrowthPerStep * (s + 1);
                var dp = length(ecsTraj[s] - refTraj[s]);
                worst = max(worst, dp);
                if (violation == null && dp > band)
                    violation =
                        $"Position parity broke at step {s}: |ECS - GameObject| = {dp} m exceeds the band {band} m. "
                        + $"ECS={ecsTraj[s]}, GameObject={refTraj[s]} (gravity {gravity}).";
            }
            var travel = length(ecsTraj[stepCount - 1] - ecsTraj[0]);
            Debug.Log(
                $"[PHYSICS2D-PARITY-GRAVITY-EDITMODE] scene={sceneName} gravity={gravity} WORST_POS_ERR={worst:E6} travel={travel:F3}"
            );
            Assert.Greater(travel, 1.0f, $"The baked faller barely moved ({travel:F3} m) — a silently no-op bake.");
            Assert.IsNull(violation, violation);
        }

        Scene OpenReferenceSceneNoCapture()
        {
            _refScene = EditorSceneManager.OpenScene(_childPath, OpenSceneMode.Additive);
            _refSceneOpen = true;
            return _refScene;
        }

        /// <summary>
        /// Tight ECS-vs-ECS displacement parity for a fixture authoring exactly two dynamic bodies (custom +
        /// built-in), compared start-relative. Mirror of <c>CustomAuthoringParityHarness.RunCustomVsBuiltInParity</c>:
        /// both bodies bake into one world and run the same v3 solver, so they must agree near-exactly. Body 0 is
        /// the more-negative X (custom), body 1 the more-positive (built-in).
        /// </summary>
        protected void RunCustomVsBuiltInParity(
            Action<GameObject> populate,
            string sceneName,
            int stepCount,
            float nearExactMeters,
            float nearExactRadians
        )
        {
            LoadSubScene(populate, sceneName);

            var liveQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var bakedQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            Assert.AreEqual(2, CountDynamic(bakedQuery), $"Expected 2 dynamic baked bodies in '{sceneName}'.");

            CreateBodies();
            Assert.AreEqual(2, CountDynamic(liveQuery), "Body creation did not run for both baked bodies.");

            var traj = new float2[stepCount][];
            var ang = new float[stepCount][];
            for (var s = 0; s < stepCount; s++)
            {
                FixedGroup.Update();
                CaptureByStartX(liveQuery, out traj[s], out ang[s]);
            }

            Assert.AreEqual(2, traj[0].Length, "The tight gate requires exactly two dynamic bodies.");
            var custom0 = traj[0][0];
            var builtin0 = traj[0][1];
            var worstPos = 0f;
            var worstAng = 0f;
            string posViolation = null;
            string angViolation = null;
            for (var s = 0; s < stepCount; s++)
            {
                var customDisp = traj[s][0] - custom0;
                var builtinDisp = traj[s][1] - builtin0;
                var dp = length(customDisp - builtinDisp);
                var da = abs(AngleDelta(ang[s][0], ang[s][1]));
                worstPos = max(worstPos, dp);
                worstAng = max(worstAng, da);
                if (posViolation == null && dp > nearExactMeters)
                    posViolation =
                        $"Custom-vs-built-in displacement diverged at step {s}: {dp} m exceeds {nearExactMeters} m.";
                if (angViolation == null && da > nearExactRadians)
                    angViolation =
                        $"Custom-vs-built-in angle diverged at step {s}: {da} rad exceeds {nearExactRadians} rad.";
            }
            var customDrop = abs((traj[stepCount - 1][0] - custom0).y);
            var builtinDrop = abs((traj[stepCount - 1][1] - builtin0).y);
            Debug.Log(
                $"[PHYSICS2D-CUSTOM-PARITY-EDITMODE] scene={sceneName} WORST_POS_ERR={worstPos:E6} "
                    + $"WORST_ANG_ERR={worstAng:E6} customDrop={customDrop:F4} builtinDrop={builtinDrop:F4}"
            );
            string travelViolation = null;
            if (customDrop < 0.5f || builtinDrop < 0.5f)
                travelViolation =
                    $"A body barely moved (customDrop={customDrop}, builtinDrop={builtinDrop}) — a no-op bake.";
            Assert.IsNull(travelViolation, travelViolation);
            Assert.IsNull(posViolation, posViolation);
            Assert.IsNull(angViolation, angViolation);
        }

        static void CaptureByStartX(EntityQuery liveQuery, out float2[] positions, out float[] angles)
        {
            using var ltws = liveQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            using var defs = liveQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var list = new List<(float2 pos, float ang)>();
            for (var i = 0; i < ltws.Length; i++)
            {
                if (defs[i].bodyType != PhysicsBody.BodyType.Dynamic)
                    continue;
                var m = ltws[i].Value;
                list.Add((new float2(m.c3.x, m.c3.y), atan2(m.c0.y, m.c0.x)));
            }
            list.Sort((a, b) => a.pos.x.CompareTo(b.pos.x));
            positions = new float2[list.Count];
            angles = new float[list.Count];
            for (var i = 0; i < list.Count; i++)
            {
                positions[i] = list[i].pos;
                angles[i] = list[i].ang;
            }
        }

        void RunParityCore(
            Action<GameObject> populate,
            string sceneName,
            float dt,
            int stepCount,
            ParityEnvelope envelope,
            bool customAuthoredReference
        )
        {
            LoadSubScene(populate, sceneName);
            FixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(dt);

            var bakedQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var liveQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            var bakedCount = CountNonStatic(bakedQuery);
            Assert.Greater(bakedCount, 0, $"No baked dynamic body in fixture '{sceneName}'.");

            // GameObject reference: open the same child scene additively (Script mode entered first so nothing
            // auto-steps), build/collect the live reference bodies.
            var fallback = NewReferenceMaterial("ParityReference");
            var childScene = OpenReferenceScene();
            var bodies = customAuthoredReference
                ? BuildReferenceFromCustomAuthoring(childScene, fallback)
                : CollectReferenceBodies(childScene, fallback);
            Assert.AreEqual(
                bakedCount,
                bodies.Count,
                $"Authored body count mismatch in '{sceneName}': SubScene baked {bakedCount} non-static bodies "
                    + $"but the child scene instantiated {bodies.Count} live Rigidbody2D bodies."
            );
            SyncReferenceTransforms();

            // First group Update runs body creation + write-back (no integration); both backends sit at the
            // shared authored start, then advance one step per loop iteration.
            CreateBodies();
            Assert.AreEqual(
                bakedCount,
                CountNonStatic(liveQuery),
                $"Body creation did not run for every baked body in '{sceneName}'."
            );

            var ecsTraj = new Pose[stepCount][];
            var refTraj = new Pose[stepCount][];
            for (var s = 0; s < stepCount; s++)
            {
                FixedGroup.Update();
                ecsTraj[s] = CaptureEcsPoses(liveQuery);
                SimulateReference(dt);
                refTraj[s] = CaptureReferencePoses(bodies);
            }

            AssertParity(ecsTraj, refTraj, dt, envelope);
        }

        // ---- parity capture/compare (verbatim ports of the PlayMode harness math) -------------------------

        public struct Pose
        {
            public float2 position;
            public float angleRadians;
        }

        public struct ParityEnvelope
        {
            public float positionBaseMeters;
            public float positionGrowthPerStep;
            public float angleCapRadians;
            public float2 settleRegionMin;
            public float2 settleRegionMax;
            public float minTravelMeters;
        }

        protected static Pose[] CaptureReferencePoses(List<Rigidbody2D> bodies)
        {
            var poses = new Pose[bodies.Count];
            for (var i = 0; i < bodies.Count; i++)
                poses[i] = new Pose
                {
                    position = new float2(bodies[i].position.x, bodies[i].position.y),
                    angleRadians = radians(bodies[i].rotation),
                };
            return poses;
        }

        // Capture the non-static baked bodies in query order, symmetric with the GameObject reference.
        protected static Pose[] CaptureEcsPoses(EntityQuery liveQuery)
        {
            using var ltws = liveQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            using var defs = liveQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var poses = new List<Pose>(ltws.Length);
            for (var i = 0; i < ltws.Length; i++)
            {
                if (defs[i].bodyType == PhysicsBody.BodyType.Static)
                    continue;
                var m = ltws[i].Value;
                poses.Add(new Pose { position = new float2(m.c3.x, m.c3.y), angleRadians = atan2(m.c0.y, m.c0.x) });
            }
            return poses.ToArray();
        }

        protected static int CountNonStatic(EntityQuery query)
        {
            using var defs = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var n = 0;
            for (var i = 0; i < defs.Length; i++)
                if (defs[i].bodyType != PhysicsBody.BodyType.Static)
                    n++;
            return n;
        }

        protected static int CountDynamic(EntityQuery query)
        {
            using var defs = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var n = 0;
            for (var i = 0; i < defs.Length; i++)
                if (defs[i].bodyType == PhysicsBody.BodyType.Dynamic)
                    n++;
            return n;
        }

        // Verbatim port of PhysicsParityHarness.AssertParity (the growth-bounded band + disqualifiers).
        protected static void AssertParity(Pose[][] ecsTraj, Pose[][] refTraj, float dt, ParityEnvelope envelope)
        {
            var stepCount = ecsTraj.Length;
            var bodyCount = ecsTraj[0].Length;

            var ecsOrder = OrderByInitialPose(ecsTraj[0]);
            var refOrder = OrderByInitialPose(refTraj[0]);

            var log = new System.Text.StringBuilder();
            log.AppendLine($"[PHYSICS2D-PARITY-EDITMODE] bodies={bodyCount} steps={stepCount} dt={dt}");
            log.AppendLine(
                $"[PHYSICS2D-PARITY-EDITMODE] envelope: posBase={envelope.positionBaseMeters} "
                    + $"posGrowth/step={envelope.positionGrowthPerStep} angCap={envelope.angleCapRadians}"
            );
            log.AppendLine("step\tmaxPosErr\tmeanPosErr\tmaxAngErr\tposBand");

            var worstPos = 0f;
            var worstAng = 0f;
            string posViolation = null;
            string angViolation = null;
            string nanViolation = null;
            for (var s = 0; s < stepCount; s++)
            {
                var posBand = envelope.positionBaseMeters + envelope.positionGrowthPerStep * (s + 1);
                var maxPos = 0f;
                var sumPos = 0f;
                var maxAng = 0f;
                for (var b = 0; b < bodyCount; b++)
                {
                    var e = ecsTraj[s][ecsOrder[b]];
                    var r = refTraj[s][refOrder[b]];

                    if (
                        nanViolation == null
                        && (
                            isnan(e.position.x)
                            || isnan(e.position.y)
                            || isinf(e.position.x)
                            || isinf(e.position.y)
                            || isnan(e.angleRadians)
                            || isinf(e.angleRadians)
                        )
                    )
                        nanViolation =
                            $"ECS body {b} produced NaN/Inf at step {s}: pos={e.position}, ang={e.angleRadians}.";

                    var dp = length(e.position - r.position);
                    var da = abs(AngleDelta(e.angleRadians, r.angleRadians));
                    maxPos = max(maxPos, dp);
                    sumPos += dp;
                    maxAng = max(maxAng, da);

                    if (posViolation == null && dp > posBand)
                        posViolation =
                            $"Position parity broke at step {s}, body {b}: |ECS - GameObject| = {dp} m "
                            + $"exceeds the growth-bounded band {posBand} m. ECS={e.position}, GameObject={r.position}.";
                    if (angViolation == null && da > envelope.angleCapRadians)
                        angViolation =
                            $"Angle parity broke at step {s}, body {b}: |ECS - GameObject| = {da} rad "
                            + $"exceeds the cap {envelope.angleCapRadians} rad. ECS={e.angleRadians}, GameObject={r.angleRadians}.";
                }
                worstPos = max(worstPos, maxPos);
                worstAng = max(worstAng, maxAng);
                log.AppendLine($"{s}\t{maxPos:E6}\t{(sumPos / bodyCount):E6}\t{maxAng:E6}\t{posBand:E6}");
            }

            string travelViolation = null;
            string settleViolation = null;
            for (var b = 0; b < bodyCount; b++)
            {
                var start = ecsTraj[0][ecsOrder[b]].position;
                var end = ecsTraj[stepCount - 1][ecsOrder[b]].position;
                var travel = length(end - start);
                if (travelViolation == null && travel < envelope.minTravelMeters)
                    travelViolation =
                        $"Body {b} barely moved ({travel} m < {envelope.minTravelMeters} m) — a silently no-op bake.";
                if (
                    settleViolation == null
                    && !(
                        end.x >= envelope.settleRegionMin.x
                        && end.x <= envelope.settleRegionMax.x
                        && end.y >= envelope.settleRegionMin.y
                        && end.y <= envelope.settleRegionMax.y
                    )
                )
                    settleViolation =
                        $"Body {b} ended outside the expected region: end={end}, "
                        + $"region=[{envelope.settleRegionMin}..{envelope.settleRegionMax}].";
            }

            log.AppendLine($"[PHYSICS2D-PARITY-EDITMODE] WORST_POS_ERR={worstPos:E6} WORST_ANG_ERR={worstAng:E6}");
            if (nanViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY-EDITMODE] NAN: {nanViolation}");
            if (posViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY-EDITMODE] POS: {posViolation}");
            if (angViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY-EDITMODE] ANG: {angViolation}");
            if (travelViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY-EDITMODE] TRAVEL: {travelViolation}");
            if (settleViolation != null)
                log.AppendLine($"[PHYSICS2D-PARITY-EDITMODE] SETTLE: {settleViolation}");
            Debug.Log(log.ToString());

            Assert.IsNull(nanViolation, nanViolation);
            Assert.IsNull(travelViolation, travelViolation);
            Assert.IsNull(settleViolation, settleViolation);
            Assert.IsNull(posViolation, posViolation);
            Assert.IsNull(angViolation, angViolation);
        }

        static int[] OrderByInitialPose(Pose[] firstStep)
        {
            var order = new int[firstStep.Length];
            for (var i = 0; i < order.Length; i++)
                order[i] = i;
            Array.Sort(
                order,
                (a, b) =>
                {
                    var pa = firstStep[a].position;
                    var pb = firstStep[b].position;
                    var cy = pa.y.CompareTo(pb.y);
                    return cy != 0 ? cy : pa.x.CompareTo(pb.x);
                }
            );
            return order;
        }

        protected static float AngleDelta(float a, float b)
        {
            var d = a - b;
            while (d > PI)
                d -= 2f * PI;
            while (d < -PI)
                d += 2f * PI;
            return d;
        }
    }
}
