using System.Collections;
using System.Collections.Generic;
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
    /// Phase-10a hard GameObject-parity gate for the three force-field effectors (Area / Buoyancy / Point).
    /// The fresh-eyes validating gate the Phase-10a self-review escalated: each invariant is driven in BOTH
    /// mediums — the package (Box2D-v3, a disposable ECS World stepped one fixed step per group update) and a
    /// GameObject oracle (Box2D-v2, the real <c>AreaEffector2D</c>/<c>BuoyancyEffector2D</c>/<c>PointEffector2D</c>
    /// authored on a trigger collider + a test <c>Rigidbody2D</c>, stepped via
    /// <c>Physics2D.simulationMode = Script</c> + <c>Physics2D.Simulate(dt)</c>) — and the cross-medium delta is
    /// asserted within a generous envelope, or measured-and-documented as a known gap.
    /// </summary>
    /// <remarks>
    /// <para><b>Why a code-authored dual-medium driver, not the SubScene <see cref="PhysicsParityHarness"/>.</b>
    /// The SubScene harness authors a single built-in source and bakes it for the ECS side, but its GameObject
    /// reference collects only live <c>Rigidbody2D</c> bodies — it has no effector-authoring path, and the
    /// effector force is applied PRE-Simulate (a window the harness's lockstep loop does not expose around the
    /// reference's own <c>Physics2D.Simulate</c>). An effector is authored identically from code on both sides
    /// here (the package side via <see cref="DirectPhysics2DAuthoring"/> — the runtime archetype the bakers
    /// produce, proven equivalent by the Phase-10a smoke; the GameObject side via the real component on a trigger
    /// collider), so a single authored intent feeds both, deterministically (every test sets variation 0).</para>
    ///
    /// <para><b>Why bit-identical deterministic witnesses across two runs.</b> Every effector force here is
    /// variation 0, every body starts from rest at an authored pose, and every step count is fixed — so both the
    /// package and the GameObject sides produce the SAME numbers on every run. The two-consecutive-green evidence
    /// is the identical witness table on both runs; a thrown test that leaked a native body into a shared world
    /// would perturb a later test, which is why every world-mutating test owns a disposable <see cref="World"/>
    /// and tears down all global <c>Physics2D</c> state it touched.</para>
    ///
    /// <para><b>Matched gravity is its own pin.</b> The buoyancy equilibrium balances the baked
    /// <c>gravityMagnitude</c> against the body's integrated gravity, so the comparison is meaningless unless the
    /// package world's gravity equals <c>Physics2D.gravity</c>. <see cref="GravityMatch_PackageWorldEqualsPhysics2D"/>
    /// pins that equality directly; every buoyancy test then sets both mediums to the same gravity.</para>
    ///
    /// <para><b>The known gap.</b> The package approximates the buoyancy submerged fraction by the body's AABB
    /// vertical extent; the GameObject Box2D-v2 buoyancy uses the true submerged-AREA of the collider shape. For a
    /// CIRCLE the two diverge away from the half-submerged point (a circle's AABB-depth fraction is not its
    /// area fraction). <see cref="Buoyancy_CircleEquilibriumDepth_MeasuredBothMedia"/> measures both equilibria and
    /// reports the delta; it is GREEN within a generous band and the measured numbers are recorded as the
    /// documented AABB-vs-true-area gap, not forced to a false bit-match.</para>
    /// </remarks>
    public sealed class Phase10aForceFieldEffectorGate
    {
        const float Dt = 1f / 60f;

        // The package world's gravity (read once from PhysicsWorldDefinition.defaultDefinition, the def
        // PhysicsWorld2DSystem.CreateWorld uses). Set on the GameObject medium so both integrate under the same
        // gravity — the precondition for any buoyancy/drag/trajectory comparison. Resolved in OneTimeSetUp.
        static Vector2 s_PackageGravity;
        static float s_PackageGravityMag;

        [OneTimeSetUp]
        public void ResolvePackageGravity()
        {
            // PhysicsWorld2DSystem.CreateWorld() builds from PhysicsWorldDefinition.defaultDefinition with only
            // drawOptions/simulationType changed — gravity is the default. Read that exact value here so the
            // GameObject medium is set to the SAME gravity (not an assumed 9.81), and the gravity-match pin
            // asserts the magnitude equality the buoyancy balance depends on.
            s_PackageGravity = (Vector2)PhysicsWorldDefinition.defaultDefinition.gravity;
            s_PackageGravityMag = s_PackageGravity.magnitude;
        }

        // ----------------------------------------------------------------------------------------------------
        // Package medium: a disposable ECS World with the four FixedStep systems, one fixed step per Update.

        sealed class PackageMedium : System.IDisposable
        {
            readonly World _world;
            readonly FixedStepSimulationSystemGroup _group;
            bool _created;

            public PackageMedium()
            {
                _world = new World("Physics2DEffectorGateWorld");
                _group = _world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
                _group.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
                _group.AddSystemToUpdateList(_world.GetOrCreateSystem<PhysicsWorld2DSystem>());
                _group.AddSystemToUpdateList(_world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
                _group.AddSystemToUpdateList(_world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
                _group.SortSystems();
            }

            public EntityManager Em => _world.EntityManager;

            // A static effector: a sensor (isTrigger) box/circle region carrying a PhysicsEffector2D — the exact
            // archetype AreaEffector2DBaker/etc. produce (a collider-only static body + a sensor shape + the
            // definition).
            public Entity SpawnEffector(float2 pos, PhysicsShape2D region, PhysicsEffector2D eff)
            {
                region.isTrigger = true;
                var entity = DirectPhysics2DAuthoring.Create(
                    Em,
                    new PhysicsBody2DDefinition
                    {
                        bodyType = PhysicsBody.BodyType.Static,
                        initialPosition = pos,
                        useAutoMass = false,
                    },
                    region
                );
                Em.AddComponentData(entity, eff);
                return entity;
            }

            // A static effector at an authored body ROTATION — for the Area local-angle direction variant, where
            // the zone force angle is relative to the effector body's rotation. The rotation rides the body
            // definition's initialRotationRadians; the runtime reads body.rotation for the local-angle add.
            public Entity SpawnEffectorRotated(
                float2 pos,
                float rotationRadians,
                PhysicsShape2D region,
                PhysicsEffector2D eff
            )
            {
                region.isTrigger = true;
                var entity = DirectPhysics2DAuthoring.Create(
                    Em,
                    new PhysicsBody2DDefinition
                    {
                        bodyType = PhysicsBody.BodyType.Static,
                        initialPosition = pos,
                        initialRotationRadians = rotationRadians,
                        useAutoMass = false,
                    },
                    region
                );
                Em.AddComponentData(entity, eff);
                return entity;
            }

            // A dynamic affected circle. density 1 → a unit-density body; gravityScale caller-chosen.
            // categoryBits sets the body's layer bit for the colliderMask exclusion pin (0 = default everything).
            public Entity SpawnBody(
                float2 pos,
                float radius,
                float gravityScale,
                ulong categoryBits = 0ul,
                ulong contactBits = 0ul
            )
            {
                return DirectPhysics2DAuthoring.Create(
                    Em,
                    new PhysicsBody2DDefinition
                    {
                        bodyType = PhysicsBody.BodyType.Dynamic,
                        gravityScale = gravityScale,
                        initialPosition = pos,
                        useAutoMass = true,
                    },
                    new PhysicsShape2D
                    {
                        kind = PhysicsShape2DKind.Circle,
                        radius = radius,
                        density = 1f,
                        friction = 0.4f,
                        categoryBits = categoryBits,
                        contactBits = contactBits,
                    }
                );
            }

            public PhysicsBody BodyOf(Entity e) => Em.GetComponentData<PhysicsBody2D>(e).body;

            // First Update creates the Box2D bodies (no step). Returns the created live body for a spawned entity.
            public void Create()
            {
                _group.Update();
                _created = true;
            }

            public void Step()
            {
                Assert.IsTrue(_created, "PackageMedium.Create() must run before Step().");
                _group.Update();
            }

            public float2 Velocity(Entity e) => (float2)(Vector2)BodyOf(e).linearVelocity;

            public float2 Position(Entity e) => (float2)(Vector2)BodyOf(e).position;

            public void Dispose() => _world.Dispose();
        }

        // ----------------------------------------------------------------------------------------------------
        // GameObject medium: real *Effector2D on a trigger collider + a test Rigidbody2D body, Physics2D.Simulate.
        // Fully qualified UnityEngine.Physics2D: this namespace's "Physics2D" segment shadows the bare token.

        sealed class GameObjectMedium : System.IDisposable
        {
            readonly SimulationMode2D _prevMode;
            readonly Vector2 _prevGravity;
            readonly List<GameObject> _spawned = new();

            public GameObjectMedium(Vector2 gravity)
            {
                _prevMode = UnityEngine.Physics2D.simulationMode;
                _prevGravity = UnityEngine.Physics2D.gravity;
                UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
                UnityEngine.Physics2D.gravity = gravity;
            }

            // Author a force-field effector on a GameObject carrying a trigger collider used by the effector.
            // `configure` sets the effector's typed fields. The region collider is a sensor (isTrigger + usedBy
            // Effector), so a body overlaps without a collision response (it floats / falls through), exactly as
            // the example effector scenes author it.
            GameObject SpawnEffectorBoxRegion<TEffector>(
                Vector2 pos,
                Vector2 boxSize,
                System.Action<TEffector> configure
            )
                where TEffector : Effector2D
            {
                var go = new GameObject(typeof(TEffector).Name + "Region");
                go.transform.position = pos;
                var box = go.AddComponent<BoxCollider2D>();
                box.size = boxSize;
                box.isTrigger = true;
                box.usedByEffector = true;
                var eff = go.AddComponent<TEffector>();
                configure(eff);
                _spawned.Add(go);
                return go;
            }

            GameObject SpawnEffectorCircleRegion<TEffector>(
                Vector2 pos,
                float radius,
                System.Action<TEffector> configure
            )
                where TEffector : Effector2D
            {
                var go = new GameObject(typeof(TEffector).Name + "Region");
                go.transform.position = pos;
                var circle = go.AddComponent<CircleCollider2D>();
                circle.radius = radius;
                circle.isTrigger = true;
                circle.usedByEffector = true;
                var eff = go.AddComponent<TEffector>();
                configure(eff);
                _spawned.Add(go);
                return go;
            }

            public void SpawnAreaBox(Vector2 pos, Vector2 size, System.Action<AreaEffector2D> configure) =>
                SpawnEffectorBoxRegion(pos, size, configure);

            public void SpawnBuoyancyBox(Vector2 pos, Vector2 size, System.Action<BuoyancyEffector2D> configure) =>
                SpawnEffectorBoxRegion(pos, size, configure);

            public void SpawnPointCircle(Vector2 pos, float radius, System.Action<PointEffector2D> configure) =>
                SpawnEffectorCircleRegion(pos, radius, configure);

            // A dynamic affected circle body. density 1 (auto-mass) so its mass equals the package body's. layer
            // sets the GameObject layer for the colliderMask exclusion pin.
            public Rigidbody2D SpawnBody(Vector2 pos, float radius, float gravityScale, int layer = 0)
            {
                var go = new GameObject("AffectedBody") { layer = layer };
                go.transform.position = pos;
                var rb = go.AddComponent<Rigidbody2D>();
                rb.bodyType = RigidbodyType2D.Dynamic;
                rb.gravityScale = gravityScale;
                rb.useAutoMass = true;
                rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
                var circle = go.AddComponent<CircleCollider2D>();
                circle.radius = radius;
                circle.density = 1f;
                _spawned.Add(go);
                UnityEngine.Physics2D.SyncTransforms();
                return rb;
            }

            public void Step() => UnityEngine.Physics2D.Simulate(Dt);

            public void Dispose()
            {
                foreach (var go in _spawned)
                    if (go != null)
                        Object.Destroy(go);
                UnityEngine.Physics2D.gravity = _prevGravity;
                UnityEngine.Physics2D.simulationMode = _prevMode;
            }
        }

        // ====================================================================================================
        // GRAVITY-MATCH PIN — the precondition every buoyancy/drag comparison depends on.

        [Test]
        public void GravityMatch_PackageWorldEqualsPhysics2D()
        {
            // The package world is created from PhysicsWorldDefinition.defaultDefinition; the GameObject medium is
            // set to that same gravity. The buoyancy effector bakes gravityMagnitude from Physics2D.gravity.
            // .magnitude (Effector2DBaking.GravityMagnitude). If the default world gravity and Physics2D.gravity
            // disagree, the baked buoyancy balance is wrong and every buoyancy parity is meaningless — so this is
            // pinned as its own fact, NOT assumed 9.81.
            var physics2DGravity = (Vector2)UnityEngine.Physics2D.gravity;
            Debug.Log(
                $"[P10A-GATE-GRAVITY] packageWorldGravity={s_PackageGravity} (|{s_PackageGravityMag:F4}|) "
                    + $"Physics2D.gravity={physics2DGravity} (|{physics2DGravity.magnitude:F4}|)"
            );

            Assert.AreEqual(
                physics2DGravity.magnitude,
                s_PackageGravityMag,
                1e-3f,
                $"Package world gravity magnitude {s_PackageGravityMag} != Physics2D.gravity magnitude "
                    + $"{physics2DGravity.magnitude}. The baked buoyancy gravityMagnitude balances against the "
                    + "package world's gravity, so a mismatch breaks every buoyancy equilibrium parity."
            );
            // The default-def gravity must be a real downward gravity (not zero), or buoyancy has nothing to
            // balance and the equilibrium test is vacuous.
            Assert.Greater(
                s_PackageGravityMag,
                1f,
                "Package world gravity is ~zero — buoyancy has nothing to balance."
            );
        }

        // ====================================================================================================
        // AREA — directional acceleration, drag-equilibrium terminal velocity, direction variants.

        [UnityTest]
        public IEnumerator Area_DirectionalAcceleration_MatchesGameObject()
        {
            // A +X zone, magnitude 50, NO drag — a clean acceleration read. A gravity-free body at rest in the
            // zone accelerates along +X at forceMagnitude/mass; the velocity after N steps must match GameObject
            // within a generous band. Pins the core directional-acceleration decision point in both media.
            const int steps = 20;
            const float mag = 50f;

            float2 pkgV,
                pkgP;
            using (var pkg = new PackageMedium())
            {
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(20f, 20f) },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Area,
                        colliderMask = 0ul,
                        forceMagnitude = mag,
                        forceAngleRadians = 0f,
                        useGlobalAngle = 1,
                        forceTargetIsRigidbody = 1,
                    }
                );
                var body = pkg.SpawnBody(new float2(0f, 0f), 0.5f, gravityScale: 0f);
                pkg.Create();
                for (var i = 0; i < steps; i++)
                    pkg.Step();
                pkgV = pkg.Velocity(body);
                pkgP = pkg.Position(body);
            }

            Vector2 goV,
                goP;
            using (var go = new GameObjectMedium(s_PackageGravity))
            {
                go.SpawnAreaBox(
                    new Vector2(0f, 0f),
                    new Vector2(20f, 20f),
                    a =>
                    {
                        a.forceMagnitude = mag;
                        a.forceAngle = 0f;
                        a.forceVariation = 0f;
                        a.useGlobalAngle = true;
                        a.forceTarget = EffectorSelection2D.Rigidbody;
                        a.linearDamping = 0f;
                        a.angularDamping = 0f;
                    }
                );
                var rb = go.SpawnBody(new Vector2(0f, 0f), 0.5f, gravityScale: 0f);
                for (var i = 0; i < steps; i++)
                    go.Step();
                goV = rb.linearVelocity;
                goP = rb.position;
                yield return null;
            }

            Debug.Log(
                $"[P10A-GATE-AREA-ACCEL] mag={mag} steps={steps}: pkg v={pkgV} p={pkgP} | " + $"GO v={goV} p={goP}"
            );

            // Both must accelerate in +X with negligible Y drift.
            Assert.Greater(pkgV.x, 1f, $"Package body did not accelerate +X (v={pkgV}).");
            Assert.Greater(goV.x, 1f, $"GameObject body did not accelerate +X (v={goV}).");
            Assert.Less(abs(pkgV.y), 0.1f, $"Package body Y drift (v={pkgV}).");
            Assert.Less(abs(goV.y), 0.1f, $"GameObject body Y drift (v={goV}).");

            // Cross-medium velocity parity within a generous band (v2-vs-v3 + the mass auto-compute convergence).
            // A unit-density radius-0.5 circle has mass ~0.785 on both; accel ~64 m/s2; v after 20 steps ~21 m/s.
            // 15% relative band absorbs the cross-solver integration convention without masking a wrong direction
            // or a wrong magnitude.
            var rel = abs(pkgV.x - goV.x) / max(abs(goV.x), 1e-3f);
            Assert.Less(
                rel,
                0.15f,
                $"Area directional velocity diverged: pkg vx={pkgV.x}, GO vx={goV.x} (rel={rel:F3} > 0.15)."
            );
        }

        [UnityTest]
        public IEnumerator Area_DragEquilibrium_TerminalVelocity_MatchesGameObject()
        {
            // The escalated drag-equilibrium case: under linear drag the body reaches a terminal velocity where
            // the per-step force gain balances the per-step drag decay. With force-accel a = forceMag/mass and the
            // velocity multiplier v *= 1/(1+c*dt), the fixed point is v* = a/c. Step long enough that both media
            // settle at terminal, then assert the SAME terminal velocity in both. forceMag 50, drag 4.
            const int steps = 400;
            const float mag = 50f;
            const float drag = 4f;

            float2 pkgVPrev,
                pkgV;
            using (var pkg = new PackageMedium())
            {
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(40f, 40f) },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Area,
                        colliderMask = 0ul,
                        forceMagnitude = mag,
                        forceAngleRadians = 0f,
                        useGlobalAngle = 1,
                        forceTargetIsRigidbody = 1,
                        linearDamping = drag,
                    }
                );
                var body = pkg.SpawnBody(new float2(0f, 0f), 0.5f, gravityScale: 0f);
                pkg.Create();
                for (var i = 0; i < steps - 1; i++)
                    pkg.Step();
                pkgVPrev = pkg.Velocity(body);
                pkg.Step();
                pkgV = pkg.Velocity(body);
            }

            Vector2 goVPrev,
                goV;
            using (var go = new GameObjectMedium(s_PackageGravity))
            {
                go.SpawnAreaBox(
                    new Vector2(0f, 0f),
                    new Vector2(40f, 40f),
                    a =>
                    {
                        a.forceMagnitude = mag;
                        a.forceAngle = 0f;
                        a.forceVariation = 0f;
                        a.useGlobalAngle = true;
                        a.forceTarget = EffectorSelection2D.Rigidbody;
                        a.linearDamping = drag;
                    }
                );
                var rb = go.SpawnBody(new Vector2(0f, 0f), 0.5f, gravityScale: 0f);
                for (var i = 0; i < steps - 1; i++)
                    go.Step();
                goVPrev = rb.linearVelocity;
                go.Step();
                goV = rb.linearVelocity;
                yield return null;
            }

            // Terminal = the velocity stopped growing (the last step changed it by < 1% of itself).
            var pkgGrowth = abs(pkgV.x - pkgVPrev.x) / max(abs(pkgV.x), 1e-3f);
            var goGrowth = abs(goV.x - goVPrev.x) / max(abs(goV.x), 1e-3f);

            Debug.Log(
                $"[P10A-GATE-AREA-DRAG] mag={mag} drag={drag} after {steps} steps: pkg vTerm={pkgV.x:F4} "
                    + $"(growth {pkgGrowth:E3}) | GO vTerm={goV.x:F4} (growth {goGrowth:E3})"
            );

            Assert.Less(pkgGrowth, 0.02f, $"Package velocity not yet terminal (vx={pkgV.x}, growth={pkgGrowth}).");
            Assert.Less(goGrowth, 0.02f, $"GameObject velocity not yet terminal (vx={goV.x}, growth={goGrowth}).");

            // The terminal velocity itself must match within a generous band. Both are at the force-vs-drag fixed
            // point; the value depends on mass (auto-computed identically) and the damping integrator (a velocity
            // multiplier on both). 15% relative band.
            var rel = abs(pkgV.x - goV.x) / max(abs(goV.x), 1e-3f);
            Assert.Less(
                rel,
                0.15f,
                $"Area drag-equilibrium terminal velocity diverged: pkg={pkgV.x}, GO={goV.x} (rel={rel:F3})."
            );
        }

        [UnityTest]
        public IEnumerator Area_LocalAngle_RotatedZoneBlowsAlongLocalAxis()
        {
            // The useGlobalAngle=false direction variant: a zone whose forceAngle is 0 but whose body is rotated
            // +90deg must blow along its LOCAL axis = world +Y, not world +X (global). Package side only — the
            // GameObject side's rotated-effector-rotation authoring through a code-spawned transform is the same
            // formula; the package decision point under test is `forceAngleRadians + effectorAngle` when
            // useGlobalAngle==0. A global-angle control proves the rotation actually changes the direction.
            const int steps = 15;

            float2 localV,
                globalV;
            using (var pkg = new PackageMedium())
            {
                // Local-angle effector, body rotated +90deg (PI/2): forceAngle 0 + bodyRot PI/2 → +Y.
                var localEff = pkg.SpawnEffectorRotated(
                    new float2(-5f, 0f),
                    PI / 2f,
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(10f, 10f) },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Area,
                        colliderMask = 0ul,
                        forceMagnitude = 50f,
                        forceAngleRadians = 0f,
                        useGlobalAngle = 0, // relative to body rotation
                        forceTargetIsRigidbody = 1,
                    }
                );
                var localBody = pkg.SpawnBody(new float2(-5f, 0f), 0.5f, gravityScale: 0f);

                // Global-angle control at the same rotation: forceAngle 0, global → +X regardless of body rot.
                pkg.SpawnEffectorRotated(
                    new float2(5f, 0f),
                    PI / 2f,
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(10f, 10f) },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Area,
                        colliderMask = 0ul,
                        forceMagnitude = 50f,
                        forceAngleRadians = 0f,
                        useGlobalAngle = 1, // world-space
                        forceTargetIsRigidbody = 1,
                    }
                );
                var globalBody = pkg.SpawnBody(new float2(5f, 0f), 0.5f, gravityScale: 0f);

                pkg.Create();
                for (var i = 0; i < steps; i++)
                    pkg.Step();
                localV = pkg.Velocity(localBody);
                globalV = pkg.Velocity(globalBody);
                yield return null;
            }

            Debug.Log(
                $"[P10A-GATE-AREA-LOCALANGLE] body rotated +90deg, forceAngle=0: localAngle v={localV} "
                    + $"(should be +Y) | globalAngle v={globalV} (should be +X)"
            );

            // Local-angle: blows along the body's local +X which, rotated +90deg, is world +Y.
            Assert.Greater(localV.y, 1f, $"Local-angle zone did not blow +Y under +90deg body rotation (v={localV}).");
            Assert.Less(abs(localV.x), 0.5f, $"Local-angle zone leaked +X (v={localV}).");
            // Global-angle control: world +X regardless of body rotation.
            Assert.Greater(globalV.x, 1f, $"Global-angle zone did not blow +X (v={globalV}).");
            Assert.Less(abs(globalV.y), 0.5f, $"Global-angle zone leaked +Y (v={globalV}).");
        }

        // ====================================================================================================
        // POINT — falloff-mode ratios at multiple distances, attract vs repel, vs GameObject.

        [UnityTest]
        public IEnumerator Point_FalloffModes_RatiosAtMultipleDistances_MatchGameObject()
        {
            // The escalated falloff-mode pin. Measure the INSTANTANEOUS acceleration (one-step velocity delta from
            // rest) at three distances d in {2, 4, 8} for each of Constant / InverseLinear / InverseSquared, in
            // both media, and assert the acceleration-vs-distance RELATIONSHIP:
            //   Constant       a(d) ≈ flat        → a(2)/a(8) ≈ 1
            //   InverseLinear  a(d) ∝ 1/d         → a(2)/a(8) ≈ 4
            //   InverseSquared a(d) ∝ 1/d²        → a(2)/a(8) ≈ 16
            // The ratio is what makes the three modes distinguishable; a single point cannot tell them apart.
            // Cross-medium: the package ratio must match the GameObject ratio (both run the documented falloff).
            var distances = new[] { 2f, 4f, 8f };
            const float baseMag = -300f; // attract (a gravity well)

            for (byte mode = 0; mode <= 2; mode++)
            {
                var pkgAccel = MeasurePackagePointAccel(mode, baseMag, distances);
                var goAccel = MeasureGameObjectPointAccel(mode, baseMag, distances);

                var modeName =
                    mode == 0 ? "Constant"
                    : mode == 1 ? "InverseLinear"
                    : "InverseSquared";
                // Ratio of accel at the nearest to the farthest distance (d=2 vs d=8, a 4x distance span).
                var pkgRatio = pkgAccel[0] / max(pkgAccel[2], 1e-4f);
                var goRatio = goAccel[0] / max(goAccel[2], 1e-4f);
                var expectedRatio =
                    mode == 0 ? 1f
                    : mode == 1 ? 4f
                    : 16f;

                Debug.Log(
                    $"[P10A-GATE-POINT-{modeName}] accel@d2={pkgAccel[0]:F3} d4={pkgAccel[1]:F3} d8={pkgAccel[2]:F3} (pkg) | "
                        + $"d2={goAccel[0]:F3} d4={goAccel[1]:F3} d8={goAccel[2]:F3} (GO) | ratio(d2/d8): "
                        + $"pkg={pkgRatio:F3} GO={goRatio:F3} expected≈{expectedRatio}"
                );

                // The package ratio matches the documented falloff (20% band: the one-step accel read carries a
                // small position-drift error since the body moves a hair during the step before the read).
                Assert.That(
                    pkgRatio,
                    Is.EqualTo(expectedRatio).Within(expectedRatio * 0.2f),
                    $"Point {modeName} package ratio {pkgRatio:F3} != expected {expectedRatio} (1/1, 1/d, 1/d²)."
                );
                // The package ratio matches the GameObject ratio (the cross-medium falloff parity).
                Assert.That(
                    pkgRatio,
                    Is.EqualTo(goRatio).Within(goRatio * 0.2f),
                    $"Point {modeName} falloff ratio diverged: pkg={pkgRatio:F3}, GO={goRatio:F3}."
                );
                yield return null;
            }
        }

        // Measure the package per-step acceleration toward the point at each distance, as the velocity DELTA over
        // one step after a warm-up step. Measuring the delta (not the absolute velocity after step 1) makes the
        // package and GameObject media symmetric: the package applies the effector force on its first step, but a
        // GameObject effector needs one Simulate to register the body's trigger overlap before the force applies,
        // so a single-step-from-rest read would see zero on the GameObject side (the trigger-enter latency).
        // Reading Δv over one step, after one warm-up step on both sides, removes that confound. The body barely
        // moves (gravity-free, sub-cm in two steps), so the distance is still effectively d. Separate world per
        // distance so each body's delta is clean.
        static float[] MeasurePackagePointAccel(byte mode, float mag, float[] distances)
        {
            var accel = new float[distances.Length];
            for (var i = 0; i < distances.Length; i++)
            {
                using var pkg = new PackageMedium();
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Circle, radius = 50f },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Point,
                        colliderMask = 0ul,
                        forceMagnitude = mag,
                        distanceScale = 1f,
                        forceMode = mode,
                        forceSourceIsRigidbody = 0,
                    }
                );
                var body = pkg.SpawnBody(new float2(distances[i], 0f), 0.5f, gravityScale: 0f);
                pkg.Create();
                pkg.Step(); // warm-up: establish the overlap, symmetric with the GameObject trigger-enter step
                var vBefore = pkg.Velocity(body);
                pkg.Step();
                var vAfter = pkg.Velocity(body);
                accel[i] = length(vAfter - vBefore) / Dt; // Δ|v|/dt = per-step acceleration magnitude
            }
            return accel;
        }

        static float[] MeasureGameObjectPointAccel(byte mode, float mag, float[] distances)
        {
            var accel = new float[distances.Length];
            for (var i = 0; i < distances.Length; i++)
            {
                using var go = new GameObjectMedium(s_PackageGravity);
                go.SpawnPointCircle(
                    new Vector2(0f, 0f),
                    50f,
                    p =>
                    {
                        p.forceMagnitude = mag;
                        p.forceVariation = 0f;
                        p.distanceScale = 1f;
                        p.forceMode = (EffectorForceMode2D)mode;
                        p.forceSource = EffectorSelection2D.Collider;
                        p.forceTarget = EffectorSelection2D.Collider;
                        p.linearDamping = 0f;
                        p.angularDamping = 0f;
                    }
                );
                var rb = go.SpawnBody(new Vector2(distances[i], 0f), 0.5f, gravityScale: 0f);
                go.Step(); // warm-up: the first Simulate registers the body's trigger overlap with the effector
                var vBefore = rb.linearVelocity;
                go.Step();
                var vAfter = rb.linearVelocity;
                accel[i] = (vAfter - vBefore).magnitude / Dt; // Δ|v|/dt = per-step acceleration magnitude
            }
            return accel;
        }

        [UnityTest]
        public IEnumerator Point_AttractAndRepel_DirectionMatchesSign()
        {
            // The sign-of-forceMagnitude decision point: a negative magnitude attracts (the body moves TOWARD the
            // point), a positive magnitude repels (AWAY). Pinned in both media, Constant mode.
            const int steps = 12;

            float pkgAttractX,
                pkgRepelX,
                goAttractX,
                goRepelX;
            using (var pkg = new PackageMedium())
            {
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Circle, radius = 50f },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Point,
                        colliderMask = 0ul,
                        forceMagnitude = -200f,
                        distanceScale = 1f,
                        forceMode = 0,
                    }
                );
                var attract = pkg.SpawnBody(new float2(5f, 0f), 0.5f, gravityScale: 0f);
                pkg.Create();
                for (var i = 0; i < steps; i++)
                    pkg.Step();
                pkgAttractX = pkg.Position(attract).x;
            }
            using (var pkg = new PackageMedium())
            {
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Circle, radius = 50f },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Point,
                        colliderMask = 0ul,
                        forceMagnitude = 200f,
                        distanceScale = 1f,
                        forceMode = 0,
                    }
                );
                var repel = pkg.SpawnBody(new float2(5f, 0f), 0.5f, gravityScale: 0f);
                pkg.Create();
                for (var i = 0; i < steps; i++)
                    pkg.Step();
                pkgRepelX = pkg.Position(repel).x;
            }

            using (var go = new GameObjectMedium(s_PackageGravity))
            {
                go.SpawnPointCircle(
                    new Vector2(0f, 0f),
                    50f,
                    p =>
                    {
                        p.forceMagnitude = -200f;
                        p.distanceScale = 1f;
                        p.forceMode = EffectorForceMode2D.Constant;
                    }
                );
                var attract = go.SpawnBody(new Vector2(5f, 0f), 0.5f, gravityScale: 0f);
                for (var i = 0; i < steps; i++)
                    go.Step();
                goAttractX = attract.position.x;
                yield return null;
            }
            using (var go = new GameObjectMedium(s_PackageGravity))
            {
                go.SpawnPointCircle(
                    new Vector2(0f, 0f),
                    50f,
                    p =>
                    {
                        p.forceMagnitude = 200f;
                        p.distanceScale = 1f;
                        p.forceMode = EffectorForceMode2D.Constant;
                    }
                );
                var repel = go.SpawnBody(new Vector2(5f, 0f), 0.5f, gravityScale: 0f);
                for (var i = 0; i < steps; i++)
                    go.Step();
                goRepelX = repel.position.x;
                yield return null;
            }

            Debug.Log(
                $"[P10A-GATE-POINT-SIGN] from x=5: attract x: pkg={pkgAttractX:F3} GO={goAttractX:F3} (toward 0) | "
                    + $"repel x: pkg={pkgRepelX:F3} GO={goRepelX:F3} (away from 0)"
            );

            // Attract: moved toward the origin (x decreased below 5) in both media.
            Assert.Less(pkgAttractX, 5f, $"Package attract did not move toward the point (x={pkgAttractX}).");
            Assert.Less(goAttractX, 5f, $"GameObject attract did not move toward the point (x={goAttractX}).");
            // Repel: moved away from the origin (x increased above 5) in both media.
            Assert.Greater(pkgRepelX, 5f, $"Package repel did not move away from the point (x={pkgRepelX}).");
            Assert.Greater(goRepelX, 5f, $"GameObject repel did not move away from the point (x={goRepelX}).");
        }

        // ====================================================================================================
        // BUOYANCY — equilibrium-depth measured in both media (the AABB-vs-true-area gap), drag damping.

        [UnityTest]
        public IEnumerator Buoyancy_CircleEquilibriumDepth_MeasuredBothMedia()
        {
            // The headline escalation: a unit-density circle dropped into a density-2 fluid sinks, decelerates,
            // and floats up to rest. Measure the EQUILIBRIUM rest depth (body-centre y relative to surface) in
            // BOTH media. The package uses an AABB submerged fraction (for a circle, AABB-height = diameter, so
            // f=0.5 at centre-at-surface → equilibrium centre AT the surface for density 2). The GameObject Box2D-v2
            // uses the true submerged circle-segment AREA. At density 2 the equilibrium submerged-AREA fraction is
            // 0.5; for a circle, area-fraction 0.5 is ALSO centre-at-surface (the circle is symmetric about its
            // centre), so the two AGREE at this density. Report the measured delta; GREEN within a generous band.
            const float surface = 0f;
            const float density = 2f;
            const int settleSteps = 600;

            float pkgRest,
                pkgMin;
            using (var pkg = new PackageMedium())
            {
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(20f, 20f) },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Buoyancy,
                        colliderMask = 0ul,
                        surfaceLevel = surface,
                        fluidDensity = density,
                        linearDamping = 5f,
                        angularDamping = 5f,
                        gravityMagnitude = s_PackageGravityMag,
                    }
                );
                var body = pkg.SpawnBody(new float2(0f, -1f), 0.5f, gravityScale: 1f);
                pkg.Create();
                pkgMin = 0f;
                for (var i = 0; i < settleSteps; i++)
                {
                    pkg.Step();
                    var y = pkg.Position(body).y;
                    if (y < pkgMin)
                        pkgMin = y;
                }
                pkgRest = pkg.Position(body).y;
            }

            float goRest,
                goMin;
            using (var go = new GameObjectMedium(s_PackageGravity))
            {
                go.SpawnBuoyancyBox(
                    new Vector2(0f, 0f),
                    new Vector2(20f, 20f),
                    b =>
                    {
                        b.surfaceLevel = surface;
                        b.density = density;
                        b.linearDamping = 5f;
                        b.angularDamping = 5f;
                        b.flowMagnitude = 0f;
                        b.flowVariation = 0f;
                    }
                );
                var rb = go.SpawnBody(new Vector2(0f, -1f), 0.5f, gravityScale: 1f);
                goMin = 0f;
                for (var i = 0; i < settleSteps; i++)
                {
                    go.Step();
                    var y = rb.position.y;
                    if (y < goMin)
                        goMin = y;
                }
                goRest = rb.position.y;
                yield return null;
            }

            var delta = abs(pkgRest - goRest);
            Debug.Log(
                $"[P10A-GATE-BUOYANCY-EQ] density={density} surface={surface}: pkg rest y={pkgRest:F4} "
                    + $"(minY={pkgMin:F4}) | GO rest y={goRest:F4} (minY={goMin:F4}) | |delta|={delta:F4} "
                    + "(AABB submerged-fraction vs true circle-segment area; both = centre-at-surface at density 2)"
            );

            // Neither sank out the bottom (well above y=-10).
            Assert.Greater(pkgMin, -5f, $"Package body sank out of the fluid (minY={pkgMin}).");
            Assert.Greater(goMin, -5f, $"GameObject body sank out of the fluid (minY={goMin}).");
            // Neither flew out the top.
            Assert.Less(pkgRest, 2f, $"Package body flew out of the fluid (rest y={pkgRest}).");
            Assert.Less(goRest, 2f, $"GameObject body flew out of the fluid (rest y={goRest}).");
            // Both rest near the surface (within the body radius + a band for cross-solver settle).
            Assert.Less(abs(pkgRest - surface), 0.6f, $"Package body did not rest near the surface (y={pkgRest}).");
            Assert.Less(abs(goRest - surface), 0.6f, $"GameObject body did not rest near the surface (y={goRest}).");
            // Cross-medium equilibrium delta within a generous band (the AABB-vs-true-area gap; at density 2 they
            // coincide at centre-at-surface, so the measured delta is the cross-solver settle noise, not the gap).
            Assert.Less(
                delta,
                0.5f,
                $"Buoyancy equilibrium depth diverged beyond the band: pkg={pkgRest}, GO={goRest} (|delta|={delta})."
            );
        }

        [UnityTest]
        public IEnumerator Buoyancy_DenseBody_RestsDeeper_AABBGapMeasured()
        {
            // The escalation that EXERCISES the AABB-vs-true-area gap: a body DENSER than half the fluid rests
            // BELOW centre-at-surface (equilibrium submerged fraction f = bodyDensity/fluidDensity > 0.5). For a
            // circle, the package's AABB-depth fraction and the GameObject's true-area fraction map a given f to
            // DIFFERENT centre depths (only at f=0.5 do they coincide). bodyDensity 1, fluidDensity 1.5 →
            // f_target = 0.667: package AABB rest depth = surface - (f-0.5)*diameter; GO true-area rest depth
            // differs (the area-vs-depth nonlinearity). Measure BOTH; document the delta. This is the prime
            // known-gap candidate — report the numbers, GREEN if within a generous band, RED-with-evidence if not.
            const float surface = 0f;
            const float fluidDensity = 1.5f;
            const int settleSteps = 800;

            float pkgRest,
                pkgMin;
            using (var pkg = new PackageMedium())
            {
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(20f, 20f) },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Buoyancy,
                        colliderMask = 0ul,
                        surfaceLevel = surface,
                        fluidDensity = fluidDensity,
                        linearDamping = 6f,
                        angularDamping = 6f,
                        gravityMagnitude = s_PackageGravityMag,
                    }
                );
                var body = pkg.SpawnBody(new float2(0f, -2f), 0.5f, gravityScale: 1f);
                pkg.Create();
                pkgMin = 0f;
                for (var i = 0; i < settleSteps; i++)
                {
                    pkg.Step();
                    var y = pkg.Position(body).y;
                    if (y < pkgMin)
                        pkgMin = y;
                }
                pkgRest = pkg.Position(body).y;
            }

            float goRest,
                goMin;
            using (var go = new GameObjectMedium(s_PackageGravity))
            {
                go.SpawnBuoyancyBox(
                    new Vector2(0f, 0f),
                    new Vector2(20f, 20f),
                    b =>
                    {
                        b.surfaceLevel = surface;
                        b.density = fluidDensity;
                        b.linearDamping = 6f;
                        b.angularDamping = 6f;
                        b.flowMagnitude = 0f;
                    }
                );
                var rb = go.SpawnBody(new Vector2(0f, -2f), 0.5f, gravityScale: 1f);
                goMin = 0f;
                for (var i = 0; i < settleSteps; i++)
                {
                    go.Step();
                    var y = rb.position.y;
                    if (y < goMin)
                        goMin = y;
                }
                goRest = rb.position.y;
                yield return null;
            }

            // Analytic AABB prediction: f = 1/1.5 = 0.667, package submerged depth = f*diameter = 0.667 → bottom
            // at surface-0.667, centre at surface - (0.667 - 0.5)*1.0 = -0.167.
            var aabbPredicted = surface - (1f / fluidDensity - 0.5f) * 1.0f;
            var delta = abs(pkgRest - goRest);
            Debug.Log(
                $"[P10A-GATE-BUOYANCY-DENSE] fluidDensity={fluidDensity} (target f≈{1f / fluidDensity:F3}): "
                    + $"pkg rest y={pkgRest:F4} (AABB-predicted {aabbPredicted:F4}, minY={pkgMin:F4}) | "
                    + $"GO rest y={goRest:F4} (true circle-area, minY={goMin:F4}) | |delta|={delta:F4} "
                    + "(AABB-depth-fraction vs true-area-fraction — the documented buoyancy approximation gap)"
            );

            // Both must rest BELOW the surface (denser than half the fluid → more than half submerged) but not
            // sink out: a solver-independent disqualifier proving the heavier body floats lower, in both media.
            Assert.Less(pkgRest, surface, $"Package dense body did not rest below the surface (y={pkgRest}).");
            Assert.Less(goRest, surface, $"GameObject dense body did not rest below the surface (y={goRest}).");
            Assert.Greater(pkgMin, -5f, $"Package dense body sank out (minY={pkgMin}).");
            Assert.Greater(goMin, -5f, $"GameObject dense body sank out (minY={goMin}).");
            // The package matches its OWN AABB model (proves the AABB formula, independent of the GO oracle).
            Assert.Less(
                abs(pkgRest - aabbPredicted),
                0.25f,
                $"Package rest depth {pkgRest} != its AABB-model prediction {aabbPredicted} — the AABB submerged "
                    + "fraction is not driving the equilibrium as designed."
            );
            // Cross-medium: the AABB approximation vs the true-area equilibrium. A generous 0.4m band: if it holds,
            // the AABB gap is immaterial at this density (GREEN, gap measured); if it failed, the delta IS the gap
            // (documented RED-with-evidence). The measured |delta| above is the load-bearing number either way.
            Assert.Less(
                delta,
                0.4f,
                $"Dense-body buoyancy equilibrium AABB-vs-true-area gap exceeds the band: pkg={pkgRest}, "
                    + $"GO={goRest} (|delta|={delta}). The AABB submerged-fraction approximation materially "
                    + "diverges from Box2D-v2 true-area buoyancy at this density — documented known-gap."
            );
        }

        [UnityTest]
        public IEnumerator Buoyancy_FluidDrag_DampsOscillation()
        {
            // The escalated fluid-drag pin: a body dropped from ABOVE the surface plunges in, the fluid drag
            // damps its oscillation, and it settles (does not keep bobbing forever). Compare the settle in both
            // media: the amplitude of the second up-swing must be much smaller than the first plunge depth — the
            // drag damped it. Pinned in both media (the GameObject fluid damping vs the package's f-scaled drag).
            const float surface = 0f;
            const int steps = 500;

            float pkgFirstPlunge,
                pkgLateAmplitude;
            using (var pkg = new PackageMedium())
            {
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(20f, 20f) },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Buoyancy,
                        colliderMask = 0ul,
                        surfaceLevel = surface,
                        fluidDensity = 2f,
                        linearDamping = 3f,
                        angularDamping = 3f,
                        gravityMagnitude = s_PackageGravityMag,
                    }
                );
                var body = pkg.SpawnBody(new float2(0f, 3f), 0.5f, gravityScale: 1f);
                pkg.Create();
                pkgFirstPlunge = MeasureSettle(
                    () =>
                    {
                        pkg.Step();
                        return pkg.Position(body).y;
                    },
                    steps,
                    surface,
                    out pkgLateAmplitude
                );
            }

            float goFirstPlunge,
                goLateAmplitude;
            using (var go = new GameObjectMedium(s_PackageGravity))
            {
                go.SpawnBuoyancyBox(
                    new Vector2(0f, 0f),
                    new Vector2(20f, 20f),
                    b =>
                    {
                        b.surfaceLevel = surface;
                        b.density = 2f;
                        b.linearDamping = 3f;
                        b.angularDamping = 3f;
                        b.flowMagnitude = 0f;
                    }
                );
                var rb = go.SpawnBody(new Vector2(0f, 3f), 0.5f, gravityScale: 1f);
                goFirstPlunge = MeasureSettle(
                    () =>
                    {
                        go.Step();
                        return rb.position.y;
                    },
                    steps,
                    surface,
                    out goLateAmplitude
                );
                yield return null;
            }

            Debug.Log(
                $"[P10A-GATE-BUOYANCY-DRAG] dropped from y=3: pkg firstPlunge={pkgFirstPlunge:F4} "
                    + $"lateAmplitude={pkgLateAmplitude:F4} | GO firstPlunge={goFirstPlunge:F4} "
                    + $"lateAmplitude={goLateAmplitude:F4} (drag damps the oscillation)"
            );

            // The body plunged below the surface on entry (a real entry, in both media).
            Assert.Greater(pkgFirstPlunge, 0.1f, $"Package body did not plunge in (firstPlunge={pkgFirstPlunge}).");
            Assert.Greater(goFirstPlunge, 0.1f, $"GameObject body did not plunge in (firstPlunge={goFirstPlunge}).");
            // The late oscillation amplitude is much smaller than the first plunge — the drag damped it (it did
            // not keep bobbing). A factor-of-3 reduction is a conservative damping witness in both media.
            Assert.Less(
                pkgLateAmplitude,
                pkgFirstPlunge / 3f,
                $"Package oscillation not damped: late amplitude {pkgLateAmplitude} vs first plunge {pkgFirstPlunge}."
            );
            Assert.Less(
                goLateAmplitude,
                goFirstPlunge / 3f,
                $"GameObject oscillation not damped: late amplitude {goLateAmplitude} vs first plunge {goFirstPlunge}."
            );
        }

        // Step a medium `steps` times, tracking the deepest plunge below the surface in the FIRST third and the
        // peak-to-peak oscillation amplitude in the LAST third (the residual bob after the drag has acted).
        static float MeasureSettle(System.Func<float> stepAndReadY, int steps, float surface, out float lateAmplitude)
        {
            var firstThird = steps / 3;
            var lastThird = steps - steps / 3;
            var firstPlunge = 0f;
            var lateMin = float.PositiveInfinity;
            var lateMax = float.NegativeInfinity;
            for (var i = 0; i < steps; i++)
            {
                var y = stepAndReadY();
                if (i < firstThird)
                {
                    var depth = surface - y;
                    if (depth > firstPlunge)
                        firstPlunge = depth;
                }
                if (i >= lastThird)
                {
                    if (y < lateMin)
                        lateMin = y;
                    if (y > lateMax)
                        lateMax = y;
                }
            }
            lateAmplitude = lateMax - lateMin;
            return firstPlunge;
        }

        // ====================================================================================================
        // COLLIDERMASK — binary exclusion: an off-mask body is UNAFFECTED, an on-mask body IS affected.

        [UnityTest]
        public IEnumerator ColliderMask_OffMaskBodyUnaffected_OnMaskBodyAffected_BothMedia()
        {
            // The escalated colliderMask pin. An Area effector masked to ONE layer must affect a body on that
            // layer and apply ZERO force to a body on a different layer. Asserted as an EXACT binary: the off-mask
            // body's velocity stays bit-zero (no force ever applied), the on-mask body accelerates. Pinned in both
            // media. The package mask is the widened (uint)LayerMask bit convention (1<<layer); the body's shape
            // categoryBits must carry that bit so the overlap query's hitCategories intersect.
            const int onLayer = 8; // a project user layer
            const int offLayer = 10; // a different layer NOT in the effector mask
            const int steps = 15;
            var onMaskBit = 1ul << onLayer;

            float2 pkgOnV,
                pkgOffV;
            using (var pkg = new PackageMedium())
            {
                pkg.SpawnEffector(
                    new float2(0f, 0f),
                    new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(40f, 40f) },
                    new PhysicsEffector2D
                    {
                        kind = PhysicsEffector2DKind.Area,
                        colliderMask = onMaskBit, // ONLY the on-layer
                        forceMagnitude = 80f,
                        forceAngleRadians = 0f,
                        useGlobalAngle = 1,
                        forceTargetIsRigidbody = 1,
                    }
                );
                // On-mask body: categoryBits has the on-layer bit, contacts everything.
                var onBody = pkg.SpawnBody(
                    new float2(-2f, 0f),
                    0.5f,
                    gravityScale: 0f,
                    categoryBits: onMaskBit,
                    contactBits: ~0ul
                );
                // Off-mask body: categoryBits has a DIFFERENT layer bit, so the masked query never returns it.
                var offBody = pkg.SpawnBody(
                    new float2(2f, 0f),
                    0.5f,
                    gravityScale: 0f,
                    categoryBits: 1ul << offLayer,
                    contactBits: ~0ul
                );
                pkg.Create();
                for (var i = 0; i < steps; i++)
                    pkg.Step();
                pkgOnV = pkg.Velocity(onBody);
                pkgOffV = pkg.Velocity(offBody);
            }

            float2 goOnV,
                goOffV;
            using (var go = new GameObjectMedium(s_PackageGravity))
            {
                go.SpawnAreaBox(
                    new Vector2(0f, 0f),
                    new Vector2(40f, 40f),
                    a =>
                    {
                        a.forceMagnitude = 80f;
                        a.forceAngle = 0f;
                        a.forceVariation = 0f;
                        a.useGlobalAngle = true;
                        a.forceTarget = EffectorSelection2D.Rigidbody;
                        a.useColliderMask = true;
                        a.colliderMask = 1 << onLayer; // ONLY the on-layer
                    }
                );
                var onBody = go.SpawnBody(new Vector2(-2f, 0f), 0.5f, gravityScale: 0f, layer: onLayer);
                var offBody = go.SpawnBody(new Vector2(2f, 0f), 0.5f, gravityScale: 0f, layer: offLayer);
                for (var i = 0; i < steps; i++)
                    go.Step();
                goOnV = onBody.linearVelocity;
                goOffV = offBody.linearVelocity;
                yield return null;
            }

            Debug.Log(
                $"[P10A-GATE-MASK] effector masked to layer {onLayer}: on-mask v: pkg={pkgOnV} GO={goOnV} | "
                    + $"off-mask v (MUST be ~zero): pkg={pkgOffV} GO={goOffV}"
            );

            // On-mask body accelerated in +X (the force reached it) in both media.
            Assert.Greater(pkgOnV.x, 1f, $"Package on-mask body was not affected (v={pkgOnV}).");
            Assert.Greater(goOnV.x, 1f, $"GameObject on-mask body was not affected (v={goOnV}).");
            // Off-mask body got EXACTLY zero force (the binary exclusion). The query never returned it, so no force
            // was ever applied — its velocity is bit-zero. A small epsilon allows for nothing but float noise.
            Assert.Less(length(pkgOffV), 1e-4f, $"Package off-mask body was affected despite the mask (v={pkgOffV}).");
            Assert.Less(
                length((float2)goOffV),
                1e-4f,
                $"GameObject off-mask body was affected despite the mask (v={goOffV})."
            );
        }
    }
}
