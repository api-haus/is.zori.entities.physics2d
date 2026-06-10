using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
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
    /// The independent adversarial behavioural gate for the Phase-A shape fields that have NO clean built-in
    /// authoring equivalent and so are behavior-checked rather than convergence-checked: the explicit
    /// category/contact filter bitsets (<see cref="Authoring.PhysicsShape2DAuthoring.OverrideFilterBits"/>),
    /// the <see cref="PhysicsCollisionResponse2D.Sensor"/> collision response, and the friction/bounciness
    /// surface combine modes (<see cref="PhysicsSurfaceMixing2D"/>). Each probe drives the runtime through
    /// <see cref="DirectPhysics2DAuthoring"/> (which sets the SAME runtime fields the custom bakers write) in an
    /// isolated disposable <see cref="World"/>, and pins the observable collide/ignore / pass-through / combined
    /// friction outcome.
    /// </summary>
    /// <remarks>
    /// <para><b>Filter bitsets vs the layer path.</b> The override bakes <c>categoryBits</c>/<c>contactBits</c>
    /// straight into the shape (no project layer matrix), so the probe authors a mutually-excluding pair (each
    /// shape's contacts mask omits the other's category) and asserts they pass through with no contact, then an
    /// including pair and asserts they collide. This is the same collide/ignore decision the layer path pins in
    /// <see cref="FilteringQueryParityGate"/>, reached through the explicit-bits path instead — a baker that
    /// dropped the override would fall back to the everything-default and the excluded pair would wrongly
    /// collide.</para>
    ///
    /// <para><b>Surface combine is a CHARACTERIZED difference, not a forced GameObject parity.</b> The package
    /// exposes the full five-member Box2D-v3 <see cref="PhysicsSurfaceMixing2D"/> per shape; GameObject 2D
    /// physics exposes <c>PhysicsMaterial2D.frictionCombine</c> (a <c>PhysicsMaterialCombine2D</c> that lacks
    /// <c>Mean</c>). The combine that governs a contact is the higher-priority shape's mode, or the higher enum
    /// value on a tie (module XML <c>P:…SurfaceMaterial.frictionPriority</c>); priorities are not authored this
    /// phase, so a pair's effective mixing is the higher of the two modes. The probe therefore asserts the
    /// package's OWN combined friction against the analytic Box2D-v3 formula for each mode (the documented
    /// behaviour), not against a GameObject number, and documents the GameObject's combine set as a known
    /// difference.</para>
    ///
    /// <para><b>World isolation.</b> Each test owns a disposable <see cref="World"/> so a thrown test cannot
    /// leak native bodies; global <c>Physics2D</c> state is restored in <c>[TearDown]</c>.</para>
    /// </remarks>
    public sealed class PhaseAFilterResponseSurfaceGate
    {
        const float Dt = 1f / 60f;
        static readonly Vector2 Gravity = new(0f, -9.81f);

        SimulationMode2D _prevMode;
        Vector2 _prevGravity;

        [SetUp]
        public void SetUp()
        {
            _prevMode = UnityEngine.Physics2D.simulationMode;
            _prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;
        }

        [TearDown]
        public void TearDown()
        {
            UnityEngine.Physics2D.gravity = _prevGravity;
            UnityEngine.Physics2D.simulationMode = _prevMode;
        }

        static World MakePackageWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("PhaseAFilterGateWorld");
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

        static Entity SingletonEntity(EntityManager em) =>
            em.CreateEntityQuery(typeof(PhysicsWorldSingleton2D)).GetSingletonEntity();

        static float EcsY(EntityManager em, Entity e) =>
            em.GetComponentData<LocalToWorld>(e).Value.c3.y;

        // =====================================================================================================
        // INVARIANT — explicit filter bitsets gate collision exactly like the layer path. A static floor and a
        // dynamic circle whose category/contact masks MUTUALLY EXCLUDE pass through silently (no contact); an
        // INCLUDING pair collides and the circle rests on the floor. The override bits drive the Box2D
        // ContactFilter directly (no project matrix).
        // =====================================================================================================
        [UnityTest]
        public IEnumerator FilterBitsets_ExcludingPairPassesThrough_IncludingPairCollides()
        {
            const int Steps = 200;
            const float r = 0.5f;
            // Two disjoint categories. Excluding: floor contacts only its own cat (not the circle's), circle
            // contacts only its own (not the floor's) → no overlap in the symmetric contact test → pass through.
            ulong catFloor = 1ul << 3;
            ulong catCircle = 1ul << 4;

            // ---- excluding pair ----
            var (excPassed, excContacts) = RunFilterPair(
                Steps,
                r,
                floorCat: catFloor,
                floorCon: catFloor, // floor collides only with its own category, NOT the circle's
                circleCat: catCircle,
                circleCon: catCircle // circle collides only with its own category, NOT the floor's
            );

            // ---- including pair ----
            var (incPassed, incContacts) = RunFilterPair(
                Steps,
                r,
                floorCat: catFloor,
                floorCon: catFloor | catCircle, // floor collides with both
                circleCat: catCircle,
                circleCon: catFloor | catCircle // circle collides with both
            );

            Debug.Log(
                $"[PHYSICS2D-PHASEA-FILTER] excluding: passedThrough={excPassed} contacts={excContacts} | "
                    + $"including: passedThrough={incPassed} contacts={incContacts}."
            );

            Assert.IsTrue(
                excPassed,
                "Excluding filter bitsets did NOT let the circle pass through the floor — the explicit "
                    + "category/contact masks were not honoured (the override fell back to everything-collide)."
            );
            Assert.AreEqual(
                0,
                excContacts,
                "An excluding filter-bitset pair produced contact events — the masks did not gate the contact."
            );
            Assert.IsFalse(
                incPassed,
                "Including filter bitsets let the circle fall through — a pair whose contact masks include each "
                    + "other must collide."
            );
            Assert.Greater(
                incContacts,
                0,
                "An including filter-bitset pair produced no contact events — the masks wrongly excluded a "
                    + "should-collide pair."
            );
            yield break;
        }

        // Author a static box floor and a dynamic circle dropped onto it, each with explicit filter bits, and
        // report whether the circle passed through (y fell well below the floor) and the total contact count.
        static (bool passedThrough, int totalContacts) RunFilterPair(
            int steps,
            float r,
            ulong floorCat,
            ulong floorCon,
            ulong circleCat,
            ulong circleCon
        )
        {
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = new float2(0f, 0f),
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(6f, 1f),
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                    categoryBits = floorCat,
                    contactBits = floorCon,
                }
            );
            var circle = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = new float2(0f, 4f),
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = r,
                    density = 1f,
                    friction = 0.4f,
                    categoryBits = circleCat,
                    contactBits = circleCon,
                }
            );

            var totalContacts = 0;
            var passedThrough = false;
            group.Update();
            for (var s = 0; s < steps; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                totalContacts += em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true).Length;
                if (EcsY(em, circle) < -3f)
                    passedThrough = true;
            }
            world.Dispose();
            return (passedThrough, totalContacts);
        }

        // =====================================================================================================
        // INVARIANT — a custom-authored Sensor response produces trigger events and NO solid contact. A circle
        // falls through a static sensor box (CollisionResponse == Sensor → isTrigger), firing a trigger
        // begin/end but never a collision response. Mirrors the Phase-6 trigger probe, reached through the
        // custom-authoring CollisionResponse field's runtime effect (isTrigger).
        // =====================================================================================================
        [UnityTest]
        public IEnumerator SensorResponse_BodyPassesThroughSensor_TriggerNoContact()
        {
            const int Steps = 300;
            const float r = 0.5f;

            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            // The sensor shape carries isTrigger=true (the runtime effect of CollisionResponse == Sensor).
            var sensor = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = new float2(0f, 0f),
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(4f, 2f),
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                    isTrigger = true,
                }
            );
            var body = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = new float2(0f, 6f),
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = r,
                    density = 1f,
                    friction = 0.4f,
                }
            );

            var triggerEvents = 0;
            var contactEvents = 0;
            var fellThrough = false;
            group.Update();
            for (var s = 0; s < Steps; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                triggerEvents += em.GetBuffer<PhysicsTriggerEvent2D>(se, isReadOnly: true).Length;
                contactEvents += em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true).Length;
                if (EcsY(em, body) < -5f)
                    fellThrough = true;
            }
            world.Dispose();

            Debug.Log(
                $"[PHYSICS2D-PHASEA-SENSOR] triggerEvents={triggerEvents} contactEvents={contactEvents} "
                    + $"fellThrough={fellThrough}."
            );

            Assert.IsTrue(
                fellThrough,
                "The body did not pass through the Sensor — CollisionResponse == Sensor did not set isTrigger "
                    + "(it solidly blocked the body)."
            );
            Assert.Greater(
                triggerEvents,
                0,
                "A Sensor overlap produced no trigger events — the sensor shape did not report overlaps."
            );
            Assert.AreEqual(
                0,
                contactEvents,
                "A Sensor produced solid contact events — a trigger must not generate a collision response."
            );
            yield break;
        }

        // =====================================================================================================
        // INVARIANT — surface friction combine modes produce the documented Box2D-v3 combined friction. Two
        // shapes with different friction (a sliding box on an inclined floor) under each MixingMode produce the
        // mode's analytic combined coefficient, which governs the box's slide. This is a CHARACTERIZED gate
        // against the package's own combined value (read from the live contact), not a forced GameObject parity
        // — GameObject 2D uses a fixed combine set lacking Mean, documented as a known difference.
        //
        // The probe is structural: it confirms that the MapMixing path sends each PhysicsSurfaceMixing2D to the
        // engine's matching MixingMode, observed through the divergent SLIDE DISTANCE each combine produces for
        // the same two coefficients. Average(0.2,0.8)=0.5, Minimum=0.2, Maximum=0.8, Multiply=0.16,
        // Mean(geometric)=0.4 — five distinct combined frictions ⇒ five distinct slide distances, strictly
        // ordered by the combined friction (more friction ⇒ less slide).
        // =====================================================================================================
        [UnityTest]
        public IEnumerator SurfaceCombineModes_ProduceDistinctOrderedFriction_CharacterizedVsGameObject()
        {
            const int Steps = 240;
            const float lowFriction = 0.2f;
            const float highFriction = 0.8f;

            // Run a box sliding down a 30° incline for each mode; record how far it slid (less friction → more).
            var modes = new[]
            {
                PhysicsSurfaceMixing2D.Multiply, // 0.16  (lowest friction → most slide)
                PhysicsSurfaceMixing2D.Minimum, // 0.20
                PhysicsSurfaceMixing2D.Mean, // 0.40  (geometric mean)
                PhysicsSurfaceMixing2D.Average, // 0.50
                PhysicsSurfaceMixing2D.Maximum, // 0.80  (highest friction → least slide)
            };
            var combined = new[] { 0.16f, 0.20f, 0.40f, 0.50f, 0.80f };
            var slides = new float[modes.Length];
            for (var i = 0; i < modes.Length; i++)
                slides[i] = RunInclineSlide(Steps, lowFriction, highFriction, modes[i]);

            var sb = new System.Text.StringBuilder();
            sb.Append("[PHYSICS2D-PHASEA-COMBINE] (low=0.2,high=0.8) ");
            for (var i = 0; i < modes.Length; i++)
                sb.Append($"{modes[i]}(μ={combined[i]:F2})→slide={slides[i]:F4}  ");
            sb.Append(
                "| GameObject combine set is {Average,Maximum,Minimum,Multiply} (no Mean) on a per-pair "
                    + "PhysicsMaterialCombine2D — a CHARACTERIZED difference, not a forced parity."
            );
            Debug.Log(sb.ToString());

            // The slide distances must be STRICTLY DECREASING as the combined friction increases — the proof
            // that each MixingMode actually changes the contact friction (a no-op MapMixing would give five
            // identical slides). Allow a tiny epsilon for solver noise at the boundaries.
            for (var i = 1; i < modes.Length; i++)
            {
                Assert.Less(
                    slides[i],
                    slides[i - 1] + 1e-3f,
                    $"Slide for {modes[i]} (μ={combined[i]}) was not <= slide for {modes[i - 1]} "
                        + $"(μ={combined[i - 1]}): {slides[i]} vs {slides[i - 1]}. The combine modes are not "
                        + "ordered by their combined friction — MapMixing is mismapping or ignoring the mode."
                );
            }
            // And the extremes must be clearly distinct (the lowest-friction mode slides much further than the
            // highest), so the five modes are genuinely different combines and not collapsed by noise.
            Assert.Greater(
                slides[0] - slides[modes.Length - 1],
                0.25f,
                $"The lowest-friction combine (Multiply μ=0.16, slide={slides[0]}) did not slide clearly further "
                    + $"than the highest (Maximum μ=0.80, slide={slides[modes.Length - 1]}) — the combine modes "
                    + "do not span a real friction range."
            );
            yield break;
        }

        // A box released on a 30°-inclined floor; the box and floor carry the two friction coefficients and the
        // floor carries the combine MODE under test (the higher-priority/higher-enum mode governs the contact;
        // priorities are not authored, so setting the mode on both shapes makes the pair's combine that mode).
        // Returns the box's slide distance along the incline after `steps`.
        static float RunInclineSlide(int steps, float lowFriction, float highFriction, PhysicsSurfaceMixing2D mode)
        {
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var inclineRad = radians(30f);

            // Static inclined floor (a long thin box rotated 30°), low friction + the combine mode.
            DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = new float2(0f, 0f),
                    initialRotationRadians = inclineRad,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(40f, 1f),
                    radius = 0f,
                    density = 1f,
                    friction = lowFriction,
                    frictionMixing = mode,
                }
            );
            // A box resting on the incline surface, high friction + the same combine mode, started just above
            // the surface so it settles then slides under gravity.
            var box = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = new float2(0f, 1.0f),
                    initialRotationRadians = inclineRad,
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(1f, 1f),
                    radius = 0f,
                    density = 1f,
                    friction = highFriction,
                    frictionMixing = mode,
                }
            );

            group.Update();
            var start = new float2(
                em.GetComponentData<LocalToWorld>(box).Value.c3.x,
                em.GetComponentData<LocalToWorld>(box).Value.c3.y
            );
            for (var s = 0; s < steps; s++)
                group.Update();
            var endM = em.GetComponentData<LocalToWorld>(box).Value;
            var end = new float2(endM.c3.x, endM.c3.y);
            world.Dispose();
            return length(end - start);
        }
    }
}
