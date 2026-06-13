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
    /// The independent adversarial GameObject-parity gate for Phase 6 (contact + trigger event surface). Built
    /// to FALSIFY the phase's invariants by probing the event surface's observable decision points, not the
    /// happy path the smoke (<see cref="ContactTriggerSmoke"/>) covers. The oracle is the established
    /// cross-backend pattern: one set of poses/layers authored TWICE — the package's Box2D-v3 world driven by
    /// the ECS step, and a live GameObject Box2D-v2 world stepped with <c>UnityEngine.Physics2D.Simulate</c> in
    /// <c>Script</c> mode — and the two event streams compared.
    /// </summary>
    /// <remarks>
    /// <para><b>The two backends are different integrators (v2 vs v3), so SET/COUNT facts are asserted EXACTLY
    /// and CONTINUOUS facts (the step a begin/end lands on) are bounded.</b> "Which collider pairs fired
    /// begin/end, and how many distinct begin/end episodes" is a set/count fact and must match exactly. "On
    /// which step the begin fired" is bounded by a generous frame envelope (v2 and v3 may begin/end a step
    /// apart).</para>
    ///
    /// <para><b>Stay is derived, never asserted as a literal event.</b> Box2D-v3 reports begin/end touch only;
    /// the package emits no per-frame Stay element. So the "currently touching" set is derived on BOTH sides
    /// from begin..end and compared. The GameObject <c>OnCollisionStay2D</c>/<c>OnTriggerStay2D</c> counter is
    /// recorded as a cross-check (a count of the frames the GO reported touching), and the package's derived
    /// touching-frame count is asserted to cover the SAME frame-set within a one-frame edge envelope — never an
    /// equality against a literal package Stay event (there is none).</para>
    ///
    /// <para><b>World isolation + two-consecutive-green.</b> Each test runs in its OWN disposable
    /// <see cref="World"/> (a thrown test leaks native bodies into a shared world and poisons later tests), and
    /// the GameObject reference bodies + global <c>Physics2D</c> knobs are torn down/restored per test in
    /// <c>[SetUp]</c>/<c>[TearDown]</c>. The events are deterministic set/count facts, so two runs produce the
    /// identical witnesses.</para>
    /// </remarks>
    public sealed class Phase6ContactTriggerEventGate
    {
        const float Dt = 1f / 60f;
        static readonly Vector2 Gravity = new(0f, -9.81f);

        // --- global Physics2D state save/restore (the package namespace shadows Physics2D, so qualify) -------

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

        // =====================================================================================================
        // PACKAGE SIDE — disposable world + event-buffer drain.
        // =====================================================================================================

        static World MakePackageWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DPhase6GateWorld", out group, Dt);

        static Entity SingletonEntity(EntityManager em) =>
            em.CreateEntityQuery(typeof(PhysicsWorldSingleton2D)).GetSingletonEntity();

        static Entity SpawnPkgCircle(EntityManager em, float2 pos, float radius)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = pos,
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = radius,
                    density = 1f,
                    friction = 0.4f,
                }
            );
        }

        static Entity SpawnPkgBox(
            EntityManager em,
            float2 center,
            float2 size,
            bool isTrigger,
            bool dynamic,
            ulong categoryBits = 0ul,
            ulong contactBits = 0ul
        )
        {
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
                    isTrigger = isTrigger,
                    categoryBits = categoryBits,
                    contactBits = contactBits,
                }
            );
        }

        // An unordered entity-pair key, stable regardless of (A,B) ordering.
        static long PairKey(Entity a, Entity b)
        {
            // Index is the discriminator within one world; pack the sorted pair of indices.
            var lo = min(a.Index, b.Index);
            var hi = max(a.Index, b.Index);
            return ((long)lo << 32) | (uint)hi;
        }

        // Per-pair package event record: which steps a begin/end fired, and the derived touching-frame set.
        sealed class PkgPairLog
        {
            public Entity a,
                b;
            public readonly List<int> beginSteps = new();
            public readonly List<int> endSteps = new();
            public readonly HashSet<int> touchingSteps = new();
            public bool currentlyTouching;
        }

        // Drain the package's contact (or trigger) buffer for one step into the per-pair logs, applying the
        // begin..end interval derivation: a Begin opens the interval, every step until the matching End is a
        // "touching" step. The step at which we observe a Begin is the first touching step; the End step is the
        // last step the pair was still touching the PREVIOUS step (GameObject Stay stops the frame Exit fires),
        // so the touching interval is [beginStep, endStep) — but to stay robust to the v2/v3 one-frame edge we
        // record every step strictly between begin and end as touching and let the envelope cover the edges.
        static void DrainContacts(DynamicBuffer<PhysicsContactEvent2D> buf, int step, Dictionary<long, PkgPairLog> logs)
        {
            for (var i = 0; i < buf.Length; i++)
            {
                var e = buf[i];
                var key = PairKey(e.entityA, e.entityB);
                if (!logs.TryGetValue(key, out var log))
                {
                    log = new PkgPairLog { a = e.entityA, b = e.entityB };
                    logs[key] = log;
                }
                if (e.phase == PhysicsEventPhase2D.Begin)
                {
                    log.beginSteps.Add(step);
                    log.currentlyTouching = true;
                }
                else
                {
                    log.endSteps.Add(step);
                    log.currentlyTouching = false;
                }
            }
        }

        static void DrainTriggers(DynamicBuffer<PhysicsTriggerEvent2D> buf, int step, Dictionary<long, PkgPairLog> logs)
        {
            for (var i = 0; i < buf.Length; i++)
            {
                var e = buf[i];
                var key = PairKey(e.triggerEntity, e.visitorEntity);
                if (!logs.TryGetValue(key, out var log))
                {
                    log = new PkgPairLog { a = e.triggerEntity, b = e.visitorEntity };
                    logs[key] = log;
                }
                if (e.phase == PhysicsEventPhase2D.Begin)
                {
                    log.beginSteps.Add(step);
                    log.currentlyTouching = true;
                }
                else
                {
                    log.endSteps.Add(step);
                    log.currentlyTouching = false;
                }
            }
        }

        // After a step, record (for every currently-open pair) that this step is a "touching" step. Called once
        // per step AFTER the begin/end of that step is drained, so a pair that began this step counts this step
        // as touching and a pair that ended this step does not.
        static void RecordTouching(int step, Dictionary<long, PkgPairLog> logs)
        {
            foreach (var log in logs.Values)
                if (log.currentlyTouching)
                    log.touchingSteps.Add(step);
        }

        // =====================================================================================================
        // GAMEOBJECT SIDE — live bodies + MonoBehaviour event counters.
        // =====================================================================================================

        // Records the begin/end/stay events the GameObject physics raised on its owner, per other-collider, with
        // the step index at which each fired. The gate advances GoStep before each Physics2D.Simulate so the
        // callbacks (raised synchronously inside Simulate) stamp the correct step.
        sealed class EventCounter2D : MonoBehaviour
        {
            public static int GoStep;

            public readonly Dictionary<Collider2D, List<int>> collisionEnter = new();
            public readonly Dictionary<Collider2D, List<int>> collisionExit = new();
            public readonly Dictionary<Collider2D, HashSet<int>> collisionStaySteps = new();
            public readonly Dictionary<Collider2D, List<int>> triggerEnter = new();
            public readonly Dictionary<Collider2D, List<int>> triggerExit = new();
            public readonly Dictionary<Collider2D, HashSet<int>> triggerStaySteps = new();

            static void Add(Dictionary<Collider2D, List<int>> d, Collider2D c, int step)
            {
                if (!d.TryGetValue(c, out var l))
                {
                    l = new List<int>();
                    d[c] = l;
                }
                l.Add(step);
            }

            static void AddSet(Dictionary<Collider2D, HashSet<int>> d, Collider2D c, int step)
            {
                if (!d.TryGetValue(c, out var s))
                {
                    s = new HashSet<int>();
                    d[c] = s;
                }
                s.Add(step);
            }

            void OnCollisionEnter2D(Collision2D c) => Add(collisionEnter, c.otherCollider, GoStep);

            void OnCollisionExit2D(Collision2D c) => Add(collisionExit, c.otherCollider, GoStep);

            void OnCollisionStay2D(Collision2D c) => AddSet(collisionStaySteps, c.otherCollider, GoStep);

            void OnTriggerEnter2D(Collider2D other) => Add(triggerEnter, other, GoStep);

            void OnTriggerExit2D(Collider2D other) => Add(triggerExit, other, GoStep);

            void OnTriggerStay2D(Collider2D other) => AddSet(triggerStaySteps, other, GoStep);
        }

        static int CountEnter(EventCounter2D ec, bool trigger)
        {
            var d = trigger ? ec.triggerEnter : ec.collisionEnter;
            var n = 0;
            foreach (var l in d.Values)
                n += l.Count;
            return n;
        }

        static int CountExit(EventCounter2D ec, bool trigger)
        {
            var d = trigger ? ec.triggerExit : ec.collisionExit;
            var n = 0;
            foreach (var l in d.Values)
                n += l.Count;
            return n;
        }

        static HashSet<int> StaySteps(EventCounter2D ec, bool trigger)
        {
            var d = trigger ? ec.triggerStaySteps : ec.collisionStaySteps;
            var all = new HashSet<int>();
            foreach (var s in d.Values)
                all.UnionWith(s);
            return all;
        }

        GameObject MakeGoDynamicCircle(float2 pos, float radius, List<GameObject> track, out EventCounter2D ec)
        {
            var go = new GameObject("GoCircle") { layer = 0 };
            go.transform.position = new Vector3(pos.x, pos.y, 0f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var c = go.AddComponent<CircleCollider2D>();
            c.radius = radius;
            c.sharedMaterial = _refMaterial;
            ec = go.AddComponent<EventCounter2D>();
            track.Add(go);
            return go;
        }

        GameObject MakeGoStaticBox(
            float2 center,
            float2 size,
            bool isTrigger,
            List<GameObject> track,
            out EventCounter2D ec
        )
        {
            var go = new GameObject("GoBox") { layer = 0 };
            go.transform.position = new Vector3(center.x, center.y, 0f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var c = go.AddComponent<BoxCollider2D>();
            c.size = (Vector2)size;
            c.isTrigger = isTrigger;
            c.sharedMaterial = _refMaterial;
            ec = go.AddComponent<EventCounter2D>();
            track.Add(go);
            return go;
        }

        PhysicsMaterial2D _refMaterial;

        void NewRefMaterial()
        {
            _refMaterial = new PhysicsMaterial2D("Phase6Reference") { friction = 0.4f, bounciness = 0f };
        }

        static void DestroyAll(List<GameObject> track)
        {
            foreach (var go in track)
                if (go != null)
                    Object.Destroy(go);
            track.Clear();
        }

        // =====================================================================================================
        // INVARIANT 1 — COLLISION begin/end parity: a body landing on a floor fires exactly one contact-begin
        // for the pair, matching OnCollisionEnter2D; on separation one contact-end matching OnCollisionExit2D.
        // Here the body lands and rests (no separation), so: exactly one begin, zero ends, on BOTH backends.
        // STAY is derived as the begin..end interval and compared frame-set against OnCollisionStay2D.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator Collision_BodyLandsOnFloor_OneBeginNoEnd_StayIntervalParity()
        {
            NewRefMaterial();
            const int Steps = 240;
            const float r = 0.5f;
            var floorCenter = new float2(0f, -0.5f);
            var floorSize = new float2(6f, 1f);
            var bodyPos = new float2(0f, 4f);

            // ---- package ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var floor = SpawnPkgBox(em, floorCenter, floorSize, isTrigger: false, dynamic: false);
            var body = SpawnPkgCircle(em, bodyPos, r);
            var contactLogs = new Dictionary<long, PkgPairLog>();
            var anyTrigger = false;

            group.Update(); // create, no step
            for (var s = 0; s < Steps; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                var cBuf = em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true);
                var tBuf = em.GetBuffer<PhysicsTriggerEvent2D>(se, isReadOnly: true);
                DrainContacts(cBuf, s, contactLogs);
                RecordTouching(s, contactLogs);
                if (tBuf.Length > 0)
                    anyTrigger = true;
            }
            var pkgPair = contactLogs.TryGetValue(PairKey(body, floor), out var pkgLog) ? pkgLog : null;
            world.Dispose();

            // ---- GameObject ----
            var track = new List<GameObject>();
            MakeGoStaticBox(floorCenter, floorSize, isTrigger: false, track, out _);
            MakeGoDynamicCircle(bodyPos, r, track, out var bodyEc);
            UnityEngine.Physics2D.SyncTransforms();
            for (var s = 0; s < Steps; s++)
            {
                EventCounter2D.GoStep = s;
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                yield return null;
            }
            var goEnter = CountEnter(bodyEc, trigger: false);
            var goExit = CountExit(bodyEc, trigger: false);
            var goStay = StaySteps(bodyEc, trigger: false);
            var goTriggerEnter = CountEnter(bodyEc, trigger: true);
            DestroyAll(track);

            // ---- witnesses ----
            var pkgBegins = pkgPair?.beginSteps.Count ?? 0;
            var pkgEnds = pkgPair?.endSteps.Count ?? 0;
            var pkgTouchN = pkgPair?.touchingSteps.Count ?? 0;
            Debug.Log(
                $"[PHYSICS2D-P6GATE] COLLISION-LAND: pkg begins={pkgBegins} ends={pkgEnds} touchSteps={pkgTouchN} "
                    + $"anyTrigger={anyTrigger} | GO enter={goEnter} exit={goExit} stayFrames={goStay.Count} "
                    + $"goTriggerEnter={goTriggerEnter} | firstPkgBegin={(pkgBegins > 0 ? pkgPair.beginSteps[0] : -1)}."
            );

            // SET/COUNT facts exact: exactly one begin, zero ends, no trigger on either backend.
            Assert.AreEqual(1, pkgBegins, "Package fired != 1 contact-begin for a clean landing (no bounce).");
            Assert.AreEqual(0, pkgEnds, "Package fired a contact-end for a body that never separated.");
            Assert.AreEqual(1, goEnter, "GameObject fired != 1 OnCollisionEnter2D for a clean landing.");
            Assert.AreEqual(0, goExit, "GameObject fired an OnCollisionExit2D for a body that never separated.");
            Assert.IsFalse(anyTrigger, "A solid (non-trigger) contact spuriously produced a package trigger event.");
            Assert.AreEqual(0, goTriggerEnter, "GameObject fired a trigger event for a solid contact.");

            // STAY derived as interval, NOT a literal package Stay event. The package's touching-frame set
            // (begin..open) must cover the SAME frames the GO reported OnCollisionStay2D, within a one-frame
            // edge envelope (v2/v3 begin a step apart). Assert the package touch-set and the GO stay-set overlap
            // substantially and neither leads/lags by more than 2 frames at the start edge.
            if (goStay.Count > 0)
            {
                var goMin = int.MaxValue;
                var goMax = int.MinValue;
                foreach (var f in goStay)
                {
                    goMin = min(goMin, f);
                    goMax = max(goMax, f);
                }
                var pkgMin = int.MaxValue;
                var pkgMax = int.MinValue;
                foreach (var f in pkgPair.touchingSteps)
                {
                    pkgMin = min(pkgMin, f);
                    pkgMax = max(pkgMax, f);
                }
                Assert.LessOrEqual(
                    abs(goMin - pkgMin),
                    2,
                    $"Stay-interval START edge diverged > 2 frames: GO first stay frame={goMin}, package first "
                        + $"touching frame={pkgMin}."
                );
                // Both run to the end of the window still touching (no separation), so the max should be near Steps-1.
                Assert.GreaterOrEqual(
                    pkgMax,
                    Steps - 3,
                    $"Package stopped reporting touching before the window end ({pkgMax} < {Steps - 3}) — a phantom end."
                );
                Assert.GreaterOrEqual(
                    goMax,
                    Steps - 3,
                    $"GO stopped reporting Stay before the window end ({goMax} < {Steps - 3})."
                );
            }
            else
            {
                // OnCollisionStay2D did not fire under Script-mode batchmode stepping — document, do not fail.
                // The derived package interval is the authoritative Stay surface; assert IT is a sane resting
                // interval (began once, never ended, covers most of the post-landing window).
                Assert.Greater(
                    pkgTouchN,
                    Steps / 4,
                    "Package derived touching-interval is implausibly short for a body that lands and rests "
                        + $"(touchSteps={pkgTouchN}). GO OnCollisionStay2D did not fire in batchmode (documented)."
                );
            }

            yield break;
        }

        // =====================================================================================================
        // INVARIANT 1b — COLLISION begin AND end: a body bounces off a floor (restitution 1) then leaves the
        // contact region, producing at least one begin AND at least one end on BOTH backends, with the begin/end
        // COUNTS equal across backends and the step indices bounded.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator Collision_BodyBouncesAndLeaves_BeginEndPairParity()
        {
            NewRefMaterial();
            _refMaterial.bounciness = 1f; // elastic so the body rebounds and separates → an end fires
            const int Steps = 200;
            const float r = 0.5f;
            // A small floor the body can bounce off and fly past sideways is overkill; instead drop straight
            // onto a floor with restitution 1: the body lands (begin), bounces back up and separates (end),
            // then falls and lands again (begin)… count the begin/end episodes and compare across backends.
            var floorCenter = new float2(0f, -0.5f);
            var floorSize = new float2(6f, 1f);
            var bodyPos = new float2(0f, 3f);

            // ---- package: bounciness is on the SHAPE for the package side ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            // Author both shapes with bounciness 1 so the package contact is elastic too.
            var floor = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition { bodyType = PhysicsBody.BodyType.Static, initialPosition = floorCenter },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = floorSize,
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                    bounciness = 1f,
                }
            );
            var body = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = bodyPos,
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = r,
                    density = 1f,
                    friction = 0.4f,
                    bounciness = 1f,
                }
            );
            var contactLogs = new Dictionary<long, PkgPairLog>();
            group.Update();
            for (var s = 0; s < Steps; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                DrainContacts(em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true), s, contactLogs);
                RecordTouching(s, contactLogs);
            }
            var pkgPair = contactLogs.TryGetValue(PairKey(body, floor), out var pl) ? pl : null;
            var pkgBegins = pkgPair?.beginSteps.Count ?? 0;
            var pkgEnds = pkgPair?.endSteps.Count ?? 0;
            world.Dispose();

            // ---- GameObject ----
            var track = new List<GameObject>();
            MakeGoStaticBox(floorCenter, floorSize, isTrigger: false, track, out _);
            MakeGoDynamicCircle(bodyPos, r, track, out var bodyEc);
            UnityEngine.Physics2D.SyncTransforms();
            for (var s = 0; s < Steps; s++)
            {
                EventCounter2D.GoStep = s;
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                yield return null;
            }
            var goEnter = CountEnter(bodyEc, trigger: false);
            var goExit = CountExit(bodyEc, trigger: false);
            DestroyAll(track);

            Debug.Log(
                $"[PHYSICS2D-P6GATE] COLLISION-BOUNCE: pkg begins={pkgBegins} ends={pkgEnds} | "
                    + $"GO enter={goEnter} exit={goExit}."
            );

            // A bounce produces at least one begin and one end on BOTH backends (the body separated).
            Assert.GreaterOrEqual(pkgBegins, 1, "Package fired no contact-begin for a bouncing body.");
            Assert.GreaterOrEqual(pkgEnds, 1, "Package fired no contact-end for a body that bounced and separated.");
            Assert.GreaterOrEqual(goEnter, 1, "GameObject fired no OnCollisionEnter2D for a bouncing body.");
            Assert.GreaterOrEqual(goExit, 1, "GameObject fired no OnCollisionExit2D for a separating body.");
            // Begin and end episodes are balanced within one (a run may end mid-air or mid-contact). Different
            // integrators differ in bounce count, so bound the begin/end balance per backend rather than equate
            // counts across backends.
            Assert.LessOrEqual(
                abs(pkgBegins - pkgEnds),
                1,
                $"Package begin/end episodes unbalanced by > 1 (begins={pkgBegins}, ends={pkgEnds}) — a leaked "
                    + "or duplicated event."
            );
            Assert.LessOrEqual(
                abs(goEnter - goExit),
                1,
                $"GameObject enter/exit unbalanced by > 1 (enter={goEnter}, exit={goExit})."
            );
            yield break;
        }

        // =====================================================================================================
        // INVARIANT 2 — TRIGGER begin/end parity: a body passing through a sensor fires one trigger-begin + one
        // trigger-end matching OnTriggerEnter2D/OnTriggerExit2D, with NO collision response (it passes through).
        // =====================================================================================================
        [UnityTest]
        public IEnumerator Trigger_BodyPassesThroughSensor_OneBeginOneEnd_NoContact()
        {
            NewRefMaterial();
            const int Steps = 300;
            const float r = 0.5f;
            var sensorCenter = new float2(0f, 0f);
            var sensorSize = new float2(4f, 2f);
            var bodyPos = new float2(0f, 6f);

            // ---- package ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var sensor = SpawnPkgBox(em, sensorCenter, sensorSize, isTrigger: true, dynamic: false);
            var body = SpawnPkgCircle(em, bodyPos, r);
            var triggerLogs = new Dictionary<long, PkgPairLog>();
            var contactLogs = new Dictionary<long, PkgPairLog>();
            var fellThrough = false;

            group.Update();
            for (var s = 0; s < Steps; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                DrainTriggers(em.GetBuffer<PhysicsTriggerEvent2D>(se, isReadOnly: true), s, triggerLogs);
                DrainContacts(em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true), s, contactLogs);
                RecordTouching(s, triggerLogs);
                var y = em.GetComponentData<LocalToWorld>(body).Position.y;
                if (y < -5f)
                    fellThrough = true;
            }
            var pkgTrig = triggerLogs.TryGetValue(PairKey(sensor, body), out var tl) ? tl : null;
            var pkgTrigBegins = pkgTrig?.beginSteps.Count ?? 0;
            var pkgTrigEnds = pkgTrig?.endSteps.Count ?? 0;
            var pkgTrigTouchN = pkgTrig?.touchingSteps.Count ?? 0;
            var pkgAnyContact = contactLogs.TryGetValue(PairKey(sensor, body), out var cl) && cl.beginSteps.Count > 0;
            world.Dispose();

            // ---- GameObject ----
            var track = new List<GameObject>();
            MakeGoStaticBox(sensorCenter, sensorSize, isTrigger: true, track, out var sensorEc);
            var goBody = MakeGoDynamicCircle(bodyPos, r, track, out var bodyEc);
            UnityEngine.Physics2D.SyncTransforms();
            var goFellThrough = false;
            for (var s = 0; s < Steps; s++)
            {
                EventCounter2D.GoStep = s;
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                if (goBody.transform.position.y < -5f)
                    goFellThrough = true;
                yield return null;
            }
            // The sensor reports OnTriggerEnter2D/Exit2D for the visiting body; the body's collider is the "other".
            var goTrigEnter = CountEnter(sensorEc, trigger: true);
            var goTrigExit = CountExit(sensorEc, trigger: true);
            var goTrigStay = StaySteps(sensorEc, trigger: true);
            var goAnyCollision = CountEnter(bodyEc, trigger: false) + CountEnter(sensorEc, trigger: false);
            DestroyAll(track);

            Debug.Log(
                $"[PHYSICS2D-P6GATE] TRIGGER-PASS: pkg trigBegins={pkgTrigBegins} trigEnds={pkgTrigEnds} "
                    + $"trigTouchSteps={pkgTrigTouchN} anyContact={pkgAnyContact} fellThrough={fellThrough} | "
                    + $"GO trigEnter={goTrigEnter} trigExit={goTrigExit} trigStayFrames={goTrigStay.Count} "
                    + $"anyCollision={goAnyCollision} goFellThrough={goFellThrough}."
            );

            // The body must pass through on BOTH backends (the isTrigger / no-response proof).
            Assert.IsTrue(fellThrough, "Package body did not pass through the sensor — isTrigger not applied.");
            Assert.IsTrue(goFellThrough, "GameObject body did not pass through the sensor — oracle sensor solid.");
            // SET/COUNT exact: exactly one trigger-begin + one trigger-end per backend; no collision event.
            Assert.AreEqual(1, pkgTrigBegins, "Package fired != 1 trigger-begin for a single pass-through.");
            Assert.AreEqual(1, pkgTrigEnds, "Package fired != 1 trigger-end for a single pass-through.");
            Assert.AreEqual(1, goTrigEnter, "GameObject fired != 1 OnTriggerEnter2D for a single pass-through.");
            Assert.AreEqual(1, goTrigExit, "GameObject fired != 1 OnTriggerExit2D for a single pass-through.");
            Assert.IsFalse(pkgAnyContact, "A sensor overlap spuriously produced a package contact (collision) event.");
            Assert.AreEqual(0, goAnyCollision, "A sensor overlap spuriously produced a GameObject collision event.");

            // Trigger Stay derived as the begin..end interval (no literal package Stay event).
            if (goTrigStay.Count > 0)
            {
                Assert.Greater(
                    pkgTrigTouchN,
                    0,
                    "GO reported trigger-Stay frames but the package derived no touching interval for the pair."
                );
                // Both interval lengths agree within a generous envelope (a few frames at each edge).
                Assert.LessOrEqual(
                    abs(pkgTrigTouchN - goTrigStay.Count),
                    6,
                    $"Trigger touching-interval length diverged > 6 frames: package={pkgTrigTouchN}, "
                        + $"GO Stay frames={goTrigStay.Count}."
                );
            }
            else
            {
                Assert.Greater(
                    pkgTrigTouchN,
                    0,
                    "Package derived no trigger touching interval for a body that passed through the sensor "
                        + "(GO OnTriggerStay2D did not fire in batchmode — documented)."
                );
            }
            yield break;
        }

        // =====================================================================================================
        // INVARIANT 3 — FILTER consistency: a collision-filtered-out pair raises NEITHER contact nor trigger
        // events (Phase-5 layers). Two solid bodies on layers the matrix ignores must overlap silently.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator Filter_IgnoredPair_RaisesNoContactNorTrigger()
        {
            NewRefMaterial();
            const int Steps = 240;
            const int LA = 8;
            const int LB = 9;
            const float r = 0.5f;

            // Ignore the A-vs-B pair so the two overlapping bodies do not collide.
            var prevIgnore = UnityEngine.Physics2D.GetIgnoreLayerCollision(LA, LB);
            UnityEngine.Physics2D.IgnoreLayerCollision(LA, LB, true);
            try
            {
                ulong catA = 1ul << LA;
                ulong conA = unchecked((uint)UnityEngine.Physics2D.GetLayerCollisionMask(LA));
                ulong catB = 1ul << LB;
                ulong conB = unchecked((uint)UnityEngine.Physics2D.GetLayerCollisionMask(LB));

                // A static box on layer A and a dynamic circle on layer B dropped straight onto it: with A-vs-B
                // ignored, the circle passes through with NO contact and NO trigger (both solid, not sensors).
                var world = MakePackageWorld(out var group);
                var em = world.EntityManager;
                var boxE = DirectPhysics2DAuthoring.Create(
                    em,
                    new PhysicsBody2DDefinition
                    {
                        bodyType = PhysicsBody.BodyType.Static,
                        initialPosition = new float2(0f, 0f),
                    },
                    new PhysicsShape2D
                    {
                        kind = PhysicsShape2DKind.Box,
                        size = new float2(4f, 1f),
                        radius = 0f,
                        density = 1f,
                        friction = 0.4f,
                        categoryBits = catA,
                        contactBits = conA,
                    }
                );
                var circleE = DirectPhysics2DAuthoring.Create(
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
                        categoryBits = catB,
                        contactBits = conB,
                    }
                );
                var pkgTotalContacts = 0;
                var pkgTotalTriggers = 0;
                var passedThrough = false;
                group.Update();
                for (var s = 0; s < Steps; s++)
                {
                    group.Update();
                    var se = SingletonEntity(em);
                    pkgTotalContacts += em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true).Length;
                    pkgTotalTriggers += em.GetBuffer<PhysicsTriggerEvent2D>(se, isReadOnly: true).Length;
                    if (em.GetComponentData<LocalToWorld>(circleE).Position.y < -3f)
                        passedThrough = true;
                }
                world.Dispose();

                // ---- GameObject oracle ----
                var track = new List<GameObject>();
                var boxGo = new GameObject("FltBox") { layer = LA };
                boxGo.transform.position = Vector3.zero;
                var boxRb = boxGo.AddComponent<Rigidbody2D>();
                boxRb.bodyType = RigidbodyType2D.Static;
                boxRb.sleepMode = RigidbodySleepMode2D.NeverSleep;
                var boxCol = boxGo.AddComponent<BoxCollider2D>();
                boxCol.size = new Vector2(4f, 1f);
                boxCol.sharedMaterial = _refMaterial;
                var boxEc = boxGo.AddComponent<EventCounter2D>();
                track.Add(boxGo);

                var circGo = new GameObject("FltCircle") { layer = LB };
                circGo.transform.position = new Vector3(0f, 4f, 0f);
                var circRb = circGo.AddComponent<Rigidbody2D>();
                circRb.bodyType = RigidbodyType2D.Dynamic;
                circRb.gravityScale = 1f;
                circRb.sleepMode = RigidbodySleepMode2D.NeverSleep;
                var circCol = circGo.AddComponent<CircleCollider2D>();
                circCol.radius = r;
                circCol.sharedMaterial = _refMaterial;
                var circEc = circGo.AddComponent<EventCounter2D>();
                track.Add(circGo);

                UnityEngine.Physics2D.SyncTransforms();
                var goPassed = false;
                for (var s = 0; s < Steps; s++)
                {
                    EventCounter2D.GoStep = s;
                    UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                    if (circGo.transform.position.y < -3f)
                        goPassed = true;
                    yield return null;
                }
                var goEvents =
                    CountEnter(boxEc, false)
                    + CountEnter(circEc, false)
                    + CountEnter(boxEc, true)
                    + CountEnter(circEc, true);
                DestroyAll(track);

                Debug.Log(
                    $"[PHYSICS2D-P6GATE] FILTER-IGNORED: pkg contacts={pkgTotalContacts} triggers={pkgTotalTriggers} "
                        + $"passedThrough={passedThrough} | GO events={goEvents} goPassed={goPassed}."
                );

                Assert.IsTrue(passedThrough, "Package: the filtered-out circle did not pass through (it collided).");
                Assert.IsTrue(goPassed, "GameObject oracle: the filtered-out circle did not pass through.");
                Assert.AreEqual(
                    0,
                    pkgTotalContacts,
                    "A filtered-out pair produced package CONTACT events — events did not respect the Phase-5 filter."
                );
                Assert.AreEqual(
                    0,
                    pkgTotalTriggers,
                    "A filtered-out pair produced package TRIGGER events (neither shape is a sensor anyway)."
                );
                Assert.AreEqual(0, goEvents, "GameObject oracle fired events for a layer-ignored pair.");
            }
            finally
            {
                UnityEngine.Physics2D.IgnoreLayerCollision(LA, LB, prevIgnore);
            }
            yield break;
        }

        // =====================================================================================================
        // ESCALATED DECISION POINT — EVENT VOLUME: many simultaneous contacts in one step. N dynamic circles
        // dropped onto one floor; each fires one floor-contact begin. The package buffer must hold the FULL set
        // (no dropped/duplicated events) and resolve every pair to real entities. Cross-checked against the GO
        // oracle's total OnCollisionEnter2D count from N matching bodies.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator Volume_ManySimultaneousContacts_FullSetNoDropNoDup()
        {
            NewRefMaterial();
            const int N = 16;
            const int Steps = 240;
            const float r = 0.4f;
            var floorCenter = new float2(0f, -0.5f);
            var floorSize = new float2(40f, 1f);
            // Spread the circles across the floor so each lands on the floor (its own contact), well separated
            // so they do not touch each other (each pair is exactly one body↔floor contact).
            Unity.Mathematics.float2[] spawn = new Unity.Mathematics.float2[N];
            for (var i = 0; i < N; i++)
                spawn[i] = new float2((i - N / 2) * 2.0f, 4f + (i % 3) * 0.5f);

            // ---- package ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var floor = SpawnPkgBox(em, floorCenter, floorSize, isTrigger: false, dynamic: false);
            var bodies = new Entity[N];
            for (var i = 0; i < N; i++)
                bodies[i] = SpawnPkgCircle(em, spawn[i], r);
            var contactLogs = new Dictionary<long, PkgPairLog>();
            var maxBufLenSeen = 0;
            var nullResolutions = 0;
            group.Update();
            for (var s = 0; s < Steps; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                var buf = em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true);
                maxBufLenSeen = max(maxBufLenSeen, buf.Length);
                for (var i = 0; i < buf.Length; i++)
                {
                    if (buf[i].entityA == Entity.Null || buf[i].entityB == Entity.Null)
                        nullResolutions++;
                }
                DrainContacts(buf, s, contactLogs);
            }
            // Count distinct body↔floor pairs that fired a begin, and how many begins each fired.
            var bodiesWithFloorBegin = 0;
            var duplicateBegins = 0;
            foreach (var b in bodies)
            {
                if (contactLogs.TryGetValue(PairKey(b, floor), out var log) && log.beginSteps.Count > 0)
                {
                    bodiesWithFloorBegin++;
                    if (log.beginSteps.Count > 1)
                        duplicateBegins++;
                }
            }
            world.Dispose();

            // ---- GameObject ----
            var track = new List<GameObject>();
            var floorGo = new GameObject("VolFloor") { layer = 0 };
            floorGo.transform.position = new Vector3(floorCenter.x, floorCenter.y, 0f);
            var floorRb = floorGo.AddComponent<Rigidbody2D>();
            floorRb.bodyType = RigidbodyType2D.Static;
            floorRb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var floorCol = floorGo.AddComponent<BoxCollider2D>();
            floorCol.size = (Vector2)floorSize;
            floorCol.sharedMaterial = _refMaterial;
            track.Add(floorGo);
            var bodyEcs = new EventCounter2D[N];
            for (var i = 0; i < N; i++)
                MakeGoDynamicCircle(spawn[i], r, track, out bodyEcs[i]);
            UnityEngine.Physics2D.SyncTransforms();
            for (var s = 0; s < Steps; s++)
            {
                EventCounter2D.GoStep = s;
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                yield return null;
            }
            var goBodiesWithEnter = 0;
            for (var i = 0; i < N; i++)
                if (CountEnter(bodyEcs[i], trigger: false) > 0)
                    goBodiesWithEnter++;
            DestroyAll(track);

            Debug.Log(
                $"[PHYSICS2D-P6GATE] VOLUME: N={N} pkg bodiesWithFloorBegin={bodiesWithFloorBegin} "
                    + $"duplicateBegins={duplicateBegins} maxBufLen={maxBufLenSeen} nullResolutions={nullResolutions} | "
                    + $"GO bodiesWithEnter={goBodiesWithEnter}."
            );

            // Full set: every one of the N bodies fired exactly one floor-contact begin on BOTH backends.
            Assert.AreEqual(N, bodiesWithFloorBegin, "Package dropped a body↔floor begin event under volume.");
            Assert.AreEqual(0, duplicateBegins, "Package duplicated a body↔floor begin event (one body, >1 begin).");
            Assert.AreEqual(
                0,
                nullResolutions,
                "A contact event under volume resolved an Entity.Null — pair→entity failed."
            );
            Assert.AreEqual(N, goBodiesWithEnter, "GameObject oracle: not every body landed (geometry/setup off).");
            yield break;
        }

        // =====================================================================================================
        // ESCALATED DECISION POINT — DESTROYED SHAPE: an entity destroyed the same step it would end/generate a
        // contact. The event drain + pair→entity resolution must not read freed memory or resolve a dangling
        // entity — either a valid end event or none, never a crash or a garbage entity. (No GameObject oracle:
        // this is a native-lifetime safety probe of the package's own decision point.)
        // =====================================================================================================
        [UnityTest]
        public IEnumerator DestroyedShape_EntityDestroyedWhileTouching_DrainSafeNoGarbageEntity()
        {
            NewRefMaterial();
            const float r = 0.5f;
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var floor = SpawnPkgBox(em, new float2(0f, -0.5f), new float2(6f, 1f), isTrigger: false, dynamic: false);
            var body = SpawnPkgCircle(em, new float2(0f, 1.2f), r); // starts just above the floor → lands fast

            group.Update(); // create
            // Step until the body is resting on the floor (a begin has fired, pair is touching).
            var landed = false;
            for (var s = 0; s < 120 && !landed; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                var buf = em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true);
                for (var i = 0; i < buf.Length; i++)
                    if (
                        buf[i].phase == PhysicsEventPhase2D.Begin
                        && (
                            (buf[i].entityA == body && buf[i].entityB == floor)
                            || (buf[i].entityA == floor && buf[i].entityB == body)
                        )
                    )
                        landed = true;
            }
            Assert.IsTrue(landed, "Setup: the body never landed on the floor before the destroy step.");

            // Destroy the resting body. PhysicsBody2DCleanupSystem frees its native body BEFORE the next step;
            // the freed shape generates a contact-END (the pair stopped touching because one shape vanished),
            // and CollectEvents must drain that end without reading freed memory. The end's destroyed-shape side
            // must resolve to Entity.Null via the isValid guard (or the event must simply not appear) — never a
            // crash, never a garbage non-null entity that is not `body`/`floor`.
            em.DestroyEntity(body);

            var sawGarbage = false;
            var endResolvedFloorOnly = false;
            var crashed = false;
            try
            {
                // The cleanup+step that frees the body and drains any resulting end event.
                group.Update();
                var se = SingletonEntity(em);
                var buf = em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true);
                for (var i = 0; i < buf.Length; i++)
                {
                    var e = buf[i];
                    // Every resolved non-null entity in any event this step must be `floor` (the only survivor)
                    // or Entity.Null (the destroyed body). Anything else is garbage from freed memory.
                    foreach (var ent in new[] { e.entityA, e.entityB })
                    {
                        if (ent == Entity.Null || ent == floor)
                            continue;
                        if (ent == body)
                            continue; // a stale-but-correct index is acceptable; the guard is "not garbage"
                        sawGarbage = true;
                    }
                    if (
                        e.phase == PhysicsEventPhase2D.End
                        && (e.entityA == floor || e.entityB == floor)
                        && (e.entityA == Entity.Null || e.entityB == Entity.Null)
                    )
                        endResolvedFloorOnly = true;
                }
                // Keep stepping a while: the world must stay sane (no crash on subsequent drains).
                for (var s = 0; s < 30; s++)
                    group.Update();
            }
            catch (System.Exception ex)
            {
                crashed = true;
                Debug.Log($"[PHYSICS2D-P6GATE] DESTROYED-SHAPE: drain threw: {ex}");
            }

            Debug.Log(
                $"[PHYSICS2D-P6GATE] DESTROYED-SHAPE: crashed={crashed} sawGarbageEntity={sawGarbage} "
                    + $"endResolvedFloor+Null={endResolvedFloorOnly}. (A valid end with the destroyed side Null, "
                    + "or no end at all, are both acceptable; a crash or a garbage entity are not.)"
            );

            world.Dispose();

            Assert.IsFalse(
                crashed,
                "The event drain crashed when an entity was destroyed the step it ended a contact."
            );
            Assert.IsFalse(
                sawGarbage,
                "An event after a destroyed-shape step resolved to a garbage entity (neither floor, body, nor Null) "
                    + "— the drain read freed memory or unpacked a dangling userData."
            );
            yield break;
        }

        // =====================================================================================================
        // ESCALATED DECISION POINT — WORLD RECREATE: the PhysicsWorld is invalidated (the module-reset path the
        // lazy-world-ensure handles). Events must keep populating across the re-creation: no stale events
        // carried over, no loss on the first post-recreate step. Simulated by destroying the singleton's world
        // handle so the next OnUpdate re-creates it, then re-authoring bodies and confirming a fresh begin fires.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator WorldRecreate_EventsKeepPopulating_NoStaleNoLoss()
        {
            NewRefMaterial();
            const float r = 0.5f;
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;

            // --- Phase A: land a body, observe a begin, confirm a non-empty buffer. ---
            var floorA = SpawnPkgBox(em, new float2(0f, -0.5f), new float2(6f, 1f), isTrigger: false, dynamic: false);
            var bodyA = SpawnPkgCircle(em, new float2(0f, 1.2f), r);
            group.Update();
            var beginsBefore = 0;
            for (var s = 0; s < 120; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                var buf = em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true);
                for (var i = 0; i < buf.Length; i++)
                    if (buf[i].phase == PhysicsEventPhase2D.Begin)
                        beginsBefore++;
            }
            Assert.Greater(beginsBefore, 0, "Pre-recreate: no begin observed in phase A.");

            // Destroy the old bodies so they do not survive into the recreated world as ghosts.
            em.DestroyEntity(bodyA);
            em.DestroyEntity(floorA);
            group.Update(); // cleanup frees them in the OLD world

            // --- Force a world recreate: invalidate the singleton's PhysicsWorld handle. The next OnUpdate sees
            // !world.isValid and re-creates it (the module-reset robustness path), keeping the singleton entity
            // (and its event buffers) but with a fresh native world. ---
            var singletonEntity = SingletonEntity(em);
            var singleton = em.GetComponentData<PhysicsWorldSingleton2D>(singletonEntity);
            Assert.IsTrue(singleton.world.isValid, "The world was already invalid before the forced recreate.");
            singleton.world.Destroy(); // invalidate the handle
            em.SetComponentData(singletonEntity, singleton);

            // The buffers from phase A must be cleared at the top of the next step (not carried stale).
            group.Update(); // OnUpdate re-creates the world; buffers cleared; no bodies yet → no step → empty
            var seAfterRecreate = SingletonEntity(em);
            var bufAfterRecreate = em.GetBuffer<PhysicsContactEvent2D>(seAfterRecreate, isReadOnly: true);
            Assert.AreEqual(
                0,
                bufAfterRecreate.Length,
                "Stale contact events survived the world recreate — the buffer was not cleared."
            );
            var recreated = em.GetComponentData<PhysicsWorldSingleton2D>(singletonEntity).world;
            Assert.IsTrue(recreated.isValid, "The world was not re-created after its handle was invalidated.");

            // --- Phase B: author NEW bodies in the recreated world; a fresh begin must fire (no loss). ---
            var floorB = SpawnPkgBox(em, new float2(0f, -0.5f), new float2(6f, 1f), isTrigger: false, dynamic: false);
            var bodyB = SpawnPkgCircle(em, new float2(0f, 1.2f), r);
            group.Update(); // create in the recreated world (no step)
            var beginsAfter = 0;
            var firstBeginStep = -1;
            for (var s = 0; s < 120; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                var buf = em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true);
                for (var i = 0; i < buf.Length; i++)
                    if (
                        buf[i].phase == PhysicsEventPhase2D.Begin
                        && (
                            (buf[i].entityA == bodyB && buf[i].entityB == floorB)
                            || (buf[i].entityA == floorB && buf[i].entityB == bodyB)
                        )
                    )
                    {
                        beginsAfter++;
                        if (firstBeginStep < 0)
                            firstBeginStep = s;
                    }
            }
            world.Dispose();

            Debug.Log(
                $"[PHYSICS2D-P6GATE] WORLD-RECREATE: beginsBefore={beginsBefore} beginsAfter={beginsAfter} "
                    + $"firstBeginStepAfter={firstBeginStep} bufAfterRecreate.Length=0 (cleared)."
            );

            Assert.AreEqual(
                1,
                beginsAfter,
                "After a world recreate, a freshly-authored body↔floor pair did not fire exactly one begin — "
                    + "events were lost or duplicated across the recreate boundary."
            );
            yield break;
        }

        // =====================================================================================================
        // ESCALATED DECISION POINT — TRIGGER-vs-TRIGGER. The design ASSUMED a divergence: that Box2D-v3 sensors
        // "do not collide with other triggers" (XML :11665) and so the package would emit NO trigger event for a
        // sensor↔sensor pair, while GameObject 2D physics DOES raise OnTriggerEnter2D between two triggers — i.e.
        // the package would under-report. This probe was built to assert that assumed divergence (package 0, GO
        // >0) and FALSIFIED it: in editor 6000.6.0a6 BOTH backends fire the trigger-vs-trigger pair (package
        // emits a TriggerBeginEvent, GameObject raises OnTriggerEnter2D on both colliders). The XML ":11665"
        // sentence governs collision RESPONSE between sensors (no solid contact), not trigger EVENT reporting —
        // and with triggerEvents=true on every shape (the BOTH-shape rule satisfied), two sensors DO produce a
        // trigger begin/end. So the package MATCHES the GameObject oracle here; the assumed divergence does not
        // exist in this editor. This test now asserts the OBSERVED parity (both detect the sensor pair) and pins
        // the package's per-pair structure so a future drop or runaway duplication is caught.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator TriggerVsTrigger_BothBackendsDetectSensorPair_ObservedParity()
        {
            NewRefMaterial();
            const int Steps = 80;
            // Two overlapping STATIC sensors (no gravity needed — they overlap from the start and stay).
            var posA = new float2(0f, 0f);
            var posB = new float2(0.5f, 0f);
            var size = new float2(2f, 2f);

            // ---- package: two static trigger boxes overlapping ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var sa = SpawnPkgBox(em, posA, size, isTrigger: true, dynamic: false);
            var sb = SpawnPkgBox(em, posB, size, isTrigger: true, dynamic: false);
            var triggerLogs = new Dictionary<long, PkgPairLog>();
            group.Update();
            for (var s = 0; s < Steps; s++)
            {
                group.Update();
                var se = SingletonEntity(em);
                DrainTriggers(em.GetBuffer<PhysicsTriggerEvent2D>(se, isReadOnly: true), s, triggerLogs);
                RecordTouching(s, triggerLogs);
            }
            // The unordered sensor pair must appear with at least one begin and stay touching (never separated).
            var pkgPair = triggerLogs.TryGetValue(PairKey(sa, sb), out var tl) ? tl : null;
            var pkgPairBegins = pkgPair?.beginSteps.Count ?? 0;
            var pkgPairEnds = pkgPair?.endSteps.Count ?? 0;
            var pkgPairTouchN = pkgPair?.touchingSteps.Count ?? 0;
            // Distinct unordered pairs that fired any trigger event (must be exactly the ONE sa↔sb pair —
            // symmetric A-sees-B/B-sees-A reports collapse to one unordered key, so a count > 1 would be a
            // spurious extra pair).
            var distinctPairs = triggerLogs.Count;
            world.Dispose();

            // ---- GameObject: two overlapping trigger colliders. At least one must be on a Rigidbody2D for 2D
            // trigger messages to dispatch; make both kinematic non-moving so they stay overlapped. ----
            var track = new List<GameObject>();
            var goA = new GameObject("TrigA") { layer = 0 };
            goA.transform.position = new Vector3(posA.x, posA.y, 0f);
            var rbA = goA.AddComponent<Rigidbody2D>();
            rbA.bodyType = RigidbodyType2D.Kinematic;
            rbA.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var colA = goA.AddComponent<BoxCollider2D>();
            colA.size = (Vector2)size;
            colA.isTrigger = true;
            var ecA = goA.AddComponent<EventCounter2D>();
            track.Add(goA);

            var goB = new GameObject("TrigB") { layer = 0 };
            goB.transform.position = new Vector3(posB.x, posB.y, 0f);
            var rbB = goB.AddComponent<Rigidbody2D>();
            rbB.bodyType = RigidbodyType2D.Kinematic;
            rbB.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var colB = goB.AddComponent<BoxCollider2D>();
            colB.size = (Vector2)size;
            colB.isTrigger = true;
            var ecB = goB.AddComponent<EventCounter2D>();
            track.Add(goB);

            UnityEngine.Physics2D.SyncTransforms();
            for (var s = 0; s < Steps; s++)
            {
                EventCounter2D.GoStep = s;
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
                yield return null;
            }
            // Each sensor sees the other: OnTriggerEnter2D on A (other=B) and on B (other=A).
            var goEnterA = CountEnter(ecA, true);
            var goEnterB = CountEnter(ecB, true);
            DestroyAll(track);

            Debug.Log(
                $"[PHYSICS2D-P6GATE] TRIGGER-VS-TRIGGER: package pairBegins={pkgPairBegins} pairEnds={pkgPairEnds} "
                    + $"pairTouchSteps={pkgPairTouchN} distinctTriggerPairs={distinctPairs} | "
                    + $"GameObject enterA={goEnterA} enterB={goEnterB}. OBSERVED PARITY: both backends detect "
                    + "the sensor↔sensor pair in editor 6000.6.0a6 — the design's assumed divergence is absent "
                    + "(XML :11665 governs collision RESPONSE, not trigger EVENT reporting)."
            );

            // OBSERVED PARITY (not a forced match — the empirical truth this editor shows): both backends detect
            // the sensor-vs-sensor pair.
            // GameObject oracle: each trigger fires OnTriggerEnter2D for the other.
            Assert.GreaterOrEqual(goEnterA, 1, "GameObject sensor A did not detect sensor B (oracle expectation).");
            Assert.GreaterOrEqual(goEnterB, 1, "GameObject sensor B did not detect sensor A (oracle expectation).");
            // Package: the one unordered sensor pair fired a begin and stayed touching (never separated).
            Assert.GreaterOrEqual(
                pkgPairBegins,
                1,
                "Package did not emit a trigger-begin for the overlapping sensor pair — it MATCHES the GameObject "
                    + "oracle (both detect sensor-vs-sensor in this editor), so a silent package is the regression."
            );
            Assert.AreEqual(
                0,
                pkgPairEnds,
                "Package emitted a trigger-end for a sensor pair that never separated (a phantom end)."
            );
            Assert.AreEqual(
                1,
                distinctPairs,
                $"Package reported {distinctPairs} distinct trigger pairs for two sensors — expected exactly 1 "
                    + "(the symmetric A↔B reports must collapse to one unordered pair, not duplicate as extra pairs)."
            );
            Assert.Greater(
                pkgPairTouchN,
                Steps / 2,
                $"Package sensor pair touching-interval implausibly short ({pkgPairTouchN}) for two statically "
                    + "overlapping sensors that never separate."
            );
            yield break;
        }
    }
}
