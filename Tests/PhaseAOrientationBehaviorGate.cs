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
    /// The independent adversarial behavioural gate for the Phase-A free box/capsule ORIENTATION fields
    /// (<see cref="Authoring.PhysicsShape2DAuthoring.BoxAngle"/> /
    /// <see cref="Authoring.PhysicsShape2DAuthoring.CapsuleAngle"/>, baked to
    /// <c>PhysicsShape2D.boxAngleRadians</c> / the rotated capsule centres). These rotate the COLLISION GEOMETRY
    /// while leaving the body pose un-rotated — the built-in <c>BoxCollider2D</c>/<c>CapsuleCollider2D</c> cannot
    /// express that (their rotation is their Transform's), so the probe asserts the rotated shape COLLIDES as
    /// rotated, against a GameObject collider on a Transform rotated by the same angle (the closest equivalent).
    /// </summary>
    /// <remarks>
    /// <para><b>The falsifying observable is the rest HEIGHT, not the body angle.</b> A 1×1 box rotated 45° and
    /// FROZEN in rotation rests on a floor with a corner down: its centre settles at
    /// <c>floorTop + halfDiagonal = 0.5 + 0.5·√2 ≈ 1.207</c>, distinctly above the axis-aligned box's
    /// <c>floorTop + halfHeight = 1.0</c>. Freezing rotation isolates the GEOMETRY orientation (an un-frozen box
    /// would topple flat and erase the difference). A baker that dropped <c>boxAngleRadians</c> bakes an
    /// axis-aligned box that rests at ~1.0, which this gate catches; the rotated rest height is cross-checked
    /// against a GameObject <c>BoxCollider2D</c> on a Transform rotated 45°, also rotation-frozen — the same
    /// rotated square reached the built-in way.</para>
    ///
    /// <para><b>World isolation.</b> Each test owns a disposable <see cref="World"/>; the GameObject reference is
    /// torn down and global <c>Physics2D</c> state restored in <c>[TearDown]</c>. The rest height is a
    /// deterministic settle, so two runs produce the same witness.</para>
    /// </remarks>
    public sealed class PhaseAOrientationBehaviorGate
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
            var world = new World("PhaseAOrientGateWorld");
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

        static float EcsY(EntityManager em, Entity e) =>
            em.GetComponentData<LocalToWorld>(e).Value.c3.y;

        // =====================================================================================================
        // INVARIANT — a free-rotated box collides at its rotated extent. A 1×1 box authored with boxAngleRadians
        // = 45° and rotation frozen rests, corner-down, with its centre at floorTop + half-diagonal ≈ 1.207; an
        // axis-aligned box rests at floorTop + half-height = 1.0. The rotated rest height matches a GameObject
        // BoxCollider2D on a Transform rotated 45° (also rotation-frozen).
        // =====================================================================================================
        [UnityTest]
        public IEnumerator BoxOrientation_RotatedBoxRestsAtRotatedHeight_MatchesTransformRotatedBuiltIn()
        {
            const int Steps = 240;
            const float floorTop = 0.5f; // a floor box of size (40,1) centred at y=0 → top at 0.5
            var angle = radians(45f);
            var halfDiagonal = 0.5f * sqrt(2f); // ≈ 0.7071 for a unit box's corner-to-centre
            var expectedRotated = floorTop + halfDiagonal; // ≈ 1.207
            var expectedAxisAligned = floorTop + 0.5f; // = 1.0

            // ---- package: rotated box (boxAngleRadians) + axis-aligned control, both rotation-frozen ----
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var rotated = SpawnPkgFrozenBox(em, new float2(0f, 4f), new float2(1f, 1f), angle);
            var axis = SpawnPkgFrozenBox(em, new float2(8f, 4f), new float2(1f, 1f), 0f);
            SpawnPkgFloor(em);
            group.Update();
            for (var s = 0; s < Steps; s++)
                group.Update();
            var pkgRotatedY = EcsY(em, rotated);
            var pkgAxisY = EcsY(em, axis);
            world.Dispose();

            // ---- GameObject oracle: a BoxCollider2D on a Transform rotated 45°, rotation-frozen ----
            var goRotatedY = RunGoFrozenBoxOnFloor(45f, floorTop);

            Debug.Log(
                $"[PHYSICS2D-PHASEA-BOXORIENT] pkgRotatedRestY={pkgRotatedY:F4} (expect≈{expectedRotated:F4}) "
                    + $"pkgAxisAlignedRestY={pkgAxisY:F4} (expect≈{expectedAxisAligned:F4}) | "
                    + $"GO rotatedRestY={goRotatedY:F4}."
            );

            // 1. The rotated box rests notably higher than the axis-aligned control — the rotation reached the
            //    collision geometry (a dropped boxAngleRadians would leave both at ~1.0).
            Assert.Greater(
                pkgRotatedY - pkgAxisY,
                0.12f,
                $"The boxAngle-rotated box rested at {pkgRotatedY} vs the axis-aligned {pkgAxisY} — the free box "
                    + "rotation did not reach the collision geometry (boxAngleRadians dropped at bake/create)."
            );
            // 2. The rotated rest height matches the analytic corner-down height within a contact tolerance.
            Assert.Less(
                abs(pkgRotatedY - expectedRotated),
                0.08f,
                $"The rotated box rested at {pkgRotatedY}, not the corner-down height {expectedRotated} — the "
                    + "box is rotated by the wrong angle."
            );
            // 3. The axis-aligned control rests at the flat height (sanity: the control is genuinely unrotated).
            Assert.Less(
                abs(pkgAxisY - expectedAxisAligned),
                0.08f,
                $"The axis-aligned control rested at {pkgAxisY}, not the flat height {expectedAxisAligned}."
            );
            // 4. GameObject parity: a BoxCollider2D on a 45°-rotated Transform rests at the same rotated height.
            Assert.Less(
                abs(pkgRotatedY - goRotatedY),
                0.10f,
                $"The package rotated-box rest height {pkgRotatedY} disagrees with the GameObject "
                    + $"Transform-rotated BoxCollider2D {goRotatedY} beyond the v2/v3 contact band."
            );
            yield break;
        }

        // =====================================================================================================
        // INVARIANT — a free-rotated capsule collides at its rotated extent. A vertical capsule (1×2) authored
        // with capsuleAngle = 90° becomes a HORIZONTAL capsule whose collision half-height is its radius (0.5),
        // so frozen it rests at floorTop + 0.5 = 1.0; the un-rotated vertical capsule rests at floorTop + 1.0 =
        // 1.5. The rotated rest height matches a GameObject CapsuleCollider2D on a Transform rotated 90°.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator CapsuleOrientation_RotatedCapsuleRestsAtRotatedHeight_MatchesTransformRotatedBuiltIn()
        {
            const int Steps = 240;
            const float floorTop = 0.5f;
            // A vertical capsule of size (1,2): radius 0.5, vertical half-extent 1.0. Rotated 90° it lies
            // horizontal → vertical half-extent becomes the radius 0.5.
            var expectedRotated = floorTop + 0.5f; // ≈ 1.0 (lying on its side)
            var expectedVertical = floorTop + 1.0f; // = 1.5 (standing up)

            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var rotated = SpawnPkgFrozenCapsule(em, new float2(0f, 4f), new float2(1f, 2f), radians(90f));
            var vertical = SpawnPkgFrozenCapsule(em, new float2(8f, 4f), new float2(1f, 2f), 0f);
            SpawnPkgFloor(em);
            group.Update();
            for (var s = 0; s < Steps; s++)
                group.Update();
            var pkgRotatedY = EcsY(em, rotated);
            var pkgVerticalY = EcsY(em, vertical);
            world.Dispose();

            var goRotatedY = RunGoFrozenCapsuleOnFloor(90f, floorTop);

            Debug.Log(
                $"[PHYSICS2D-PHASEA-CAPORIENT] pkgRotatedRestY={pkgRotatedY:F4} (expect≈{expectedRotated:F4}) "
                    + $"pkgVerticalRestY={pkgVerticalY:F4} (expect≈{expectedVertical:F4}) | "
                    + $"GO rotatedRestY={goRotatedY:F4}."
            );

            // 1. The rotated capsule rests notably LOWER than the un-rotated vertical one (it is lying down).
            Assert.Greater(
                pkgVerticalY - pkgRotatedY,
                0.3f,
                $"The capsuleAngle-rotated capsule rested at {pkgRotatedY} vs the vertical {pkgVerticalY} — the "
                    + "free capsule rotation did not reach the collision geometry (the rotated centres dropped)."
            );
            // 2. The rotated capsule rests at the lying-down height (radius above the floor).
            Assert.Less(
                abs(pkgRotatedY - expectedRotated),
                0.10f,
                $"The rotated capsule rested at {pkgRotatedY}, not the lying-down height {expectedRotated}."
            );
            // 3. The vertical control rests at the standing height.
            Assert.Less(
                abs(pkgVerticalY - expectedVertical),
                0.10f,
                $"The vertical control capsule rested at {pkgVerticalY}, not the standing height {expectedVertical}."
            );
            // 4. GameObject parity: a CapsuleCollider2D on a 90°-rotated Transform rests at the same height.
            Assert.Less(
                abs(pkgRotatedY - goRotatedY),
                0.12f,
                $"The package rotated-capsule rest height {pkgRotatedY} disagrees with the GameObject "
                    + $"Transform-rotated CapsuleCollider2D {goRotatedY} beyond the v2/v3 contact band."
            );
            yield break;
        }

        // ----- package spawn helpers -----

        static Entity SpawnPkgFrozenBox(EntityManager em, float2 pos, float2 size, float boxAngleRadians)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = pos,
                    useAutoMass = true,
                    // Freeze rotation so the box keeps its authored geometry orientation as it settles (otherwise
                    // it topples flat and the rotated-vs-axis-aligned rest height difference vanishes).
                    constraints = PhysicsBody.BodyConstraints.Rotation,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = size,
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                    boxAngleRadians = boxAngleRadians,
                }
            );
        }

        static Entity SpawnPkgFrozenCapsule(EntityManager em, float2 pos, float2 size, float capsuleAngleRadians)
        {
            // Derive the two end-cap centres for a vertical capsule, then rotate them by capsuleAngleRadians —
            // the same derivation PhysicsShape2DAuthoring.GetCapsuleCenters performs at bake time.
            var halfSize = size * 0.5f;
            var radius = halfSize.x;
            var half = max(0f, halfSize.y - radius);
            var c1 = new float2(0f, -half);
            var c2 = new float2(0f, half);
            if (capsuleAngleRadians != 0f)
            {
                sincos(capsuleAngleRadians, out var sn, out var cs);
                c1 = new float2(cs * c1.x - sn * c1.y, sn * c1.x + cs * c1.y);
                c2 = new float2(cs * c2.x - sn * c2.y, sn * c2.x + cs * c2.y);
            }
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = pos,
                    useAutoMass = true,
                    constraints = PhysicsBody.BodyConstraints.Rotation,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Capsule,
                    radius = radius,
                    capsuleCenter1 = c1,
                    capsuleCenter2 = c2,
                    density = 1f,
                    friction = 0.4f,
                }
            );
        }

        static void SpawnPkgFloor(EntityManager em)
        {
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
                    size = new float2(40f, 1f),
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
        }

        // ----- GameObject oracles: a collider on a Transform rotated by `angleDeg`, rotation-frozen, dropped
        // onto a static floor, run to rest. Returns the settled centre y. -----

        static float RunGoFrozenBoxOnFloor(float angleDeg, float floorTop)
        {
            var track = new List<GameObject>();
            MakeGoFloor(track, floorTop);
            var go = new GameObject("GoOrientBox") { layer = 0 };
            go.transform.position = new Vector3(0f, 4f, 0f);
            go.transform.rotation = Quaternion.Euler(0f, 0f, angleDeg);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.useAutoMass = true;
            rb.constraints = RigidbodyConstraints2D.FreezeRotation;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            go.AddComponent<BoxCollider2D>().size = new Vector2(1f, 1f);
            track.Add(go);
            UnityEngine.Physics2D.SyncTransforms();
            for (var s = 0; s < 240; s++)
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            var y = go.transform.position.y;
            foreach (var g in track)
                Object.Destroy(g);
            return y;
        }

        static float RunGoFrozenCapsuleOnFloor(float angleDeg, float floorTop)
        {
            var track = new List<GameObject>();
            MakeGoFloor(track, floorTop);
            var go = new GameObject("GoOrientCapsule") { layer = 0 };
            go.transform.position = new Vector3(0f, 4f, 0f);
            go.transform.rotation = Quaternion.Euler(0f, 0f, angleDeg);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.useAutoMass = true;
            rb.constraints = RigidbodyConstraints2D.FreezeRotation;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var cap = go.AddComponent<CapsuleCollider2D>();
            cap.size = new Vector2(1f, 2f);
            cap.direction = CapsuleDirection2D.Vertical;
            track.Add(go);
            UnityEngine.Physics2D.SyncTransforms();
            for (var s = 0; s < 240; s++)
                UnityEngine.Physics2D.Simulate(Dt, UnityEngine.Physics2D.AllLayers);
            var y = go.transform.position.y;
            foreach (var g in track)
                Object.Destroy(g);
            return y;
        }

        static void MakeGoFloor(List<GameObject> track, float floorTop)
        {
            var floor = new GameObject("GoOrientFloor") { layer = 0 };
            floor.transform.position = new Vector3(0f, floorTop - 0.5f, 0f);
            var rb = floor.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            floor.AddComponent<BoxCollider2D>().size = new Vector2(40f, 1f);
            track.Add(floor);
        }
    }
}
