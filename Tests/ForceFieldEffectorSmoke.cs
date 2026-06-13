using System.Collections;
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
    /// Phase-10a smoke for the three force-field effectors (Area / Buoyancy / Point). Three minimal mechanism
    /// witnesses (the hard GameObject-parity e2e gate is a separate validating agent's deliverable):
    /// <list type="bullet">
    /// <item><b>Area</b> — a dynamic body inside a directional force zone accelerates in the force direction.</item>
    /// <item><b>Buoyancy</b> — a dynamic body dropped into a fluid volume decelerates and floats up to rest near
    /// the fluid surface (rather than sinking out the bottom).</item>
    /// <item><b>Point</b> — a dynamic body near an attracting point (negative forceMagnitude) accelerates toward
    /// the point.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/> holding the package's four FixedStep systems,
    /// driven one fixed step per <c>group.Update()</c>. The effector is authored directly as a static body + a
    /// sensor (<c>isTrigger</c>) shape (its region) + a <see cref="PhysicsEffector2D"/> definition — the runtime
    /// archetype the bakers produce. The affected body is a dynamic circle. The first <c>group.Update()</c>
    /// creates the bodies (no step); each later <c>group.Update()</c> applies the effector force pre-<c>Simulate</c>
    /// and steps once.
    /// </remarks>
    public sealed class ForceFieldEffectorSmoke
    {
        const float Dt = 1f / 60f;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("Physics2DEffectorSmokeWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }

        // A static effector entity: a sensor box/circle region (so a body overlaps without a collision response)
        // carrying a PhysicsEffector2D definition. The collider-only static body is the effector's own body.
        static Entity SpawnEffector(EntityManager em, float2 pos, PhysicsShape2D region, PhysicsEffector2D eff)
        {
            region.isTrigger = true; // the effector region is a sensor: overlaps, no collision response
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = pos,
                    useAutoMass = false,
                },
                region
            );
            em.AddComponentData(entity, eff);
            return entity;
        }

        // A dynamic affected circle. gravityScale is caller-chosen (0 for a clean force read, 1 for the
        // buoyancy drop). density 1 → a unit-density body.
        static Entity SpawnAffected(EntityManager em, float2 pos, float radius, float gravityScale)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
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
                }
            );
        }

        static PhysicsBody BodyOf(EntityManager em, Entity e) => em.GetComponentData<PhysicsBody2D>(e).body;

        [UnityTest]
        public IEnumerator Area_BodyInsideZone_AcceleratesInForceDirection()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A large gravity-free zone at the origin, pushing +X with magnitude 50.
            SpawnEffector(
                em,
                new float2(0f, 0f),
                new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(20f, 20f) },
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Area,
                    colliderMask = 0ul, // hit everything
                    forceMagnitude = 50f,
                    forceAngleRadians = 0f, // +X
                    useGlobalAngle = 1,
                    forceTargetIsRigidbody = 1,
                }
            );
            // A gravity-free dynamic body at rest inside the zone.
            var body = SpawnAffected(em, new float2(0f, 0f), 0.5f, gravityScale: 0f);

            group.Update(); // create (no step)
            var b = BodyOf(em, body);
            Assert.IsTrue(b.isValid, "Affected body was not created.");
            var v0 = (float2)(Vector2)b.linearVelocity;
            Assert.Less(length(v0), 1e-4f, $"Body was not at rest before the zone force (v0={v0}).");

            // Step several frames; the zone should accelerate the body in +X.
            for (var i = 0; i < 10; i++)
                group.Update();

            var v = (float2)(Vector2)b.linearVelocity;
            var p = (float2)(Vector2)b.position;
            Assert.Greater(v.x, 0.5f, $"Body did not accelerate in +X under the zone force (v={v}).");
            Assert.Less(abs(v.y), 0.1f, $"Body picked up unexpected Y velocity (v={v}).");
            Assert.Greater(p.x, 0.01f, $"Body did not move in +X (p={p}).");

            Debug.Log(
                $"[PHYSICS2D-EFFECTOR-AREA] zone +X mag=50 → after 10 steps v={v}, p={p} "
                    + "(accelerated in the force direction)."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator Buoyancy_DroppedBody_FloatsUpToRestNearSurface()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A fluid volume centred at the origin (10x10), surface at y=0, density 2, heavy drag so a dropped
            // unit-density body decelerates and settles half-submerged near the surface rather than bobbing.
            const float surfaceLevel = 0f;
            SpawnEffector(
                em,
                new float2(0f, 0f),
                new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(10f, 10f) },
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Buoyancy,
                    colliderMask = 0ul,
                    surfaceLevel = surfaceLevel,
                    fluidDensity = 2f,
                    linearDamping = 5f,
                    angularDamping = 5f,
                    gravityMagnitude = 9.81f,
                }
            );
            // A unit-density body dropped from just below the surface so it is in the fluid from the start.
            var body = SpawnAffected(em, new float2(0f, -1f), 0.5f, gravityScale: 1f);

            group.Update(); // create (no step)
            var b = BodyOf(em, body);
            Assert.IsTrue(b.isValid, "Affected body was not created.");

            // Track the lowest point reached and the final resting position over a long settle.
            var minY = 0f;
            var lastY = 0f;
            for (var i = 0; i < 400; i++)
            {
                group.Update();
                lastY = ((Vector2)b.position).y;
                if (lastY < minY)
                    minY = lastY;
            }

            // It sank somewhat (dropped in) but did NOT fall out the bottom of the 10-tall volume (bottom y=-5):
            // buoyancy + drag arrested the descent.
            Assert.Greater(minY, -4f, $"Body sank out of the fluid volume (minY={minY}) — no buoyancy arrest.");
            // It came to rest near the surface (a unit body in density-2 fluid floats ~half-submerged, body
            // centre near surfaceLevel within the body radius + a generous band for the v2-vs-v3 solver).
            Assert.Less(
                abs(lastY - surfaceLevel),
                1.0f,
                $"Body did not settle near the surface (lastY={lastY}, surface={surfaceLevel})."
            );

            Debug.Log(
                $"[PHYSICS2D-EFFECTOR-BUOYANCY] dropped at y=-1 in density-2 fluid (surface y=0): "
                    + $"minY={minY:F3} (arrested), rest y={lastY:F3} (floated up near the surface)."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator Point_BodyNearPoint_AcceleratesTowardPoint()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // An attracting point at the origin (negative magnitude = attract), Constant mode, large region.
            SpawnEffector(
                em,
                new float2(0f, 0f),
                new PhysicsShape2D { kind = PhysicsShape2DKind.Circle, radius = 20f },
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Point,
                    colliderMask = 0ul,
                    forceMagnitude = -200f, // attract
                    distanceScale = 1f,
                    forceMode = 0, // Constant
                    forceSourceIsRigidbody = 0,
                }
            );
            // A gravity-free dynamic body off to the +X side of the point.
            var body = SpawnAffected(em, new float2(5f, 0f), 0.5f, gravityScale: 0f);

            group.Update(); // create (no step)
            var b = BodyOf(em, body);
            Assert.IsTrue(b.isValid, "Affected body was not created.");
            var x0 = ((Vector2)b.position).x;

            for (var i = 0; i < 10; i++)
                group.Update();

            var v = (float2)(Vector2)b.linearVelocity;
            var x1 = ((Vector2)b.position).x;
            // Starting at +X, attraction toward the origin means a NEGATIVE X velocity and the body moving in -X.
            Assert.Less(v.x, -0.5f, $"Body did not accelerate toward the point (v={v}).");
            Assert.Less(x1, x0, $"Body did not move toward the point (x0={x0}, x1={x1}).");

            Debug.Log(
                $"[PHYSICS2D-EFFECTOR-POINT] attract mag=-200 from x=5 → after 10 steps v={v}, x={x1:F3} "
                    + $"(moved toward the point from x={x0:F3})."
            );

            world.Dispose();
            yield break;
        }
    }
}
