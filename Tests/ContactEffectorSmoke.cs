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
    /// Phase-10b smoke for the two contact-response effectors (Platform / Surface). Two minimal mechanism
    /// witnesses (the hard GameObject-parity e2e gate is a separate validating agent's deliverable):
    /// <list type="bullet">
    /// <item><b>Platform</b> — a body dropped from ABOVE onto a one-way platform rests on it, while a body launched
    /// UP from below passes THROUGH it.</item>
    /// <item><b>Surface</b> — a box dropped on a conveyor belt accelerates tangentially to the belt speed and rides
    /// along (its tangential velocity converges to the belt speed without overshoot).</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/> holding the package's four FixedStep systems,
    /// driven one fixed step per <c>group.Update()</c>. Unlike the Phase-10a force-field effectors, the
    /// platform/surface collider is SOLID (not a sensor) — a body rests on it. The first <c>group.Update()</c>
    /// creates the bodies (no step); each later <c>group.Update()</c> applies the effector pre-<c>Simulate</c> and
    /// steps once.
    /// </remarks>
    public sealed class ContactEffectorSmoke
    {
        const float Dt = 1f / 60f;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DContactEffectorSmokeWorld", out group, Dt);

        // A static effector entity with a SOLID (non-trigger) collider — bodies rest ON it (the Phase-10b
        // inversion from the sensor force-field effectors). Carries a PhysicsEffector2D definition.
        static Entity SpawnSolidEffector(EntityManager em, float2 pos, PhysicsShape2D region, PhysicsEffector2D eff)
        {
            region.isTrigger = false; // SOLID: a body rests on / rides the platform/belt
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

        // A dynamic affected box.
        static Entity SpawnBox(EntityManager em, float2 pos, float2 size, float gravityScale, float2 initialVel)
        {
            var entity = DirectPhysics2DAuthoring.Create(
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
                    kind = PhysicsShape2DKind.Box,
                    size = size,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            if (!initialVel.Equals(Unity.Mathematics.float2.zero))
                em.AddComponentData(entity, new PhysicsBody2DInitialVelocity { linearVelocity = initialVel });
            return entity;
        }

        static PhysicsBody BodyOf(EntityManager em, Entity e) => em.GetComponentData<PhysicsBody2D>(e).body;

        [UnityTest]
        public IEnumerator Platform_RestsFromAbove_PassesFromBelow()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A one-way platform: a thin solid box at the origin, surfaceArc 180° (top-facing), rotationalOffset 0.
            SpawnSolidEffector(
                em,
                new float2(0f, 0f),
                new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(6f, 0.4f) },
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Platform,
                    colliderMask = 0ul, // every layer
                    surfaceArcRadians = radians(180f),
                    rotationalOffsetRadians = 0f,
                    useOneWay = 1,
                }
            );

            // (1) A body dropped from ABOVE should COME TO REST on the platform (top surface near y = +0.2).
            var above = SpawnBox(
                em,
                new float2(-1.5f, 3f),
                new float2(0.5f, 0.5f),
                gravityScale: 1f,
                Unity.Mathematics.float2.zero
            );

            group.Update(); // create (no step)
            var aboveBody = BodyOf(em, above);
            Assert.IsTrue(aboveBody.isValid, "Above body was not created.");

            var aboveMinY = 3f;
            for (var i = 0; i < 240; i++)
            {
                group.Update();
                var y = ((Vector2)aboveBody.position).y;
                if (y < aboveMinY)
                    aboveMinY = y;
            }
            var aboveRestY = ((Vector2)aboveBody.position).y;
            // It rested ON TOP of the platform: did not fall far below the platform top (~0.2 + half box 0.25).
            Assert.Greater(
                aboveRestY,
                0f,
                $"Body dropped from above did not rest on the one-way platform (restY={aboveRestY}, minY={aboveMinY})."
            );

            // (2) A SEPARATE world: a body launched UP from BELOW should PASS THROUGH the platform.
            var world2 = MakePhysicsWorld(out var group2);
            var em2 = world2.EntityManager;
            SpawnSolidEffector(
                em2,
                new float2(0f, 0f),
                new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(6f, 0.4f) },
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Platform,
                    colliderMask = 0ul,
                    surfaceArcRadians = radians(180f),
                    rotationalOffsetRadians = 0f,
                    useOneWay = 1,
                }
            );
            // Below the platform, launched upward fast, gravity off so it keeps rising through the platform.
            var below = SpawnBox(
                em2,
                new float2(0f, -2f),
                new float2(0.5f, 0.5f),
                gravityScale: 0f,
                new float2(0f, 20f)
            );

            group2.Update(); // create (no step)
            var belowBody = BodyOf(em2, below);
            Assert.IsTrue(belowBody.isValid, "Below body was not created.");

            for (var i = 0; i < 30; i++)
                group2.Update();
            var belowY = ((Vector2)belowBody.position).y;
            // It passed THROUGH: it is now well ABOVE the platform (a solid platform would have stopped it at ~ -0.45).
            Assert.Greater(
                belowY,
                1f,
                $"Body launched from below did not pass through the one-way platform (y={belowY})."
            );

            Debug.Log(
                $"[PHYSICS2D-EFFECTOR-PLATFORM] one-way surfaceArc=180: body from above rested at y={aboveRestY:F3} "
                    + $"(minY={aboveMinY:F3}); body launched from below passed through to y={belowY:F3}."
            );

            world.Dispose();
            world2.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator Surface_BoxOnBelt_AcceleratesToBeltSpeed()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A horizontal conveyor belt: a wide thin solid box at the origin, driving +X at speed 5, forceScale 1.
            const float beltSpeed = 5f;
            SpawnSolidEffector(
                em,
                new float2(0f, 0f),
                new PhysicsShape2D { kind = PhysicsShape2DKind.Box, size = new float2(20f, 0.4f) },
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Surface,
                    colliderMask = 0ul,
                    surfaceSpeed = beltSpeed,
                    forceScale = 1f,
                    useContactForce = 0,
                    surfaceUseFriction = 1,
                }
            );
            // A box dropped onto the belt (top surface ~ y=0.2), gravity on so it lands and stays in contact.
            var box = SpawnBox(
                em,
                new float2(0f, 1f),
                new float2(0.5f, 0.5f),
                gravityScale: 1f,
                Unity.Mathematics.float2.zero
            );

            group.Update(); // create (no step)
            var b = BodyOf(em, box);
            Assert.IsTrue(b.isValid, "Box was not created.");

            // Let it land and ride.
            for (var i = 0; i < 120; i++)
                group.Update();

            var v = (float2)(Vector2)b.linearVelocity;
            var x = ((Vector2)b.position).x;
            // Tangential (X) velocity converged to the belt speed (within a band; no overshoot above it), and the
            // box moved +X along the belt.
            Assert.Greater(v.x, beltSpeed * 0.5f, $"Box did not accelerate toward the belt speed (v={v}).");
            Assert.Less(v.x, beltSpeed * 1.2f, $"Box overshot the belt speed (v={v}).");
            Assert.Greater(x, 0.5f, $"Box did not ride +X along the belt (x={x}).");

            Debug.Log(
                $"[PHYSICS2D-EFFECTOR-SURFACE] belt speed={beltSpeed} → after landing+riding v={v} (converged to "
                    + $"the belt speed, no overshoot), x={x:F3} (rode +X)."
            );

            world.Dispose();
            yield break;
        }
    }
}
