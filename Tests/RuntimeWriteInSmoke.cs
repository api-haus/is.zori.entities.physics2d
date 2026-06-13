using System.Collections;
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
    /// Phase-7 smoke for the runtime write-in surface (<see cref="PhysicsBody2DCommands"/>). Two minimal
    /// mechanism witnesses (the hard GameObject-parity e2e gate is a separate validating agent's deliverable):
    /// <list type="bullet">
    /// <item><b>Impulse</b> — an applied <c>Impulse</c> command on a gravity-free body changes its linear
    /// velocity by the expected Δv = J/m this step (mass-scaled by the body's own mass), and the buffer is
    /// cleared so the impulse is one-shot (the next step adds no further Δv). Proves the command drains onto the
    /// body before <c>Simulate</c>, the Box2D <c>ApplyLinearImpulseToCenter</c> mass-scaling, and the clear.</item>
    /// <item><b>MovePosition</b> — a kinematic <c>MovePosition(target)</c> command lands the body at the target
    /// next step. Proves the <c>SetTransformTarget</c> kinematic sweep reaches the target over the step.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/> holding the package's four FixedStep systems,
    /// driven one fixed step per <c>group.Update()</c>. Bodies are authored directly via
    /// <see cref="DirectPhysics2DAuthoring"/>; the command buffer is added explicitly and filled through
    /// <see cref="PhysicsBody2DCommands"/>. The first <c>group.Update()</c> creates the body (no step); each later
    /// <c>group.Update()</c> drains the buffer onto the body and steps once.
    /// </remarks>
    public sealed class RuntimeWriteInSmoke
    {
        const float Dt = 1f / 60f;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DWriteInSmokeWorld", out group, Dt);

        // A dynamic circle authored directly, with the runtime command buffer added so the user can drive it.
        // gravityScale 0 so gravity does not contaminate the measured velocity delta.
        static Entity SpawnDrivable(EntityManager em, float2 pos, float radius)
        {
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f,
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
            em.AddBuffer<PhysicsBody2DCommand>(entity);
            return entity;
        }

        static PhysicsBody BodyOf(EntityManager em, Entity e)
        {
            return em.GetComponentData<PhysicsBody2D>(e).body;
        }

        static DynamicBuffer<PhysicsBody2DCommand> CommandsOf(EntityManager em, Entity e)
        {
            return em.GetBuffer<PhysicsBody2DCommand>(e);
        }

        [UnityTest]
        public IEnumerator AppliedImpulse_ChangesVelocityByExpectedDelta_AndIsOneShot()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            var entity = SpawnDrivable(em, new float2(0f, 0f), 0.5f);

            // First update creates the body (no step). The Box2D mass is now resolved from the circle's density.
            group.Update();

            var body = BodyOf(em, entity);
            Assert.IsTrue(body.isValid, "Body was not created on the first update.");
            var mass = body.mass;
            Assert.Greater(mass, 0f, "Dynamic body has non-positive mass — density did not produce auto-mass.");

            // The body starts at rest (no initial velocity, no gravity).
            var v0 = (float2)(Vector2)body.linearVelocity;
            Assert.Less(length(v0), 1e-4f, $"Body was not at rest before the impulse (v0={v0}).");

            // Append one impulse this frame. The expected instantaneous velocity change is J/m.
            var impulse = new float2(10f, 0f);
            PhysicsBody2DCommands.AddForce(CommandsOf(em, entity), impulse, PhysicsForceMode2D.Impulse);

            // This update drains the command onto the body and steps once.
            group.Update();

            var v1 = (float2)(Vector2)body.linearVelocity;
            var expected = impulse / mass;
            Assert.Less(
                length(v1 - expected),
                0.1f * length(expected) + 1e-3f,
                $"Applied impulse did not produce the expected velocity change. v1={v1}, expected≈J/m={expected} "
                    + $"(J={impulse}, m={mass:F4}). Either the command did not drain onto the body before Simulate, "
                    + "or ApplyLinearImpulseToCenter did not mass-scale."
            );

            // The buffer must have been cleared — the next step (no new command) adds no further velocity.
            Assert.AreEqual(0, CommandsOf(em, entity).Length, "Command buffer was not cleared after apply.");
            group.Update();
            var v2 = (float2)(Vector2)body.linearVelocity;
            Assert.Less(
                length(v2 - v1),
                1e-3f,
                $"Velocity changed on a step with no command (v1={v1} → v2={v2}) — the impulse was not one-shot."
            );

            Debug.Log(
                $"[PHYSICS2D-WRITEIN] impulse J={impulse} on m={mass:F4} → v={v1} (expected≈{expected}); "
                    + $"one-shot: next step v={v2} unchanged."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator KinematicMovePosition_LandsBodyAtTargetNextStep()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A kinematic body (no gravity influence, moved only by MovePosition) starting at the origin.
            var entity = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Kinematic,
                    initialPosition = new float2(0f, 0f),
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddBuffer<PhysicsBody2DCommand>(entity);

            // First update creates the body (no step).
            group.Update();
            var body = BodyOf(em, entity);
            Assert.IsTrue(body.isValid, "Body was not created on the first update.");

            // Command a sweep to a target, then step once: the body should reach the target this step.
            var target = new float2(3f, -2f);
            PhysicsBody2DCommands.MovePosition(CommandsOf(em, entity), target);
            group.Update();

            var landed = (float2)(Vector2)body.position;
            Assert.Less(
                length(landed - target),
                0.05f,
                $"MovePosition did not land the body at the target next step. landed={landed}, target={target}. "
                    + "SetTransformTarget did not sweep the body to the target over the step."
            );

            // The LocalToWorld write-back reflects the same landed pose.
            var ltwPos = em.GetComponentData<LocalToWorld>(entity).Position;
            Assert.Less(
                length(new float2(ltwPos.x, ltwPos.y) - landed),
                1e-3f,
                $"LocalToWorld ({ltwPos.x:F3},{ltwPos.y:F3}) did not match the body pose ({landed})."
            );

            Debug.Log($"[PHYSICS2D-WRITEIN] MovePosition target={target} → landed={landed} (LocalToWorld agrees).");

            world.Dispose();
            yield break;
        }
    }
}
