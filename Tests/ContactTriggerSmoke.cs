using System.Collections;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-6 smoke for contact / trigger events. Two minimal mechanism witnesses (the hard GameObject-parity
    /// e2e gate is a separate validating agent's deliverable):
    /// <list type="bullet">
    /// <item><b>Contact</b> — a dynamic body landing on a static floor produces a contact-Begin for that entity
    /// pair (and no trigger event). Proves <c>contactEvents</c> + <c>startStaticContacts</c> at creation, the
    /// post-step span read into the buffer, and the pair→entity resolution.</item>
    /// <item><b>Trigger</b> — a dynamic body falling through a static sensor produces one trigger-Begin then one
    /// trigger-End for that pair (and no contact event between them). Proves the <c>isTrigger</c> bake (the body
    /// passes through, no collision response) and the trigger-event span read.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/> holding the package's four FixedStep systems,
    /// driven one fixed step per <c>group.Update()</c>. Bodies are authored directly via
    /// <see cref="DirectPhysics2DAuthoring"/>. Events are accumulated across the settle window by reading the
    /// singleton's <c>DynamicBuffer&lt;PhysicsContactEvent2D&gt;</c> / <c>PhysicsTriggerEvent2D</c> after each
    /// <c>group.Update()</c> (the producer fills them in <c>PhysicsWorld2DSystem</c> within that tick).
    /// </remarks>
    public sealed class ContactTriggerSmoke
    {
        const float Dt = 1f / 60f;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("Physics2DContactTriggerSmokeWorld", out group, Dt);

        static Entity SpawnDynamicCircle(EntityManager em, float2 pos, float radius)
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
                    // bounciness 0 → a single clean landing (no bounce/re-touch cycles).
                }
            );
        }

        static Entity SpawnStaticBox(EntityManager em, float2 center, float2 size, bool isTrigger)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition { bodyType = PhysicsBody.BodyType.Static, initialPosition = center },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = size,
                    radius = 0f,
                    density = 1f,
                    friction = 0.4f,
                    isTrigger = isTrigger,
                }
            );
        }

        // True if either ordering of (a,b) appears as the pair in a contact event of the given phase.
        static bool HasContact(DynamicBuffer<PhysicsContactEvent2D> buf, Entity a, Entity b, PhysicsEventPhase2D phase)
        {
            for (var i = 0; i < buf.Length; i++)
            {
                var e = buf[i];
                if (e.phase != phase)
                    continue;
                if ((e.entityA == a && e.entityB == b) || (e.entityA == b && e.entityB == a))
                    return true;
            }
            return false;
        }

        static bool HasTrigger(
            DynamicBuffer<PhysicsTriggerEvent2D> buf,
            Entity trigger,
            Entity visitor,
            PhysicsEventPhase2D phase
        )
        {
            for (var i = 0; i < buf.Length; i++)
            {
                var e = buf[i];
                if (e.phase != phase)
                    continue;
                // Trigger/visitor roles are fixed (sensor vs solid), but accept either assignment defensively.
                if (
                    (e.triggerEntity == trigger && e.visitorEntity == visitor)
                    || (e.triggerEntity == visitor && e.visitorEntity == trigger)
                )
                    return true;
            }
            return false;
        }

        [UnityTest]
        public IEnumerator BodyLandingOnFloor_ProducesContactBeginForThatPair()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A static floor (top surface at y=0: box centered at y=-0.5, height 1) and a dynamic circle dropped
            // from y=4 that lands and rests on it.
            var floor = SpawnStaticBox(em, new float2(0f, -0.5f), new float2(6f, 1f), isTrigger: false);
            var body = SpawnDynamicCircle(em, new float2(0f, 4f), 0.5f);

            var contactBegin = false;
            var anyTrigger = false;

            // First update creates the bodies (no step, no events). Then step until the body lands; accumulate
            // whether the body↔floor contact-begin ever fired and whether any trigger event spuriously appeared.
            group.Update();
            for (var f = 0; f < 240; f++)
            {
                group.Update();
                var cBuf = em.CreateEntityQuery(typeof(PhysicsWorldSingleton2D)).GetSingletonEntity();
                var contacts = em.GetBuffer<PhysicsContactEvent2D>(cBuf, isReadOnly: true);
                var triggers = em.GetBuffer<PhysicsTriggerEvent2D>(cBuf, isReadOnly: true);
                if (HasContact(contacts, body, floor, PhysicsEventPhase2D.Begin))
                    contactBegin = true;
                if (triggers.Length > 0)
                    anyTrigger = true;
            }

            Assert.IsTrue(
                contactBegin,
                "A dynamic body landing on a static floor produced no contact-Begin event for that entity pair. "
                    + "Either contactEvents was not enabled, startStaticContacts was not set on the static floor "
                    + "(so the floor created no contact), or the shape→entity resolution is wrong."
            );
            Assert.IsFalse(
                anyTrigger,
                "A non-trigger contact spuriously produced a trigger event — isTrigger leaked onto a solid shape."
            );

            Debug.Log(
                $"[PHYSICS2D-CONTACT] body={body} landed on floor={floor}; contact-Begin observed for the pair, "
                    + $"no trigger events. final bodyY={em.GetComponentData<Unity.Transforms.LocalToWorld>(body).Position.y:F3}."
            );

            world.Dispose();
            yield break;
        }

        [UnityTest]
        public IEnumerator BodyPassingThroughTrigger_ProducesTriggerBeginThenEnd()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // A static SENSOR box centered at y=0 (a trigger: no collision response) and a dynamic circle dropped
            // from above that falls THROUGH it — entering (trigger-Begin) then leaving (trigger-End).
            var sensor = SpawnStaticBox(em, new float2(0f, 0f), new float2(4f, 2f), isTrigger: true);
            var body = SpawnDynamicCircle(em, new float2(0f, 6f), 0.5f);

            var triggerBegin = false;
            var triggerEnd = false;
            var anyContact = false;
            var fellThrough = false;

            group.Update();
            for (var f = 0; f < 300; f++)
            {
                group.Update();
                var se = em.CreateEntityQuery(typeof(PhysicsWorldSingleton2D)).GetSingletonEntity();
                var triggers = em.GetBuffer<PhysicsTriggerEvent2D>(se, isReadOnly: true);
                var contacts = em.GetBuffer<PhysicsContactEvent2D>(se, isReadOnly: true);
                if (HasTrigger(triggers, sensor, body, PhysicsEventPhase2D.Begin))
                    triggerBegin = true;
                if (HasTrigger(triggers, sensor, body, PhysicsEventPhase2D.End))
                    triggerEnd = true;
                if (HasContact(contacts, sensor, body, PhysicsEventPhase2D.Begin))
                    anyContact = true;
                var y = em.GetComponentData<Unity.Transforms.LocalToWorld>(body).Position.y;
                if (y < -5f)
                    fellThrough = true;
            }

            // The body must actually pass through the sensor (no collision response) — that is the isTrigger
            // proof. If isTrigger had not been applied, the body would rest on the sensor and never fall below.
            Assert.IsTrue(
                fellThrough,
                "The body did not pass through the sensor (it never fell below y=-5). The isTrigger bake did not "
                    + "take effect — the sensor behaved as a solid floor."
            );
            Assert.IsTrue(
                triggerBegin,
                "No trigger-Begin event fired for the sensor↔body pair as the body entered the sensor."
            );
            Assert.IsTrue(
                triggerEnd,
                "No trigger-End event fired for the sensor↔body pair as the body left the sensor."
            );
            Assert.IsFalse(
                anyContact,
                "A trigger overlap spuriously produced a contact (collision) event — a sensor must not collide."
            );

            Debug.Log(
                $"[PHYSICS2D-TRIGGER] body={body} passed through sensor={sensor}; trigger-Begin and trigger-End "
                    + "both observed for the pair, no contact events."
            );

            world.Dispose();
            yield break;
        }
    }
}
