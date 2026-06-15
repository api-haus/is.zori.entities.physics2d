using Unity.Entities;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The phase of a touch event. Box2D-v3 reports a pair beginning to touch (<see cref="Begin"/>) and ending
    /// (<see cref="End"/>) as two distinct event lists; there is no per-frame "still touching" event. The
    /// GameObject <c>OnCollisionStay2D</c>/<c>OnTriggerStay2D</c> "currently touching" set is the begin..end
    /// interval a consumer derives by applying <see cref="Begin"/> (insert the pair) and <see cref="End"/>
    /// (remove the pair) — the package does not synthesise a Stay element per frame.
    /// </summary>
    public enum PhysicsEventPhase2D : byte
    {
        /// <summary>The pair began touching this step — the analogue of <c>OnCollisionEnter2D</c> /
        /// <c>OnTriggerEnter2D</c>.</summary>
        Begin,

        /// <summary>The pair stopped touching this step — the analogue of <c>OnCollisionExit2D</c> /
        /// <c>OnTriggerExit2D</c>.</summary>
        End,
    }

    /// <summary>
    /// One contact (non-trigger) begin/end event from the most recent simulation step, as the package's
    /// analogue of <c>OnCollisionEnter2D</c> / <c>OnCollisionExit2D</c>. Both shapes are non-triggers (a pair
    /// where one is a trigger produces a <see cref="PhysicsTriggerEvent2D"/> instead). Lives in a
    /// <c>DynamicBuffer&lt;PhysicsContactEvent2D&gt;</c> on the <see cref="PhysicsWorldSingleton2D"/> entity,
    /// cleared and refilled by <c>PhysicsWorld2DSystem</c> each step.
    /// </summary>
    /// <remarks>
    /// <b>Validity window.</b> The buffer holds the events of the most recent <c>Simulate</c>. It is valid for
    /// any system that runs after <c>PhysicsWorld2DSystem</c> within the same
    /// <c>FixedStepSimulationSystemGroup</c> tick (and before the next tick clears it). Read it with
    /// <c>SystemAPI.GetSingletonBuffer&lt;PhysicsContactEvent2D&gt;(isReadOnly: true)</c> from a system ordered
    /// <c>[UpdateAfter(typeof(PhysicsWorld2DSystem))]</c> in that group.
    ///
    /// <b>What is and is not here.</b> Box2D-v3's contact begin/end events carry only the shape pair (and a
    /// volatile contact id), NOT the contact point / normal / relative velocity — that geometry lives on a
    /// separate threshold-gated hit-event channel (<c>PhysicsEvents.ContactHitEvent</c>), not bound here. So
    /// this is a touch signal: which pair of entities, beginning or ending, this step.
    ///
    /// <b>Entities vs shapes.</b> <see cref="entityA"/>/<see cref="entityB"/> are the stable owning entities
    /// (resolved via the body's packed userData) and are safe to keep. The raw
    /// <see cref="shapeA"/>/<see cref="shapeB"/> handles are a same-frame convenience (read <c>shapeType</c>
    /// etc.) and are volatile beyond this frame — a shape may be destroyed next step. An entity that could not
    /// be resolved (a shape destroyed since the step, or a body the package did not pack) is
    /// <see cref="Entity.Null"/>.
    /// </remarks>
    public struct PhysicsContactEvent2D : IBufferElementData
    {
        /// <summary>Begin (the pair started touching) or End (stopped touching).</summary>
        public PhysicsEventPhase2D phase;

        /// <summary>The owning entity of <see cref="shapeA"/>, or <see cref="Entity.Null"/> if unresolved.</summary>
        public Entity entityA;

        /// <summary>The owning entity of <see cref="shapeB"/>, or <see cref="Entity.Null"/> if unresolved.</summary>
        public Entity entityB;

        /// <summary>One of the two shapes in the contact. Valid only this frame.</summary>
        public PhysicsShape shapeA;

        /// <summary>The other shape in the contact. Valid only this frame.</summary>
        public PhysicsShape shapeB;
    }
}
