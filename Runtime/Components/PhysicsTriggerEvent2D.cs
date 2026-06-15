using Unity.Entities;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// One trigger (overlap, no collision response) begin/end event from the most recent simulation step, as the
    /// package's analogue of <c>OnTriggerEnter2D</c> / <c>OnTriggerExit2D</c>. A trigger event is produced when
    /// a pair where exactly one shape is a sensor (<c>Collider2D.isTrigger</c> baked to
    /// <c>PhysicsShapeDefinition.isTrigger</c>) begins or stops overlapping. Lives in a
    /// <c>DynamicBuffer&lt;PhysicsTriggerEvent2D&gt;</c> on the <see cref="PhysicsWorldSingleton2D"/> entity,
    /// cleared and refilled by <c>PhysicsWorld2DSystem</c> each step.
    /// </summary>
    /// <remarks>
    /// Same validity window and Begin/End/Stay semantics as <see cref="PhysicsContactEvent2D"/> (read after
    /// <c>PhysicsWorld2DSystem</c> in the fixed group; Stay is the begin..end interval the consumer derives).
    /// The trigger and visitor are named after Box2D's terminology: <see cref="triggerEntity"/> owns the sensor
    /// shape, <see cref="visitorEntity"/> owns the shape that entered/left it. Two overlapping sensors DO produce
    /// trigger events — one per sensor's perspective (the symmetric trigger/visitor reports), matching GameObject
    /// 2D physics, which raises <c>OnTriggerEnter2D</c> on both trigger colliders. (The Box2D "triggers do not
    /// collide with other triggers" rule governs collision RESPONSE — two sensors never produce a solid contact —
    /// not trigger-event reporting; with trigger events enabled on every shape, a sensor pair fires begin/end
    /// just like a sensor/solid pair.)
    /// </remarks>
    public struct PhysicsTriggerEvent2D : IBufferElementData
    {
        /// <summary>Begin (the pair started overlapping) or End (stopped overlapping).</summary>
        public PhysicsEventPhase2D phase;

        /// <summary>The owning entity of the sensor (<see cref="triggerShape"/>), or
        /// <see cref="Entity.Null"/> if unresolved.</summary>
        public Entity triggerEntity;

        /// <summary>The owning entity of the shape that entered/left the sensor (<see cref="visitorShape"/>), or
        /// <see cref="Entity.Null"/> if unresolved.</summary>
        public Entity visitorEntity;

        /// <summary>The sensor shape. Valid only this frame.</summary>
        public PhysicsShape triggerShape;

        /// <summary>The shape that entered/left the sensor. Valid only this frame.</summary>
        public PhysicsShape visitorShape;
    }
}
