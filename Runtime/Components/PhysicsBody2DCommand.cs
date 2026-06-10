using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// How a force / torque write-in is applied, mirroring <c>UnityEngine.ForceMode2D</c>: <see cref="Force"/>
    /// integrates a continuous force over the step (mass-scaled), <see cref="Impulse"/> applies an instantaneous
    /// velocity change (mass-scaled). A package-local enum rather than a reference to the built-in
    /// <c>ForceMode2D</c>, so the renderer-agnostic runtime surface carries no built-in-authoring dependency.
    /// </summary>
    public enum PhysicsForceMode2D : byte
    {
        /// <summary>A continuous force integrated over the step using the body's mass — Δv = (F/m)·dt.</summary>
        Force,

        /// <summary>An instantaneous velocity change using the body's mass — Δv = J/m.</summary>
        Impulse,
    }

    /// <summary>
    /// The kind of write-in a <see cref="PhysicsBody2DCommand"/> carries, naming the Box2D call it maps to.
    /// </summary>
    public enum PhysicsBody2DCommandKind : byte
    {
        /// <summary><c>AddForce(f, Force)</c> → <c>PhysicsBody.ApplyForceToCenter</c> (force in N, step-integrated).</summary>
        Force,

        /// <summary><c>AddForceAtPosition(f, p, Force)</c> → <c>PhysicsBody.ApplyForce</c> (force at a world point → linear + torque).</summary>
        ForceAtPosition,

        /// <summary><c>AddForce(f, Impulse)</c> → <c>PhysicsBody.ApplyLinearImpulseToCenter</c> (impulse in N·s, instantaneous Δv).</summary>
        Impulse,

        /// <summary><c>AddForceAtPosition(f, p, Impulse)</c> → <c>PhysicsBody.ApplyLinearImpulse</c> (impulse at a world point → Δv + Δω).</summary>
        ImpulseAtPosition,

        /// <summary><c>AddTorque(t, Force)</c> → <c>PhysicsBody.ApplyTorque</c> (N·m, step-integrated about the centre of mass).</summary>
        Torque,

        /// <summary><c>AddTorque(t, Impulse)</c> → <c>PhysicsBody.ApplyAngularImpulse</c> (instantaneous Δω).</summary>
        AngularImpulse,

        /// <summary><c>linearVelocity = v</c> → a direct <c>PhysicsBody.linearVelocity</c> set (m/s), waking the body.</summary>
        SetLinearVelocity,

        /// <summary><c>angularVelocity = w</c> → a direct <c>PhysicsBody.angularVelocity</c> set (deg/sec), waking the body.</summary>
        SetAngularVelocity,

        /// <summary><c>MovePosition(target)</c> → <c>PhysicsBody.SetTransformTarget</c> at (target position, current rotation).</summary>
        MovePosition,

        /// <summary><c>MoveRotation(target)</c> → <c>PhysicsBody.SetTransformTarget</c> at (current position, target rotation).</summary>
        MoveRotation,

        /// <summary><c>MovePositionAndRotation(p, r)</c> → <c>PhysicsBody.SetTransformTarget</c> at (target position, target rotation).</summary>
        MovePositionAndRotation,
    }

    /// <summary>
    /// One runtime write-in command for a body, drained onto the body's <c>PhysicsBody</c> before the next
    /// <c>Simulate</c> and then cleared. A <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> on a body's entity is
    /// the per-entity write-in queue: a user system appends commands each frame (via
    /// <see cref="PhysicsBody2DCommands"/>), and <c>PhysicsWorld2DSystem</c> applies them in order to the body
    /// immediately before the step, then clears the buffer — so each command applies exactly once at one step,
    /// matching the per-<c>FixedUpdate</c> one-shot semantics of <c>Rigidbody2D.AddForce</c>/<c>MovePosition</c>.
    /// </summary>
    /// <remarks>
    /// The buffer is optional — only entities a user adds it to carry it, and the apply loop queries
    /// <c>WithAll&lt;PhysicsBody2D, PhysicsBody2DCommand&gt;</c>, so a body with no command buffer costs nothing.
    /// A buffer (not a single command component) is what lets multiple <c>AddForce</c>/<c>AddTorque</c> in one
    /// frame accumulate the way they do on a GameObject: the commands drain in order before one <c>Simulate</c>,
    /// and Box2D's own force accumulator sums the continuous-force commands exactly as the built-in solver does.
    ///
    /// All fields are blittable. The meaning of <see cref="linear"/>/<see cref="angular"/>/<see cref="worldPoint"/>
    /// depends on <see cref="kind"/> (see <see cref="PhysicsBody2DCommandKind"/>): <see cref="linear"/> is the
    /// force / impulse / target position / linear velocity; <see cref="angular"/> is the torque / angular impulse
    /// / target rotation (radians) / angular velocity (deg/sec); <see cref="worldPoint"/> is the world application
    /// point for the <c>*AtPosition</c> kinds and unused otherwise.
    /// </remarks>
    public struct PhysicsBody2DCommand : IBufferElementData
    {
        public PhysicsBody2DCommandKind kind;
        public float2 linear;
        public float angular;
        public float2 worldPoint;
    }
}
