using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The ECS-facing runtime write-in surface — the analogue of <c>Rigidbody2D.AddForce</c> /
    /// <c>AddForceAtPosition</c> / <c>AddTorque</c> / the <c>linearVelocity</c>/<c>angularVelocity</c> writes /
    /// <c>MovePosition</c> / <c>MoveRotation</c>. Each helper appends one <see cref="PhysicsBody2DCommand"/> to a
    /// body entity's <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c>; <c>PhysicsWorld2DSystem</c> drains the
    /// buffer onto the Box2D body immediately before the step and clears it, so a command authored this frame
    /// drives exactly one step — the same one-shot, per-step semantics as the GameObject calls.
    /// </summary>
    /// <remarks>
    /// <b>Usage.</b> Add the buffer once (<c>EntityManager.AddBuffer&lt;PhysicsBody2DCommand&gt;(entity)</c>), then
    /// each frame get it (<c>SystemAPI.GetBuffer&lt;PhysicsBody2DCommand&gt;(entity)</c>) and call these helpers.
    /// Multiple calls in one frame accumulate: continuous <see cref="AddForce(DynamicBuffer{PhysicsBody2DCommand},float2,PhysicsForceMode2D)"/>
    /// commands sum through Box2D's force accumulator at the step exactly as repeated <c>Rigidbody2D.AddForce</c>
    /// calls do within a <c>FixedUpdate</c>.
    ///
    /// <b>Mass / freeze.</b> A force/impulse is mass-scaled by the body's resolved mass/inertia inside Box2D (the
    /// helpers pass the raw vector, never pre-dividing by mass), and a write-in on a frozen DOF is cancelled by
    /// the solver, exactly as a GameObject <c>AddForce</c> on a frozen axis is ignored. Every force/impulse/move
    /// wakes a sleeping body, matching the GameObject behaviour.
    ///
    /// <b>Burst.</b> Plain <c>static</c> helpers (no <c>[BurstCompile]</c>, per the entry-point-only rule): they
    /// are HPC#-clean buffer appends, so a caller's own <c>[BurstCompile]</c> job that holds the buffer can call
    /// them and they auto-compile from that context, while a main-thread system calls them managed.
    ///
    /// <b>Angular units (explicit).</b> A ROTATION target is in <b>radians</b>
    /// (<see cref="MoveRotation"/>, <see cref="MovePositionAndRotation"/>): the engine's rotation type
    /// <c>PhysicsRotate</c> is unit-agnostic (it accepts either <c>FromRadians</c> or <c>FromDegrees</c>), so the
    /// package picks radians to match the Entities / <c>Unity.Mathematics</c> / <c>float4x4</c> ecosystem. An
    /// angular VELOCITY is in <b>degrees per second</b> (<see cref="SetAngularVelocity"/>): the engine's
    /// <c>PhysicsBody.angularVelocity</c> is documented deg/sec (module XML) and so is
    /// <c>Rigidbody2D.angularVelocity</c>, so the package complies rather than rebase a deg/sec engine onto
    /// rad/sec. The single canonical statement of this convention is the angular-unit section of
    /// <c>Documentation~/parity-matrix.md</c>.
    /// </remarks>
    public static class PhysicsBody2DCommands
    {
        /// <summary>Apply a continuous force (<see cref="PhysicsForceMode2D.Force"/>) at the body's centre of mass —
        /// the <c>AddForce(f, ForceMode2D.Force)</c> default.</summary>
        public static void AddForce(DynamicBuffer<PhysicsBody2DCommand> buffer, float2 force)
        {
            AddForce(buffer, force, PhysicsForceMode2D.Force);
        }

        /// <summary>Apply a force at the body's centre of mass in the given mode —
        /// <c>AddForce(f, ForceMode2D.Force|Impulse)</c>.</summary>
        public static void AddForce(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float2 force,
            PhysicsForceMode2D mode
        )
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind =
                        mode == PhysicsForceMode2D.Impulse
                            ? PhysicsBody2DCommandKind.Impulse
                            : PhysicsBody2DCommandKind.Force,
                    linear = force,
                }
            );
        }

        /// <summary>Apply a force at a world point (generating a linear + angular effect when off the centre of
        /// mass) — <c>AddForceAtPosition(f, p, mode)</c>.</summary>
        public static void AddForceAtPosition(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float2 force,
            float2 worldPoint,
            PhysicsForceMode2D mode
        )
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind =
                        mode == PhysicsForceMode2D.Impulse
                            ? PhysicsBody2DCommandKind.ImpulseAtPosition
                            : PhysicsBody2DCommandKind.ForceAtPosition,
                    linear = force,
                    worldPoint = worldPoint,
                }
            );
        }

        /// <summary>Apply a torque about the body's centre of mass — <c>AddTorque(t, mode)</c>. The
        /// <paramref name="torque"/> is a torque in N·m (Force mode) or an angular impulse (Impulse mode), NOT an
        /// angle, so it carries no radians/degrees unit.</summary>
        public static void AddTorque(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float torque,
            PhysicsForceMode2D mode
        )
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind =
                        mode == PhysicsForceMode2D.Impulse
                            ? PhysicsBody2DCommandKind.AngularImpulse
                            : PhysicsBody2DCommandKind.Torque,
                    angular = torque,
                }
            );
        }

        /// <summary>Set the body's linear velocity directly (m/s), waking it — the
        /// <c>Rigidbody2D.linearVelocity = v</c> write.</summary>
        public static void SetLinearVelocity(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float2 velocity
        )
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind = PhysicsBody2DCommandKind.SetLinearVelocity,
                    linear = velocity,
                }
            );
        }

        /// <summary>Set the body's angular velocity directly, waking it — the
        /// <c>Rigidbody2D.angularVelocity = w</c> write. The value is in <b>degrees per second</b>: the engine's
        /// <c>PhysicsBody.angularVelocity</c> is deg/sec (module XML), matching <c>Rigidbody2D.angularVelocity</c>
        /// (the angular-unit convention in <c>Documentation~/parity-matrix.md</c>).</summary>
        public static void SetAngularVelocity(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float degreesPerSecond
        )
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind = PhysicsBody2DCommandKind.SetAngularVelocity,
                    angular = degreesPerSecond,
                }
            );
        }

        /// <summary>Sweep the body to a target world position over the next step (keeping its current rotation) —
        /// <c>MovePosition(target)</c>, a swept, collision-aware kinematic move, not a teleport.</summary>
        public static void MovePosition(DynamicBuffer<PhysicsBody2DCommand> buffer, float2 target)
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind = PhysicsBody2DCommandKind.MovePosition,
                    linear = target,
                }
            );
        }

        /// <summary>Sweep the body to a target rotation over the next step (keeping its current position) —
        /// <c>MoveRotation(targetRadians)</c>, a swept, collision-aware kinematic move, not a teleport. The
        /// <paramref name="targetRadians"/> is in <b>radians</b> (the engine rotor is unit-agnostic; the package
        /// chooses radians — see the angular-unit convention in <c>Documentation~/parity-matrix.md</c>). Note
        /// <c>Rigidbody2D.MoveRotation</c> takes degrees, so a verbatim port must convert with
        /// <c>math.radians</c>.</summary>
        public static void MoveRotation(DynamicBuffer<PhysicsBody2DCommand> buffer, float targetRadians)
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind = PhysicsBody2DCommandKind.MoveRotation,
                    angular = targetRadians,
                }
            );
        }

        /// <summary>Sweep the body to a target world position AND rotation over the next step —
        /// <c>MovePositionAndRotation(p, r)</c>. The <paramref name="targetRadians"/> is in <b>radians</b> (the
        /// same convention as <see cref="MoveRotation"/>; <c>Documentation~/parity-matrix.md</c>).</summary>
        public static void MovePositionAndRotation(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float2 targetPosition,
            float targetRadians
        )
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind = PhysicsBody2DCommandKind.MovePositionAndRotation,
                    linear = targetPosition,
                    angular = targetRadians,
                }
            );
        }
    }
}
