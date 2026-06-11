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
        public static void MoveRotation(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float targetRadians
        )
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

        // The per-axis flag bits carried in PhysicsBody2DCommand.worldPoint.x for a SetTransform command (see the
        // struct doc-comment): bit 0 sets position, bit 1 sets rotation. A combined SetTransform sets both; the
        // SetPosition/SetRotation helpers set one.
        const float SetPositionFlag = 1f;
        const float SetRotationFlag = 2f;

        /// <summary>INSTANTANEOUSLY set the body's world position AND rotation — a hard teleport, the 2D analogue of
        /// writing <c>LocalTransform.Position</c>/<c>.Rotation</c> directly. Unlike <see cref="MovePositionAndRotation"/>
        /// (a velocity-based swept move that the world <c>maximumLinearSpeed</c> clamp gates and that resolves
        /// collisions along the path), this is a direct write to the body's <c>transform</c> (native
        /// <c>b2Body_SetTransform</c>) with no <c>deltaTime</c> and no velocity, so it reaches an arbitrarily far
        /// destination in one step regardless of distance. It does NOT clear the body's velocity — pair it with a
        /// <see cref="SetLinearVelocity"/> / <see cref="SetAngularVelocity"/> for a respawn-style teleport that drops
        /// momentum, and follow it with <see cref="SkipInterpolation"/> so an interpolated body does not draw a streak
        /// from the old pose to the new. The <paramref name="rotationRadians"/> is in <b>radians</b> (the package's
        /// rotation convention; <c>Documentation~/parity-matrix.md</c>).</summary>
        public static void SetTransform(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float2 position,
            float rotationRadians
        )
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind = PhysicsBody2DCommandKind.SetTransform,
                    linear = position,
                    angular = rotationRadians,
                    worldPoint = new float2(SetPositionFlag + SetRotationFlag, 0f),
                }
            );
        }

        /// <summary>INSTANTANEOUSLY set the body's world position only, keeping its current rotation — the
        /// position-only form of <see cref="SetTransform"/> (a hard teleport, not the swept <see cref="MovePosition"/>).</summary>
        public static void SetPosition(DynamicBuffer<PhysicsBody2DCommand> buffer, float2 position)
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind = PhysicsBody2DCommandKind.SetTransform,
                    linear = position,
                    worldPoint = new float2(SetPositionFlag, 0f),
                }
            );
        }

        /// <summary>INSTANTANEOUSLY set the body's rotation only, keeping its current position — the rotation-only
        /// form of <see cref="SetTransform"/> (a hard set, not the swept <see cref="MoveRotation"/>, so it lands the
        /// exact angle in one step with none of <c>MoveRotation</c>'s single-step undershoot). The
        /// <paramref name="rotationRadians"/> is in <b>radians</b>.</summary>
        public static void SetRotation(
            DynamicBuffer<PhysicsBody2DCommand> buffer,
            float rotationRadians
        )
        {
            buffer.Add(
                new PhysicsBody2DCommand
                {
                    kind = PhysicsBody2DCommandKind.SetTransform,
                    angular = rotationRadians,
                    worldPoint = new float2(SetRotationFlag, 0f),
                }
            );
        }

        /// <summary>Suppress the next render-rate interpolation for this body so the next frame draws the body's
        /// current (just-set) pose with NO interpolation streak from its previous pose — the 2D analogue of the 3D
        /// <c>CharacterInterpolation.SkipNextInterpolation()</c>. Pair it with <see cref="SetTransform"/> for a
        /// teleport: <c>SetTransform</c> moves the body instantly, <c>SkipInterpolation</c> stops the render-rate
        /// smoothing from drawing a one-step slide from the old location to the new one. The reset reads the body's
        /// live pose at drain time, so it must be appended AFTER the <see cref="SetTransform"/> in the same frame
        /// (both drain in order before the step). A no-op on a body with no <see cref="PhysicsBody2DSmoothing"/>
        /// (interpolation <see cref="PhysicsBody2DInterpolation.None"/>) — such a body has no streak to suppress.</summary>
        public static void SkipInterpolation(DynamicBuffer<PhysicsBody2DCommand> buffer)
        {
            buffer.Add(
                new PhysicsBody2DCommand { kind = PhysicsBody2DCommandKind.SkipInterpolation }
            );
        }
    }
}
