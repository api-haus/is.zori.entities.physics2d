using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The per-body smoothing state an interpolated/extrapolated body carries: its previous and current
    /// physics poses (captured by <c>PhysicsBody2DWriteBackSystem</c> each fixed step) plus its post-step
    /// velocity (for extrapolation). <c>PhysicsBody2DSmoothingSystem</c> reads these at RENDER rate to write
    /// the smoothed <see cref="Unity.Transforms.LocalToWorld"/> between fixed steps. Only bodies whose baked
    /// <see cref="PhysicsBody2DInterpolation"/> is not <see cref="PhysicsBody2DInterpolation.None"/> carry this
    /// component, so a non-interpolated body costs nothing and keeps its fixed-rate <c>LocalToWorld</c>.
    /// </summary>
    /// <remarks>
    /// The rotation is stored as the (cos, sin) pair the write-back already reduces a 2D
    /// <c>PhysicsBody.BatchTransform.rotation</c> to, so the render-rate normalized-lerp works on the pair
    /// directly in Burst (the 2D analogue of <c>math.nlerp</c> on a quaternion: <c>normalize(lerp(prev, cur,
    /// t))</c>), with no managed <c>PhysicsRotate</c> call from the job. <see cref="angularVelRad"/> is
    /// radians/sec, converted at capture from the body's deg/sec <c>angularVelocity</c>. The DOTS posture is
    /// held: smoothing is computed in ECS over these stored poses and written to <c>LocalToWorld</c> — the
    /// Box2D <c>TransformWriteMode.Interpolate</c> managed-Transform tween is never enabled.
    /// </remarks>
    public struct PhysicsBody2DSmoothing : IComponentData
    {
        /// <summary>Previous-step world position.</summary>
        public float2 prevPos;

        /// <summary>Previous-step rotation as (cos, sin).</summary>
        public float2 prevCosSin;

        /// <summary>Current (just-stepped) world position.</summary>
        public float2 curPos;

        /// <summary>Current (just-stepped) rotation as (cos, sin).</summary>
        public float2 curCosSin;

        /// <summary>Post-step linear velocity (m/s), for <see cref="PhysicsBody2DInterpolation.Extrapolate"/>.</summary>
        public float2 linearVel;

        /// <summary>Post-step angular velocity in RADIANS/sec, for extrapolation (converted from the body's
        /// deg/sec <c>angularVelocity</c> at capture).</summary>
        public float angularVelRad;

        /// <summary>The smoothing mode (a <see cref="PhysicsBody2DInterpolation"/> value).</summary>
        public byte mode;

        /// <summary>0 until a second pose has been captured — before two poses exist there is nothing to
        /// interpolate, so the smoothing system writes the current pose.</summary>
        public byte hasPrev;
    }

    /// <summary>
    /// The most recent fixed-step time, written by <c>PhysicsWorld2DSystem</c> on each stepping frame and read
    /// by the render-rate <c>PhysicsBody2DSmoothingSystem</c> to compute how far the variable-rate render time
    /// is ahead of the last physics step. A singleton (one per ECS world), the 2D analogue of
    /// <c>com.unity.physics</c>'s <c>MostRecentFixedTime</c>.
    /// </summary>
    public struct PhysicsFixedStepTime2D : IComponentData
    {
        /// <summary><c>SystemAPI.Time.ElapsedTime</c> of the fixed group at the last physics step.</summary>
        public double elapsedTime;

        /// <summary>The fixed step's <c>DeltaTime</c> — the normalizer for the render-ahead fraction.</summary>
        public float deltaTime;
    }
}
