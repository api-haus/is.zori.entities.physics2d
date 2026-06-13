using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Smooths the RENDERED pose of an interpolated/extrapolated body between fixed physics steps, the package's
    /// analogue of <c>Rigidbody2D.interpolation</c>. Runs at variable/render rate in
    /// <see cref="TransformSystemGroup"/>, <c>[UpdateBefore(LocalToWorldSystem)]</c>, and overwrites each
    /// smoothed body's <see cref="LocalToWorld"/> with the pose interpolated between its previous and current
    /// physics states (Interpolate — one step of render lag) or extrapolated ahead of its current state using
    /// its post-step velocity (Extrapolate). The DOTS posture is held: the smoothed matrix is written to
    /// <c>LocalToWorld</c> in ECS — the Box2D managed-Transform tween is never enabled.
    /// </summary>
    /// <remarks>
    /// <b>Why this group / order.</b> <see cref="TransformSystemGroup"/> defaults into the per-frame
    /// <c>SimulationSystemGroup</c>, so it runs at the variable/render rate (the fixed physics step runs in
    /// <c>FixedStepSimulationSystemGroup</c>). <c>[UpdateBefore(LocalToWorldSystem)]</c> places the smoothing
    /// write just before <c>LocalToWorld</c> is consumed for rendering — exactly <c>com.unity.physics</c>'s
    /// <c>SmoothRigidBodiesGraphicalMotion</c> placement. <see cref="LocalToWorldSystem"/> does NOT overwrite the
    /// smoothed write: its root job only rewrites <c>LocalToWorld</c> when the chunk's <c>LocalTransform</c>
    /// changed, and the package never writes a baked body's <c>LocalTransform</c> after baking, so it stays
    /// unchanged. (The same property is why the fixed-step write-back's <c>LocalToWorld</c> survives.)
    ///
    /// <b>Time model.</b> <c>PhysicsWorld2DSystem</c> records the last fixed step's
    /// <c>(ElapsedTime, DeltaTime)</c> into the <see cref="PhysicsFixedStepTime2D"/> singleton; this system reads
    /// its own (variable-rate) <c>SystemAPI.Time.ElapsedTime</c> and forms
    /// <c>timeAhead = renderElapsed − fixedElapsed</c>, normalized by the fixed step. When physics and render
    /// run at the same rate (e.g. PlayMode-in-batchmode with no sub-step headroom) <c>timeAhead ≈ 0</c> and the
    /// smoothing is an identity write of the current pose — correct, just invisible.
    ///
    /// <b>Burst.</b> The per-body lerp/extrapolate + matrix build is a <c>[BurstCompile] IJobChunk</c> — the
    /// second Burst entry point in the package (alongside the write-back job); the time read is on the main
    /// thread in <c>OnUpdate</c> and passed in as job fields.
    /// </remarks>
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateBefore(typeof(LocalToWorldSystem))]
    public partial struct PhysicsBody2DSmoothingSystem : ISystem
    {
        ComponentTypeHandle<PhysicsBody2DSmoothing> _smoothingType;
        ComponentTypeHandle<LocalToWorld> _localToWorldType;
        EntityTypeHandle _entityType;
        ComponentLookup<PhysicsBody2DRenderScale> _renderScaleLookup;
        EntityQuery _query;

        public void OnCreate(ref SystemState state)
        {
            _smoothingType = state.GetComponentTypeHandle<PhysicsBody2DSmoothing>(isReadOnly: true);
            _localToWorldType = state.GetComponentTypeHandle<LocalToWorld>(isReadOnly: false);
            _entityType = state.GetEntityTypeHandle();
            _renderScaleLookup = state.GetComponentLookup<PhysicsBody2DRenderScale>(isReadOnly: true);
            _query = SystemAPI.QueryBuilder().WithAll<PhysicsBody2DSmoothing>().WithAllRW<LocalToWorld>().Build();
            state.RequireForUpdate(_query);
            state.RequireForUpdate<PhysicsFixedStepTime2D>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var fixedTime = SystemAPI.GetSingleton<PhysicsFixedStepTime2D>();
            if (fixedTime.deltaTime <= 0f)
                return; // no step has run yet → nothing to smooth toward

            // How far the variable-rate render time is ahead of the last physics step, normalized by the fixed
            // step. Clamp the normalized fraction to [0, 1] (interpolation lives in that range); timeAhead (the
            // unclamped seconds-ahead) drives extrapolation.
            var timeAhead = (float)(SystemAPI.Time.ElapsedTime - fixedTime.elapsedTime);
            if (timeAhead < 0f)
                timeAhead = 0f;
            var normalizedTimeAhead = clamp(timeAhead / fixedTime.deltaTime, 0f, 1f);

            _smoothingType.Update(ref state);
            _localToWorldType.Update(ref state);
            _entityType.Update(ref state);
            _renderScaleLookup.Update(ref state);
            state.Dependency = new SmoothJob
            {
                SmoothingType = _smoothingType,
                LocalToWorldType = _localToWorldType,
                EntityType = _entityType,
                RenderScaleLookup = _renderScaleLookup,
                TimeAhead = timeAhead,
                NormalizedTimeAhead = normalizedTimeAhead,
            }.ScheduleParallel(_query, state.Dependency);
        }

        [BurstCompile]
        struct SmoothJob : IJobChunk
        {
            [ReadOnly]
            public ComponentTypeHandle<PhysicsBody2DSmoothing> SmoothingType;
            public ComponentTypeHandle<LocalToWorld> LocalToWorldType;

            [ReadOnly]
            public EntityTypeHandle EntityType;

            [ReadOnly]
            public ComponentLookup<PhysicsBody2DRenderScale> RenderScaleLookup;
            public float TimeAhead;
            public float NormalizedTimeAhead;

            public void Execute(
                in ArchetypeChunk chunk,
                int unfilteredChunkIndex,
                bool useEnabledMask,
                in v128 chunkEnabledMask
            )
            {
                var smoothings = chunk.GetNativeArray(ref SmoothingType);
                var localToWorlds = chunk.GetNativeArray(ref LocalToWorldType);
                var entities = chunk.GetNativeArray(EntityType);

                for (int i = 0, n = chunk.Count; i < n; i++)
                {
                    var s = smoothings[i];

                    float2 pos;
                    float2 cosSin;

                    // Before two poses have been captured there is nothing to interpolate — write the current.
                    if (s.hasPrev == 0)
                    {
                        pos = s.curPos;
                        cosSin = s.curCosSin;
                    }
                    else if (s.mode == (byte)PhysicsBody2DInterpolation.Extrapolate)
                    {
                        // Predict ahead of the current pose using the post-step velocity: pos advances linearly,
                        // angle advances by ω·timeAhead (the 2D rigid-body integration GraphicalSmoothingUtility
                        // .Extrapolate does in 3D).
                        pos = s.curPos + s.linearVel * TimeAhead;
                        var ang = atan2(s.curCosSin.y, s.curCosSin.x) + s.angularVelRad * TimeAhead;
                        sincos(ang, out var sn, out var cs);
                        cosSin = float2(cs, sn);
                    }
                    else
                    {
                        // Interpolate between previous and current (one step of render lag). Position is a plain
                        // lerp; rotation is a normalized lerp of the (cos, sin) pair — the 2D analogue of
                        // math.nlerp on a quaternion, and what PhysicsRotate.LerpRotation does internally.
                        pos = lerp(s.prevPos, s.curPos, NormalizedTimeAhead);
                        var lerped = lerp(s.prevCosSin, s.curCosSin, NormalizedTimeAhead);
                        var len = length(lerped);
                        cosSin = len > 1e-6f ? lerped / len : s.curCosSin;
                    }

                    // Build the same column-major T·R·S matrix the fixed-step write-back builds: rotation
                    // columns scaled by the entity's graphics scale (absent → (1, 1)) so a scaled body keeps
                    // its scale through the render-rate smoothing too, never just at the fixed step.
                    var c = cosSin.x;
                    var sgn = cosSin.y;
                    var entity = entities[i];
                    var scale = RenderScaleLookup.HasComponent(entity)
                        ? RenderScaleLookup[entity].value
                        : new float2(1f, 1f);
                    localToWorlds[i] = new LocalToWorld
                    {
                        Value = float4x4(
                            c * scale.x,
                            -sgn * scale.y,
                            0f,
                            pos.x,
                            sgn * scale.x,
                            c * scale.y,
                            0f,
                            pos.y,
                            0f,
                            0f,
                            1f,
                            0f,
                            0f,
                            0f,
                            0f,
                            1f
                        ),
                    };
                }
            }
        }
    }
}
