using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Reads every live body's pose in one bulk native call and a Burst job writes each into
    /// <see cref="LocalToWorld"/>. Ordered after <see cref="PhysicsWorld2DSystem"/> in
    /// <see cref="Physics2DSimulationSystemGroup"/> so it runs on the just-stepped poses.
    /// </summary>
    /// <remarks>
    /// Write-back addressing (the slice's simplest reliable form): one pass over the
    /// <see cref="PhysicsBody2D"/> query builds two index-aligned arrays — the body handles for the bulk
    /// <c>GetBatchTransform</c> read and the matching entities — then the Burst job
    /// scatters each read pose into that entity's <see cref="LocalToWorld"/> via a
    /// <see cref="ComponentLookup{T}"/>. This inherits the POC's index-alignment assumption
    /// (<c>GetBatchTransform</c> returns results in input-span order); the chunk-native form that makes
    /// alignment per-entity-explicit is the roadmap escape if alignment ever bites.
    ///
    /// Writing <see cref="LocalToWorld"/> directly is the cleaner mechanism for a flat, unparented
    /// hierarchy: the matrix is what the body's pose already is, with no parent to compose against. The
    /// query carries <c>WithAll&lt;LocalToWorld&gt;</c> so the lookup write always finds the component
    /// (the baker requests <c>TransformUsageFlags.Dynamic</c>, which adds it).
    /// </remarks>
    [UpdateInGroup(typeof(Physics2DSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsWorld2DSystem))]
    public partial struct PhysicsBody2DWriteBackSystem : ISystem
    {
        ComponentLookup<LocalToWorld> _localToWorldLookup;
        ComponentLookup<PhysicsBody2DSmoothing> _smoothingLookup;
        ComponentLookup<PhysicsBody2DRenderScale> _renderScaleLookup;

        public void OnCreate(ref SystemState state)
        {
            _localToWorldLookup = state.GetComponentLookup<LocalToWorld>(isReadOnly: false);
            _smoothingLookup = state.GetComponentLookup<PhysicsBody2DSmoothing>(isReadOnly: false);
            _renderScaleLookup = state.GetComponentLookup<PhysicsBody2DRenderScale>(isReadOnly: true);
            state.RequireForUpdate<PhysicsBody2D>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var query = SystemAPI.QueryBuilder().WithAll<PhysicsBody2D, LocalToWorld>().Build();
            var count = query.CalculateEntityCount();
            if (count == 0)
                return;

            var entities = query.ToEntityArray(Allocator.TempJob);
            var bodyComponents = query.ToComponentDataArray<PhysicsBody2D>(Allocator.TempJob);
            var bodies = new NativeArray<PhysicsBody>(count, Allocator.TempJob);
            for (var i = 0; i < count; i++)
                bodies[i] = bodyComponents[i].body;
            bodyComponents.Dispose();

            // The bulk read is index-aligned with the input body span (and therefore with `entities`).
            var transforms = LowLevelPhysics2DCompat.GetBatchTransform(bodies, count, Allocator.TempJob);

            // Capture the prev/cur/velocity smoothing state for each interpolated body BEFORE scheduling the
            // write-back job. This is a main-thread pass (it reads the body's managed velocity via the
            // Unity.U2D.Physics handle, which is not Burst, and the transforms/bodies arrays directly) so it runs
            // synchronously here while the arrays are valid and the job has not started. Only entities carrying
            // PhysicsBody2DSmoothing are touched — a non-interpolated body has no such component and is skipped,
            // keeping its fixed-rate LocalToWorld as the final pose.
            _smoothingLookup.Update(ref state);
            CaptureSmoothing(transforms, bodies, entities, count, _smoothingLookup);

            _localToWorldLookup.Update(ref state);
            _renderScaleLookup.Update(ref state);
            var job = new BatchTransformToLocalToWorldJob
            {
                Transforms = transforms,
                Entities = entities,
                RenderScaleLookup = _renderScaleLookup,
                LocalToWorldLookup = _localToWorldLookup,
            };
            var handle = job.Schedule(count, 64, state.Dependency);

            // The per-step native arrays are TempJob, disposed once the job that reads them completes.
            handle = bodies.Dispose(handle);
            handle = transforms.Dispose(handle);
            handle = entities.Dispose(handle);
            state.Dependency = handle;
        }

        // Shift cur→prev and write the new cur pose (from the just-read BatchTransform) + the body's post-step
        // velocity into each interpolated entity's PhysicsBody2DSmoothing. The render-rate smoothing system reads
        // these between fixed steps. Rotation is stored as the (cos, sin) pair the BatchTransform already exposes
        // (no managed PhysicsRotate). Angular velocity is converted deg/sec → rad/sec for the extrapolation math.
        // Managed Unity.U2D.Physics velocity reads on the main thread — not Burst, like the world/body calls.
        static void CaptureSmoothing(
            NativeArray<PhysicsBody.BatchTransform> transforms,
            NativeArray<PhysicsBody> bodies,
            NativeArray<Entity> entities,
            int count,
            ComponentLookup<PhysicsBody2DSmoothing> smoothingLookup
        )
        {
            for (var i = 0; i < count; i++)
            {
                var entity = entities[i];
                if (!smoothingLookup.HasComponent(entity))
                    continue;

                var t = transforms[i];
                var curPos = float2(t.position.x, t.position.y);
                var curCosSin = float2(t.rotation.cos, t.rotation.sin);

                var body = bodies[i];
                var linVel = Unity.Mathematics.float2.zero;
                var angVelRad = 0f;
                if (body.isValid)
                {
                    var v = body.linearVelocity; // Vector2, m/s
                    linVel = float2(v.x, v.y);
                    // PhysicsBody.angularVelocity is degrees/sec (module XML); the smoothing math is radians/sec.
                    angVelRad = radians(body.angularVelocity);
                }

                var s = smoothingLookup[entity];
                // The previous cur becomes prev; the just-read pose becomes cur. After the first capture
                // hasPrev flips to 1 so the smoothing system has two poses to interpolate between.
                s.prevPos = s.curPos;
                s.prevCosSin = s.curCosSin;
                s.curPos = curPos;
                s.curCosSin = curCosSin;
                s.linearVel = linVel;
                s.angularVelRad = angVelRad;
                s.hasPrev = 1;
                smoothingLookup[entity] = s;
            }
        }
    }
}
