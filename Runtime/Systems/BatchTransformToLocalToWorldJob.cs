using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Burst-vectorised conversion of the bulk physics-transform read into each body's
    /// <see cref="LocalToWorld"/> matrix. One <see cref="PhysicsBody.BatchTransform"/> in, one
    /// <see cref="LocalToWorld"/> scattered out to the entity at the same index.
    /// </summary>
    /// <remarks>
    /// The only <c>[BurstCompile]</c> entry point in the package (project entry-point-only rule,
    /// <c>docs/unity/burst/compilation-context.md:33-54</c>): the world/body calls are managed
    /// <c>Unity.U2D.Physics</c> APIs that run on the main thread, only this <c>float4x4</c> math is
    /// Burst-eligible. The matrix construction is verbatim from the POC's
    /// <c>BodyTransformToInstanceJob</c> — the body's <c>cos</c>/<c>sin</c> rotation plus translation,
    /// no scale, column-major.
    ///
    /// <see cref="Transforms"/> and <see cref="Entities"/> are index-aligned: index <c>i</c> of the
    /// batch read corresponds to <see cref="Entities"/>[<c>i</c>], because both arrays were built in one
    /// pass over the same query order before the bulk read. The lookup carries
    /// <c>[NativeDisableParallelForRestriction]</c> because each index writes a distinct entity, so the
    /// per-index scatter never races (the entity array holds no duplicates).
    ///
    /// The body runs at unit scale in Box2D (the collider geometry has the entity scale baked into it), so
    /// the entity's transform scale is re-applied here from <see cref="RenderScaleLookup"/> — the matrix is
    /// <c>T · R · S</c> (scale the rotation columns), the standard TRS the renderer expects, so a scaled
    /// sprite keeps its scale while collision uses the scaled shape. A body that carries no
    /// <see cref="PhysicsBody2DRenderScale"/> (a batch-created body, a pre-existing scale-1 fixture) is
    /// treated as scale <c>(1, 1)</c>, leaving its matrix unchanged.
    /// </remarks>
    [BurstCompile]
    public struct BatchTransformToLocalToWorldJob : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<PhysicsBody.BatchTransform> Transforms;

        [ReadOnly]
        public NativeArray<Entity> Entities;

        [ReadOnly]
        public ComponentLookup<PhysicsBody2DRenderScale> RenderScaleLookup;

        [NativeDisableParallelForRestriction]
        public ComponentLookup<LocalToWorld> LocalToWorldLookup;

        public void Execute(int i)
        {
            var entity = Entities[i];
            var t = Transforms[i];
            float2 p = t.position;
            var r = t.rotation;
            float c = r.cos;
            float s = r.sin;

            // Re-apply the entity's graphics scale (absent → (1, 1)). The rotation columns are scaled, so the
            // composed matrix is T·R·S — the Box2D body stays unit-scale, the rendered transform keeps scale.
            var scale = RenderScaleLookup.HasComponent(entity)
                ? RenderScaleLookup[entity].value
                : new float2(1f, 1f);

            float4x4 m = float4x4(
                c * scale.x, -s * scale.y, 0f, p.x,
                s * scale.x, c * scale.y, 0f, p.y,
                0f, 0f, 1f, 0f,
                0f, 0f, 0f, 1f
            );

            LocalToWorldLookup[entity] = new LocalToWorld { Value = m };
        }
    }
}
