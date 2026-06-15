using Unity.Collections;
using UnityEngine;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Bridges the two low-level 2D physics surfaces this package targets: the API ships under
    /// <c>UnityEngine.LowLevelPhysics2D</c> on Unity 6000.3/6000.4 and moved to <c>Unity.U2D.Physics</c>
    /// (a new <c>PhysicsCore2DModule</c>) on 6000.6. The bare namespace move is handled per-file by a
    /// <c>#if UNITY_6000_6_OR_NEWER</c> using-swap; the two members here cover the places where 6000.6 also
    /// changed the member surface: it added the <see cref="PhysicsBody"/> bulk transform reader and a
    /// <c>vertexScale</c>-free <c>PolygonGeometry.CreatePolygons</c> overload that the earlier surface lacks.
    /// Each member calls the 6000.6 form and reconstructs the identical result on 6000.3/6000.4.
    /// </summary>
    static class LowLevelPhysics2DCompat
    {
        /// <summary>
        /// The index-aligned bulk transform read over the first <paramref name="count"/> entries of
        /// <paramref name="bodies"/>. On 6000.6 this is the native <c>PhysicsBody.GetBatchTransform</c>;
        /// the earlier surface has no bulk reader, so the same array is assembled by reading each body's
        /// live <c>transform</c> into a <c>BatchTransform</c>. The caller owns the returned array and
        /// disposes it.
        /// </summary>
        public static NativeArray<PhysicsBody.BatchTransform> GetBatchTransform(
            NativeArray<PhysicsBody> bodies,
            int count,
            Allocator allocator
        )
        {
#if UNITY_6000_6_OR_NEWER
            return PhysicsBody.GetBatchTransform(bodies.AsReadOnlySpan(), allocator);
#else
            var transforms = new NativeArray<PhysicsBody.BatchTransform>(count, allocator);
            for (var i = 0; i < count; i++)
            {
                var tf = bodies[i].transform;
                transforms[i] = new PhysicsBody.BatchTransform(bodies[i])
                {
                    position = tf.position,
                    rotation = tf.rotation,
                };
            }
            return transforms;
#endif
        }

        /// <summary>
        /// Decompose a closed outline into convex polygon fragments. The 6000.6 three-argument
        /// <c>PolygonGeometry.CreatePolygons</c> forwards <c>vertexScale = Vector2.one</c> and
        /// <c>radius = 0</c>; the earlier surface exposes only the explicit-<c>vertexScale</c> overload, so
        /// <c>Vector2.one</c> reproduces it exactly. The caller owns the returned array and disposes it.
        /// </summary>
        public static NativeArray<PolygonGeometry> CreatePolygons(
            System.ReadOnlySpan<Vector2> vertices,
            PhysicsTransform transform,
            Allocator allocator
        )
        {
#if UNITY_6000_6_OR_NEWER
            return PolygonGeometry.CreatePolygons(vertices, transform, allocator);
#else
            return PolygonGeometry.CreatePolygons(vertices, transform, Vector2.one, allocator);
#endif
        }
    }
}
