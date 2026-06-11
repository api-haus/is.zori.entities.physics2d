using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The result of a <see cref="PhysicsQueries2D.ClosestPoint"/> query — the nearest world body to a query
    /// point, the closest point on that body, the separation distance, and the surface normal. The ECS-facing
    /// analogue of <c>com.unity.physics</c>'s <c>DistanceHit</c> for the point-distance case, carrying the same
    /// data a controller needs for anchor detection (which body, where, how far) and depenetration (the push-out
    /// direction).
    /// </summary>
    /// <remarks>
    /// This is a plain blittable value (not an <see cref="IComponentData"/>): the query returns it through an
    /// <c>out</c>, it is not stored on an entity.
    /// <para><see cref="distance"/> is the separation between the query point and the body surface, zero when the
    /// point is inside the body (Box2D's <c>ShapeDistance</c> returns zero for an overlap). <see cref="normal"/>
    /// points from the query point toward the body and is degenerate when <see cref="distance"/> is zero — a
    /// caller that needs a push-out direction from inside a body uses the overlap+cast-back path, not this query.</para>
    /// <para><see cref="point"/> is the closest point on the body's surface (the <c>DistanceResult.pointB</c>);
    /// for a point query outside the body it is the nearest surface point, the position a snap/clamp targets.</para>
    /// </remarks>
    public struct ClosestPoint2D
    {
        /// <summary>The entity owning the nearest body, or <see cref="Entity.Null"/> if not a package body.</summary>
        public Entity entity;

        /// <summary>The closest point on the nearest body's surface to the query point.</summary>
        public float2 point;

        /// <summary>The surface normal pointing from the query point toward the body. Degenerate when
        /// <see cref="distance"/> is zero (the query point is inside the body).</summary>
        public float2 normal;

        /// <summary>The separation distance between the query point and the body surface. Zero when the point is
        /// inside the body.</summary>
        public float distance;

        /// <summary>The raw Box2D shape that was nearest, for callers needing the shape type or other detail.</summary>
        public PhysicsShape shape;
    }
}
