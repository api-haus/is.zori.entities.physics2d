using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// One hit returned by a <see cref="PhysicsQueries2D"/> spatial query, the ECS-facing analogue of
    /// <c>UnityEngine.RaycastHit2D</c> / <c>Collider2D</c> overlap results. It carries the same fields the
    /// GameObject calls expose — the owning <see cref="entity"/> (the collider's owner), the contact
    /// <see cref="point"/>, the surface <see cref="normal"/>, and the cast <see cref="fraction"/> — plus the
    /// raw Box2D <see cref="shape"/> for callers that need the shape type or other low-level detail.
    /// </summary>
    /// <remarks>
    /// This is a plain blittable value (not an <see cref="IComponentData"/>): a query returns hits into a
    /// caller-owned <c>NativeList&lt;PhysicsQueryHit2D&gt;</c> or an <c>out</c>, it is not stored on an entity.
    /// <para><see cref="entity"/> is resolved from the hit shape's body userData
    /// (<c>shape.body.userData.int64Value</c>, packed at body creation). A hit on a body the package did not
    /// create — e.g. a shapeless joint world-anchor with no userData — resolves to <see cref="Entity.Null"/>,
    /// which a caller can filter out if it only wants package entities.</para>
    /// <para>Overlap queries (point / circle / box overlap) report only the overlapping shape, so
    /// <see cref="point"/> / <see cref="normal"/> / <see cref="fraction"/> are zero for them; only cast queries
    /// (raycast, circle/box cast) fill the contact geometry.</para>
    /// </remarks>
    public struct PhysicsQueryHit2D
    {
        /// <summary>The entity owning the hit shape's body, or <see cref="Entity.Null"/> if not a package body.</summary>
        public Entity entity;

        /// <summary>The world-space contact point (cast queries). Zero for overlap queries.</summary>
        public float2 point;

        /// <summary>The world-space surface normal at the contact (cast queries). Zero for overlap queries.</summary>
        public float2 normal;

        /// <summary>The fraction of the cast distance to the contact, in [0, 1] (cast queries). Zero otherwise.</summary>
        public float fraction;

        /// <summary>The raw Box2D shape that was hit, for callers needing the shape type or other detail.</summary>
        public PhysicsShape shape;
    }
}
