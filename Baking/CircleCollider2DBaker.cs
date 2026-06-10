using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="CircleCollider2D"/> into a <see cref="PhysicsShape2D"/> of kind
    /// <see cref="PhysicsShape2DKind.Circle"/>, mapping <see cref="CircleCollider2D.radius"/> and
    /// <c>Collider2D.offset</c> to the Box2D <c>CircleGeometry{ radius, center }</c> at creation.
    /// </summary>
    /// <remarks>
    /// Targets the same entity as <see cref="Rigidbody2DBaker"/> via the same
    /// <c>GetEntity(TransformUsageFlags.Dynamic)</c>, so a body+collider GameObject produces one entity
    /// with both components. A collider-only GameObject (no <see cref="Rigidbody2D"/>) is a static body, so —
    /// like every collider baker since Phase 1A — it emits a default static
    /// <see cref="PhysicsBody2DDefinition"/> via <see cref="Collider2DBaking.AddStaticBodyIfNoRigidbody"/>
    /// (a no-op when a <see cref="Rigidbody2D"/> is present). Surface friction/bounciness/density are read
    /// from the collider's <c>sharedMaterial</c>/<c>density</c> like the other shapes.
    /// </remarks>
    public sealed class CircleCollider2DBaker : Baker<CircleCollider2D>
    {
        public override void Bake(CircleCollider2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            // A circle cannot become an ellipse, so the radius takes the LARGER absolute axis scale
            // (CircleCollider2D's rule); the offset scales signed per-axis.
            var scale = Collider2DBaking.ReadScale(authoring.transform);
            Collider2DBaking.ReadSurface(
                authoring,
                out var friction,
                out var bounciness,
                out var density,
                out var frictionMixing,
                out var bouncinessMixing
            );
            Collider2DBaking.ReadFilter(authoring, out var categoryBits, out var contactBits);
            AddComponent(
                entity,
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = Collider2DBaking.ScaleCircleRadius(authoring.radius, scale),
                    offset = Collider2DBaking.ScaleOffset((float2)authoring.offset, scale),
                    friction = friction,
                    bounciness = bounciness,
                    density = density,
                    frictionMixing = frictionMixing,
                    bouncinessMixing = bouncinessMixing,
                    categoryBits = categoryBits,
                    contactBits = contactBits,
                    isTrigger = authoring.isTrigger,
                }
            );
            Collider2DBaking.AddStaticBodyIfNoRigidbody(this);
        }
    }
}
