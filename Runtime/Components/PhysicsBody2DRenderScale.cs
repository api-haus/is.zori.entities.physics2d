using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The entity's authored 2D transform scale, carried to graphics. A baked body's collider geometry has
    /// the GameObject's <c>lossyScale</c> baked INTO it (a Box2D shape has no per-shape or per-body scale —
    /// the engine geometry structs carry only positions/radii), so the Box2D body itself runs at unit scale.
    /// The write-back must therefore re-apply this scale to the body's <c>LocalToWorld</c> every step or the
    /// rendered sprite silently loses its scale. This is the 2D form of <c>com.unity.physics</c>'s
    /// "bake scale into the collider, carry a scale for graphics" split: the collision uses the scaled
    /// geometry, and <c>LocalToWorld</c> keeps the entity scale.
    /// </summary>
    /// <remarks>
    /// Baked from the body GameObject's <c>transform.lossyScale.xy</c> by the body bakers
    /// (<c>Rigidbody2DBaker</c>, the collider-only static fallback, the custom-body authoring), and defaulted
    /// to <c>(1, 1)</c> on the direct/batch paths, which have no GameObject scale to carry. Both the fixed-step
    /// write-back (<c>BatchTransformToLocalToWorldJob</c>) and the render-rate smoothing
    /// (<c>PhysicsBody2DSmoothingSystem</c>) read it through a guarded <c>ComponentLookup</c>, treating its
    /// absence as <c>(1, 1)</c> — so a body that never carries it (a batch body, a pre-existing scale-1
    /// fixture) is unperturbed. Z scale is always 1 (planar physics).
    /// </remarks>
    public struct PhysicsBody2DRenderScale : IComponentData
    {
        /// <summary>The entity's authored X/Y scale, re-applied to <c>LocalToWorld</c> at write-back so the
        /// rendered transform is <c>T · R · S</c> while the Box2D body stays at unit scale.</summary>
        public float2 value;
    }
}
