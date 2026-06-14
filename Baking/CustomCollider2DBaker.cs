using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="CustomCollider2D"/> — an explicit set of low-level physics shapes supplied by
    /// code in a <c>PhysicsShapeGroup2D</c> — into the package's multi-shape channel: one
    /// <see cref="PhysicsShape2D"/> per custom shape, with shape 0 as the primary <see cref="PhysicsShape2D"/>
    /// component and the rest as <see cref="PhysicsShape2DElement"/> buffer elements on the SAME body entity.
    /// Each custom shape's <c>PhysicsShapeType2D</c> maps 1:1 onto the package's existing
    /// <see cref="PhysicsShape2DKind"/> and the Box2D geometry the creation system already builds, so a
    /// custom-shape body collides identically to the equivalent primitives.
    /// </summary>
    /// <remarks>
    /// <para><b>Shape mapping</b> (the <c>UnityEngine.PhysicsShape2D</c> descriptor + its group-local vertices):
    /// <list type="bullet">
    /// <item><b>Circle</b> — 1 vertex = the center, <c>radius</c> = the radius → <see cref="PhysicsShape2DKind.Circle"/>.</item>
    /// <item><b>Capsule</b> — 2 vertices = the two end-cap centers, <c>radius</c> = the cap radius →
    /// <see cref="PhysicsShape2DKind.Capsule"/> (exactly the package's <c>capsuleCenter1/2 + radius</c>).</item>
    /// <item><b>Polygon</b> — N convex-hull vertices, <c>radius</c> = corner rounding →
    /// <see cref="PhysicsShape2DKind.Polygon"/> (decomposed at creation only if it exceeds the single-hull cap).</item>
    /// <item><b>Edges</b> — N chain vertices → <see cref="PhysicsShape2DKind.Edge"/>, a non-loop one-sided chain
    /// (the same static-surface caveat as <c>EdgeCollider2D</c>).</item>
    /// </list></para>
    ///
    /// <para>Surface/filter come from the custom collider's own material/layer (<c>Collider2DBaking.ReadSurface</c>
    /// / <c>.ReadFilter</c>), the same as a built-in collider. The static-body fallback applies (a custom collider
    /// with no <c>Rigidbody2D</c> is static). The <c>UnityEngine.PhysicsShape2D</c> name collides with the
    /// package's runtime <see cref="PhysicsShape2D"/>, so the built-in struct is fully qualified throughout.
    /// Editor-only assembly, never in a player build.</para>
    /// </remarks>
    public sealed class CustomCollider2DBaker : Baker<CustomCollider2D>
    {
        public override void Bake(CustomCollider2D authoring)
        {
            var group = new PhysicsShapeGroup2D();
            authoring.GetCustomShapes(group);
            var shapeCount = group.shapeCount;
            if (shapeCount <= 0)
                return; // a custom collider with no shapes bakes nothing (and no body)

            var entity = GetEntity(TransformUsageFlags.Dynamic);
            // The custom shapes are in the collider GameObject's local space, so the transform scale is baked
            // into each shape's geometry: circle radius cmax + centre signed, capsule centres signed + radius
            // cmax, polygon/edge vertices signed (+ winding-reverse on a mirror). A custom capsule stores two
            // explicit centres (no Vertical/Horizontal axis), so the orthogonal-axis radius rule is ill-defined
            // under non-uniform scale; the simpler cmax(abs(scale)) over-approximates (a known approximation
            // for the code-authored advanced surface — see the doc's Assumptions).
            var scale = Collider2DBaking.ReadScale(authoring.transform);
            var flip = Collider2DBaking.FlipsWinding(scale);
            var radiusScale = max(abs(scale.x), abs(scale.y));
            // Register the shared material as a bake dependency so editing it re-bakes (symmetric with the
            // custom shape baker's DependsOn(MaterialTemplate)).
            Collider2DBaking.DependsOnSharedMaterial(this, authoring);
            Collider2DBaking.ReadSurface(
                authoring,
                out var friction,
                out var bounciness,
                out var density,
                out var frictionMixing,
                out var bouncinessMixing
            );
            Collider2DBaking.ReadFilter(authoring, out var categoryBits, out var contactBits);

            var verts = new List<Vector2>();
            var baked = 0;
            for (var s = 0; s < shapeCount; s++)
            {
                var desc = group.GetShape(s);
                verts.Clear();
                group.GetShapeVertices(s, verts);

                var shape = new PhysicsShape2D
                {
                    friction = friction,
                    bounciness = bounciness,
                    density = density,
                    frictionMixing = frictionMixing,
                    bouncinessMixing = bouncinessMixing,
                    categoryBits = categoryBits,
                    contactBits = contactBits,
                    isTrigger = authoring.isTrigger,
                };

                switch (desc.shapeType)
                {
                    case PhysicsShapeType2D.Circle:
                        if (verts.Count < 1)
                            continue;
                        shape.kind = PhysicsShape2DKind.Circle;
                        shape.radius = desc.radius * radiusScale;
                        // The circle center is its single vertex; fold it into the shape offset (the creation
                        // system places a circle at CircleGeometry.center = offset).
                        shape.offset = (float2)verts[0] * scale;
                        break;

                    case PhysicsShapeType2D.Capsule:
                        if (verts.Count < 2)
                            continue;
                        shape.kind = PhysicsShape2DKind.Capsule;
                        shape.radius = desc.radius * radiusScale;
                        shape.capsuleCenter1 = (float2)verts[0] * scale;
                        shape.capsuleCenter2 = (float2)verts[1] * scale;
                        break;

                    case PhysicsShapeType2D.Polygon:
                        if (verts.Count < 3)
                            continue;
                        shape.kind = PhysicsShape2DKind.Polygon;
                        shape.radius = desc.radius * radiusScale;
                        shape.vertices = BuildVertexBlob(verts, scale, flip);
                        // Decompose only if the polygon exceeds the single-hull vertex cap; a normal small convex
                        // custom polygon keeps the single-hull path (decompose false).
                        shape.polygonDecompose = verts.Count > PhysicsConstants.MaxPolygonVertices;
                        break;

                    case PhysicsShapeType2D.Edges:
                        if (verts.Count < 2)
                            continue;
                        shape.kind = PhysicsShape2DKind.Edge;
                        shape.edgeIsLoop = false; // a custom Edges shape is an open chain (one-sided, static use)
                        shape.vertices = BuildVertexBlob(verts, scale, flip);
                        break;

                    default:
                        continue;
                }

                if (baked == 0)
                {
                    AddComponent(entity, shape);
                }
                else
                {
                    // Extra shapes ride the DynamicBuffer<PhysicsShape2DElement> alongside the primary
                    // PhysicsShape2D. AppendToBuffer on the primary entity requires this baker to have ADDED the
                    // buffer first (it throws "component hasn't been added by the baker yet" otherwise), so add
                    // an empty buffer the first time an extra shape is emitted, then append into it.
                    if (baked == 1)
                        AddBuffer<PhysicsShape2DElement>(entity);
                    AppendToBuffer(entity, new PhysicsShape2DElement { shape = shape });
                }
                baked++;
            }

            if (baked == 0)
                return; // every custom shape was degenerate — nothing to bake, no body

            Collider2DBaking.AddStaticBodyIfNoRigidbody(this);
        }

        /// <summary>
        /// Bake a custom shape's vertices into a <see cref="PhysicsShape2DVertices"/> blob, the same form the
        /// built-in polygon/edge bakers and the composite baker produce so the creation system reads it
        /// identically. The transform scale is baked into each vertex (signed); a winding-flipping (mirror)
        /// scale reverses the order so a polygon hull stays CCW and an edge chain's solid side is preserved.
        /// </summary>
        BlobAssetReference<PhysicsShape2DVertices> BuildVertexBlob(List<Vector2> points, float2 scale, bool flip)
        {
            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<PhysicsShape2DVertices>();
            var array = builder.Allocate(ref root.points, points.Count);
            for (var i = 0; i < points.Count; i++)
            {
                var src = flip ? points.Count - 1 - i : i;
                array[i] = (float2)points[src] * scale;
            }
            var blob = builder.CreateBlobAssetReference<PhysicsShape2DVertices>(Allocator.Persistent);
            builder.Dispose();
            AddBlobAsset(ref blob, out _);
            return blob;
        }
    }
}
