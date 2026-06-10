using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes the customisable <see cref="PhysicsShape2DAuthoring"/> into the SAME <see cref="PhysicsShape2D"/>
    /// the built-in collider bakers produce, reading geometry + material from the authoring component's
    /// inline fields instead of a built-in <c>Collider2D</c>. A circle, box, capsule, polygon, or edge
    /// authored this way is indistinguishable at runtime from the same shape authored on the built-in
    /// collider — the convergence the dual surface relies on.
    /// </summary>
    /// <remarks>
    /// When the GameObject carries no <see cref="PhysicsBody2DAuthoring"/> the shape is a collider-only
    /// static body (the same rule built-in 2D physics applies to a <c>Collider2D</c> with no
    /// <c>Rigidbody2D</c>), so this baker emits a default static <see cref="PhysicsBody2DDefinition"/> for
    /// it — the custom-surface analogue of <c>Collider2DBaking.AddStaticBodyIfNoRigidbody</c>, checking for
    /// the custom body component rather than the built-in one. A GameObject carrying
    /// <see cref="PhysicsBody2DAuthoring"/> gets its body definition from
    /// <see cref="PhysicsBody2DAuthoringBaker"/>; this baker must not also add one or the entity carries two
    /// and baking throws.
    /// </remarks>
    public sealed class PhysicsShape2DAuthoringBaker : Baker<PhysicsShape2DAuthoring>
    {
        public override void Bake(PhysicsShape2DAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            // The transform scale is baked into the geometry exactly as the built-in collider bakers do, so a
            // shape authored via this custom surface on a scaled GameObject is indistinguishable at runtime
            // from the same shape on a built-in collider (the dual-surface convergence the bake-contract
            // promises). Same per-kind rules: per-axis box extents, cmax circle/capsule radius, signed +
            // winding-reversed vertices, signed offset.
            var scale = Collider2DBaking.ReadScale(authoring.transform);
            var flip = Collider2DBaking.FlipsWinding(scale);

            // Resolve the contact-filter bits. OverrideFilterBits authors the category/contact masks explicitly
            // (the raw 32-bit masks widened to 64 bits with the upper bits zero, exactly as the layer path's
            // unchecked-uint widening does). Otherwise the optional Layer drives them: a Layer of -1 (the
            // default) leaves both masks 0, so the creation system applies the everything-default (collide with
            // everything) — the custom surface's historical behaviour; a Layer in [0..31] bakes the same
            // 1<<layer / matrix-row pair the built-in collider bakers do.
            ulong categoryBits = 0ul;
            ulong contactBits = 0ul;
            if (authoring.OverrideFilterBits)
            {
                categoryBits = unchecked((uint)authoring.CategoryBits);
                contactBits = unchecked((uint)authoring.ContactBits);
            }
            else if (authoring.Layer >= 0)
            {
                categoryBits = 1ul << authoring.Layer;
                // Fully qualified: the package root namespace shadows the bare "Physics2D" token.
                contactBits = unchecked(
                    (uint)UnityEngine.Physics2D.GetLayerCollisionMask(authoring.Layer)
                );
            }

            var shape = new PhysicsShape2D
            {
                kind = authoring.Kind,
                offset = Collider2DBaking.ScaleOffset(authoring.Offset, scale),
                friction = authoring.Friction,
                bounciness = authoring.Bounciness,
                density = authoring.Density,
                frictionMixing = authoring.FrictionCombine,
                bouncinessMixing = authoring.BouncinessCombine,
                categoryBits = categoryBits,
                contactBits = contactBits,
                // The 2D-expressible collision-response subset: Sensor → a trigger, Collide → solid.
                isTrigger = authoring.CollisionResponse == PhysicsCollisionResponse2D.Sensor,
            };

            switch (authoring.Kind)
            {
                case PhysicsShape2DKind.Circle:
                    shape.radius = Collider2DBaking.ScaleCircleRadius(authoring.Radius, scale);
                    break;

                case PhysicsShape2DKind.Box:
                    shape.size = Collider2DBaking.ScaleBoxSize(authoring.BoxSize, scale);
                    shape.radius = Collider2DBaking.ScaleRoundingRadius(authoring.Radius, scale);
                    // The free box z-rotation (degrees → radians, the package convention), folded into the box
                    // geometry at creation. A mirror scale (odd negative-axis count) reverses the apparent sense
                    // of rotation, so negate the angle to keep the box's authored orientation under a flip.
                    shape.boxAngleRadians = radians(
                        flip ? -authoring.BoxAngle : authoring.BoxAngle
                    );
                    break;

                case PhysicsShape2DKind.Capsule:
                    // Scale the authored capsule centres signed and the radius by the cmax circle rule (the
                    // custom capsule stores explicit centres, so the orthogonal-axis rule is ill-defined under
                    // non-uniform scale — same approximation as CustomCollider2DBaker).
                    authoring.GetCapsuleCenters(out var capsuleRadius, out var c1, out var c2);
                    shape.radius = Collider2DBaking.ScaleCircleRadius(capsuleRadius, scale);
                    shape.capsuleCenter1 = c1 * scale;
                    shape.capsuleCenter2 = c2 * scale;
                    break;

                case PhysicsShape2DKind.Polygon:
                    shape.radius = Collider2DBaking.ScaleRoundingRadius(authoring.Radius, scale);
                    shape.vertices = BuildVertexBlob(authoring.Vertices, scale, flip);
                    break;

                case PhysicsShape2DKind.Edge:
                    shape.edgeIsLoop = authoring.EdgeIsLoop;
                    shape.vertices = BuildVertexBlob(authoring.Vertices, scale, flip);
                    break;
            }

            AddComponent(entity, shape);

            AddStaticBodyIfNoCustomBody(authoring, scale);
        }

        /// <summary>
        /// Bake the authored vertex list into a <see cref="PhysicsShape2DVertices"/> blob, the same form the
        /// built-in <c>PolygonCollider2DBaker</c>/<c>EdgeCollider2DBaker</c> produce so the creation system's
        /// Polygon/Edge arms read it identically.
        /// </summary>
        BlobAssetReference<PhysicsShape2DVertices> BuildVertexBlob(
            Vector2[] points,
            float2 scale,
            bool flip
        )
        {
            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<PhysicsShape2DVertices>();
            var array = builder.Allocate(ref root.points, points.Length);
            for (var i = 0; i < points.Length; i++)
            {
                var src = flip ? points.Length - 1 - i : i;
                array[i] = (float2)points[src] * scale;
            }
            var blob = builder.CreateBlobAssetReference<PhysicsShape2DVertices>(
                Allocator.Persistent
            );
            builder.Dispose();
            AddBlobAsset(ref blob, out _);
            return blob;
        }

        /// <summary>
        /// Emit a default static body definition when no <see cref="PhysicsBody2DAuthoring"/> is present —
        /// the custom-surface analogue of <c>Collider2DBaking.AddStaticBodyIfNoRigidbody</c>. The
        /// <c>GetComponent</c> call registers the dependency so a later-added body authoring re-bakes
        /// correctly.
        /// </summary>
        void AddStaticBodyIfNoCustomBody(PhysicsShape2DAuthoring authoring, float2 scale)
        {
            if (GetComponent<PhysicsBody2DAuthoring>() != null)
                return;

            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var t = GetComponent<Transform>();
            AddComponent(
                entity,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    gravityScale = 0f,
                    linearDamping = 0f,
                    angularDamping = 0f,
                    initialPosition = ((float3)t.position).xy,
                    initialRotationRadians = radians(t.eulerAngles.z),
                    constraints = PhysicsBody.BodyConstraints.None,
                    mass = 0f,
                    useAutoMass = false,
                }
            );
            // The collider-only static body carries the entity scale to graphics (the same scale baked into
            // the shape geometry above), so the write-back re-applies it to LocalToWorld.
            AddComponent(entity, new PhysicsBody2DRenderScale { value = scale });
        }
    }
}
