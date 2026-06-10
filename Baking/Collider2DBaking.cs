using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Shared baking helpers for the collider family. The collider-only static-body fallback lives here so
    /// every collider baker emits it identically: a <see cref="Collider2D"/> on a GameObject with no
    /// <see cref="Rigidbody2D"/> is a static body in built-in 2D physics, so the baker must add a default
    /// static <see cref="PhysicsBody2DDefinition"/> for it (otherwise the creation system never makes a body
    /// and the shape has nothing to attach to). A GameObject that DOES carry a <see cref="Rigidbody2D"/> gets
    /// its <see cref="PhysicsBody2DDefinition"/> from <see cref="Rigidbody2DBaker"/>; the collider baker must
    /// not also add one, or the entity carries two and baking throws.
    /// </summary>
    public static class Collider2DBaking
    {
        /// <summary>
        /// Emit a default static <see cref="PhysicsBody2DDefinition"/> for a collider-only GameObject. Called
        /// from each collider baker after it adds its shape; a no-op when a <see cref="Rigidbody2D"/> is
        /// present (that path bakes the body definition itself). The no-arg <c>Baker</c> queries target the
        /// primary authoring GameObject, which is the collider's own GameObject — so <paramref name="baker"/>
        /// is the concrete collider baker and the helper reads its primary authoring object.
        /// </summary>
        public static void AddStaticBodyIfNoRigidbody<TAuthoring>(Baker<TAuthoring> baker)
            where TAuthoring : Component
        {
            // GetComponent registers the dependency so a later-added Rigidbody2D re-bakes correctly.
            if (baker.GetComponent<Rigidbody2D>() != null)
                return;

            var entity = baker.GetEntity(TransformUsageFlags.Dynamic);
            var t = baker.GetComponent<Transform>();
            baker.AddComponent(
                entity,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    gravityScale = 0f,
                    linearDamping = 0f,
                    angularDamping = 0f,
                    initialPosition = ((float3)t.position).xy,
                    initialRotationRadians = radians(t.eulerAngles.z),
                    // A static body never integrates, so constraints/mass are inert; useAutoMass false
                    // matches the built-in default but is moot for a body Box2D never moves.
                    constraints = PhysicsBody.BodyConstraints.None,
                    mass = 0f,
                    useAutoMass = false,
                }
            );
            // The collider-only static body IS the body for this GameObject, so it carries the entity's
            // transform scale to graphics — the "even for static colliders" case the user named. The scale
            // is baked INTO the shape geometry (above, in the collider baker), so the Box2D body is unit
            // scale; this component re-applies the scale to LocalToWorld at write-back.
            baker.AddComponent(entity, new PhysicsBody2DRenderScale { value = ReadScale(t) });
        }

        /// <summary>
        /// The body GameObject's authored X/Y world scale, the factor baked into every collider's geometry
        /// (a Box2D shape carries no scale field — module XML: <c>CircleGeometry</c>/<c>PolygonGeometry</c>/
        /// <c>CapsuleGeometry</c> expose only positions/radii/vertices) and carried to graphics on
        /// <see cref="PhysicsBody2DRenderScale"/>. Read from <c>transform.lossyScale</c> (the world scale,
        /// honouring any parent scale); accessing it through the baker's <c>GetComponent&lt;Transform&gt;()</c>
        /// result registers the transform dependency so a scale change re-bakes. The package authors one
        /// collider on the body's own (leaf) GameObject, so the body scale and the shape scale are the same
        /// <c>lossyScale</c> — there is no shape-vs-body relative transform to decompose (unlike the 3D
        /// <c>com.unity.physics</c> compound-collider path).
        /// </summary>
        public static float2 ReadScale(Transform t) => ((float3)t.lossyScale).xy;

        /// <summary>
        /// Scale a box's full extents per-axis, matching <c>BoxCollider2D</c> under transform scale (the
        /// editor box gizmo follows X and Y independently). <c>abs</c> because a negative (flipped) axis
        /// mirrors a symmetric box about its centre rather than shrinking it — the mirror is carried by the
        /// signed <see cref="ScaleOffset"/> moving the centre, while the extents stay positive.
        /// </summary>
        public static float2 ScaleBoxSize(float2 size, float2 scale) => abs(size * scale);

        /// <summary>
        /// Scale a circle radius, matching <c>CircleCollider2D</c> under transform scale: a circle cannot
        /// become an ellipse, so the LARGER absolute axis scale is used (the circle gizmo grows to the larger
        /// axis). The 2D analogue of <c>com.unity.physics</c>'s <c>cmax(abs(lossyScale))</c> sphere rule.
        /// </summary>
        public static float ScaleCircleRadius(float radius, float2 scale) =>
            radius * max(abs(scale.x), abs(scale.y));

        /// <summary>
        /// Scale a corner-rounding radius (box <c>edgeRadius</c>, polygon corner radius). A rounding radius is
        /// a circle of curvature, so the isotropic circle rule applies; under non-uniform scale this is an
        /// approximation (a true scaled rounded box has elliptical corners Box2D cannot express).
        /// </summary>
        public static float ScaleRoundingRadius(float radius, float2 scale) =>
            ScaleCircleRadius(radius, scale);

        /// <summary>
        /// Scale a collider's local <c>offset</c> per-axis, SIGNED — a flip moves the offset to the mirrored
        /// side (the centre of a symmetric shape moves while its extents stay positive). Matches
        /// <c>Collider2D.offset</c> being a point in the collider's scaled local space.
        /// </summary>
        public static float2 ScaleOffset(float2 offset, float2 scale) => offset * scale;

        /// <summary>
        /// Whether the scale mirrors the plane (an ODD number of negative axes), which reverses polygon/edge
        /// winding. Box2D wants CCW convex hulls and a chain's solid side is its winding (the Phase-1A/9
        /// winding sensitivity), so a baker that scales vertices signed must REVERSE the vertex order when
        /// this is true to restore the authored winding.
        /// </summary>
        public static bool FlipsWinding(float2 scale) => scale.x * scale.y < 0f;

        /// <summary>
        /// Read the friction/bounciness/density a built-in collider contributes to its baked
        /// <see cref="PhysicsShape2D"/>. <c>Collider2D.sharedMaterial</c> (a <c>PhysicsMaterial2D</c>) carries
        /// friction + bounciness; <c>Collider2D.density</c> carries the per-shape density. When no
        /// <c>sharedMaterial</c> is assigned the engine uses friction 0.4 / bounciness 0 (the Physics2D
        /// settings default), so the baker stores those defaults to match a material-less built-in collider.
        /// The surface values are stored as plain floats on the shape (the Box2D <c>SurfaceMaterial</c> struct
        /// is built at creation), keeping <see cref="PhysicsShape2D"/> blittable.
        /// </summary>
        public static void ReadSurface(
            Collider2D collider,
            out float friction,
            out float bounciness,
            out float density,
            out PhysicsSurfaceMixing2D frictionMixing,
            out PhysicsSurfaceMixing2D bouncinessMixing
        )
        {
            var material = collider.sharedMaterial;
            if (material != null)
            {
                friction = material.friction;
                bounciness = material.bounciness;
                // The built-in combine modes map onto the low-level mixing modes; read them so a built-in
                // collider and a custom shape with the same combine bake identical mixing (the convergence the
                // dual surface relies on). UnityEngine.PhysicsMaterialCombine2D and the low-level
                // SurfaceMaterial.MixingMode share the same five members (Average / Maximum / Mean / Minimum /
                // Multiply, CoreModule.xml T:…PhysicsMaterialCombine2D), so the map is a 1:1 by-name match.
                frictionMixing = MapCombine(material.frictionCombine);
                bouncinessMixing = MapCombine(material.bounceCombine);
            }
            else
            {
                // Built-in default when a collider has no material assigned.
                friction = 0.4f;
                bounciness = 0f;
                // The default combine mode for a material-less collider; the package default mirrors the engine
                // default SurfaceMaterial mixing, so an un-overridden shape bakes the contact it always did.
                frictionMixing = PhysicsSurfaceMixing2D.Average;
                bouncinessMixing = PhysicsSurfaceMixing2D.Average;
            }
            density = collider.density;
        }

        /// <summary>
        /// Map the built-in <c>UnityEngine.PhysicsMaterialCombine2D</c> friction/bounce combine policy onto the
        /// package-local <see cref="PhysicsSurfaceMixing2D"/> (which mirrors the low-level
        /// <c>PhysicsShape.SurfaceMaterial.MixingMode</c>). Both enums share the same five members in the same
        /// declaration order (<c>Average / Maximum / Mean / Minimum / Multiply</c>, CoreModule.xml
        /// <c>T:…PhysicsMaterialCombine2D</c>), so this is a 1:1 by-name match; <c>Average</c> covers any future
        /// addition.
        /// </summary>
        static PhysicsSurfaceMixing2D MapCombine(PhysicsMaterialCombine2D combine) =>
            combine switch
            {
                PhysicsMaterialCombine2D.Maximum => PhysicsSurfaceMixing2D.Maximum,
                PhysicsMaterialCombine2D.Mean => PhysicsSurfaceMixing2D.Mean,
                PhysicsMaterialCombine2D.Minimum => PhysicsSurfaceMixing2D.Minimum,
                PhysicsMaterialCombine2D.Multiply => PhysicsSurfaceMixing2D.Multiply,
                _ => PhysicsSurfaceMixing2D.Average,
            };

        /// <summary>
        /// Resolve the Box2D contact-filter category + contacts bits a collider contributes to its baked
        /// <see cref="PhysicsShape2D"/>, from the authoring GameObject's layer and the project's 2D
        /// layer-collision-matrix. <paramref name="authoring"/> is the collider (any
        /// <c>UnityEngine.Component</c>), read for its <c>gameObject.layer</c>:
        /// <list type="bullet">
        /// <item><c>categoryBits = 1 &lt;&lt; layer</c> — the single category this shape is in.</item>
        /// <item><c>contactBits = Physics2D.GetLayerCollisionMask(layer)</c> — the matrix row of layers this
        /// layer collides with, the authoritative mask the GameObject runtime itself consults. The 32-bit
        /// <c>LayerMask</c> result is widened to <c>ulong</c> with the upper 32 bits zero, exactly as
        /// <c>PhysicsMask.#ctor(LayerMask)</c> does.</item>
        /// </list>
        /// This is a bake-time read of the project matrix (the matrix is a project constant a baked subscene
        /// freezes), so the runtime creation system never touches <c>UnityEngine.Physics2D</c> — it just wraps
        /// these bits in a <c>PhysicsMask</c>. A default-layer (0) GameObject under the default all-on matrix
        /// bakes <c>categoryBits = 1</c> / <c>contactBits = 0xFFFFFFFF</c> — collides with everything, matching
        /// the pre-filtering behaviour so existing fixtures are unperturbed.
        /// </summary>
        public static void ReadFilter(
            Component authoring,
            out ulong categoryBits,
            out ulong contactBits
        )
        {
            var layer = authoring.gameObject.layer;
            categoryBits = 1ul << layer;
            // Fully qualified: the package root namespace Zori.Entities.Physics2D shadows the bare token
            // "Physics2D", so the UnityEngine type must be named explicitly.
            contactBits = unchecked((uint)UnityEngine.Physics2D.GetLayerCollisionMask(layer));
        }

        /// <summary>
        /// Whether a built-in collider is merged into a <c>CompositeCollider2D</c> and must therefore NOT bake its
        /// own shape/body. A child collider declares it is composited by setting <c>compositeOperation</c> to
        /// anything other than <c>None</c> (the <c>6000.6.0a6</c> replacement for the legacy <c>usedByComposite</c>
        /// bool — <c>Merge</c> is the common case). Such a collider's geometry is already represented inside the
        /// composite's merged paths (<c>CompositeCollider2D.GetPath</c>), so baking the child again would create a
        /// second overlapping shape on its own static body — the double-bake a composite exists to avoid. Each
        /// collider baker returns early when this is true, emitting nothing; the <c>CompositeCollider2DBaker</c>
        /// bakes the merged surface instead.
        /// </summary>
        public static bool IsUsedByComposite(Collider2D collider) =>
            collider.compositeOperation != Collider2D.CompositeOperation.None;
    }

    /// <summary>
    /// Bakes a built-in <see cref="BoxCollider2D"/> into a <see cref="PhysicsShape2D"/> of kind
    /// <see cref="PhysicsShape2DKind.Box"/>: <c>BoxCollider2D.size</c> (full extents) → the box size,
    /// <c>BoxCollider2D.edgeRadius</c> → the corner-rounding radius, <c>Collider2D.offset</c> folded at
    /// creation. The Box2D mapping is <c>PolygonGeometry.CreateBox(size, radius, transform, inscribe)</c>.
    /// </summary>
    public sealed class BoxCollider2DBaker : Baker<BoxCollider2D>
    {
        public override void Bake(BoxCollider2D authoring)
        {
            // A collider merged into a CompositeCollider2D contributes to the composite's merged geometry, not as a
            // standalone shape — the CompositeCollider2DBaker bakes the merged surface. Bake nothing here.
            if (Collider2DBaking.IsUsedByComposite(authoring))
                return;

            var entity = GetEntity(TransformUsageFlags.Dynamic);
            // The transform scale is baked INTO the geometry: per-axis on the box extents, isotropic on the
            // corner-rounding radius, signed on the offset. A Box2D box carries no scale, so this is the only
            // way a scaled BoxCollider2D collides at its rendered size (the manual-QA wide-floor bug).
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
                    kind = PhysicsShape2DKind.Box,
                    size = Collider2DBaking.ScaleBoxSize((float2)authoring.size, scale),
                    radius = Collider2DBaking.ScaleRoundingRadius(authoring.edgeRadius, scale),
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

    /// <summary>
    /// Bakes a built-in <see cref="CapsuleCollider2D"/> into a <see cref="PhysicsShape2D"/> of kind
    /// <see cref="PhysicsShape2DKind.Capsule"/>. The built-in capsule is authored as a <c>size</c> +
    /// <c>direction</c>; Box2D's capsule is two end-cap centers + a radius, so the baker converts: for a
    /// Vertical capsule of size (w, h) the radius is w/2 and the two centers sit on the Y axis at
    /// ±(h/2 − radius); for Horizontal, radius is h/2 and the centers sit on the X axis at ±(w/2 − radius).
    /// The local centers are stored offset-free; <c>Collider2D.offset</c> is folded at creation.
    /// </summary>
    public sealed class CapsuleCollider2DBaker : Baker<CapsuleCollider2D>
    {
        public override void Bake(CapsuleCollider2D authoring)
        {
            // A capsule is not compositeCapable today, but the guard is uniform across colliders and documents the
            // contract (a never-true branch here, correct if a future Unity makes capsules compositable).
            if (Collider2DBaking.IsUsedByComposite(authoring))
                return;

            var entity = GetEntity(TransformUsageFlags.Dynamic);

            // Scale the authored size per-axis BEFORE deriving the caps, matching CapsuleCollider2D under
            // transform scale (the gizmo deforms its caps from the scaled size). Scaling the precomputed caps
            // uniformly would be wrong — a non-uniform scale must change both the cap radius and the segment.
            var scale = Collider2DBaking.ReadScale(authoring.transform);
            var halfSize = Collider2DBaking.ScaleBoxSize((float2)authoring.size, scale) * 0.5f;
            float capsuleRadius;
            float2 c1,
                c2;
            if (authoring.direction == CapsuleDirection2D.Vertical)
            {
                capsuleRadius = halfSize.x;
                var half = max(0f, halfSize.y - capsuleRadius);
                c1 = new float2(0f, -half);
                c2 = new float2(0f, half);
            }
            else
            {
                capsuleRadius = halfSize.y;
                var half = max(0f, halfSize.x - capsuleRadius);
                c1 = new float2(-half, 0f);
                c2 = new float2(half, 0f);
            }

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
                    kind = PhysicsShape2DKind.Capsule,
                    radius = capsuleRadius,
                    capsuleCenter1 = c1,
                    capsuleCenter2 = c2,
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

    /// <summary>
    /// Bakes a built-in <see cref="PolygonCollider2D"/> into a <see cref="PhysicsShape2D"/> of kind
    /// <see cref="PhysicsShape2DKind.Polygon"/>. The first path's points (<c>GetPath(0)</c>) are baked into a
    /// <see cref="PhysicsShape2DVertices"/> blob; <c>Collider2D.offset</c> is folded at creation via a
    /// <c>PhysicsTransform</c>. Box2D's <c>PolygonGeometry.Create</c> requires a convex hull of 3..8 vertices,
    /// so a parity fixture must author a convex polygon (a non-convex built-in polygon would need a
    /// composite/decomposed path, which is later-phase work).
    /// </summary>
    public sealed class PolygonCollider2DBaker : Baker<PolygonCollider2D>
    {
        public override void Bake(PolygonCollider2D authoring)
        {
            // A polygon merged into a CompositeCollider2D contributes to the merged geometry, not as a standalone
            // shape — the CompositeCollider2DBaker bakes the merged surface. Bake nothing here.
            if (Collider2DBaking.IsUsedByComposite(authoring))
                return;

            var entity = GetEntity(TransformUsageFlags.Dynamic);

            // Scale each vertex signed and reverse the order on a winding-flipping (mirror) scale, so the
            // convex hull stays CCW for PolygonGeometry.Create's validation. The offset is scaled signed.
            var scale = Collider2DBaking.ReadScale(authoring.transform);
            var points = authoring.GetPath(0);
            var flip = Collider2DBaking.FlipsWinding(scale);
            var builder = new BlobBuilder(Unity.Collections.Allocator.Temp);
            ref var root = ref builder.ConstructRoot<PhysicsShape2DVertices>();
            var array = builder.Allocate(ref root.points, points.Length);
            for (var i = 0; i < points.Length; i++)
            {
                var src = flip ? points.Length - 1 - i : i;
                array[i] = (float2)points[src] * scale;
            }
            var blob = builder.CreateBlobAssetReference<PhysicsShape2DVertices>(
                Unity.Collections.Allocator.Persistent
            );
            builder.Dispose();
            AddBlobAsset(ref blob, out _);

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
                    kind = PhysicsShape2DKind.Polygon,
                    radius = 0f,
                    offset = Collider2DBaking.ScaleOffset((float2)authoring.offset, scale),
                    vertices = blob,
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

    /// <summary>
    /// Bakes a built-in <see cref="EdgeCollider2D"/> into a <see cref="PhysicsShape2D"/> of kind
    /// <see cref="PhysicsShape2DKind.Edge"/>: its point list (<c>EdgeCollider2D.points</c>) becomes a
    /// <see cref="PhysicsShape2DVertices"/> blob, created at runtime as a Box2D <c>ChainGeometry</c> via
    /// <c>CreateChain</c>. <c>Collider2D.offset</c> is folded per-vertex at creation. An
    /// <see cref="EdgeCollider2D"/> is an open chain, so the chain is not a loop.
    /// </summary>
    public sealed class EdgeCollider2DBaker : Baker<EdgeCollider2D>
    {
        public override void Bake(EdgeCollider2D authoring)
        {
            // An edge merged into a CompositeCollider2D contributes to the merged geometry, not as a standalone
            // shape — the CompositeCollider2DBaker bakes the merged surface. Bake nothing here.
            if (Collider2DBaking.IsUsedByComposite(authoring))
                return;

            var entity = GetEntity(TransformUsageFlags.Dynamic);

            // Scale each chain point signed and reverse the order on a winding-flipping (mirror) scale: a
            // chain's solid side IS its winding (Phase-1A edge fixture), so a mirror that did not reverse the
            // order would flip the solid side and let bodies fall through. The offset is scaled signed (the
            // creation system folds it per-vertex at runtime).
            var scale = Collider2DBaking.ReadScale(authoring.transform);
            var points = authoring.points;
            var flip = Collider2DBaking.FlipsWinding(scale);
            var builder = new BlobBuilder(Unity.Collections.Allocator.Temp);
            ref var root = ref builder.ConstructRoot<PhysicsShape2DVertices>();
            var array = builder.Allocate(ref root.points, points.Length);
            for (var i = 0; i < points.Length; i++)
            {
                var src = flip ? points.Length - 1 - i : i;
                array[i] = (float2)points[src] * scale;
            }
            var blob = builder.CreateBlobAssetReference<PhysicsShape2DVertices>(
                Unity.Collections.Allocator.Persistent
            );
            builder.Dispose();
            AddBlobAsset(ref blob, out _);

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
                    kind = PhysicsShape2DKind.Edge,
                    offset = Collider2DBaking.ScaleOffset((float2)authoring.offset, scale),
                    edgeIsLoop = false,
                    vertices = blob,
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
