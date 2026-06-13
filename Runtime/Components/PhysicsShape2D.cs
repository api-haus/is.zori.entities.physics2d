using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Which Box2D geometry a <see cref="PhysicsShape2D"/> carries. Circle and Box ride entirely in the
    /// struct's inline fields; Capsule adds two inline local centers; Polygon and Edge carry a variable-length
    /// vertex array, so they reference a <see cref="PhysicsShape2DVertices"/> blob rather than inline floats.
    /// A new shape kind is a baker plus one creation-switch arm, not a new archetype.
    /// </summary>
    public enum PhysicsShape2DKind : byte
    {
        Circle,
        Box,
        Capsule,
        Polygon,
        Edge,
    }

    /// <summary>
    /// The vertex span of a Polygon or Edge collider, baked into a blob because its length is not known at
    /// compile time. This is the design's "tagged-union flips if" escape — the moment a shape kind needs a
    /// variable-length array (polygon vertices, edge points), the inline union would be mostly-wasted memory,
    /// so that kind references a <c>BlobAssetReference</c> instead, the way <c>com.unity.physics</c> uses
    /// collider blobs. Circle/Box/Capsule keep their fixed-size data inline; only Polygon/Edge allocate a blob.
    /// </summary>
    public struct PhysicsShape2DVertices
    {
        public BlobArray<float2> points;
    }

    /// <summary>
    /// The baked collider geometry, a tagged union over the supported shape kinds. Fixed-size kinds
    /// (Circle/Box/Capsule) live in inline fields; variable-length kinds (Polygon/Edge) reference a
    /// <see cref="PhysicsShape2DVertices"/> blob via <see cref="vertices"/>. The unused inline fields for a
    /// given kind are a few dead floats per entity — cheaper than a component type per shape kind (the
    /// alternative multiplies archetypes and forces the creation system to branch over component presence
    /// rather than a field).
    /// </summary>
    public struct PhysicsShape2D : IComponentData
    {
        /// <summary>The discriminant. Decides which of the fields below are live.</summary>
        public PhysicsShape2DKind kind;

        /// <summary>
        /// <c>Collider2D.offset</c> — the collider's local offset from the body origin, folded into the
        /// geometry at creation (Box/Polygon via a <c>PhysicsTransform</c>, Capsule by translating both
        /// centers, Edge/Polygon by translating each vertex, Circle into <c>CircleGeometry.center</c>).
        /// </summary>
        public float2 offset;

        /// <summary>
        /// Circle: <c>CircleCollider2D.radius</c>. Box: corner-rounding radius (<c>BoxCollider2D.edgeRadius</c>).
        /// Capsule: the capsule end radius. Polygon: corner-rounding radius. Unused for Edge.
        /// </summary>
        public float radius;

        /// <summary>Box: <c>BoxCollider2D.size</c> (full extents). Unused for the other kinds.</summary>
        public float2 size;

        /// <summary>
        /// Box only: the box's local z-rotation in <b>radians</b> (the package rotation-angle convention,
        /// matching <see cref="PhysicsBody2DDefinition.initialRotationRadians"/>), folded into the box geometry
        /// at creation via <c>PolygonGeometry.CreateBox(size, radius, PhysicsTransform(offset,
        /// PhysicsRotate.FromRadians(boxAngleRadians)), inscribe)</c>. The built-in <c>BoxCollider2D</c> has no
        /// own rotation (its rotation is its Transform's), so the built-in box baker leaves this 0; the custom
        /// authoring surface exposes a free box angle the built-in component cannot. 0 (the default) is an
        /// identity rotation, exactly the pre-existing offset-only <c>PhysicsTransform</c>. Unused for the other
        /// kinds (a capsule's rotation is folded into its two centers at bake; a circle has no rotation).
        /// </summary>
        public float boxAngleRadians;

        /// <summary>Capsule: the two local end-cap centers. Unused for the other kinds.</summary>
        public float2 capsuleCenter1;
        public float2 capsuleCenter2;

        /// <summary>
        /// Edge: whether the chain closes into a loop (an <c>EdgeCollider2D</c> is an open chain, so this is
        /// false for the built-in baker). Unused for the other kinds.
        /// </summary>
        public bool edgeIsLoop;

        /// <summary>
        /// Polygon/Edge: the variable-length vertex span. <c>BlobAssetReference.IsCreated</c> is false for the
        /// inline kinds (Circle/Box/Capsule), which never read it.
        /// </summary>
        public BlobAssetReference<PhysicsShape2DVertices> vertices;

        /// <summary>
        /// Polygon only: when true, the <see cref="vertices"/> describe a closed (possibly concave, possibly
        /// over-<c>MaxPolygonVertices</c>) outline that must be decomposed into convex polygon fragments at
        /// creation via <c>PolygonGeometry.CreatePolygons</c> + <c>CreateShapeBatch</c>, rather than treated as a
        /// single convex hull (<c>PolygonGeometry.Create</c>). Set by the <c>CompositeCollider2D</c> baker (whose
        /// merged Polygons paths can exceed the single-hull cap) and by a concave/large custom polygon; left false
        /// by a simple convex authored <c>PolygonCollider2D</c>, which keeps the single-hull path unchanged.
        /// </summary>
        public bool polygonDecompose;

        /// <summary>
        /// Surface friction (Coulomb coefficient) from the collider's <c>PhysicsMaterial2D.friction</c>, baked
        /// into <c>PhysicsShapeDefinition.surfaceMaterial.friction</c> at creation (XML
        /// <c>P:…PhysicsShape.SurfaceMaterial.friction</c>). When the collider has no <c>sharedMaterial</c>, the
        /// baker stores the engine's default (0.4) so the baked shape matches a built-in collider with no
        /// material assigned. Stored as a float (not the <c>SurfaceMaterial</c> struct) to keep the component
        /// blittable; the struct is built at creation.
        /// </summary>
        public float friction;

        /// <summary>
        /// Surface bounciness (coefficient of restitution) from the collider's
        /// <c>PhysicsMaterial2D.bounciness</c>, baked into
        /// <c>PhysicsShapeDefinition.surfaceMaterial.bounciness</c> at creation (XML
        /// <c>P:…PhysicsShape.SurfaceMaterial.bounciness</c>). Default 0 (no material → no bounce).
        /// </summary>
        public float bounciness;

        /// <summary>
        /// <c>Collider2D.density</c> (XML <c>P:…PhysicsShapeDefinition.density</c>), baked into the shape's
        /// density so an auto-mass body derives the same mass the built-in collider would. The built-in
        /// default density is 1; a value &lt;= 0 leaves Box2D's default.
        /// </summary>
        public float density;

        /// <summary>
        /// How this shape's friction is mixed with a contacting shape's friction (XML
        /// <c>P:…PhysicsShape.SurfaceMaterial.frictionMixing</c>), baked from the built-in
        /// <c>PhysicsMaterial2D.frictionCombine</c> or the custom authoring's combine field. The creation system
        /// writes it into <c>PhysicsShapeDefinition.surfaceMaterial.frictionMixing</c>. The default mirrors the
        /// engine's default <c>SurfaceMaterial</c> mixing, so a shape that does not override it bakes the same
        /// contact the package produced before this field existed.
        /// </summary>
        public PhysicsSurfaceMixing2D frictionMixing;

        /// <summary>
        /// How this shape's bounciness is mixed with a contacting shape's bounciness (XML
        /// <c>P:…PhysicsShape.SurfaceMaterial.bouncinessMixing</c>), baked from the built-in
        /// <c>PhysicsMaterial2D.bounceCombine</c> or the custom authoring's combine field. Written into
        /// <c>PhysicsShapeDefinition.surfaceMaterial.bouncinessMixing</c> at creation; the default mirrors the
        /// engine default so an un-overridden shape is unchanged.
        /// </summary>
        public PhysicsSurfaceMixing2D bouncinessMixing;

        /// <summary>
        /// The Box2D contact-filter <em>categories</em> mask — which categories this shape is in — as a raw
        /// 64-bit value (XML <c>P:…PhysicsShape.ContactFilter.categories</c>, a <c>PhysicsMask</c>). Baked from
        /// the authoring GameObject's layer as <c>1 &lt;&lt; gameObject.layer</c>; the creation system wraps it
        /// in a <c>PhysicsMask{ bitMask = categoryBits }</c> and writes it into
        /// <c>PhysicsShapeDefinition.contactFilter.categories</c>. Stored as a plain <c>ulong</c> (not a
        /// <c>PhysicsMask</c>) to keep the component blittable and dependency-light, exactly as the surface
        /// floats above are stored and the Box2D struct built at creation. A value of <c>0</c> means "no layer
        /// resolved" (a direct/custom-authored shape with no GameObject layer) — the creation system then uses
        /// the everything-default filter so such a shape collides with everything, preserving the dual-surface
        /// default. A built-in-baked body always has a non-zero category (<c>1 &lt;&lt; layer</c> is never 0).
        /// </summary>
        public ulong categoryBits;

        /// <summary>
        /// The Box2D contact-filter <em>contacts</em> mask — which categories this shape produces contacts with
        /// (XML <c>P:…PhysicsShape.ContactFilter.contacts</c>). Baked from the project layer-collision-matrix
        /// row for the GameObject's layer (<c>Physics2D.GetLayerCollisionMask(layer)</c>, the authoritative mask
        /// the GameObject runtime itself consults). The symmetric matrix makes the pair-collision decision
        /// reproduce the GameObject's regardless of the world's <c>ContactFilterMode</c>. Used only when
        /// <see cref="categoryBits"/> is non-zero; otherwise the everything-default filter is applied.
        /// </summary>
        public ulong contactBits;

        /// <summary>
        /// <c>Collider2D.isTrigger</c> — whether this shape is a sensor (XML
        /// <c>P:…PhysicsShapeDefinition.isTrigger</c>). A trigger shape generates overlap (trigger) events but
        /// never a collision response, and does not detect other triggers. Baked from the built-in collider's
        /// <c>isTrigger</c> (or the custom authoring's <c>CollisionResponse == Sensor</c>); the creation system
        /// sets <c>PhysicsShapeDefinition.isTrigger</c> from it. A trigger pair surfaces as a
        /// <see cref="PhysicsTriggerEvent2D"/>; a non-trigger pair as a <see cref="PhysicsContactEvent2D"/>.
        /// </summary>
        public bool isTrigger;
    }

    /// <summary>
    /// An EXTRA shape on a multi-shape body, beyond the primary <see cref="PhysicsShape2D"/> component. A body
    /// with one collider carries only the primary component and NO buffer — the single-shape archetype. A body
    /// that merges several colliders into one collision surface (a
    /// <c>CompositeCollider2D</c>) or carries an explicit set of low-level shapes (a <c>CustomCollider2D</c>)
    /// carries the primary component as shape 0 and this buffer as shapes 1..K-1; the creation system attaches
    /// the primary first, then every buffer element, to the SAME Box2D body via the same
    /// <c>CreateShapeForBody</c> path. A <see cref="PhysicsShape2D"/> cannot be both an <c>IComponentData</c>
    /// and an <c>IBufferElementData</c>, so the buffer wraps one; the wrapper is otherwise free of behaviour.
    /// All shapes of a body share its <c>userData</c> entity packing (query resolution is per-body) and its
    /// lifetime (destroying the body cascades to every shape), so no per-extra-shape teardown or resolution is
    /// needed.
    /// </summary>
    public struct PhysicsShape2DElement : IBufferElementData
    {
        public PhysicsShape2D shape;
    }
}
