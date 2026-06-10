using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Authoring
{
    /// <summary>
    /// The customisable shape authoring component — the 2D analogue of <c>com.unity.physics</c>'s
    /// <c>PhysicsShapeAuthoring</c> (3D, structure-only reference). It authors the package's runtime
    /// <see cref="PhysicsShape2D"/> <em>directly</em>, carrying the <see cref="PhysicsShape2DKind"/> union and
    /// the surface material inline, without a built-in <c>Collider2D</c> in between. This lets a user supply
    /// explicit geometry (a circle radius, a box size, a capsule's two end-cap centers, a polygon/edge
    /// vertex list) and explicit friction/bounciness/density that the matching built-in collider would
    /// round-trip through sprite/mesh resolution.
    /// </summary>
    /// <remarks>
    /// Its baker (<c>PhysicsShape2DAuthoringBaker</c>) emits the SAME <see cref="PhysicsShape2D"/> the
    /// built-in collider bakers emit (and the same collider-only static-body fallback when no
    /// <see cref="PhysicsBody2DAuthoring"/> is present), so a custom-authored shape and the equivalent
    /// built-in collider converge on one runtime archetype. The component lives in a compiled package
    /// assembly (testable); the importable sample references it.
    ///
    /// <para><b>Geometry by kind.</b> Circle reads <see cref="Radius"/>. Box reads <see cref="BoxSize"/> +
    /// <see cref="Radius"/> (corner rounding). Capsule reads <see cref="CapsuleSize"/> +
    /// <see cref="CapsuleVertical"/> and converts to two end-cap centers exactly as the built-in capsule
    /// baker does. Polygon/Edge read <see cref="Vertices"/> (a convex 3..8-vertex hull for Polygon; an open
    /// chain for Edge). <see cref="Offset"/> is the collider's local offset, folded into the geometry at
    /// creation. The unused fields for a given kind are inert.</para>
    /// </remarks>
    [AddComponentMenu("Zori/Entities Physics 2D/Physics Shape 2D")]
    public sealed class PhysicsShape2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Which geometry this shape carries.")]
        PhysicsShape2DKind m_Kind = PhysicsShape2DKind.Circle;

        [SerializeField]
        [Tooltip("Local offset of the collider from the body origin (Collider2D.offset).")]
        float2 m_Offset = float2.zero;

        [SerializeField]
        [Tooltip(
            "Circle: the circle radius. Box/Polygon: the corner-rounding radius. Capsule: the end radius."
        )]
        float m_Radius = 0.5f;

        [SerializeField]
        [Tooltip("Box: full extents (width, height).")]
        float2 m_BoxSize = new float2(1f, 1f);

        [SerializeField]
        [Tooltip(
            "Box: local z-rotation in degrees. The free box orientation the built-in BoxCollider2D cannot "
                + "express (its rotation is its Transform's). 0 is axis-aligned."
        )]
        float m_BoxAngle;

        [SerializeField]
        [Tooltip("Capsule: full size (width, height).")]
        float2 m_CapsuleSize = new float2(1f, 2f);

        [SerializeField]
        [Tooltip(
            "Capsule: true for a vertical capsule (long axis Y), false for horizontal (long axis X)."
        )]
        bool m_CapsuleVertical = true;

        [SerializeField]
        [Tooltip(
            "Capsule: local z-rotation in degrees, applied on top of the vertical/horizontal long axis. A "
                + "free-oriented capsule the built-in CapsuleCollider2D cannot express. 0 is axis-aligned."
        )]
        float m_CapsuleAngle;

        [SerializeField]
        [Tooltip("Edge: whether the chain closes into a loop. An open edge/chain is false.")]
        bool m_EdgeIsLoop;

        [SerializeField]
        [Tooltip("Polygon (convex 3..8-vertex hull) or Edge (open chain) local-space vertices.")]
        Vector2[] m_Vertices = System.Array.Empty<Vector2>();

        [SerializeField]
        [Tooltip(
            "Surface friction (Coulomb coefficient). The material-less built-in default is 0.4."
        )]
        float m_Friction = 0.4f;

        [SerializeField]
        [Tooltip(
            "Surface bounciness (coefficient of restitution). The material-less default is 0."
        )]
        float m_Bounciness;

        [SerializeField]
        [Tooltip(
            "Per-shape density driving the auto-mass-from-shapes path (Collider2D.density). Default 1."
        )]
        float m_Density = 1f;

        [SerializeField]
        [Tooltip(
            "Surface friction combine mode — how this shape's friction mixes with a contacting shape's."
        )]
        PhysicsSurfaceMixing2D m_FrictionCombine = PhysicsSurfaceMixing2D.Average;

        [SerializeField]
        [Tooltip(
            "Surface bounciness combine mode — how this shape's bounciness mixes with a contacting shape's."
        )]
        PhysicsSurfaceMixing2D m_BouncinessCombine = PhysicsSurfaceMixing2D.Average;

        [SerializeField]
        [Tooltip(
            "Collision layer [0..31] used to resolve the Box2D contact filter from the project's 2D layer "
                + "collision matrix, the custom-surface analogue of a GameObject's Layer. -1 (the default) "
                + "means unfiltered (collide with everything), matching the built-in everything default. "
                + "Ignored when OverrideFilterBits is true."
        )]
        int m_Layer = -1;

        [SerializeField]
        [Tooltip(
            "Author the Box2D contact-filter category/contact bitsets explicitly instead of resolving them from "
                + "Layer. Off (the default): the filter comes from Layer + the project layer-collision matrix."
        )]
        bool m_OverrideFilterBits;

        [SerializeField]
        [Tooltip(
            "Explicit contact-filter category bits (which categories this shape is in). Used when OverrideFilterBits is true."
        )]
        int m_CategoryBits = ~0;

        [SerializeField]
        [Tooltip(
            "Explicit contact-filter contact bits (which categories this shape collides with). Used when OverrideFilterBits is true."
        )]
        int m_ContactBits = ~0;

        [SerializeField]
        [Tooltip(
            "How this shape responds to overlaps. Collide: a solid shape with a collision response. Sensor: a "
                + "trigger that overlaps and reports trigger events but never produces a collision response "
                + "(Collider2D.isTrigger). Default Collide."
        )]
        PhysicsCollisionResponse2D m_CollisionResponse = PhysicsCollisionResponse2D.Collide;

        public PhysicsShape2DKind Kind
        {
            get => m_Kind;
            set => m_Kind = value;
        }

        public float2 Offset
        {
            get => m_Offset;
            set => m_Offset = value;
        }

        public float Radius
        {
            get => m_Radius;
            set => m_Radius = math.max(0f, value);
        }

        public float2 BoxSize
        {
            get => m_BoxSize;
            set => m_BoxSize = value;
        }

        /// <summary>Box local z-rotation in <b>degrees</b>. Baked to <c>PhysicsShape2D.boxAngleRadians</c>
        /// (converted to radians) and folded into the box geometry at creation.</summary>
        public float BoxAngle
        {
            get => m_BoxAngle;
            set => m_BoxAngle = value;
        }

        public float2 CapsuleSize
        {
            get => m_CapsuleSize;
            set => m_CapsuleSize = value;
        }

        public bool CapsuleVertical
        {
            get => m_CapsuleVertical;
            set => m_CapsuleVertical = value;
        }

        /// <summary>Capsule local z-rotation in <b>degrees</b>, applied on top of the vertical/horizontal long
        /// axis. Rotates the two derived end-cap centers in <see cref="GetCapsuleCenters"/>.</summary>
        public float CapsuleAngle
        {
            get => m_CapsuleAngle;
            set => m_CapsuleAngle = value;
        }

        public bool EdgeIsLoop
        {
            get => m_EdgeIsLoop;
            set => m_EdgeIsLoop = value;
        }

        public Vector2[] Vertices
        {
            get => m_Vertices;
            set => m_Vertices = value ?? System.Array.Empty<Vector2>();
        }

        public float Friction
        {
            get => m_Friction;
            set => m_Friction = math.max(0f, value);
        }

        public float Bounciness
        {
            get => m_Bounciness;
            set => m_Bounciness = value;
        }

        public float Density
        {
            get => m_Density;
            set => m_Density = value;
        }

        /// <summary>
        /// The collision layer [0..31] this shape is on, or -1 for "unfiltered" (collide with everything). When
        /// in [0..31] the baker resolves the Box2D category + contacts bits from the project layer matrix
        /// exactly as the built-in collider bakers do (<c>1 &lt;&lt; layer</c> /
        /// <c>Physics2D.GetLayerCollisionMask(layer)</c>); when -1 the shape bakes zero bits and the creation
        /// system applies the everything-default, preserving the custom surface's collide-with-everything
        /// default and the dual-surface convergence.
        /// </summary>
        public int Layer
        {
            get => m_Layer;
            set => m_Layer = value < 0 ? -1 : math.clamp(value, 0, 31);
        }

        /// <summary>How friction is combined with a contacting shape's friction. Baked to
        /// <see cref="PhysicsShape2D.frictionMixing"/>.</summary>
        public PhysicsSurfaceMixing2D FrictionCombine
        {
            get => m_FrictionCombine;
            set => m_FrictionCombine = value;
        }

        /// <summary>How bounciness is combined with a contacting shape's bounciness. Baked to
        /// <see cref="PhysicsShape2D.bouncinessMixing"/>.</summary>
        public PhysicsSurfaceMixing2D BouncinessCombine
        {
            get => m_BouncinessCombine;
            set => m_BouncinessCombine = value;
        }

        /// <summary>When true, <see cref="CategoryBits"/> / <see cref="ContactBits"/> are baked directly into the
        /// Box2D contact filter, bypassing the <see cref="Layer"/> + project-matrix resolution.</summary>
        public bool OverrideFilterBits
        {
            get => m_OverrideFilterBits;
            set => m_OverrideFilterBits = value;
        }

        /// <summary>Explicit contact-filter category bits (which categories this shape is in), as a 32-bit mask.
        /// Used when <see cref="OverrideFilterBits"/> is true; baked into <see cref="PhysicsShape2D.categoryBits"/>
        /// (widened to 64 bits with the upper 32 bits zero, exactly as the layer path does).</summary>
        public int CategoryBits
        {
            get => m_CategoryBits;
            set => m_CategoryBits = value;
        }

        /// <summary>Explicit contact-filter contact bits (which categories this shape collides with), as a 32-bit
        /// mask. Used when <see cref="OverrideFilterBits"/> is true; baked into
        /// <see cref="PhysicsShape2D.contactBits"/>.</summary>
        public int ContactBits
        {
            get => m_ContactBits;
            set => m_ContactBits = value;
        }

        /// <summary>
        /// How this shape responds to overlaps — the 2D-expressible subset of the 3D collision-response policy.
        /// <see cref="PhysicsCollisionResponse2D.Sensor"/> bakes <c>PhysicsShape2D.isTrigger = true</c> (a trigger
        /// that overlaps without a collision response); <see cref="PhysicsCollisionResponse2D.Collide"/> bakes a
        /// solid shape. The creation system maps it to <c>PhysicsShapeDefinition.isTrigger</c>.
        /// </summary>
        public PhysicsCollisionResponse2D CollisionResponse
        {
            get => m_CollisionResponse;
            set => m_CollisionResponse = value;
        }

        /// <summary>
        /// The two local end-cap centers for a Capsule, derived from <see cref="CapsuleSize"/> +
        /// <see cref="CapsuleVertical"/> exactly as the built-in capsule baker derives them, then rotated by
        /// <see cref="CapsuleAngle"/> about the origin: a vertical capsule of size (w, h) has radius w/2 and
        /// centers on the Y axis at ±(h/2 − radius); a horizontal capsule has radius h/2 and centers on the X
        /// axis at ±(w/2 − radius). A non-zero <see cref="CapsuleAngle"/> rotates both centers, giving the free
        /// capsule orientation the built-in <c>CapsuleCollider2D</c> cannot express. The runtime stores two free
        /// centers, so the rotated centers bake directly with no runtime change.
        /// </summary>
        public void GetCapsuleCenters(
            out float capsuleRadius,
            out float2 center1,
            out float2 center2
        )
        {
            var halfSize = m_CapsuleSize * 0.5f;
            if (m_CapsuleVertical)
            {
                capsuleRadius = halfSize.x;
                var half = math.max(0f, halfSize.y - capsuleRadius);
                center1 = new float2(0f, -half);
                center2 = new float2(0f, half);
            }
            else
            {
                capsuleRadius = halfSize.y;
                var half = math.max(0f, halfSize.x - capsuleRadius);
                center1 = new float2(-half, 0f);
                center2 = new float2(half, 0f);
            }

            if (m_CapsuleAngle != 0f)
            {
                math.sincos(math.radians(m_CapsuleAngle), out var s, out var c);
                center1 = new float2(c * center1.x - s * center1.y, s * center1.x + c * center1.y);
                center2 = new float2(c * center2.x - s * center2.y, s * center2.x + c * center2.y);
            }
        }

        void OnValidate()
        {
            m_Radius = math.max(0f, m_Radius);
            m_Friction = math.max(0f, m_Friction);
            m_Density = math.max(0f, m_Density);
        }
    }
}
