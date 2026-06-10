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
        [Tooltip("Circle: the circle radius. Box/Polygon: the corner-rounding radius. Capsule: the end radius.")]
        float m_Radius = 0.5f;

        [SerializeField]
        [Tooltip("Box: full extents (width, height).")]
        float2 m_BoxSize = new float2(1f, 1f);

        [SerializeField]
        [Tooltip("Capsule: full size (width, height).")]
        float2 m_CapsuleSize = new float2(1f, 2f);

        [SerializeField]
        [Tooltip("Capsule: true for a vertical capsule (long axis Y), false for horizontal (long axis X).")]
        bool m_CapsuleVertical = true;

        [SerializeField]
        [Tooltip("Edge: whether the chain closes into a loop. An open edge/chain is false.")]
        bool m_EdgeIsLoop;

        [SerializeField]
        [Tooltip("Polygon (convex 3..8-vertex hull) or Edge (open chain) local-space vertices.")]
        Vector2[] m_Vertices = System.Array.Empty<Vector2>();

        [SerializeField]
        [Tooltip("Surface friction (Coulomb coefficient). The material-less built-in default is 0.4.")]
        float m_Friction = 0.4f;

        [SerializeField]
        [Tooltip("Surface bounciness (coefficient of restitution). The material-less default is 0.")]
        float m_Bounciness;

        [SerializeField]
        [Tooltip("Per-shape density driving the auto-mass-from-shapes path (Collider2D.density). Default 1.")]
        float m_Density = 1f;

        [SerializeField]
        [Tooltip(
            "Collision layer [0..31] used to resolve the Box2D contact filter from the project's 2D layer "
                + "collision matrix, the custom-surface analogue of a GameObject's Layer. -1 (the default) "
                + "means unfiltered (collide with everything), matching the built-in everything default."
        )]
        int m_Layer = -1;

        [SerializeField]
        [Tooltip(
            "Whether this shape is a sensor (the custom-surface analogue of Collider2D.isTrigger). A trigger "
                + "shape produces overlap (trigger) events but never a collision response, and does not detect "
                + "other triggers. Default false (a solid, responding shape)."
        )]
        bool m_IsTrigger;

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

        /// <summary>
        /// Whether this shape is a sensor (the custom-surface analogue of <c>Collider2D.isTrigger</c>). A trigger
        /// shape produces overlap (trigger) events but never a collision response. The baker bakes it to
        /// <c>PhysicsShape2D.isTrigger</c>, which the creation system maps to
        /// <c>PhysicsShapeDefinition.isTrigger</c>.
        /// </summary>
        public bool IsTrigger
        {
            get => m_IsTrigger;
            set => m_IsTrigger = value;
        }

        /// <summary>
        /// The two local end-cap centers for a Capsule, derived from <see cref="CapsuleSize"/> +
        /// <see cref="CapsuleVertical"/> exactly as the built-in capsule baker derives them: a vertical
        /// capsule of size (w, h) has radius w/2 and centers on the Y axis at ±(h/2 − radius); a horizontal
        /// capsule has radius h/2 and centers on the X axis at ±(w/2 − radius).
        /// </summary>
        public void GetCapsuleCenters(out float capsuleRadius, out float2 center1, out float2 center2)
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
        }

        void OnValidate()
        {
            m_Radius = math.max(0f, m_Radius);
            m_Friction = math.max(0f, m_Friction);
            m_Density = math.max(0f, m_Density);
        }
    }
}
