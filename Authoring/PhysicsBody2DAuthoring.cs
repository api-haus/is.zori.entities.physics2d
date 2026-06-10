using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Authoring
{
    /// <summary>
    /// The body type a <see cref="PhysicsBody2DAuthoring"/> declares directly, the 2D analogue of the DOTS
    /// custom sample's <c>BodyMotionType</c>. The values are the package's own enum (not the built-in
    /// <c>RigidbodyType2D</c>) so the custom surface never depends on a built-in component; the baker maps
    /// each to <c>Unity.U2D.Physics.PhysicsBody.BodyType</c>.
    /// </summary>
    public enum PhysicsBody2DMotionType : byte
    {
        Dynamic,
        Kinematic,
        Static,
    }

    /// <summary>
    /// The customisable body authoring component — the 2D analogue of <c>com.unity.physics</c>'s
    /// <c>PhysicsBodyAuthoring</c> (3D, structure-only reference). It authors the package's runtime
    /// <see cref="PhysicsBody2DDefinition"/> (and, when a non-zero seed is set,
    /// <see cref="PhysicsBody2DInitialVelocity"/>) <em>directly</em>, without a built-in
    /// <c>Rigidbody2D</c> in between, so a user gains low-level control the built-in component cannot
    /// express as first-class inspector fields: an explicit <see cref="BodyType"/>, explicit
    /// <see cref="Mass"/> / <see cref="UseAutoMass"/>, per-DOF freeze <see cref="Constraints"/>, and an
    /// initial linear/angular velocity (which on a <c>Rigidbody2D</c> is runtime-only and cannot be baked).
    /// </summary>
    /// <remarks>
    /// Its baker (<c>PhysicsBody2DAuthoringBaker</c>, in the editor-only Baking assembly) emits the SAME
    /// runtime components <c>Rigidbody2DBaker</c> emits, so a body authored this way and an equivalent
    /// <c>Rigidbody2D</c>-authored body converge on one runtime archetype and one Box2D solver — the
    /// dual-surface convergence the design relies on. This component lives in a compiled package assembly
    /// (not only in <c>Samples~</c>) so it is unit-testable; the importable <c>Samples~/CustomAuthoring2D</c>
    /// sample references it and ships authored scenes that use it.
    ///
    /// <para><b>Deliberately omitted knobs.</b> The DOTS 3D sample also exposes <c>WorldIndex</c>,
    /// <c>SolverType</c>, and a custom inertia tensor. Those are NOT exposed here: world-index sharding
    /// needs the multi-world model the package defers, and a custom 2D solver/inertia override has no field
    /// on the current runtime archetype. Adding them would drag in deferred infrastructure for no current
    /// need (the design's negative-space rule). They are the natural extension when that infrastructure
    /// lands — an additive extension component, not a fork.</para>
    /// </remarks>
    [AddComponentMenu("Zori/Entities Physics 2D/Physics Body 2D")]
    [DisallowMultipleComponent]
    public sealed class PhysicsBody2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip(
            "Whether the body is fully simulated (Dynamic), moved directly (Kinematic), or fixed (Static)."
        )]
        PhysicsBody2DMotionType m_BodyType = PhysicsBody2DMotionType.Dynamic;

        [SerializeField]
        [Tooltip("Scales the amount of gravity applied to this body (Rigidbody2D.gravityScale).")]
        float m_GravityScale = 1f;

        [SerializeField]
        [Tooltip("Reduces linear velocity over time (Rigidbody2D.linearDamping).")]
        float m_LinearDamping;

        [SerializeField]
        [Tooltip("Reduces angular velocity over time (Rigidbody2D.angularDamping).")]
        float m_AngularDamping;

        [SerializeField]
        [Tooltip(
            "When false, the explicit Mass is applied; when true, mass is derived from the shape density "
                + "(Rigidbody2D.useAutoMass)."
        )]
        bool m_UseAutoMass;

        [SerializeField]
        [Tooltip("Explicit body mass, applied when UseAutoMass is false (Rigidbody2D.mass).")]
        float m_Mass = 1f;

        [SerializeField]
        [Tooltip("Freeze linear motion along the world X axis.")]
        bool m_FreezePositionX;

        [SerializeField]
        [Tooltip("Freeze linear motion along the world Y axis.")]
        bool m_FreezePositionY;

        [SerializeField]
        [Tooltip("Freeze rotation about the Z axis.")]
        bool m_FreezeRotation;

        [SerializeField]
        [Tooltip(
            "Initial world-space linear velocity (m/s). Baked into PhysicsBody2DInitialVelocity — on a "
                + "Rigidbody2D this is runtime-only and cannot be authored at bake time."
        )]
        float2 m_InitialLinearVelocity = float2.zero;

        [SerializeField]
        [Tooltip("Initial angular velocity (deg/s). Baked into PhysicsBody2DInitialVelocity.")]
        float m_InitialAngularVelocity;

        [SerializeField]
        [Tooltip(
            "Render-rate pose smoothing between fixed physics steps (Rigidbody2D.interpolation). None: the "
                + "rendered pose is the fixed-step pose. Interpolate: one step of render lag, smoothed between "
                + "the previous and current physics states. Extrapolate: predicted ahead using velocity."
        )]
        PhysicsBody2DInterpolation m_Interpolation = PhysicsBody2DInterpolation.None;

        [SerializeField]
        [Tooltip(
            "Continuous collision detection (Rigidbody2D.collisionDetectionMode). Continuous makes the body a "
                + "fast (bullet) body that does not tunnel a thin collider in one step; Discrete is the default."
        )]
        PhysicsCollisionDetection2D m_CollisionDetection = PhysicsCollisionDetection2D.Discrete;

        [SerializeField]
        [Tooltip(
            "Override the shape-derived center of mass and rotational inertia with explicit values (the 2D "
                + "analogue of the DOTS sample's OverrideDefaultMassDistribution). Off: mass distribution is "
                + "computed from the shapes' density."
        )]
        bool m_OverrideMassDistribution;

        [SerializeField]
        [Tooltip(
            "Explicit local-space center of mass, applied when OverrideMassDistribution is true."
        )]
        float2 m_CenterOfMass = float2.zero;

        [SerializeField]
        [Tooltip(
            "Explicit rotational inertia (kg·m², about the center of mass), applied when "
                + "OverrideMassDistribution is true and this is > 0. A 2D body has one rotational DOF, so its "
                + "inertia is a single scalar. 0 leaves the shape-derived inertia."
        )]
        float m_RotationalInertia;

        public PhysicsBody2DMotionType BodyType
        {
            get => m_BodyType;
            set => m_BodyType = value;
        }

        public float GravityScale
        {
            get => m_GravityScale;
            set => m_GravityScale = value;
        }

        public float LinearDamping
        {
            get => m_LinearDamping;
            set => m_LinearDamping = math.max(0f, value);
        }

        public float AngularDamping
        {
            get => m_AngularDamping;
            set => m_AngularDamping = math.max(0f, value);
        }

        public bool UseAutoMass
        {
            get => m_UseAutoMass;
            set => m_UseAutoMass = value;
        }

        public float Mass
        {
            get => m_Mass;
            set => m_Mass = math.max(0f, value);
        }

        public bool FreezePositionX
        {
            get => m_FreezePositionX;
            set => m_FreezePositionX = value;
        }

        public bool FreezePositionY
        {
            get => m_FreezePositionY;
            set => m_FreezePositionY = value;
        }

        public bool FreezeRotation
        {
            get => m_FreezeRotation;
            set => m_FreezeRotation = value;
        }

        public float2 InitialLinearVelocity
        {
            get => m_InitialLinearVelocity;
            set => m_InitialLinearVelocity = value;
        }

        public float InitialAngularVelocity
        {
            get => m_InitialAngularVelocity;
            set => m_InitialAngularVelocity = value;
        }

        /// <summary>Render-rate pose smoothing mode (<c>Rigidbody2D.interpolation</c>). Bakes to
        /// <see cref="PhysicsBody2DDefinition.interpolation"/>; a non-<c>None</c> body gains a
        /// <c>PhysicsBody2DSmoothing</c> component at creation.</summary>
        public PhysicsBody2DInterpolation Interpolation
        {
            get => m_Interpolation;
            set => m_Interpolation = value;
        }

        /// <summary>Continuous-collision mode (<c>Rigidbody2D.collisionDetectionMode</c>). Bakes to
        /// <see cref="PhysicsBody2DDefinition.fastCollisions"/>.</summary>
        public PhysicsCollisionDetection2D CollisionDetection
        {
            get => m_CollisionDetection;
            set => m_CollisionDetection = value;
        }

        /// <summary>When true the explicit <see cref="CenterOfMass"/> / <see cref="RotationalInertia"/> override
        /// the shape-derived mass distribution (a dynamic body only). Bakes to
        /// <see cref="PhysicsBody2DDefinition.overrideMassDistribution"/>.</summary>
        public bool OverrideMassDistribution
        {
            get => m_OverrideMassDistribution;
            set => m_OverrideMassDistribution = value;
        }

        /// <summary>The explicit local-space center of mass, applied when
        /// <see cref="OverrideMassDistribution"/> is true.</summary>
        public float2 CenterOfMass
        {
            get => m_CenterOfMass;
            set => m_CenterOfMass = value;
        }

        /// <summary>The explicit rotational inertia (kg·m²), applied when
        /// <see cref="OverrideMassDistribution"/> is true and the value is &gt; 0.</summary>
        public float RotationalInertia
        {
            get => m_RotationalInertia;
            set => m_RotationalInertia = math.max(0f, value);
        }

        /// <summary>True when a non-zero velocity seed is authored, so the baker only emits the optional
        /// <see cref="PhysicsBody2DInitialVelocity"/> when it is meaningful.</summary>
        public bool HasInitialVelocity =>
            !m_InitialLinearVelocity.Equals(float2.zero) || m_InitialAngularVelocity != 0f;

        void OnValidate()
        {
            m_LinearDamping = math.max(0f, m_LinearDamping);
            m_AngularDamping = math.max(0f, m_AngularDamping);
            m_Mass = math.max(0f, m_Mass);
            m_RotationalInertia = math.max(0f, m_RotationalInertia);
        }
    }
}
