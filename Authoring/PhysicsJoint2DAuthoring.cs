using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Authoring
{
    /// <summary>
    /// The customisable joint authoring component — the 2D analogue of <c>com.unity.physics</c>'s joint
    /// MonoBehaviour family (3D, structure-only reference), and the DOTS-native alternative to the built-in
    /// <c>UnityEngine.*Joint2D</c> components, exactly as <see cref="PhysicsBody2DAuthoring"/> /
    /// <see cref="PhysicsShape2DAuthoring"/> are the custom alternative to <c>Rigidbody2D</c> /
    /// <c>Collider2D</c>. It authors the package's runtime <see cref="PhysicsJoint2DDefinition"/>
    /// <em>directly</em>, carrying the <see cref="PhysicsJoint2DKind"/> selector and the full union of joint
    /// parameters, without a built-in <c>*Joint2D</c> in between.
    /// </summary>
    /// <remarks>
    /// <para><b>Unified, not per-type.</b> Where the 3D sample ships one MonoBehaviour per joint type over a
    /// <c>BaseJoint</c> (BallAndSocket / FreeHinge / LimitedHinge / …), this is ONE component with a
    /// <see cref="Kind"/> selector — the 2D package's own convention (<see cref="PhysicsShape2DAuthoring"/>
    /// has a shape-kind selector, <see cref="PhysicsBody2DAuthoring"/> is one body component) and the shape of
    /// the runtime joint def, which is itself a single tagged union over <see cref="PhysicsJoint2DKind"/>. The
    /// unused fields for a given kind are inert, exactly as the shape component's unused geometry fields are.</para>
    ///
    /// <para><b>Convergence.</b> Its baker (<c>PhysicsJoint2DAuthoringBaker</c>) emits the SAME
    /// <see cref="PhysicsJoint2DDefinition"/> the built-in <c>*Joint2DBaker</c> emits for the equivalent
    /// <c>*Joint2D</c> of the same parameters, so a custom-authored joint and the built-in joint converge on
    /// one runtime joint and one Box2D constraint — the dual-surface convergence the design relies on. The
    /// runtime joint def + the creation system are reused unchanged; this is a second authoring surface, not a
    /// second runtime model.</para>
    ///
    /// <para><b>Connected body + world anchor.</b> <see cref="ConnectedBody"/> is the joint's second body
    /// (Box2D <c>bodyA</c>); the GameObject carrying this component is the joint-owner body (<c>bodyB</c>),
    /// matching the built-in convention where the joint rides on <c>bodyB</c> and <c>connectedBody</c> is
    /// <c>bodyA</c>. A null <see cref="ConnectedBody"/> bakes to <see cref="Unity.Entities.Entity.Null"/>,
    /// which the creation system resolves to a shared static world anchor at the origin — the built-in "joint
    /// to a point in space", and the path the <see cref="PhysicsJoint2DKind.Target"/> joint always takes.</para>
    ///
    /// <para><b>Angular convention.</b> Joint angle limits (<see cref="LowerLimit"/>/<see cref="UpperLimit"/>
    /// for a Hinge), the slide/suspension <see cref="AxisAngle"/>, the motor <see cref="MotorSpeed"/> for a
    /// Hinge/Wheel, and the relative <see cref="AngularOffset"/> are all in DEGREES, matching the built-in
    /// <c>*Joint2D</c> and the runtime (the creation system folds them to radians at the single creation call).</para>
    /// </remarks>
    [AddComponentMenu("Zori/Entities Physics 2D/Physics Joint 2D")]
    public sealed class PhysicsJoint2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip(
            "Which joint this authors. Decides which per-type sub-surface (motor / limit / spring / "
                + "offset) is meaningful; the others are inert."
        )]
        PhysicsJoint2DKind m_Kind = PhysicsJoint2DKind.Hinge;

        [SerializeField]
        [Tooltip(
            "The connected body (the joint's second body, Box2D bodyA). The GameObject carrying THIS "
                + "component is the joint owner (bodyB). Leave null for a joint to a point in the world (a static "
                + "world anchor) — the path the Target joint always takes."
        )]
        PhysicsBody2DAuthoring m_ConnectedBody;

        [SerializeField]
        [Tooltip(
            "Anchor on the joint-owner body (this GameObject, bodyB), in that body's local space "
                + "(AnchoredJoint2D.anchor)."
        )]
        float2 m_Anchor = float2.zero;

        [SerializeField]
        [Tooltip(
            "Anchor on the connected body (bodyA), in that body's local space "
                + "(AnchoredJoint2D.connectedAnchor). For a Target joint this is the WORLD-space target point."
        )]
        float2 m_ConnectedAnchor = float2.zero;

        [SerializeField]
        [Tooltip(
            "Slider / Wheel: the slide / suspension axis angle in DEGREES (SliderJoint2D.angle / "
                + "WheelJoint2D.suspension.angle). Unused for the other kinds."
        )]
        float m_AxisAngle;

        [SerializeField]
        [Tooltip("Hinge / Slider / Wheel: enable the joint motor (useMotor).")]
        bool m_UseMotor;

        [SerializeField]
        [Tooltip(
            "Motor target speed. Hinge / Wheel: degrees per second. Slider: metres per second "
                + "(JointMotor2D.motorSpeed)."
        )]
        float m_MotorSpeed;

        [SerializeField]
        [Tooltip(
            "Motor maximum effort: torque (N·m) for a Hinge / Wheel, force (N) for a Slider "
                + "(JointMotor2D.maxMotorTorque / maxMotorForce)."
        )]
        float m_MaxMotorEffort;

        [SerializeField]
        [Tooltip(
            "Hinge / Slider: enable the joint limit (useLimits). A Wheel has no built-in translation "
                + "limit, so the toggle is inert for it."
        )]
        bool m_UseLimits;

        [SerializeField]
        [Tooltip(
            "Lower limit. Hinge: angle in DEGREES (JointAngleLimits2D.min). Slider: translation in "
                + "metres (JointTranslationLimits2D.min)."
        )]
        float m_LowerLimit;

        [SerializeField]
        [Tooltip("Upper limit. Hinge: angle in DEGREES. Slider: translation in metres.")]
        float m_UpperLimit;

        [SerializeField]
        [Tooltip(
            "Spring / Wheel / Target / Fixed: spring frequency in Hz (the suspension / spring "
                + "stiffness). A Distance joint is rigid (no spring); a Fixed joint of frequency 0 is a rigid weld."
        )]
        float m_Frequency = 1f;

        [SerializeField]
        [Tooltip(
            "Spring / Wheel / Target / Fixed: spring damping ratio (non-dimensional). 1 is critically " + "damped."
        )]
        float m_DampingRatio = 1f;

        [SerializeField]
        [Tooltip(
            "Distance / Spring: the rest length the constraint holds between the two anchors "
                + "(DistanceJoint2D.distance / SpringJoint2D.distance)."
        )]
        float m_RestLength = 1f;

        [SerializeField]
        [Tooltip(
            "Relative: the maintained linear offset of bodyB relative to bodyA "
                + "(RelativeJoint2D.linearOffset), in metres. Zero for Friction / Target."
        )]
        float2 m_LinearOffset = float2.zero;

        [SerializeField]
        [Tooltip(
            "Relative: the maintained angular offset of bodyB relative to bodyA "
                + "(RelativeJoint2D.angularOffset), in DEGREES. Zero for Friction / Target."
        )]
        float m_AngularOffset;

        [SerializeField]
        [Tooltip(
            "Relative / Friction / Target: the maximum linear correction force (N). Zero turns the "
                + "force cap off (Box2D special case)."
        )]
        float m_MaxForce;

        [SerializeField]
        [Tooltip(
            "Relative / Friction: the maximum angular correction torque (N·m). Zero turns the torque "
                + "cap off. Target has no torque cap (a point constraint)."
        )]
        float m_MaxTorque;

        [SerializeField]
        [Tooltip(
            "Whether the two jointed bodies' shapes collide with each other (Joint2D.enableCollision → "
                + "Box2D collideConnected). Default false: jointed bodies pass through each other."
        )]
        bool m_CollideConnected;

        [SerializeField]
        [Tooltip(
            "What happens when the joint's reaction force / torque exceeds the break threshold. Default "
                + "Destroy matches the built-in Joint2D.breakAction default; with the Infinity break thresholds "
                + "(also the built-in default) the threshold is never reached, so a default joint never breaks "
                + "regardless of the action — the action only matters once a finite break force / torque is set."
        )]
        PhysicsJointBreakAction2D m_BreakAction = PhysicsJointBreakAction2D.Destroy;

        [SerializeField]
        [Tooltip(
            "Reaction force that breaks the joint (Joint2D.breakForce). Infinity (the default) never "
                + "breaks. Armed only when finite AND the break action is not Ignore."
        )]
        float m_BreakForce = float.PositiveInfinity;

        [SerializeField]
        [Tooltip(
            "Reaction torque that breaks the joint (Joint2D.breakTorque). Infinity (the default) never " + "breaks."
        )]
        float m_BreakTorque = float.PositiveInfinity;

        /// <summary>Which joint this authors. Decides the per-type sub-surface; the others are inert.</summary>
        public PhysicsJoint2DKind Kind
        {
            get => m_Kind;
            set => m_Kind = value;
        }

        /// <summary>The connected body (Box2D <c>bodyA</c>); the owner GameObject is <c>bodyB</c>. Null is a
        /// joint to a static world anchor at the origin (the built-in null-<c>connectedBody</c> path, and the
        /// Target joint's only path).</summary>
        public PhysicsBody2DAuthoring ConnectedBody
        {
            get => m_ConnectedBody;
            set => m_ConnectedBody = value;
        }

        /// <summary>Anchor on the owner body (<c>bodyB</c>), in that body's local space.</summary>
        public float2 Anchor
        {
            get => m_Anchor;
            set => m_Anchor = value;
        }

        /// <summary>Anchor on the connected body (<c>bodyA</c>), body-local. For a Target joint, the
        /// world-space target point.</summary>
        public float2 ConnectedAnchor
        {
            get => m_ConnectedAnchor;
            set => m_ConnectedAnchor = value;
        }

        /// <summary>Slider / Wheel slide / suspension axis angle in <b>degrees</b>. Inert for the other
        /// kinds.</summary>
        public float AxisAngle
        {
            get => m_AxisAngle;
            set => m_AxisAngle = value;
        }

        /// <summary>Enable the joint motor (Hinge / Slider / Wheel).</summary>
        public bool UseMotor
        {
            get => m_UseMotor;
            set => m_UseMotor = value;
        }

        /// <summary>Motor target speed: deg/s for Hinge / Wheel, m/s for Slider.</summary>
        public float MotorSpeed
        {
            get => m_MotorSpeed;
            set => m_MotorSpeed = value;
        }

        /// <summary>Motor maximum effort: torque for Hinge / Wheel, force for Slider.</summary>
        public float MaxMotorEffort
        {
            get => m_MaxMotorEffort;
            set => m_MaxMotorEffort = math.max(0f, value);
        }

        /// <summary>Enable the joint limit (Hinge angle / Slider translation). Inert for a Wheel (no built-in
        /// translation limit).</summary>
        public bool UseLimits
        {
            get => m_UseLimits;
            set => m_UseLimits = value;
        }

        /// <summary>Lower limit: Hinge angle in <b>degrees</b>, Slider translation in metres.</summary>
        public float LowerLimit
        {
            get => m_LowerLimit;
            set => m_LowerLimit = value;
        }

        /// <summary>Upper limit: Hinge angle in <b>degrees</b>, Slider translation in metres.</summary>
        public float UpperLimit
        {
            get => m_UpperLimit;
            set => m_UpperLimit = value;
        }

        /// <summary>Spring frequency (Hz) for Wheel / Spring / Target / Fixed. A Fixed joint of frequency 0
        /// is a rigid weld.</summary>
        public float Frequency
        {
            get => m_Frequency;
            set => m_Frequency = math.max(0f, value);
        }

        /// <summary>Spring damping ratio (non-dimensional) for Wheel / Spring / Target / Fixed.</summary>
        public float DampingRatio
        {
            get => m_DampingRatio;
            set => m_DampingRatio = math.max(0f, value);
        }

        /// <summary>Distance / Spring rest length the constraint holds between the two anchors.</summary>
        public float RestLength
        {
            get => m_RestLength;
            set => m_RestLength = math.max(0f, value);
        }

        /// <summary>Relative joint maintained linear offset of <c>bodyB</c> w.r.t. <c>bodyA</c>, metres.</summary>
        public float2 LinearOffset
        {
            get => m_LinearOffset;
            set => m_LinearOffset = value;
        }

        /// <summary>Relative joint maintained angular offset of <c>bodyB</c> w.r.t. <c>bodyA</c>, <b>degrees</b>.</summary>
        public float AngularOffset
        {
            get => m_AngularOffset;
            set => m_AngularOffset = value;
        }

        /// <summary>Relative / Friction / Target maximum linear correction force (N). Zero turns the cap off.</summary>
        public float MaxForce
        {
            get => m_MaxForce;
            set => m_MaxForce = math.max(0f, value);
        }

        /// <summary>Relative / Friction maximum angular correction torque (N·m). Zero turns the cap off.</summary>
        public float MaxTorque
        {
            get => m_MaxTorque;
            set => m_MaxTorque = math.max(0f, value);
        }

        /// <summary>Whether the two jointed bodies' shapes collide with each other (Box2D
        /// <c>collideConnected</c>).</summary>
        public bool CollideConnected
        {
            get => m_CollideConnected;
            set => m_CollideConnected = value;
        }

        /// <summary>What the package does when the break threshold is exceeded. Ignore (default): never break.</summary>
        public PhysicsJointBreakAction2D BreakAction
        {
            get => m_BreakAction;
            set => m_BreakAction = value;
        }

        /// <summary>Reaction force that breaks the joint. Infinity (default) never breaks.</summary>
        public float BreakForce
        {
            get => m_BreakForce;
            set => m_BreakForce = value;
        }

        /// <summary>Reaction torque that breaks the joint. Infinity (default) never breaks.</summary>
        public float BreakTorque
        {
            get => m_BreakTorque;
            set => m_BreakTorque = value;
        }

        void OnValidate()
        {
            m_MaxMotorEffort = math.max(0f, m_MaxMotorEffort);
            m_Frequency = math.max(0f, m_Frequency);
            m_DampingRatio = math.max(0f, m_DampingRatio);
            m_RestLength = math.max(0f, m_RestLength);
            m_MaxForce = math.max(0f, m_MaxForce);
            m_MaxTorque = math.max(0f, m_MaxTorque);
        }
    }
}
