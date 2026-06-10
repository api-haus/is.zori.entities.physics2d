using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Which built-in 2D joint a <see cref="PhysicsJoint2DDefinition"/> carries, mapped onto a Box2D joint
    /// definition at creation. Phase 2A shipped Hinge / Slider / Wheel; Phase 2B adds the rest, each one
    /// enum value plus one creation-switch arm and one baker — the same negative-space property the
    /// <see cref="PhysicsShape2DKind"/> union has. Several built-in joints share a Box2D kind: a rigid
    /// <see cref="Distance"/> and an oscillating <see cref="Spring"/> are both the Box2D
    /// <c>PhysicsDistanceJoint</c> (the spring enable flag distinguishes them); <see cref="Relative"/>,
    /// <see cref="Friction"/>, and <see cref="Target"/> are all the Box2D <c>PhysicsRelativeJoint</c>
    /// (offset-tracking, zero-offset friction, and world-target respectively). The enum carries the
    /// <em>built-in</em> identity, not the Box2D kind, so each baker keeps its 1:1 source mapping.
    /// </summary>
    public enum PhysicsJoint2DKind : byte
    {
        Hinge,
        Slider,
        Wheel,
        Distance,
        Spring,
        Fixed,
        Relative,
        Friction,
        Target,
    }

    /// <summary>
    /// What happens when a joint's reaction force/torque exceeds its baked break threshold, mirroring
    /// <c>UnityEngine.JointBreakAction2D</c>. The package sets the native Box2D
    /// <c>forceThreshold</c>/<c>torqueThreshold</c> so the engine fires a joint-threshold event the same step
    /// the load is exceeded, then applies this action.
    /// </summary>
    public enum PhysicsJointBreakAction2D : byte
    {
        /// <summary><c>JointBreakAction2D.Ignore</c> — never breaks (no threshold is set, no event fires).</summary>
        Ignore,

        /// <summary><c>JointBreakAction2D.CallbackOnly</c> — surface a <c>PhysicsJointBreakEvent2D</c> when the
        /// threshold is exceeded but KEEP the joint (the constraint still holds), matching the built-in callback-
        /// only action where <c>OnJointBreak2D</c> fires but the joint is not destroyed.</summary>
        CallbackOnly,

        /// <summary><c>JointBreakAction2D.Destroy</c> — surface the event and destroy the Box2D joint; the bodies
        /// separate. (The built-in also destroys the component; the package destroys the constraint and surfaces
        /// the action so a consumer can mirror the component-destroy on its side.)</summary>
        Destroy,

        /// <summary><c>JointBreakAction2D.Disable</c> — surface the event and destroy the Box2D constraint (Box2D
        /// has no disable-joint primitive, so the bodies separate as with Destroy); the action is carried in the
        /// surfaced event so a consumer can mirror the built-in disable-the-component semantics.</summary>
        Disable,
    }

    /// <summary>
    /// The baked joint authoring, consumed exactly once when the joint-creation system creates the Box2D
    /// joint. A single tagged-union component carries every built-in 2D joint parameter Phase 2A maps —
    /// anchors, the slide/suspension axis, the motor, the limits, the spring — discriminated by
    /// <see cref="kind"/>, plus an <see cref="Entity"/> reference to the connected body the baker resolves
    /// from <c>Joint2D.connectedBody</c> via <c>GetEntity</c>. The connected entity is the second body
    /// (Box2D <c>bodyA</c>); the entity carrying this component is the joint-owner body (Box2D
    /// <c>bodyB</c>), exactly the built-in convention where the joint component rides on <c>bodyB</c> and
    /// <c>connectedBody</c> is <c>bodyA</c>.
    /// </summary>
    /// <remarks>
    /// All fields are blittable. Anchors are stored as <see cref="float2"/> body-local offsets (the
    /// built-in <c>anchor</c>/<c>connectedAnchor</c> are already each body's local space), folded into the
    /// Box2D <c>localAnchorA</c>/<c>localAnchorB</c> frames at creation; the slide/suspension axis is stored
    /// as <see cref="axisAngleRadians"/> and folded into <c>localAnchorA</c>'s rotation (the frame whose
    /// local X is the slide/suspension direction). Motor/limit/spring enables are bools; their parameters
    /// keep meaningful values even when disabled so toggling on at runtime needs no re-bake (Phase 2A does
    /// not expose that, but the component is shaped for it).
    ///
    /// <para>A joint references a <em>second</em> body, so unlike a body/shape it cannot be created until the
    /// owner entity (and, for a concrete connected body, that entity) carries a live
    /// <see cref="PhysicsBody2D"/> handle — the creation is deferred (see <c>PhysicsJoint2DCreationSystem</c>).
    /// The connected entity bakes to <see cref="Entity.Null"/> when the built-in <c>connectedBody</c> is null,
    /// which built-in 2D physics treats as a joint to a point in space (a static world anchor); the creation
    /// system supplies a shared static anchor body at the origin for that case, and
    /// <see cref="connectedAnchor"/> is then a WORLD-space point. This is the pendulum-pinned-to-the-world
    /// fixture form Phase 2A uses, and it keeps the parity harness body counts symmetric (no extra
    /// <c>Rigidbody2D</c> on the anchor side).</para>
    /// </remarks>
    public struct PhysicsJoint2DDefinition : IComponentData
    {
        /// <summary>The discriminant. Decides which Box2D joint definition the creation system builds.</summary>
        public PhysicsJoint2DKind kind;

        /// <summary>
        /// The connected body (built-in <c>Joint2D.connectedBody</c>) → Box2D <c>bodyA</c>. The entity
        /// carrying this component is <c>bodyB</c>. <see cref="Entity.Null"/> means the built-in joint had
        /// no connected body (an implicit static world anchor) — not exercised in Phase 2A.
        /// </summary>
        public Entity connectedBody;

        /// <summary>
        /// <c>AnchoredJoint2D.anchor</c> — the anchor on the joint-owner body (this entity, <c>bodyB</c>),
        /// in that body's local space → Box2D <c>localAnchorB</c> position.
        /// </summary>
        public float2 anchor;

        /// <summary>
        /// <c>AnchoredJoint2D.connectedAnchor</c> — the anchor on the connected body (<c>bodyA</c>), in that
        /// body's local space → Box2D <c>localAnchorA</c> position. When the built-in joint
        /// auto-configures the connected anchor, the resolved value is read at bake.
        /// </summary>
        public float2 connectedAnchor;

        /// <summary>
        /// Slider/Wheel: the slide/suspension axis angle in DEGREES (<c>SliderJoint2D.angle</c> /
        /// <c>WheelJoint2D.suspension.angle</c> are both "in degrees" world angles), folded into
        /// <c>localAnchorA</c>'s rotation at creation (via <c>PhysicsRotate.FromDegrees</c>) so the frame's
        /// local X is the slide/suspension direction. Stored in the source unit (degrees) and converted at
        /// the single creation call. Unused for Hinge (a hinge has no translation axis). The built-in angle
        /// is a WORLD angle; against a static, unrotated <c>bodyA</c> (the Phase 2A fixtures) the world and
        /// body-A-relative axis coincide — a rotated connected body is a Phase-2B refinement.
        /// </summary>
        public float axisAngleDegrees;

        // --- Motor (Hinge/Slider/Wheel all have one; the max-effort field is torque for Hinge/Wheel, force
        // for Slider). ---

        /// <summary><c>useMotor</c> → enable the joint motor.</summary>
        public bool enableMotor;

        /// <summary>
        /// <c>JointMotor2D.motorSpeed</c> — target motor speed. Hinge: deg/sec about the hinge. Slider:
        /// units/sec along the axis. Wheel: deg/sec about the wheel. Carried verbatim from the built-in
        /// motor struct (the Box2D motor-speed unit matches the built-in joint's per type).
        /// </summary>
        public float motorSpeed;

        /// <summary>
        /// <c>JointMotor2D.maxMotorTorque</c> — the motor's maximum effort. Mapped to
        /// <c>maxMotorTorque</c> for Hinge/Wheel and <c>maxMotorForce</c> for Slider (the built-in stores
        /// both under the same struct field; the joint type decides which Box2D field it feeds).
        /// </summary>
        public float maxMotorEffort;

        // --- Limit (Hinge: angle; Slider/Wheel: translation). ---

        /// <summary><c>useLimits</c> → clamp the joint's angle (Hinge) or translation (Slider/Wheel).</summary>
        public bool enableLimit;

        /// <summary>
        /// Lower limit. Hinge: <c>JointAngleLimits2D.min</c> in DEGREES → <c>lowerAngleLimit</c> (the engine
        /// hinge limit is documented "in degrees", module XML, so this stores and feeds degrees unchanged — see
        /// the angular convention in <c>Documentation~/parity-matrix.md</c>). Slider/Wheel:
        /// <c>JointTranslationLimits2D.min</c> in meters → <c>lowerTranslationLimit</c>.
        /// </summary>
        public float lowerLimit;

        /// <summary>Upper limit, the counterpart of <see cref="lowerLimit"/> → <c>upper*Limit</c> (Hinge:
        /// degrees; Slider/Wheel: meters).</summary>
        public float upperLimit;

        // --- Spring (Hinge: angular spring; Wheel: suspension spring; Slider supports one too). ---

        /// <summary>
        /// Enable the joint spring. Hinge: <c>HingeJoint2D.useSpring</c> is not a built-in field, so a
        /// hinge bakes this false. Wheel: always true (a wheel's suspension is its defining spring).
        /// Slider: not exposed by the built-in component, baked false.
        /// </summary>
        public bool enableSpring;

        /// <summary>
        /// Spring frequency, Hz. Wheel: <c>JointSuspension2D.frequency</c> → <c>springFrequency</c>. Unused
        /// when <see cref="enableSpring"/> is false.
        /// </summary>
        public float springFrequency;

        /// <summary>
        /// Spring damping ratio. Wheel: <c>JointSuspension2D.dampingRatio</c> → <c>springDamping</c>. Unused
        /// when <see cref="enableSpring"/> is false.
        /// </summary>
        public float springDamping;

        /// <summary>
        /// Whether the two jointed bodies' shapes collide with each other (<c>Joint2D.enableCollision</c> →
        /// Box2D <c>collideConnected</c>). Built-in default is false (jointed bodies pass through each other).
        /// </summary>
        public bool collideConnected;

        // --- Distance / Spring (Box2D PhysicsDistanceJoint). The two anchors are held at a rest length;
        // a rigid Distance keeps that length exactly, a Spring oscillates toward it (springFrequency/Damping
        // above are reused for the distance spring). ---

        /// <summary>
        /// The rest length the distance constraint holds between the two anchors.
        /// <c>DistanceJoint2D.distance</c> / <c>SpringJoint2D.distance</c> → Box2D
        /// <c>PhysicsDistanceJointDefinition.distance</c>. Unused for non-distance kinds.
        /// </summary>
        public float restLength;

        // --- Fixed (Box2D PhysicsFixedJoint). Locks the two bodies in their relative pose. The built-in
        // FixedJoint2D exposes a single frequency/dampingRatio that feeds BOTH the Box2D linear and angular
        // stiffness; a frequency of zero is the rigid (maximum-stiffness) lock. Stored in the shared
        // springFrequency/springDamping fields above, applied to both linear and angular at creation. ---

        // --- Relative / Friction / Target (Box2D PhysicsRelativeJoint). Maintains a relative offset/pose
        // between the two bodies (Relative tracks a target offset; Friction is a zero-offset relative joint
        // whose force/torque caps make relative motion damp out; Target drives the owner body toward a world
        // point via a static world anchor). ---

        /// <summary>
        /// The maintained linear offset of <c>bodyB</c> relative to <c>bodyA</c> (<c>RelativeJoint2D.linearOffset</c>),
        /// folded into the Box2D relative-joint anchor frames at creation. Zero for Friction (a relative joint
        /// with no offset, so it only damps relative motion). Unused for non-relative kinds.
        /// </summary>
        public float2 linearOffset;

        /// <summary>
        /// The maintained angular offset of <c>bodyB</c> relative to <c>bodyA</c>, in DEGREES
        /// (<c>RelativeJoint2D.angularOffset</c>). Zero for Friction. Unused for non-relative kinds.
        /// </summary>
        public float angularOffsetDegrees;

        /// <summary>
        /// The relative joint's maximum linear correction force, usually newtons
        /// (<c>RelativeJoint2D.maxForce</c> / <c>FrictionJoint2D.maxForce</c> / <c>TargetJoint2D.maxForce</c>)
        /// → Box2D <c>PhysicsRelativeJointDefinition.maxForce</c>. Zero turns the force limit off (Box2D
        /// special case). Unused for non-relative kinds.
        /// </summary>
        public float maxForce;

        /// <summary>
        /// The relative joint's maximum angular correction torque, usually newton-metres
        /// (<c>RelativeJoint2D.maxTorque</c> / <c>FrictionJoint2D.maxTorque</c>) → Box2D
        /// <c>PhysicsRelativeJointDefinition.maxTorque</c>. Zero turns the torque limit off. Target has no
        /// torque cap (it constrains a single anchor point), so it bakes zero. Unused for non-relative kinds.
        /// </summary>
        public float maxTorque;

        // --- Break (every *Joint2D shares Joint2D.breakForce/breakTorque/breakAction). ---

        /// <summary>
        /// <c>Joint2D.breakForce</c> — the reaction force that breaks the joint. Set into the native Box2D
        /// <c>PhysicsJoint.forceThreshold</c> ONLY when finite (the built-in default is <c>Infinity</c> =
        /// never break) and the action is not <see cref="PhysicsJointBreakAction2D.Ignore"/>. When exceeded,
        /// Box2D fires a joint-threshold event and the package applies <see cref="breakAction"/>.
        /// </summary>
        public float breakForce;

        /// <summary>
        /// <c>Joint2D.breakTorque</c> — the reaction torque that breaks the joint. Set into the native Box2D
        /// <c>PhysicsJoint.torqueThreshold</c> ONLY when finite (default <c>Infinity</c>). Counterpart of
        /// <see cref="breakForce"/>.
        /// </summary>
        public float breakTorque;

        /// <summary>
        /// <c>Joint2D.breakAction</c> mapped to <see cref="PhysicsJointBreakAction2D"/> — what the package does
        /// when the threshold is exceeded (never break / surface only / destroy the constraint).
        /// </summary>
        public PhysicsJointBreakAction2D breakAction;
    }
}
