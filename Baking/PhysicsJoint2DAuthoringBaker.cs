using Unity.Entities;
using Unity.Mathematics;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes the customisable <see cref="PhysicsJoint2DAuthoring"/> into the SAME
    /// <see cref="PhysicsJoint2DDefinition"/> the built-in <c>*Joint2DBaker</c> family produces, reading the
    /// joint parameters from the authoring component's inline fields instead of a built-in <c>*Joint2D</c>.
    /// A joint authored this way is indistinguishable at runtime from the same joint authored on the built-in
    /// component — the convergence the dual surface relies on, matching the body/shape custom-authoring path.
    /// </summary>
    /// <remarks>
    /// The per-kind switch selects EXACTLY the fields the corresponding built-in baker selects (see
    /// <c>HingeJoint2DBaker</c> … <c>TargetJoint2DBaker</c>), so a custom Hinge of given parameters bakes a
    /// <see cref="PhysicsJoint2DDefinition"/> field-identical (modulo the connected-body entity reference) to
    /// the built-in <c>HingeJoint2D</c> baker's output for the same parameters. The runtime joint def + the
    /// creation system are reused unchanged. Editor-only assembly, so this never reaches a player build.
    ///
    /// <para><b>enableSpring is DERIVED, not authored.</b> The built-in <c>*Joint2D</c> components do not
    /// expose a uniform spring toggle (a Wheel's suspension is always a spring; a Distance is always rigid; a
    /// Spring is always sprung; a Fixed carries frequency/damping with <c>enableSpring=false</c> and the
    /// creation system applies them to both linear and angular stiffness; a Relative uses a hardcoded stiff
    /// critically-damped spring). The baker sets <c>enableSpring</c> per kind to match each built-in baker
    /// exactly, so the custom surface cannot author a state with no built-in equivalent and the convergence
    /// holds.</para>
    /// </remarks>
    public sealed class PhysicsJoint2DAuthoringBaker : Baker<PhysicsJoint2DAuthoring>
    {
        public override void Bake(PhysicsJoint2DAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            // The shared surface every kind carries: the connected body (bodyA), the two body-local anchors,
            // collideConnected, and the break threshold/action. The custom surface authors the package
            // PhysicsJointBreakAction2D directly, so unlike the built-in bakers there is no JointBreakAction2D
            // mapping — the value is baked verbatim.
            var def = new PhysicsJoint2DDefinition
            {
                kind = authoring.Kind,
                connectedBody = ResolveConnectedBody(authoring),
                anchor = authoring.Anchor,
                connectedAnchor = authoring.ConnectedAnchor,
                collideConnected = authoring.CollideConnected,
                breakForce = authoring.BreakForce,
                breakTorque = authoring.BreakTorque,
                breakAction = authoring.BreakAction,
            };

            // The per-kind sub-surface, selecting the same fields the matching built-in baker selects so the
            // inert fields (e.g. a Hinge's axisAngleDegrees / restLength / linearOffset) bake to the same
            // zero/disabled defaults the built-in baker leaves them at.
            switch (authoring.Kind)
            {
                case PhysicsJoint2DKind.Hinge:
                    // HingeJoint2DBaker: a hinge constrains rotation only — no axis. Motor (deg/s + maxMotorTorque)
                    // and angle limit (degrees) on; no spring.
                    def.axisAngleDegrees = 0f;
                    def.enableMotor = authoring.UseMotor;
                    def.motorSpeed = authoring.MotorSpeed;
                    def.maxMotorEffort = authoring.MaxMotorEffort;
                    def.enableLimit = authoring.UseLimits;
                    def.lowerLimit = authoring.LowerLimit;
                    def.upperLimit = authoring.UpperLimit;
                    def.enableSpring = false;
                    break;

                case PhysicsJoint2DKind.Slider:
                    // SliderJoint2DBaker: slide axis (degrees), linear motor (m/s + maxMotorTorque→maxMotorForce),
                    // translation limit (metres); no spring.
                    def.axisAngleDegrees = authoring.AxisAngle;
                    def.enableMotor = authoring.UseMotor;
                    def.motorSpeed = authoring.MotorSpeed;
                    def.maxMotorEffort = authoring.MaxMotorEffort;
                    def.enableLimit = authoring.UseLimits;
                    def.lowerLimit = authoring.LowerLimit;
                    def.upperLimit = authoring.UpperLimit;
                    def.enableSpring = false;
                    break;

                case PhysicsJoint2DKind.Wheel:
                    // WheelJoint2DBaker: suspension axis (degrees), rotational motor, NO translation limit (the
                    // built-in WheelJoint2D has no limit field), suspension spring ALWAYS on.
                    def.axisAngleDegrees = authoring.AxisAngle;
                    def.enableMotor = authoring.UseMotor;
                    def.motorSpeed = authoring.MotorSpeed;
                    def.maxMotorEffort = authoring.MaxMotorEffort;
                    def.enableLimit = false;
                    def.lowerLimit = 0f;
                    def.upperLimit = 0f;
                    def.enableSpring = true;
                    def.springFrequency = authoring.Frequency;
                    def.springDamping = authoring.DampingRatio;
                    break;

                case PhysicsJoint2DKind.Distance:
                    // DistanceJoint2DBaker: rigid (no spring), the rest length the constraint holds.
                    def.restLength = authoring.RestLength;
                    def.enableSpring = false;
                    break;

                case PhysicsJoint2DKind.Spring:
                    // SpringJoint2DBaker: a distance joint whose spring is ALWAYS on; rest length + spring freq/damp.
                    def.restLength = authoring.RestLength;
                    def.enableSpring = true;
                    def.springFrequency = authoring.Frequency;
                    def.springDamping = authoring.DampingRatio;
                    break;

                case PhysicsJoint2DKind.Fixed:
                    // FixedJoint2DBaker: enableSpring FALSE; the frequency/damping ride in the shared spring
                    // fields and the creation system feeds BOTH the linear and angular Box2D stiffness (frequency
                    // 0 = rigid weld).
                    def.enableSpring = false;
                    def.springFrequency = authoring.Frequency;
                    def.springDamping = authoring.DampingRatio;
                    break;

                case PhysicsJoint2DKind.Relative:
                    // RelativeJoint2DBaker: no anchors (the constraint is on the body origins); the maintained
                    // offset + force/torque caps; a hardcoded stiff critically-damped spring approximates the
                    // built-in's rigid pull toward the offset (the built-in RelativeJoint2D has no frequency knob).
                    def.anchor = float2.zero;
                    def.connectedAnchor = float2.zero;
                    def.linearOffset = authoring.LinearOffset;
                    def.angularOffsetDegrees = authoring.AngularOffset;
                    def.maxForce = authoring.MaxForce;
                    def.maxTorque = authoring.MaxTorque;
                    def.enableSpring = true;
                    def.springFrequency = 8f; // stiff position controller (the built-in has no frequency to map)
                    def.springDamping = 1f; // critically damped → converges to the offset without overshoot
                    break;

                case PhysicsJoint2DKind.Friction:
                    // FrictionJoint2DBaker: a relative joint with a ZERO offset and no spring — pure velocity-
                    // control friction capped by maxForce/maxTorque.
                    def.anchor = float2.zero;
                    def.connectedAnchor = float2.zero;
                    def.linearOffset = float2.zero;
                    def.angularOffsetDegrees = 0f;
                    def.maxForce = authoring.MaxForce;
                    def.maxTorque = authoring.MaxTorque;
                    def.enableSpring = false;
                    break;

                case PhysicsJoint2DKind.Target:
                    // TargetJoint2DBaker: a SINGLE-body joint — no connected body (the static world anchor); the
                    // body-local anchor + the world-space target (connectedAnchor), a linear position spring
                    // (frequency/damping) pulling toward the target, force cap, NO torque cap.
                    def.connectedBody = Entity.Null;
                    def.linearOffset = float2.zero;
                    def.angularOffsetDegrees = 0f;
                    def.maxForce = authoring.MaxForce;
                    def.maxTorque = 0f;
                    def.enableSpring = true;
                    def.springFrequency = authoring.Frequency;
                    def.springDamping = authoring.DampingRatio;
                    break;
            }

            AddComponent(entity, def);
        }

        /// <summary>
        /// Resolve the authored <see cref="PhysicsJoint2DAuthoring.ConnectedBody"/> (a
        /// <see cref="PhysicsBody2DAuthoring"/>) to the entity the runtime joint definition references,
        /// registering the bake dependency so a change to the connected body re-bakes the joint. Mirrors
        /// <c>Joint2DBaking.ResolveConnectedBody</c> (which is typed on the built-in <c>Joint2D</c> for the
        /// nine built-in bakers) for the custom body surface; <c>GetEntity(..., Dynamic)</c> resolves the same
        /// entity whether the connected body was authored by <see cref="PhysicsBody2DAuthoring"/> or a built-in
        /// <c>Rigidbody2D</c>, so the connected reference is surface-agnostic. A null reference is a joint to a
        /// static world anchor (<see cref="Entity.Null"/>), the creation system's world-anchor path.
        /// </summary>
        Entity ResolveConnectedBody(PhysicsJoint2DAuthoring authoring)
        {
            var connected = authoring.ConnectedBody;
            // Register a dependency on the connected body so an edit to it re-bakes this joint. DependsOn(null)
            // is a benign no-op.
            DependsOn(connected);
            if (connected == null)
                return Entity.Null;
            return GetEntity(connected.gameObject, TransformUsageFlags.Dynamic);
        }
    }
}
