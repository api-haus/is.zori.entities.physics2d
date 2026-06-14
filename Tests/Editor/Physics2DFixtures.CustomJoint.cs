using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Custom-joint CONVERGENCE fixture, ported verbatim from <c>CustomJointConvergenceFixtureBuilder</c>: a single
    /// SubScene carrying, for each of nine joint kinds, a CUSTOM <see cref="PhysicsJoint2DAuthoring"/> owner (with a
    /// custom static anchor) AND the equivalent built-in <c>*Joint2D</c> owner with byte-identical parameters (with a
    /// built-in <c>Rigidbody2D</c> anchor). Both bake to a <c>PhysicsJoint2DDefinition</c>; the bake gate asserts the
    /// two baked definitions are field-equal and the parity gate asserts they simulate the same. The X-positions,
    /// anchor Y, and every joint/motor/limit/spring/break field reproduce the builder exactly.
    /// </summary>
    public static partial class Physics2DFixtures
    {
        // X-keys for each owner body: even X = custom, odd-offset X = built-in twin (verbatim from the builder).
        public const float CjXHingeCustom = -10f;
        public const float CjXHingeBuiltIn = -8f;
        public const float CjXWheelCustom = -4f;
        public const float CjXWheelBuiltIn = -2f;
        public const float CjXRelativeCustom = 2f;
        public const float CjXRelativeBuiltIn = 4f;
        public const float CjXTargetCustom = 8f;
        public const float CjXTargetBuiltIn = 10f;
        public const float CjXSliderCustom = 20f;
        public const float CjXSliderBuiltIn = 22f;
        public const float CjXDistanceCustom = 26f;
        public const float CjXDistanceBuiltIn = 28f;
        public const float CjXSpringCustom = 32f;
        public const float CjXSpringBuiltIn = 34f;
        public const float CjXFixedCustom = 38f;
        public const float CjXFixedBuiltIn = 40f;
        public const float CjXFrictionCustom = 44f;
        public const float CjXFrictionBuiltIn = 46f;

        const float CjOwnerY = 5f;
        const float CjAnchorY = 8f;

        public static void CustomJointConvergence(GameObject root)
        {
            CjBuildHingePair(root);
            CjBuildWheelPair(root);
            CjBuildRelativePair(root);
            CjBuildTargetPair(root);
            CjBuildSliderPair(root);
            CjBuildDistancePair(root);
            CjBuildSpringPair(root);
            CjBuildFixedPair(root);
            CjBuildFrictionPair(root);
        }

        // ---- Hinge: anchors + motor + angle limit. ----
        static void CjBuildHingePair(GameObject root)
        {
            var (customOwner, customAnchor) = CjNewCustomPair(root, CjXHingeCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Hinge;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(-1f, 0f);
            cj.ConnectedAnchor = Unity.Mathematics.float2.zero;
            cj.UseMotor = true;
            cj.MotorSpeed = 90f; // deg/s
            cj.MaxMotorEffort = 250f; // torque
            cj.UseLimits = true;
            cj.LowerLimit = -30f; // degrees
            cj.UpperLimit = 60f;
            cj.CollideConnected = false;
            cj.BreakForce = 500f;
            cj.BreakTorque = 750f;
            cj.BreakAction = PhysicsJointBreakAction2D.CallbackOnly;

            var (builtinOwner, builtinAnchorRb) = CjNewBuiltInPair(root, CjXHingeBuiltIn);
            var hinge = builtinOwner.AddComponent<HingeJoint2D>();
            hinge.connectedBody = builtinAnchorRb;
            hinge.autoConfigureConnectedAnchor = false;
            hinge.anchor = new Vector2(-1f, 0f);
            hinge.connectedAnchor = Vector2.zero;
            hinge.useMotor = true;
            var hm = hinge.motor;
            hm.motorSpeed = 90f;
            hm.maxMotorTorque = 250f;
            hinge.motor = hm;
            hinge.useLimits = true;
            var hl = hinge.limits;
            hl.min = -30f;
            hl.max = 60f;
            hinge.limits = hl;
            hinge.enableCollision = false;
            hinge.breakForce = 500f;
            hinge.breakTorque = 750f;
            hinge.breakAction = JointBreakAction2D.CallbackOnly;
        }

        // ---- Wheel: suspension axis + spring, no limit. ----
        static void CjBuildWheelPair(GameObject root)
        {
            var (customOwner, customAnchor) = CjNewCustomPair(root, CjXWheelCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Wheel;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = Unity.Mathematics.float2.zero;
            cj.ConnectedAnchor = Unity.Mathematics.float2.zero;
            cj.AxisAngle = 90f; // vertical suspension axis, degrees
            cj.UseMotor = true;
            cj.MotorSpeed = 180f;
            cj.MaxMotorEffort = 50f;
            cj.Frequency = 2f; // suspension spring Hz
            cj.DampingRatio = 0.2f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = CjNewBuiltInPair(root, CjXWheelBuiltIn);
            var wheel = builtinOwner.AddComponent<WheelJoint2D>();
            wheel.connectedBody = builtinAnchorRb;
            wheel.autoConfigureConnectedAnchor = false;
            wheel.anchor = Vector2.zero;
            wheel.connectedAnchor = Vector2.zero;
            var susp = wheel.suspension;
            susp.angle = 90f;
            susp.frequency = 2f;
            susp.dampingRatio = 0.2f;
            wheel.suspension = susp;
            wheel.useMotor = true;
            var wm = wheel.motor;
            wm.motorSpeed = 180f;
            wm.maxMotorTorque = 50f;
            wheel.motor = wm;
            wheel.enableCollision = false;
        }

        // ---- Relative: zero-anchor + linear/angular offset + force/torque caps. ----
        static void CjBuildRelativePair(GameObject root)
        {
            var (customOwner, customAnchor) = CjNewCustomPair(root, CjXRelativeCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Relative;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.LinearOffset = new Unity.Mathematics.float2(2f, 0f);
            cj.AngularOffset = 15f; // degrees
            cj.MaxForce = 1000f;
            cj.MaxTorque = 1000f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = CjNewBuiltInPair(root, CjXRelativeBuiltIn);
            var rel = builtinOwner.AddComponent<RelativeJoint2D>();
            rel.connectedBody = builtinAnchorRb;
            rel.autoConfigureOffset = false;
            rel.linearOffset = new Vector2(2f, 0f);
            rel.angularOffset = 15f;
            rel.maxForce = 1000f;
            rel.maxTorque = 1000f;
            rel.enableCollision = false;
        }

        // ---- Target: null connected body (static world anchor) + world target + linear spring + force cap. ----
        static void CjBuildTargetPair(GameObject root)
        {
            var customOwner = CjNewCustomOwnerOnly(root, CjXTargetCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Target;
            cj.ConnectedBody = null; // world anchor
            cj.Anchor = Unity.Mathematics.float2.zero;
            cj.ConnectedAnchor = new Unity.Mathematics.float2(CjXTargetCustom + 3f, CjOwnerY); // world target
            cj.MaxForce = 1000f;
            cj.Frequency = 5f;
            cj.DampingRatio = 1f;
            cj.CollideConnected = true;

            var builtinOwner = CjNewBuiltInOwnerOnly(root, CjXTargetBuiltIn);
            var target = builtinOwner.AddComponent<TargetJoint2D>();
            target.autoConfigureTarget = false;
            target.anchor = Vector2.zero;
            target.target = new Vector2(CjXTargetCustom + 3f, CjOwnerY); // SAME world target as the custom side
            target.maxForce = 1000f;
            target.frequency = 5f;
            target.dampingRatio = 1f;
        }

        // ---- Slider: slide axis (deg) + linear motor (m/s, max FORCE) + translation limit (m), no spring. ----
        static void CjBuildSliderPair(GameObject root)
        {
            var (customOwner, customAnchor) = CjNewCustomPair(root, CjXSliderCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Slider;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(0.25f, -0.1f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(-0.3f, 0.2f);
            cj.AxisAngle = 30f; // a NON-zero slide axis
            cj.UseMotor = true;
            cj.MotorSpeed = 4f; // m/s along the axis
            cj.MaxMotorEffort = 120f; // → maxMotorForce for the slider
            cj.UseLimits = true;
            cj.LowerLimit = -2.5f; // metres
            cj.UpperLimit = 3.5f;
            cj.CollideConnected = true;
            cj.BreakForce = 333f;
            cj.BreakTorque = 444f;
            cj.BreakAction = PhysicsJointBreakAction2D.Disable;

            var (builtinOwner, builtinAnchorRb) = CjNewBuiltInPair(root, CjXSliderBuiltIn);
            var slider = builtinOwner.AddComponent<SliderJoint2D>();
            slider.connectedBody = builtinAnchorRb;
            slider.autoConfigureConnectedAnchor = false;
            slider.autoConfigureAngle = false;
            slider.anchor = new Vector2(0.25f, -0.1f);
            slider.connectedAnchor = new Vector2(-0.3f, 0.2f);
            slider.angle = 30f;
            slider.useMotor = true;
            var sm = slider.motor;
            sm.motorSpeed = 4f;
            sm.maxMotorTorque = 120f; // the built-in stores the cap in maxMotorTorque; the slider reads it as force
            slider.motor = sm;
            slider.useLimits = true;
            var sl = slider.limits;
            sl.min = -2.5f;
            sl.max = 3.5f;
            slider.limits = sl;
            slider.enableCollision = true;
            slider.breakForce = 333f;
            slider.breakTorque = 444f;
            slider.breakAction = JointBreakAction2D.Disable;
        }

        // ---- Distance: anchors + rest length, RIGID (enableSpring false), no motor/limit/axis. ----
        static void CjBuildDistancePair(GameObject root)
        {
            var (customOwner, customAnchor) = CjNewCustomPair(root, CjXDistanceCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Distance;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(0.4f, 0f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(0f, -0.2f);
            cj.RestLength = 2.75f; // the fixed separation the rigid distance joint holds
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = CjNewBuiltInPair(root, CjXDistanceBuiltIn);
            var dist = builtinOwner.AddComponent<DistanceJoint2D>();
            dist.connectedBody = builtinAnchorRb;
            dist.autoConfigureConnectedAnchor = false;
            dist.autoConfigureDistance = false;
            dist.anchor = new Vector2(0.4f, 0f);
            dist.connectedAnchor = new Vector2(0f, -0.2f);
            dist.distance = 2.75f;
            dist.maxDistanceOnly = false;
            dist.enableCollision = false;
        }

        // ---- Spring: anchors + rest length + spring freq/damp, enableSpring TRUE. ----
        static void CjBuildSpringPair(GameObject root)
        {
            var (customOwner, customAnchor) = CjNewCustomPair(root, CjXSpringCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Spring;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(-0.2f, 0.3f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(0.1f, 0f);
            cj.RestLength = 1.5f;
            cj.Frequency = 3.5f; // Hz — must reach springFrequency (the Spring arm is enableSpring TRUE)
            cj.DampingRatio = 0.4f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = CjNewBuiltInPair(root, CjXSpringBuiltIn);
            var spring = builtinOwner.AddComponent<SpringJoint2D>();
            spring.connectedBody = builtinAnchorRb;
            spring.autoConfigureConnectedAnchor = false;
            spring.autoConfigureDistance = false;
            spring.anchor = new Vector2(-0.2f, 0.3f);
            spring.connectedAnchor = new Vector2(0.1f, 0f);
            spring.distance = 1.5f;
            spring.frequency = 3.5f;
            spring.dampingRatio = 0.4f;
            spring.enableCollision = false;
        }

        // ---- Fixed: anchors + freq/damp ride the shared spring fields with enableSpring FALSE. ----
        static void CjBuildFixedPair(GameObject root)
        {
            var (customOwner, customAnchor) = CjNewCustomPair(root, CjXFixedCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Fixed;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(0.15f, -0.25f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(-0.05f, 0.35f);
            cj.Frequency = 2.5f; // non-zero stiffness, carried in springFrequency with enableSpring FALSE
            cj.DampingRatio = 0.8f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = CjNewBuiltInPair(root, CjXFixedBuiltIn);
            var fixedJoint = builtinOwner.AddComponent<FixedJoint2D>();
            fixedJoint.connectedBody = builtinAnchorRb;
            fixedJoint.autoConfigureConnectedAnchor = false;
            fixedJoint.anchor = new Vector2(0.15f, -0.25f);
            fixedJoint.connectedAnchor = new Vector2(-0.05f, 0.35f);
            fixedJoint.frequency = 2.5f;
            fixedJoint.dampingRatio = 0.8f;
            fixedJoint.enableCollision = false;
        }

        // ---- Friction: zero anchors + ZERO offset + force/torque caps, no spring. ----
        static void CjBuildFrictionPair(GameObject root)
        {
            var (customOwner, customAnchor) = CjNewCustomPair(root, CjXFrictionCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Friction;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            // Author NON-zero anchors/offset so the gate proves the arm actually zeroes them.
            cj.Anchor = new Unity.Mathematics.float2(9f, 9f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(-9f, -9f);
            cj.LinearOffset = new Unity.Mathematics.float2(7f, 7f);
            cj.AngularOffset = 33f;
            cj.MaxForce = 75f;
            cj.MaxTorque = 60f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = CjNewBuiltInPair(root, CjXFrictionBuiltIn);
            var friction = builtinOwner.AddComponent<FrictionJoint2D>();
            friction.connectedBody = builtinAnchorRb;
            friction.maxForce = 75f;
            friction.maxTorque = 60f;
            friction.enableCollision = false;
        }

        // A custom dynamic owner body + a custom static anchor body, authored into root via NewChild.
        static (GameObject owner, GameObject anchor) CjNewCustomPair(GameObject root, float ownerX)
        {
            var owner = CjNewCustomOwnerOnly(root, ownerX);

            var anchor = NewChild(root, "CustomAnchor_" + ownerX, new Vector3(ownerX, CjAnchorY, 0f));
            var aBody = anchor.AddComponent<PhysicsBody2DAuthoring>();
            aBody.BodyType = PhysicsBody2DMotionType.Static;
            var aShape = anchor.AddComponent<PhysicsShape2DAuthoring>();
            aShape.Kind = PhysicsShape2DKind.Circle;
            aShape.Radius = 0.05f;
            return (owner, anchor);
        }

        static GameObject CjNewCustomOwnerOnly(GameObject root, float ownerX)
        {
            var owner = NewChild(root, "CustomJointOwner_" + ownerX, new Vector3(ownerX, CjOwnerY, 0f));
            var body = owner.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.UseAutoMass = true;
            var shape = owner.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = 0.5f;
            shape.Density = 1f;
            return owner;
        }

        // A built-in dynamic owner body + a built-in static Rigidbody2D anchor, authored into root via NewChild.
        static (GameObject owner, Rigidbody2D anchorRb) CjNewBuiltInPair(GameObject root, float ownerX)
        {
            var owner = CjNewBuiltInOwnerOnly(root, ownerX);

            var anchor = NewChild(root, "BuiltInAnchor_" + ownerX, new Vector3(ownerX, CjAnchorY, 0f));
            var anchorRb = anchor.AddComponent<Rigidbody2D>();
            anchorRb.bodyType = RigidbodyType2D.Static;
            var aCol = anchor.AddComponent<CircleCollider2D>();
            aCol.radius = 0.05f;
            return (owner, anchorRb);
        }

        static GameObject CjNewBuiltInOwnerOnly(GameObject root, float ownerX)
        {
            var owner = NewChild(root, "BuiltInJointOwner_" + ownerX, new Vector3(ownerX, CjOwnerY, 0f));
            var rb = owner.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            var circle = owner.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            circle.density = 1f;
            return owner;
        }
    }
}
