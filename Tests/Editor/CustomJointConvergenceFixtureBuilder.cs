using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free authoring of the Phase-F custom-joint CONVERGENCE fixture: a single SubScene carrying, for
    /// each representative joint kind, a CUSTOM <see cref="PhysicsJoint2DAuthoring"/> body AND the equivalent
    /// built-in <c>*Joint2D</c> body with byte-identical parameters. Both bake to a
    /// <see cref="PhysicsJoint2DDefinition"/> — the custom one through
    /// <c>PhysicsJoint2DAuthoringBaker</c>, the built-in one through the matching <c>*Joint2DBaker</c> — and
    /// <see cref="CustomJointConvergenceBakeGate"/> asserts the two baked definitions are field-equal (the
    /// convergence smoke: a custom joint bakes the same runtime joint as the built-in joint of the same params).
    /// </summary>
    /// <remarks>
    /// <para>Each pair is keyed by the joint-owner body's baked <c>initialPosition.x</c> so the runtime gate can
    /// match the custom def to its built-in twin without referencing the Editor builder (the runtime Tests
    /// asmdef cannot see the Editor-platform builder — the package's established X-key pattern, see
    /// <see cref="MaterialTemplateBakeGate"/> / <see cref="FilterBakeParityGate"/>).</para>
    ///
    /// <para><b>Representative subset</b> (the brief allows a representative subset; these four cover every
    /// distinct field arm of the baker's per-kind switch): Hinge (anchors + motor + angle limit), Wheel (axis +
    /// suspension spring, no limit), Relative (zero-anchor + linear/angular offset + force/torque caps), Target
    /// (null connected body → static world anchor + world-target + linear spring + force cap).</para>
    ///
    /// <para>The connected anchor is a per-side body: the custom joint connects to a custom-authored static
    /// anchor body, the built-in joint to a static <see cref="Rigidbody2D"/> anchor. The two
    /// <c>connectedBody</c> entities therefore differ, which is the one field the gate excludes from the
    /// field-equality compare (the geometry/motor/limit/spring/break fields are what convergence is about).
    /// The Target pair has no anchor body (null connected → <c>Entity.Null</c> on both), so even
    /// <c>connectedBody</c> matches there.</para>
    ///
    /// <para>Run via <c>-executeMethod
    /// Zori.Entities.Physics2D.Tests.Editor.CustomJointConvergenceFixtureBuilder.Build</c>.</para>
    /// </remarks>
    public static class CustomJointConvergenceFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";
        public const string ParentPath = FixtureRoot + "/CustomJointConvergence.unity";
        public const string ChildPath = FixtureRoot + "/CustomJointConvergence_Sub.unity";

        // X-keys for each owner body: even X = custom, odd-offset X = built-in twin. Distinct per kind so the
        // gate can pick out each pair. Anchors sit just below their owner so they never collide with the X-keys.
        // The original four representative kinds occupy X ∈ [−10, 10]; the five kinds the validation gate adds
        // (Slider/Distance/Spring/Fixed/Friction) sit in a clear second band at X ≥ 20 so no owner is within the
        // gate's 0.25 X-pick tolerance of another. The gate now pins all nine baker arms, not just four.
        public const float XHingeCustom = -10f;
        public const float XHingeBuiltIn = -8f;
        public const float XWheelCustom = -4f;
        public const float XWheelBuiltIn = -2f;
        public const float XRelativeCustom = 2f;
        public const float XRelativeBuiltIn = 4f;
        public const float XTargetCustom = 8f;
        public const float XTargetBuiltIn = 10f;
        public const float XSliderCustom = 20f;
        public const float XSliderBuiltIn = 22f;
        public const float XDistanceCustom = 26f;
        public const float XDistanceBuiltIn = 28f;
        public const float XSpringCustom = 32f;
        public const float XSpringBuiltIn = 34f;
        public const float XFixedCustom = 38f;
        public const float XFixedBuiltIn = 40f;
        public const float XFrictionCustom = 44f;
        public const float XFrictionBuiltIn = 46f;

        const float OwnerY = 5f;
        const float AnchorY = 8f;

        [MenuItem("Tools/Zori/Build Entities Physics2D Custom Joint Convergence Fixture")]
        public static void Build()
        {
            Directory.CreateDirectory(FixtureRoot);

            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            BuildHingePair();
            BuildWheelPair();
            BuildRelativePair();
            BuildTargetPair();
            BuildSliderPair();
            BuildDistancePair();
            BuildSpringPair();
            BuildFixedPair();
            BuildFrictionPair();

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, ChildPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject("CustomJointConvergence SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(ChildPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, ParentPath);

            RegisterSceneInBuildSettings(ParentPath);
            RegisterSceneInBuildSettings(ChildPath);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                "Entities Physics2D custom-joint convergence fixture built (all NINE kinds: Hinge / Wheel / "
                    + "Relative / Target / Slider / Distance / Spring / Fixed / Friction custom-vs-built-in pairs)."
            );
        }

        // ---- Hinge: anchors + motor + angle limit. ----
        static void BuildHingePair()
        {
            // Custom: a dynamic body with a custom hinge to a custom static anchor.
            var (customOwner, customAnchor) = NewCustomPair(XHingeCustom);
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
            // Pin the break surface to a NON-default value so the convergence smoke proves the break-force +
            // break-action mapping, not just matching defaults.
            cj.BreakForce = 500f;
            cj.BreakTorque = 750f;
            cj.BreakAction = PhysicsJointBreakAction2D.CallbackOnly;

            // Built-in: a dynamic Rigidbody2D with a HingeJoint2D of identical params to a Rigidbody2D anchor.
            var (builtinOwner, builtinAnchorRb) = NewBuiltInPair(XHingeBuiltIn);
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
            // Same non-default break surface as the custom side, so the gate pins the break-force + break-action
            // mapping (JointBreakAction2D.CallbackOnly → PhysicsJointBreakAction2D.CallbackOnly).
            hinge.breakForce = 500f;
            hinge.breakTorque = 750f;
            hinge.breakAction = JointBreakAction2D.CallbackOnly;
        }

        // ---- Wheel: suspension axis + spring, no limit. ----
        static void BuildWheelPair()
        {
            var (customOwner, customAnchor) = NewCustomPair(XWheelCustom);
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

            var (builtinOwner, builtinAnchorRb) = NewBuiltInPair(XWheelBuiltIn);
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
        static void BuildRelativePair()
        {
            var (customOwner, customAnchor) = NewCustomPair(XRelativeCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Relative;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.LinearOffset = new Unity.Mathematics.float2(2f, 0f);
            cj.AngularOffset = 15f; // degrees
            cj.MaxForce = 1000f;
            cj.MaxTorque = 1000f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = NewBuiltInPair(XRelativeBuiltIn);
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
        static void BuildTargetPair()
        {
            // Target is a single-body joint: NO anchor body on either side (null connected → static world anchor).
            var customOwner = NewCustomOwnerOnly(XTargetCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Target;
            cj.ConnectedBody = null; // world anchor
            cj.Anchor = Unity.Mathematics.float2.zero;
            cj.ConnectedAnchor = new Unity.Mathematics.float2(XTargetCustom + 3f, OwnerY); // world target
            cj.MaxForce = 1000f;
            cj.Frequency = 5f;
            cj.DampingRatio = 1f;
            // TargetJoint2D is single-body: its enableCollision is effectively always true (there is no second
            // body to collide with), so the built-in baker bakes collideConnected=true regardless. The custom
            // side matches it so the pair converges — see the negative-space note in 08-phaseF.
            cj.CollideConnected = true;

            var builtinOwner = NewBuiltInOwnerOnly(XTargetBuiltIn);
            var target = builtinOwner.AddComponent<TargetJoint2D>();
            target.autoConfigureTarget = false;
            target.anchor = Vector2.zero;
            target.target = new Vector2(XTargetCustom + 3f, OwnerY); // SAME world target as the custom side
            target.maxForce = 1000f;
            target.frequency = 5f;
            target.dampingRatio = 1f;
            // TargetJoint2D.enableCollision is single-body and bakes collideConnected=true regardless of this
            // setter; the custom side is set to true to match (negative space, documented in 08-phaseF).
        }

        // ---- Slider: slide axis (deg) + linear motor (m/s, max FORCE) + translation limit (m), no spring. ----
        // This arm also re-proves the break-force + break-action mapping (a NON-default break surface with the
        // Disable action) and the collideConnected field (true), on a fresh kind — so a future regression in the
        // shared-field handling trips here as well as on the Hinge pair.
        static void BuildSliderPair()
        {
            var (customOwner, customAnchor) = NewCustomPair(XSliderCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Slider;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(0.25f, -0.1f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(-0.3f, 0.2f);
            cj.AxisAngle = 30f; // a NON-zero slide axis: a Hinge bakes this to 0, so a kind mismap diverges here
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

            var (builtinOwner, builtinAnchorRb) = NewBuiltInPair(XSliderBuiltIn);
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
        static void BuildDistancePair()
        {
            var (customOwner, customAnchor) = NewCustomPair(XDistanceCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Distance;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(0.4f, 0f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(0f, -0.2f);
            cj.RestLength = 2.75f; // the fixed separation the rigid distance joint holds
            // Frequency/DampingRatio default to 1; the Distance arm must DROP them (enableSpring false, spring
            // fields zero). If the baker leaked the authored 1f into a Distance's spring fields, the gate trips.
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = NewBuiltInPair(XDistanceBuiltIn);
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
        static void BuildSpringPair()
        {
            var (customOwner, customAnchor) = NewCustomPair(XSpringCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Spring;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(-0.2f, 0.3f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(0.1f, 0f);
            cj.RestLength = 1.5f;
            cj.Frequency = 3.5f; // Hz — must reach springFrequency (the Spring arm is enableSpring TRUE)
            cj.DampingRatio = 0.4f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = NewBuiltInPair(XSpringBuiltIn);
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

        // ---- Fixed: anchors + freq/damp ride the shared spring fields with enableSpring FALSE (the trickiest
        // arm: the spring fields carry data the creation system applies to BOTH linear and angular stiffness,
        // yet enableSpring stays false). A NON-zero frequency here proves the Fixed arm bakes the spring fields
        // WITHOUT flipping enableSpring — distinct from the Spring arm above. ----
        static void BuildFixedPair()
        {
            var (customOwner, customAnchor) = NewCustomPair(XFixedCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Fixed;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            cj.Anchor = new Unity.Mathematics.float2(0.15f, -0.25f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(-0.05f, 0.35f);
            cj.Frequency = 2.5f; // non-zero stiffness, carried in springFrequency with enableSpring FALSE
            cj.DampingRatio = 0.8f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = NewBuiltInPair(XFixedBuiltIn);
            var fixedJoint = builtinOwner.AddComponent<FixedJoint2D>();
            fixedJoint.connectedBody = builtinAnchorRb;
            fixedJoint.autoConfigureConnectedAnchor = false;
            fixedJoint.anchor = new Vector2(0.15f, -0.25f);
            fixedJoint.connectedAnchor = new Vector2(-0.05f, 0.35f);
            fixedJoint.frequency = 2.5f;
            fixedJoint.dampingRatio = 0.8f;
            fixedJoint.enableCollision = false;
        }

        // ---- Friction: zero anchors + ZERO offset + force/torque caps, no spring (velocity-control friction). ----
        static void BuildFrictionPair()
        {
            var (customOwner, customAnchor) = NewCustomPair(XFrictionCustom);
            var cj = customOwner.AddComponent<PhysicsJoint2DAuthoring>();
            cj.Kind = PhysicsJoint2DKind.Friction;
            cj.ConnectedBody = customAnchor.GetComponent<PhysicsBody2DAuthoring>();
            // The Friction arm forces anchors + offset to zero regardless of what is authored; author NON-zero
            // values so the gate proves the arm actually zeroes them (a leak would diverge from the built-in,
            // whose FrictionJoint2D has no anchors and bakes zero).
            cj.Anchor = new Unity.Mathematics.float2(9f, 9f);
            cj.ConnectedAnchor = new Unity.Mathematics.float2(-9f, -9f);
            cj.LinearOffset = new Unity.Mathematics.float2(7f, 7f);
            cj.AngularOffset = 33f;
            cj.MaxForce = 75f;
            cj.MaxTorque = 60f;
            cj.CollideConnected = false;

            var (builtinOwner, builtinAnchorRb) = NewBuiltInPair(XFrictionBuiltIn);
            var friction = builtinOwner.AddComponent<FrictionJoint2D>();
            friction.connectedBody = builtinAnchorRb;
            friction.maxForce = 75f;
            friction.maxTorque = 60f;
            friction.enableCollision = false;
        }

        // A custom dynamic owner body + a custom static anchor body, returned as the GameObjects. The owner
        // carries a tiny custom shape (so the creation system creates its body); the anchor carries one too.
        static (GameObject owner, GameObject anchor) NewCustomPair(float ownerX)
        {
            var owner = NewCustomOwnerOnly(ownerX);

            var anchor = new GameObject("CustomAnchor_" + ownerX);
            anchor.transform.position = new Vector3(ownerX, AnchorY, 0f);
            var aBody = anchor.AddComponent<PhysicsBody2DAuthoring>();
            aBody.BodyType = PhysicsBody2DMotionType.Static;
            var aShape = anchor.AddComponent<PhysicsShape2DAuthoring>();
            aShape.Kind = PhysicsShape2DKind.Circle;
            aShape.Radius = 0.05f;
            return (owner, anchor);
        }

        static GameObject NewCustomOwnerOnly(float ownerX)
        {
            var owner = new GameObject("CustomJointOwner_" + ownerX);
            owner.transform.position = new Vector3(ownerX, OwnerY, 0f);
            var body = owner.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.UseAutoMass = true;
            var shape = owner.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = 0.5f;
            shape.Density = 1f;
            return owner;
        }

        // A built-in dynamic owner body + a built-in static Rigidbody2D anchor.
        static (GameObject owner, Rigidbody2D anchorRb) NewBuiltInPair(float ownerX)
        {
            var owner = NewBuiltInOwnerOnly(ownerX);

            var anchor = new GameObject("BuiltInAnchor_" + ownerX);
            anchor.transform.position = new Vector3(ownerX, AnchorY, 0f);
            var anchorRb = anchor.AddComponent<Rigidbody2D>();
            anchorRb.bodyType = RigidbodyType2D.Static;
            var aCol = anchor.AddComponent<CircleCollider2D>();
            aCol.radius = 0.05f;
            return (owner, anchorRb);
        }

        static GameObject NewBuiltInOwnerOnly(float ownerX)
        {
            var owner = new GameObject("BuiltInJointOwner_" + ownerX);
            owner.transform.position = new Vector3(ownerX, OwnerY, 0f);
            var rb = owner.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            var circle = owner.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            circle.density = 1f;
            return owner;
        }

        static void RegisterSceneInBuildSettings(string scenePath)
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (scenes.Exists(s => s.path == scenePath))
                return;
            scenes.Add(new EditorBuildSettingsScene(scenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }
    }
}
