using System.Collections.Generic;
using System.IO;
using Unity.Mathematics;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Portable, in-package generator for every scene-loading EditMode gate's fixture. It authors a parent scene
    /// + SubScene child into a CALLER-SUPPLIED <c>Assets/</c>-relative folder from a populate lambda and registers
    /// NOTHING in the build settings. The EditMode harness creates a temp folder, builds the fixture into it,
    /// loads the SubScene child by GUID through the entities scene system (and, for the parity gates, opens the
    /// same child additively as live built-in bodies), and deletes the folder on teardown — so the package is
    /// testable after a bare clone with no project assets, no build-profile writes.
    /// </summary>
    /// <remarks>
    /// The populate methods here carry the SAME geometry the former PlayMode <c>*FixtureBuilder.cs</c> builders
    /// authored; they were moved verbatim, dropping only the per-builder <c>RegisterSceneInBuildSettings</c> and
    /// the hard-coded <c>Assets/EntitiesPhysics2DFixture/</c> paths. Material assets that a fixture references are
    /// created inside the caller's temp folder so they are deleted with it.
    /// </remarks>
    public static partial class Physics2DFixtures
    {
        // The folder the current BuildScene is writing into — used by populate methods that must persist a
        // material asset (PhysicsMaterial2D) inside the same temp folder so it is deleted with the fixture.
        static string s_CurrentFolder;

        // Accessor for partial-class fixture files in sibling files that must persist an asset in the temp folder.
        internal static string CurrentFolder => s_CurrentFolder;

        /// <summary>
        /// Author a fixture (parent scene + SubScene child) under <paramref name="folder"/> from a populate
        /// lambda, and return the SubScene child's asset path (what the EditMode harness loads by GUID and opens
        /// additively). <paramref name="folder"/> is an <c>Assets/</c>-relative path the caller owns and deletes;
        /// this method writes only the two scenes and their SubScene wiring — it never touches
        /// <see cref="EditorBuildSettings.scenes"/>.
        /// </summary>
        public static string BuildScene(string folder, string sceneName, System.Action<GameObject> populate)
        {
            Directory.CreateDirectory(folder);
            AssetDatabase.Refresh();
            s_CurrentFolder = folder;

            var childPath = folder + "/" + sceneName + "_Sub.unity";
            var parentPath = folder + "/" + sceneName + ".unity";

            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var root = new GameObject("FixtureRoot");
            populate(root);
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(Path.GetFileNameWithoutExtension(childPath) + " SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(childPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, parentPath);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            s_CurrentFolder = null;
            return childPath;
        }

        // ---- FallingBody (FallingBodyValidation slice + CircleParityValidation) ----------------------------

        // The authored start height the falling-body gates assert world-Y decreases from.
        public const float FallingBodyStartY = 10f;

        // One Dynamic Rigidbody2D + CircleCollider2D raised so it has room to fall under gravity (built-in
        // authoring, single source for both the ECS bake and the GameObject reference).
        public static void FallingBody(GameObject root)
        {
            var bodyGo = new GameObject("FallingBody");
            Parent(root, bodyGo, new Vector3(0f, FallingBodyStartY, 0f));
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = bodyGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
        }

        // ---- BodyParamParityValidation fixtures (Bounce/Friction/Velocity/FreezeX/Mass/StaticFallback) -----

        const float BodyParamFloorY = 0f;
        static readonly Vector2 BodyParamFloorSize = new(40f, 1f);

        public static void Bounce(GameObject root)
        {
            var bouncy = CreateMaterialInFolder("BouncyMaterial", friction: 0.0f, bounciness: 0.8f);

            var floor = NewChild(root, "Floor", new Vector3(0f, BodyParamFloorY, 0f));
            var floorBox = floor.AddComponent<BoxCollider2D>();
            floorBox.size = BodyParamFloorSize;
            floorBox.sharedMaterial = bouncy;

            var ballGo = NewChild(root, "BouncyBall", new Vector3(0f, 6f, 0f));
            var rb = ballGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = ballGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            circle.sharedMaterial = bouncy;
        }

        public static void Friction(GameObject root)
        {
            var grippy = CreateMaterialInFolder("HighFrictionMaterial", friction: 0.6f, bounciness: 0f);
            var slick = CreateMaterialInFolder("SlipperyMaterial", friction: 0.0f, bounciness: 0f);

            var floor = NewChild(root, "Floor", new Vector3(0f, BodyParamFloorY, 0f));
            var floorBox = floor.AddComponent<BoxCollider2D>();
            floorBox.size = new Vector2(120f, 1f);
            floorBox.sharedMaterial = grippy;

            var grippyBox = MakeSlidingBox(root, "GrippyBox", startY: 1.0f, startX: -20f, material: grippy);
            SeedVelocity(grippyBox, new Vector2(10f, 0f));

            var floor2 = NewChild(root, "Floor2", new Vector3(0f, 3f, 0f));
            var floor2Box = floor2.AddComponent<BoxCollider2D>();
            floor2Box.size = new Vector2(120f, 1f);
            floor2Box.sharedMaterial = slick;

            var slickBox = MakeSlidingBox(root, "SlickBox", startY: 4.0f, startX: -20f, material: slick);
            SeedVelocity(slickBox, new Vector2(10f, 0f));
        }

        public static void Velocity(GameObject root)
        {
            var go = NewChild(root, "LaunchedBody", new Vector3(0f, 0f, 0f));
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            go.AddComponent<CircleCollider2D>().radius = 0.5f;
            SeedVelocity(go, new Vector2(5f, 12f));
        }

        public static void FreezeX(GameObject root)
        {
            var go = NewChild(root, "FrozenXBody", new Vector3(0f, 5f, 0f));
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.constraints = RigidbodyConstraints2D.FreezePositionX;
            go.AddComponent<CircleCollider2D>().radius = 0.5f;
            SeedVelocity(go, new Vector2(8f, 0f));
        }

        public static void Mass(GameObject root)
        {
            var heavy = NewChild(root, "HeavyBody", new Vector3(-2f, 5f, 0f));
            var heavyRb = heavy.AddComponent<Rigidbody2D>();
            heavyRb.bodyType = RigidbodyType2D.Dynamic;
            heavyRb.gravityScale = 1f;
            heavyRb.useAutoMass = false;
            heavyRb.mass = 50f;
            heavy.AddComponent<CircleCollider2D>().radius = 0.5f;

            var light = NewChild(root, "LightBody", new Vector3(2f, 8f, 0f));
            var lightRb = light.AddComponent<Rigidbody2D>();
            lightRb.bodyType = RigidbodyType2D.Dynamic;
            lightRb.gravityScale = 1f;
            lightRb.useAutoMass = false;
            lightRb.mass = 0.2f;
            light.AddComponent<CircleCollider2D>().radius = 0.5f;
        }

        public static void StaticFallback(GameObject root)
        {
            var ground = NewChild(root, "StaticCircleGround", new Vector3(0f, -5f, 0f));
            ground.AddComponent<CircleCollider2D>().radius = 5f;

            var ballGo = NewChild(root, "RestingBall", new Vector3(0f, 5f, 0f));
            var rb = ballGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            ballGo.AddComponent<CircleCollider2D>().radius = 0.5f;
        }

        static GameObject MakeSlidingBox(
            GameObject root,
            string name,
            float startY,
            float startX,
            PhysicsMaterial2D material
        )
        {
            var go = NewChild(root, name, new Vector3(startX, startY, 0f));
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.constraints = RigidbodyConstraints2D.FreezeRotation;
            var box = go.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);
            box.sharedMaterial = material;
            return go;
        }

        // ---- ColliderShapeParityValidation fixtures (Box/Capsule/Polygon/Edge on floor) --------------------

        const float ShapeStartY = 5f;

        public static void BoxOnFloor(GameObject root) =>
            ShapeOnFloor(
                root,
                "BoxBody",
                go =>
                {
                    var box = go.AddComponent<BoxCollider2D>();
                    box.size = new Vector2(1f, 1f);
                }
            );

        public static void CapsuleOnFloor(GameObject root) =>
            ShapeOnFloor(
                root,
                "CapsuleBody",
                go =>
                {
                    var cap = go.AddComponent<CapsuleCollider2D>();
                    cap.size = new Vector2(1f, 2f);
                    cap.direction = CapsuleDirection2D.Vertical;
                }
            );

        public static void PolygonOnFloor(GameObject root) =>
            ShapeOnFloor(
                root,
                "PolygonBody",
                go =>
                {
                    var poly = go.AddComponent<PolygonCollider2D>();
                    poly.SetPath(
                        0,
                        new[]
                        {
                            new Vector2(0f, 0.6f),
                            new Vector2(-0.6f, 0.2f),
                            new Vector2(-0.35f, -0.5f),
                            new Vector2(0.35f, -0.5f),
                            new Vector2(0.6f, 0.2f),
                        }
                    );
                }
            );

        // Edge fixture: the EdgeCollider2D is the STATIC surface (right-to-left winding so the solid side faces
        // up), a dynamic CircleCollider2D body falls onto it.
        public static void EdgeOnFloor(GameObject root)
        {
            var edgeGo = NewChild(root, "EdgeGround", new Vector3(0f, BodyParamFloorY, 0f));
            var edge = edgeGo.AddComponent<EdgeCollider2D>();
            edge.points = new[]
            {
                new Vector2(10f, 1f),
                new Vector2(3f, 0f),
                new Vector2(-3f, 0f),
                new Vector2(-10f, 1f),
            };

            var bodyGo = NewChild(root, "EdgeRestingBody", new Vector3(0f, ShapeStartY, 0f));
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = bodyGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
        }

        static void ShapeOnFloor(GameObject root, string bodyName, System.Action<GameObject> addShape)
        {
            var floor = NewChild(root, "Floor", new Vector3(0f, BodyParamFloorY, 0f));
            var floorBox = floor.AddComponent<BoxCollider2D>();
            floorBox.size = BodyParamFloorSize;

            var bodyGo = NewChild(root, bodyName, new Vector3(0f, ShapeStartY, 0f));
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            addShape(bodyGo);
        }

        // ---- shared authoring helpers ----------------------------------------------------------------------

        static GameObject NewChild(GameObject root, string name, Vector3 position)
        {
            var go = new GameObject(name);
            Parent(root, go, position);
            return go;
        }

        static void SeedVelocity(GameObject go, Vector2 linear, float angular = 0f)
        {
            var seed = go.AddComponent<InitialVelocity2DAuthoring>();
            seed.linearVelocity = linear;
            seed.angularVelocity = angular;
        }

        // ---- JointParityValidation fixtures (Hinge/Slider/Wheel/Distance/Spring/Fixed/Relative/Friction/Target)

        public static void HingeJoint(GameObject root) =>
            JointFixture(
                root,
                "HingeArm",
                new Vector3(1f, 5f, 0f),
                new Vector3(0f, 5f, 0f),
                (arm, anchorRb) =>
                {
                    var box = arm.AddComponent<BoxCollider2D>();
                    box.size = new Vector2(2f, 0.4f);

                    var hinge = arm.AddComponent<HingeJoint2D>();
                    hinge.connectedBody = anchorRb;
                    hinge.autoConfigureConnectedAnchor = false;
                    hinge.anchor = new Vector2(-1f, 0f);
                    hinge.connectedAnchor = Vector2.zero;
                    hinge.useMotor = false;
                    hinge.useLimits = false;
                }
            );

        public static void SliderJoint(GameObject root) =>
            JointFixture(
                root,
                "SliderBlock",
                new Vector3(0f, 5f, 0f),
                new Vector3(0f, 5f, 0f),
                (block, anchorRb) =>
                {
                    var box = block.AddComponent<BoxCollider2D>();
                    box.size = new Vector2(1f, 1f);

                    var slider = block.AddComponent<SliderJoint2D>();
                    slider.connectedBody = anchorRb;
                    slider.autoConfigureAngle = false;
                    slider.angle = 0f;
                    slider.autoConfigureConnectedAnchor = false;
                    slider.anchor = Vector2.zero;
                    slider.connectedAnchor = Vector2.zero;
                    slider.useMotor = false;
                    slider.useLimits = false;

                    var seed = block.AddComponent<InitialVelocity2DAuthoring>();
                    seed.linearVelocity = new Vector2(6f, 0f);
                    seed.angularVelocity = 0f;
                }
            );

        public static void WheelJoint(GameObject root) =>
            JointFixture(
                root,
                "WheelHub",
                new Vector3(0f, 5f, 0f),
                new Vector3(0f, 5f, 0f),
                (hub, anchorRb) =>
                {
                    var circle = hub.AddComponent<CircleCollider2D>();
                    circle.radius = 0.5f;

                    var wheel = hub.AddComponent<WheelJoint2D>();
                    wheel.connectedBody = anchorRb;
                    wheel.autoConfigureConnectedAnchor = false;
                    wheel.anchor = Vector2.zero;
                    wheel.connectedAnchor = Vector2.zero;
                    var suspension = wheel.suspension;
                    suspension.angle = 90f;
                    suspension.frequency = 2f;
                    suspension.dampingRatio = 0.2f;
                    wheel.suspension = suspension;
                    wheel.useMotor = false;
                }
            );

        public static void DistanceJoint(GameObject root) =>
            JointFixture(
                root,
                "DistanceDisc",
                new Vector3(3f, 5f, 0f),
                new Vector3(0f, 5f, 0f),
                (disc, anchorRb) =>
                {
                    var circle = disc.AddComponent<CircleCollider2D>();
                    circle.radius = 0.5f;

                    var distance = disc.AddComponent<DistanceJoint2D>();
                    distance.connectedBody = anchorRb;
                    distance.autoConfigureConnectedAnchor = false;
                    distance.autoConfigureDistance = false;
                    distance.anchor = Vector2.zero;
                    distance.connectedAnchor = Vector2.zero;
                    distance.distance = 3f;
                    distance.maxDistanceOnly = false;
                }
            );

        public static void SpringJoint(GameObject root) =>
            JointFixture(
                root,
                "SpringDisc",
                new Vector3(0f, 3f, 0f),
                new Vector3(0f, 5f, 0f),
                (disc, anchorRb) =>
                {
                    var circle = disc.AddComponent<CircleCollider2D>();
                    circle.radius = 0.5f;

                    var spring = disc.AddComponent<SpringJoint2D>();
                    spring.connectedBody = anchorRb;
                    spring.autoConfigureConnectedAnchor = false;
                    spring.autoConfigureDistance = false;
                    spring.anchor = Vector2.zero;
                    spring.connectedAnchor = Vector2.zero;
                    spring.distance = 1f;
                    spring.frequency = 1.5f;
                    spring.dampingRatio = 0.15f;
                }
            );

        public static void FixedJoint(GameObject root) =>
            JointFixture(
                root,
                "FixedBlock",
                new Vector3(2f, 5f, 0f),
                new Vector3(0f, 5f, 0f),
                (block, anchorRb) =>
                {
                    var box = block.AddComponent<BoxCollider2D>();
                    box.size = new Vector2(1f, 1f);

                    var fixedJoint = block.AddComponent<FixedJoint2D>();
                    fixedJoint.connectedBody = anchorRb;
                    fixedJoint.autoConfigureConnectedAnchor = false;
                    fixedJoint.anchor = Vector2.zero;
                    fixedJoint.connectedAnchor = new Vector2(2f, 0f);
                    fixedJoint.frequency = 0f;
                    fixedJoint.dampingRatio = 1f;
                }
            );

        public static void RelativeJoint(GameObject root) =>
            JointFixture(
                root,
                "RelativeBlock",
                new Vector3(2f, 5f, 0f),
                new Vector3(0f, 5f, 0f),
                (block, anchorRb) =>
                {
                    var box = block.AddComponent<BoxCollider2D>();
                    box.size = new Vector2(1f, 1f);

                    var relative = block.AddComponent<RelativeJoint2D>();
                    relative.connectedBody = anchorRb;
                    relative.autoConfigureOffset = false;
                    relative.linearOffset = new Vector2(2f, 0f);
                    relative.angularOffset = 0f;
                    relative.maxForce = 1000f;
                    relative.maxTorque = 1000f;
                }
            );

        public static void FrictionJoint(GameObject root) =>
            JointFixture(
                root,
                "FrictionBlock",
                new Vector3(0f, 5f, 0f),
                new Vector3(0f, 5f, 0f),
                (block, anchorRb) =>
                {
                    var box = block.AddComponent<BoxCollider2D>();
                    box.size = new Vector2(1f, 1f);

                    var friction = block.AddComponent<FrictionJoint2D>();
                    friction.connectedBody = anchorRb;
                    friction.maxForce = 30f;
                    friction.maxTorque = 30f;

                    var seed = block.AddComponent<InitialVelocity2DAuthoring>();
                    seed.linearVelocity = new Vector2(5f, 0f);
                    seed.angularVelocity = 0f;
                }
            );

        public static void TargetJoint(GameObject root)
        {
            var disc = NewChild(root, "TargetDisc", new Vector3(0f, 5f, 0f));
            var rb = disc.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;

            var circle = disc.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;

            var target = disc.AddComponent<TargetJoint2D>();
            target.autoConfigureTarget = false;
            target.anchor = Vector2.zero;
            target.target = new Vector2(3f, 5f);
            target.maxForce = 1000f;
            target.frequency = 5f;
            target.dampingRatio = 1f;
        }

        static void JointFixture(
            GameObject root,
            string bodyName,
            Vector3 bodyPos,
            Vector3 anchorPos,
            System.Action<GameObject, Rigidbody2D> addJoint
        )
        {
            var anchorGo = NewChild(root, bodyName + "Anchor", anchorPos);
            var anchorRb = anchorGo.AddComponent<Rigidbody2D>();
            anchorRb.bodyType = RigidbodyType2D.Static;
            var anchorCol = anchorGo.AddComponent<CircleCollider2D>();
            anchorCol.radius = 0.05f;

            var bodyGo = NewChild(root, bodyName, bodyPos);
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            addJoint(bodyGo, anchorRb);
        }

        // ---- AutoFitBakeGate fixture (geometry set by the REAL PhysicsShape2DAutoFit at build time) ---------

        public const float AutoFitBoxW = 4f;
        public const float AutoFitBoxH = 2f;
        public const float AutoFitXFitted = -10f;
        public const float AutoFitXHand = 10f;
        public const float AutoFitCircleRadius = 1f;
        public const float AutoFitFloorTop = 0.5f;

        public static void AutoFitBake(GameObject root)
        {
            var fittedGo = NewChild(root, "FittedBoxStatic", new Vector3(AutoFitXFitted, 3f, 0f));
            var fittedShape = fittedGo.AddComponent<PhysicsShape2DAuthoring>();
            var boxCloud = new List<float2>
            {
                new(-AutoFitBoxW * 0.5f, -AutoFitBoxH * 0.5f),
                new(AutoFitBoxW * 0.5f, -AutoFitBoxH * 0.5f),
                new(AutoFitBoxW * 0.5f, AutoFitBoxH * 0.5f),
                new(-AutoFitBoxW * 0.5f, AutoFitBoxH * 0.5f),
            };
            if (!PhysicsShape2DAutoFit.FitTo(fittedShape, boxCloud, PhysicsShape2DKind.Box, float2.zero))
                Debug.LogError("Physics2DFixtures.AutoFitBake: FitTo(Box) returned false — fixture invalid.");

            var handGo = NewChild(root, "HandBoxStatic", new Vector3(AutoFitXHand, 3f, 0f));
            var handShape = handGo.AddComponent<PhysicsShape2DAuthoring>();
            handShape.Kind = PhysicsShape2DKind.Box;
            handShape.BoxSize = new float2(AutoFitBoxW, AutoFitBoxH);
            handShape.BoxAngle = 0f;
            handShape.Radius = 0f;

            var floorGo = NewChild(root, "Floor", new Vector3(0f, 0f, 0f));
            var floorShape = floorGo.AddComponent<PhysicsShape2DAuthoring>();
            floorShape.Kind = PhysicsShape2DKind.Box;
            floorShape.BoxSize = new float2(40f, 1f);
            floorShape.Radius = 0f;

            var circleGo = NewChild(root, "FittedFallingCircle", new Vector3(0f, 6f, 0f));
            var body = circleGo.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.GravityScale = 1f;
            body.UseAutoMass = true;
            var circleShape = circleGo.AddComponent<PhysicsShape2DAuthoring>();
            var ring = new List<float2>();
            for (var i = 0; i < 16; i++)
            {
                var a = Mathf.Deg2Rad * (360f / 16f * i);
                ring.Add(new float2(Mathf.Cos(a), Mathf.Sin(a)) * AutoFitCircleRadius);
            }
            if (!PhysicsShape2DAutoFit.FitTo(circleShape, ring, PhysicsShape2DKind.Circle, float2.zero))
                Debug.LogError("Physics2DFixtures.AutoFitBake: FitTo(Circle) returned false — fixture invalid.");
            circleShape.Density = 1f;
        }

        // ---- MaterialTemplateBakeGate fixture (template/override/default precedence + built-in oracle) ------

        public const float MtTemplateFriction = 0.123f;
        public const float MtTemplateBounciness = 0.456f;
        public const PhysicsMaterialCombine2D MtTemplateFrictionCombine = PhysicsMaterialCombine2D.Maximum;
        public const PhysicsMaterialCombine2D MtTemplateBounceCombine = PhysicsMaterialCombine2D.Minimum;
        public const float MtOverrideFriction = 0.777f;
        public const PhysicsSurfaceMixing2D MtOverrideBouncinessCombine = PhysicsSurfaceMixing2D.Multiply;
        public const float MtDefaultFriction = 0.4f;
        public const float MtDefaultBounciness = 0f;
        public const float MtXTemplateCustom = -6f;
        public const float MtXTemplateBuiltIn = -4f;
        public const float MtXOverrideCustom = -2f;
        public const float MtXDefaultCustom = 0f;

        public static void MaterialTemplate(GameObject root)
        {
            var path = s_CurrentFolder + "/PhaseBTemplate.physicsMaterial2D";
            var material = new PhysicsMaterial2D("PhaseBTemplate")
            {
                friction = MtTemplateFriction,
                bounciness = MtTemplateBounciness,
                frictionCombine = MtTemplateFrictionCombine,
                bounceCombine = MtTemplateBounceCombine,
            };
            AssetDatabase.CreateAsset(material, path);
            EditorUtility.SetDirty(material);
            AssetDatabase.SaveAssetIfDirty(material);

            var templateCustom = NewChild(root, "TemplateCustom", new Vector3(MtXTemplateCustom, 0f, 0f));
            var tcShape = templateCustom.AddComponent<PhysicsShape2DAuthoring>();
            tcShape.Kind = PhysicsShape2DKind.Box;
            tcShape.BoxSize = new float2(1f, 1f);
            tcShape.MaterialTemplate = material;
            tcShape.Density = 1f;

            var templateBuiltIn = NewChild(root, "TemplateBuiltIn", new Vector3(MtXTemplateBuiltIn, 0f, 0f));
            var box = templateBuiltIn.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);
            box.density = 1f;
            box.sharedMaterial = material;

            var overrideCustom = NewChild(root, "OverrideCustom", new Vector3(MtXOverrideCustom, 0f, 0f));
            var ocShape = overrideCustom.AddComponent<PhysicsShape2DAuthoring>();
            ocShape.Kind = PhysicsShape2DKind.Box;
            ocShape.BoxSize = new float2(1f, 1f);
            ocShape.MaterialTemplate = material;
            ocShape.Density = 1f;
            ocShape.OverrideFriction = true;
            ocShape.Friction = MtOverrideFriction;
            ocShape.OverrideBouncinessCombine = true;
            ocShape.BouncinessCombine = MtOverrideBouncinessCombine;

            var defaultCustom = NewChild(root, "DefaultCustom", new Vector3(MtXDefaultCustom, 0f, 0f));
            var dcShape = defaultCustom.AddComponent<PhysicsShape2DAuthoring>();
            dcShape.Kind = PhysicsShape2DKind.Box;
            dcShape.BoxSize = new float2(1f, 1f);
            dcShape.Density = 1f;
        }

        // ---- FilterBakeParityGate fixture (4 static circles on distinct layers, keyed by Y) -----------------

        public const int FbLA = 8;
        public const int FbLB = 9;
        public const int FbLDefault = 0;
        public const int FbLX = 11;
        public const float FbYA = 2f;
        public const float FbYB = 4f;
        public const float FbYDefault = 6f;
        public const float FbYX = 8f;

        public static void FilterBake(GameObject root)
        {
            MakeLayerCircle(root, "BodyA", FbYA, FbLA);
            MakeLayerCircle(root, "BodyB", FbYB, FbLB);
            MakeLayerCircle(root, "BodyDefault", FbYDefault, FbLDefault);
            MakeLayerCircle(root, "BodyX", FbYX, FbLX);
        }

        static void MakeLayerCircle(GameObject root, string name, float y, int layer)
        {
            var go = NewChild(root, name, new Vector3(0f, y, 0f));
            go.layer = layer;
            go.AddComponent<CircleCollider2D>().radius = 0.5f;
        }

        static void Parent(GameObject root, GameObject child, Vector3 position)
        {
            child.transform.SetParent(root.transform, false);
            child.transform.position = position;
        }

        // Persist a PhysicsMaterial2D inside the current temp folder (deleted with the fixture). Used by the
        // material-driven fixtures, which need a serialized sharedMaterial asset to survive the scene save +
        // SubScene bake + additive reopen.
        static PhysicsMaterial2D CreateMaterialInFolder(string fileName, float friction, float bounciness)
        {
            var path = s_CurrentFolder + "/" + fileName + ".physicsMaterial2D";
            var material = new PhysicsMaterial2D(fileName) { friction = friction, bounciness = bounciness };
            AssetDatabase.CreateAsset(material, path);
            return material;
        }
    }
}
