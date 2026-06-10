using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the Phase-2A joint parity fixtures: one child SubScene per joint
    /// (Hinge / Slider / Wheel), each a single dynamic body jointed to a STATIC anchor body, plus a parent
    /// scene carrying the SubScene. Both child and parent are registered in build settings so the runtime
    /// parity harness can load the parent (ECS bake) and additively load the child (the GameObject reference)
    /// by name. Single authoring: the same child scene — with its built-in <c>*Joint2D</c> component riding on
    /// the dynamic body — feeds both backends.
    /// </summary>
    /// <remarks>
    /// Each fixture authors one dynamic <see cref="Rigidbody2D"/> carrying the joint component, plus one
    /// STATIC <see cref="Rigidbody2D"/> anchor the joint's <c>connectedBody</c> references. A concrete static
    /// anchor (rather than a null <c>connectedBody</c> world anchor) is what both the built-in
    /// <c>Physics2D</c> reference and the package's Box2D joint honour identically — built-in
    /// <c>WheelJoint2D</c>/<c>HingeJoint2D</c> with a null connected body behaved unlike a Box2D world anchor
    /// (the reference hub free-fell), so a real static body removes that asymmetry. The static anchor is
    /// excluded from the compared set on both backends (the ECS side filters non-static; the harness's
    /// <c>CollectReferenceBodies</c> filters static reference bodies), so only the dynamic body is matched.
    /// The joint observable is what each fixture is designed to make hard to fake:
    /// <list type="bullet">
    /// <item>Hinge: a horizontal arm pinned at one end swings down under gravity — a pendulum arc, with the
    /// pinned end staying at the anchor.</item>
    /// <item>Slider: a body on a horizontal slide axis, launched along it, with gravity pulling down — motion
    /// is confined to the axis (off-axis Y displacement ~0).</item>
    /// <item>Wheel: a body on a vertical suspension axis sags under gravity and the spring bounces it — the
    /// suspension travels along the axis and settles.</item>
    /// </list>
    ///
    /// <para>Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.JointFixtureBuilder.BuildAll</c> before the
    /// PlayMode joint parity tests.</para>
    /// </remarks>
    public static class JointFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        public const string HingeParent = FixtureRoot + "/HingeJoint.unity";
        public const string HingeChild = FixtureRoot + "/HingeJoint_Sub.unity";
        public const string SliderParent = FixtureRoot + "/SliderJoint.unity";
        public const string SliderChild = FixtureRoot + "/SliderJoint_Sub.unity";
        public const string WheelParent = FixtureRoot + "/WheelJoint.unity";
        public const string WheelChild = FixtureRoot + "/WheelJoint_Sub.unity";
        public const string DistanceParent = FixtureRoot + "/DistanceJoint.unity";
        public const string DistanceChild = FixtureRoot + "/DistanceJoint_Sub.unity";
        public const string SpringParent = FixtureRoot + "/SpringJoint.unity";
        public const string SpringChild = FixtureRoot + "/SpringJoint_Sub.unity";
        public const string FixedParent = FixtureRoot + "/FixedJoint.unity";
        public const string FixedChild = FixtureRoot + "/FixedJoint_Sub.unity";
        public const string RelativeParent = FixtureRoot + "/RelativeJoint.unity";
        public const string RelativeChild = FixtureRoot + "/RelativeJoint_Sub.unity";
        public const string FrictionParent = FixtureRoot + "/FrictionJoint.unity";
        public const string FrictionChild = FixtureRoot + "/FrictionJoint_Sub.unity";
        public const string TargetParent = FixtureRoot + "/TargetJoint.unity";
        public const string TargetChild = FixtureRoot + "/TargetJoint_Sub.unity";

        [MenuItem("Tools/Zori/Build Entities Physics2D Joint Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildHinge();
            BuildSlider();
            BuildWheel();
            BuildDistance();
            BuildSpring();
            BuildFixed();
            BuildRelative();
            BuildFriction();
            BuildTarget();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                "Entities Physics2D joint fixtures built (Hinge/Slider/Wheel/Distance/Spring/Fixed/"
                    + "Relative/Friction/Target)."
            );
        }

        // Hinge: a 2-unit-long horizontal arm whose LEFT end is pinned to a static anchor at (0, 5). Released
        // horizontal, gravity swings it down about the pivot — a pendulum. The arm's centre starts at (1, 5),
        // tracing a circle of radius 1 about the pivot. anchor = (-1, 0) (left end, arm-local); the static
        // anchor sits at the pivot, so connectedAnchor = (0, 0) (anchor-body-local).
        static void BuildHinge()
        {
            BuildJointFixture(
                HingeChild,
                HingeParent,
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
                    hinge.anchor = new Vector2(-1f, 0f); // left end of the arm, arm-local
                    hinge.connectedAnchor = Vector2.zero; // the anchor body sits at the pivot
                    hinge.useMotor = false;
                    hinge.useLimits = false;
                }
            );
        }

        // Slider: a body on a HORIZONTAL slide axis (angle 0°), connected to a static anchor at its start,
        // launched along +X. Gravity pulls down but the slider confines motion to the axis, so Y stays
        // ~constant — the off-axis-displacement observable. A serialized InitialVelocity2DAuthoring seeds the
        // +X launch on both backends (Rigidbody2D.linearVelocity is runtime-only).
        static void BuildSlider()
        {
            BuildJointFixture(
                SliderChild,
                SliderParent,
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
                    slider.angle = 0f; // horizontal slide axis
                    slider.autoConfigureConnectedAnchor = false;
                    slider.anchor = Vector2.zero; // block-local origin
                    slider.connectedAnchor = Vector2.zero; // anchor body at the block's start
                    slider.useMotor = false;
                    slider.useLimits = false;

                    var seed = block.AddComponent<InitialVelocity2DAuthoring>();
                    seed.linearVelocity = new Vector2(6f, 0f); // launch along +X
                    seed.angularVelocity = 0f;
                }
            );
        }

        // Wheel: a body on a VERTICAL suspension axis (angle 90°), connected to a static anchor at its start.
        // Gravity sags it down the axis; the suspension spring resists and bounces it, settling — the
        // suspension-travel observable. Spring frequency/damping are modest so the body oscillates a few times
        // within the capture window before settling.
        static void BuildWheel()
        {
            BuildJointFixture(
                WheelChild,
                WheelParent,
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
                    wheel.connectedAnchor = Vector2.zero; // anchor body at the hub's start
                    var suspension = wheel.suspension;
                    suspension.angle = 90f; // vertical suspension axis
                    suspension.frequency = 2f; // 2 Hz spring
                    suspension.dampingRatio = 0.2f; // lightly damped → a few visible bounces
                    wheel.suspension = suspension;
                    wheel.useMotor = false;
                }
            );
        }

        // Distance: a 1-unit dynamic disc 3 units to the RIGHT of a static anchor at (0, 5), joined by a rigid
        // distance joint of rest length 3. Released, gravity swings the disc down about the anchor like a rod
        // pendulum, but the rigid distance constraint holds the anchor→disc separation at exactly 3 the whole
        // time. The hard-to-fake observable is that held separation: a broken joint lets the disc free-fall
        // (separation grows without bound, y → −∞); a correct distance joint keeps |disc − (0,5)| ≈ 3 and the
        // disc on the radius-3 circle about the anchor.
        static void BuildDistance()
        {
            BuildJointFixture(
                DistanceChild,
                DistanceParent,
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
                    distance.anchor = Vector2.zero; // disc-local origin
                    distance.connectedAnchor = Vector2.zero; // the anchor body at the pivot
                    distance.distance = 3f; // rest length = the authored separation
                    distance.maxDistanceOnly = false; // rigid (absolute) distance, not a one-sided rope
                }
            );
        }

        // Spring: a 1-unit dynamic disc 2 units BELOW a static anchor at (0, 5) (i.e. at (0, 3)), joined by a
        // spring joint of rest length 1. The disc is far below its rest length, so the spring yanks it up,
        // overshoots, and oscillates vertically about the rest position (0, 4) at ~1.5 Hz, settling as the
        // light damping bleeds it out. The hard-to-fake observable is that oscillation toward the rest length:
        // a broken joint lets the disc free-fall (y → −∞); a correct spring oscillates around y ≈ 4 (anchor
        // minus rest length) and settles a touch below it (the gravity sag).
        static void BuildSpring()
        {
            BuildJointFixture(
                SpringChild,
                SpringParent,
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
                    spring.distance = 1f; // rest length: the spring pulls the disc to 1 unit below the anchor
                    spring.frequency = 1.5f; // 1.5 Hz → period ~0.67 s, several cycles in the window
                    spring.dampingRatio = 0.15f; // lightly damped → a few visible oscillations before settle
                }
            );
        }

        // Fixed: a 1-unit dynamic block welded to a static anchor at (0, 5), the block authored 2 units to the
        // RIGHT at (2, 5). A rigid fixed joint (frequency 0 = maximum stiffness) locks the block in that
        // relative pose, so under gravity it neither falls nor rotates away — it stays at (2, 5). The
        // hard-to-fake observable is the held relative pose: a broken joint lets the block free-fall (y → −∞);
        // a correct fixed joint pins it at (2, 5) with ~0 displacement and ~0 rotation for every step.
        static void BuildFixed()
        {
            BuildJointFixture(
                FixedChild,
                FixedParent,
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
                    fixedJoint.connectedAnchor = new Vector2(2f, 0f); // anchor-local: where the block sits
                    fixedJoint.frequency = 0f; // 0 Hz = rigid weld (max stiffness)
                    fixedJoint.dampingRatio = 1f;
                }
            );
        }

        // Relative: a 1-unit dynamic block joined to a static anchor at (0, 5), authored 2 units RIGHT at
        // (2, 5), holding a linear offset of (2, 0) from the anchor. The relative joint drives the block to
        // that offset and holds it against gravity. The hard-to-fake observable is the maintained offset: a
        // broken joint lets the block fall (y → −∞); a correct relative joint keeps it near (2, 5). The
        // built-in v2 RelativeJoint2D and the Box2D v3 relative joint agree on the offset once the package
        // encodes the offset in the same frame direction the built-in uses (see PhysicsJoint2DCreationSystem's
        // relative arm — the offset is added, not subtracted, to match the v2 linearOffset sign).
        static void BuildRelative()
        {
            BuildJointFixture(
                RelativeChild,
                RelativeParent,
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
                    relative.linearOffset = new Vector2(2f, 0f); // hold the block 2 right of the anchor
                    relative.angularOffset = 0f;
                    relative.maxForce = 1000f; // ample to hold the ~1 kg block against gravity
                    relative.maxTorque = 1000f;
                }
            );
        }

        // Friction: a 1-unit dynamic block on a friction joint to a static anchor at the block's start (0, 5),
        // launched at +5 m/s along +X by a serialized velocity seed. The friction joint resists relative
        // motion between block and (static) anchor up to a force cap, so the launch velocity damps out — the
        // block slides a little, decelerating, and stops. The hard-to-fake observable is that damping: a
        // broken/absent joint lets the block keep its launch velocity (sliding far) and also fall under
        // gravity; a correct friction joint brings the block to rest near its start, held against gravity too.
        static void BuildFriction()
        {
            BuildJointFixture(
                FrictionChild,
                FrictionParent,
                "FrictionBlock",
                new Vector3(0f, 5f, 0f),
                new Vector3(0f, 5f, 0f),
                (block, anchorRb) =>
                {
                    var box = block.AddComponent<BoxCollider2D>();
                    box.size = new Vector2(1f, 1f);

                    var friction = block.AddComponent<FrictionJoint2D>();
                    friction.connectedBody = anchorRb;
                    // A modest cap so the launch slides a VISIBLE fraction of a metre before friction arrests
                    // it (a 200 N cap brakes it within mm — too fast to show the damping). This is the "moving,
                    // then damped" observable: the block must travel enough to prove it is not frozen, yet stop
                    // far short of the ~10 m a ballistic (jointless) launch covers.
                    friction.maxForce = 30f;
                    friction.maxTorque = 30f;

                    var seed = block.AddComponent<InitialVelocity2DAuthoring>();
                    seed.linearVelocity = new Vector2(5f, 0f); // launch along +X; friction damps it out
                    seed.angularVelocity = 0f;
                }
            );
        }

        // Target: a 1-unit dynamic disc authored at (0, 5) and pulled by a target joint toward a FIXED world
        // target at (3, 5). A target joint is normally mouse-driven; here the target is a serialized fixed
        // point (autoConfigureTarget = false), the single-authoring analogue of the InitialVelocity2DAuthoring
        // pattern, so both backends pull toward the same point. Under gravity the disc is drawn from (0, 5)
        // toward (3, 5) and settles near it (a touch low from the gravity sag). The hard-to-fake observable is
        // reaching the target: a broken/absent joint lets the disc free-fall straight down from (0, 5); a
        // correct target joint moves it RIGHT toward x ≈ 3 and holds it near (3, 5). This is the one joint with
        // no static anchor body — it targets a point in the world (the null-connectedBody world anchor).
        static void BuildTarget()
        {
            BuildSingleBodyJointFixture(
                TargetChild,
                TargetParent,
                "TargetDisc",
                new Vector3(0f, 5f, 0f),
                disc =>
                {
                    var circle = disc.AddComponent<CircleCollider2D>();
                    circle.radius = 0.5f;

                    var target = disc.AddComponent<TargetJoint2D>();
                    target.autoConfigureTarget = false;
                    target.anchor = Vector2.zero; // grab the disc at its centre
                    target.target = new Vector2(3f, 5f); // fixed world target, serialized
                    target.maxForce = 1000f; // ample pull to reach and hold the target
                    target.frequency = 5f; // a stiff-ish spring so it converges within the window
                    target.dampingRatio = 1f; // critically damped → reaches the target without overshoot
                }
            );
        }

        /// <summary>
        /// Author one joint fixture: a child scene with a static anchor <see cref="Rigidbody2D"/> at
        /// <paramref name="anchorPos"/> and a single dynamic body at <paramref name="bodyPos"/> configured by
        /// <paramref name="addJoint"/> (which wires the joint's <c>connectedBody</c> to the passed anchor), and
        /// a parent scene carrying the child as an auto-loaded SubScene. Registers both in build settings.
        /// </summary>
        static void BuildJointFixture(
            string childPath,
            string parentPath,
            string bodyName,
            Vector3 bodyPos,
            Vector3 anchorPos,
            System.Action<GameObject, Rigidbody2D> addJoint
        )
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Static anchor body the joint connects to. It carries a TINY collider so the package's
            // body-creation system (which creates a body only for an entity that has both a
            // PhysicsBody2DDefinition AND a PhysicsShape2D) creates it and the joint's connected body gains a
            // live handle. The joint's collideConnected is false (default), so the dynamic body passes through
            // this collider rather than bouncing off it. The anchor is excluded from the compared set on both
            // backends (static), so it never inflates the body count or is matched against the dynamic body.
            var anchorGo = new GameObject(bodyName + "Anchor");
            anchorGo.transform.position = anchorPos;
            var anchorRb = anchorGo.AddComponent<Rigidbody2D>();
            anchorRb.bodyType = RigidbodyType2D.Static;
            var anchorCol = anchorGo.AddComponent<CircleCollider2D>();
            anchorCol.radius = 0.05f;

            var bodyGo = new GameObject(bodyName);
            bodyGo.transform.position = bodyPos;
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            addJoint(bodyGo, anchorRb);

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(bodyName + " SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(childPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, parentPath);

            RegisterSceneInBuildSettings(parentPath);
            RegisterSceneInBuildSettings(childPath);
        }

        /// <summary>
        /// Author a single-body joint fixture: one dynamic body at <paramref name="bodyPos"/> configured by
        /// <paramref name="addJoint"/>, with NO static anchor body — for a joint that constrains the body
        /// against a point in the world rather than against a second body (the <see cref="TargetJoint2D"/>
        /// world-target case). The joint's null <c>connectedBody</c> resolves to the package's static world
        /// anchor at runtime; the GameObject reference's <see cref="TargetJoint2D"/> pulls toward its serialized
        /// world target directly. Registers both child and parent in build settings.
        /// </summary>
        static void BuildSingleBodyJointFixture(
            string childPath,
            string parentPath,
            string bodyName,
            Vector3 bodyPos,
            System.Action<GameObject> addJoint
        )
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var bodyGo = new GameObject(bodyName);
            bodyGo.transform.position = bodyPos;
            var rb = bodyGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            addJoint(bodyGo);

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(bodyName + " SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(childPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, parentPath);

            RegisterSceneInBuildSettings(parentPath);
            RegisterSceneInBuildSettings(childPath);
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
