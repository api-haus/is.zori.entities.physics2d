using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the Phase-1B parity fixtures: surface materials (bounce /
    /// friction), <see cref="Rigidbody2D"/> body parameters (velocity seed, freeze-X constraint, heavy-vs-light
    /// mass), and an explicit collider-only static-body case. Each is a single-authored child SubScene plus a
    /// parent scene carrying the SubScene; both are registered in build settings so the runtime parity harness
    /// can load the parent (ECS bake) and additively load the child (the GameObject reference) by name.
    /// </summary>
    /// <remarks>
    /// Single authoring is the whole point: the same child scene feeds both backends, so a material assigned on
    /// a collider, a velocity set on a Rigidbody2D, or a constraint flag is read by both the package's bakers
    /// and the GameObject <c>Physics2D.Simulate</c> reference from the identical serialized source. Materials
    /// are saved as <c>PhysicsMaterial2D</c> assets and assigned as <c>sharedMaterial</c> so they persist in
    /// the scene YAML and survive both the additive load and the SubScene bake.
    ///
    /// <para>Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.BodyParamFixtureBuilder.BuildAll</c> before the
    /// PlayMode parity tests.</para>
    /// </remarks>
    public static class BodyParamFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        const float FloorY = 0f;
        static readonly Vector2 FloorSize = new(40f, 1f);

        public const string BounceParent = FixtureRoot + "/Bounce.unity";
        public const string BounceChild = FixtureRoot + "/Bounce_Sub.unity";
        public const string FrictionParent = FixtureRoot + "/Friction.unity";
        public const string FrictionChild = FixtureRoot + "/Friction_Sub.unity";
        public const string VelocityParent = FixtureRoot + "/Velocity.unity";
        public const string VelocityChild = FixtureRoot + "/Velocity_Sub.unity";
        public const string FreezeXParent = FixtureRoot + "/FreezeX.unity";
        public const string FreezeXChild = FixtureRoot + "/FreezeX_Sub.unity";
        public const string MassParent = FixtureRoot + "/Mass.unity";
        public const string MassChild = FixtureRoot + "/Mass_Sub.unity";
        public const string StaticParent = FixtureRoot + "/StaticFallback.unity";
        public const string StaticChild = FixtureRoot + "/StaticFallback_Sub.unity";

        const string BounceMaterialPath = FixtureRoot + "/BouncyMaterial.physicsMaterial2D";
        const string FrictionMaterialPath = FixtureRoot + "/HighFrictionMaterial.physicsMaterial2D";
        const string SlipperyMaterialPath = FixtureRoot + "/SlipperyMaterial.physicsMaterial2D";

        [MenuItem("Tools/Zori/Build Entities Physics2D Body-Param Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildBounce();
            BuildFriction();
            BuildVelocity();
            BuildFreezeX();
            BuildMass();
            BuildStaticFallback();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Entities Physics2D body-param fixtures built (Bounce/Friction/Velocity/FreezeX/Mass/Static).");
        }

        // A bouncy ball dropped onto a bouncy floor: bounciness 0.8 on both surfaces, so the ball rebounds
        // several times before settling. Bounce apex/rest is the observable the material mapping must match.
        static void BuildBounce()
        {
            var bouncy = CreateMaterial(BounceMaterialPath, friction: 0.0f, bounciness: 0.8f);

            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var floor = new GameObject("Floor");
            floor.transform.position = new Vector3(0f, FloorY, 0f);
            var floorBox = floor.AddComponent<BoxCollider2D>();
            floorBox.size = FloorSize;
            floorBox.sharedMaterial = bouncy;

            var ballGo = new GameObject("BouncyBall");
            ballGo.transform.position = new Vector3(0f, 6f, 0f);
            var rb = ballGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = ballGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            circle.sharedMaterial = bouncy;

            SaveFixture(child, BounceChild, BounceParent, "Bounce SubScene");
        }

        // Two BOXES launched horizontally along a long floor, one on a high-friction surface and one on a
        // slippery surface, so the high-friction box decelerates and stops far sooner than the slippery one.
        // Distinct X-stop is the friction observable. Boxes (not circles) on purpose: a circle on a floor
        // converts sliding into rolling (no rolling resistance by default) and keeps moving regardless of
        // friction, so friction is NOT observable on a rolling circle; a box can only slide, so its stopping
        // distance is a clean function of friction. The two boxes are in separate Y lanes (distinct Y so the
        // matching key is stable) and never interact.
        static void BuildFriction()
        {
            var grippy = CreateMaterial(FrictionMaterialPath, friction: 0.6f, bounciness: 0f);
            var slick = CreateMaterial(SlipperyMaterialPath, friction: 0.0f, bounciness: 0f);

            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Floor for the grippy lane (top at 0.5), carrying the grippy material.
            var floor = new GameObject("Floor");
            floor.transform.position = new Vector3(0f, FloorY, 0f);
            var floorBox = floor.AddComponent<BoxCollider2D>();
            floorBox.size = new Vector2(120f, 1f);
            floorBox.sharedMaterial = grippy;

            // High-friction box, lower lane (y matching key). Rests cleanly on the floor (top 0.5 + half-height
            // 0.5 = centre 1.0), launched right via the serialized velocity seed (Rigidbody2D.linearVelocity is
            // runtime-only — see InitialVelocity2DAuthoring).
            var grippyBox = MakeSlidingBox("GrippyBox", startY: 1.0f, startX: -20f, material: grippy);
            SeedVelocity(grippyBox, new Vector2(10f, 0f));

            // Floor for the slippery lane (top at 3.5), raised so the two boxes never interact.
            var floor2 = new GameObject("Floor2");
            floor2.transform.position = new Vector3(0f, 3f, 0f);
            var floor2Box = floor2.AddComponent<BoxCollider2D>();
            floor2Box.size = new Vector2(120f, 1f);
            floor2Box.sharedMaterial = slick;

            // Slippery box, upper lane, resting cleanly on floor2 (top 3.5 + half-height 0.5 = centre 4.0).
            var slickBox = MakeSlidingBox("SlickBox", startY: 4.0f, startX: -20f, material: slick);
            SeedVelocity(slickBox, new Vector2(10f, 0f));

            SaveFixture(child, FrictionChild, FrictionParent, "Friction SubScene");
        }

        // Seed an initial velocity via the serialized InitialVelocity2DAuthoring component. Rigidbody2D's own
        // linearVelocity/angularVelocity are runtime-only and are discarded on scene save, so a velocity set
        // directly on the Rigidbody2D bakes to zero — the serialized component is the single-authoring seed
        // both backends read.
        static void SeedVelocity(GameObject go, Vector2 linear, float angular = 0f)
        {
            var seed = go.AddComponent<Zori.Entities.Physics2D.InitialVelocity2DAuthoring>();
            seed.linearVelocity = linear;
            seed.angularVelocity = angular;
        }

        static GameObject MakeSlidingBox(string name, float startY, float startX, PhysicsMaterial2D material)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(startX, startY, 0f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            // Freeze rotation so the box slides flat: a free box can tip on its leading edge during a hard
            // friction stop, turning the trajectory into a tumble rather than a clean slide-and-stop.
            rb.constraints = RigidbodyConstraints2D.FreezeRotation;
            var box = go.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);
            box.sharedMaterial = material;
            return go;
        }

        // A body given an initial up-and-right linear velocity, falling freely: a parabola. No floor, so the
        // velocity seed alone (plus gravity) determines the whole trajectory — the observable is the arc.
        static void BuildVelocity()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var go = new GameObject("LaunchedBody");
            go.transform.position = new Vector3(0f, 0f, 0f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            go.AddComponent<CircleCollider2D>().radius = 0.5f;
            SeedVelocity(go, new Vector2(5f, 12f));

            SaveFixture(child, VelocityChild, VelocityParent, "Velocity SubScene");
        }

        // A body with FreezePositionX given an initial rightward velocity, falling: it must fall straight down
        // (X locked), so the X never changes despite the seeded horizontal velocity. The observable is the
        // X-invariance under a velocity that would otherwise move it.
        static void BuildFreezeX()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var go = new GameObject("FrozenXBody");
            go.transform.position = new Vector3(0f, 5f, 0f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.constraints = RigidbodyConstraints2D.FreezePositionX;
            go.AddComponent<CircleCollider2D>().radius = 0.5f;
            // Seed a horizontal velocity the freeze must cancel — proves the constraint, not just a zero start.
            SeedVelocity(go, new Vector2(8f, 0f));

            SaveFixture(child, FreezeXChild, FreezeXParent, "FreezeX SubScene");
        }

        // A heavy body and a light body, same shape, both falling freely. In free fall under gravity mass does
        // not change the trajectory (a=g for both), so this fixture verifies that an explicit mass mapping does
        // NOT corrupt the free-fall path — and the disqualifier set still checks both fell. (A mass-sensitive
        // observable like a contact-momentum transfer is a richer scene; this slice's parity gate is that the
        // explicit-mass bodies fall identically to the GameObject reference, which is the mass mapping's
        // correctness floor.) The two start at distinct Y so the matching key is stable.
        static void BuildMass()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var heavy = new GameObject("HeavyBody");
            heavy.transform.position = new Vector3(-2f, 5f, 0f);
            var heavyRb = heavy.AddComponent<Rigidbody2D>();
            heavyRb.bodyType = RigidbodyType2D.Dynamic;
            heavyRb.gravityScale = 1f;
            heavyRb.useAutoMass = false;
            heavyRb.mass = 50f;
            heavy.AddComponent<CircleCollider2D>().radius = 0.5f;

            var light = new GameObject("LightBody");
            light.transform.position = new Vector3(2f, 8f, 0f);
            var lightRb = light.AddComponent<Rigidbody2D>();
            lightRb.bodyType = RigidbodyType2D.Dynamic;
            lightRb.gravityScale = 1f;
            lightRb.useAutoMass = false;
            lightRb.mass = 0.2f;
            light.AddComponent<CircleCollider2D>().radius = 0.5f;

            SaveFixture(child, MassChild, MassParent, "Mass SubScene");
        }

        // An explicit collider-only static-body case: a dynamic circle falls onto a collider-only static
        // CircleCollider2D floor (a large static circle), exercising the CircleCollider2DBaker's NEW static
        // fallback specifically. The dynamic body must rest on the static circle's top.
        static void BuildStaticFallback()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Collider-only static circle (no Rigidbody2D) → CircleCollider2DBaker's static fallback.
            var ground = new GameObject("StaticCircleGround");
            ground.transform.position = new Vector3(0f, -5f, 0f);
            ground.AddComponent<CircleCollider2D>().radius = 5f;

            // Dynamic circle that falls onto the top of the static circle (top at y = -5 + 5 = 0) and rests.
            var ballGo = new GameObject("RestingBall");
            ballGo.transform.position = new Vector3(0f, 5f, 0f);
            var rb = ballGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            ballGo.AddComponent<CircleCollider2D>().radius = 0.5f;

            SaveFixture(child, StaticChild, StaticParent, "Static SubScene");
        }

        static PhysicsMaterial2D CreateMaterial(string path, float friction, float bounciness)
        {
            var existing = AssetDatabase.LoadAssetAtPath<PhysicsMaterial2D>(path);
            if (existing != null)
            {
                existing.friction = friction;
                existing.bounciness = bounciness;
                EditorUtility.SetDirty(existing);
                return existing;
            }
            var material = new PhysicsMaterial2D(Path.GetFileNameWithoutExtension(path))
            {
                friction = friction,
                bounciness = bounciness,
            };
            AssetDatabase.CreateAsset(material, path);
            return material;
        }

        static void SaveFixture(Scene child, string childPath, string parentPath, string subSceneName)
        {
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(subSceneName);
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
