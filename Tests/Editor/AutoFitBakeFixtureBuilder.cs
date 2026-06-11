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
    /// Click-free authoring of the Phase-C AUTO-FIT bake-correctness fixture. Unlike a hand-authored fixture,
    /// the custom shapes here have their geometry set by the REAL <see cref="PhysicsShape2DAutoFit"/> utility at
    /// build time (the fit runs against a known point source, then the SubScene is baked through the normal
    /// importer). This pins the load-bearing claim that a fit emits UNSCALED local authoring fields that bake
    /// exactly as a hand-authored shape — proven end-to-end through the real baker, not asserted at the field
    /// boundary.
    /// </summary>
    /// <remarks>
    /// One scene, two custom-authored bodies plus a custom floor:
    /// <list type="bullet">
    /// <item><b>FittedBoxStatic</b> (X = -3) — a collider-only STATIC body whose Box geometry was set by
    /// auto-fitting a known 4×2 rectangle point source. The bake gate reads its baked <c>PhysicsShape2D.size</c>
    /// and asserts it equals the fitted (and hence the source) extent, scaled by the baker.</item>
    /// <item><b>HandBoxStatic</b> (X = +3) — the CONVERGENCE oracle: a hand-authored Box of the identical
    /// 4×2 local size. The gate asserts the fitted body's baked shape is bit-identical to this one.</item>
    /// <item><b>FittedFallingCircle</b> (X = 0, Y = 6) — a DYNAMIC body whose Circle radius was set by
    /// auto-fitting a radius-1 ring, falling onto the static floor. The broad gate (a GameObject-oracle or a
    /// rest check) proves a fitted body actually collides at its fitted extent.</item>
    /// <item><b>Floor</b> (Y = 0) — a custom-authored static box floor for the falling circle to rest on.</item>
    /// </list>
    /// Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.AutoFitBakeFixtureBuilder.Build</c>
    /// before the PlayMode AutoFitBakeGate.
    /// </remarks>
    public static class AutoFitBakeFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";
        public const string Parent = FixtureRoot + "/AutoFitBake.unity";
        public const string Child = FixtureRoot + "/AutoFitBake_Sub.unity";

        // The fitted-box source extent (a 4×2 axis-aligned rectangle) and the hand-authored oracle extent.
        public const float BoxW = 4f;
        public const float BoxH = 2f;

        // The two static box bodies sit far in X (±10) so they only BAKE and never lie in the falling circle's
        // (X=0) drop column — the circle must fall cleanly to the floor, not wedge between the boxes.
        public const float XFitted = -10f;
        public const float XHand = 10f;

        public const float CircleRadius = 1f;
        public const float CircleStartX = 0f;
        public const float CircleStartY = 6f;

        public const float FloorY = 0f;
        public const float FloorTopHalf = 0.5f; // floor box is 40×1 → top at +0.5

        [MenuItem("Tools/Zori/Build Entities Physics2D AutoFit Bake Fixture")]
        public static void Build()
        {
            Directory.CreateDirectory(FixtureRoot);

            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // --- FittedBoxStatic: geometry set by the REAL auto-fit utility from a 4x2 rectangle cloud. ---
            var fittedGo = new GameObject("FittedBoxStatic");
            fittedGo.transform.position = new Vector3(XFitted, 3f, 0f);
            var fittedShape = fittedGo.AddComponent<PhysicsShape2DAuthoring>();
            var boxCloud = new List<Unity.Mathematics.float2>
            {
                new(-BoxW * 0.5f, -BoxH * 0.5f),
                new(BoxW * 0.5f, -BoxH * 0.5f),
                new(BoxW * 0.5f, BoxH * 0.5f),
                new(-BoxW * 0.5f, BoxH * 0.5f),
            };
            var fitOk = PhysicsShape2DAutoFit.FitTo(
                fittedShape,
                boxCloud,
                PhysicsShape2DKind.Box,
                Unity.Mathematics.float2.zero
            );
            if (!fitOk)
                Debug.LogError("AutoFitBakeFixtureBuilder: FitTo(Box) returned false — fixture invalid.");

            // --- HandBoxStatic: the identical local geometry, hand-authored (the convergence oracle). ---
            var handGo = new GameObject("HandBoxStatic");
            handGo.transform.position = new Vector3(XHand, 3f, 0f);
            var handShape = handGo.AddComponent<PhysicsShape2DAuthoring>();
            handShape.Kind = PhysicsShape2DKind.Box;
            handShape.BoxSize = new Unity.Mathematics.float2(BoxW, BoxH);
            handShape.BoxAngle = 0f;
            handShape.Radius = 0f;

            // --- Floor: custom-authored static box. ---
            var floorGo = new GameObject("Floor");
            floorGo.transform.position = new Vector3(0f, FloorY, 0f);
            var floorShape = floorGo.AddComponent<PhysicsShape2DAuthoring>();
            floorShape.Kind = PhysicsShape2DKind.Box;
            floorShape.BoxSize = new Unity.Mathematics.float2(40f, 1f);
            // Radius on a Box is the CORNER-ROUNDING radius; the component defaults it to 0.5, which would raise
            // the floor's effective top surface by 0.5 (rounded edge). A flat floor wants a sharp edge — set 0,
            // so the floor top is exactly +0.5 and the falling circle rests at floorTop + fittedRadius = 1.5.
            floorShape.Radius = 0f;

            // --- FittedFallingCircle: radius set by auto-fitting a ring; falls onto the floor. ---
            var circleGo = new GameObject("FittedFallingCircle");
            circleGo.transform.position = new Vector3(CircleStartX, CircleStartY, 0f);
            var body = circleGo.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.GravityScale = 1f;
            body.UseAutoMass = true;
            var circleShape = circleGo.AddComponent<PhysicsShape2DAuthoring>();
            var ring = new List<Unity.Mathematics.float2>();
            for (var i = 0; i < 16; i++)
            {
                var a = Mathf.Deg2Rad * (360f / 16f * i);
                ring.Add(new Unity.Mathematics.float2(Mathf.Cos(a), Mathf.Sin(a)) * CircleRadius);
            }
            var circOk = PhysicsShape2DAutoFit.FitTo(
                circleShape,
                ring,
                PhysicsShape2DKind.Circle,
                Unity.Mathematics.float2.zero
            );
            if (!circOk)
                Debug.LogError("AutoFitBakeFixtureBuilder: FitTo(Circle) returned false — fixture invalid.");
            circleShape.Density = 1f;

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, Child);

            SaveParentWithSubScene(Parent, Child, "AutoFitBake SubScene");

            RegisterSceneInBuildSettings(Parent);
            RegisterSceneInBuildSettings(Child);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(
                $"AutoFit bake fixture built. Fitted box size set by FitTo = ({fittedShape.BoxSize.x}, "
                    + $"{fittedShape.BoxSize.y}); fitted circle radius = {circleShape.Radius}."
            );
        }

        static void SaveParentWithSubScene(string parentPath, string childPath, string subSceneName)
        {
            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(subSceneName);
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(childPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, parentPath);
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
