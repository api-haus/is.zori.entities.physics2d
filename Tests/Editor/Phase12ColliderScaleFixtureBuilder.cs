using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free authoring of the Phase-12 collider transform-scale GameObject-parity fixtures: real built-in
    /// colliders on Transforms with NON-UNIT (and non-uniform, and negative) scale, baked through the actual
    /// Phase-12 scale-aware bakers, plus the SAME authored scene driven live by the GameObject
    /// <c>Physics2D.Simulate</c> oracle. The validating gate (<c>Phase12ColliderScaleGate</c>) loads these and
    /// proves the baked geometry collides at its RENDERED scaled size in both mediums — the bug that shipped a
    /// scaled <see cref="BoxCollider2D"/> as an unscaled 1×1 box.
    /// </summary>
    /// <remarks>
    /// <para><b>The shape of every fixture: a scaled STATIC floor + an unscaled DYNAMIC faller.</b> The floor is
    /// the collider whose transform scale is under test; it is a collider-only static body (no
    /// <see cref="Rigidbody2D"/>), so it is the CONTACT SURFACE — excluded from the parity-compared set on both
    /// backends (the harness compares only non-static bodies). The faller is a unit-scale
    /// <c>Rigidbody2D</c>+<c>CircleCollider2D</c> disc, the one compared body, dropped from above. Where the disc
    /// settles is the witness: a correctly-scaled floor catches it at the scaled top; an unscaled (buggy) floor
    /// lets it fall past everywhere except the unscaled 1×1 centre. Single authoring (the same SubScene feeds both
    /// the ECS bake and the live GameObject oracle) keeps the two sides from drifting by authoring.</para>
    ///
    /// <para><b>Why these are REAL components on REAL scaled Transforms.</b> The Phase-12 smoke
    /// (<c>ColliderScaleBakeSmoke</c>) pins the pure helper math (<c>ScaleBoxSize</c>, <c>ScaleCircleRadius</c>,
    /// …) without ever running a baker against a scaled GameObject. The bug lived in the bakers reading
    /// <c>lossyScale</c> and in the write-back rebuilding <c>LocalToWorld</c> — neither is exercised until the
    /// engine bakes a real scaled component and the runtime steps it. These fixtures put a real
    /// <c>transform.localScale</c> on a real collider so the baker's <c>ReadScale</c>, the per-kind geometry
    /// scaling, the winding-flip on a mirror, and the <c>PhysicsBody2DRenderScale</c> write-back all ship
    /// executed against the GameObject oracle, not just the smoke's chosen math.</para>
    ///
    /// <para>Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.Phase12ColliderScaleFixtureBuilder.BuildAll</c>
    /// before the PlayMode gate.</para>
    /// </remarks>
    public static class Phase12ColliderScaleFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";

        // (1) The headline manual-QA case: a 1×1 BoxCollider2D on Scale X=18.1822 — an 18×-wide floor. Two discs
        //     dropped near the EDGE (x=±7, well outside the unscaled 1×1 centre) must rest ON it, not fall past.
        public const string WideBoxParent = FixtureRoot + "/P12WideBox.unity";
        public const string WideBoxChild = FixtureRoot + "/P12WideBox_Sub.unity";
        public const float WideBoxScaleX = 18.1822f;

        // (2) A CircleCollider2D floor under NON-UNIFORM scale (3, 1.4): the radius rule (cmax vs something else)
        //     decides the rest height. The disc settles on the scaled circle's top.
        public const string NonUniformCircleParent = FixtureRoot + "/P12NonUniformCircle.unity";
        public const string NonUniformCircleChild = FixtureRoot + "/P12NonUniformCircle_Sub.unity";

        // (3) A CapsuleCollider2D floor under NON-UNIFORM scale: caps re-derived from the scaled size.
        public const string NonUniformCapsuleParent = FixtureRoot + "/P12NonUniformCapsule.unity";
        public const string NonUniformCapsuleChild = FixtureRoot + "/P12NonUniformCapsule_Sub.unity";

        // (4) A NEGATIVE-X-scaled asymmetric PolygonCollider2D floor — a mirror. Winding must reverse or the hull
        //     is inside-out and the disc falls through. The disc rests on the mirrored solid side.
        public const string NegativePolygonParent = FixtureRoot + "/P12NegativePolygon.unity";
        public const string NegativePolygonChild = FixtureRoot + "/P12NegativePolygon_Sub.unity";

        // (5) A NEGATIVE-X-scaled asymmetric EdgeCollider2D floor (a chain) — the solid side IS the winding, so a
        //     mirror that did not reverse the order would face solid-side away and the disc would fall through.
        public const string NegativeEdgeParent = FixtureRoot + "/P12NegativeEdge.unity";
        public const string NegativeEdgeChild = FixtureRoot + "/P12NegativeEdge_Sub.unity";

        // (6) A BoxCollider2D floor with a NON-ZERO Offset under scale — the offset must scale per-axis (signed),
        //     so the shape sits where the GameObject puts it. The disc settles over the offset-shifted floor.
        public const string ScaledOffsetParent = FixtureRoot + "/P12ScaledOffset.unity";
        public const string ScaledOffsetChild = FixtureRoot + "/P12ScaledOffset_Sub.unity";

        // (7) A CompositeCollider2D floor under non-unit scale — merged paths scaled, body rests at scaled extent.
        public const string ScaledCompositeParent = FixtureRoot + "/P12ScaledComposite.unity";
        public const string ScaledCompositeChild = FixtureRoot + "/P12ScaledComposite_Sub.unity";

        // (8) A CustomCollider2D floor under non-unit scale — group vertices/radii scaled, body rests at extent.
        public const string ScaledCustomParent = FixtureRoot + "/P12ScaledCustom.unity";
        public const string ScaledCustomChild = FixtureRoot + "/P12ScaledCustom_Sub.unity";

        // (9) A SCALED DYNAMIC faller for the rendering-scale-preserved + smoothing pin: a non-uniformly-scaled
        //     Rigidbody2D disc that falls onto an unscaled floor. Its baked LocalToWorld must carry the scale, and
        //     the interpolation smoothing must preserve it too.
        public const string ScaledDynamicParent = FixtureRoot + "/P12ScaledDynamic.unity";
        public const string ScaledDynamicChild = FixtureRoot + "/P12ScaledDynamic_Sub.unity";

        [MenuItem("Tools/Zori/Build Entities Physics2D Phase-12 Collider-Scale Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildWideBox();
            BuildNonUniformCircle();
            BuildNonUniformCapsule();
            BuildNegativePolygon();
            BuildNegativeEdge();
            BuildScaledOffset();
            BuildScaledComposite();
            BuildScaledCustom();
            BuildScaledDynamic();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Entities Physics2D Phase-12 collider-scale fixtures built (9 scaled-collider scenes).");
        }

        // ---------------------------------------------------------------------------------------------------
        // Shared authoring helpers.

        // A unit-scale dynamic disc faller (the compared body). NeverSleep + matched friction are applied by the
        // harness at load; here we just author the body + collider at the drop pose.
        static GameObject MakeDisc(
            UnityEngine.SceneManagement.Scene scene,
            string name,
            Vector2 pos,
            float radius
        )
        {
            var go = new GameObject(name);
            go.transform.position = pos;
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = go.AddComponent<CircleCollider2D>();
            circle.radius = radius;
            EditorSceneManager.MoveGameObjectToScene(go, scene);
            return go;
        }

        // ---------------------------------------------------------------------------------------------------
        // (1) Wide box floor: Scale X = 18.1822 on a 1×1 BoxCollider2D. Two discs near the edges.

        static void BuildWideBox()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // The QA floor: a unit BoxCollider2D, top at y=0, on Scale (18.1822, 1). The baked half-extent must
            // be 18.1822/2 ≈ 9.09 in x, so the floor spans roughly x ∈ [-9.09, 9.09] at y ∈ [-0.5, 0.5].
            var floor = new GameObject("WideBoxFloor");
            floor.transform.position = new Vector3(0f, -0.5f, 0f); // top edge at y=0
            floor.transform.localScale = new Vector3(WideBoxScaleX, 1f, 1f);
            var box = floor.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);
            EditorSceneManager.MoveGameObjectToScene(floor, child);

            // Two discs dropped near the EDGES of the wide floor — x=±7 is outside the unscaled 1×1 centre
            // (which spans only x ∈ [-0.5, 0.5]) but inside the scaled floor (x ∈ [-9.09, 9.09]). A buggy
            // unscaled floor would let these fall past; a correctly-scaled floor catches them at y≈0.5.
            MakeDisc(child, "EdgeDiscLeft", new Vector2(-7f, 4f), 0.5f);
            MakeDisc(child, "EdgeDiscRight", new Vector2(7f, 6f), 0.5f);

            SaveChildAndParent(child, WideBoxChild, WideBoxParent, "P12WideBox");
        }

        // ---------------------------------------------------------------------------------------------------
        // (2) Non-uniform circle floor. Scale (3, 1.4) on a radius-1 circle. cmax rule → effective radius = 3.

        static void BuildNonUniformCircle()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A CircleCollider2D radius 1, centered so that under the cmax rule (effective radius = max(3,1.4)=3)
            // its top sits at a known height. Place the circle center at y = -3 so a cmax=3 circle's top is at
            // y=0; the disc then rests at center y ≈ 0.5. If GameObject used a DIFFERENT rule (e.g. min-axis or
            // y-axis), the top would be at a different height and the rest parity would break — the empirical
            // pin of the radius rule.
            var floor = new GameObject("NonUniformCircleFloor");
            floor.transform.position = new Vector3(0f, -3f, 0f);
            floor.transform.localScale = new Vector3(3f, 1.4f, 1f);
            var circle = floor.AddComponent<CircleCollider2D>();
            circle.radius = 1f;
            EditorSceneManager.MoveGameObjectToScene(floor, child);

            // Disc dropped straight down onto the circle's apex (x=0): a sphere top is its highest point, so the
            // disc rests on the apex at center y ≈ 0.5 if effective radius = 3.
            MakeDisc(child, "ApexDisc", new Vector2(0f, 5f), 0.5f);

            SaveChildAndParent(child, NonUniformCircleChild, NonUniformCircleParent, "P12NonUniformCircle");
        }

        // ---------------------------------------------------------------------------------------------------
        // (3) Non-uniform capsule floor. Scale (2.5, 1.2) on a vertical capsule — caps re-derived from scaled size.

        static void BuildNonUniformCapsule()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A horizontal CapsuleCollider2D (a wide flat lozenge) so its top is a broad near-flat surface a disc
            // can rest on. Size (4, 2), direction Horizontal: radius = h/2 = 1, caps at x=±(w/2 - r)=±1. Under
            // scale (2.5, 1.2): scaled size (10, 2.4) → radius = 2.4/2 = 1.2, caps at x=±(5-1.2)=±3.8. The top of
            // a horizontal capsule is at center.y + radius. Place center at y = -1.2 so the scaled top is at y=0;
            // disc rests at center y ≈ 0.5. The cap-from-scaled-size deformation is what the rest height pins.
            var floor = new GameObject("NonUniformCapsuleFloor");
            floor.transform.position = new Vector3(0f, -1.2f, 0f);
            floor.transform.localScale = new Vector3(2.5f, 1.2f, 1f);
            var capsule = floor.AddComponent<CapsuleCollider2D>();
            capsule.size = new Vector2(4f, 2f);
            capsule.direction = CapsuleDirection2D.Horizontal;
            EditorSceneManager.MoveGameObjectToScene(floor, child);

            MakeDisc(child, "CapsuleDisc", new Vector2(0f, 5f), 0.5f);

            SaveChildAndParent(child, NonUniformCapsuleChild, NonUniformCapsuleParent, "P12NonUniformCapsule");
        }

        // ---------------------------------------------------------------------------------------------------
        // (4) Negative-X-scaled asymmetric polygon floor — a winding-flip probe.

        static void BuildNegativePolygon()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // An ASYMMETRIC convex polygon (a right trapezoid) authored CCW, with a FLAT TOP at y=0 so a disc has
            // a definite resting surface regardless of the mirror. Under Scale X = -2 the shape mirrors about x=0;
            // the flat top stays at y=0 (y unscaled) but the asymmetry flips left↔right. Box2D needs CCW: the
            // baker must reverse winding on the flip or PolygonGeometry.Create rejects the inside-out hull and the
            // disc falls through. Vertices CCW: bottom-left, bottom-right(wide), top-right, top-left(narrow).
            var floor = new GameObject("NegativePolygonFloor");
            floor.transform.position = new Vector3(0f, 0f, 0f);
            floor.transform.localScale = new Vector3(-2f, 1f, 1f);
            var poly = floor.AddComponent<PolygonCollider2D>();
            poly.points = new[]
            {
                new Vector2(-2f, -1f), // bottom-left
                new Vector2(3f, -1f), // bottom-right (wider on the right — the asymmetry)
                new Vector2(2f, 0f), // top-right
                new Vector2(-1f, 0f), // top-left (narrower)
            };
            EditorSceneManager.MoveGameObjectToScene(floor, child);

            // Disc dropped at x=0 (inside the flat top span both before and after the mirror) rests at y ≈ 0.5.
            MakeDisc(child, "PolyDisc", new Vector2(0f, 5f), 0.5f);

            SaveChildAndParent(child, NegativePolygonChild, NegativePolygonParent, "P12NegativePolygon");
        }

        // ---------------------------------------------------------------------------------------------------
        // (5) Negative-X-scaled asymmetric edge (chain) floor — the solid-side-is-winding probe.

        static void BuildNegativeEdge()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A wide edge chain whose winding faces solid-side UP (the proven Phase-1A right-to-left dished
            // winding). Under Scale X = -2 the chain mirrors; a baker that did not reverse the point order would
            // flip the solid side DOWN and the disc would fall through. The flat middle sits at y=0; a disc at x=0
            // rests at center y ≈ 0.5 only if the mirrored chain still faces solid-side up.
            var edgeGo = new GameObject("NegativeEdgeFloor");
            edgeGo.transform.position = new Vector3(0f, 0f, 0f);
            edgeGo.transform.localScale = new Vector3(-2f, 1f, 1f);
            var edge = edgeGo.AddComponent<EdgeCollider2D>();
            edge.points = new[]
            {
                new Vector2(8f, 1f),
                new Vector2(3f, 0f),
                new Vector2(-3f, 0f),
                new Vector2(-8f, 1f),
            };
            EditorSceneManager.MoveGameObjectToScene(edgeGo, child);

            MakeDisc(child, "EdgeDisc", new Vector2(0f, 5f), 0.5f);

            SaveChildAndParent(child, NegativeEdgeChild, NegativeEdgeParent, "P12NegativeEdge");
        }

        // ---------------------------------------------------------------------------------------------------
        // (6) Scaled-offset box floor — the offset must scale per-axis (signed).

        static void BuildScaledOffset()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A 1×1 BoxCollider2D with Offset (4, 0) on Scale (3, 1). The collider sits NOT at the GameObject
            // origin but at offset*scale = (12, 0) in world space (the GameObject is at origin). A disc dropped at
            // x=12 must rest on the offset-shifted, scaled box (width 3, spanning x ∈ [10.5, 13.5]); a disc at x=0
            // (the GameObject origin, where a DROPPED offset would put the box) must fall PAST (nothing there).
            // We compare only the resting disc; the harness oracle puts the GameObject box at the same scaled
            // offset, so a wrong offset scaling diverges the rest x.
            var floor = new GameObject("ScaledOffsetFloor");
            floor.transform.position = new Vector3(0f, -0.5f, 0f); // box top (with offset y=0) at world y=0
            floor.transform.localScale = new Vector3(3f, 1f, 1f);
            var box = floor.AddComponent<BoxCollider2D>();
            box.size = new Vector2(1f, 1f);
            box.offset = new Vector2(4f, 0f); // scaled to world +12 in x
            EditorSceneManager.MoveGameObjectToScene(floor, child);

            // Disc over the offset-shifted box top.
            MakeDisc(child, "OffsetDisc", new Vector2(12f, 5f), 0.5f);

            SaveChildAndParent(child, ScaledOffsetChild, ScaledOffsetParent, "P12ScaledOffset");
        }

        // ---------------------------------------------------------------------------------------------------
        // (7) Scaled composite floor.

        static GameObject MakeComposite(
            UnityEngine.SceneManagement.Scene scene,
            string name,
            Vector3 worldScale,
            Vector2[] childBoxCenters,
            Vector2 childBoxSize
        )
        {
            var root = new GameObject(name);
            root.transform.position = Vector3.zero;
            root.transform.localScale = worldScale;
            var rb = root.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var composite = root.AddComponent<CompositeCollider2D>();
            composite.geometryType = CompositeCollider2D.GeometryType.Polygons;
            composite.generationType = CompositeCollider2D.GenerationType.Synchronous;

            for (var i = 0; i < childBoxCenters.Length; i++)
            {
                var childGo = new GameObject($"{name}_Box{i}");
                childGo.transform.SetParent(root.transform, worldPositionStays: false);
                childGo.transform.localPosition = childBoxCenters[i];
                var box = childGo.AddComponent<BoxCollider2D>();
                box.size = childBoxSize;
                box.compositeOperation = Collider2D.CompositeOperation.Merge;
            }

            composite.GenerateGeometry();
            EditorSceneManager.MoveGameObjectToScene(root, scene);
            return root;
        }

        static void BuildScaledComposite()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A flat merged bar of 3 unit boxes (local span x ∈ [-1.5, 1.5], y ∈ [-1, 0], top at local y=0) on a
            // NON-UNIT scale (4, 1.5). The merged path is in the composite's local space, so the baker scales each
            // path point by (4, 1.5): the scaled bar spans x ∈ [-6, 6], top at world y=0. A disc dropped at x=5
            // (well outside the UNSCALED bar's x ∈ [-1.5, 1.5]) rests on the scaled bar only if the merged path
            // was scaled.
            MakeComposite(
                child,
                "ScaledCompositeBar",
                new Vector3(4f, 1.5f, 1f),
                new[]
                {
                    new Vector2(-1f, -0.5f),
                    new Vector2(0f, -0.5f),
                    new Vector2(1f, -0.5f),
                },
                new Vector2(1f, 1f)
            );
            MakeDisc(child, "CompositeDisc", new Vector2(5f, 5f), 0.5f);

            SaveChildAndParent(child, ScaledCompositeChild, ScaledCompositeParent, "P12ScaledComposite");
        }

        // ---------------------------------------------------------------------------------------------------
        // (8) Scaled custom-collider floor.

        static void BuildScaledCustom()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // A static CustomCollider2D carrying a single wide polygon base quad (local span x ∈ [-2, 2],
            // y ∈ [-1, 0], top at local y=0) on a NON-UNIT scale (3.5, 1.5). The custom shape's group-local
            // vertices are scaled by (3.5, 1.5): the scaled quad spans x ∈ [-7, 7], top at world y=0. A disc at
            // x=6 (outside the UNSCALED quad's x ∈ [-2, 2]) rests only if the group vertices were scaled.
            var root = new GameObject("ScaledCustomBody");
            root.transform.position = Vector3.zero;
            root.transform.localScale = new Vector3(3.5f, 1.5f, 1f);
            var rb = root.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var custom = root.AddComponent<CustomCollider2D>();

            var group = new PhysicsShapeGroup2D();
            group.AddPolygon(
                new List<Vector2>
                {
                    new Vector2(-2f, -1f),
                    new Vector2(2f, -1f),
                    new Vector2(2f, 0f),
                    new Vector2(-2f, 0f),
                }
            );
            custom.SetCustomShapes(group);
            EditorSceneManager.MoveGameObjectToScene(root, child);

            MakeDisc(child, "CustomDisc", new Vector2(6f, 5f), 0.5f);

            SaveChildAndParent(child, ScaledCustomChild, ScaledCustomParent, "P12ScaledCustom");
        }

        // ---------------------------------------------------------------------------------------------------
        // (9) Scaled DYNAMIC faller for the rendering-scale-preserved + smoothing pin.

        static void BuildScaledDynamic()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // An unscaled wide static floor (a plain box) to catch the faller.
            var floor = new GameObject("DynFloor");
            floor.transform.position = new Vector3(0f, -0.5f, 0f);
            var box = floor.AddComponent<BoxCollider2D>();
            box.size = new Vector2(40f, 1f);
            EditorSceneManager.MoveGameObjectToScene(floor, child);

            // A NON-UNIFORMLY scaled dynamic disc: a Rigidbody2D+CircleCollider2D on Scale (2, 3). Its baked
            // LocalToWorld must carry (2, 3) as the column lengths (rendering-scale-preserved), and the smoothing
            // system must preserve the scale through interpolation. The disc carries Interpolate so the smoothing
            // system processes it. Its collision radius is the cmax (=3) scaled circle, so it rests higher than a
            // unit disc — but the COMPARED witness here is the decomposed LocalToWorld scale, asserted directly by
            // the gate (not via RunParity, whose oracle would also need the scaled-circle radius, which it has
            // since the GameObject CircleCollider2D scales the same way).
            var disc = new GameObject("ScaledDynDisc");
            disc.transform.position = new Vector3(0f, 5f, 0f);
            disc.transform.localScale = new Vector3(2f, 3f, 1f);
            var rb = disc.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.interpolation = RigidbodyInterpolation2D.Interpolate;
            var circle = disc.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            EditorSceneManager.MoveGameObjectToScene(disc, child);

            SaveChildAndParent(child, ScaledDynamicChild, ScaledDynamicParent, "P12ScaledDynamic");
        }

        // ---------------------------------------------------------------------------------------------------

        static void SaveChildAndParent(
            UnityEngine.SceneManagement.Scene child,
            string childPath,
            string parentPath,
            string subSceneName
        )
        {
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(subSceneName + " SubScene");
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
