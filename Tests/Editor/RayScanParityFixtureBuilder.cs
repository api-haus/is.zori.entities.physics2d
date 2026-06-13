using System.Collections.Generic;
using System.IO;
using Unity.Mathematics;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Click-free authoring of the ray-scan parity SubScene fixture: per shape kind (Box, Circle, Capsule,
    /// Polygon) and per scale×size mode (unit-scale×unit-size; 2×scale×half-size; non-uniform), TWO collider-only
    /// static bodies at distinct world centres — one built-in <c>*Collider2D</c> ("builtin-bake" lane) and one
    /// <see cref="PhysicsShape2DAuthoring"/> ("custom-bake" lane) — authored to the SAME intended world shape and
    /// baked through the REAL package bakers. The PlayMode gate (<see cref="AuthoringBakeRayScanGate"/>) loads
    /// this, scans each baked shape with 360 inward rays, and asserts the builtin-bake lane, the custom-bake lane,
    /// and a live native <c>*Collider2D</c> built in-test all return epsilon-matching per-ray distances.
    /// </summary>
    /// <remarks>
    /// <para>This is the FAITHFUL authoring→bake confirmation the precursor <c>PhysicalExtentParityGate</c> never
    /// ran: that gate built runtime <see cref="PhysicsShape2D"/> structs directly and held transform scale at 1, so
    /// it never exercised <c>PhysicsShape2DAuthoringBaker</c> reading <c>BoxSize</c> / <c>lossyScale</c>. Here the
    /// shapes are REAL components on REAL scaled Transforms, baked by the importer, so a divergence between the
    /// built-in box baker and the custom box baker — or in how <c>BoxSize</c> is read — would surface. The
    /// in-process bake API (<c>BakingUtility.BakeGameObjects</c>) is internal to <c>Unity.Entities.Hybrid</c>, so a
    /// SubScene is the only in-package way to drive the real bakers (FilterBakeParityGate / Phase12ColliderScaleGate
    /// precedent).</para>
    ///
    /// <para>Every static body is collider-only (no <c>Rigidbody2D</c> / <c>PhysicsBody2DAuthoring</c>), so each
    /// bakes to one static body carrying its shape, and the gate matches baked entities to (kind, mode, lane) by
    /// the baked <c>initialPosition</c>. Centres are spaced far apart (≥ 12 m) so a small ring's 360 rays never
    /// reach a neighbouring shape. Run via the menu or
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.RayScanParityFixtureBuilder.BuildAll</c>.</para>
    /// </remarks>
    public static class RayScanParityFixtureBuilder
    {
        public const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";
        public const string Parent = FixtureRoot + "/RayScanParity.unity";
        public const string Child = FixtureRoot + "/RayScanParity_Sub.unity";

        // The layout MUST match AuthoringBakeRayScanGate's mirror constants. A 4 (kinds) × 3 (modes) × 2 (lanes)
        // grid: kind selects the row block, mode the sub-row, lane the column. Centres are spaced 14 m so the
        // largest ring (R ≈ 3) never reaches a neighbour.
        const float ColSpacing = 14f; // builtin-bake at x=0, custom-bake at x=14 within a (kind,mode)
        const float RowSpacing = 30f; // each (kind,mode) on its own y row

        // Per-kind base intended-WORLD geometry (matches the gate).
        public static readonly float2 BoxBaseSize = new(1f, 1f);
        public const float CircleBaseRadius = 0.5f;
        public static readonly float2 CapsuleBaseSize = new(1f, 2f);
        public const float PolygonCircumradius = 1f;

        [MenuItem("Tools/Zori/Build Entities Physics2D Ray-Scan Parity Fixture")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);
            BuildScene();
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Entities Physics2D ray-scan parity fixture built (RayScanParity).");
        }

        // The three modes' (scale, authoredSize-factor). AuthoredSize = baseSize / scale, so the world shape is the
        // same in every mode (matches the gate's AuthoredSizeFor).
        enum Mode
        {
            UnitScaleUnitSize,
            DoubleScaleHalfSize,
            NonUniformScaleSize,
        }

        static readonly Mode[] Modes = { Mode.UnitScaleUnitSize, Mode.DoubleScaleHalfSize, Mode.NonUniformScaleSize };

        static float2 ScaleFor(Mode m) =>
            m switch
            {
                Mode.UnitScaleUnitSize => new float2(1f, 1f),
                Mode.DoubleScaleHalfSize => new float2(2f, 2f),
                _ => new float2(2f, 0.5f),
            };

        // The world centre for a (kind, mode, lane). Mirrored verbatim in the gate.
        public static float2 CentreFor(int kindIndex, int modeIndex, int lane) =>
            new float2(lane * ColSpacing, (kindIndex * 3 + modeIndex) * RowSpacing);

        static void BuildScene()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            for (var mi = 0; mi < Modes.Length; mi++)
            {
                var m = Modes[mi];
                var scale = ScaleFor(m);

                // BOX (kindIndex 0)
                {
                    var box = BoxBaseSize / scale;
                    BuiltinBox(child, CentreFor(0, mi, 0), scale, (Vector2)box);
                    CustomBox(child, CentreFor(0, mi, 1), scale, box);
                }
                // CIRCLE (kindIndex 1) — authored radius / cmax(scale) yields the cmax world radius.
                {
                    var r = CircleBaseRadius / math.max(math.abs(scale.x), math.abs(scale.y));
                    BuiltinCircle(child, CentreFor(1, mi, 0), scale, r);
                    CustomCircle(child, CentreFor(1, mi, 1), scale, r);
                }
                // CAPSULE vertical (kindIndex 2)
                {
                    var cs = CapsuleBaseSize / scale;
                    BuiltinCapsule(child, CentreFor(2, mi, 0), scale, (Vector2)cs);
                    CustomCapsule(child, CentreFor(2, mi, 1), scale, cs);
                }
                // POLYGON hexagon (kindIndex 3)
                {
                    BuiltinPolygon(child, CentreFor(3, mi, 0), scale);
                    CustomPolygon(child, CentreFor(3, mi, 1), scale);
                }
            }

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, Child);
            SaveParentWithSubScene(Parent, Child, "RayScanParity SubScene");
            RegisterSceneInBuildSettings(Parent);
            RegisterSceneInBuildSettings(Child);
        }

        // ---- builtin-collider lanes (collider-only static, baked by the package built-in bakers) --------------

        static GameObject Make(Scene scene, string name, float2 centre, float2 scale)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(centre.x, centre.y, 0f);
            go.transform.localScale = new Vector3(scale.x, scale.y, 1f);
            EditorSceneManager.MoveGameObjectToScene(go, scene);
            return go;
        }

        static void BuiltinBox(Scene s, float2 c, float2 scale, Vector2 size)
        {
            var go = Make(s, "BuiltinBox", c, scale);
            go.AddComponent<BoxCollider2D>().size = size;
        }

        static void BuiltinCircle(Scene s, float2 c, float2 scale, float radius)
        {
            var go = Make(s, "BuiltinCircle", c, scale);
            go.AddComponent<CircleCollider2D>().radius = radius;
        }

        static void BuiltinCapsule(Scene s, float2 c, float2 scale, Vector2 size)
        {
            var go = Make(s, "BuiltinCapsule", c, scale);
            var col = go.AddComponent<CapsuleCollider2D>();
            col.direction = CapsuleDirection2D.Vertical;
            col.size = size;
        }

        static void BuiltinPolygon(Scene s, float2 c, float2 scale)
        {
            var go = Make(s, "BuiltinPolygon", c, scale);
            var col = go.AddComponent<PolygonCollider2D>();
            col.SetPath(0, Hexagon(scale));
        }

        // ---- custom PhysicsShape2DAuthoring lanes (collider-only static via the shape baker's fallback) --------

        static void CustomBox(Scene s, float2 c, float2 scale, float2 size)
        {
            var go = Make(s, "CustomBox", c, scale);
            var sh = go.AddComponent<PhysicsShape2DAuthoring>();
            sh.Kind = PhysicsShape2DKind.Box;
            sh.BoxSize = size;
        }

        static void CustomCircle(Scene s, float2 c, float2 scale, float radius)
        {
            var go = Make(s, "CustomCircle", c, scale);
            var sh = go.AddComponent<PhysicsShape2DAuthoring>();
            sh.Kind = PhysicsShape2DKind.Circle;
            sh.Radius = radius;
        }

        static void CustomCapsule(Scene s, float2 c, float2 scale, float2 size)
        {
            var go = Make(s, "CustomCapsule", c, scale);
            var sh = go.AddComponent<PhysicsShape2DAuthoring>();
            sh.Kind = PhysicsShape2DKind.Capsule;
            sh.CapsuleVertical = true;
            sh.CapsuleSize = size;
        }

        static void CustomPolygon(Scene s, float2 c, float2 scale)
        {
            var go = Make(s, "CustomPolygon", c, scale);
            var sh = go.AddComponent<PhysicsShape2DAuthoring>();
            sh.Kind = PhysicsShape2DKind.Polygon;
            sh.PolygonDecompose = false;
            var hex = Hexagon(scale);
            var verts = new Vector2[hex.Length];
            for (var i = 0; i < hex.Length; i++)
                verts[i] = hex[i];
            sh.Vertices = verts;
        }

        // The unscaled local-space hexagon (CCW, circumradius PolygonCircumradius) divided by the transform scale,
        // so the transform localScale folds it back to the intended world hexagon (matches the gate's Hexagon).
        static Vector2[] Hexagon(float2 scale)
        {
            var v = new Vector2[6];
            for (var i = 0; i < 6; i++)
            {
                var a = math.radians(60f * i);
                math.sincos(a, out var s, out var co);
                var world = new float2(co, s) * PolygonCircumradius;
                v[i] = (Vector2)(world / scale);
            }
            return v;
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
            if (scenes.Exists(x => x.path == scenePath))
                return;
            scenes.Add(new EditorBuildSettingsScene(scenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }
    }
}
