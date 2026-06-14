using Unity.Mathematics;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Populate method for the ray-scan parity fixture: per shape kind (Box, Circle, Capsule, Polygon) and per
    /// scale×size mode (unit-scale×unit-size; 2×scale×half-size; non-uniform), TWO collider-only static bodies
    /// at distinct world centres — one built-in <c>*Collider2D</c> ("builtin-bake" lane) and one
    /// <see cref="PhysicsShape2DAuthoring"/> ("custom-bake" lane) — authored to the SAME intended world shape.
    /// Verbatim port of <c>RayScanParityFixtureBuilder.BuildScene</c>'s 4×3×2 grid (every position, scale, size,
    /// vertex), authored through <see cref="Physics2DFixtures"/>' <c>NewChild</c> so the EditMode harness can bake
    /// it from a temp SubScene rather than the registered-build-settings scene the PlayMode builder wrote.
    /// </summary>
    public static partial class Physics2DFixtures
    {
        // Per-kind base intended-WORLD geometry (matches RayScanParityFixtureBuilder).
        static readonly float2 RsBoxBaseSize = new(1f, 1f);
        const float RsCircleBaseRadius = 0.5f;
        static readonly float2 RsCapsuleBaseSize = new(1f, 2f);
        const float RsPolygonCircumradius = 1f;

        // A 4 (kinds) × 3 (modes) × 2 (lanes) grid: kind selects the row block, mode the sub-row, lane the column.
        const float RsColSpacing = 14f; // builtin-bake at x=0, custom-bake at x=14 within a (kind,mode)
        const float RsRowSpacing = 30f; // each (kind,mode) on its own y row

        enum RsMode
        {
            UnitScaleUnitSize,
            DoubleScaleHalfSize,
            NonUniformScaleSize,
        }

        static readonly RsMode[] RsModes =
        {
            RsMode.UnitScaleUnitSize,
            RsMode.DoubleScaleHalfSize,
            RsMode.NonUniformScaleSize,
        };

        static float2 RsScaleFor(RsMode m) =>
            m switch
            {
                RsMode.UnitScaleUnitSize => new float2(1f, 1f),
                RsMode.DoubleScaleHalfSize => new float2(2f, 2f),
                _ => new float2(2f, 0.5f),
            };

        static float2 RsCentreFor(int kindIndex, int modeIndex, int lane) =>
            new float2(lane * RsColSpacing, (kindIndex * 3 + modeIndex) * RsRowSpacing);

        public static void RayScanParity(GameObject root)
        {
            for (var mi = 0; mi < RsModes.Length; mi++)
            {
                var m = RsModes[mi];
                var scale = RsScaleFor(m);

                // BOX (kindIndex 0)
                {
                    var box = RsBoxBaseSize / scale;
                    RsBuiltinBox(root, RsCentreFor(0, mi, 0), scale, (Vector2)box);
                    RsCustomBox(root, RsCentreFor(0, mi, 1), scale, box);
                }
                // CIRCLE (kindIndex 1) — authored radius / cmax(scale) yields the cmax world radius.
                {
                    var r = RsCircleBaseRadius / max(abs(scale.x), abs(scale.y));
                    RsBuiltinCircle(root, RsCentreFor(1, mi, 0), scale, r);
                    RsCustomCircle(root, RsCentreFor(1, mi, 1), scale, r);
                }
                // CAPSULE vertical (kindIndex 2)
                {
                    var cs = RsCapsuleBaseSize / scale;
                    RsBuiltinCapsule(root, RsCentreFor(2, mi, 0), scale, (Vector2)cs);
                    RsCustomCapsule(root, RsCentreFor(2, mi, 1), scale, cs);
                }
                // POLYGON hexagon (kindIndex 3)
                {
                    RsBuiltinPolygon(root, RsCentreFor(3, mi, 0), scale);
                    RsCustomPolygon(root, RsCentreFor(3, mi, 1), scale);
                }
            }
        }

        static GameObject RsMake(GameObject root, string name, float2 centre, float2 scale)
        {
            var go = NewChild(root, name, new Vector3(centre.x, centre.y, 0f));
            go.transform.localScale = new Vector3(scale.x, scale.y, 1f);
            return go;
        }

        // ---- builtin-collider lanes (collider-only static, baked by the package built-in bakers) --------------

        static void RsBuiltinBox(GameObject root, float2 c, float2 scale, Vector2 size)
        {
            var go = RsMake(root, "BuiltinBox", c, scale);
            go.AddComponent<BoxCollider2D>().size = size;
        }

        static void RsBuiltinCircle(GameObject root, float2 c, float2 scale, float radius)
        {
            var go = RsMake(root, "BuiltinCircle", c, scale);
            go.AddComponent<CircleCollider2D>().radius = radius;
        }

        static void RsBuiltinCapsule(GameObject root, float2 c, float2 scale, Vector2 size)
        {
            var go = RsMake(root, "BuiltinCapsule", c, scale);
            var col = go.AddComponent<CapsuleCollider2D>();
            col.direction = CapsuleDirection2D.Vertical;
            col.size = size;
        }

        static void RsBuiltinPolygon(GameObject root, float2 c, float2 scale)
        {
            var go = RsMake(root, "BuiltinPolygon", c, scale);
            var col = go.AddComponent<PolygonCollider2D>();
            col.SetPath(0, RsHexagon(scale));
        }

        // ---- custom PhysicsShape2DAuthoring lanes (collider-only static via the shape baker's fallback) --------

        static void RsCustomBox(GameObject root, float2 c, float2 scale, float2 size)
        {
            var go = RsMake(root, "CustomBox", c, scale);
            var sh = go.AddComponent<PhysicsShape2DAuthoring>();
            sh.Kind = PhysicsShape2DKind.Box;
            sh.BoxSize = size;
        }

        static void RsCustomCircle(GameObject root, float2 c, float2 scale, float radius)
        {
            var go = RsMake(root, "CustomCircle", c, scale);
            var sh = go.AddComponent<PhysicsShape2DAuthoring>();
            sh.Kind = PhysicsShape2DKind.Circle;
            sh.Radius = radius;
        }

        static void RsCustomCapsule(GameObject root, float2 c, float2 scale, float2 size)
        {
            var go = RsMake(root, "CustomCapsule", c, scale);
            var sh = go.AddComponent<PhysicsShape2DAuthoring>();
            sh.Kind = PhysicsShape2DKind.Capsule;
            sh.CapsuleVertical = true;
            sh.CapsuleSize = size;
        }

        static void RsCustomPolygon(GameObject root, float2 c, float2 scale)
        {
            var go = RsMake(root, "CustomPolygon", c, scale);
            var sh = go.AddComponent<PhysicsShape2DAuthoring>();
            sh.Kind = PhysicsShape2DKind.Polygon;
            sh.PolygonDecompose = false;
            var hex = RsHexagon(scale);
            var verts = new Vector2[hex.Length];
            for (var i = 0; i < hex.Length; i++)
                verts[i] = hex[i];
            sh.Vertices = verts;
        }

        // The unscaled local-space hexagon (CCW, circumradius RsPolygonCircumradius) divided by the transform scale,
        // so the transform localScale folds it back to the intended world hexagon (matches the gate's Hexagon).
        static Vector2[] RsHexagon(float2 scale)
        {
            var v = new Vector2[6];
            for (var i = 0; i < 6; i++)
            {
                sincos(radians(60f * i), out var s, out var co);
                var world = new float2(co, s) * RsPolygonCircumradius;
                v[i] = (Vector2)(world / scale);
            }
            return v;
        }
    }
}
