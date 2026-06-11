using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using Zori.Entities.Physics2D;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Editor
{
    /// <summary>
    /// Draws the 2D shape outline as a scene gizmo, always-on (dim) and emphasised when selected (bright). A
    /// <c>[DrawGizmo]</c> static in the editor assembly rather than an <c>OnDrawGizmos</c> on the authoring
    /// component, so the always-included Authoring assembly carries no editor-only <c>Gizmos</c> calls and the
    /// whole editor surface stays <c>includePlatforms:[Editor]</c>. The outline points come from the pure
    /// <see cref="PhysicsShape2DGizmos"/> math (shared with the scene handles), transformed by the GameObject's
    /// <c>localToWorldMatrix</c>.
    /// </summary>
    static class PhysicsShape2DGizmoDrawer
    {
        static readonly Color k_Selected = new Color(0.57f, 0.96f, 0.55f, 0.95f);
        static readonly Color k_Unselected = new Color(0.33f, 0.78f, 0.30f, 0.45f);

        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected | GizmoType.Pickable)]
        static void DrawShapeGizmo(PhysicsShape2DAuthoring shape, GizmoType type)
        {
            var selected = (type & GizmoType.Selected) != 0;
            var prevColor = Gizmos.color;
            var prevMatrix = Gizmos.matrix;
            Gizmos.color = selected ? k_Selected : k_Unselected;
            Gizmos.matrix = shape.transform.localToWorldMatrix;

            switch (shape.Kind)
            {
                case PhysicsShape2DKind.Circle:
                    DrawClosed(PhysicsShape2DGizmos.CircleOutline(shape.Offset, shape.Radius), true);
                    break;
                case PhysicsShape2DKind.Box:
                    DrawClosed(
                        PhysicsShape2DGizmos.BoxOutline(shape.Offset, shape.BoxSize, shape.BoxAngle),
                        true
                    );
                    break;
                case PhysicsShape2DKind.Capsule:
                    shape.GetCapsuleCenters(out var r, out var c1, out var c2);
                    DrawClosed(
                        PhysicsShape2DGizmos.CapsuleOutline(
                            c1 + shape.Offset,
                            c2 + shape.Offset,
                            r
                        ),
                        true
                    );
                    break;
                case PhysicsShape2DKind.Polygon:
                    DrawClosed(
                        PhysicsShape2DGizmos.PolygonOutline(ToFloat2(shape.Vertices), shape.Offset),
                        true
                    );
                    break;
                case PhysicsShape2DKind.Edge:
                    DrawClosed(
                        PhysicsShape2DGizmos.EdgeOutline(
                            ToFloat2(shape.Vertices),
                            shape.Offset,
                            shape.EdgeIsLoop
                        ),
                        shape.EdgeIsLoop
                    );
                    break;
            }

            Gizmos.color = prevColor;
            Gizmos.matrix = prevMatrix;
        }

        static void DrawClosed(float2[] pts, bool close)
        {
            if (pts == null || pts.Length < 2)
                return;
            for (var i = 0; i < pts.Length - 1; i++)
                Gizmos.DrawLine(ToV3(pts[i]), ToV3(pts[i + 1]));
            if (close)
                Gizmos.DrawLine(ToV3(pts[pts.Length - 1]), ToV3(pts[0]));
        }

        static Vector3 ToV3(float2 p) => new Vector3(p.x, p.y, 0f);

        static float2[] ToFloat2(Vector2[] verts)
        {
            var pts = new float2[verts.Length];
            for (var i = 0; i < verts.Length; i++)
                pts[i] = (float2)verts[i];
            return pts;
        }
    }
}
