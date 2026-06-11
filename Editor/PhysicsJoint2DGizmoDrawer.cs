using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using Zori.Entities.Physics2D;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Editor
{
    /// <summary>
    /// Draws the joint outline (anchor markers + the connecting line + the axis + the limit arc / segment) as a
    /// scene gizmo, always-on (dim) and emphasised when selected (bright). A <c>[DrawGizmo]</c> static in the
    /// editor assembly rather than an <c>OnDrawGizmos</c> on the authoring component, so the always-included
    /// Authoring assembly carries no editor-only <c>Gizmos</c> calls — the same placement as
    /// <see cref="PhysicsShape2DGizmoDrawer"/>. The limit visualizations come from the pure
    /// <see cref="PhysicsJoint2DGizmos"/> POLYLINE math (shared with the scene handles), because <c>Gizmos</c>
    /// has no arc primitive.
    /// </summary>
    static class PhysicsJoint2DGizmoDrawer
    {
        static readonly Color k_Selected = new Color(0.45f, 0.78f, 1f, 0.95f);
        static readonly Color k_Unselected = new Color(0.30f, 0.55f, 0.85f, 0.45f);

        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected | GizmoType.Pickable)]
        static void DrawJointGizmo(PhysicsJoint2DAuthoring joint, GizmoType type)
        {
            var selected = (type & GizmoType.Selected) != 0;
            var prevColor = Gizmos.color;
            var prevMatrix = Gizmos.matrix;
            Gizmos.color = selected ? k_Selected : k_Unselected;
            Gizmos.matrix = joint.transform.localToWorldMatrix;

            var kind = joint.Kind;
            var connectedLocal = ConnectedPointInOwnerLocal(joint);
            var markerR = MarkerRadius(joint);

            if (UsesAnchors(kind))
            {
                Gizmos.DrawLine(ToV3(joint.Anchor), ToV3(connectedLocal));
                DrawMarker(joint.Anchor, markerR);
                DrawMarker(connectedLocal, markerR);
            }
            else
            {
                // Relative / Friction constrain the body origins.
                Gizmos.DrawLine(ToV3(float2.zero), ToV3(connectedLocal));
            }

            switch (kind)
            {
                case PhysicsJoint2DKind.Hinge:
                    DrawAngleLimit(joint, markerR);
                    break;
                case PhysicsJoint2DKind.Slider:
                    DrawAxis(joint, markerR);
                    DrawTranslationLimit(joint);
                    break;
                case PhysicsJoint2DKind.Wheel:
                    DrawAxis(joint, markerR);
                    break;
            }

            Gizmos.color = prevColor;
            Gizmos.matrix = prevMatrix;
        }

        static bool UsesAnchors(PhysicsJoint2DKind k) =>
            k != PhysicsJoint2DKind.Relative && k != PhysicsJoint2DKind.Friction;

        static float MarkerRadius(PhysicsJoint2DAuthoring joint)
        {
            // a small, scale-stable marker; the gizmo matrix carries the TRS, so use a constant local size.
            var s = joint.transform.lossyScale;
            var maxScale = Mathf.Max(1e-3f, Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.y)));
            return 0.08f / maxScale;
        }

        static void DrawMarker(float2 p, float r)
        {
            Gizmos.DrawWireSphere(ToV3(p), r);
        }

        static void DrawAxis(PhysicsJoint2DAuthoring joint, float r)
        {
            var endpoint = PhysicsJoint2DGizmos.AxisEndpoint(joint.Anchor, joint.AxisAngle, r * 10f);
            Gizmos.DrawLine(ToV3(joint.Anchor), ToV3(endpoint));
        }

        static void DrawAngleLimit(PhysicsJoint2DAuthoring joint, float r)
        {
            if (!joint.UseLimits)
                return;
            var arc = PhysicsJoint2DGizmos.AngleLimitArcPoints(
                joint.Anchor,
                joint.LowerLimit,
                joint.UpperLimit,
                r * 10f
            );
            DrawPolyline(arc);
            // the two limit radii
            Gizmos.DrawLine(
                ToV3(joint.Anchor),
                ToV3(PhysicsJoint2DGizmos.AngleLimitMarker(joint.Anchor, joint.LowerLimit, r * 10f))
            );
            Gizmos.DrawLine(
                ToV3(joint.Anchor),
                ToV3(PhysicsJoint2DGizmos.AngleLimitMarker(joint.Anchor, joint.UpperLimit, r * 10f))
            );
        }

        static void DrawTranslationLimit(PhysicsJoint2DAuthoring joint)
        {
            if (!joint.UseLimits)
                return;
            PhysicsJoint2DGizmos.TranslationLimitSegment(
                joint.Anchor,
                joint.AxisAngle,
                joint.LowerLimit,
                joint.UpperLimit,
                out var a,
                out var b
            );
            Gizmos.DrawLine(ToV3(a), ToV3(b));
        }

        static float2 ConnectedPointInOwnerLocal(PhysicsJoint2DAuthoring joint)
        {
            var ownerTransform = joint.transform;
            Vector3 world;
            if (joint.Kind == PhysicsJoint2DKind.Target)
                world = new Vector3(joint.ConnectedAnchor.x, joint.ConnectedAnchor.y, 0f);
            else if (joint.ConnectedBody != null)
                world = joint.ConnectedBody.transform.TransformPoint(
                    new Vector3(joint.ConnectedAnchor.x, joint.ConnectedAnchor.y, 0f)
                );
            else
                world = new Vector3(joint.ConnectedAnchor.x, joint.ConnectedAnchor.y, 0f);
            var local = ownerTransform.InverseTransformPoint(world);
            return new float2(local.x, local.y);
        }

        static void DrawPolyline(float2[] pts)
        {
            if (pts == null || pts.Length < 2)
                return;
            for (var i = 0; i < pts.Length - 1; i++)
                Gizmos.DrawLine(ToV3(pts[i]), ToV3(pts[i + 1]));
        }

        static Vector3 ToV3(float2 p) => new Vector3(p.x, p.y, 0f);
    }
}
