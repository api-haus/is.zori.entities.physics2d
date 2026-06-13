using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using Zori.Entities.Physics2D;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Editor
{
    /// <summary>
    /// The custom inspector for <see cref="PhysicsJoint2DAuthoring"/> — the 2D-native analogue of the DOTS
    /// sample's joint editor family (<c>BallAndSocketJointEditor</c> / <c>LimitedHingeJointEditor</c> /
    /// <c>RagdollJointEditor</c>, structure-only reference), built exactly as
    /// <see cref="PhysicsShape2DAuthoringEditor"/> was. A KIND-SWITCHED inspector shows only the fields the
    /// selected joint kind's baker consumes (the <see cref="PhysicsJoint2DGizmos"/>-backed scene handles likewise
    /// switch on kind), and the <see cref="PhysicsJoint2DKind.Target"/> single-body joint hides its connected-body,
    /// collide-connected, and max-torque fields (all forced by the Target baker arm). No perspective math: flat
    /// <c>UnityEditor.Handles</c> primitives in the XY plane.
    /// </summary>
    [CustomEditor(typeof(PhysicsJoint2DAuthoring))]
    [CanEditMultipleObjects]
    public sealed class PhysicsJoint2DAuthoringEditor : UnityEditor.Editor
    {
        SerializedProperty m_Kind;
        SerializedProperty m_ConnectedBody;
        SerializedProperty m_Anchor;
        SerializedProperty m_ConnectedAnchor;
        SerializedProperty m_AxisAngle;
        SerializedProperty m_UseMotor;
        SerializedProperty m_MotorSpeed;
        SerializedProperty m_MaxMotorEffort;
        SerializedProperty m_UseLimits;
        SerializedProperty m_LowerLimit;
        SerializedProperty m_UpperLimit;
        SerializedProperty m_Frequency;
        SerializedProperty m_DampingRatio;
        SerializedProperty m_RestLength;
        SerializedProperty m_LinearOffset;
        SerializedProperty m_AngularOffset;
        SerializedProperty m_MaxForce;
        SerializedProperty m_MaxTorque;
        SerializedProperty m_CollideConnected;
        SerializedProperty m_BreakAction;
        SerializedProperty m_BreakForce;
        SerializedProperty m_BreakTorque;

        const string k_UndoHandle = "Edit Physics Joint 2D";
        static readonly Color k_HandleColor = new Color(0.45f, 0.78f, 1f, 1f);

        void OnEnable()
        {
            m_Kind = serializedObject.FindProperty("m_Kind");
            m_ConnectedBody = serializedObject.FindProperty("m_ConnectedBody");
            m_Anchor = serializedObject.FindProperty("m_Anchor");
            m_ConnectedAnchor = serializedObject.FindProperty("m_ConnectedAnchor");
            m_AxisAngle = serializedObject.FindProperty("m_AxisAngle");
            m_UseMotor = serializedObject.FindProperty("m_UseMotor");
            m_MotorSpeed = serializedObject.FindProperty("m_MotorSpeed");
            m_MaxMotorEffort = serializedObject.FindProperty("m_MaxMotorEffort");
            m_UseLimits = serializedObject.FindProperty("m_UseLimits");
            m_LowerLimit = serializedObject.FindProperty("m_LowerLimit");
            m_UpperLimit = serializedObject.FindProperty("m_UpperLimit");
            m_Frequency = serializedObject.FindProperty("m_Frequency");
            m_DampingRatio = serializedObject.FindProperty("m_DampingRatio");
            m_RestLength = serializedObject.FindProperty("m_RestLength");
            m_LinearOffset = serializedObject.FindProperty("m_LinearOffset");
            m_AngularOffset = serializedObject.FindProperty("m_AngularOffset");
            m_MaxForce = serializedObject.FindProperty("m_MaxForce");
            m_MaxTorque = serializedObject.FindProperty("m_MaxTorque");
            m_CollideConnected = serializedObject.FindProperty("m_CollideConnected");
            m_BreakAction = serializedObject.FindProperty("m_BreakAction");
            m_BreakForce = serializedObject.FindProperty("m_BreakForce");
            m_BreakTorque = serializedObject.FindProperty("m_BreakTorque");
        }

        // ----- the per-kind consumed-field map, derived from PhysicsJoint2DAuthoringBaker (one source of truth) -----

        static bool UsesAnchors(PhysicsJoint2DKind k) =>
            // Relative / Friction zero both anchors at bake — showing them would mislead.
            k != PhysicsJoint2DKind.Relative
            && k != PhysicsJoint2DKind.Friction;

        static bool UsesAxis(PhysicsJoint2DKind k) => k == PhysicsJoint2DKind.Slider || k == PhysicsJoint2DKind.Wheel;

        static bool UsesMotor(PhysicsJoint2DKind k) =>
            k == PhysicsJoint2DKind.Hinge || k == PhysicsJoint2DKind.Slider || k == PhysicsJoint2DKind.Wheel;

        static bool UsesLimit(PhysicsJoint2DKind k) =>
            // Wheel's m_UseLimits is inert (the baker forces enableLimit=false) — only Hinge + Slider have a limit.
            k == PhysicsJoint2DKind.Hinge
            || k == PhysicsJoint2DKind.Slider;

        static bool UsesSpring(PhysicsJoint2DKind k) =>
            k == PhysicsJoint2DKind.Wheel
            || k == PhysicsJoint2DKind.Spring
            || k == PhysicsJoint2DKind.Fixed
            || k == PhysicsJoint2DKind.Target;

        static bool UsesRestLength(PhysicsJoint2DKind k) =>
            k == PhysicsJoint2DKind.Distance || k == PhysicsJoint2DKind.Spring;

        static bool UsesOffset(PhysicsJoint2DKind k) =>
            // Only Relative authors a maintained offset; Friction / Target zero it at bake.
            k == PhysicsJoint2DKind.Relative;

        static bool UsesForceCaps(PhysicsJoint2DKind k) =>
            k == PhysicsJoint2DKind.Relative || k == PhysicsJoint2DKind.Friction || k == PhysicsJoint2DKind.Target;

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUI.BeginChangeCheck();

            EditorGUILayout.PropertyField(m_Kind);
            var kind = (PhysicsJoint2DKind)m_Kind.enumValueIndex;
            var target = kind == PhysicsJoint2DKind.Target;

            // --- connection: Target is single-body (no connected body) ---
            if (!target)
                EditorGUILayout.PropertyField(m_ConnectedBody);

            if (UsesAnchors(kind))
            {
                EditorGUILayout.PropertyField(m_Anchor);
                EditorGUILayout.PropertyField(
                    m_ConnectedAnchor,
                    target
                        ? new GUIContent("World Target", "The world-space point this body is pulled toward.")
                        : new GUIContent("Connected Anchor")
                );
            }

            // --- per-kind sub-surfaces (only the fields the matching baker arm consumes) ---
            if (UsesAxis(kind))
                EditorGUILayout.PropertyField(
                    m_AxisAngle,
                    new GUIContent("Axis Angle (deg)", "The slide / suspension axis direction in degrees.")
                );

            if (UsesMotor(kind))
            {
                EditorGUILayout.PropertyField(m_UseMotor);
                if (m_UseMotor.boolValue || m_UseMotor.hasMultipleDifferentValues)
                    using (new EditorGUI.IndentLevelScope())
                    {
                        var speedUnit = kind == PhysicsJoint2DKind.Slider ? "m/s" : "deg/s";
                        EditorGUILayout.PropertyField(m_MotorSpeed, new GUIContent($"Motor Speed ({speedUnit})"));
                        var effortUnit = kind == PhysicsJoint2DKind.Slider ? "N" : "N·m";
                        EditorGUILayout.PropertyField(
                            m_MaxMotorEffort,
                            new GUIContent($"Max Motor Effort ({effortUnit})")
                        );
                    }
            }

            if (UsesLimit(kind))
            {
                EditorGUILayout.PropertyField(m_UseLimits);
                if (m_UseLimits.boolValue || m_UseLimits.hasMultipleDifferentValues)
                    using (new EditorGUI.IndentLevelScope())
                    {
                        var limitUnit = kind == PhysicsJoint2DKind.Hinge ? "deg" : "m";
                        EditorGUILayout.PropertyField(m_LowerLimit, new GUIContent($"Lower Limit ({limitUnit})"));
                        EditorGUILayout.PropertyField(m_UpperLimit, new GUIContent($"Upper Limit ({limitUnit})"));
                    }
            }

            if (UsesSpring(kind))
            {
                var weld = kind == PhysicsJoint2DKind.Fixed;
                EditorGUILayout.PropertyField(
                    m_Frequency,
                    new GUIContent(
                        weld ? "Weld Frequency (Hz)" : "Spring Frequency (Hz)",
                        weld
                            ? "0 Hz is a rigid weld; higher values let the weld flex like a spring."
                            : "The spring / suspension stiffness in Hz."
                    )
                );
                EditorGUILayout.PropertyField(m_DampingRatio, new GUIContent(weld ? "Weld Damping" : "Spring Damping"));
            }

            if (UsesRestLength(kind))
                EditorGUILayout.PropertyField(
                    m_RestLength,
                    new GUIContent("Rest Length (m)", "The distance the constraint holds between the anchors.")
                );

            if (UsesOffset(kind))
            {
                EditorGUILayout.PropertyField(m_LinearOffset, new GUIContent("Linear Offset (m)"));
                EditorGUILayout.PropertyField(m_AngularOffset, new GUIContent("Angular Offset (deg)"));
            }

            if (UsesForceCaps(kind))
            {
                EditorGUILayout.PropertyField(
                    m_MaxForce,
                    new GUIContent("Max Force (N)", "Maximum linear correction force; 0 turns the cap off.")
                );
                // Target is a point constraint — the baker forces maxTorque = 0, so the field is hidden.
                if (!target)
                    EditorGUILayout.PropertyField(
                        m_MaxTorque,
                        new GUIContent("Max Torque (N·m)", "Maximum angular correction torque; 0 turns the cap off.")
                    );
            }

            // --- collide + break (shared; Target's collide-connected is a single-body no-op, hidden) ---
            if (!target)
                EditorGUILayout.PropertyField(m_CollideConnected);

            EditorGUILayout.PropertyField(m_BreakAction);
            EditorGUILayout.PropertyField(m_BreakForce);
            EditorGUILayout.PropertyField(m_BreakTorque);

            if (EditorGUI.EndChangeCheck())
                serializedObject.ApplyModifiedProperties();

            DrawStatus(kind);
        }

        void DrawStatus(PhysicsJoint2DKind kind)
        {
            if (kind == PhysicsJoint2DKind.Target)
            {
                EditorGUILayout.HelpBox(
                    "Target is a single-body joint: it pulls this body toward the World Target point via a "
                        + "spring. It has no connected body, no collide-connected, and no torque cap.",
                    MessageType.Info
                );
            }
            else if (m_ConnectedBody.objectReferenceValue == null && !m_ConnectedBody.hasMultipleDifferentValues)
            {
                EditorGUILayout.HelpBox(
                    "No connected body — this joint anchors to a static point at the world origin (the "
                        + "built-in null-connectedBody path). Connected Anchor is then a world-space point.",
                    MessageType.Info
                );
            }

            if (
                UsesLimit(kind)
                && m_UseLimits.boolValue
                && !m_UseLimits.hasMultipleDifferentValues
                && !m_LowerLimit.hasMultipleDifferentValues
                && !m_UpperLimit.hasMultipleDifferentValues
                && m_LowerLimit.floatValue > m_UpperLimit.floatValue
            )
            {
                EditorGUILayout.HelpBox(
                    "Lower Limit exceeds Upper Limit — the joint has an empty range.",
                    MessageType.Warning
                );
            }

            // The genuinely-sprung kinds (Wheel / Spring / Target) want a positive frequency; Fixed's 0 Hz is a
            // valid rigid weld, so it is excluded.
            if (
                (
                    kind == PhysicsJoint2DKind.Wheel
                    || kind == PhysicsJoint2DKind.Spring
                    || kind == PhysicsJoint2DKind.Target
                )
                && !m_Frequency.hasMultipleDifferentValues
                && m_Frequency.floatValue <= 0f
            )
            {
                EditorGUILayout.HelpBox(
                    "Spring Frequency 0 disables the spring — this kind needs a positive frequency to act.",
                    MessageType.Info
                );
            }
        }

        // ----- the scene-view 2D handles -----

        void OnSceneGUI()
        {
            var joint = (PhysicsJoint2DAuthoring)target;
            var ownerTransform = joint.transform;

            var prevMatrix = Handles.matrix;
            var prevColor = Handles.color;
            Handles.matrix = ownerTransform.localToWorldMatrix; // owner = bodyB frame; anchors are bodyB-local
            Handles.color = k_HandleColor;

            var kind = joint.Kind;

            // The connected point expressed in the OWNER's local frame, so one Handles.matrix draws everything.
            var connectedLocal = ConnectedPointInOwnerLocal(joint);

            if (UsesAnchors(kind))
            {
                AnchorHandles(joint, connectedLocal);
            }
            else
            {
                // Relative / Friction constrain the body ORIGINS — draw only the origin-to-connected line.
                Handles.DrawLine(ToV3(float2.zero), ToV3(connectedLocal));
            }

            switch (kind)
            {
                case PhysicsJoint2DKind.Hinge:
                    AngleLimitHandles(joint);
                    break;
                case PhysicsJoint2DKind.Slider:
                    AxisHandle(joint);
                    TranslationLimitHandles(joint);
                    break;
                case PhysicsJoint2DKind.Wheel:
                    AxisHandle(joint);
                    break;
            }

            Handles.matrix = prevMatrix;
            Handles.color = prevColor;
        }

        // The connected anchor, mapped into the owner's local space. World for Target; the connected body's
        // local-anchor transformed to owner-local when a body is set; the world origin otherwise.
        static float2 ConnectedPointInOwnerLocal(PhysicsJoint2DAuthoring joint)
        {
            var ownerTransform = joint.transform;
            Vector3 world;
            if (joint.Kind == PhysicsJoint2DKind.Target)
            {
                // ConnectedAnchor is already a world-space target point.
                world = new Vector3(joint.ConnectedAnchor.x, joint.ConnectedAnchor.y, 0f);
            }
            else if (joint.ConnectedBody != null)
            {
                world = joint.ConnectedBody.transform.TransformPoint(
                    new Vector3(joint.ConnectedAnchor.x, joint.ConnectedAnchor.y, 0f)
                );
            }
            else
            {
                // Null connected body → the static world anchor at the origin; ConnectedAnchor is a world point.
                world = new Vector3(joint.ConnectedAnchor.x, joint.ConnectedAnchor.y, 0f);
            }
            var local = ownerTransform.InverseTransformPoint(world);
            return new float2(local.x, local.y);
        }

        void AnchorHandles(PhysicsJoint2DAuthoring joint, float2 connectedLocal)
        {
            // connecting line owner-anchor → connected point (body / world anchor / target)
            Handles.DrawLine(ToV3(joint.Anchor), ToV3(connectedLocal));

            // owner anchor (bodyB-local)
            EditorGUI.BeginChangeCheck();
            var newAnchor = Handles.FreeMoveHandle(
                ToV3(joint.Anchor),
                HandleSize(joint.Anchor),
                Vector3.zero,
                Handles.CircleHandleCap
            );
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(joint, k_UndoHandle);
                joint.Anchor = ToF2(newAnchor);
            }

            // connected / world-target anchor (drawn in owner-local; written back in its own frame)
            EditorGUI.BeginChangeCheck();
            var newConnected = Handles.FreeMoveHandle(
                ToV3(connectedLocal),
                HandleSize(connectedLocal),
                Vector3.zero,
                Handles.DotHandleCap
            );
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(joint, k_UndoHandle);
                joint.ConnectedAnchor = ConnectedAnchorFromOwnerLocal(joint, ToF2(newConnected));
            }
        }

        // Inverse of ConnectedPointInOwnerLocal: map a dragged owner-local point back to the field's own frame.
        static float2 ConnectedAnchorFromOwnerLocal(PhysicsJoint2DAuthoring joint, float2 ownerLocal)
        {
            var ownerTransform = joint.transform;
            var world = ownerTransform.TransformPoint(new Vector3(ownerLocal.x, ownerLocal.y, 0f));
            if (joint.Kind == PhysicsJoint2DKind.Target || joint.ConnectedBody == null)
            {
                // world-space target point / world anchor frame
                return new float2(world.x, world.y);
            }
            var local = joint.ConnectedBody.transform.InverseTransformPoint(world);
            return new float2(local.x, local.y);
        }

        void AxisHandle(PhysicsJoint2DAuthoring joint)
        {
            var anchor = joint.Anchor;
            var len = HandleSize(anchor) * 12f; // a screen-relative axis length; the angle math is exact regardless
            var endpoint = PhysicsJoint2DGizmos.AxisEndpoint(anchor, joint.AxisAngle, len);
            Handles.DrawLine(ToV3(anchor), ToV3(endpoint));

            EditorGUI.BeginChangeCheck();
            var moved = Handles.FreeMoveHandle(
                ToV3(endpoint),
                HandleSize(endpoint),
                Vector3.zero,
                Handles.ConeHandleCap
            );
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(joint, k_UndoHandle);
                joint.AxisAngle = PhysicsJoint2DGizmos.AngleFromAxisDrag(anchor, ToF2(moved));
            }
        }

        void AngleLimitHandles(PhysicsJoint2DAuthoring joint)
        {
            if (!joint.UseLimits)
                return;
            var center = joint.Anchor;
            var r = HandleSize(center) * 12f; // screen-relative arc radius (cosmetic; the angle math is exact)

            // the swept arc as a polyline
            var arc = PhysicsJoint2DGizmos.AngleLimitArcPoints(center, joint.LowerLimit, joint.UpperLimit, r);
            DrawPolyline(arc);

            // a drag marker at each limit
            DragAngleMarker(joint, center, r, joint.LowerLimit, lower: true);
            DragAngleMarker(joint, center, r, joint.UpperLimit, lower: false);
        }

        void DragAngleMarker(PhysicsJoint2DAuthoring joint, float2 center, float radius, float limitDeg, bool lower)
        {
            var marker = PhysicsJoint2DGizmos.AngleLimitMarker(center, limitDeg, radius);
            Handles.DrawLine(ToV3(center), ToV3(marker));
            EditorGUI.BeginChangeCheck();
            var moved = Handles.FreeMoveHandle(ToV3(marker), HandleSize(marker), Vector3.zero, Handles.DotHandleCap);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(joint, k_UndoHandle);
                var deg = PhysicsJoint2DGizmos.AngleFromLimitDrag(center, ToF2(moved));
                if (lower)
                    joint.LowerLimit = deg;
                else
                    joint.UpperLimit = deg;
            }
        }

        void TranslationLimitHandles(PhysicsJoint2DAuthoring joint)
        {
            if (!joint.UseLimits)
                return;
            var anchor = joint.Anchor;
            PhysicsJoint2DGizmos.TranslationLimitSegment(
                anchor,
                joint.AxisAngle,
                joint.LowerLimit,
                joint.UpperLimit,
                out var a,
                out var b
            );
            Handles.DrawLine(ToV3(a), ToV3(b));

            DragTranslationMarker(joint, a, lower: true);
            DragTranslationMarker(joint, b, lower: false);
        }

        void DragTranslationMarker(PhysicsJoint2DAuthoring joint, float2 markerPos, bool lower)
        {
            EditorGUI.BeginChangeCheck();
            var moved = Handles.FreeMoveHandle(
                ToV3(markerPos),
                HandleSize(markerPos),
                Vector3.zero,
                Handles.DotHandleCap
            );
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(joint, k_UndoHandle);
                var t = PhysicsJoint2DGizmos.TranslationFromLimitDrag(joint.Anchor, joint.AxisAngle, ToF2(moved));
                if (lower)
                    joint.LowerLimit = t;
                else
                    joint.UpperLimit = t;
            }
        }

        static float HandleSize(float2 localPoint) => HandleUtility.GetHandleSize(ToV3(localPoint)) * 0.08f;

        static void DrawPolyline(float2[] pts)
        {
            if (pts == null || pts.Length < 2)
                return;
            var line = new Vector3[pts.Length];
            for (var i = 0; i < pts.Length; i++)
                line[i] = ToV3(pts[i]);
            Handles.DrawAAPolyLine(line);
        }

        static Vector3 ToV3(float2 p) => new Vector3(p.x, p.y, 0f);

        static float2 ToF2(Vector3 p) => new float2(p.x, p.y);
    }
}
