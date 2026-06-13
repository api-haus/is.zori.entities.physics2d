using System.Collections.Generic;
using Unity.Mathematics;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;
using Zori.Entities.Physics2D;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Editor
{
    /// <summary>
    /// The custom inspector for <see cref="PhysicsShape2DAuthoring"/> — the 2D-native analogue of the DOTS
    /// sample's <c>PhysicsShapeAuthoringEditor</c> (structure-only reference). It mirrors the 3D layout
    /// (shape-type selector → per-type controls → material section → filter section → status → Fit dropdown →
    /// scene handles) and rewrites the scene machinery for 2D: flat <c>UnityEditor.Handles</c> primitives in the
    /// XY plane, no perspective corner-horizon math, no async previews. The handle / outline geometry is the pure
    /// <see cref="PhysicsShape2DGizmos"/> math; this editor is its thin <c>Handles</c> shell.
    /// </summary>
    [CustomEditor(typeof(PhysicsShape2DAuthoring))]
    [CanEditMultipleObjects]
    public sealed class PhysicsShape2DAuthoringEditor : UnityEditor.Editor
    {
        SerializedProperty m_Kind;
        SerializedProperty m_Offset;
        SerializedProperty m_Radius;
        SerializedProperty m_BoxCornerRadius;
        SerializedProperty m_BoxSize;
        SerializedProperty m_BoxAngle;
        SerializedProperty m_CapsuleSize;
        SerializedProperty m_CapsuleVertical;
        SerializedProperty m_CapsuleAngle;
        SerializedProperty m_EdgeIsLoop;
        SerializedProperty m_Vertices;
        SerializedProperty m_PolygonDecompose;
        SerializedProperty m_MaterialTemplate;
        SerializedProperty m_OverrideFriction;
        SerializedProperty m_Friction;
        SerializedProperty m_OverrideBounciness;
        SerializedProperty m_Bounciness;
        SerializedProperty m_Density;
        SerializedProperty m_OverrideFrictionCombine;
        SerializedProperty m_FrictionCombine;
        SerializedProperty m_OverrideBouncinessCombine;
        SerializedProperty m_BouncinessCombine;
        SerializedProperty m_Layer;
        SerializedProperty m_OverrideFilterBits;
        SerializedProperty m_CategoryBits;
        SerializedProperty m_ContactBits;
        SerializedProperty m_CollisionResponse;

        [SerializeField]
        bool m_ShowMaterial = true;

        [SerializeField]
        bool m_ShowFilter = true;

        static readonly GUIContent k_CornerRadiusLabel = new GUIContent(
            "Corner Radius",
            "Corner-rounding radius (BoxCollider2D.edgeRadius / polygon rounding)."
        );
        static readonly GUIContent k_FrictionLabel = new GUIContent("Friction");
        static readonly GUIContent k_BouncinessLabel = new GUIContent("Bounciness");
        static readonly GUIContent k_FrictionCombineLabel = new GUIContent("Friction Combine");
        static readonly GUIContent k_BouncinessCombineLabel = new GUIContent("Bounciness Combine");
        const string k_UndoFit = "Fit Physics Shape 2D";
        const string k_UndoHandle = "Edit Physics Shape 2D";

        void OnEnable()
        {
            m_Kind = serializedObject.FindProperty("m_Kind");
            m_Offset = serializedObject.FindProperty("m_Offset");
            m_Radius = serializedObject.FindProperty("m_Radius");
            m_BoxCornerRadius = serializedObject.FindProperty("m_BoxCornerRadius");
            m_BoxSize = serializedObject.FindProperty("m_BoxSize");
            m_BoxAngle = serializedObject.FindProperty("m_BoxAngle");
            m_CapsuleSize = serializedObject.FindProperty("m_CapsuleSize");
            m_CapsuleVertical = serializedObject.FindProperty("m_CapsuleVertical");
            m_CapsuleAngle = serializedObject.FindProperty("m_CapsuleAngle");
            m_EdgeIsLoop = serializedObject.FindProperty("m_EdgeIsLoop");
            m_Vertices = serializedObject.FindProperty("m_Vertices");
            m_PolygonDecompose = serializedObject.FindProperty("m_PolygonDecompose");
            m_MaterialTemplate = serializedObject.FindProperty("m_MaterialTemplate");
            m_OverrideFriction = serializedObject.FindProperty("m_OverrideFriction");
            m_Friction = serializedObject.FindProperty("m_Friction");
            m_OverrideBounciness = serializedObject.FindProperty("m_OverrideBounciness");
            m_Bounciness = serializedObject.FindProperty("m_Bounciness");
            m_Density = serializedObject.FindProperty("m_Density");
            m_OverrideFrictionCombine = serializedObject.FindProperty("m_OverrideFrictionCombine");
            m_FrictionCombine = serializedObject.FindProperty("m_FrictionCombine");
            m_OverrideBouncinessCombine = serializedObject.FindProperty("m_OverrideBouncinessCombine");
            m_BouncinessCombine = serializedObject.FindProperty("m_BouncinessCombine");
            m_Layer = serializedObject.FindProperty("m_Layer");
            m_OverrideFilterBits = serializedObject.FindProperty("m_OverrideFilterBits");
            m_CategoryBits = serializedObject.FindProperty("m_CategoryBits");
            m_ContactBits = serializedObject.FindProperty("m_ContactBits");
            m_CollisionResponse = serializedObject.FindProperty("m_CollisionResponse");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUI.BeginChangeCheck();

            EditorGUILayout.PropertyField(m_Kind);
            var kind = (PhysicsShape2DKind)m_Kind.enumValueIndex;

            DrawGeometry(kind);
            EditorGUILayout.Space();
            DrawMaterialSection();
            EditorGUILayout.Space();
            DrawFilterSection();

            if (EditorGUI.EndChangeCheck())
                serializedObject.ApplyModifiedProperties();

            DrawStatus(kind);
            EditorGUILayout.Space();
            DrawFitDropdown(kind);
        }

        void DrawGeometry(PhysicsShape2DKind kind)
        {
            switch (kind)
            {
                case PhysicsShape2DKind.Circle:
                    EditorGUILayout.PropertyField(m_Radius);
                    EditorGUILayout.PropertyField(m_Offset);
                    break;
                case PhysicsShape2DKind.Box:
                    EditorGUILayout.PropertyField(m_BoxSize);
                    EditorGUILayout.PropertyField(m_BoxAngle);
                    EditorGUILayout.PropertyField(m_BoxCornerRadius, k_CornerRadiusLabel);
                    EditorGUILayout.PropertyField(m_Offset);
                    break;
                case PhysicsShape2DKind.Capsule:
                    EditorGUILayout.PropertyField(m_CapsuleSize);
                    EditorGUILayout.PropertyField(m_CapsuleVertical);
                    EditorGUILayout.PropertyField(m_CapsuleAngle);
                    EditorGUILayout.PropertyField(m_Offset);
                    break;
                case PhysicsShape2DKind.Polygon:
                    EditorGUILayout.PropertyField(m_Vertices, true);
                    EditorGUILayout.PropertyField(m_PolygonDecompose);
                    EditorGUILayout.PropertyField(m_BoxCornerRadius, k_CornerRadiusLabel);
                    EditorGUILayout.PropertyField(m_Offset);
                    break;
                case PhysicsShape2DKind.Edge:
                    EditorGUILayout.PropertyField(m_Vertices, true);
                    EditorGUILayout.PropertyField(m_EdgeIsLoop);
                    EditorGUILayout.PropertyField(m_Offset);
                    break;
            }
        }

        void DrawMaterialSection()
        {
            m_ShowMaterial = EditorGUILayout.Foldout(m_ShowMaterial, "Material", true);
            if (!m_ShowMaterial)
                return;

            using (new EditorGUI.IndentLevelScope())
            {
                EditorGUILayout.PropertyField(m_MaterialTemplate);
                var template = m_MaterialTemplate.objectReferenceValue as PhysicsMaterial2D;
                var hasTemplate = template != null;

                DrawOverrideFloatRow(
                    k_FrictionLabel,
                    m_OverrideFriction,
                    m_Friction,
                    hasTemplate ? template.friction : 0f,
                    hasTemplate
                );
                DrawOverrideFloatRow(
                    k_BouncinessLabel,
                    m_OverrideBounciness,
                    m_Bounciness,
                    hasTemplate ? template.bounciness : 0f,
                    hasTemplate
                );
                DrawOverrideEnumRow(
                    k_FrictionCombineLabel,
                    m_OverrideFrictionCombine,
                    m_FrictionCombine,
                    hasTemplate ? (int)MapCombine(template.frictionCombine) : 0,
                    hasTemplate
                );
                DrawOverrideEnumRow(
                    k_BouncinessCombineLabel,
                    m_OverrideBouncinessCombine,
                    m_BouncinessCombine,
                    hasTemplate ? (int)MapCombine(template.bounceCombine) : 0,
                    hasTemplate
                );

                EditorGUILayout.PropertyField(m_Density);
                EditorGUILayout.PropertyField(m_CollisionResponse);
            }
        }

        void DrawFilterSection()
        {
            m_ShowFilter = EditorGUILayout.Foldout(m_ShowFilter, "Filter", true);
            if (!m_ShowFilter)
                return;

            using (new EditorGUI.IndentLevelScope())
            {
                // The layer popup — the 2D-native "named categories" reuse the project's Unity layer names.
                DrawLayerPopup();

                EditorGUILayout.PropertyField(m_OverrideFilterBits);
                if (m_OverrideFilterBits.boolValue || m_OverrideFilterBits.hasMultipleDifferentValues)
                {
                    using (new EditorGUI.IndentLevelScope())
                    {
                        DrawFilterBitsMask("Category Bits", m_CategoryBits);
                        DrawFilterBitsMask("Contact Bits", m_ContactBits);
                    }
                }
            }
        }

        void DrawLayerPopup()
        {
            // m_Layer is -1 (unfiltered) or [0..31]. Render as a single-select popup over the named layers plus
            // an explicit "Default (Unfiltered)" entry for -1, so a user picks a named layer or the everything-default.
            var layerNames = InternalEditorUtility.layers; // only the layers that have names
            var options = new string[layerNames.Length + 1];
            options[0] = "Default (Unfiltered)";
            for (var i = 0; i < layerNames.Length; i++)
                options[i + 1] = layerNames[i];

            var current = m_Layer.intValue;
            var selectedIndex = 0;
            if (current >= 0)
            {
                var name = LayerMask.LayerToName(current);
                for (var i = 0; i < layerNames.Length; i++)
                {
                    if (layerNames[i] == name)
                    {
                        selectedIndex = i + 1;
                        break;
                    }
                }
            }

            EditorGUI.showMixedValue = m_Layer.hasMultipleDifferentValues;
            EditorGUI.BeginChangeCheck();
            var newIndex = EditorGUILayout.Popup(
                new GUIContent("Layer", "Resolves the contact filter from the project 2D layer matrix."),
                selectedIndex,
                options
            );
            if (EditorGUI.EndChangeCheck())
            {
                m_Layer.intValue = newIndex == 0 ? -1 : LayerMask.NameToLayer(layerNames[newIndex - 1]);
            }
            EditorGUI.showMixedValue = false;
        }

        void DrawFilterBitsMask(string label, SerializedProperty prop)
        {
            // Render the raw int mask with the Unity layer names as bit labels (the 2D-native category naming).
            var layerNames = InternalEditorUtility.layers;
            EditorGUI.showMixedValue = prop.hasMultipleDifferentValues;
            EditorGUI.BeginChangeCheck();
            var masked = InternalEditorUtility.LayerMaskToConcatenatedLayersMask(prop.intValue);
            var newMasked = EditorGUILayout.MaskField(new GUIContent(label), masked, layerNames);
            if (EditorGUI.EndChangeCheck())
                prop.intValue = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(newMasked);
            EditorGUI.showMixedValue = false;
        }

        // ----- the material override-row helper (toggle + value; inherited template value shown when off) -----

        void DrawOverrideFloatRow(
            GUIContent label,
            SerializedProperty overrideProp,
            SerializedProperty valueProp,
            float inheritedValue,
            bool hasTemplate
        )
        {
            var rect = EditorGUILayout.GetControlRect();
            var toggleRect = new Rect(rect.x, rect.y, rect.width, rect.height);
            var inheriting = hasTemplate && !overrideProp.boolValue;

            EditorGUI.BeginProperty(toggleRect, label, overrideProp);
            EditorGUI.showMixedValue = overrideProp.hasMultipleDifferentValues;
            EditorGUI.BeginChangeCheck();
            var newOverride = EditorGUI.ToggleLeft(
                new Rect(rect.x, rect.y, EditorGUIUtility.labelWidth, rect.height),
                label,
                overrideProp.boolValue
            );
            if (EditorGUI.EndChangeCheck())
                overrideProp.boolValue = newOverride;
            EditorGUI.showMixedValue = false;
            EditorGUI.EndProperty();

            var valueRect = new Rect(
                rect.x + EditorGUIUtility.labelWidth,
                rect.y,
                rect.width - EditorGUIUtility.labelWidth,
                rect.height
            );
            using (new EditorGUI.DisabledScope(inheriting))
            {
                if (inheriting)
                {
                    // show the inherited template value (read-only) so the user sees what is being used
                    EditorGUI.FloatField(valueRect, inheritedValue);
                }
                else
                {
                    EditorGUI.showMixedValue = valueProp.hasMultipleDifferentValues;
                    EditorGUI.BeginChangeCheck();
                    var v = EditorGUI.FloatField(valueRect, valueProp.floatValue);
                    if (EditorGUI.EndChangeCheck())
                        valueProp.floatValue = v;
                    EditorGUI.showMixedValue = false;
                }
            }
        }

        void DrawOverrideEnumRow(
            GUIContent label,
            SerializedProperty overrideProp,
            SerializedProperty valueProp,
            int inheritedEnumIndex,
            bool hasTemplate
        )
        {
            var rect = EditorGUILayout.GetControlRect();
            var inheriting = hasTemplate && !overrideProp.boolValue;

            EditorGUI.BeginProperty(rect, label, overrideProp);
            EditorGUI.showMixedValue = overrideProp.hasMultipleDifferentValues;
            EditorGUI.BeginChangeCheck();
            var newOverride = EditorGUI.ToggleLeft(
                new Rect(rect.x, rect.y, EditorGUIUtility.labelWidth, rect.height),
                label,
                overrideProp.boolValue
            );
            if (EditorGUI.EndChangeCheck())
                overrideProp.boolValue = newOverride;
            EditorGUI.showMixedValue = false;
            EditorGUI.EndProperty();

            var valueRect = new Rect(
                rect.x + EditorGUIUtility.labelWidth,
                rect.y,
                rect.width - EditorGUIUtility.labelWidth,
                rect.height
            );
            using (new EditorGUI.DisabledScope(inheriting))
            {
                var displayIndex = inheriting ? inheritedEnumIndex : valueProp.enumValueIndex;
                EditorGUI.showMixedValue = !inheriting && valueProp.hasMultipleDifferentValues;
                EditorGUI.BeginChangeCheck();
                var newIndex = EditorGUI.Popup(valueRect, displayIndex, valueProp.enumDisplayNames);
                if (EditorGUI.EndChangeCheck() && !inheriting)
                    valueProp.enumValueIndex = newIndex;
                EditorGUI.showMixedValue = false;
            }
        }

        void DrawStatus(PhysicsShape2DKind kind)
        {
            if (kind == PhysicsShape2DKind.Polygon)
            {
                var n = m_Vertices.arraySize;
                if (!m_PolygonDecompose.boolValue && (n < 3 || n > PhysicsShape2DAutoFit.MaxPolygonVertices))
                {
                    EditorGUILayout.HelpBox(
                        $"A single-hull polygon needs 3–{PhysicsShape2DAutoFit.MaxPolygonVertices} convex "
                            + "vertices. Enable Polygon Decompose for a concave or larger outline.",
                        MessageType.Warning
                    );
                }
            }
            else if (kind == PhysicsShape2DKind.Edge && m_Vertices.arraySize < 2)
            {
                EditorGUILayout.HelpBox("An edge needs at least 2 vertices.", MessageType.Warning);
            }

            if (
                m_OverrideFilterBits.boolValue
                && !m_OverrideFilterBits.hasMultipleDifferentValues
                && !m_CategoryBits.hasMultipleDifferentValues
                && m_CategoryBits.intValue == 0
            )
            {
                EditorGUILayout.HelpBox(
                    "Category Bits 0 puts this shape in no category — contact filters will not detect it.",
                    MessageType.Info
                );
            }
        }

        // ----- the "Fit to…" dropdown (wires Phase C's PhysicsShape2DAutoFit) -----

        void DrawFitDropdown(PhysicsShape2DKind kind)
        {
            // Auto-fit targets only the four enclosing kinds; Edge is an open chain (not a fit target).
            var fittable =
                kind == PhysicsShape2DKind.Box
                || kind == PhysicsShape2DKind.Circle
                || kind == PhysicsShape2DKind.Capsule
                || kind == PhysicsShape2DKind.Polygon;

            using (new EditorGUI.DisabledScope(targets.Length != 1))
            {
                var content = new GUIContent(
                    "Fit To…",
                    "Fit this shape to a Sprite physics shape, a SpriteRenderer's bounds, or a PolygonCollider2D path."
                );
                var rect = EditorGUILayout.GetControlRect();
                if (!EditorGUI.DropdownButton(rect, content, FocusType.Keyboard))
                    return;
                if (targets.Length != 1)
                    return;

                var shape = (PhysicsShape2DAuthoring)target;
                var menu = new GenericMenu();
                var sprite = shape.GetComponent<SpriteRenderer>();
                var poly = shape.GetComponent<PolygonCollider2D>();
                var fitKind = fittable ? kind : PhysicsShape2DKind.Box;

                if (sprite != null && sprite.sprite != null)
                {
                    AddFitItem(menu, "Sprite Physics Shape", shape, fitKind, FitSource.SpriteShape, sprite, poly);
                    AddFitItem(menu, "Sprite Renderer Bounds", shape, fitKind, FitSource.SpriteBounds, sprite, poly);
                }
                if (poly != null)
                {
                    AddFitItem(menu, "Polygon Collider 2D", shape, fitKind, FitSource.PolygonCollider, sprite, poly);
                }

                if (menu.GetItemCount() == 0)
                {
                    menu.AddDisabledItem(new GUIContent("No fit source (add a SpriteRenderer or PolygonCollider2D)"));
                }
                menu.DropDown(rect);
            }
        }

        enum FitSource
        {
            SpriteShape,
            SpriteBounds,
            PolygonCollider,
        }

        void AddFitItem(
            GenericMenu menu,
            string label,
            PhysicsShape2DAuthoring shape,
            PhysicsShape2DKind kind,
            FitSource source,
            SpriteRenderer sprite,
            PolygonCollider2D poly
        )
        {
            menu.AddItem(
                new GUIContent($"{label}/{kind}"),
                false,
                () =>
                {
                    Undo.RecordObject(shape, k_UndoFit);
                    var changed = source switch
                    {
                        FitSource.SpriteShape => PhysicsShape2DAutoFit.FitToSprite(shape, sprite.sprite, kind),
                        FitSource.SpriteBounds => PhysicsShape2DAutoFit.FitToSpriteRenderer(shape, sprite, kind),
                        FitSource.PolygonCollider => PhysicsShape2DAutoFit.FitToPolygonCollider2D(shape, poly, kind),
                        _ => false,
                    };
                    if (changed)
                    {
                        EditorUtility.SetDirty(shape);
                        SceneView.RepaintAll();
                    }
                }
            );
        }

        // ----- the scene-view 2D handles -----

        void OnSceneGUI()
        {
            var shape = (PhysicsShape2DAuthoring)target;
            var prevMatrix = Handles.matrix;
            var prevColor = Handles.color;
            Handles.matrix = shape.transform.localToWorldMatrix;
            Handles.color = new Color(0.57f, 0.96f, 0.55f, 1f);

            switch (shape.Kind)
            {
                case PhysicsShape2DKind.Circle:
                    CircleHandles(shape);
                    break;
                case PhysicsShape2DKind.Box:
                    BoxHandles(shape);
                    break;
                case PhysicsShape2DKind.Capsule:
                    CapsuleHandles(shape);
                    break;
                case PhysicsShape2DKind.Polygon:
                case PhysicsShape2DKind.Edge:
                    VertexHandles(shape);
                    break;
            }

            OffsetHandle(shape);

            Handles.matrix = prevMatrix;
            Handles.color = prevColor;
        }

        static float HandleSize(float2 worldLocalPoint) => HandleUtility.GetHandleSize(ToV3(worldLocalPoint)) * 0.08f;

        void OffsetHandle(PhysicsShape2DAuthoring shape)
        {
            var pos = shape.Offset;
            EditorGUI.BeginChangeCheck();
            var newPos = Handles.Slider2D(
                ToV3(pos),
                Vector3.forward,
                Vector3.right,
                Vector3.up,
                HandleSize(pos),
                Handles.RectangleHandleCap,
                Vector2.zero,
                false
            );
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(shape, k_UndoHandle);
                shape.Offset = ToF2(newPos);
            }
        }

        void CircleHandles(PhysicsShape2DAuthoring shape)
        {
            Handles.DrawWireDisc(ToV3(shape.Offset), Vector3.forward, shape.Radius);
            var handlePos = shape.Offset + new float2(shape.Radius, 0f);
            EditorGUI.BeginChangeCheck();
            var newPos = Handles.FreeMoveHandle(
                ToV3(handlePos),
                HandleSize(handlePos),
                Vector3.zero,
                Handles.CircleHandleCap
            );
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(shape, k_UndoHandle);
                shape.Radius = PhysicsShape2DGizmos.CircleRadiusFromDrag(shape.Offset, ToF2(newPos));
            }
        }

        void BoxHandles(PhysicsShape2DAuthoring shape)
        {
            // outline
            DrawClosedHandles(PhysicsShape2DGizmos.BoxOutline(shape.Offset, shape.BoxSize, shape.BoxAngle), true);

            // two positive half-extent face handles (+X at index 0, +Y at index 1)
            var faces = PhysicsShape2DGizmos.BoxEdgeHandlePositions(shape.Offset, shape.BoxSize, shape.BoxAngle);
            for (var edge = 0; edge < 2; edge++)
            {
                var p = faces[edge];
                EditorGUI.BeginChangeCheck();
                var moved = Handles.FreeMoveHandle(ToV3(p), HandleSize(p), Vector3.zero, Handles.DotHandleCap);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(shape, k_UndoHandle);
                    shape.BoxSize = PhysicsShape2DGizmos.BoxSizeFromEdgeDrag(
                        shape.BoxSize,
                        edge,
                        ToF2(moved),
                        shape.BoxAngle,
                        shape.Offset
                    );
                }
            }

            // rotation ring handle
            var ringR = 0.5f * math.length(shape.BoxSize) + 0.25f;
            var ringPos = PhysicsShape2DGizmos.BoxRotationHandlePosition(shape.Offset, shape.BoxAngle, ringR);
            EditorGUI.BeginChangeCheck();
            var newRing = Handles.FreeMoveHandle(
                ToV3(ringPos),
                HandleSize(ringPos),
                Vector3.zero,
                Handles.CircleHandleCap
            );
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(shape, k_UndoHandle);
                shape.BoxAngle = PhysicsShape2DGizmos.AngleFromRotationDrag(shape.Offset, ToF2(newRing));
            }
        }

        void CapsuleHandles(PhysicsShape2DAuthoring shape)
        {
            shape.GetCapsuleCenters(out var radius, out var c1, out var c2);
            var w1 = c1 + shape.Offset;
            var w2 = c2 + shape.Offset;
            DrawClosedHandles(PhysicsShape2DGizmos.CapsuleOutline(w1, w2, radius), true);

            // two end-cap drag handles
            EditorGUI.BeginChangeCheck();
            var n1 = Handles.FreeMoveHandle(ToV3(w1), HandleSize(w1), Vector3.zero, Handles.CircleHandleCap);
            var n2 = Handles.FreeMoveHandle(ToV3(w2), HandleSize(w2), Vector3.zero, Handles.CircleHandleCap);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(shape, k_UndoHandle);
                // map the dragged caps (relative to offset) back to size + vertical + angle + centre
                PhysicsShape2DGizmos.CapsuleFieldsFromHandles(
                    ToF2(n1) - shape.Offset,
                    ToF2(n2) - shape.Offset,
                    radius,
                    out var size,
                    out var vertical,
                    out var angle,
                    out var center
                );
                shape.CapsuleSize = size;
                shape.CapsuleVertical = vertical;
                shape.CapsuleAngle = angle;
                shape.Offset = shape.Offset + center;
            }

            // radius handle: perpendicular to the spine at cap 2
            var spine = math.normalizesafe(w2 - w1, new float2(0f, 1f));
            var perp = new float2(-spine.y, spine.x);
            var radiusPos = w2 + perp * radius;
            EditorGUI.BeginChangeCheck();
            var newR = Handles.FreeMoveHandle(
                ToV3(radiusPos),
                HandleSize(radiusPos),
                Vector3.zero,
                Handles.DotHandleCap
            );
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(shape, k_UndoHandle);
                var newRadius = math.max(1e-3f, math.abs(math.dot(ToF2(newR) - w2, perp)) + radius);
                // changing the end radius changes the minor capsule size component; keep the spine fixed.
                PhysicsShape2DGizmos.CapsuleFieldsFromHandles(
                    c1,
                    c2,
                    newRadius,
                    out var size,
                    out var vertical,
                    out var angle,
                    out _
                );
                shape.CapsuleSize = size;
                shape.CapsuleVertical = vertical;
                shape.CapsuleAngle = angle;
            }
        }

        void VertexHandles(PhysicsShape2DAuthoring shape)
        {
            var verts = shape.Vertices;
            if (verts.Length == 0)
                return;

            var loop = shape.Kind == PhysicsShape2DKind.Polygon || shape.EdgeIsLoop;
            var outline = new float2[verts.Length];
            for (var i = 0; i < verts.Length; i++)
                outline[i] = (float2)verts[i] + shape.Offset;
            DrawClosedHandles(outline, loop);

            var changed = false;
            for (var i = 0; i < verts.Length; i++)
            {
                var p = (float2)verts[i] + shape.Offset;
                EditorGUI.BeginChangeCheck();
                var moved = Handles.FreeMoveHandle(ToV3(p), HandleSize(p), Vector3.zero, Handles.DotHandleCap);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(shape, k_UndoHandle);
                    verts[i] = (Vector2)(ToF2(moved) - shape.Offset);
                    changed = true;
                }
            }
            if (changed)
            {
                shape.Vertices = verts;
                EditorUtility.SetDirty(shape);
            }
        }

        void DrawClosedHandles(float2[] pts, bool close)
        {
            if (pts == null || pts.Length < 2)
                return;
            var line = new Vector3[close ? pts.Length + 1 : pts.Length];
            for (var i = 0; i < pts.Length; i++)
                line[i] = ToV3(pts[i]);
            if (close)
                line[pts.Length] = ToV3(pts[0]);
            Handles.DrawAAPolyLine(line);
        }

        // ----- shared mapping of the engine combine enum to the package mixing enum (read-only preview) -----

        static PhysicsSurfaceMixing2D MapCombine(PhysicsMaterialCombine2D combine) =>
            combine switch
            {
                PhysicsMaterialCombine2D.Average => PhysicsSurfaceMixing2D.Average,
                PhysicsMaterialCombine2D.Maximum => PhysicsSurfaceMixing2D.Maximum,
                PhysicsMaterialCombine2D.Mean => PhysicsSurfaceMixing2D.Mean,
                PhysicsMaterialCombine2D.Minimum => PhysicsSurfaceMixing2D.Minimum,
                PhysicsMaterialCombine2D.Multiply => PhysicsSurfaceMixing2D.Multiply,
                _ => PhysicsSurfaceMixing2D.Average,
            };

        static Vector3 ToV3(float2 p) => new Vector3(p.x, p.y, 0f);

        static float2 ToF2(Vector3 p) => new float2(p.x, p.y);
    }
}
