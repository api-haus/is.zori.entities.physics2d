using UnityEditor;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Editor
{
    /// <summary>
    /// The custom inspector for <see cref="PhysicsBody2DAuthoring"/> — the 2D-native analogue of the DOTS
    /// sample's <c>PhysicsBodyAuthoringEditor</c> (structure-only reference). It mirrors the 3D layout: a body-type
    /// selector, fields shown conditionally on the motion type, an Advanced foldout for the less-common knobs
    /// (CCD, the mass-distribution override), and a status HelpBox. The 3D-only knobs (solver type, world index, an
    /// inertia-tensor orientation) are 2D negative space and absent.
    /// </summary>
    [CustomEditor(typeof(PhysicsBody2DAuthoring))]
    [CanEditMultipleObjects]
    public sealed class PhysicsBody2DAuthoringEditor : UnityEditor.Editor
    {
        SerializedProperty m_BodyType;
        SerializedProperty m_GravityScale;
        SerializedProperty m_LinearDamping;
        SerializedProperty m_AngularDamping;
        SerializedProperty m_UseAutoMass;
        SerializedProperty m_Mass;
        SerializedProperty m_FreezePositionX;
        SerializedProperty m_FreezePositionY;
        SerializedProperty m_FreezeRotation;
        SerializedProperty m_InitialLinearVelocity;
        SerializedProperty m_InitialAngularVelocity;
        SerializedProperty m_Interpolation;
        SerializedProperty m_CollisionDetection;
        SerializedProperty m_OverrideMassDistribution;
        SerializedProperty m_CenterOfMass;
        SerializedProperty m_RotationalInertia;

        static readonly GUIContent k_MassLabel = new GUIContent("Mass");
        static readonly GUIContent k_ConstraintsLabel = new GUIContent("Constraints");
        static readonly GUIContent k_AdvancedLabel = new GUIContent("Advanced", "Less-common body knobs.");

        [SerializeField]
        bool m_ShowAdvanced;

        void OnEnable()
        {
            m_BodyType = serializedObject.FindProperty("m_BodyType");
            m_GravityScale = serializedObject.FindProperty("m_GravityScale");
            m_LinearDamping = serializedObject.FindProperty("m_LinearDamping");
            m_AngularDamping = serializedObject.FindProperty("m_AngularDamping");
            m_UseAutoMass = serializedObject.FindProperty("m_UseAutoMass");
            m_Mass = serializedObject.FindProperty("m_Mass");
            m_FreezePositionX = serializedObject.FindProperty("m_FreezePositionX");
            m_FreezePositionY = serializedObject.FindProperty("m_FreezePositionY");
            m_FreezeRotation = serializedObject.FindProperty("m_FreezeRotation");
            m_InitialLinearVelocity = serializedObject.FindProperty("m_InitialLinearVelocity");
            m_InitialAngularVelocity = serializedObject.FindProperty("m_InitialAngularVelocity");
            m_Interpolation = serializedObject.FindProperty("m_Interpolation");
            m_CollisionDetection = serializedObject.FindProperty("m_CollisionDetection");
            m_OverrideMassDistribution = serializedObject.FindProperty("m_OverrideMassDistribution");
            m_CenterOfMass = serializedObject.FindProperty("m_CenterOfMass");
            m_RotationalInertia = serializedObject.FindProperty("m_RotationalInertia");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUI.BeginChangeCheck();

            EditorGUILayout.PropertyField(m_BodyType);

            var bodyType = (PhysicsBody2DMotionType)m_BodyType.enumValueIndex;
            var dynamic = bodyType == PhysicsBody2DMotionType.Dynamic;
            var staticBody = bodyType == PhysicsBody2DMotionType.Static;

            if (!staticBody)
                EditorGUILayout.PropertyField(m_Interpolation);

            if (dynamic)
            {
                EditorGUILayout.PropertyField(m_UseAutoMass);
                using (new EditorGUI.DisabledScope(m_UseAutoMass.boolValue))
                    EditorGUILayout.PropertyField(m_Mass, k_MassLabel);
                EditorGUILayout.PropertyField(m_LinearDamping);
                EditorGUILayout.PropertyField(m_AngularDamping);
                EditorGUILayout.PropertyField(m_GravityScale);
            }
            else
            {
                // A non-Dynamic body has infinite mass — show a disabled ∞ field (the 3D inspector's touch).
                using (new EditorGUI.DisabledScope(true))
                {
                    var rect = EditorGUILayout.GetControlRect(
                        true,
                        EditorGUIUtility.singleLineHeight
                    );
                    EditorGUI.BeginProperty(rect, k_MassLabel, m_Mass);
                    EditorGUI.FloatField(rect, k_MassLabel, float.PositiveInfinity);
                    EditorGUI.EndProperty();
                }
            }

            if (!staticBody)
            {
                EditorGUILayout.PropertyField(m_InitialLinearVelocity);
                EditorGUILayout.PropertyField(m_InitialAngularVelocity);
            }

            // Constraints (the three freeze toggles) apply to any non-static body.
            EditorGUILayout.LabelField(k_ConstraintsLabel, EditorStyles.boldLabel);
            using (new EditorGUI.IndentLevelScope())
            {
                EditorGUILayout.PropertyField(m_FreezePositionX);
                EditorGUILayout.PropertyField(m_FreezePositionY);
                EditorGUILayout.PropertyField(m_FreezeRotation);
            }

            m_ShowAdvanced = EditorGUILayout.Foldout(m_ShowAdvanced, k_AdvancedLabel, true);
            if (m_ShowAdvanced)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(m_CollisionDetection);
                    if (!staticBody)
                    {
                        EditorGUILayout.PropertyField(m_OverrideMassDistribution);
                        if (m_OverrideMassDistribution.boolValue)
                        {
                            using (new EditorGUI.IndentLevelScope())
                            {
                                EditorGUILayout.PropertyField(m_CenterOfMass);
                                using (new EditorGUI.DisabledScope(!dynamic))
                                    EditorGUILayout.PropertyField(m_RotationalInertia);
                            }
                        }
                    }
                }
            }

            if (EditorGUI.EndChangeCheck())
                serializedObject.ApplyModifiedProperties();

            DrawStatus(dynamic);
        }

        void DrawStatus(bool dynamic)
        {
            // A dynamic body with no shape anywhere in its hierarchy has no collider and falls through everything.
            if (dynamic && targets.Length == 1)
            {
                var body = (PhysicsBody2DAuthoring)target;
                if (body.GetComponentInChildren<PhysicsShape2DAuthoring>() == null)
                {
                    EditorGUILayout.HelpBox(
                        "This dynamic body has no Physics Shape 2D on itself or its children — it will have a "
                            + "collider-less default mass and not collide. Add a Physics Shape 2D.",
                        MessageType.Warning
                    );
                }
            }

            if (
                m_OverrideMassDistribution.boolValue
                && !m_OverrideMassDistribution.hasMultipleDifferentValues
                && m_RotationalInertia.floatValue == 0f
            )
            {
                EditorGUILayout.HelpBox(
                    "Rotational Inertia 0 keeps the shape-derived inertia; only the Center of Mass is overridden.",
                    MessageType.Info
                );
            }
        }
    }
}
