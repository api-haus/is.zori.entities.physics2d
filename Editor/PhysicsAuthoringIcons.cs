using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Zori.Entities.Physics2D.Editor
{
    /// <summary>
    /// Assigns recognizable inspector / hierarchy / project icons to the custom-authoring MonoBehaviours by
    /// borrowing Unity's own built-in 2D-physics glyphs. Our components mirror the built-in
    /// <c>Rigidbody2D</c> / <c>Collider2D</c>, so reusing those icons is instantly readable and needs no bundled art.
    /// </summary>
    /// <remarks>
    /// The mechanism is <see cref="MonoImporter.SetIcon"/> + <see cref="AssetDatabase.ImportAsset"/>, which writes the
    /// icon reference into the script's <c>.cs.meta</c> <c>icon:</c> field as a built-in extra-resource reference.
    /// Each entry lists candidate built-in icon names in priority order — built-in icon names drift across editor
    /// versions, so the first one that <see cref="EditorGUIUtility.IconContent(string)"/> resolves to a non-null
    /// texture wins. Extending this to future effector / joint authoring is one row in <see cref="k_Assignments"/>.
    /// </remarks>
    public static class PhysicsAuthoringIcons
    {
        /// <summary>One authoring type and the built-in icon names to try for it, best first.</summary>
        readonly struct IconAssignment
        {
            public readonly Type AuthoringType;
            public readonly string[] IconNameCandidates;

            public IconAssignment(Type authoringType, params string[] iconNameCandidates)
            {
                AuthoringType = authoringType;
                IconNameCandidates = iconNameCandidates;
            }
        }

        static readonly IconAssignment[] k_Assignments =
        {
            // Body mirrors Rigidbody2D.
            new IconAssignment(
                typeof(Authoring.PhysicsBody2DAuthoring),
                "Rigidbody2D Icon",
                "d_Rigidbody2D Icon"
            ),
            // Shape mirrors a 2D collider — prefer the generic Collider2D glyph, fall back to a concrete one.
            new IconAssignment(
                typeof(Authoring.PhysicsShape2DAuthoring),
                "Collider2D Icon",
                "d_Collider2D Icon",
                "BoxCollider2D Icon",
                "d_BoxCollider2D Icon",
                "PolygonCollider2D Icon",
                "d_PolygonCollider2D Icon"
            ),
            // Step is a simulation-settings singleton — a settings / gear glyph.
            new IconAssignment(
                typeof(Authoring.PhysicsStep2DAuthoring),
                "SettingsIcon",
                "d_SettingsIcon",
                "Settings Icon",
                "d_Settings Icon"
            ),
        };

        [MenuItem("Tools/Zori/Entities Physics 2D/Assign Authoring Icons")]
        public static void AssignAuthoringIcons()
        {
            var assigned = 0;
            var failures = new List<string>();

            foreach (var assignment in k_Assignments)
            {
                var scriptPath = ResolveScriptPath(assignment.AuthoringType);
                if (scriptPath == null)
                {
                    failures.Add($"{assignment.AuthoringType.Name}: no MonoScript asset found");
                    continue;
                }

                var importer = AssetImporter.GetAtPath(scriptPath) as MonoImporter;
                if (importer == null)
                {
                    failures.Add($"{assignment.AuthoringType.Name}: not a MonoImporter at {scriptPath}");
                    continue;
                }

                var icon = ResolveIcon(assignment.IconNameCandidates, out var iconName);
                if (icon == null)
                {
                    failures.Add(
                        $"{assignment.AuthoringType.Name}: none of [{string.Join(", ", assignment.IconNameCandidates)}] resolved"
                    );
                    continue;
                }

                importer.SetIcon(icon);
                AssetDatabase.ImportAsset(scriptPath, ImportAssetOptions.ForceUpdate);
                Debug.Log(
                    $"[PhysicsAuthoringIcons] {assignment.AuthoringType.Name} <- '{iconName}' ({scriptPath})"
                );
                assigned++;
            }

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Debug.Log($"[PhysicsAuthoringIcons] assigned {assigned}/{k_Assignments.Length} icons.");
            if (failures.Count > 0)
                Debug.LogError($"[PhysicsAuthoringIcons] failures:\n  {string.Join("\n  ", failures)}");
        }

        /// <summary>Finds the script asset path for a MonoBehaviour type via its MonoScript.</summary>
        static string ResolveScriptPath(Type type)
        {
            var guids = AssetDatabase.FindAssets($"{type.Name} t:MonoScript");
            foreach (var guid in guids)
            {
                var path = AssetDatabase.GUIDToAssetPath(guid);
                var script = AssetDatabase.LoadAssetAtPath<MonoScript>(path);
                if (script != null && script.GetClass() == type)
                    return path;
            }

            return null;
        }

        /// <summary>Returns the first built-in icon texture that resolves to non-null, with its name.</summary>
        static Texture2D ResolveIcon(string[] candidates, out string resolvedName)
        {
            foreach (var name in candidates)
            {
                var tex = EditorGUIUtility.IconContent(name)?.image as Texture2D;
                if (tex != null)
                {
                    resolvedName = name;
                    return tex;
                }
            }

            resolvedName = null;
            return null;
        }
    }
}
