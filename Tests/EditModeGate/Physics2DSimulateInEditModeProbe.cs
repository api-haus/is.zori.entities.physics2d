using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// Load-bearing probe for the parity-gate conversion: does <c>UnityEngine.Physics2D.Simulate(dt)</c> with
    /// <c>simulationMode = SimulationMode2D.Script</c> actually step a built-in <c>PhysicsScene2D</c> populated
    /// by GameObjects opened in EDIT MODE via <see cref="EditorSceneManager.OpenScene"/> (Additive), under
    /// batchmode? The parity reference side of <c>PhysicsParityHarness</c> depends on this; if it does not step
    /// in EditMode the parity gates cannot convert without losing coverage.
    /// </summary>
    /// <remarks>
    /// Authors one Dynamic <c>Rigidbody2D</c> + <c>CircleCollider2D</c> at Y=3 into a temp folder (NO SubScene,
    /// NO build-settings), opens that child scene additively with the editor API, switches the global 2D physics
    /// to Script mode + gravity, syncs transforms, and steps 60 manual <c>Physics2D.Simulate(dt)</c> calls. A
    /// body that integrated under gravity has a strictly lower Y; one that never stepped sits at Y=3.
    /// </remarks>
    public sealed class Physics2DSimulateInEditModeProbe
    {
        const float FixedDt = 1f / 60f;
        const float StartY = 3f;

        string _folder;

        // Tolerate the unrelated [Error] a vendor MicroSplat asset postprocessor logs during AssetDatabase.Refresh
        // (a malformed demo-texture .meta in another package), which the Unity Test Framework would otherwise turn
        // into a failure — same guard the parity harness applies. No physics check is relaxed: the asserts throw
        // regardless of the log policy.
        [SetUp]
        public void SetUp() => LogAssert.ignoreFailingMessages = true;

        [TearDown]
        public void TearDown()
        {
            LogAssert.ignoreFailingMessages = false;

            if (!string.IsNullOrEmpty(_folder) && AssetDatabase.IsValidFolder(_folder))
            {
                AssetDatabase.DeleteAsset(_folder);
                AssetDatabase.Refresh();
            }
            _folder = null;
        }

        [Test]
        public void Physics2DSimulate_StepsAnEditModeOpenedScene_Deterministically()
        {
            _folder = "Assets/__p2d_simulate_probe_" + Guid.NewGuid().ToString("N") + "__";
            Directory.CreateDirectory(_folder);
            AssetDatabase.Refresh();

            var childPath = _folder + "/SimulateProbe.unity";

            // Author the child scene: one Dynamic body + circle collider at Y = StartY.
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var go = new GameObject("ProbeBody");
            go.transform.position = new Vector3(0f, StartY, 0f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
            var circle = go.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;

            Scene opened = default;
            var bodies = new List<Rigidbody2D>();
            try
            {
                UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
                UnityEngine.Physics2D.gravity = new Vector2(0f, -9.81f);

                opened = EditorSceneManager.OpenScene(childPath, OpenSceneMode.Additive);
                foreach (var root in opened.GetRootGameObjects())
                    bodies.AddRange(root.GetComponentsInChildren<Rigidbody2D>(true));
                Assert.AreEqual(
                    1,
                    bodies.Count,
                    "Probe authored exactly one Rigidbody2D — the edit-mode-opened scene must surface it."
                );

                UnityEngine.Physics2D.SyncTransforms();

                var firstY = bodies[0].position.y;
                var prevY = firstY;
                var sawStrictDecrease = false;
                for (var i = 0; i < 60; i++)
                {
                    UnityEngine.Physics2D.Simulate(FixedDt);
                    var y = bodies[0].position.y;
                    if (y < prevY - 1e-6f)
                        sawStrictDecrease = true;
                    prevY = y;
                }
                var finalY = bodies[0].position.y;

                Debug.Log(
                    $"[PHYSICS2D-SIMULATE-EDITMODE-PROBE] startY={firstY:F5} finalY={finalY:F5} "
                        + $"dropped={(firstY - finalY):F5} sawStrictDecrease={sawStrictDecrease}"
                );

                Assert.IsTrue(
                    sawStrictDecrease,
                    "Y never strictly decreased across consecutive Physics2D.Simulate calls — the edit-mode "
                        + "PhysicsScene2D did not step."
                );
                Assert.Less(
                    finalY,
                    firstY - 0.5f,
                    $"Body did not fall: startY={firstY}, finalY={finalY}. Physics2D.Simulate(Script) does not "
                        + "integrate an edit-mode-opened built-in 2D physics scene in batchmode."
                );
            }
            finally
            {
                UnityEngine.Physics2D.gravity = prevGravity;
                UnityEngine.Physics2D.simulationMode = prevMode;
                if (opened.IsValid())
                    EditorSceneManager.CloseScene(opened, removeScene: true);
            }
        }
    }
}
