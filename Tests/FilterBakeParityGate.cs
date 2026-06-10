using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-5 BAKE-side filter WIRING gate: proves the package's collider bakers actually call
    /// <c>Collider2DBaking.ReadFilter</c> and write its output into the baked <see cref="PhysicsShape2D"/> —
    /// the authoring-<c>gameObject.layer</c> → baked-<c>categoryBits</c>/<c>contactBits</c> path. The fixture
    /// (<c>FilterBakeFixtureBuilder</c>) authors a SubScene of four collider-only bodies on layers
    /// {0, 8, 9, 11}; this PlayMode test loads it and asserts each baked entity's bits against the project's
    /// PERSISTED layer-collision-matrix (read here the same way the import-time bake read it).
    /// </summary>
    /// <remarks>
    /// <para><b>Why this asserts against the persisted matrix, not a per-test mutation.</b> A SubScene in this
    /// project does NOT bake live under a runtime <c>IgnoreLayerCollision</c> call: the bake runs inside the
    /// <c>SubSceneImporter</c> (an asset import worker), which reads the PERSISTED
    /// <c>ProjectSettings/Physics2DSettings.asset</c> matrix, and the importer result is cached keyed on the
    /// scene asset, NOT on the matrix. Two earlier framings confirmed this empirically: mutating the matrix in
    /// the fixture builder (then restoring before the import ran) and mutating it in the test's <c>[SetUp]</c>
    /// (after the cached bake) both left the baked layer-8 row at <c>0xFFFFFFFF</c> while the live
    /// <c>GetLayerCollisionMask(8)</c> read the mutated <c>0xFFFFFDFF</c> — the bake reads the persisted matrix
    /// at import time, full stop. The in-process bake API (<c>BakingUtility.BakeGameObjects</c>) that WOULD let
    /// a test drive the bakers under a runtime matrix is <c>internal</c> to <c>Unity.Entities.Hybrid</c> and
    /// not visible to a package test assembly. So the SubScene path structurally cannot carry a per-test
    /// matrix.</para>
    ///
    /// <para><b>What this gate therefore pins, and what the runtime gate pins.</b> Against the persisted
    /// (default all-on) matrix this gate pins the BAKER WIRING that is otherwise unobservable: each baked body
    /// carries its OWN layer's category (<c>1 &lt;&lt; layer</c>: <c>0x1</c> for layer 0, <c>0x100</c> for
    /// layer 8, …) and the matrix row as its contacts. A baker that forgot to call <c>ReadFilter</c>, or
    /// hardcoded one category for all bodies, or wrote zero, fails here. The NON-default-matrix tracking — that
    /// the resolved bits reproduce an arbitrary disabled-pair matrix row, including the escalated
    /// layer-0-is-not-All case — is pinned by <see cref="FilteringQueryParityGate"/>, which exercises the
    /// IDENTICAL bit formula <c>ReadFilter</c> contains (<c>1&lt;&lt;layer</c> /
    /// <c>(uint)GetLayerCollisionMask(layer)</c>) end-to-end against the GameObject oracle across five matrix
    /// configurations. The two gates together cover the wiring (here) and the matrix-tracking (there).</para>
    /// </remarks>
    public sealed class FilterBakeParityGate
    {
        const int LoadTimeoutFrames = 600;
        const string ParentScenePath = "Assets/EntitiesPhysics2DFixture/FilterBake.unity";

        // Mirror the fixture's authored layers + per-layer Y (the runtime Tests asmdef cannot reference the
        // Editor-platform fixture builder; duplicating the load-bearing constants is the package's pattern).
        const int LA = 8;
        const int LB = 9;
        const int LDefault = 0;
        const int LX = 11;
        const float YA = 2f;
        const float YB = 4f;
        const float YDefault = 6f;
        const float YX = 8f;

        [UnityTest]
        public IEnumerator BakedFilter_CarriesPerLayerCategoryAndPersistedMatrixRow()
        {
            SceneManager.LoadScene(ParentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");
            var em = world.EntityManager;

            var query = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );

            var framesWaited = 0;
            while (query.CalculateEntityCount() < 4 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            var count = query.CalculateEntityCount();
            Assert.GreaterOrEqual(
                count,
                4,
                $"Only {count} baked bodies appeared after {framesWaited} frames — build the fixture first via "
                    + "-executeMethod Zori.Entities.Physics2D.Tests.Editor.FilterBakeFixtureBuilder.Build."
            );

            using var shapes = query.ToComponentDataArray<PhysicsShape2D>(Allocator.Temp);
            using var defs = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);

            // The contacts row the import-time bake SHOULD have produced for a layer is the persisted matrix
            // row, read the same way ReadFilter reads it. (The matrix is the project default all-on here; the
            // assertion is exact against whatever it is, so a future non-default project matrix still passes
            // iff the baker tracked it.)
            ulong PersistedRow(int layer) =>
                unchecked((uint)UnityEngine.Physics2D.GetLayerCollisionMask(layer));

            var sawA = false;
            var sawB = false;
            var sawDefault = false;
            var sawX = false;
            for (var i = 0; i < shapes.Length; i++)
            {
                var y = defs[i].initialPosition.y;
                int layer;
                if (abs(y - YA) < 0.25f) { layer = LA; sawA = true; }
                else if (abs(y - YB) < 0.25f) { layer = LB; sawB = true; }
                else if (abs(y - YDefault) < 0.25f) { layer = LDefault; sawDefault = true; }
                else if (abs(y - YX) < 0.25f) { layer = LX; sawX = true; }
                else continue;

                var expectedCat = 1ul << layer;
                var expectedCon = PersistedRow(layer);
                Debug.Log(
                    $"[PHYSICS2D-BAKE-FILTER] layer {layer} @y={y:F2}: baked cat=0x{shapes[i].categoryBits:X} "
                        + $"con=0x{shapes[i].contactBits:X} | persisted matrix cat=0x{expectedCat:X} con=0x{expectedCon:X}"
                );
                // The category is THIS body's own layer — proves the baker read gameObject.layer per body and
                // routed it through ReadFilter (a forgotten/hardcoded filter would give a uniform or zero cat).
                Assert.AreEqual(
                    expectedCat,
                    shapes[i].categoryBits,
                    $"Baked categoryBits for the layer-{layer} body = 0x{shapes[i].categoryBits:X} != "
                        + $"1<<{layer} = 0x{expectedCat:X}. The baker did not bake this body's own layer "
                        + "category — ReadFilter was not called per body, or its category is hardcoded."
                );
                // The contacts are the persisted matrix row — proves ReadFilter wrote GetLayerCollisionMask's
                // result, not a zero, a garbage value, or a mismatched row.
                Assert.AreEqual(
                    expectedCon,
                    shapes[i].contactBits,
                    $"Baked contactBits for the layer-{layer} body = 0x{shapes[i].contactBits:X} != the "
                        + $"persisted matrix row GetLayerCollisionMask({layer}) = 0x{expectedCon:X}. "
                        + "Collider2DBaking.ReadFilter did not bake the matrix row at import time."
                );
            }

            Assert.IsTrue(sawA && sawB && sawDefault && sawX, "Missing one of the four baked layer bodies.");

            // The categories are DISTINCT per layer — a single shared/hardcoded category would collapse them.
            Debug.Log(
                "[PHYSICS2D-BAKE-FILTER] four baked bodies carry distinct per-layer categories "
                    + "(0x1, 0x100, 0x200, 0x800) and the persisted matrix row as contacts — ReadFilter is "
                    + "wired into the bakers per body. Non-default-matrix tracking is pinned by "
                    + "FilteringQueryParityGate's five configs."
            );
            yield break;
        }
    }
}
