using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The FAITHFUL authoring→bake confirmation of the 360-ray collider-parity scan. For each shape kind and each
    /// of three scale×size modes, it compares THREE representations of the same intended world shape, ray-for-ray:
    /// (1) a built-in <c>*Collider2D</c> baked through the package into its Box2D-v3 world; (3) a
    /// <see cref="PhysicsShape2DAuthoring"/> baked through the package into the same world; (2) a live native
    /// <c>*Collider2D</c> driving Unity's own Box2D-v2 physics. The baked shapes (reps 1, 3) come from a SubScene
    /// authored by <c>RayScanParityFixtureBuilder</c> and run through the REAL bakers — the path the precursor
    /// <c>PhysicalExtentParityGate</c> never exercised (it built runtime structs directly and held scale at 1). The
    /// native shape (rep 2) is built live in this test.
    /// </summary>
    /// <remarks>
    /// <para>This is the gate that would catch a divergence between the built-in box baker and the custom box
    /// baker, or a mis-read of <c>PhysicsShape2DAuthoring.BoxSize</c> vs <c>BoxCollider2D.size</c> — the layer the
    /// reported "PhysicsShape2D box is 2× a BoxCollider2D of the same size" defect would live in if it is in the
    /// package at all. The cheap direct-world disambiguator (runtime geometry + bake-scale helper math) is
    /// <see cref="ColliderParityRayScanGate"/>; this is the slower SubScene-bake confirmation.</para>
    ///
    /// <para><b>Build the fixture first</b> via
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.RayScanParityFixtureBuilder.BuildAll</c>. The gate
    /// matches the 24 baked static bodies (4 kinds × 3 modes × 2 lanes) to their (kind, mode, lane) by the baked
    /// <c>initialPosition</c> — the layout constants here MUST mirror the builder's
    /// <c>CentreFor</c>/<c>ColSpacing</c>/<c>RowSpacing</c> (the runtime Tests asmdef cannot reference the Editor
    /// builder; duplicating the load-bearing layout is the package's established pattern,
    /// FilterBakeParityGate/Phase12ColliderScaleGate). The capsule's custom-bake lane under NON-UNIFORM scale uses
    /// the custom baker's derive-then-scale-caps approximation (a documented package behaviour distinct from the
    /// built-in scale-then-derive), so that one cell is asserted only against itself's hit/miss and logged, not
    /// pinned tight against native — see <c>CapsuleCustomNonUniformIsApproximate</c>.</para>
    /// </remarks>
    public sealed class AuthoringBakeRayScanGate
    {
        const int LoadTimeoutFrames = 600;
        const int Rays = 360;
        const float BoxPolyEps = 0.02f;
        const float RoundEps = 0.01f;
        const string ParentScenePath = "Assets/EntitiesPhysics2DFixture/RayScanParity.unity";

        // Mirror of RayScanParityFixtureBuilder layout constants.
        const float ColSpacing = 14f;
        const float RowSpacing = 30f;

        static readonly float2 BoxBaseSize = new(1f, 1f);
        const float CircleBaseRadius = 0.5f;
        static readonly float2 CapsuleBaseSize = new(1f, 2f);
        const float PolygonCircumradius = 1f;

        static float2 CentreFor(int kindIndex, int modeIndex, int lane) =>
            new float2(lane * ColSpacing, (kindIndex * 3 + modeIndex) * RowSpacing);

        static float2 ScaleFor(int modeIndex) =>
            modeIndex switch
            {
                0 => new float2(1f, 1f),
                1 => new float2(2f, 2f),
                _ => new float2(2f, 0.5f),
            };

        static readonly string[] ModeName = { "UnitScaleUnitSize", "DoubleScaleHalfSize", "NonUniformScaleSize" };

        // --- native GameObject physics state ----------------------------------------------------------------

        SimulationMode2D _prevMode;
        bool _prevStartInColliders;

        [SetUp]
        public void SetUp()
        {
            _prevMode = UnityEngine.Physics2D.simulationMode;
            _prevStartInColliders = UnityEngine.Physics2D.queriesStartInColliders;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.queriesStartInColliders = true;
        }

        [TearDown]
        public void TearDown()
        {
            UnityEngine.Physics2D.queriesStartInColliders = _prevStartInColliders;
            UnityEngine.Physics2D.simulationMode = _prevMode;
        }

        // --- the scan ---------------------------------------------------------------------------------------

        static float[] ScanPackage(PhysicsWorld pw, float2 center, float ringRadius, ulong onlyEntity, Dictionary<Entity, byte> wanted)
        {
            var d = new float[Rays];
            for (var i = 0; i < Rays; i++)
            {
                sincos(radians((float)i), out var s, out var c);
                var ringPoint = center + new float2(c, s) * ringRadius;
                var dir = -new float2(c, s);
                d[i] = -1f;
                // Use AllSorted and take the nearest hit whose body is the wanted entity, so a neighbouring shape
                // (should never be in range, but be robust) cannot contaminate this lane's fingerprint.
                using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
                PhysicsQueries2D.Raycast(pw, ringPoint, dir, ringRadius, 0ul, hits);
                for (var h = 0; h < hits.Length; h++)
                {
                    if (!wanted.ContainsKey(hits[h].entity))
                        continue;
                    d[i] = length(hits[h].point - ringPoint);
                    break; // nearest-first
                }
            }
            return d;
        }

        static float[] ScanNative(float2 center, float ringRadius)
        {
            var d = new float[Rays];
            for (var i = 0; i < Rays; i++)
            {
                sincos(radians((float)i), out var s, out var c);
                var ringPoint = center + new float2(c, s) * ringRadius;
                var dir = -new float2(c, s);
                var hit = UnityEngine.Physics2D.Raycast((Vector2)ringPoint, (Vector2)dir, ringRadius);
                d[i] = hit.collider != null ? hit.distance : -1f;
            }
            return d;
        }

        static (float worst, int worstAngle, int viol, float pv, float gv, int misA, int misB) Compare(
            float[] a,
            float[] b,
            float eps
        )
        {
            var worst = 0f;
            var worstAngle = -1;
            var viol = -1;
            var pv = 0f;
            var gv = 0f;
            var misA = 0;
            var misB = 0;
            for (var i = 0; i < Rays; i++)
            {
                if (a[i] < 0f)
                    misA++;
                if (b[i] < 0f)
                    misB++;
                if (a[i] >= 0f && b[i] >= 0f)
                {
                    var delta = abs(a[i] - b[i]);
                    if (delta > worst)
                    {
                        worst = delta;
                        worstAngle = i;
                    }
                    if (delta > eps && viol < 0)
                    {
                        viol = i;
                        pv = a[i];
                        gv = b[i];
                    }
                }
            }
            return (worst, worstAngle, viol, pv, gv, misA, misB);
        }

        // Assert a pair of fingerprints match (all-hit + epsilon). `tight` false → log + hit/miss only (the
        // capsule custom non-uniform approximation cell).
        static void AssertPair(string label, float[] a, float[] b, float eps, bool tight)
        {
            var (worst, wa, viol, pv, gv, misA, misB) = Compare(a, b, eps);
            Debug.Log(
                $"[PHYSICS2D-BAKE-RAYSCAN] {label}: worstΔ={worst:F5} @θ={wa} eps={eps} tight={tight} "
                    + $"missA={misA} missB={misB} | θ0 {a[0]:F4}/{b[0]:F4} θ90 {a[90]:F4}/{b[90]:F4} "
                    + $"θ45 {a[45]:F4}/{b[45]:F4} θ180 {a[180]:F4}/{b[180]:F4} θ270 {a[270]:F4}/{b[270]:F4}"
            );
            Assert.AreEqual(0, misA, $"[{label}] {misA} rays missed shape A — geometry hole / too small.");
            Assert.AreEqual(0, misB, $"[{label}] {misB} rays missed shape B.");
            if (tight)
                Assert.Less(
                    viol,
                    0,
                    $"[{label}] per-ray distance parity BROKE at θ={viol}: A={pv:F4} m B={gv:F4} m "
                        + $"(Δ={abs(pv - gv):F4} > eps {eps}). A 2× extent shows as the smaller-extent shape's "
                        + "surface reached ~half-a-base-size sooner at the face angles."
                );
        }

        // --- baked-entity discovery -------------------------------------------------------------------------

        // Find the baked static entity whose initialPosition matches `center` (within 0.25 m).
        static Entity FindBakedAt(EntityManager em, EntityQuery q, float2 center)
        {
            using var ents = q.ToEntityArray(Allocator.Temp);
            using var defs = q.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            for (var i = 0; i < ents.Length; i++)
                if (lengthsq(defs[i].initialPosition - center) < 0.0625f)
                    return ents[i];
            return Entity.Null;
        }

        // --- the gate ---------------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator ThreeWayRayParity_AcrossKindsModes_BuiltinBake_CustomBake_Native()
        {
            SceneManager.LoadScene(ParentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");
            var em = world.EntityManager;

            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup.");
            // Let the world tick so bodies are CREATED (the package needs one update to create the Box2D bodies
            // and pack userData), but the bodies are static so they never integrate — leaving the group enabled
            // is fine and is what creates the queryable world.
            var shapeQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );
            var liveQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );

            const int Expected = 4 * 3 * 2; // kinds × modes × lanes
            var framesWaited = 0;
            while (shapeQuery.CalculateEntityCount() < Expected && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.AreEqual(
                Expected,
                shapeQuery.CalculateEntityCount(),
                $"Expected {Expected} baked static bodies after {framesWaited} frames; got "
                    + $"{shapeQuery.CalculateEntityCount()}. Build the fixture first via -executeMethod "
                    + "Zori.Entities.Physics2D.Tests.Editor.RayScanParityFixtureBuilder.BuildAll."
            );

            // Ensure the Box2D bodies are created (one update) so the query world is populated and userData packed.
            framesWaited = 0;
            while (liveQuery.CalculateEntityCount() < Expected && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.AreEqual(
                Expected,
                liveQuery.CalculateEntityCount(),
                "Box2D bodies were not all created for the baked shapes."
            );

            var pw = GetWorld(em);

            // The baked geometry the scan measures, on the record (the divergence is LOCATED here, not guessed):
            // the custom-bake box/polygon carry a non-zero corner-rounding `radius` (from PhysicsShape2DAuthoring's
            // m_Radius default 0.5, scaled by cmax), while the built-in collider bake carries radius 0 — see the
            // class-level diagnosis. This dump makes that visible whenever the gate runs.
            using (var dShapes = shapeQuery.ToComponentDataArray<PhysicsShape2D>(Allocator.Temp))
            using (var dDefs = shapeQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp))
            {
                for (var i = 0; i < dShapes.Length; i++)
                    Debug.Log(
                        $"[PHYSICS2D-BAKE-DUMP] pos={dDefs[i].initialPosition} kind={dShapes[i].kind} "
                            + $"size={dShapes[i].size} radius={dShapes[i].radius} offset={dShapes[i].offset} "
                            + $"c1={dShapes[i].capsuleCenter1} c2={dShapes[i].capsuleCenter2}"
                    );
            }

            // Iterate kinds × modes; scan the builtin-bake lane (rep 1), the custom-bake lane (rep 3), and a live
            // native collider (rep 2) at the SAME world centre, and assert three-way ray parity.
            for (var ki = 0; ki < 4; ki++)
            {
                for (var mi = 0; mi < 3; mi++)
                {
                    var label = $"{KindName(ki)}/{ModeName[mi]}";
                    var R = RingRadiusFor(ki);
                    var eps = (ki == 0 || ki == 3) ? BoxPolyEps : RoundEps; // box/poly looser (skin), circle/cap tight

                    var builtinCentre = CentreFor(ki, mi, 0);
                    var customCentre = CentreFor(ki, mi, 1);

                    var builtinEntity = FindBakedAt(em, shapeQuery, builtinCentre);
                    var customEntity = FindBakedAt(em, shapeQuery, customCentre);
                    Assert.AreNotEqual(Entity.Null, builtinEntity, $"[{label}] no baked builtin-bake body at {builtinCentre}.");
                    Assert.AreNotEqual(Entity.Null, customEntity, $"[{label}] no baked custom-bake body at {customCentre}.");

                    var builtinSet = new Dictionary<Entity, byte> { { builtinEntity, 1 } };
                    var customSet = new Dictionary<Entity, byte> { { customEntity, 1 } };

                    var builtinScan = ScanPackage(pw, builtinCentre, R, 0ul, builtinSet);
                    var customScan = ScanPackage(pw, customCentre, R, 0ul, customSet);

                    // Native lane built live at the builtin centre (separate physics world; reusing the centre is
                    // fine because UnityEngine.Physics2D is a distinct world from the package's PhysicsWorld).
                    var nativeGo = BuildNative(ki, mi, builtinCentre);
                    var nativeScan = ScanNative(builtinCentre, R);
                    Object.DestroyImmediate(nativeGo);

                    // The capsule custom-bake lane under non-uniform scale (mi==2) uses the custom baker's
                    // derive-then-scale-caps approximation — a documented package behaviour that legitimately
                    // differs from native/built-in. Assert it hit-complete and log it, but do not pin it tight.
                    var capsuleCustomApprox = ki == 2 && mi == 2;

                    // (a) built-in-bake vs native — the package's built-in baker must match native Unity physics.
                    AssertPair($"{label} builtinBake-vs-native", builtinScan, nativeScan, eps, tight: true);
                    // (b) custom-bake vs built-in-bake — the dual-surface convergence (the user's question).
                    AssertPair($"{label} customBake-vs-builtinBake", customScan, builtinScan, eps, tight: !capsuleCustomApprox);
                    // (c) custom-bake vs native — the full chain.
                    AssertPair($"{label} customBake-vs-native", customScan, nativeScan, eps, tight: !capsuleCustomApprox);
                }
            }

            yield break;
        }

        // --- helpers ----------------------------------------------------------------------------------------

        static PhysicsWorld GetWorld(EntityManager em)
        {
            var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        static string KindName(int ki) =>
            ki switch
            {
                0 => "BOX",
                1 => "CIRCLE",
                2 => "CAPSULE",
                _ => "POLYGON",
            };

        static float RingRadiusFor(int ki) =>
            ki switch
            {
                0 => length(BoxBaseSize) + 1f, // box
                1 => 2f * CircleBaseRadius + 1f, // circle
                2 => CapsuleBaseSize.y + 1f, // capsule
                _ => PolygonCircumradius + 1f, // polygon
            };

        // Build the live native collider for (kind, mode) at `center`, scaled by the mode's transform scale, so
        // Unity's own physics folds the scale exactly as the package baker reads lossyScale.
        static GameObject BuildNative(int ki, int mi, float2 center)
        {
            var scale = ScaleFor(mi);
            var go = new GameObject($"NativeRayScan_{KindName(ki)}_{mi}");
            go.transform.position = new Vector3(center.x, center.y, 0f);
            go.transform.localScale = new Vector3(scale.x, scale.y, 1f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            switch (ki)
            {
                case 0:
                    go.AddComponent<BoxCollider2D>().size = (Vector2)(BoxBaseSize / scale);
                    break;
                case 1:
                    go.AddComponent<CircleCollider2D>().radius =
                        CircleBaseRadius / max(abs(scale.x), abs(scale.y));
                    break;
                case 2:
                    var cap = go.AddComponent<CapsuleCollider2D>();
                    cap.direction = CapsuleDirection2D.Vertical;
                    cap.size = (Vector2)(CapsuleBaseSize / scale);
                    break;
                default:
                    go.AddComponent<PolygonCollider2D>().SetPath(0, Hexagon(scale));
                    break;
            }
            UnityEngine.Physics2D.SyncTransforms();
            return go;
        }

        static Vector2[] Hexagon(float2 scale)
        {
            var v = new Vector2[6];
            for (var i = 0; i < 6; i++)
            {
                sincos(radians(60f * i), out var s, out var c);
                var world = new float2(c, s) * PolygonCircumradius;
                v[i] = (Vector2)(world / scale);
            }
            return v;
        }
    }
}
