using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif
using UnityEngine;
using UnityEngine.TestTools;
using Zori.Entities.Physics2D.Baking;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The 360-ray inward distance-scan parity gate for the collider→shape conversion, the instrument the user
    /// specified to pin (or refute) the reported "<see cref="PhysicsShape2D"/> box occupies ~2× a
    /// <c>BoxCollider2D</c> of the same authored size" defect. For each shape kind (Box, Circle, Capsule,
    /// Polygon) and each of three scale×size modes (unit-scale×unit-size; 2×scale×half-size; non-uniform), it
    /// builds the SAME intended world shape two ways and scans both with 360 rays cast inward from a common ring
    /// toward a shared centre, asserting epsilon-matching hit distances ray-for-ray.
    /// </summary>
    /// <remarks>
    /// <para><b>The two representations here.</b> (P) the PACKAGE runtime geometry: the bake helpers
    /// (<see cref="Collider2DBaking.ScaleBoxSize"/> / <see cref="Collider2DBaking.ScaleCircleRadius"/> / the
    /// capsule cap-from-scaled-size derivation the bakers use) applied to the authored fields + the mode's scale,
    /// direct-authored as a <see cref="PhysicsShape2D"/> via <see cref="DirectPhysics2DAuthoring.Create"/> and
    /// created through the REAL <c>CreateShapeForBody</c> — the runtime geometry both the built-in-collider bake
    /// and the custom-authoring bake converge on. (G) the native GameObject: the matching built-in
    /// <c>*Collider2D</c> on a <c>Rigidbody2D</c>(static), on a GameObject whose <c>localScale</c> is the mode's
    /// scale, queried with <c>UnityEngine.Physics2D.Raycast</c>. The faithful authoring→bake confirmation (driving
    /// the real bakers through a SubScene) is <see cref="AuthoringBakeRayScanGate"/>; this gate is the fast
    /// direct-world disambiguator (runtime geometry + the bake-scale helper math) and runs with no scene
    /// streaming.</para>
    ///
    /// <para><b>Why epsilon is tight.</b> A ray-vs-static-shape query has no integration step, so the v2-vs-v3
    /// free-fall trajectory drift the trajectory harness bounds (~1.5e-3 m/step) does NOT apply. The only residual
    /// is float query rounding plus the Box2D-v3 polygon skin radius (a box is a polygon), so box/polygon use
    /// <see cref="BoxPolyEps"/> and circle/capsule (no polygon skin) use <see cref="RoundEps"/>. The reported 2×
    /// on a unit box is a 0.5 m per-ray shift — far above either band, so the band cannot mask it. Each test logs
    /// the worst per-ray Δ so the achieved tightness is visible.</para>
    ///
    /// <para><b>Capsule under non-uniform scale.</b> The built-in capsule baker scales the size then derives caps;
    /// the custom capsule baker derives caps then scales them (a documented approximation, see
    /// <c>PhysicsShape2DAuthoringBaker</c>). They agree under uniform scale (modes A, B). This gate's package side
    /// uses the built-in (scale-then-derive) derivation so it tracks the native <c>CapsuleCollider2D</c>; the
    /// custom-bake capsule's non-uniform behaviour is exercised by <see cref="AuthoringBakeRayScanGate"/>.</para>
    /// </remarks>
    public sealed class ColliderParityRayScanGate
    {
        const float Dt = 1f / 60f;
        const int Rays = 360;
        const float BoxPolyEps = 0.02f; // generous over the Box2D-v3 polygon skin radius
        const float RoundEps = 0.01f; // circle/capsule have no polygon skin — tighter

        // ---- modes -----------------------------------------------------------------------------------------

        enum Mode
        {
            UnitScaleUnitSize, // A: localScale (1,1,1), authored size = base
            DoubleScaleHalfSize, // B: localScale (2,2,1), authored size = base/2
            NonUniformScaleSize, // C: localScale (2,0.5,1), authored size = base / (2,0.5)
        }

        static float2 ScaleFor(Mode m) =>
            m switch
            {
                Mode.UnitScaleUnitSize => new float2(1f, 1f),
                Mode.DoubleScaleHalfSize => new float2(2f, 2f),
                _ => new float2(2f, 0.5f),
            };

        // The authored full size that, under the mode's scale, yields the intended base world size. Per-axis so
        // the net world box extent equals `baseSize` in every mode.
        static float2 AuthoredSizeFor(Mode m, float2 baseSize) => baseSize / ScaleFor(m);

        // ---- package world (the FilteringQueryParityGate pattern) ------------------------------------------

        static World MakePackageWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("RayScanParityWorld", out group, Dt);

        static PhysicsWorld GetWorld(EntityManager em)
        {
            var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        // Create one static package body carrying the given shape at the world centre, and return the live world.
        static (World world, FixedStepSimulationSystemGroup group) SpawnPackage(float2 center, PhysicsShape2D shape)
        {
            var world = MakePackageWorld(out var group);
            shape.density = 1f;
            shape.friction = 0.4f;
            DirectPhysics2DAuthoring.Create(
                world.EntityManager,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    gravityScale = 0f,
                    initialPosition = center,
                    useAutoMass = false,
                },
                shape
            );
            group.Update(); // create the Box2D body (no step)
            return (world, group);
        }

        // ---- the package shape for a kind + mode, via the SAME public bake helpers the bakers call -----------

        static PhysicsShape2D PackageBox(Mode m, float2 baseSize)
        {
            var scale = ScaleFor(m);
            var authored = AuthoredSizeFor(m, baseSize);
            return new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Box,
                size = Collider2DBaking.ScaleBoxSize(authored, scale),
                radius = 0f,
            };
        }

        static PhysicsShape2D PackageCircle(Mode m, float baseRadius)
        {
            var scale = ScaleFor(m);
            // The authored radius that, under cmax(scale), yields baseRadius. cmax picks the larger absolute axis.
            var authored = baseRadius / max(abs(scale.x), abs(scale.y));
            return new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Circle,
                radius = Collider2DBaking.ScaleCircleRadius(authored, scale),
            };
        }

        // Built-in capsule derivation (scale the size, THEN derive caps) — tracks the native CapsuleCollider2D.
        static PhysicsShape2D PackageCapsuleVertical(Mode m, float2 baseSize)
        {
            var scale = ScaleFor(m);
            var authored = AuthoredSizeFor(m, baseSize);
            var halfSize = Collider2DBaking.ScaleBoxSize(authored, scale) * 0.5f;
            var capsuleRadius = halfSize.x;
            var half = max(0f, halfSize.y - capsuleRadius);
            return new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Capsule,
                radius = capsuleRadius,
                capsuleCenter1 = new float2(0f, -half),
                capsuleCenter2 = new float2(0f, half),
            };
        }

        static PhysicsShape2D PackagePolygon(Mode m, float2[] baseVerts)
        {
            var scale = ScaleFor(m);
            var flip = Collider2DBaking.FlipsWinding(scale);
            // Author the unscaled hull so that scaled it reproduces baseVerts; then bake-scale it exactly as the
            // polygon baker does (signed per-axis, reversed order on a winding flip). With a positive-determinant
            // scale (all modes here), flip is false, so the order is preserved.
            var authored = new float2[baseVerts.Length];
            for (var i = 0; i < baseVerts.Length; i++)
                authored[i] = baseVerts[i] / scale;
            var b = new BlobBuilder(Allocator.Temp);
            ref var root = ref b.ConstructRoot<PhysicsShape2DVertices>();
            var arr = b.Allocate(ref root.points, authored.Length);
            for (var i = 0; i < authored.Length; i++)
            {
                var src = flip ? authored.Length - 1 - i : i;
                arr[i] = authored[src] * scale;
            }
            var blob = b.CreateBlobAssetReference<PhysicsShape2DVertices>(Allocator.Persistent);
            b.Dispose();
            s_Blobs.Add(blob);
            return new PhysicsShape2D
            {
                kind = PhysicsShape2DKind.Polygon,
                polygonDecompose = false,
                radius = 0f,
                vertices = blob,
            };
        }

        static readonly List<BlobAssetReference<PhysicsShape2DVertices>> s_Blobs = new();

        [TearDown]
        public void TearDown()
        {
            foreach (var b in s_Blobs)
                if (b.IsCreated)
                    b.Dispose();
            s_Blobs.Clear();
        }

        // ---- native GameObject reference -------------------------------------------------------------------

        SimulationMode2D _prevMode;
        bool _prevStartInColliders;

        [SetUp]
        public void SetUp()
        {
            _prevMode = UnityEngine.Physics2D.simulationMode;
            _prevStartInColliders = UnityEngine.Physics2D.queriesStartInColliders;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            // The ring origin is OUTSIDE the shape, but assert ray-start hits anyway for robustness.
            UnityEngine.Physics2D.queriesStartInColliders = true;
        }

        [TearDown]
        public void RestorePhysics()
        {
            UnityEngine.Physics2D.queriesStartInColliders = _prevStartInColliders;
            UnityEngine.Physics2D.simulationMode = _prevMode;
        }

        static GameObject SpawnNative<T>(float2 center, float2 scale, System.Action<T> configure)
            where T : Collider2D
        {
            var go = new GameObject($"NativeRef_{typeof(T).Name}");
            go.transform.position = new Vector3(center.x, center.y, 0f);
            go.transform.localScale = new Vector3(scale.x, scale.y, 1f);
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Static;
            var col = go.AddComponent<T>();
            configure(col);
            UnityEngine.Physics2D.SyncTransforms();
            return go;
        }

        // ---- the 360-ray scan ------------------------------------------------------------------------------

        // Distance from the ring point to the first inward-hit surface, per ray. -1 = miss. ringRadius R.
        static float[] ScanPackage(PhysicsWorld pw, float2 center, float ringRadius)
        {
            var d = new float[Rays];
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            for (var i = 0; i < Rays; i++)
            {
                sincos(radians((float)i), out var s, out var c);
                var ringPoint = center + new float2(c, s) * ringRadius;
                var dir = -new float2(c, s); // inward, toward the centre
                if (PhysicsQueries2D.RaycastClosest(pw, ringPoint, dir, ringRadius, 0ul, out var hit))
                    d[i] = length(hit.point - ringPoint);
                else
                    d[i] = -1f;
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

        // Assert the two distance fingerprints match ray-for-ray: identical hit/miss (binary, exact) and
        // epsilon-matching distances on the rays that hit (continuous, bounded). Logs the worst Δ and the angles.
        static void AssertRayParity(string label, float[] pkg, float[] gameObj, float eps)
        {
            Assert.AreEqual(Rays, pkg.Length);
            Assert.AreEqual(Rays, gameObj.Length);

            var worst = 0f;
            var worstAngle = -1;
            var hitMissMismatch = -1;
            var pkgMisses = 0;
            var goMisses = 0;
            var epsViolationAngle = -1;
            var epsViolationVals = (0f, 0f);

            for (var i = 0; i < Rays; i++)
            {
                var ph = pkg[i] >= 0f;
                var gh = gameObj[i] >= 0f;
                if (!ph)
                    pkgMisses++;
                if (!gh)
                    goMisses++;
                if (ph != gh && hitMissMismatch < 0)
                    hitMissMismatch = i;
                if (ph && gh)
                {
                    var delta = abs(pkg[i] - gameObj[i]);
                    if (delta > worst)
                    {
                        worst = delta;
                        worstAngle = i;
                    }
                    if (delta > eps && epsViolationAngle < 0)
                    {
                        epsViolationAngle = i;
                        epsViolationVals = (pkg[i], gameObj[i]);
                    }
                }
            }

            // Sample the cardinal/diagonal rays into the log so a 2× / offset / rotation is legible at a glance.
            string Sample(int a) => $"θ={a}: pkg={pkg[a]:F4} go={gameObj[a]:F4} Δ={abs(pkg[a] - gameObj[a]):F4}";
            Debug.Log(
                $"[PHYSICS2D-RAYSCAN] {label}: worstΔ={worst:F5} @θ={worstAngle} eps={eps} "
                    + $"pkgMisses={pkgMisses} goMisses={goMisses} | {Sample(0)} | {Sample(90)} | "
                    + $"{Sample(45)} | {Sample(180)} | {Sample(270)}"
            );

            Assert.AreEqual(
                0,
                pkgMisses,
                $"[{label}] {pkgMisses} of 360 inward rays MISSED the package shape — the ring should enclose a "
                    + "convex shape that contains its centre, so every ray must hit. A miss is a geometry hole "
                    + "or a shape far smaller than intended."
            );
            Assert.AreEqual(0, goMisses, $"[{label}] {goMisses} of 360 inward rays MISSED the native collider.");
            Assert.Less(
                hitMissMismatch,
                0,
                $"[{label}] hit/miss disagreement at θ={hitMissMismatch}: package hit={pkg[max(0, hitMissMismatch)] >= 0f} "
                    + $"native hit={gameObj[max(0, hitMissMismatch)] >= 0f}."
            );
            Assert.Less(
                epsViolationAngle,
                0,
                $"[{label}] per-ray distance parity BROKE at θ={epsViolationAngle}: package={epsViolationVals.Item1:F4} m "
                    + $"native={epsViolationVals.Item2:F4} m (Δ={abs(epsViolationVals.Item1 - epsViolationVals.Item2):F4} > "
                    + $"eps {eps}). A 2× extent shows here as the package surface reached ~half-a-base-size sooner than "
                    + "native at the face angles."
            );
        }

        // One full kind×mode case: build the package shape + native collider for the same intended world shape,
        // scan both from a ring of radius `R` about the shared centre, assert ray parity.
        static void RunCase(
            string label,
            float2 center,
            float ringRadius,
            PhysicsShape2D pkgShape,
            GameObject nativeGo,
            float eps
        )
        {
            var (world, _) = SpawnPackage(center, pkgShape);
            try
            {
                var pw = GetWorld(world.EntityManager);
                var pkgScan = ScanPackage(pw, center, ringRadius);
                var goScan = ScanNative(center, ringRadius);
                AssertRayParity(label, pkgScan, goScan, eps);
            }
            finally
            {
                world.Dispose();
                if (nativeGo != null)
                    Object.DestroyImmediate(nativeGo);
            }
        }

        // ====================================================================================================
        // BOX — the user's reported case. base world size (1,1); ring radius = length(size)+1.
        // ====================================================================================================

        static IEnumerator BoxMode(Mode m)
        {
            var baseSize = new float2(1f, 1f);
            var center = new float2(0f, 0f);
            var R = length(baseSize) + 1f;
            var authored = AuthoredSizeFor(m, baseSize);
            var go = SpawnNative<BoxCollider2D>(center, ScaleFor(m), c => c.size = (Vector2)authored);
            RunCase($"BOX/{m}", center, R, PackageBox(m, baseSize), go, BoxPolyEps);
            yield return null;
        }

        [UnityTest]
        public IEnumerator Box_UnitScaleUnitSize() => BoxMode(Mode.UnitScaleUnitSize);

        [UnityTest]
        public IEnumerator Box_DoubleScaleHalfSize() => BoxMode(Mode.DoubleScaleHalfSize);

        [UnityTest]
        public IEnumerator Box_NonUniformScaleSize() => BoxMode(Mode.NonUniformScaleSize);

        // ====================================================================================================
        // CIRCLE — base world radius 0.5; the native CircleCollider2D uses the larger-axis rule (cmax), so both
        // pick the same effective radius. Ring radius = 2r + 1.
        // ====================================================================================================

        static IEnumerator CircleMode(Mode m)
        {
            const float baseR = 0.5f;
            var center = new float2(0f, 0f);
            var R = 2f * baseR + 1f;
            var scale = ScaleFor(m);
            var authored = baseR / max(abs(scale.x), abs(scale.y));
            var go = SpawnNative<CircleCollider2D>(center, scale, c => c.radius = authored);
            RunCase($"CIRCLE/{m}", center, R, PackageCircle(m, baseR), go, RoundEps);
            yield return null;
        }

        [UnityTest]
        public IEnumerator Circle_UnitScaleUnitSize() => CircleMode(Mode.UnitScaleUnitSize);

        [UnityTest]
        public IEnumerator Circle_DoubleScaleHalfSize() => CircleMode(Mode.DoubleScaleHalfSize);

        [UnityTest]
        public IEnumerator Circle_NonUniformScaleSize() => CircleMode(Mode.NonUniformScaleSize);

        // ====================================================================================================
        // CAPSULE (vertical) — base world size (1,2). Package uses the built-in scale-then-derive derivation so
        // it tracks the native CapsuleCollider2D in every mode. Ring radius = baseHeight + 1.
        // ====================================================================================================

        static IEnumerator CapsuleMode(Mode m)
        {
            var baseSize = new float2(1f, 2f);
            var center = new float2(0f, 0f);
            var R = baseSize.y + 1f;
            var authored = AuthoredSizeFor(m, baseSize);
            var go = SpawnNative<CapsuleCollider2D>(
                center,
                ScaleFor(m),
                c =>
                {
                    c.direction = CapsuleDirection2D.Vertical;
                    c.size = (Vector2)authored;
                }
            );
            RunCase($"CAPSULE/{m}", center, R, PackageCapsuleVertical(m, baseSize), go, RoundEps);
            yield return null;
        }

        [UnityTest]
        public IEnumerator Capsule_UnitScaleUnitSize() => CapsuleMode(Mode.UnitScaleUnitSize);

        [UnityTest]
        public IEnumerator Capsule_DoubleScaleHalfSize() => CapsuleMode(Mode.DoubleScaleHalfSize);

        [UnityTest]
        public IEnumerator Capsule_NonUniformScaleSize() => CapsuleMode(Mode.NonUniformScaleSize);

        // ====================================================================================================
        // POLYGON — a convex regular hexagon of circumradius 1 (exercises the multi-vertex hull path, not a box
        // in disguise). Ring radius = circumradius + 1.
        // ====================================================================================================

        static float2[] Hexagon(float circumradius)
        {
            var v = new float2[6];
            for (var i = 0; i < 6; i++)
            {
                sincos(radians(60f * i), out var s, out var c);
                v[i] = new float2(c, s) * circumradius; // CCW
            }
            return v;
        }

        static IEnumerator PolygonMode(Mode m)
        {
            const float circumradius = 1f;
            var center = new float2(0f, 0f);
            var R = circumradius + 1f;
            var baseVerts = Hexagon(circumradius);
            var scale = ScaleFor(m);
            // The native PolygonCollider2D path: author the unscaled hull, let the transform localScale fold it.
            var authored = new Vector2[baseVerts.Length];
            for (var i = 0; i < baseVerts.Length; i++)
                authored[i] = (Vector2)(baseVerts[i] / scale);
            var go = SpawnNative<PolygonCollider2D>(center, scale, c => c.SetPath(0, authored));
            RunCase($"POLYGON/{m}", center, R, PackagePolygon(m, baseVerts), go, BoxPolyEps);
            yield return null;
        }

        [UnityTest]
        public IEnumerator Polygon_UnitScaleUnitSize() => PolygonMode(Mode.UnitScaleUnitSize);

        [UnityTest]
        public IEnumerator Polygon_DoubleScaleHalfSize() => PolygonMode(Mode.DoubleScaleHalfSize);

        [UnityTest]
        public IEnumerator Polygon_NonUniformScaleSize() => PolygonMode(Mode.NonUniformScaleSize);
    }
}
