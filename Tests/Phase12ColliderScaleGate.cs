using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-12 GameObject-parity gate for collider transform-scale baking — the adversarial validator the
    /// Phase-12 smoke (<c>ColliderScaleBakeSmoke</c>, which pins only the helper math) and the impl doc
    /// explicitly escalate to. Each test bakes a REAL scaled collider through the actual scale-aware bakers in a
    /// SubScene and pins one decision point two ways:
    /// <list type="bullet">
    /// <item><b>Bake witness</b> (exact/binary): the baked <see cref="PhysicsShape2D"/> geometry read straight off
    /// the baked entity — box half-extent == scale·half-size, circle radius == cmax·radius, polygon winding
    /// reversed on a mirror, offset signed-scaled, dynamic <see cref="LocalToWorld"/> scale == authored
    /// lossyScale. These read the ECS component/blob directly, with NO Box2D body created, so they cannot leak a
    /// native body.</item>
    /// <item><b>Runtime parity</b> (bounded): a body dropped onto the scaled floor rests ON it — and at the
    /// height/x the GameObject <c>Physics2D.Simulate</c> oracle of the SAME scaled scene rests — via
    /// <see cref="PhysicsParityHarness.RunParity"/>. This is the EMPIRICAL pin the impl could not self-certify:
    /// whether the chosen cmax/winding/offset math equals the real GameObject contact behaviour.</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// <para><b>World isolation.</b> Every test loads its parent scene with <c>LoadSceneMode.Single</c>, which
    /// tears the prior scene's entities out of the default world before the next bakes, and disables the
    /// <c>FixedStepSimulationSystemGroup</c> through the bake-wait (the <c>PhysicsParityHarness</c> discipline), so
    /// a thrown test cannot leak stepped bodies into a later one. The bake-witness tests never create a Box2D body
    /// at all (they read the ECS <see cref="PhysicsShape2D"/> the baker wrote); the parity tests delegate world
    /// lifetime to <see cref="PhysicsParityHarness.RunParity"/>, which tears its reference GameObjects down and
    /// restores every global <c>Physics2D</c> knob.</para>
    ///
    /// <para><b>Why a scaled STATIC floor + an unscaled DYNAMIC faller.</b> The floor is the collider under test;
    /// it is collider-only (static), so it is the contact SURFACE — excluded from the parity-compared set on both
    /// backends. The faller is the one compared body. A correctly-scaled floor catches the faller at the scaled
    /// extent; the buggy unscaled floor would let it fall past everywhere but the unscaled 1×1 centre, which the
    /// settle-region disqualifier catches loudly.</para>
    /// </remarks>
    public sealed class Phase12ColliderScaleGate
    {
        const int LoadTimeoutFrames = 600;
        const float Dt = 1f / 60f;

        // Fixture scene paths/names — mirrored as literals here because the fixture BUILDER lives in the Editor
        // test assembly (Zori.Entities.Physics2D.Tests.Editor), which the runtime test assembly does NOT
        // reference (the dependency runs Editor→Runtime only). The Phase-9 gate uses the same path-literal
        // pattern. These MUST match Phase12ColliderScaleFixtureBuilder's constants.
        const string FixtureRoot = "Assets/EntitiesPhysics2DFixture";
        const string WideBoxParent = FixtureRoot + "/P12WideBox.unity";
        const float WideBoxScaleX = 18.1822f;
        const string NonUniformCircleParent = FixtureRoot + "/P12NonUniformCircle.unity";
        const string NonUniformCapsuleParent = FixtureRoot + "/P12NonUniformCapsule.unity";
        const string NegativePolygonParent = FixtureRoot + "/P12NegativePolygon.unity";
        const string NegativeEdgeParent = FixtureRoot + "/P12NegativeEdge.unity";
        const string ScaledOffsetParent = FixtureRoot + "/P12ScaledOffset.unity";
        const string ScaledDynamicParent = FixtureRoot + "/P12ScaledDynamic.unity";

        // ===============================================================================================
        // Shared bake reader: load a parent SubScene, wait for the bake, hand the caller the world + manager
        // WITHOUT creating any Box2D body (the ECS PhysicsShape2D carries the baked geometry directly). The
        // FixedStepSimulationSystemGroup is held DISABLED the whole time so nothing integrates or creates a body.

        static IEnumerator LoadBakeReadOnly(
            string parentScenePath,
            System.Action<EntityManager> onReady
        )
        {
            SceneManager.LoadScene(parentScenePath, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");
            var em = world.EntityManager;

            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            fixedGroup.Enabled = false; // never create a Box2D body — we only read the baked ECS shape.

            var shapeQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );
            var framesWaited = 0;
            while (shapeQuery.CalculateEntityCount() < 1 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.Greater(
                shapeQuery.CalculateEntityCount(),
                0,
                $"No baked shape appeared after {framesWaited} frames — the SubScene '{parentScenePath}' did not "
                    + "stream/bake. Build the fixtures first via -executeMethod "
                    + "Zori.Entities.Physics2D.Tests.Editor.Phase12ColliderScaleFixtureBuilder.BuildAll."
            );

            onReady(em);

            fixedGroup.Enabled = false;
        }

        // Read the single STATIC body's primary PhysicsShape2D (the scaled floor). Asserts exactly one static body
        // exists so a stray bake cannot silently feed the wrong shape into the witness.
        static PhysicsShape2D ReadStaticFloorShape(EntityManager em, out Entity staticEntity)
        {
            var query = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );
            using var entities = query.ToEntityArray(Allocator.Temp);
            staticEntity = Entity.Null;
            var staticCount = 0;
            foreach (var e in entities)
            {
                if (em.GetComponentData<PhysicsBody2DDefinition>(e).bodyType != PhysicsBody.BodyType.Static)
                    continue;
                staticCount++;
                staticEntity = e;
            }
            Assert.AreEqual(1, staticCount, $"Expected exactly ONE baked static floor body; saw {staticCount}.");
            return em.GetComponentData<PhysicsShape2D>(staticEntity);
        }

        // ===============================================================================================
        // (1) WIDE BOX — the headline. Baked half-extent == scale·half-size, AND two edge discs rest on it.

        [UnityTest]
        public IEnumerator WideBox_BakedExtentScales_AndEdgeDiscsRestAcrossFullWidth()
        {
            PhysicsShape2D shape = default;
            yield return LoadBakeReadOnly(
                WideBoxParent,
                em => shape = ReadStaticFloorShape(em, out _)
            );

            Debug.Log(
                $"[PHYSICS2D-P12-WIDEBOX] kind={shape.kind} size={shape.size} offset={shape.offset} "
                    + $"(expected size.x = 1*{WideBoxScaleX})"
            );

            // BAKE WITNESS (exact): a 1×1 box on Scale X=18.1822 bakes to an 18.1822-wide box — the manual-QA case.
            Assert.AreEqual(PhysicsShape2DKind.Box, shape.kind, "Floor must bake as a Box.");
            Assert.AreEqual(
                WideBoxScaleX,
                shape.size.x,
                1e-3f,
                "Baked box width must equal authored size.x (1) × transform scale.x (18.1822) — the bug baked it "
                    + "as an unscaled 1.0."
            );
            Assert.AreEqual(1f, shape.size.y, 1e-3f, "Baked box height must equal the unscaled y size (1).");
        }

        [UnityTest]
        public IEnumerator WideBox_EdgeDiscs_RestOnScaledFloor_ParityWithGameObject()
        {
            // RUNTIME PARITY: two discs dropped at x=±7 — outside the UNSCALED 1×1 centre (x ∈ [-0.5, 0.5]) but
            // inside the scaled floor (x ∈ [-9.09, 9.09]) — rest on the floor at center y ≈ 0.5 in BOTH mediums. A
            // buggy unscaled floor lets them fall to y≈-100; the settle region (y ∈ [0.2, 1.0]) disqualifies that.
            yield return PhysicsParityHarness.RunParity(
                "P12WideBox",
                "P12WideBox_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.05f,
                    positionGrowthPerStep = 0.012f,
                    angleCapRadians = 1.2f, // symmetric discs: roll is not a parity observable
                    settleRegionMin = new float2(-9.5f, 0.2f),
                    settleRegionMax = new float2(9.5f, 1.1f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // ===============================================================================================
        // (2) NON-UNIFORM CIRCLE — the escalated cmax rule, pinned by bake witness AND empirical rest height.

        [UnityTest]
        public IEnumerator NonUniformCircle_BakedRadiusIsCmax()
        {
            PhysicsShape2D shape = default;
            yield return LoadBakeReadOnly(
                NonUniformCircleParent,
                em => shape = ReadStaticFloorShape(em, out _)
            );

            Debug.Log(
                $"[PHYSICS2D-P12-CIRCLE] kind={shape.kind} radius={shape.radius} "
                    + "(authored radius=1, scale=(3,1.4), cmax → 3)"
            );

            // BAKE WITNESS (exact): a radius-1 circle on Scale (3, 1.4). cmax(|3|,|1.4|)=3 → effective radius 3. A
            // circle cannot be an ellipse, so the LARGER absolute axis is used.
            Assert.AreEqual(PhysicsShape2DKind.Circle, shape.kind, "Floor must bake as a Circle.");
            Assert.AreEqual(
                3f,
                shape.radius,
                1e-4f,
                "Baked circle radius must be cmax(abs(scale))·radius = max(3, 1.4)·1 = 3 — the max-axis rule."
            );
        }

        [UnityTest]
        public IEnumerator NonUniformCircle_DiscRestsOnScaledApex_ParityWithGameObject()
        {
            // EMPIRICAL PIN of the radius rule: the floor circle center is at y=-3, so a cmax=3 circle's apex is at
            // y=0 and the disc rests at center y ≈ 0.5. If GameObject's CircleCollider2D used a DIFFERENT rule
            // (min-axis → r=1.4, apex at y=-1.6; or y-axis → r=1.4) the GameObject oracle would rest at a different
            // height and the position parity band would break. Both backends agreeing at y≈0.5 is the proof that
            // the package's cmax matches the GameObject contact behaviour — the thing the smoke could not show.
            yield return PhysicsParityHarness.RunParity(
                "P12NonUniformCircle",
                "P12NonUniformCircle_Sub",
                Dt,
                300,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.06f,
                    positionGrowthPerStep = 0.012f,
                    angleCapRadians = 1.5f, // a disc on a curved apex rolls; roll is not a parity observable
                    settleRegionMin = new float2(-2.0f, 0.0f),
                    settleRegionMax = new float2(2.0f, 1.2f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // ===============================================================================================
        // (3) NON-UNIFORM CAPSULE — caps re-derived from scaled size, pinned by bake witness AND rest height.

        [UnityTest]
        public IEnumerator NonUniformCapsule_BakedCapsuleFromScaledSize()
        {
            PhysicsShape2D shape = default;
            yield return LoadBakeReadOnly(
                NonUniformCapsuleParent,
                em => shape = ReadStaticFloorShape(em, out _)
            );

            Debug.Log(
                $"[PHYSICS2D-P12-CAPSULE] kind={shape.kind} radius={shape.radius} "
                    + $"c1={shape.capsuleCenter1} c2={shape.capsuleCenter2} "
                    + "(horizontal size (4,2), scale (2.5,1.2) → scaled size (10,2.4): radius=1.2, caps x=±3.8)"
            );

            // BAKE WITNESS (exact): horizontal capsule size (4,2) on scale (2.5,1.2). Scaled size = (10, 2.4).
            // Horizontal → radius = scaledHeight/2 = 1.2; caps at x = ±(scaledWidth/2 − radius) = ±(5 − 1.2) = ±3.8.
            // This is the cap-from-scaled-SIZE rule: the radius follows the y-axis scale (height), the segment the
            // x-axis scale (width) — NOT a single uniform factor.
            Assert.AreEqual(PhysicsShape2DKind.Capsule, shape.kind, "Floor must bake as a Capsule.");
            Assert.AreEqual(1.2f, shape.radius, 1e-4f, "Capsule radius = scaledHeight/2 = 2.4/2 = 1.2.");
            var capX = max(abs(shape.capsuleCenter1.x), abs(shape.capsuleCenter2.x));
            Assert.AreEqual(3.8f, capX, 1e-4f, "Capsule end-cap centers at x = ±(scaledWidth/2 − radius) = ±3.8.");
        }

        [UnityTest]
        public IEnumerator NonUniformCapsule_DiscRestsOnScaledTop_ParityWithGameObject()
        {
            // EMPIRICAL PIN: the capsule center is at y=-1.2; with radius 1.2 the scaled top is at y=0, so the disc
            // rests at center y ≈ 0.5. If GameObject's CapsuleCollider2D deformed differently (a uniform-scaled cap
            // rather than cap-from-scaled-size), the top height would differ and the parity band would break.
            yield return PhysicsParityHarness.RunParity(
                "P12NonUniformCapsule",
                "P12NonUniformCapsule_Sub",
                Dt,
                300,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.06f,
                    positionGrowthPerStep = 0.012f,
                    angleCapRadians = 1.5f,
                    settleRegionMin = new float2(-3.8f, 0.0f),
                    settleRegionMax = new float2(3.8f, 1.2f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // ===============================================================================================
        // (4) NEGATIVE-SCALE POLYGON — winding reversed on a mirror, pinned by vertex order AND the disc resting.

        [UnityTest]
        public IEnumerator NegativePolygon_WindingReversedOnMirror()
        {
            Entity floor = Entity.Null;
            PhysicsShape2D shape = default;
            Unity.Mathematics.float2[] verts = null;
            yield return LoadBakeReadOnly(
                NegativePolygonParent,
                em =>
                {
                    shape = ReadStaticFloorShape(em, out floor);
                    Assert.IsTrue(shape.vertices.IsCreated, "Polygon must carry a vertex blob.");
                    ref var blob = ref shape.vertices.Value;
                    verts = new Unity.Mathematics.float2[blob.points.Length];
                    for (var i = 0; i < blob.points.Length; i++)
                        verts[i] = blob.points[i];
                }
            );

            // Authored CCW points: (-2,-1),(3,-1),(2,0),(-1,0). Under Scale X=-2 each is signed-scaled to
            // (4,-1),(-6,-1),(-4,0),(2,0) — which is now CLOCKWISE (a mirror reverses orientation). The baker
            // reverses the vertex ORDER on a winding-flip, so the stored blob is the reversed-then-scaled order:
            // reverse of [v0..v3] is [v3,v2,v1,v0], each ×(-2,1): (2,0),(-4,0),(-6,-1),(4,-1).
            var expected = new[]
            {
                new Unity.Mathematics.float2(2f, 0f), // v3 ×(-2,1)
                new Unity.Mathematics.float2(-4f, 0f), // v2 ×(-2,1)
                new Unity.Mathematics.float2(-6f, -1f), // v1 ×(-2,1)
                new Unity.Mathematics.float2(4f, -1f), // v0 ×(-2,1)
            };

            Debug.Log(
                $"[PHYSICS2D-P12-NEGPOLY] kind={shape.kind} verts=[{string.Join(", ", verts)}] "
                    + $"expected=[{string.Join(", ", expected)}] signedArea={SignedArea(verts):F3}"
            );

            Assert.AreEqual(PhysicsShape2DKind.Polygon, shape.kind, "Floor must bake as a Polygon.");
            Assert.AreEqual(expected.Length, verts.Length, "Baked vertex count must match the authored path.");
            for (var i = 0; i < expected.Length; i++)
            {
                Assert.AreEqual(expected[i].x, verts[i].x, 1e-4f, $"vertex {i}.x (signed-scaled + reversed order)");
                Assert.AreEqual(expected[i].y, verts[i].y, 1e-4f, $"vertex {i}.y (signed-scaled + reversed order)");
            }

            // The load-bearing invariant Box2D needs: the stored winding is CCW (positive signed area). The
            // authored points were CCW; the mirror made the signed-scaled points CW; the order reversal restores
            // CCW. A baker that scaled the vertices but did NOT reverse the order would leave a CW (negative-area)
            // hull that PolygonGeometry.Create rejects as inside-out.
            Assert.Greater(
                SignedArea(verts),
                0f,
                "Baked polygon winding must be CCW (positive signed area) after the mirror — the winding reversal "
                    + "restores it; without the reversal the signed-scaled hull is CW (inside-out) and Box2D "
                    + "rejects it."
            );
        }

        [UnityTest]
        public IEnumerator NegativePolygon_DiscRestsOnMirroredSurface_ParityWithGameObject()
        {
            // EMPIRICAL PIN: a disc dropped at x=0 onto the flat top (y=0) of the mirrored trapezoid rests at
            // center y ≈ 0.5 in BOTH mediums. A degenerate/inside-out hull (missed winding reversal) would either
            // fail to create a body or let the disc fall through — the settle region (y ∈ [0.2, 1.0]) catches it.
            yield return PhysicsParityHarness.RunParity(
                "P12NegativePolygon",
                "P12NegativePolygon_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.06f,
                    positionGrowthPerStep = 0.012f,
                    angleCapRadians = 1.2f,
                    settleRegionMin = new float2(-2.0f, 0.2f),
                    settleRegionMax = new float2(2.0f, 1.1f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // ===============================================================================================
        // (5) NEGATIVE-SCALE EDGE — the chain's solid side IS its winding; a mirror must reverse the point order.

        [UnityTest]
        public IEnumerator NegativeEdge_PointOrderReversedOnMirror()
        {
            PhysicsShape2D shape = default;
            Unity.Mathematics.float2[] verts = null;
            yield return LoadBakeReadOnly(
                NegativeEdgeParent,
                em =>
                {
                    shape = ReadStaticFloorShape(em, out _);
                    Assert.IsTrue(shape.vertices.IsCreated, "Edge must carry a vertex blob.");
                    ref var blob = ref shape.vertices.Value;
                    verts = new Unity.Mathematics.float2[blob.points.Length];
                    for (var i = 0; i < blob.points.Length; i++)
                        verts[i] = blob.points[i];
                }
            );

            // Authored points: (8,1),(3,0),(-3,0),(-8,1). Under Scale X=-2, reversed-then-scaled:
            // reverse → (-8,1),(-3,0),(3,0),(8,1); ×(-2,1) → (16,1),(6,0),(-6,0),(-16,1).
            var expected = new[]
            {
                new Unity.Mathematics.float2(16f, 1f),
                new Unity.Mathematics.float2(6f, 0f),
                new Unity.Mathematics.float2(-6f, 0f),
                new Unity.Mathematics.float2(-16f, 1f),
            };

            Debug.Log(
                $"[PHYSICS2D-P12-NEGEDGE] kind={shape.kind} verts=[{string.Join(", ", verts)}] "
                    + $"expected=[{string.Join(", ", expected)}]"
            );

            Assert.AreEqual(PhysicsShape2DKind.Edge, shape.kind, "Floor must bake as an Edge.");
            Assert.AreEqual(expected.Length, verts.Length, "Baked point count must match the authored chain.");
            for (var i = 0; i < expected.Length; i++)
            {
                Assert.AreEqual(expected[i].x, verts[i].x, 1e-4f, $"point {i}.x (signed-scaled + reversed order)");
                Assert.AreEqual(expected[i].y, verts[i].y, 1e-4f, $"point {i}.y (signed-scaled + reversed order)");
            }
        }

        [UnityTest]
        public IEnumerator NegativeEdge_DiscRestsOnMirroredChain_ParityWithGameObject()
        {
            // EMPIRICAL PIN: a disc at x=0 rests on the mirrored chain's solid (up-facing) side at y ≈ 0.5 in BOTH
            // mediums only if the point-order reversal kept the solid side UP. A missed reversal flips the solid
            // side DOWN and the disc falls through (the Phase-1A y=-74 failure mode), caught by the settle region.
            yield return PhysicsParityHarness.RunParity(
                "P12NegativeEdge",
                "P12NegativeEdge_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.06f,
                    positionGrowthPerStep = 0.012f,
                    angleCapRadians = 1.2f,
                    settleRegionMin = new float2(-3.0f, 0.2f),
                    settleRegionMax = new float2(3.0f, 1.1f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // ===============================================================================================
        // (6) SCALED OFFSET — the collider offset must scale per-axis (signed), placing the shape at offset·scale.

        [UnityTest]
        public IEnumerator ScaledOffset_BakedOffsetScalesPerAxis()
        {
            PhysicsShape2D shape = default;
            yield return LoadBakeReadOnly(
                ScaledOffsetParent,
                em => shape = ReadStaticFloorShape(em, out _)
            );

            Debug.Log(
                $"[PHYSICS2D-P12-OFFSET] kind={shape.kind} size={shape.size} offset={shape.offset} "
                    + "(authored offset (4,0), scale (3,1) → baked offset (12,0))"
            );

            // BAKE WITNESS (exact): offset (4,0) on scale (3,1) bakes to offset (12,0) — signed per-axis. The box
            // size also scales: (1,1) → (3,1).
            Assert.AreEqual(PhysicsShape2DKind.Box, shape.kind, "Floor must bake as a Box.");
            Assert.AreEqual(12f, shape.offset.x, 1e-4f, "Baked offset.x = authored offset.x (4) × scale.x (3) = 12.");
            Assert.AreEqual(0f, shape.offset.y, 1e-4f, "Baked offset.y stays 0.");
            Assert.AreEqual(3f, shape.size.x, 1e-4f, "Baked box width scales to 3.");
        }

        [UnityTest]
        public IEnumerator ScaledOffset_DiscRestsAtScaledOffsetLocation_ParityWithGameObject()
        {
            // EMPIRICAL PIN: a disc dropped at x=12 rests on the offset-shifted, scaled box (spanning x ∈ [10.5,
            // 13.5], top at y=0) at center ≈ (12, 0.5) in BOTH mediums. A wrong offset scaling (the box left at
            // x=4, or at x=0) would leave nothing at x=12 and the disc would fall past — the settle region (x ∈
            // [10.5, 13.5]) catches a divergent rest x.
            yield return PhysicsParityHarness.RunParity(
                "P12ScaledOffset",
                "P12ScaledOffset_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.06f,
                    positionGrowthPerStep = 0.012f,
                    angleCapRadians = 1.2f,
                    settleRegionMin = new float2(10.5f, 0.2f),
                    settleRegionMax = new float2(13.5f, 1.1f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // ===============================================================================================
        // (7) SCALED COMPOSITE — merged paths scaled; the disc rests at the scaled extent.

        [UnityTest]
        public IEnumerator ScaledComposite_DiscRestsAtScaledExtent_ParityWithGameObject()
        {
            // EMPIRICAL PIN: the merged bar (local x ∈ [-1.5, 1.5]) on scale (4, 1.5) spans world x ∈ [-6, 6] with
            // its top at y=0. A disc at x=5 — outside the UNSCALED bar (x ∈ [-1.5, 1.5]) but inside the scaled bar
            // — rests at center y ≈ 0.5 in BOTH mediums only if the merged path was scaled. The composite is the
            // STATIC floor (excluded from the compared set); the disc is the one compared body.
            yield return PhysicsParityHarness.RunParity(
                "P12ScaledComposite",
                "P12ScaledComposite_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.06f,
                    positionGrowthPerStep = 0.012f,
                    angleCapRadians = 1.2f,
                    settleRegionMin = new float2(-6.5f, 0.2f),
                    settleRegionMax = new float2(6.5f, 1.1f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // ===============================================================================================
        // (8) SCALED CUSTOM — group vertices scaled; the disc rests at the scaled extent.

        [UnityTest]
        public IEnumerator ScaledCustom_DiscRestsAtScaledExtent_ParityWithGameObject()
        {
            // EMPIRICAL PIN: the custom polygon base quad (local x ∈ [-2, 2]) on scale (3.5, 1.5) spans world x ∈
            // [-7, 7], top at y=0. A disc at x=6 — outside the UNSCALED quad (x ∈ [-2, 2]) — rests at center y ≈
            // 0.5 in BOTH mediums only if the group's vertices were scaled.
            yield return PhysicsParityHarness.RunParity(
                "P12ScaledCustom",
                "P12ScaledCustom_Sub",
                Dt,
                240,
                new PhysicsParityHarness.ParityEnvelope
                {
                    positionBaseMeters = 0.06f,
                    positionGrowthPerStep = 0.012f,
                    angleCapRadians = 1.2f,
                    settleRegionMin = new float2(-7.5f, 0.2f),
                    settleRegionMax = new float2(7.5f, 1.1f),
                    minTravelMeters = 3.0f,
                }
            );
        }

        // ===============================================================================================
        // (9) RENDERING-SCALE PRESERVED — the baked dynamic body's LocalToWorld carries the authored lossyScale,
        //     at the fixed-step write-back AND through the render-rate smoothing system.

        [UnityTest]
        public IEnumerator ScaledDynamic_LocalToWorldCarriesScale_FixedStepAndSmoothing()
        {
            SceneManager.LoadScene(ScaledDynamicParent, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world.");
            var em = world.EntityManager;
            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.Enabled = false;

            // Wait for the dynamic disc to bake. Query the dynamic body specifically (the disc carries the scale).
            var dynQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>(),
                ComponentType.ReadOnly<PhysicsBody2DRenderScale>()
            );
            var framesWaited = 0;
            while (dynQuery.CalculateEntityCount() < 1 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.Greater(
                dynQuery.CalculateEntityCount(),
                0,
                $"Scaled dynamic disc did not bake after {framesWaited} frames. Build the fixtures first."
            );

            // The render-scale component itself must carry the authored lossyScale (2, 3).
            Entity disc = Entity.Null;
            using (var ents = dynQuery.ToEntityArray(Allocator.Temp))
            {
                foreach (var e in ents)
                {
                    if (em.GetComponentData<PhysicsBody2DDefinition>(e).bodyType == PhysicsBody.BodyType.Dynamic)
                        disc = e;
                }
            }
            Assert.AreNotEqual(Entity.Null, disc, "No dynamic disc entity found.");
            var renderScale = em.GetComponentData<PhysicsBody2DRenderScale>(disc).value;
            Debug.Log($"[PHYSICS2D-P12-RENDERSCALE] PhysicsBody2DRenderScale={renderScale} (authored (2,3))");
            Assert.AreEqual(2f, renderScale.x, 1e-4f, "RenderScale.x must carry authored lossyScale.x = 2.");
            Assert.AreEqual(3f, renderScale.y, 1e-4f, "RenderScale.y must carry authored lossyScale.y = 3.");

            // Drive the fixed-step write-back so the LocalToWorld is (re)built as T·R·S. One Update creates the
            // body (no step); the second runs a step + write-back so the matrix is rebuilt from the body pose.
            var savedRate = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);
            fixedGroup.Enabled = true;
            fixedGroup.Update(); // create
            fixedGroup.Update(); // step + write-back rebuilds LocalToWorld

            var ltwFixed = em.GetComponentData<LocalToWorld>(disc).Value;
            DecomposeScale(ltwFixed, out var sxFixed, out var syFixed);
            Debug.Log(
                $"[PHYSICS2D-P12-RENDERSCALE] fixed-step LocalToWorld col0.len={sxFixed:F5} col1.len={syFixed:F5} "
                    + "(expected (2,3))"
            );

            // FIXED-STEP WITNESS: the write-back rebuilt LocalToWorld as T·R·S, so the rotation columns carry the
            // scale — column 0 length = |scale.x| = 2, column 1 length = |scale.y| = 3. The pre-fix write-back
            // built unit-length rotation columns (no scale term), so this would read (1,1) — a loud failure.
            Assert.AreEqual(2f, sxFixed, 1e-3f, "Fixed-step LocalToWorld must carry scale.x=2 (column-0 length).");
            Assert.AreEqual(3f, syFixed, 1e-3f, "Fixed-step LocalToWorld must carry scale.y=3 (column-1 length).");

            // Now drive the render-rate smoothing at a SUB-STEP fraction. PhysicsBody2DSmoothingSystem runs in the
            // TransformSystemGroup at variable rate; advancing SystemAPI.Time past the last fixed step gives it a
            // non-zero timeAhead so it actually interpolates (not an identity write), and re-applies the scale.
            var smoothing = world.GetExistingSystemManaged<SimulationSystemGroup>();
            // Advance the world time by a fraction of a step so the smoothing system's timeAhead is in (0,1)·dt.
            // The default world's Update ticks the SimulationSystemGroup (which contains TransformSystemGroup).
            world.PushTime(new Unity.Core.TimeData(world.Time.ElapsedTime + Dt * 0.5, Dt * 0.5f));
            smoothing.Update();
            world.PopTime();

            var ltwSmoothed = em.GetComponentData<LocalToWorld>(disc).Value;
            DecomposeScale(ltwSmoothed, out var sxSm, out var sySm);
            Debug.Log(
                $"[PHYSICS2D-P12-RENDERSCALE] smoothed LocalToWorld col0.len={sxSm:F5} col1.len={sySm:F5} "
                    + "(expected (2,3) — scale preserved through interpolation)"
            );

            fixedGroup.RateManager = savedRate;
            fixedGroup.Enabled = false;

            // SMOOTHING WITNESS: the render-rate smoothing overwrites LocalToWorld with the interpolated pose, and
            // it too must re-apply the scale as T·R·S. The pre-fix smoothing built unit-length rotation columns,
            // so an interpolated frame would silently drop the scale back to (1,1) even if the fixed step kept it.
            Assert.AreEqual(2f, sxSm, 1e-3f, "Smoothed LocalToWorld must carry scale.x=2 (column-0 length).");
            Assert.AreEqual(3f, sySm, 1e-3f, "Smoothed LocalToWorld must carry scale.y=3 (column-1 length).");
        }

        // ===============================================================================================
        // Helpers.

        // The signed area of a polygon (the shoelace formula). Positive = CCW, negative = CW. Box2D wants CCW.
        static float SignedArea(Unity.Mathematics.float2[] v)
        {
            var a = 0f;
            for (var i = 0; i < v.Length; i++)
            {
                var j = (i + 1) % v.Length;
                a += v[i].x * v[j].y - v[j].x * v[i].y;
            }
            return 0.5f * a;
        }

        // Decompose the in-plane scale of a column-major T·R·S float4x4 as the lengths of its first two columns.
        // For T·R·S the rotation columns are scaled by |scale.x| / |scale.y|, so the column length recovers the
        // absolute scale (the sign of a flip is not recoverable from a column length — by design, a flipped
        // sprite's |scale| is what the renderer reads).
        static void DecomposeScale(float4x4 m, out float scaleX, out float scaleY)
        {
            scaleX = length(m.c0.xy);
            scaleY = length(m.c1.xy);
        }
    }
}
