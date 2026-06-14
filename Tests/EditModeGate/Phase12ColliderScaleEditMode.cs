using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using Zori.Entities.Physics2D.Tests.Editor;
using static Unity.Mathematics.math;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>Phase12ColliderScaleGate</c>: collider transform-scale baking pinned two ways per
    /// decision point — a bake witness reading the baked <see cref="PhysicsShape2D"/> straight off the entity (no
    /// Box2D body created), and a runtime parity that drops a disc onto the scaled floor and asserts it rests where
    /// the GameObject <c>Physics2D.Simulate</c> oracle of the same scaled scene rests. Geometry is authored by the
    /// REAL scaled colliders at fixture build time; assertions, tolerances, envelopes, and step counts are copied
    /// verbatim from the PlayMode gate.
    /// </summary>
    public sealed class Phase12ColliderScaleEditMode : Physics2DEditModeHarness
    {
        const float Dt = 1f / 60f;

        const float WideBoxScaleX = 18.1822f;

        // Read the single STATIC body's primary PhysicsShape2D (the scaled floor). Asserts exactly one static body
        // exists so a stray bake cannot silently feed the wrong shape into the witness.
        PhysicsShape2D ReadStaticFloorShape(EntityManager em, out Entity staticEntity)
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

        [Test]
        public void WideBox_BakedExtentScales_AndEdgeDiscsRestAcrossFullWidth()
        {
            LoadSubScene(Physics2DFixtures.P12WideBox, "P12WideBox");
            var shape = ReadStaticFloorShape(EntityManager, out _);

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

        [Test]
        public void WideBox_EdgeDiscs_RestOnScaledFloor_ParityWithGameObject()
        {
            // RUNTIME PARITY: two discs dropped at x=±7 — outside the UNSCALED 1×1 centre (x ∈ [-0.5, 0.5]) but
            // inside the scaled floor (x ∈ [-9.09, 9.09]) — rest on the floor at center y ≈ 0.5 in BOTH mediums. A
            // buggy unscaled floor lets them fall to y≈-100; the settle region (y ∈ [0.2, 1.0]) disqualifies that.
            RunParity(
                Physics2DFixtures.P12WideBox,
                "P12WideBox",
                Dt,
                240,
                new ParityEnvelope
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

        [Test]
        public void NonUniformCircle_BakedRadiusIsCmax()
        {
            LoadSubScene(Physics2DFixtures.P12NonUniformCircle, "P12NonUniformCircle");
            var shape = ReadStaticFloorShape(EntityManager, out _);

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

        [Test]
        public void NonUniformCircle_DiscRestsOnScaledApex_ParityWithGameObject()
        {
            // EMPIRICAL PIN of the radius rule: the floor circle center is at y=-3, so a cmax=3 circle's apex is at
            // y=0 and the disc rests at center y ≈ 0.5. If GameObject's CircleCollider2D used a DIFFERENT rule
            // (min-axis → r=1.4, apex at y=-1.6; or y-axis → r=1.4) the GameObject oracle would rest at a different
            // height and the position parity band would break. Both backends agreeing at y≈0.5 is the proof that
            // the package's cmax matches the GameObject contact behaviour — the thing the smoke could not show.
            RunParity(
                Physics2DFixtures.P12NonUniformCircle,
                "P12NonUniformCircle",
                Dt,
                300,
                new ParityEnvelope
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

        [Test]
        public void NonUniformCapsule_BakedCapsuleFromScaledSize()
        {
            LoadSubScene(Physics2DFixtures.P12NonUniformCapsule, "P12NonUniformCapsule");
            var shape = ReadStaticFloorShape(EntityManager, out _);

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

        [Test]
        public void NonUniformCapsule_DiscRestsOnScaledTop_ParityWithGameObject()
        {
            // EMPIRICAL PIN: the capsule center is at y=-1.2; with radius 1.2 the scaled top is at y=0, so the disc
            // rests at center y ≈ 0.5. If GameObject's CapsuleCollider2D deformed differently (a uniform-scaled cap
            // rather than cap-from-scaled-size), the top height would differ and the parity band would break.
            RunParity(
                Physics2DFixtures.P12NonUniformCapsule,
                "P12NonUniformCapsule",
                Dt,
                300,
                new ParityEnvelope
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

        [Test]
        public void NegativePolygon_WindingReversedOnMirror()
        {
            LoadSubScene(Physics2DFixtures.P12NegativePolygon, "P12NegativePolygon");
            var shape = ReadStaticFloorShape(EntityManager, out _);
            Assert.IsTrue(shape.vertices.IsCreated, "Polygon must carry a vertex blob.");
            ref var blob = ref shape.vertices.Value;
            var verts = new float2[blob.points.Length];
            for (var i = 0; i < blob.points.Length; i++)
                verts[i] = blob.points[i];

            // Authored CCW points: (-2,-1),(3,-1),(2,0),(-1,0). Under Scale X=-2 each is signed-scaled to
            // (4,-1),(-6,-1),(-4,0),(2,0) — which is now CLOCKWISE (a mirror reverses orientation). The baker
            // reverses the vertex ORDER on a winding-flip, so the stored blob is the reversed-then-scaled order:
            // reverse of [v0..v3] is [v3,v2,v1,v0], each ×(-2,1): (2,0),(-4,0),(-6,-1),(4,-1).
            var expected = new[]
            {
                new float2(2f, 0f), // v3 ×(-2,1)
                new float2(-4f, 0f), // v2 ×(-2,1)
                new float2(-6f, -1f), // v1 ×(-2,1)
                new float2(4f, -1f), // v0 ×(-2,1)
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

        [Test]
        public void NegativePolygon_DiscRestsOnMirroredSurface_ParityWithGameObject()
        {
            // EMPIRICAL PIN: a disc dropped at x=0 onto the flat top (y=0) of the mirrored trapezoid rests at
            // center y ≈ 0.5 in BOTH mediums. A degenerate/inside-out hull (missed winding reversal) would either
            // fail to create a body or let the disc fall through — the settle region (y ∈ [0.2, 1.0]) catches it.
            RunParity(
                Physics2DFixtures.P12NegativePolygon,
                "P12NegativePolygon",
                Dt,
                240,
                new ParityEnvelope
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

        [Test]
        public void NegativeEdge_PointOrderReversedOnMirror()
        {
            LoadSubScene(Physics2DFixtures.P12NegativeEdge, "P12NegativeEdge");
            var shape = ReadStaticFloorShape(EntityManager, out _);
            Assert.IsTrue(shape.vertices.IsCreated, "Edge must carry a vertex blob.");
            ref var blob = ref shape.vertices.Value;
            var verts = new float2[blob.points.Length];
            for (var i = 0; i < blob.points.Length; i++)
                verts[i] = blob.points[i];

            // Authored points: (8,1),(3,0),(-3,0),(-8,1). Under Scale X=-2, reversed-then-scaled:
            // reverse → (-8,1),(-3,0),(3,0),(8,1); ×(-2,1) → (16,1),(6,0),(-6,0),(-16,1).
            var expected = new[] { new float2(16f, 1f), new float2(6f, 0f), new float2(-6f, 0f), new float2(-16f, 1f) };

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

        [Test]
        public void NegativeEdge_DiscRestsOnMirroredChain_ParityWithGameObject()
        {
            // EMPIRICAL PIN: a disc at x=0 rests on the mirrored chain's solid (up-facing) side at y ≈ 0.5 in BOTH
            // mediums only if the point-order reversal kept the solid side UP. A missed reversal flips the solid
            // side DOWN and the disc falls through (the Phase-1A y=-74 failure mode), caught by the settle region.
            RunParity(
                Physics2DFixtures.P12NegativeEdge,
                "P12NegativeEdge",
                Dt,
                240,
                new ParityEnvelope
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

        [Test]
        public void ScaledOffset_BakedOffsetScalesPerAxis()
        {
            LoadSubScene(Physics2DFixtures.P12ScaledOffset, "P12ScaledOffset");
            var shape = ReadStaticFloorShape(EntityManager, out _);

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

        [Test]
        public void ScaledOffset_DiscRestsAtScaledOffsetLocation_ParityWithGameObject()
        {
            // EMPIRICAL PIN: a disc dropped at x=12 rests on the offset-shifted, scaled box (spanning x ∈ [10.5,
            // 13.5], top at y=0) at center ≈ (12, 0.5) in BOTH mediums. A wrong offset scaling (the box left at
            // x=4, or at x=0) would leave nothing at x=12 and the disc would fall past — the settle region (x ∈
            // [10.5, 13.5]) catches a divergent rest x.
            RunParity(
                Physics2DFixtures.P12ScaledOffset,
                "P12ScaledOffset",
                Dt,
                240,
                new ParityEnvelope
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

        [Test]
        public void ScaledComposite_DiscRestsAtScaledExtent_ParityWithGameObject()
        {
            // EMPIRICAL PIN: the merged bar (local x ∈ [-1.5, 1.5]) on scale (4, 1.5) spans world x ∈ [-6, 6] with
            // its top at y=0. A disc at x=5 — outside the UNSCALED bar (x ∈ [-1.5, 1.5]) but inside the scaled bar
            // — rests at center y ≈ 0.5 in BOTH mediums only if the merged path was scaled. The composite is the
            // STATIC floor (excluded from the compared set); the disc is the one compared body.
            RunParity(
                Physics2DFixtures.P12ScaledComposite,
                "P12ScaledComposite",
                Dt,
                240,
                new ParityEnvelope
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

        [Test]
        public void ScaledCustom_DiscRestsAtScaledExtent_ParityWithGameObject()
        {
            // EMPIRICAL PIN: the custom polygon base quad (local x ∈ [-2, 2]) on scale (3.5, 1.5) spans world x ∈
            // [-7, 7], top at y=0. A disc at x=6 — outside the UNSCALED quad (x ∈ [-2, 2]) — rests at center y ≈
            // 0.5 in BOTH mediums only if the group's vertices were scaled.
            RunParity(
                Physics2DFixtures.P12ScaledCustom,
                "P12ScaledCustom",
                Dt,
                240,
                new ParityEnvelope
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

        [Test]
        public void ScaledDynamic_LocalToWorldCarriesScale_FixedStepAndSmoothing()
        {
            LoadSubScene(Physics2DFixtures.P12ScaledDynamic, "P12ScaledDynamic");

            var em = EntityManager;
            var fixedGroup = FixedGroup;
            fixedGroup.Enabled = false;

            // Query the dynamic body specifically (the disc carries the scale).
            var dynQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>(),
                ComponentType.ReadOnly<PhysicsBody2DRenderScale>()
            );
            Assert.Greater(dynQuery.CalculateEntityCount(), 0, "Scaled dynamic disc did not bake.");

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
            var smoothing = World.GetExistingSystemManaged<SimulationSystemGroup>();
            // Advance the world time by a fraction of a step so the smoothing system's timeAhead is in (0,1)·dt.
            // The default world's Update ticks the SimulationSystemGroup (which contains TransformSystemGroup).
            World.PushTime(new Unity.Core.TimeData(World.Time.ElapsedTime + Dt * 0.5, Dt * 0.5f));
            smoothing.Update();
            World.PopTime();

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
        static float SignedArea(float2[] v)
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
