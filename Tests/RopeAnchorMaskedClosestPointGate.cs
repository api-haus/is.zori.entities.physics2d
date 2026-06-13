using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-0 substrate verification for the Platformer rope-grab. The grab calls
    /// <c>PlatformerRopeMath.TryDetectRopeAnchor</c>, which is exactly <see cref="PhysicsQueries2D.ClosestPoint"/>
    /// with a <c>hitLayerMask</c> the character baker builds from the authored Unity <c>LayerMask</c>
    /// (<c>(ulong)(uint)RopeAnchorLayerMask.value</c>, a <c>1&lt;&lt;layer</c> bitfield), against an anchor
    /// collider the substrate bakes to <c>categoryBits = 1&lt;&lt;gameObject.layer</c>. This gate exercises that
    /// whole conversion end to end on the substrate: it queries with the SAME mask the baker produces and asserts
    /// the masked closest-point query includes the on-layer anchor and excludes an off-layer body — the four
    /// decompositions the rope-grab failure is one of.
    /// </summary>
    /// <remarks>
    /// <para><b>Why this gate exists on top of the two it overlaps.</b> <see cref="QueryVisibilityByCategoryGate"/>
    /// already proves a masked <c>OverlapCircle</c>/<c>CircleCast</c>/<c>ClosestPoint</c> finds an on-category
    /// contacts=0 marker and a disjoint mask does not; <see cref="FilterBakeParityGate"/> already proves a
    /// GameObject layer bakes to <c>categoryBits = 1&lt;&lt;layer</c>. Neither one tests <c>ClosestPoint</c>'s
    /// mask filtering in the <b>adversarial</b> rope-grab geometry: an off-layer distractor placed <b>closer</b>
    /// than the on-layer anchor. <c>ClosestPoint</c> returns the single nearest survivor, so a query that silently
    /// drops the mask would return the nearer wrong body — the exact failure where "absurd rope length changes
    /// nothing" because the grab keeps resolving to a non-anchor (or to nothing on a strict broad-phase). This is
    /// the killer case for the primitive the grab actually runs, built from its own decision point.</para>
    ///
    /// <para>Bits are authored inline via <see cref="DirectPhysics2DAuthoring"/> (the mechanism witness, not the
    /// asset-import baker), so the test does not depend on a SubScene or the project layer matrix. The category bit
    /// is computed by the IDENTICAL formula the baker uses — <c>(ulong)(uint)((LayerMask)(1 &lt;&lt; layer)).value
    /// == 1&lt;&lt;layer</c> — so the mask under test is byte-for-byte the runtime grab mask. Mirrors the
    /// <see cref="CapsuleQuerySmoke"/> / <see cref="QueryVisibilityByCategoryGate"/> harness: a dedicated
    /// disposable <see cref="World"/> with the package FixedStep systems, one <c>group.Update()</c> to create the
    /// bodies, then the static query helper reads the created world.</para>
    /// </remarks>
    public sealed class RopeAnchorMaskedClosestPointGate
    {
        const float Dt = 1f / 60f;

        // The rope-anchor layer the Platformer sample dedicates (PlatformerSceneBuilder.RopeAnchorCategoryBit = 6).
        // The character baker bakes the authored LayerMask to (ulong)(uint)mask.value; for a single-layer mask that
        // is exactly 1<<layer, the same value the anchor collider's categoryBits bakes to (1<<gameObject.layer).
        const int AnchorLayer = 6;

        // The mask the grab passes, constructed through the SAME path the baker uses (PlatformerCharacterBaker:55):
        // a UnityEngine.LayerMask of the anchor layer, widened (ulong)(uint).value. Asserting it equals 1<<layer
        // pins the conversion itself.
        static readonly ulong RopeAnchorLayerMask = (ulong)(uint)((LayerMask)(1 << AnchorLayer)).value;

        // The off-layer distractor sits on a different layer (a stand-in for a floor/wall on the Default layer 0).
        const int DistractorLayer = 0;
        static readonly ulong DistractorCategory = 1ul << DistractorLayer;
        static readonly ulong AnchorCategory = 1ul << AnchorLayer;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("Physics2DRopeAnchorMaskedWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }

        static PhysicsWorld GetWorld(EntityManager em)
        {
            var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        // A static circle anchor authored exactly as a Unity-layer collider bake produces it: a single category
        // bit (1<<layer) and a contacts row. The discriminator between the on-layer anchor and the off-layer
        // distractor is the categoryBits the query mask intersects.
        static Entity SpawnCircle(EntityManager em, float2 pos, float radius, ulong categoryBits, ulong contactBits)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition { bodyType = PhysicsBody.BodyType.Static, initialPosition = pos },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = radius,
                    density = 1f,
                    friction = 0.4f,
                    categoryBits = categoryBits,
                    contactBits = contactBits,
                }
            );
        }

        [UnityTest]
        public IEnumerator MaskedClosestPoint_GrabsOnLayerAnchor_OverNearerOffLayerBody()
        {
            // (d) The mask the grab passes is the LayerMask→ulong conversion the baker runs; assert it equals the
            // category an anchor on the same layer bakes to. A broken conversion (sign-extension, a 0/all collapse,
            // an off-by-bit) makes the mask never intersect the anchor's category and the grab silently no-ops —
            // the "absurd values change nothing" symptom — so the conversion is pinned before any query runs.
            Assert.AreEqual(
                AnchorCategory,
                RopeAnchorLayerMask,
                $"The baker's LayerMask→ulong conversion ((ulong)(uint)mask.value) = 0x{RopeAnchorLayerMask:X} does "
                    + $"not equal the on-layer anchor's category 1<<{AnchorLayer} = 0x{AnchorCategory:X}. The grab "
                    + "mask can never select the anchor it is meant to select."
            );

            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // The rope-grab geometry, adversarial: the on-layer ANCHOR is FARTHER from the grab point than an
            // off-layer DISTRACTOR (a floor/wall on the Default layer). A correct masked ClosestPoint skips the
            // nearer distractor and returns the farther anchor; a query that ignores the mask returns the nearer
            // distractor (the wrong body) — the failure where the grab never lands on an anchor.
            //   grab point at the origin
            //   distractor at X=1.5 (NEARER), off the anchor layer, contacts=all (a normal collidable body)
            //   anchor     at X=3.0 (FARTHER), on the anchor layer, contacts=0 (a dedicated all-unchecked layer)
            var grabPoint = new float2(0f, 0f);
            var distractor = SpawnCircle(
                em,
                new float2(1.5f, 0f),
                radius: 0.3f,
                categoryBits: DistractorCategory,
                contactBits: 0xFFFFFFFFul
            );
            var anchor = SpawnCircle(
                em,
                new float2(3.0f, 0f),
                radius: 0.3f,
                categoryBits: AnchorCategory,
                contactBits: 0ul
            );

            group.Update(); // create the Box2D bodies (a static-world query needs no step)

            var pw = GetWorld(em);
            const float ropeLength = 8f; // both bodies in range; the mask, not the radius, must do the selecting
            using var scratch = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);

            // (a) UNMASKED (mask 0 = hit everything): the substrate finds SOMETHING, and the nearest is the
            //     distractor. Proves the broad-phase + ClosestPoint mechanism works at all, and fixes the baseline
            //     the mask must change.
            var foundAny = PhysicsQueries2D.ClosestPoint(pw, grabPoint, ropeLength, 0ul, scratch, out var anyClosest);

            // (b)+(c) MASKED to the rope-anchor layer: the on-layer anchor is INCLUDED and the nearer off-layer
            //     distractor is EXCLUDED, so the single nearest survivor is the anchor (the farther body).
            var foundMasked = PhysicsQueries2D.ClosestPoint(
                pw,
                grabPoint,
                ropeLength,
                RopeAnchorLayerMask,
                scratch,
                out var masked
            );

            // (c, isolated) A DISJOINT mask (a layer neither body carries) finds nothing — the mask really filters;
            //     the fix decouples visibility from contacts, it does not make every body unconditionally visible.
            var disjointMask = 1ul << (AnchorLayer + 1);
            var foundDisjoint = PhysicsQueries2D.ClosestPoint(pw, grabPoint, ropeLength, disjointMask, scratch, out _);

            Debug.Log(
                $"[ROPE-ANCHOR-MASK] grab@{grabPoint} ropeLen={ropeLength} mask=0x{RopeAnchorLayerMask:X} "
                    + $"(== 1<<{AnchorLayer}); anchor={anchor}@X3 (cat 0x{AnchorCategory:X}, contacts=0), "
                    + $"distractor={distractor}@X1.5 (cat 0x{DistractorCategory:X}):\n"
                    + $"  unmasked: found={foundAny} entity={anyClosest.entity} dist={anyClosest.distance:F3}\n"
                    + $"  masked:   found={foundMasked} entity={masked.entity} dist={masked.distance:F3}\n"
                    + $"  disjoint(0x{disjointMask:X}): found={foundDisjoint}"
            );

            // (a) The mechanism finds a body, and unmasked the NEAREST is the distractor — the geometry is set up
            //     so the mask has real work to do (the wrong body is closer).
            Assert.IsTrue(
                foundAny,
                "Unmasked ClosestPoint found NOTHING within rope length — the broad-phase or the closest-point mechanism itself is broken."
            );
            Assert.AreEqual(
                distractor,
                anyClosest.entity,
                $"Unmasked ClosestPoint did not return the NEARER body: got {anyClosest.entity}, expected the "
                    + $"distractor {distractor} at X=1.5. The geometry no longer makes the mask load-bearing."
            );

            // (b)+(c) Masked: the anchor is selected over the nearer distractor.
            Assert.IsTrue(
                foundMasked,
                "Masked ClosestPoint found NOTHING on the rope-anchor layer — the on-layer anchor is invisible to "
                    + "a query whose hitLayerMask is its own category. This is the rope-grab failure: the grab "
                    + "query returns nothing, so no rope-length / search-distance value can ever produce a grab."
            );
            Assert.AreEqual(
                anchor,
                masked.entity,
                $"Masked ClosestPoint returned the wrong entity: got {masked.entity}, expected the on-layer anchor "
                    + $"{anchor} (NOT the nearer off-layer distractor {distractor}). The mask is being ignored — the "
                    + "grab resolves to whatever body is nearest, never specifically an anchor."
            );

            // (c, isolated) The disjoint mask excludes BOTH bodies.
            Assert.IsFalse(
                foundDisjoint,
                "A disjoint layer mask still found a body — the mask is not filtering by category at all."
            );

            world.Dispose();
            yield break;
        }
    }
}
