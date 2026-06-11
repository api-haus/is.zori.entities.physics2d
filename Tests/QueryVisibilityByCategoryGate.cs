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
    /// Regression gate for query-visibility-by-category. A spatial query reaches a shape by what the shape IS
    /// (its <c>categories</c>) intersected with what the query wants to hit (its <c>hitLayerMask</c>), and never
    /// by what the shape collides with (its <c>contacts</c> row). The latent gap this closes: no prior query test
    /// passes a non-zero <c>hitLayerMask</c> or queries a shape whose collision-matrix row is empty
    /// (<c>contactBits = 0</c>) — the exact shape a rope anchor on a dedicated, all-unchecked Unity layer bakes to,
    /// which the broken filter made invisible to every query (the rope-grab failure).
    /// </summary>
    /// <remarks>
    /// Each test runs in a dedicated disposable <see cref="World"/> holding the package's four FixedStep systems,
    /// one create-step driven by <c>group.Update()</c>, mirroring <c>FilteringQuerySmoke</c>'s harness. Shapes are
    /// authored directly via <see cref="DirectPhysics2DAuthoring"/> with category/contacts bits set inline, so the
    /// test does not depend on a project layer matrix.
    /// </remarks>
    public sealed class QueryVisibilityByCategoryGate
    {
        const float Dt = 1f / 60f;

        // A single dedicated category bit, the shape a Unity layer bakes to (categoryBits = 1 << layer).
        const int Layer = 6;
        const ulong CategoryMask = 1ul << Layer;
        const ulong All = 0xFFFFFFFFul;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("Physics2DQueryVisibilityWorld");
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

        // A static circle authored exactly as a Unity-layer bake produces it: a dedicated category bit and an
        // explicit contacts row (the discriminator).
        static Entity SpawnCircle(EntityManager em, float2 pos, float radius, ulong categoryBits, ulong contactBits)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Static,
                    initialPosition = pos,
                },
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
        public IEnumerator ContactsZeroShape_FoundByCategoryMask_AndByEverythingMask()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // The shape under test: a marker on a dedicated category whose 2D collision-matrix row is fully
            // unchecked — contactBits = 0. It physically collides with nothing, exactly like a rope anchor on a
            // dedicated, all-unchecked layer.
            var marker = SpawnCircle(em, new float2(0f, 0f), radius: 0.3f, categoryBits: CategoryMask, contactBits: 0ul);

            group.Update(); // create the Box2D body

            var pw = GetWorld(em);
            const float searchRadius = 4f;
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);

            // 1) A SPECIFIC non-zero mask whose bit is the shape's category finds it — query-visibility is the
            //    shape's category intersected with the mask, independent of the shape's (empty) contacts row.
            var nByCategory = PhysicsQueries2D.OverlapCircle(pw, new float2(0f, 0f), searchRadius, CategoryMask, hits);
            var entByCategory = nByCategory > 0 ? hits[0].entity : Entity.Null;

            // 2) The "everything" masks (0 and ~0) find it too — the documented "mask 0 = hit everything" contract.
            var nZero = PhysicsQueries2D.OverlapCircle(pw, new float2(0f, 0f), searchRadius, 0ul, hits);
            var nAllOnes = PhysicsQueries2D.OverlapCircle(pw, new float2(0f, 0f), searchRadius, ~0ul, hits);

            // 3) A DISJOINT mask (a category bit the shape does NOT carry) must NOT find it — the mask still
            //    filters by category; the fix decouples from contacts, it does not make every shape always-visible.
            var disjointMask = 1ul << (Layer + 1);
            var nDisjoint = PhysicsQueries2D.OverlapCircle(pw, new float2(0f, 0f), searchRadius, disjointMask, hits);

            // 4) Casts and ClosestPoint share the same Filter — confirm the contacts=0 shape is reachable through
            //    a swept cast and the distance query the rope grab actually runs.
            var nCast = PhysicsQueries2D.CircleCast(
                pw,
                new float2(0f, -2f),
                0.1f,
                new float2(0f, 1f),
                10f,
                CategoryMask,
                hits
            );
            var gotClosest = PhysicsQueries2D.ClosestPoint(
                pw,
                new float2(0f, 0f),
                searchRadius,
                CategoryMask,
                hits,
                out var closest
            );

            Debug.Log(
                $"[QUERY-VISIBILITY] contacts=0 marker on category 0x{CategoryMask:X} (entity {marker}):\n"
                    + $"  byCategoryMask(0x{CategoryMask:X}): hits={nByCategory} entity={entByCategory}\n"
                    + $"  everythingMask(0): hits={nZero}; everythingMask(~0): hits={nAllOnes}\n"
                    + $"  disjointMask(0x{disjointMask:X}): hits={nDisjoint}\n"
                    + $"  circleCast(byCategory): hits={nCast}; closestPoint(byCategory): got={gotClosest} entity={closest.entity}"
            );

            Assert.AreEqual(
                1,
                nByCategory,
                "A contacts=0 shape on a dedicated category was NOT found by a query whose hitLayerMask is that "
                    + "category. Query-visibility is wrongly coupled to the shape's (empty) contacts row instead of "
                    + "its categories."
            );
            Assert.AreEqual(marker, entByCategory, "Category-mask query resolved the wrong entity.");
            Assert.AreEqual(1, nZero, "The contacts=0 shape was not found by the 'mask 0 = hit everything' path.");
            Assert.AreEqual(1, nAllOnes, "The contacts=0 shape was not found by the '~0 = hit everything' path.");
            Assert.AreEqual(
                0,
                nDisjoint,
                "A disjoint category mask matched the shape — the mask must still filter by category; the fix "
                    + "decouples visibility from contacts, it does not make every shape unconditionally visible."
            );
            Assert.AreEqual(1, nCast, "The contacts=0 shape was not found by a category-masked CircleCast.");
            Assert.IsTrue(gotClosest, "ClosestPoint did not find the contacts=0 shape on its own category.");
            Assert.AreEqual(marker, closest.entity, "ClosestPoint resolved the wrong entity for the contacts=0 shape.");

            world.Dispose();
            yield break;
        }
    }
}
