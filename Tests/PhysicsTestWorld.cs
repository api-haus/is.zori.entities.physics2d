using Unity.Entities;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Builds the disposable <see cref="World"/> the PlayMode gates step by hand: a
    /// <see cref="FixedStepSimulationSystemGroup"/> whose <c>RateManager</c> is swapped for a
    /// <c>FixedRateSimpleManager</c> so each <c>group.Update()</c> is exactly one fixed step, holding the
    /// package's three FixedStep systems (and, with <paramref name="withJoints"/>, the joint create/break pair).
    /// The systems sort into their declared <c>[UpdateBefore]</c>/<c>[UpdateAfter]</c> order, so insertion order
    /// does not matter. The caller owns the returned world and disposes it.
    /// </summary>
    static class PhysicsTestWorld
    {
        public static World Create(
            string name,
            out FixedStepSimulationSystemGroup group,
            float dt,
            bool withJoints = false
        )
        {
            var world = new World(name);
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            if (withJoints)
            {
                fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsJoint2DCreationSystem>());
                fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsJoint2DBreakSystem>());
            }
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }
    }
}
