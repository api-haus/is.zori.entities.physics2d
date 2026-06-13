using Unity.Entities;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The stable public group a consumer orders its own systems around, rather than around the package's
    /// individual systems. Holds the fixed-step simulation pipeline — cleanup, joint creation, the world step,
    /// write-back, and joint break — in that order, and itself runs in
    /// <see cref="FixedStepSimulationSystemGroup"/> at the group's fixed timestep. A consumer reading or writing
    /// stepped poses runs <c>[UpdateAfter(typeof(Physics2DSimulationSystemGroup))]</c>; one feeding the step runs
    /// <c>[UpdateBefore]</c> it. (Smoothing is not in this group — it runs later in
    /// <see cref="TransformSystemGroup"/>.) This mirrors <c>com.unity.physics</c>'s <c>PhysicsSystemGroup</c>.
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public partial class Physics2DSimulationSystemGroup : ComponentSystemGroup { }
}
