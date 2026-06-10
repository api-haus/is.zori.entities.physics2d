using Unity.Entities;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a <see cref="PhysicsStep2DAuthoring"/> into the <see cref="PhysicsWorld2DConfig"/> singleton that
    /// <c>PhysicsWorld2DSystem</c> reads at world creation — the 2D analogue of <c>com.unity.physics</c>'s
    /// <c>PhysicsStepBaker</c>. Keys on the authoring component exactly as that baker keys on
    /// <c>PhysicsStepAuthoring</c>; editor-only assembly, so it never reaches a player build.
    /// </summary>
    /// <remarks>
    /// <see cref="TransformUsageFlags.None"/>: the config entity is pure data and carries no pose, so it
    /// needs no <c>LocalToWorld</c> / <c>LocalTransform</c> — keeping the write-back query off it. The package
    /// is single-world, so exactly one of these is expected per baked world; two authored components surface
    /// as a singleton-query throw at world creation (documented in <c>bake-contract.md</c>).
    /// </remarks>
    public sealed class PhysicsStep2DAuthoringBaker : Baker<PhysicsStep2DAuthoring>
    {
        public override void Bake(PhysicsStep2DAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);
            AddComponent(entity, authoring.AsConfig);
        }
    }
}
