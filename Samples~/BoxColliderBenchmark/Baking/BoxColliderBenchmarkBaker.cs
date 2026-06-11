using Unity.Entities;

namespace Zori.Entities.Physics2D.Samples.Baking
{
    /// <summary>
    /// Bakes a <see cref="BoxColliderBenchmarkAuthoring"/> into the <see cref="BoxColliderBenchmarkConfig"/>
    /// singleton that arms <see cref="BoxColliderBenchmarkSpawnerSystem"/>. Editor-only (the Baking assembly
    /// references <c>Unity.Entities.Hybrid</c>), mirroring the package's <c>Authoring</c> / <c>Baking</c> split:
    /// the MonoBehaviour is all-platforms so it can sit on a scene GameObject, the baker is editor-only.
    /// </summary>
    public sealed class BoxColliderBenchmarkBaker : Baker<BoxColliderBenchmarkAuthoring>
    {
        public override void Bake(BoxColliderBenchmarkAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);
            AddComponent(entity, authoring.ToConfig());
        }
    }
}
