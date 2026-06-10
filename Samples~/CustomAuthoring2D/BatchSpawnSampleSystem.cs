using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D.Samples
{
    /// <summary>
    /// A worked example of the low-level bulk-creation surface: on the first update it enqueues one
    /// <see cref="PhysicsBody2DBatchRequest"/> for <see cref="Count"/> identical dynamic circle bodies, which
    /// <c>PhysicsBody2DBatchCreationSystem</c> turns into that many live bodies in a single
    /// <c>PhysicsWorld.CreateBodyBatch</c> native call. This is the pattern a sand-grain burst or particle
    /// volley uses instead of authoring N MonoBehaviours or N per-entity definitions.
    /// </summary>
    /// <remarks>
    /// Disabled by default (it requires-for-update a marker the sample scene adds) so importing the sample
    /// does not silently spawn bodies in every world. To run it, add a <c>BatchSpawnSampleConfig</c> singleton
    /// to a scene (or remove the <c>RequireForUpdate</c> in your fork). The sample is editable by design — it
    /// is copied into <c>Assets/Samples/</c> on import, which is the whole point of the customisable surface.
    /// </remarks>
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct BatchSpawnSampleSystem : ISystem
    {
        public const int Count = 256;

        public void OnCreate(ref SystemState state)
        {
            // Only run when a scene opts in with the config singleton, so the sample is inert on import.
            state.RequireForUpdate<BatchSpawnSampleConfig>();
        }

        public void OnUpdate(ref SystemState state)
        {
            // One-shot: emit the batch request, then disable so it does not spawn every frame.
            var config = SystemAPI.GetSingleton<BatchSpawnSampleConfig>();

            var request = state.EntityManager.CreateEntity();
            state.EntityManager.AddComponentData(
                request,
                new PhysicsBody2DBatchRequest
                {
                    count = config.count > 0 ? config.count : Count,
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    radius = config.radius > 0f ? config.radius : 0.25f,
                    density = 1f,
                    spawnMin = config.spawnMin,
                    spawnMax = config.spawnMax,
                    seed = 0x9E3779B9u,
                }
            );

            state.Enabled = false;
        }
    }

    /// <summary>
    /// Scene-authored opt-in for <see cref="BatchSpawnSampleSystem"/>: add this singleton (via a tiny
    /// authoring MonoBehaviour + baker in your fork, or from a bootstrap system) to spawn a batch on start.
    /// </summary>
    public struct BatchSpawnSampleConfig : IComponentData
    {
        public int count;
        public float radius;
        public float2 spawnMin;
        public float2 spawnMax;
    }
}
