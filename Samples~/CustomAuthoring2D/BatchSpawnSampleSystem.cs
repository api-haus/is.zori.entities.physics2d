using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D.Samples
{
    /// <summary>
    /// A worked example of the idiomatic mass-spawn path: build one self-describing prefab entity (a dynamic circle
    /// body carrying its <see cref="PhysicsBody2DDefinition"/> + <see cref="PhysicsShape2D"/> + a
    /// <see cref="PhysicsBody2DFormHash"/>), then <c>ecb.Instantiate(prefab)</c> a few copies per frame, scattering
    /// each instance's pose. This is the falling-sand spray pattern — no <c>PhysicsBody2DBatchRequest</c>, no manual
    /// batch surface: the instances are self-describing, so <c>PhysicsWorld2DSystem</c>'s creation loop recognises
    /// the shared form (via the replicated form hash) and serves them from a cached body template once the form's
    /// count crosses the configured threshold, collapsing same-frame runs through a single
    /// <c>PhysicsWorld.CreateBodyBatch</c> automatically.
    /// </summary>
    /// <remarks>
    /// Disabled by default (it requires-for-update a marker the sample scene adds) so importing the sample does not
    /// silently spawn bodies in every world. To run it, add a <c>BatchSpawnSampleConfig</c> singleton to a scene (or
    /// remove the <c>RequireForUpdate</c> in your fork). The sample is editable by design — it is copied into
    /// <c>Assets/Samples/</c> on import, which is the whole point of the customisable surface.
    ///
    /// <para>The prefab is built from code here (no SubScene bake), so its form hash is stamped directly — a baked
    /// prefab would instead carry the hash from <c>PhysicsBody2DFormHashBakingSystem</c>. The hash value only has to
    /// be equal across instances of the one prefab (it is, by replication) and distinct from other forms.</para>
    /// </remarks>
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct BatchSpawnSampleSystem : ISystem
    {
        public const int Count = 256;
        const int PerFrame = 8; // instances per frame — a cross-frame spray, not a single bulk burst

        // The fixed form hash for this sample's one prefab. Any value works; it only needs to be shared by every
        // instance (replicated for free by Instantiate) and not collide with another authored form.
        static readonly uint4 GrainFormHash = new uint4(0x5A11_D000u, 0x6B22_E111u, 0x7C33_F222u, 0x8D44_0333u);

        Entity m_Prefab;
        int m_Remaining;
        Unity.Mathematics.Random m_Rng;

        public void OnCreate(ref SystemState state)
        {
            // Only run when a scene opts in with the config singleton, so the sample is inert on import.
            state.RequireForUpdate<BatchSpawnSampleConfig>();
            m_Rng = new Unity.Mathematics.Random(0x9E3779B9u);
        }

        public void OnUpdate(ref SystemState state)
        {
            var config = SystemAPI.GetSingleton<BatchSpawnSampleConfig>();
            var em = state.EntityManager;

            // Build the prefab once, the first time we run. A prefab entity carries the Prefab tag so it never
            // simulates itself; ecb.Instantiate copies every component (including the form hash) to each instance.
            if (m_Prefab == Entity.Null)
            {
                var radius = config.radius > 0f ? config.radius : 0.25f;
                m_Prefab = DirectPhysics2DAuthoring.Create(
                    em,
                    new PhysicsBody2DDefinition
                    {
                        bodyType = PhysicsBody.BodyType.Dynamic,
                        gravityScale = 1f,
                        useAutoMass = true,
                    },
                    new PhysicsShape2D
                    {
                        kind = PhysicsShape2DKind.Circle,
                        radius = radius,
                        density = 1f,
                        friction = 0.4f,
                    }
                );
                em.AddComponentData(m_Prefab, new PhysicsBody2DFormHash { value = GrainFormHash });
                em.AddComponent<Prefab>(m_Prefab);
                m_Remaining = config.count > 0 ? config.count : Count;
            }

            if (m_Remaining <= 0)
            {
                state.Enabled = false;
                return;
            }

            // Spray a few instances this frame, each scattered to a distinct pose across the spawn AABB. The pose is
            // the only per-instance difference; the body + shape + form hash ride the prefab.
            var thisFrame = m_Remaining < PerFrame ? m_Remaining : PerFrame;
            var ecb = new EntityCommandBuffer(Unity.Collections.Allocator.Temp);
            for (var i = 0; i < thisFrame; i++)
            {
                var instance = ecb.Instantiate(m_Prefab);
                var p = m_Rng.NextFloat2(config.spawnMin, config.spawnMax);
                ecb.SetComponent(
                    instance,
                    new PhysicsBody2DDefinition
                    {
                        bodyType = PhysicsBody.BodyType.Dynamic,
                        gravityScale = 1f,
                        useAutoMass = true,
                        initialPosition = p,
                    }
                );
                ecb.SetComponent(instance, LocalToWorldAt(p));
            }
            ecb.Playback(em);
            ecb.Dispose();

            m_Remaining -= thisFrame;
        }

        static LocalToWorld LocalToWorldAt(float2 p) =>
            new LocalToWorld
            {
                Value = new float4x4(
                    1f, 0f, 0f, p.x,
                    0f, 1f, 0f, p.y,
                    0f, 0f, 1f, 0f,
                    0f, 0f, 0f, 1f
                ),
            };
    }

    /// <summary>
    /// Scene-authored opt-in for <see cref="BatchSpawnSampleSystem"/>: add this singleton (via a tiny authoring
    /// MonoBehaviour + baker in your fork, or from a bootstrap system) to spray a stream of identical bodies on
    /// start.
    /// </summary>
    public struct BatchSpawnSampleConfig : IComponentData
    {
        public int count;
        public float radius;
        public float2 spawnMin;
        public float2 spawnMax;
    }
}
