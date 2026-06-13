using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Rendering;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.Rendering;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Samples
{
    /// <summary>
    /// The benchmark workload: build ONE self-describing quad prefab — a dynamic box-collider body
    /// (<see cref="PhysicsShape2DKind.Box"/>) carrying its <see cref="PhysicsBody2DDefinition"/> +
    /// <see cref="PhysicsShape2D"/> + a baked-equivalent <see cref="PhysicsBody2DFormHash"/> + the official
    /// Unity.Entities.Graphics render components (a <c>RenderMeshArray</c> quad + <c>MaterialMeshInfo</c>) — then
    /// <c>ecb.Instantiate(prefab)</c> copies over many frames (the cross-frame spray), scattering each instance's
    /// pose. The pace is the config's three knobs: <c>spawnedPerSecondTarget</c> (the rate, applied against the
    /// frame's dt with a carried fractional remainder), <c>spawnedPerFrameMax</c> (the per-frame ceiling), and
    /// <c>spawnedTotalLimit</c> (the stop point, up to ~1M for an on-screen stress test). The instances are
    /// self-describing, so <c>PhysicsWorld2DSystem</c> recognises the shared form (via the replicated form hash)
    /// and serves them from a cached body template once the form's count crosses the threshold — the optimisation
    /// this sample measures.
    /// </summary>
    /// <remarks>
    /// The rendering integration is the whole point of the box-collider sample: the prefab carries NO transform
    /// glue. The package's <c>PhysicsBody2DWriteBackSystem</c> writes each body's pose into
    /// <see cref="LocalToWorld"/> every fixed step, and Unity.Entities.Graphics renders each instance straight
    /// off that <c>LocalToWorld</c>. The quad mesh + an unlit URP material are built in code (no asset
    /// dependency) and registered once into a <c>RenderMeshArray</c> shared by every instance.
    ///
    /// <para>Disabled until a scene opts in with a <see cref="BoxColliderBenchmarkConfig"/> singleton, so importing
    /// the sample is inert. The on/off + threshold control lives on the scene's <c>PhysicsStep2DAuthoring</c>
    /// (<c>CacheIdenticalBodies</c> / <c>IdenticalBodyThreshold</c>), not here.</para>
    ///
    /// <para>The form hash is stamped directly here because the prefab is code-built (no SubScene bake); a baked
    /// prefab would carry the hash from <c>PhysicsBody2DFormHashBakingSystem</c> instead. The value only has to be
    /// equal across instances of the one prefab (it is, by replication) and distinct from other forms.</para>
    /// </remarks>
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial class BoxColliderBenchmarkSpawnerSystem : SystemBase
    {
        // The fixed form hash for this sample's one box prefab. Any value works — it only needs to be shared by
        // every instance (replicated for free by Instantiate) and not collide with another authored form.
        static readonly uint4 BoxFormHash = new uint4(0xB0_C0_11_DEu, 0x2D_BE_4C_11u, 0x7A_55_19_33u, 0x4F_88_0C_22u);

        Entity m_Prefab;
        Mesh m_QuadMesh;
        Material m_Material;
        int m_Created;
        int m_TotalLimit;
        int m_PerFrameMax;
        float m_PerSecondTarget;

        // Fractional spawn carried across frames: round(rate * dt) per frame loses sub-1 spawns at a low rate, so
        // accumulate rate * dt and spawn the integer part, keeping the remainder for the next frame.
        float m_SpawnCarry;
        Unity.Mathematics.Random m_Rng;

        protected override void OnCreate()
        {
            // Only run when a scene opts in with the config singleton, so the sample is inert on import.
            RequireForUpdate<BoxColliderBenchmarkConfig>();
            m_Rng = new Unity.Mathematics.Random(0x1234_ABCDu);
        }

        protected override void OnDestroy()
        {
            // The runtime-built mesh and material are sample-owned managed objects; free them so repeated
            // PlayMode enter/exit does not leak.
            if (m_QuadMesh != null)
                Object.DestroyImmediate(m_QuadMesh);
            if (m_Material != null)
                Object.DestroyImmediate(m_Material);
        }

        protected override void OnUpdate()
        {
            var config = SystemAPI.GetSingleton<BoxColliderBenchmarkConfig>();
            var em = EntityManager;

            if (m_Prefab == Entity.Null)
            {
                var size = all(config.boxSize > 0f) ? config.boxSize : BoxColliderBenchmarkConfig.DefaultBoxSize;
                m_Prefab = BuildRenderPrefab(em, size);
                m_TotalLimit =
                    config.spawnedTotalLimit > 0
                        ? config.spawnedTotalLimit
                        : BoxColliderBenchmarkConfig.DefaultTotalLimit;
                m_PerFrameMax =
                    config.spawnedPerFrameMax > 0
                        ? config.spawnedPerFrameMax
                        : BoxColliderBenchmarkConfig.DefaultPerFrameMax;
                m_PerSecondTarget =
                    config.spawnedPerSecondTarget > 0f
                        ? config.spawnedPerSecondTarget
                        : BoxColliderBenchmarkConfig.DefaultPerSecondTarget;
            }

            if (m_Created >= m_TotalLimit)
            {
                Enabled = false;
                return;
            }

            // Pace the spray by the three workload knobs: accumulate the per-second target against this frame's dt
            // (carrying the fractional remainder so a low rate still spawns exactly over time), take the integer
            // part, cap it at the per-frame ceiling, and clamp to the remaining total. The pose is the only
            // per-instance difference; the body + box shape + form hash + render components ride the prefab and are
            // replicated by Instantiate.
            m_SpawnCarry += m_PerSecondTarget * SystemAPI.Time.DeltaTime;
            var requested = (int)m_SpawnCarry;
            m_SpawnCarry -= requested;
            var thisFrame = min(min(requested, m_PerFrameMax), m_TotalLimit - m_Created);
            if (thisFrame <= 0)
                return;

            var ecb = new EntityCommandBuffer(Allocator.Temp);
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

            m_Created += thisFrame;
        }

        // Build the one prefab entity: a dynamic box-collider body + the form hash + the Entities.Graphics render
        // components + the Prefab tag. EntityManager (immediate) is used because RenderMeshUtility.AddComponents is
        // a structural change that must run before the Prefab tag is added; ecb.Instantiate then replicates the
        // whole archetype (render components included, RenderMeshArray as a shared component) per instance.
        Entity BuildRenderPrefab(EntityManager em, float2 size)
        {
            var prefab = DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    useAutoMass = true,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = size,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            em.AddComponentData(prefab, new PhysicsBody2DFormHash { value = BoxFormHash });

            // Build the quad mesh + an unlit material in code (no asset), register into a RenderMeshArray, and add
            // the Entities.Graphics render components to the prefab. The renderer draws each instance from the
            // LocalToWorld the package's write-back maintains — no transform glue here.
            m_QuadMesh = BuildQuadMesh(size);
            m_Material = BuildUnlitMaterial();
            var renderMeshArray = new RenderMeshArray(new[] { m_Material }, new[] { m_QuadMesh });
            var renderDesc = new RenderMeshDescription(ShadowCastingMode.Off, receiveShadows: false);
            RenderMeshUtility.AddComponents(
                prefab,
                em,
                renderDesc,
                renderMeshArray,
                MaterialMeshInfo.FromRenderMeshArrayIndices(0, 0)
            );

            // The Prefab tag last, so the prefab never simulates / renders itself; Instantiate strips it from the
            // instances.
            em.AddComponent<Prefab>(prefab);
            return prefab;
        }

        static Mesh BuildQuadMesh(float2 size)
        {
            var hx = size.x * 0.5f;
            var hy = size.y * 0.5f;
            var mesh = new Mesh { name = "BoxColliderBenchmarkQuad" };
            mesh.SetVertices(
                new[]
                {
                    new Vector3(-hx, -hy, 0f),
                    new Vector3(hx, -hy, 0f),
                    new Vector3(hx, hy, 0f),
                    new Vector3(-hx, hy, 0f),
                }
            );
            mesh.SetUVs(
                0,
                new[] { new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(1f, 1f), new Vector2(0f, 1f) }
            );
            mesh.SetTriangles(new[] { 0, 2, 1, 0, 3, 2 }, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        static Material BuildUnlitMaterial()
        {
            // The URP unlit shader is present in mara (URP project); fall back to the built-in unlit color if a
            // non-URP project imports the sample. A solid colour is all the benchmark needs.
            var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
            var material = new Material(shader) { name = "BoxColliderBenchmarkMaterial" };
            if (material.HasProperty("_BaseColor"))
                material.SetColor("_BaseColor", new Color(0.85f, 0.45f, 0.2f, 1f));
            else if (material.HasProperty("_Color"))
                material.SetColor("_Color", new Color(0.85f, 0.45f, 0.2f, 1f));
            return material;
        }

        static LocalToWorld LocalToWorldAt(float2 p) =>
            new LocalToWorld { Value = new float4x4(1f, 0f, 0f, p.x, 0f, 1f, 0f, p.y, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f) };
    }
}
