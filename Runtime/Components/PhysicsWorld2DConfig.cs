using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// The optional per-world simulation configuration, baked from a <c>PhysicsStep2DAuthoring</c> in a
    /// SubScene and read once by <c>PhysicsWorld2DSystem</c> when it creates the <c>PhysicsWorld</c>. The
    /// 2D analogue of <c>com.unity.physics</c>'s <c>PhysicsStep</c> singleton: explicit per-scene config,
    /// nothing read from the project's <c>Physics2DSettings</c>.
    /// </summary>
    /// <remarks>
    /// Fields are the genuinely configurable subset of <c>Unity.U2D.Physics.PhysicsWorldDefinition</c>
    /// (verified against the editor <c>6000.6.0a6</c> module XML / DLL): gravity, the Box2D-v3 solver
    /// sub-step count, the worker count, the two sleep / continuous-collision toggles, and the contact-solver
    /// knobs. The world's <c>simulateType</c> is deliberately NOT a field — the package owns stepping with an
    /// explicit <c>PhysicsWorld.Simulate(dt)</c> and the world is locked to <c>SimulationType.Script</c> so the
    /// engine never auto-steps it; a user-set <c>FixedUpdate</c> / <c>Update</c> would double-step. The fixed
    /// timestep is likewise not here — it is the <c>FixedStepSimulationSystemGroup</c> rate, an ECS-global
    /// group property, not a per-physics-world value.
    ///
    /// <para><b>Backward compatibility.</b> When NO config singleton exists, <c>PhysicsWorld2DSystem</c> builds
    /// the world straight from <c>PhysicsWorldDefinition.defaultDefinition</c> — today's exact behaviour. The
    /// config overrides only when present. <see cref="Default"/> carries the same values
    /// <c>defaultDefinition</c> exposes (probed against the installed editor), so a <c>PhysicsStep2DAuthoring</c>
    /// dropped in and left at its inspector defaults is also behaviourally identical to no component.</para>
    /// </remarks>
    public struct PhysicsWorld2DConfig : IComponentData
    {
        /// <summary>Gravity vector applied to all bodies, m/s² (engine <c>PhysicsWorldDefinition.gravity</c>,
        /// a <c>Vector2</c>; stored <c>float2</c> per the package's blittable convention). Default
        /// <c>(0, -9.81)</c>.</summary>
        public float2 gravity;

        /// <summary>Box2D-v3 solver sub-steps per <c>Simulate</c> (engine
        /// <c>PhysicsWorldDefinition.simulationSubSteps</c>). Higher is more stable. Default <c>4</c>. This is
        /// the engine's only sub-step / solver-iteration field — Box2D-v3 has no separate position / velocity
        /// iteration counts like Box2D-v2.</summary>
        public int simulationSubSteps;

        /// <summary>Simulation worker thread count (engine <c>PhysicsWorldDefinition.simulationWorkers</c>),
        /// capped to the device's available cores at runtime. Default <c>64</c> (i.e. "use all cores").</summary>
        public int simulationWorkers;

        /// <summary>Whether continuous collision detection runs between Dynamic and Static bodies (engine
        /// <c>PhysicsWorldDefinition.continuousAllowed</c>). Default <c>true</c> — keep it on to stop fast
        /// bodies tunnelling static geometry.</summary>
        public bool continuousAllowed;

        /// <summary>Whether bodies sleep when idle (engine <c>PhysicsWorldDefinition.sleepingAllowed</c>).
        /// Default <c>true</c>.</summary>
        public bool sleepingAllowed;

        /// <summary>Bounce threshold, m/s (engine <c>PhysicsWorldDefinition.bounceThreshold</c>). The XML
        /// warns against very small values — they prevent bodies sleeping. Default <c>1</c>.</summary>
        public float bounceThreshold;

        /// <summary>Collision speed needed to generate a contact-hit event, m/s (engine
        /// <c>PhysicsWorldDefinition.contactHitEventThreshold</c>). Default <c>1</c>.</summary>
        public float contactHitEventThreshold;

        /// <summary>Contact stiffness, cycles per second (engine
        /// <c>PhysicsWorldDefinition.contactFrequency</c>). Default <c>30</c>.</summary>
        public float contactFrequency;

        /// <summary>Contact bounciness, with <c>1</c> being critical damping, non-dimensional (engine
        /// <c>PhysicsWorldDefinition.contactDamping</c>). Default <c>10</c>.</summary>
        public float contactDamping;

        /// <summary>Speed used to solve overlaps, m/s (engine <c>PhysicsWorldDefinition.contactSpeed</c>).
        /// Default <c>3</c>.</summary>
        public float contactSpeed;

        /// <summary>Contact recycle distance, m; <c>0</c> disables contact-point recycling (engine
        /// <c>PhysicsWorldDefinition.contactRecycleDistance</c>). Default <c>0.05</c>.</summary>
        public float contactRecycleDistance;

        /// <summary>Maximum linear speed clamp, m/s (engine
        /// <c>PhysicsWorldDefinition.maximumLinearSpeed</c>). Default <c>400</c>.</summary>
        public float maximumLinearSpeed;

        /// <summary>
        /// Enable the cached-template creation optimisation. When <c>true</c> (the default), <c>PhysicsWorld2DSystem</c>
        /// caches the prepared body+shape definition per <see cref="PhysicsBody2DFormHash"/> form once that form's
        /// seen-count crosses <see cref="identicalBodyThreshold"/>, then serves every later identical body from the
        /// template (skipping the per-entity definition construction + mass arithmetic). When <c>false</c>, every
        /// body takes the unchanged per-entity creation path. The optimisation is transparent — a cached-template
        /// body is bit-identical to a per-entity one, so on vs off produces the same simulation. Default <c>true</c>.
        /// </summary>
        public bool cacheIdenticalBodies;

        /// <summary>
        /// The form seen-count past which a template is built and used (N in the design). A form whose body count
        /// stays below N never builds a template (no warm-up waste on cold forms); a real spray crosses N almost
        /// immediately and spends the rest of the spray on the cheap cached path. Clamped to <c>&gt;= 1</c>. Default
        /// <c>8</c> (a provisional order-of-magnitude value, deferred to the benchmark sample). Inert when
        /// <see cref="cacheIdenticalBodies"/> is <c>false</c>.
        /// </summary>
        public int identicalBodyThreshold;

        /// <summary>
        /// The Box2D <c>PhysicsWorldDefinition.defaultDefinition</c> values for editor <c>6000.6.0a6</c>
        /// (probed at HEAD). A config built from these reproduces the default world exactly, so the authoring
        /// component's inspector defaults seed from here.
        /// </summary>
        public static PhysicsWorld2DConfig Default =>
            new PhysicsWorld2DConfig
            {
                gravity = new float2(0f, -9.81f),
                simulationSubSteps = 4,
                simulationWorkers = 64,
                continuousAllowed = true,
                sleepingAllowed = true,
                bounceThreshold = 1f,
                contactHitEventThreshold = 1f,
                contactFrequency = 30f,
                contactDamping = 10f,
                contactSpeed = 3f,
                contactRecycleDistance = 0.05f,
                maximumLinearSpeed = 400f,
                cacheIdenticalBodies = true,
                identicalBodyThreshold = 8,
            };
    }
}
