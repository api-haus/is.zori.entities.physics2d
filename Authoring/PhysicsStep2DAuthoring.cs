using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Authoring
{
    /// <summary>
    /// Per-scene simulation configuration — the 2D analogue of <c>com.unity.physics</c>'s
    /// <c>PhysicsStepAuthoring</c>. Place ONE on a GameObject in a SubScene; its baker emits the
    /// <see cref="PhysicsWorld2DConfig"/> singleton, which <c>PhysicsWorld2DSystem</c> reads when it creates
    /// the world. If no <see cref="PhysicsStep2DAuthoring"/> is authored, the world keeps the Box2D
    /// <c>PhysicsWorldDefinition.defaultDefinition</c> — the configuration is explicit and overrides nothing
    /// unless present. Nothing is read from the project's <c>UnityEngine.Physics2D</c> settings at bake or
    /// runtime; the inspector defaults below mirror the Box2D / Physics2D conventions only as starting values.
    /// </summary>
    /// <remarks>
    /// The exposed fields are the genuinely configurable subset of
    /// <c>Unity.U2D.Physics.PhysicsWorldDefinition</c>, verified against the editor <c>6000.6.0a6</c> module
    /// XML / DLL. Two <c>com.unity.physics</c> 3D knobs are deliberately absent because the 2D engine has no
    /// analogue or because the package locks them:
    /// <list type="bullet">
    /// <item><b>Simulation type</b> is not exposed — the package owns stepping with an explicit
    /// <c>PhysicsWorld.Simulate(dt)</c> and locks the world to <c>SimulationType.Script</c>; a user-set
    /// <c>FixedUpdate</c> / <c>Update</c> would make the engine auto-step on top of the package's step.</item>
    /// <item><b>Fixed timestep</b> is not exposed — it is the <c>FixedStepSimulationSystemGroup</c> rate, an
    /// ECS-global group property shared by every system in that group, not a per-physics-world value.</item>
    /// </list>
    /// One component per baked world (single-world package). The seed values come from
    /// <see cref="PhysicsWorld2DConfig.Default"/>, so a component left at its defaults reproduces the default
    /// world exactly.
    /// </remarks>
    [AddComponentMenu("Zori/Entities Physics 2D/Physics Step 2D")]
    [DisallowMultipleComponent]
    public sealed class PhysicsStep2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Gravity vector applied to all bodies, m/s^2 (PhysicsWorldDefinition.gravity).")]
        float2 m_Gravity = PhysicsWorld2DConfig.Default.gravity;

        [SerializeField]
        [Tooltip(
            "Box2D-v3 solver sub-steps per Simulate (PhysicsWorldDefinition.simulationSubSteps). "
                + "Higher means more stability. The engine's only sub-step / solver-iteration knob."
        )]
        int m_SimulationSubSteps = PhysicsWorld2DConfig.Default.simulationSubSteps;

        [SerializeField]
        [Tooltip(
            "Simulation worker thread count (PhysicsWorldDefinition.simulationWorkers), capped to the "
                + "device's cores at runtime. The default 64 means 'use all cores'."
        )]
        int m_SimulationWorkers = PhysicsWorld2DConfig.Default.simulationWorkers;

        [SerializeField]
        [Tooltip(
            "Continuous collision detection between Dynamic and Static bodies "
                + "(PhysicsWorldDefinition.continuousAllowed). Keep on to stop fast bodies tunnelling statics."
        )]
        bool m_ContinuousAllowed = PhysicsWorld2DConfig.Default.continuousAllowed;

        [SerializeField]
        [Tooltip("Whether bodies sleep when idle (PhysicsWorldDefinition.sleepingAllowed).")]
        bool m_SleepingAllowed = PhysicsWorld2DConfig.Default.sleepingAllowed;

        [SerializeField]
        [Tooltip(
            "Bounce threshold, m/s (PhysicsWorldDefinition.bounceThreshold). Very small values prevent "
                + "bodies from sleeping."
        )]
        float m_BounceThreshold = PhysicsWorld2DConfig.Default.bounceThreshold;

        [SerializeField]
        [Tooltip(
            "Collision speed needed to generate a contact-hit event, m/s "
                + "(PhysicsWorldDefinition.contactHitEventThreshold)."
        )]
        float m_ContactHitEventThreshold = PhysicsWorld2DConfig.Default.contactHitEventThreshold;

        [SerializeField]
        [Tooltip("Contact stiffness, cycles per second (PhysicsWorldDefinition.contactFrequency).")]
        float m_ContactFrequency = PhysicsWorld2DConfig.Default.contactFrequency;

        [SerializeField]
        [Tooltip(
            "Contact bounciness, 1 being critical damping, non-dimensional "
                + "(PhysicsWorldDefinition.contactDamping)."
        )]
        float m_ContactDamping = PhysicsWorld2DConfig.Default.contactDamping;

        [SerializeField]
        [Tooltip("Speed used to solve overlaps, m/s (PhysicsWorldDefinition.contactSpeed).")]
        float m_ContactSpeed = PhysicsWorld2DConfig.Default.contactSpeed;

        [SerializeField]
        [Tooltip(
            "Contact recycle distance, m; 0 disables contact-point recycling "
                + "(PhysicsWorldDefinition.contactRecycleDistance)."
        )]
        float m_ContactRecycleDistance = PhysicsWorld2DConfig.Default.contactRecycleDistance;

        [SerializeField]
        [Tooltip("Maximum linear speed clamp, m/s (PhysicsWorldDefinition.maximumLinearSpeed).")]
        float m_MaximumLinearSpeed = PhysicsWorld2DConfig.Default.maximumLinearSpeed;

        public float2 Gravity
        {
            get => m_Gravity;
            set => m_Gravity = value;
        }

        public int SimulationSubSteps
        {
            get => m_SimulationSubSteps;
            set => m_SimulationSubSteps = math.max(1, value);
        }

        public int SimulationWorkers
        {
            get => m_SimulationWorkers;
            set => m_SimulationWorkers = math.max(1, value);
        }

        public bool ContinuousAllowed
        {
            get => m_ContinuousAllowed;
            set => m_ContinuousAllowed = value;
        }

        public bool SleepingAllowed
        {
            get => m_SleepingAllowed;
            set => m_SleepingAllowed = value;
        }

        public float BounceThreshold
        {
            get => m_BounceThreshold;
            set => m_BounceThreshold = math.max(0f, value);
        }

        public float ContactHitEventThreshold
        {
            get => m_ContactHitEventThreshold;
            set => m_ContactHitEventThreshold = math.max(0f, value);
        }

        public float ContactFrequency
        {
            get => m_ContactFrequency;
            set => m_ContactFrequency = math.max(0f, value);
        }

        public float ContactDamping
        {
            get => m_ContactDamping;
            set => m_ContactDamping = math.max(0f, value);
        }

        public float ContactSpeed
        {
            get => m_ContactSpeed;
            set => m_ContactSpeed = math.max(0f, value);
        }

        public float ContactRecycleDistance
        {
            get => m_ContactRecycleDistance;
            set => m_ContactRecycleDistance = math.max(0f, value);
        }

        public float MaximumLinearSpeed
        {
            get => m_MaximumLinearSpeed;
            set => m_MaximumLinearSpeed = math.max(0f, value);
        }

        /// <summary>The runtime config this component bakes to. Read by <c>PhysicsStep2DAuthoringBaker</c>.</summary>
        public PhysicsWorld2DConfig AsConfig =>
            new PhysicsWorld2DConfig
            {
                gravity = m_Gravity,
                simulationSubSteps = m_SimulationSubSteps,
                simulationWorkers = m_SimulationWorkers,
                continuousAllowed = m_ContinuousAllowed,
                sleepingAllowed = m_SleepingAllowed,
                bounceThreshold = m_BounceThreshold,
                contactHitEventThreshold = m_ContactHitEventThreshold,
                contactFrequency = m_ContactFrequency,
                contactDamping = m_ContactDamping,
                contactSpeed = m_ContactSpeed,
                contactRecycleDistance = m_ContactRecycleDistance,
                maximumLinearSpeed = m_MaximumLinearSpeed,
            };

        void OnValidate()
        {
            // Sub-steps / workers must be >= 1 (the engine needs at least one of each), matching the
            // com.unity.physics PhysicsStepAuthoring clamp; the contact / threshold floats are non-negative.
            m_SimulationSubSteps = math.max(1, m_SimulationSubSteps);
            m_SimulationWorkers = math.max(1, m_SimulationWorkers);
            m_BounceThreshold = math.max(0f, m_BounceThreshold);
            m_ContactHitEventThreshold = math.max(0f, m_ContactHitEventThreshold);
            m_ContactFrequency = math.max(0f, m_ContactFrequency);
            m_ContactDamping = math.max(0f, m_ContactDamping);
            m_ContactSpeed = math.max(0f, m_ContactSpeed);
            m_ContactRecycleDistance = math.max(0f, m_ContactRecycleDistance);
            m_MaximumLinearSpeed = math.max(0f, m_MaximumLinearSpeed);
        }
    }
}
