using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// How a body's RENDERED pose is smoothed between fixed physics steps, mirroring
    /// <c>UnityEngine.RigidbodyInterpolation2D</c> 1:1. The package writes the authoritative pose into
    /// <c>LocalToWorld</c> at the fixed step rate; a render-rate smoothing system
    /// (<c>PhysicsBody2DSmoothingSystem</c>) overwrites <c>LocalToWorld</c> between steps for a body whose
    /// mode is not <see cref="None"/>. This never re-enables the Box2D managed-Transform tween — the smoothing
    /// is done in ECS over the stored previous/current physics poses.
    /// </summary>
    public enum PhysicsBody2DInterpolation : byte
    {
        /// <summary>No smoothing — the rendered pose is the fixed-step pose (choppy under fast motion at a high
        /// frame rate). The built-in default.</summary>
        None,

        /// <summary>Show the pose interpolated between the PREVIOUS and CURRENT physics states (one step of
        /// render lag), matching <c>RigidbodyInterpolation2D.Interpolate</c>.</summary>
        Interpolate,

        /// <summary>Predict the pose ahead of the current physics state using the body's velocity, matching
        /// <c>RigidbodyInterpolation2D.Extrapolate</c>.</summary>
        Extrapolate,
    }

    /// <summary>
    /// The baked body authoring, consumed exactly once when the runtime creates the body. Built from a
    /// <c>UnityEngine.Rigidbody2D</c> by <c>Rigidbody2DBaker</c> (or defaulted to a static body for a
    /// collider-only GameObject, which the vertical slice does not exercise).
    /// </summary>
    /// <remarks>
    /// All fields are blittable: <see cref="PhysicsBody.BodyType"/> is an enum value type. The pose
    /// fields are captured from the GameObject's <c>Transform</c> at bake time and feed
    /// <c>PhysicsBodyDefinition.position</c>/<c>.rotation</c> at creation — Box2D recommends creating a
    /// body at its final pose rather than moving it after shapes are attached.
    /// </remarks>
    public struct PhysicsBody2DDefinition : IComponentData
    {
        public PhysicsBody.BodyType bodyType;
        public float gravityScale;
        public float linearDamping;
        public float angularDamping;
        public float2 initialPosition;

        /// <summary>
        /// The body's initial 2D rotation in <b>radians</b>, baked from the GameObject's
        /// <c>Transform.eulerAngles.z</c> (degrees) via <c>math.radians</c> and fed to
        /// <c>PhysicsBodyDefinition.rotation</c> through the unit-agnostic <c>PhysicsRotate.FromRadians</c> at
        /// creation. Radians is the package's rotation-angle convention (the angular-unit section of
        /// <c>Documentation~/parity-matrix.md</c>).
        /// </summary>
        public float initialRotationRadians;

        /// <summary>
        /// <c>Rigidbody2D.constraints</c> mapped to <c>PhysicsBody.BodyConstraints</c> (a flags enum: PositionX
        /// / PositionY / Rotation, XML <c>T:…PhysicsBody.BodyConstraints</c>). <c>FreezePositionX</c> →
        /// <c>PositionX</c>, <c>FreezePositionY</c> → <c>PositionY</c>, <c>FreezeRotation</c> → <c>Rotation</c>.
        /// Stored as the resolved Box2D flags so the creation system writes it directly.
        /// </summary>
        public PhysicsBody.BodyConstraints constraints;

        /// <summary>
        /// Explicit body mass from <c>Rigidbody2D.mass</c>, applied via the body's
        /// <see cref="PhysicsBody.MassConfiguration"/> after shapes are created (XML
        /// <c>P:…PhysicsBody.massConfiguration</c>). Only honoured when <see cref="useAutoMass"/> is false; when
        /// true the body keeps the density-derived mass Box2D computes from its shapes (the
        /// <c>Rigidbody2D.useAutoMass</c> semantics). Ignored for non-dynamic bodies.
        /// </summary>
        public float mass;

        /// <summary>
        /// <c>Rigidbody2D.useAutoMass</c> — when true the body's mass comes from its shapes' density
        /// (Box2D's automatic mass-from-shapes), when false the explicit <see cref="mass"/> is applied. The
        /// built-in default is false (explicit mass).
        /// </summary>
        public bool useAutoMass;

        /// <summary>
        /// The resolved Box2D continuous-collision flag from <c>Rigidbody2D.collisionDetectionMode</c>:
        /// <c>Continuous</c> bakes true, <c>Discrete</c> bakes false. Set into
        /// <c>PhysicsBodyDefinition.fastCollisionsAllowed</c> at creation (XML
        /// <c>P:…PhysicsBodyDefinition.fastCollisionsAllowed</c>): a "fast" body performs continuous collision
        /// detection against Dynamic and Kinematic bodies, so it does not tunnel through them in one step.
        /// Dynamic-vs-Static CCD is the world-level <c>continuousAllowed</c> (on by default), so a Discrete body
        /// still does not tunnel a static wall, but a Continuous body additionally does not tunnel a fast
        /// Dynamic/Kinematic body — matching GameObject <c>collisionDetectionMode = Continuous</c>.
        /// </summary>
        public bool fastCollisions;

        /// <summary>
        /// <c>Rigidbody2D.interpolation</c> mapped to <see cref="PhysicsBody2DInterpolation"/>. When not
        /// <see cref="PhysicsBody2DInterpolation.None"/>, the body carries a <c>PhysicsBody2DSmoothing</c>
        /// component and its <c>LocalToWorld</c> is smoothed between fixed steps by
        /// <c>PhysicsBody2DSmoothingSystem</c> at render rate. Default <see cref="PhysicsBody2DInterpolation.None"/>.
        /// </summary>
        public PhysicsBody2DInterpolation interpolation;

        /// <summary>
        /// When true, the dynamic body's center of mass and rotational inertia are taken from
        /// <see cref="centerOfMass"/> / <see cref="rotationalInertia"/> instead of the density-derived values
        /// Box2D computes from the shapes — the 2D analogue of the 3D custom sample's
        /// <c>OverrideDefaultMassDistribution</c>. Applied post-creation via <c>PhysicsBody.massConfiguration</c>
        /// (XML <c>P:…PhysicsBody.massConfiguration</c>, settable: "if you wish to assign your own
        /// MassConfiguration"). The <c>PhysicsBodyDefinition</c> the engine creates from has no mass field, so
        /// the override is a post-create write on the live body, not a creation-time definition value. Default
        /// false → the body keeps its shape-derived mass distribution (the built-in path, which has no override).
        /// Ignored for non-dynamic bodies.
        /// </summary>
        public bool overrideMassDistribution;

        /// <summary>
        /// The body's local-space center of mass, applied to <c>PhysicsBody.MassConfiguration.center</c> (XML
        /// <c>P:…PhysicsBody.MassConfiguration.center</c>) when <see cref="overrideMassDistribution"/> is true. A
        /// single <c>float2</c> — 2D has no inertia-tensor orientation to override (the planar moment of inertia
        /// below is a scalar). Inert when <see cref="overrideMassDistribution"/> is false.
        /// </summary>
        public float2 centerOfMass;

        /// <summary>
        /// The body's rotational inertia (kg·m², about the center of mass), applied to
        /// <c>PhysicsBody.MassConfiguration.rotationalInertia</c> (XML
        /// <c>P:…PhysicsBody.MassConfiguration.rotationalInertia</c>) when <see cref="overrideMassDistribution"/>
        /// is true AND this is &gt; 0. A 2D body has one rotational DOF, so its inertia is a single scalar (the 3D
        /// 3-DOF inertia tensor reduces to this). A value &lt;= 0 leaves the shape-derived inertia even with the
        /// override on (the author overrode only the center of mass). Inert when the override is false.
        /// </summary>
        public float rotationalInertia;
    }

    /// <summary>
    /// An optional serialized initial-velocity seed baked from <see cref="InitialVelocity2DAuthoring"/>. Kept a
    /// separate component (rather than folded into <see cref="PhysicsBody2DDefinition"/>) so the velocity
    /// authoring is independent of the body baker — a body with no <see cref="InitialVelocity2DAuthoring"/>
    /// simply never carries this, and the creation system treats its absence as zero velocity. Carried as its
    /// own component because <c>Rigidbody2D.linearVelocity</c> is runtime-only and cannot be baked from the
    /// <c>Rigidbody2D</c> itself (see <see cref="InitialVelocity2DAuthoring"/>).
    /// </summary>
    public struct PhysicsBody2DInitialVelocity : IComponentData
    {
        /// <summary>Initial linear velocity of the body's origin, in m/s.</summary>
        public float2 linearVelocity;

        /// <summary>Initial angular velocity, in <b>degrees per second</b> — the unit of both
        /// <c>Rigidbody2D.angularVelocity</c> and the engine's <c>PhysicsBodyDefinition.angularVelocity</c>
        /// (module XML), so the seed bakes and feeds deg/sec unchanged (the angular-unit convention in
        /// <c>Documentation~/parity-matrix.md</c>).</summary>
        public float angularVelocity;
    }
}
