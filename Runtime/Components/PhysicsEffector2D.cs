using Unity.Entities;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Which force-field effector a <see cref="PhysicsEffector2D"/> carries — the 2D analogue of the built-in
    /// <c>AreaEffector2D</c> / <c>BuoyancyEffector2D</c> / <c>PointEffector2D</c>. Box2D-v3 has no native
    /// effector; the package reproduces each by overlap-querying the effector's (sensor) collider region each
    /// step and applying the GameObject force formula to every dynamic body inside, before <c>Simulate</c>.
    /// </summary>
    public enum PhysicsEffector2DKind : byte
    {
        /// <summary><c>AreaEffector2D</c> — a directional force zone with drag (wind / thrust volume).</summary>
        Area,

        /// <summary><c>BuoyancyEffector2D</c> — a fluid volume: submerged bodies get upward buoyancy + fluid drag + flow.</summary>
        Buoyancy,

        /// <summary><c>PointEffector2D</c> — a radial attract/repel force from a point (gravity well / explosion).</summary>
        Point,

        /// <summary><c>PlatformEffector2D</c> — a SOLID one-way platform: a body within the surface arc collides
        /// and rests, a body from outside it (below / the sides) passes through. Unlike the three force-field
        /// kinds above (which sit on a SENSOR region), this gates collision on a SOLID collider.</summary>
        Platform,

        /// <summary><c>SurfaceEffector2D</c> — a SOLID conveyor belt: a body in contact with the surface is driven
        /// tangentially toward the belt speed. Also a SOLID collider (the body rests ON the belt).</summary>
        Surface,
    }

    /// <summary>
    /// The baked force-field effector definition, a tagged union over <see cref="PhysicsEffector2DKind"/>. Rides
    /// on the effector GameObject's own entity alongside its baked sensor <c>PhysicsShape2D</c> (the effector's
    /// trigger collider) and its collider-only static <c>PhysicsBody2DDefinition</c>. Each step,
    /// <c>PhysicsWorld2DSystem</c> reads the effector's region from its baked shape, overlap-queries the world
    /// (honoring <see cref="colliderMask"/>) for the bodies inside it, and applies the per-kind force formula to
    /// each dynamic body before <c>Simulate</c> — the same pre-step force-accumulation window the runtime
    /// write-in (<c>PhysicsBody2DCommand</c>) drains in, so an effector force is mass-scaled and
    /// frozen-axis-cancelled by the solver exactly like a <c>Rigidbody2D.AddForce(_, Force)</c>.
    /// </summary>
    /// <remarks>
    /// One struct (not three per-kind components): the three effectors share the region mask + force + drag
    /// fields and differ only in a handful, so a per-kind component would triple the archetype count and force
    /// the apply to branch on component presence rather than a field — the same union shape
    /// <see cref="PhysicsShape2D"/> and <c>PhysicsJoint2DDefinition</c> use. The unused fields for a given kind
    /// are a few dead floats per effector entity (there are few effectors), cheaper than three components.
    ///
    /// <para><b>Drag is a velocity multiplier</b> (<c>v *= 1/(1 + damping·dt)</c>), the standard Unity/Box2D
    /// damping integrator the package already bakes for a body's own damping — NOT a force added to the
    /// accumulator.</para>
    /// </remarks>
    public struct PhysicsEffector2D : IComponentData
    {
        /// <summary>The discriminant. Decides which of the fields below are live.</summary>
        public PhysicsEffector2DKind kind;

        /// <summary>
        /// The layer mask the region overlap honors, as a raw 64-bit value (the <c>PhysicsMask</c> bit
        /// convention). Baked from <c>Effector2D.useColliderMask ? colliderMask : GetLayerCollisionMask(layer)</c>
        /// — the effector's explicit collider mask when it opts into one, else the effector collider's own
        /// project layer-matrix row (the global-matrix fallback the field documents). Passed as the overlap
        /// query's <c>hitLayerMask</c>, so a body on an off-mask layer is not returned and gets no force.
        /// </summary>
        public ulong colliderMask;

        // ---- Area / Point shared: force magnitude + drag --------------------------------------------------

        /// <summary>Area/Point: <c>forceMagnitude</c>. Point: negative attracts toward the point, positive repels.</summary>
        public float forceMagnitude;

        /// <summary>Area/Point: <c>forceVariation</c> — a random in [-variation, +variation] added per application.
        /// Zero (the deterministic, parity-asserted path) for every example scene; non-zero is a
        /// non-deterministic feature.</summary>
        public float forceVariation;

        /// <summary>The per-body extra linear drag the effector imposes while a body is inside, as a velocity
        /// multiplier <c>v *= 1/(1 + linearDamping·dt)</c> (<c>Effector2D.linearDamping</c>).</summary>
        public float linearDamping;

        /// <summary>The per-body extra angular drag, as an angular-velocity multiplier
        /// (<c>Effector2D.angularDamping</c>).</summary>
        public float angularDamping;

        // ---- Area ----------------------------------------------------------------------------------------

        /// <summary>Area: <c>forceAngle</c> baked to radians — the direction of the zone force.</summary>
        public float forceAngleRadians;

        /// <summary>Area: <c>useGlobalAngle</c> as a flag — 1 = the angle is world-space, 0 = relative to the
        /// effector body's rotation (a rotated wind volume blows along its local axis).</summary>
        public byte useGlobalAngle;

        /// <summary>Area: <c>forceTarget</c> (<c>EffectorSelection2D</c>) as a flag — 1 = Rigidbody (force at the
        /// body centre of mass, pure linear), 0 = Collider (force at the collider centroid, may add torque).</summary>
        public byte forceTargetIsRigidbody;

        // ---- Buoyancy ------------------------------------------------------------------------------------

        /// <summary>Buoyancy: <c>surfaceLevel</c> — the world-space horizontal fluid surface. A body below it is
        /// (partly) submerged.</summary>
        public float surfaceLevel;

        /// <summary>Buoyancy: <c>density</c> — the fluid density. A body of effective density <c>d</c> floats with
        /// fraction <c>d/fluidDensity</c> submerged (a unit-density body in density-2 fluid rests half-submerged).</summary>
        public float fluidDensity;

        /// <summary>Buoyancy: <c>flowMagnitude</c> — a constant fluid-flow force on the submerged part.</summary>
        public float flowMagnitude;

        /// <summary>Buoyancy: <c>flowVariation</c> — random variation of the flow force (zero in every scene).</summary>
        public float flowVariation;

        /// <summary>Buoyancy: <c>flowAngle</c> baked to radians (world-space) — the fluid-flow direction.</summary>
        public float flowAngleRadians;

        /// <summary>Buoyancy: <c>|Physics2D.gravity|</c> baked at bake time (a project-constant read, so the
        /// runtime never calls the shadowed/non-Burst <c>UnityEngine.Physics2D</c>). The buoyant force balances
        /// against this so a submerged body floats up rather than sinking.</summary>
        public float gravityMagnitude;

        // ---- Point ---------------------------------------------------------------------------------------

        /// <summary>Point: <c>distanceScale</c> — scales the source→target distance used in the falloff.</summary>
        public float distanceScale;

        /// <summary>Point: <c>forceMode</c> (<c>EffectorForceMode2D</c>) — 0 = Constant, 1 = InverseLinear
        /// (force/d), 2 = InverseSquared (force/d²).</summary>
        public byte forceMode;

        /// <summary>Point: <c>forceSource</c> (<c>EffectorSelection2D</c>) as a flag — 1 = Rigidbody (the source
        /// point is the effector body centre of mass), 0 = Collider (the effector collider centroid).</summary>
        public byte forceSourceIsRigidbody;

        // ---- Platform ------------------------------------------------------------------------------------

        /// <summary>Platform: <c>surfaceArc</c> baked to radians — the angular window, centred on the platform's
        /// local up, within which a contact COLLIDES (the body rests on the platform). A contact whose direction
        /// from the platform lies outside this arc PASSES THROUGH. <c>180°</c> (the OneWay scene) is the canonical
        /// top-facing platform: the whole upper half-plane collides, the lower half passes.</summary>
        public float surfaceArcRadians;

        /// <summary>Platform: <c>rotationalOffset</c> baked to radians — the offset of the surface-arc centre from
        /// the platform's local up. Added to the platform body's own rotation to form the world up reference.</summary>
        public float rotationalOffsetRadians;

        /// <summary>Platform: <c>useOneWay</c> as a flag — 1 = the one-way gating is active, 0 = the platform is a
        /// plain solid collider (no gating). The package gates the platform BODY's participation each step from the
        /// surface arc (an approximation; Box2D-v3's faithful per-contact pre-solve veto is unreachable from the
        /// package's native-poll posture — see the Phase-10b design's negative-space writeup).</summary>
        public byte useOneWay;

        // ---- Surface -------------------------------------------------------------------------------------

        /// <summary>Surface: <c>speed</c> — the tangential belt speed (along the surface's local +X) the conveyor
        /// drives contacting bodies toward. Negative drives the other way.</summary>
        public float surfaceSpeed;

        /// <summary>Surface: <c>speedVariation</c> — a random in [0, variation] added to the belt speed per
        /// application (zero in every example scene; non-zero is non-deterministic).</summary>
        public float surfaceSpeedVariation;

        /// <summary>Surface: <c>forceScale</c> — scales the per-step velocity-error impulse toward the belt speed.
        /// 1 closes the tangential-velocity error in one step (the default); smaller converges gradually.</summary>
        public float forceScale;

        /// <summary>Surface: <c>useContactForce</c> as a flag — 1 = apply the belt impulse at the contact point
        /// (an off-centre contact then also adds torque), 0 = apply it at the body centre of mass (pure linear).</summary>
        public byte useContactForce;

        /// <summary>Surface: <c>useFriction</c> as a flag — whether the surface contact uses friction. The
        /// collider's <c>PhysicsMaterial2D.friction</c> is already baked onto the shape surface material (Phase 1A),
        /// so a friction surface already has it; this flag is carried for parity-gate assertion and does not
        /// re-apply friction.</summary>
        public byte surfaceUseFriction;
    }
}
