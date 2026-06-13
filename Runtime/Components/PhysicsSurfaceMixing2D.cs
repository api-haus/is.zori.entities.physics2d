namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// How two shapes' friction or bounciness values are mixed to form a contact, the package-local mirror of
    /// <c>Unity.U2D.Physics.PhysicsShape.SurfaceMaterial.MixingMode</c> (module XML
    /// <c>T:…PhysicsShape.SurfaceMaterial.MixingMode</c>). It is the 2D combine policy — the 2D analogue of 3D
    /// <c>com.unity.physics</c>'s <c>Material.CombinePolicy</c>, and a strict superset of the built-in
    /// <c>UnityEngine.PhysicsMaterialCombine2D</c> (which lacks <see cref="Mean"/>). Carried package-local (not
    /// the engine enum) so neither the Runtime nor the Authoring assembly references <c>Unity.U2D.Physics</c>;
    /// the baker and the creation system map it to the engine enum where they already reference it.
    /// </summary>
    /// <remarks>
    /// The five values mirror the engine 1:1 in declaration order: <see cref="Average"/> (the arithmetic mean
    /// of both surface values), <see cref="Maximum"/>, <see cref="Mean"/> (the geometric mean), <see
    /// cref="Minimum"/>, <see cref="Multiply"/> (the product). The mixing actually used at a contact is decided
    /// by the higher-priority shape's mixing, or the higher enumeration value when priorities tie (XML
    /// <c>P:…PhysicsShape.SurfaceMaterial.frictionPriority</c>); priorities are not authored (no authoring
    /// surface exposes them), so a pair's effective mixing is the higher of the two shapes' modes.
    /// </remarks>
    public enum PhysicsSurfaceMixing2D : byte
    {
        /// <summary>The arithmetic average of both surface values.</summary>
        Average,

        /// <summary>The maximum of both surface values.</summary>
        Maximum,

        /// <summary>The geometric mean of both surface values.</summary>
        Mean,

        /// <summary>The minimum of both surface values.</summary>
        Minimum,

        /// <summary>The product of both surface values.</summary>
        Multiply,
    }

    /// <summary>
    /// How a body performs continuous collision detection, the package-local mirror of
    /// <c>UnityEngine.CollisionDetectionMode2D</c>. <see cref="Continuous"/> maps to the Box2D fast-collision
    /// (bullet) body flag (<c>PhysicsBodyDefinition.fastCollisionsAllowed</c>), so a fast body does not tunnel a
    /// thin collider in one step; <see cref="Discrete"/> leaves it off (the engine default). Dynamic-vs-Static
    /// CCD is the world-level <c>continuousAllowed</c> regardless of this, so a Discrete body still does not
    /// tunnel a static wall; a Continuous body additionally does not tunnel a fast Dynamic/Kinematic body.
    /// </summary>
    public enum PhysicsCollisionDetection2D : byte
    {
        /// <summary>No continuous collision against Dynamic/Kinematic bodies (the engine default).</summary>
        Discrete,

        /// <summary>Continuous collision against Dynamic/Kinematic bodies (the Box2D fast-collision flag).</summary>
        Continuous,
    }

    /// <summary>
    /// How a shape responds to overlaps — the 2D-expressible subset of 3D
    /// <c>com.unity.physics</c>'s <c>CollisionResponsePolicy</c>. 2D expresses two of the 3D four ways at the
    /// bake layer: <see cref="Collide"/> (a solid shape with a collision response) and <see cref="Sensor"/> (a
    /// trigger shape that overlaps without a collision response, <c>Collider2D.isTrigger</c>). Contact and
    /// trigger events are always enabled on every package shape (the always-on Enter/Stay/Exit posture), so the
    /// 3D <c>None</c> (no-event sensor) and an explicit per-shape contact-event opt-in are not expressible here
    /// without new runtime surface — they are deliberately absent in 2D (<c>bake-contract.md</c>).
    /// </summary>
    public enum PhysicsCollisionResponse2D : byte
    {
        /// <summary>A solid shape: it collides with a physical response (<c>Collider2D.isTrigger = false</c>).</summary>
        Collide,

        /// <summary>A sensor (trigger): it overlaps and reports trigger events but never produces a collision
        /// response, and does not detect other sensors as solid (<c>Collider2D.isTrigger = true</c>).</summary>
        Sensor,
    }
}
