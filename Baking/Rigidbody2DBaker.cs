using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="Rigidbody2D"/> into a <see cref="PhysicsBody2DDefinition"/>, keying on
    /// the engine component exactly as <c>com.unity.physics</c>'s <c>RigidbodyBaker</c> keys on the 3D
    /// <c>Rigidbody</c>. The initial pose is read from the GameObject's <c>Transform</c>; a 2D body's
    /// rotation is the Z Euler angle (planar physics ignores X/Y rotation).
    /// </summary>
    /// <remarks>
    /// Requests <c>GetEntity(TransformUsageFlags.Dynamic)</c>, which gives the baked entity a
    /// <c>LocalToWorld</c> the write-back system targets. A GameObject carrying both a
    /// <see cref="Rigidbody2D"/> and a <c>CircleCollider2D</c> bakes to one entity with both
    /// <see cref="PhysicsBody2DDefinition"/> and <c>PhysicsShape2D</c> — the natural ECS shape of
    /// "one body, one shape." Editor-only assembly, so this baking code never reaches a player build.
    /// </remarks>
    public sealed class Rigidbody2DBaker : Baker<Rigidbody2D>
    {
        public override void Bake(Rigidbody2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var t = GetComponent<Transform>();

            AddComponent(
                entity,
                new PhysicsBody2DDefinition
                {
                    bodyType = authoring.bodyType switch
                    {
                        RigidbodyType2D.Dynamic => PhysicsBody.BodyType.Dynamic,
                        RigidbodyType2D.Kinematic => PhysicsBody.BodyType.Kinematic,
                        _ => PhysicsBody.BodyType.Static,
                    },
                    gravityScale = authoring.gravityScale,
                    linearDamping = authoring.linearDamping,
                    angularDamping = authoring.angularDamping,
                    initialPosition = ((float3)t.position).xy,
                    initialRotationRadians = radians(t.eulerAngles.z),
                    // Velocity is NOT seeded here: Rigidbody2D.linearVelocity/angularVelocity are runtime-only
                    // and bake to zero from a saved scene (the field is not serialized). The serialized seed
                    // lives on InitialVelocity2DAuthoring and is baked by InitialVelocity2DBaker into a separate
                    // PhysicsBody2DInitialVelocity component the creation system folds in.
                    // Constraints: the built-in RigidbodyConstraints2D flags map onto Box2D BodyConstraints.
                    constraints = MapConstraints(authoring.constraints),
                    // Mass: Rigidbody2D.useAutoMass false → explicit mass; true → density-derived from shapes.
                    mass = authoring.mass,
                    useAutoMass = authoring.useAutoMass,
                    // CCD: collisionDetectionMode Continuous → the Box2D fast-collision (bullet) body flag, so a
                    // fast body does not tunnel a thin collider; Discrete (and the deprecated None) → false.
                    fastCollisions = authoring.collisionDetectionMode == CollisionDetectionMode2D.Continuous,
                    // Interpolation: Rigidbody2D.interpolation → the render-rate smoothing mode. None → no
                    // smoothing (fixed-rate LocalToWorld); Interpolate/Extrapolate → smoothed between steps.
                    interpolation = authoring.interpolation switch
                    {
                        RigidbodyInterpolation2D.Interpolate => PhysicsBody2DInterpolation.Interpolate,
                        RigidbodyInterpolation2D.Extrapolate => PhysicsBody2DInterpolation.Extrapolate,
                        _ => PhysicsBody2DInterpolation.None,
                    },
                }
            );

            // Carry the entity's transform scale to graphics. The collider geometry is baked at this scale
            // (so the Box2D body is unit scale), so the write-back must re-apply the scale to LocalToWorld or
            // the rendered sprite loses it. Read from lossyScale, matching the collider bakers' shape scale.
            AddComponent(
                entity,
                new Zori.Entities.Physics2D.PhysicsBody2DRenderScale { value = Collider2DBaking.ReadScale(t) }
            );
        }

        /// <summary>
        /// Map the built-in <c>RigidbodyConstraints2D</c> freeze flags onto Box2D's
        /// <c>PhysicsBody.BodyConstraints</c> flags enum: FreezePositionX → PositionX, FreezePositionY →
        /// PositionY, FreezeRotation → Rotation. The two are independent flag sets, so this is a per-flag
        /// OR-fold rather than a cast.
        /// </summary>
        static PhysicsBody.BodyConstraints MapConstraints(RigidbodyConstraints2D c)
        {
            var result = PhysicsBody.BodyConstraints.None;
            if ((c & RigidbodyConstraints2D.FreezePositionX) != 0)
                result |= PhysicsBody.BodyConstraints.PositionX;
            if ((c & RigidbodyConstraints2D.FreezePositionY) != 0)
                result |= PhysicsBody.BodyConstraints.PositionY;
            if ((c & RigidbodyConstraints2D.FreezeRotation) != 0)
                result |= PhysicsBody.BodyConstraints.Rotation;
            return result;
        }
    }
}
