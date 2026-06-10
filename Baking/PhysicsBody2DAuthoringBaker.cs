using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes the customisable <see cref="PhysicsBody2DAuthoring"/> into the SAME runtime components the
    /// built-in <see cref="Rigidbody2DBaker"/> produces — a <see cref="PhysicsBody2DDefinition"/> and, when
    /// a non-zero velocity is authored, a <see cref="PhysicsBody2DInitialVelocity"/>. This convergence is
    /// the dual-surface design property: a body authored via the custom MonoBehaviour and a body authored
    /// via a built-in <c>Rigidbody2D</c> land in one runtime archetype and run one Box2D solver, so the
    /// step + write-back systems never ask which surface created the body.
    /// </summary>
    /// <remarks>
    /// Keys on the custom authoring component exactly as the DOTS sample's <c>PhysicsBodyAuthoringBaker</c>
    /// keys on <c>PhysicsBodyAuthoring</c>. The initial pose is read from the GameObject <c>Transform</c>
    /// (Z Euler angle for the planar rotation), matching <see cref="Rigidbody2DBaker"/>. Editor-only
    /// assembly, so it never reaches a player build.
    /// </remarks>
    public sealed class PhysicsBody2DAuthoringBaker : Baker<PhysicsBody2DAuthoring>
    {
        public override void Bake(PhysicsBody2DAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var t = GetComponent<Transform>();

            AddComponent(
                entity,
                new PhysicsBody2DDefinition
                {
                    bodyType = authoring.BodyType switch
                    {
                        PhysicsBody2DMotionType.Dynamic => PhysicsBody.BodyType.Dynamic,
                        PhysicsBody2DMotionType.Kinematic => PhysicsBody.BodyType.Kinematic,
                        _ => PhysicsBody.BodyType.Static,
                    },
                    gravityScale = authoring.GravityScale,
                    linearDamping = authoring.LinearDamping,
                    angularDamping = authoring.AngularDamping,
                    initialPosition = ((float3)t.position).xy,
                    initialRotationRadians = radians(t.eulerAngles.z),
                    constraints = MapConstraints(authoring),
                    mass = authoring.Mass,
                    useAutoMass = authoring.UseAutoMass,
                    // Render-rate smoothing (Rigidbody2D.interpolation), mapped 1:1 to the runtime enum exactly
                    // as Rigidbody2DBaker does; the creation system adds the PhysicsBody2DSmoothing component for
                    // a non-None mode. None (the default) carries no smoothing, matching the prior custom-baker
                    // behaviour (which never set the field, defaulting it to None).
                    interpolation = authoring.Interpolation,
                    // Continuous collision detection (Rigidbody2D.collisionDetectionMode). Continuous → the Box2D
                    // fast-collision (bullet) body flag. Discrete (the default) bakes false, the prior value.
                    fastCollisions =
                        authoring.CollisionDetection == PhysicsCollisionDetection2D.Continuous,
                    // Custom mass distribution: the explicit center-of-mass + scalar rotational-inertia override
                    // applied post-creation via PhysicsBody.massConfiguration (ApplyMass). Off by default → the
                    // body keeps its shape-derived mass distribution, the built-in path's behaviour.
                    overrideMassDistribution = authoring.OverrideMassDistribution,
                    centerOfMass = authoring.CenterOfMass,
                    rotationalInertia = authoring.RotationalInertia,
                }
            );

            // The velocity seed is a first-class authored field here (unlike Rigidbody2D, whose
            // linearVelocity is runtime-only), so it is baked directly into PhysicsBody2DInitialVelocity —
            // the same component InitialVelocity2DBaker emits for the built-in surface, which the creation
            // system folds into the body definition. Only emitted when non-zero so a still body carries no
            // extra component.
            if (authoring.HasInitialVelocity)
                AddComponent(
                    entity,
                    new PhysicsBody2DInitialVelocity
                    {
                        linearVelocity = authoring.InitialLinearVelocity,
                        angularVelocity = authoring.InitialAngularVelocity,
                    }
                );

            // Carry the entity scale to graphics, matching the built-in Rigidbody2D path: the shape geometry
            // is baked at this scale (PhysicsShape2DAuthoringBaker), so the body is unit scale and the
            // write-back re-applies the scale to LocalToWorld.
            AddComponent(
                entity,
                new PhysicsBody2DRenderScale { value = Collider2DBaking.ReadScale(t) }
            );
        }

        /// <summary>
        /// Fold the three per-DOF freeze toggles into the Box2D <c>PhysicsBody.BodyConstraints</c> flags,
        /// the same mapping <see cref="Rigidbody2DBaker"/> applies to <c>RigidbodyConstraints2D</c>.
        /// </summary>
        static PhysicsBody.BodyConstraints MapConstraints(PhysicsBody2DAuthoring a)
        {
            var result = PhysicsBody.BodyConstraints.None;
            if (a.FreezePositionX)
                result |= PhysicsBody.BodyConstraints.PositionX;
            if (a.FreezePositionY)
                result |= PhysicsBody.BodyConstraints.PositionY;
            if (a.FreezeRotation)
                result |= PhysicsBody.BodyConstraints.Rotation;
            return result;
        }
    }
}
