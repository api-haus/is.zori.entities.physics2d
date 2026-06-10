using Unity.Entities;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="AreaEffector2D"/> into a <see cref="PhysicsEffector2D"/> of kind
    /// <see cref="PhysicsEffector2DKind.Area"/>: a directional force zone with linear/angular drag. The effector
    /// GameObject's own (trigger) <c>Collider2D</c> bakes to a sensor <c>PhysicsShape2D</c> via the existing
    /// collider baker, and its collider-only static-body fallback gives the effector entity a static body — so
    /// this baker only ADDS the effector definition; the region geometry comes from the baked sensor shape.
    /// </summary>
    /// <remarks>
    /// <c>forceAngle</c> bakes degrees → radians; <c>useGlobalAngle</c> and <c>forceTarget</c>
    /// (<c>EffectorSelection2D</c>: Rigidbody = at the body centre of mass, Collider = at the collider centroid)
    /// bake to flag bytes. Editor-only assembly, so this never reaches a player build.
    /// </remarks>
    public sealed class AreaEffector2DBaker : Baker<AreaEffector2D>
    {
        public override void Bake(AreaEffector2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(
                entity,
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Area,
                    colliderMask = Effector2DBaking.ReadMask(authoring),
                    forceMagnitude = authoring.forceMagnitude,
                    forceVariation = authoring.forceVariation,
                    linearDamping = authoring.linearDamping,
                    angularDamping = authoring.angularDamping,
                    forceAngleRadians = radians(authoring.forceAngle),
                    useGlobalAngle = (byte)(authoring.useGlobalAngle ? 1 : 0),
                    forceTargetIsRigidbody = (byte)(
                        authoring.forceTarget == EffectorSelection2D.Rigidbody ? 1 : 0
                    ),
                }
            );
        }
    }
}
