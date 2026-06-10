using Unity.Entities;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="BuoyancyEffector2D"/> into a <see cref="PhysicsEffector2D"/> of kind
    /// <see cref="PhysicsEffector2DKind.Buoyancy"/>: a fluid volume that pushes a submerged body up with a force
    /// scaled by how submerged it is, plus fluid drag and an optional flow. The effector GameObject's own
    /// (trigger) <c>Collider2D</c> bakes to a sensor <c>PhysicsShape2D</c> (the fluid region) via the existing
    /// collider baker; this baker only ADDS the effector definition.
    /// </summary>
    /// <remarks>
    /// <c>flowAngle</c> bakes degrees → radians; the world gravity magnitude is baked from
    /// <c>Physics2D.gravity</c> at bake time (a project-constant read) so the buoyant force balances against
    /// gravity without a runtime <c>UnityEngine.Physics2D</c> call. Editor-only assembly.
    /// </remarks>
    public sealed class BuoyancyEffector2DBaker : Baker<BuoyancyEffector2D>
    {
        public override void Bake(BuoyancyEffector2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(
                entity,
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Buoyancy,
                    colliderMask = Effector2DBaking.ReadMask(authoring),
                    surfaceLevel = authoring.surfaceLevel,
                    fluidDensity = authoring.density,
                    linearDamping = authoring.linearDamping,
                    angularDamping = authoring.angularDamping,
                    flowMagnitude = authoring.flowMagnitude,
                    flowVariation = authoring.flowVariation,
                    flowAngleRadians = radians(authoring.flowAngle),
                    gravityMagnitude = Effector2DBaking.GravityMagnitude(),
                }
            );
        }
    }
}
