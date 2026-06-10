using Unity.Entities;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="PointEffector2D"/> into a <see cref="PhysicsEffector2D"/> of kind
    /// <see cref="PhysicsEffector2DKind.Point"/>: a radial force toward (negative magnitude) or away from
    /// (positive) the effector's point, scaled by the <see cref="EffectorForceMode2D"/> falloff over
    /// distance × <c>distanceScale</c>. The effector GameObject's own (trigger) <c>Collider2D</c> bakes to a
    /// sensor <c>PhysicsShape2D</c> (the region) via the existing collider baker; this baker only ADDS the
    /// effector definition.
    /// </summary>
    /// <remarks>
    /// <c>forceMode</c> (Constant = 0 / InverseLinear = 1 / InverseSquared = 2) and <c>forceSource</c>
    /// (<c>EffectorSelection2D</c>: Rigidbody = the effector body centre of mass, Collider = the effector
    /// collider centroid) bake to flag/numeric bytes. Editor-only assembly.
    /// </remarks>
    public sealed class PointEffector2DBaker : Baker<PointEffector2D>
    {
        public override void Bake(PointEffector2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(
                entity,
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Point,
                    colliderMask = Effector2DBaking.ReadMask(authoring),
                    forceMagnitude = authoring.forceMagnitude,
                    forceVariation = authoring.forceVariation,
                    linearDamping = authoring.linearDamping,
                    angularDamping = authoring.angularDamping,
                    distanceScale = authoring.distanceScale,
                    forceMode = (byte)authoring.forceMode,
                    forceSourceIsRigidbody = (byte)(
                        authoring.forceSource == EffectorSelection2D.Rigidbody ? 1 : 0
                    ),
                }
            );
        }
    }
}
