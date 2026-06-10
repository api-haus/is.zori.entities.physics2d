using Unity.Entities;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="SurfaceEffector2D"/> into a <see cref="PhysicsEffector2D"/> of kind
    /// <see cref="PhysicsEffector2DKind.Surface"/>: a conveyor belt. Like the platform, the surface GameObject's
    /// own <c>Collider2D</c> is SOLID (<c>m_IsTrigger: 0</c> in the example scenes) — a body rests ON the belt and
    /// is driven tangentially toward the belt speed. The existing collider baker bakes the solid shape and the
    /// collider-only static-body fallback gives the effector entity a static body; this baker only ADDS the
    /// conveyor definition.
    /// </summary>
    /// <remarks>
    /// <c>speed</c> / <c>speedVariation</c> / <c>forceScale</c> pass through; <c>useContactForce</c> and
    /// <c>useFriction</c> bake to flag bytes. The drive is a tangential velocity-error impulse (the runtime
    /// converges a contacting body's tangential velocity to the belt speed without overshoot). Editor-only
    /// assembly.
    /// </remarks>
    public sealed class SurfaceEffector2DBaker : Baker<SurfaceEffector2D>
    {
        public override void Bake(SurfaceEffector2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(
                entity,
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Surface,
                    colliderMask = Effector2DBaking.ReadMask(authoring),
                    surfaceSpeed = authoring.speed,
                    surfaceSpeedVariation = authoring.speedVariation,
                    forceScale = authoring.forceScale,
                    useContactForce = (byte)(authoring.useContactForce ? 1 : 0),
                    surfaceUseFriction = (byte)(authoring.useFriction ? 1 : 0),
                }
            );
        }
    }
}
