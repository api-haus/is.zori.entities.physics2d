using Unity.Entities;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="PlatformEffector2D"/> into a <see cref="PhysicsEffector2D"/> of kind
    /// <see cref="PhysicsEffector2DKind.Platform"/>: a one-way platform. Unlike the force-field effectors, the
    /// platform GameObject's own <c>Collider2D</c> is SOLID (<c>m_IsTrigger: 0</c> in the example scenes), so the
    /// existing collider baker bakes it to a NON-sensor <c>PhysicsShape2D</c> and the effector entity gets a
    /// solid body that bodies actually rest on — this baker only ADDS the one-way definition; the region geometry
    /// (used by the per-step gating overlap query) comes from the baked solid shape.
    /// </summary>
    /// <remarks>
    /// <c>surfaceArc</c> and <c>rotationalOffset</c> bake degrees → radians; <c>useOneWay</c> bakes to a flag
    /// byte. The side-arc / side-friction / side-bounce behaviours are NOT modelled (they need the same
    /// per-contact pre-solve hook one-way faithfully needs, which the package cannot reach — see the Phase-10b
    /// design); the OneWay scene authors them off. Editor-only assembly, so this never reaches a player build.
    /// </remarks>
    public sealed class PlatformEffector2DBaker : Baker<PlatformEffector2D>
    {
        public override void Bake(PlatformEffector2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(
                entity,
                new PhysicsEffector2D
                {
                    kind = PhysicsEffector2DKind.Platform,
                    colliderMask = Effector2DBaking.ReadMask(authoring),
                    surfaceArcRadians = radians(authoring.surfaceArc),
                    rotationalOffsetRadians = radians(authoring.rotationalOffset),
                    useOneWay = (byte)(authoring.useOneWay ? 1 : 0),
                }
            );
        }
    }
}
