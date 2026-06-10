using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes the serialized <see cref="InitialVelocity2DAuthoring"/> seed into a
    /// <see cref="PhysicsBody2DInitialVelocity"/> component the creation system folds into the body
    /// definition at creation. Separate from <c>Rigidbody2DBaker</c> because <c>Rigidbody2D.linearVelocity</c>
    /// is runtime-only and bakes to zero from a saved scene — the serialized seed must come from a dedicated
    /// authoring component, and a body without one simply carries no velocity component (treated as zero).
    /// </summary>
    public sealed class InitialVelocity2DBaker : Baker<InitialVelocity2DAuthoring>
    {
        public override void Bake(InitialVelocity2DAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(
                entity,
                new PhysicsBody2DInitialVelocity
                {
                    linearVelocity = (float2)authoring.linearVelocity,
                    angularVelocity = authoring.angularVelocity,
                }
            );
        }
    }
}
