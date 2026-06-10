using UnityEngine;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// A serialized initial-velocity seed for a body, the single-authoring way to give a fixture's body a
    /// starting velocity. <c>Rigidbody2D.linearVelocity</c>/<c>angularVelocity</c> are <em>runtime-only</em>
    /// state — they are not serialized into a scene, so a velocity set on a <c>Rigidbody2D</c> in an editor
    /// fixture builder is discarded on save and bakes to zero. This MonoBehaviour carries the seed as
    /// serialized fields instead, so one authored source feeds both backends: the package's
    /// <c>InitialVelocity2DBaker</c> writes it into <see cref="PhysicsBody2DDefinition"/>, and the parity
    /// harness applies the same component's values to the live reference <c>Rigidbody2D</c>. This mirrors the
    /// <c>OldPhysics2D</c> samples, which seed velocity through an <c>InitialVelocity</c> MonoBehaviour for the
    /// same reason.
    /// </summary>
    [RequireComponent(typeof(Rigidbody2D))]
    public sealed class InitialVelocity2DAuthoring : MonoBehaviour
    {
        /// <summary>Initial linear velocity of the body's origin, m/s.</summary>
        public Vector2 linearVelocity;

        /// <summary>Initial angular velocity, degrees/sec (the <c>Rigidbody2D.angularVelocity</c> unit).</summary>
        public float angularVelocity;
    }
}
