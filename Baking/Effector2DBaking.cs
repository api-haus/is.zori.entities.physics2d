using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Shared baking helpers for the force-field effector family (Area / Buoyancy / Point). The colliderMask
    /// resolution lives here so every effector baker resolves it identically: <c>Effector2D.colliderMask</c>
    /// when <c>useColliderMask</c> is set, else the effector collider's own project layer-matrix row (the
    /// global-collision-matrix fallback the field documents).
    /// </summary>
    static class Effector2DBaking
    {
        /// <summary>
        /// Resolve the layer mask an effector's region overlap honors. When <c>useColliderMask</c> is true the
        /// explicit <c>colliderMask</c> (a 32-bit <c>LayerMask</c>) is widened to a 64-bit
        /// <c>PhysicsMask</c>-style value (upper 32 bits zero, exactly as <c>PhysicsMask.#ctor(LayerMask)</c>
        /// does); when false, the effector GameObject's own layer-matrix row
        /// (<c>Physics2D.GetLayerCollisionMask(layer)</c>) is used — the global-matrix fallback. The runtime then
        /// passes this as the overlap query's <c>hitLayerMask</c>, so only bodies on the allowed layers are
        /// returned. <c>UnityEngine.Physics2D</c> is fully qualified: the package root namespace shadows the bare
        /// "Physics2D" token.
        /// </summary>
        public static ulong ReadMask(Effector2D effector)
        {
            if (effector.useColliderMask)
                return unchecked((uint)effector.colliderMask);
            var layer = effector.gameObject.layer;
            return unchecked((uint)UnityEngine.Physics2D.GetLayerCollisionMask(layer));
        }

        /// <summary>The world gravity magnitude baked into a buoyancy effector so the buoyant force balances
        /// against it. A project-constant read at bake time (the same posture as the layer matrix), so the
        /// runtime never calls the shadowed/non-Burst <c>UnityEngine.Physics2D</c>.</summary>
        public static float GravityMagnitude() => UnityEngine.Physics2D.gravity.magnitude;
    }
}
