using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.Physics2D.Baking
{
    /// <summary>
    /// Bakes a built-in <see cref="TargetJoint2D"/> into a <see cref="PhysicsJoint2DDefinition"/> of kind
    /// <see cref="PhysicsJoint2DKind.Target"/>. A target joint drives a single body's anchor point toward a
    /// world-space target position with a spring, capped by a maximum force — the joint behind a mouse-drag
    /// grab — implemented on the Box2D <c>PhysicsRelativeJoint</c> against the static world anchor.
    /// Editor-only assembly.
    /// </summary>
    /// <remarks>
    /// <see cref="TargetJoint2D"/> is a SINGLE-body joint: it has no <c>connectedBody</c> (it pulls toward a
    /// point in the world, not toward another body) and no <c>connectedAnchor</c>. The baker therefore leaves
    /// the connected body <see cref="Entity.Null"/>, which the creation system resolves to the shared static
    /// world anchor (Box2D <c>bodyA</c> at the origin) — the same world-anchor path a null built-in
    /// <c>connectedBody</c> takes. The body-local <c>anchor</c> → the runtime <c>anchor</c> (Box2D
    /// <c>localAnchorB</c>); the world-space <c>target</c> → the runtime <c>connectedAnchor</c>, which against
    /// the origin-anchored <c>bodyA</c> is that body's local space, so the relative joint holds the owner's
    /// anchor at the target. <c>maxForce</c> caps the pull; the built-in spring <c>frequency</c>/
    /// <c>dampingRatio</c> have no separate Box2D relative-joint linear-spring surface here, so the joint
    /// drives toward the target under the force cap rather than a tuned spring (a target joint's defining
    /// behaviour — reach the target — is preserved; the spring softness is the part not modelled).
    ///
    /// <para><b>Runtime-vs-serialized target.</b> A target joint is normally driven at runtime (the mouse
    /// position rewrites <c>target</c> every frame). For a single-authoring parity fixture there is no runtime
    /// driver, so the fixture authors a FIXED <c>target</c> with <c>autoConfigureTarget = false</c> — the same
    /// serialized-authoring pattern <see cref="InitialVelocity2DAuthoring"/> uses for the runtime-only velocity
    /// seed. Both backends read that one serialized target, so the joint pulls the body to the same fixed point
    /// on each.</para>
    /// </remarks>
    public sealed class TargetJoint2DBaker : Baker<TargetJoint2D>
    {
        public override void Bake(TargetJoint2D authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(
                entity,
                new PhysicsJoint2DDefinition
                {
                    kind = PhysicsJoint2DKind.Target,
                    // Single-body joint: no connected body → the static world anchor (Entity.Null path).
                    connectedBody = Entity.Null,
                    // Body-local anchor the joint grabs.
                    anchor = (float2)authoring.anchor,
                    // The world-space target is bodyA-local (bodyA is the origin world anchor).
                    connectedAnchor = (float2)authoring.target,
                    // A target joint only constrains the anchor POINT — no relative offset or orientation.
                    linearOffset = float2.zero,
                    angularOffsetDegrees = 0f,
                    // The pull force cap; the target joint has no torque constraint.
                    maxForce = authoring.maxForce,
                    maxTorque = 0f,
                    // The target joint REACHES the target via the Box2D relative joint's LINEAR position spring,
                    // which maps 1:1 from the built-in TargetJoint2D's own frequency/dampingRatio — a target
                    // joint IS a spring pulling a point toward a target. maxTorque is zero, so the creation
                    // system leaves the angular spring off (a point constraint has no orientation target).
                    enableSpring = true,
                    springFrequency = authoring.frequency,
                    springDamping = authoring.dampingRatio,
                    collideConnected = authoring.enableCollision,
                    // Break: shared Joint2D.breakForce/breakTorque/breakAction. Passed through; the creation
                    // system arms the native threshold only when finite (default Infinity = never break).
                    breakForce = authoring.breakForce,
                    breakTorque = authoring.breakTorque,
                    breakAction = Joint2DBaking.MapBreakAction(authoring.breakAction),
                }
            );
        }
    }
}
