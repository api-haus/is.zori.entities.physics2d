using System.Collections;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Phase-A per-field CONVERGENCE gate — the standing custom-vs-built-in convergence proof
    /// (<see cref="CustomAuthoringParityValidation.CustomAuthoredCircle_MatchesBuiltInAuthored_NearExact"/>)
    /// extended to the new authored field whose built-in equivalent reaches the SAME body pose: the
    /// filter-via-<see cref="Authoring.PhysicsShape2DAuthoring.Layer"/> path. A custom shape with
    /// <c>Layer = 8</c> and a built-in collider whose <c>gameObject.layer = 8</c>, both dropped onto a shared
    /// static floor on layer 8, bake into one ECS world running the SAME Box2D-v3 solver and must agree to a
    /// near-exact envelope — a mismapped custom <c>Layer</c> changes the resolved category/contact bits, the
    /// body falls through the floor, and the trajectories split by metres.
    /// </summary>
    /// <remarks>
    /// <para><b>Why orientation and interpolation are NOT in this body-pose convergence gate.</b> The box/capsule
    /// free-orientation fields fold the rotation into the COLLISION GEOMETRY while leaving the body pose
    /// un-rotated, whereas the only built-in way to rotate a box/capsule is to rotate the Transform, which DOES
    /// rotate the body pose — so the two are deliberately NOT body-pose-equivalent (the custom box reports body
    /// angle 0 with a shape rotated 25°; the built-in reports body angle 25°). They are behaviour-checked in
    /// <see cref="PhaseAOrientationBehaviorGate"/> instead (a rotated box rests at a height the rotation
    /// dictates, matching a GameObject collider on a rotated Transform). Interpolation is a render-rate pose
    /// overlay with NO fixed-step physics effect, and authoring it into the shared default world would schedule a
    /// render-rate smoothing job the fixed-step convergence driver never completes; its Phase-A runtime
    /// consequence (a non-None body gains a <c>PhysicsBody2DSmoothing</c> component) is pinned in an isolated
    /// world by <see cref="PhaseAInterpolationFieldGate"/>, and the smoothing math itself by the standing
    /// <c>Phase8InterpCcdJointBreakGate</c>.</para>
    ///
    /// <para>Build the fixture first via
    /// <c>-executeMethod Zori.Entities.Physics2D.Tests.Editor.PhaseAConvergenceFixtureBuilder.BuildAll</c>.</para>
    /// </remarks>
    public sealed class PhaseAConvergenceValidation
    {
        const float Dt = 1f / 60f;

        // Contact-scene near-exact band: two identical-solver v3 bodies created through different paths agree to
        // floating-point noise; a contact/landing transient amplifies that noise slightly, so the band is a
        // couple of centimetres rather than the free-fall sub-millimetre. A real bake regression (a dropped or
        // mismapped Layer) splits the trajectories by metres, so this band is adversarially tight while non-flaky.
        const float ContactPosTol = 2e-2f;
        const float ContactAngTol = 3e-2f;

        /// <summary>
        /// Filter-via-layer convergence: a custom shape with <c>Layer = 8</c> vs a built-in collider whose
        /// <c>gameObject.layer = 8</c>, both dropped onto a shared static floor on layer 8. Both resolve the
        /// identical category/contact bits from the project matrix, so both collide with and settle on the floor
        /// identically. A mismapped custom <c>Layer</c> would change the bits and the body would pass through the
        /// floor — a divergence the convergence band catches as a metres-scale split.
        /// </summary>
        [UnityTest]
        public IEnumerator FilterLayer_CustomLayer_MatchesBuiltInGameObjectLayer()
        {
            yield return CustomAuthoringParityHarness.RunCustomVsBuiltInParity(
                "PhaseAFilterLayer",
                Dt,
                stepCount: 180,
                nearExactMeters: ContactPosTol,
                nearExactRadians: ContactAngTol
            );
        }
    }
}
