using NUnit.Framework;
using Zori.Entities.Physics2D.Tests.Editor;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>PhaseAConvergenceValidation</c> — the Phase-A per-field CONVERGENCE gate. A custom
    /// shape with <c>Layer = 8</c> and a built-in collider whose <c>gameObject.layer = 8</c>, both dropped onto a
    /// shared static floor on layer 8, bake into one ECS world running the SAME Box2D-v3 solver and must agree to
    /// a near-exact envelope — a mismapped custom <c>Layer</c> changes the resolved category/contact bits, the
    /// body falls through the floor, and the trajectories split by metres. Tolerances and step count copied
    /// verbatim from the PlayMode gate.
    /// </summary>
    public sealed class PhaseAConvergenceEditMode : Physics2DEditModeHarness
    {
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
        [Test]
        public void FilterLayer_CustomLayer_MatchesBuiltInGameObjectLayer()
        {
            RunCustomVsBuiltInParity(
                Physics2DFixtures.PhaseAFilterLayer,
                "PhaseAFilterLayer",
                180,
                ContactPosTol,
                ContactAngTol
            );
        }
    }
}
