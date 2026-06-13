using System.Collections;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine.TestTools;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// The independent gate for the Phase-A interpolation field's RUNTIME CONSEQUENCE
    /// (<see cref="Authoring.PhysicsBody2DAuthoring.Interpolation"/> →
    /// <see cref="PhysicsBody2DDefinition.interpolation"/>): a body whose interpolation mode is not
    /// <see cref="PhysicsBody2DInterpolation.None"/> gains a <see cref="PhysicsBody2DSmoothing"/> component at
    /// creation, and a <see cref="PhysicsBody2DInterpolation.None"/> body does not. That component-presence
    /// switch is the package's whole render-rate-smoothing opt-in, so it is the field's observable consequence.
    /// </summary>
    /// <remarks>
    /// <para><b>Why this is the right probe, not a convergence trajectory.</b> Interpolation is a render-rate
    /// pose overlay (<c>PhysicsBody2DSmoothingSystem</c>, in <c>TransformSystemGroup</c>) with ZERO effect on the
    /// fixed-step physics pose — two bodies that differ only in interpolation mode follow the identical
    /// trajectory under a fixed-step driver, so a trajectory-convergence gate cannot distinguish the field at
    /// all (it would pass even if the field were dropped). The observable that the field changed is the
    /// <see cref="PhysicsBody2DSmoothing"/> component the creation system adds for a non-None mode. The smoothing
    /// MATH (the interpolated/extrapolated pose) is pinned end-to-end by the standing
    /// <c>Phase8InterpCcdJointBreakGate</c>; this gate pins only that the authored field reaches the creation
    /// switch.</para>
    ///
    /// <para><b>The world carries NO smoothing system.</b> The disposable world here runs only the
    /// fixed-step creation/step/cleanup/write-back systems — never <c>PhysicsBody2DSmoothingSystem</c> — so the
    /// render-rate <c>SmoothJob</c> is never scheduled and there is no job-safety race to leak across tests.
    /// (Driving an interpolated body through the shared default world's render group is exactly the hazard this
    /// gate is shaped to avoid.)</para>
    /// </remarks>
    public sealed class PhaseAInterpolationFieldGate
    {
        const float Dt = 1f / 60f;

        static World MakePackageWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("PhaseAInterpFieldWorld", out group, Dt);

        static Entity SpawnCircle(EntityManager em, float2 pos, PhysicsBody2DInterpolation interp)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 1f,
                    initialPosition = pos,
                    useAutoMass = true,
                    interpolation = interp,
                },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
        }

        // =====================================================================================================
        // INVARIANT — a non-None interpolation body gains PhysicsBody2DSmoothing at creation; a None body does
        // not. Three bodies (Interpolate, Extrapolate, None) authored via the interpolation field; after the
        // creation Update, exactly the two non-None bodies carry the smoothing component.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator InterpolationField_NonNoneBodyGainsSmoothing_NoneBodyDoesNot()
        {
            var world = MakePackageWorld(out var group);
            var em = world.EntityManager;
            var interp = SpawnCircle(em, new float2(0f, 5f), PhysicsBody2DInterpolation.Interpolate);
            var extrap = SpawnCircle(em, new float2(3f, 5f), PhysicsBody2DInterpolation.Extrapolate);
            var none = SpawnCircle(em, new float2(6f, 5f), PhysicsBody2DInterpolation.None);

            group.Update(); // creation pass: PhysicsWorld2DSystem adds PhysicsBody2DSmoothing for non-None modes

            var interpHas = em.HasComponent<PhysicsBody2DSmoothing>(interp);
            var extrapHas = em.HasComponent<PhysicsBody2DSmoothing>(extrap);
            var noneHas = em.HasComponent<PhysicsBody2DSmoothing>(none);

            // The smoothing mode actually stored matches the authored interpolation (the byte cast in the
            // creation system).
            var interpMode = interpHas ? em.GetComponentData<PhysicsBody2DSmoothing>(interp).mode : (byte)255;
            var extrapMode = extrapHas ? em.GetComponentData<PhysicsBody2DSmoothing>(extrap).mode : (byte)255;

            UnityEngine.Debug.Log(
                $"[PHYSICS2D-PHASEA-INTERP] interpolateHasSmoothing={interpHas}(mode={interpMode}) "
                    + $"extrapolateHasSmoothing={extrapHas}(mode={extrapMode}) noneHasSmoothing={noneHas}."
            );
            world.Dispose();

            Assert.IsTrue(
                interpHas,
                "An Interpolate body did NOT gain a PhysicsBody2DSmoothing component — the Interpolation field "
                    + "did not reach the creation system's smoothing opt-in."
            );
            Assert.IsTrue(
                extrapHas,
                "An Extrapolate body did NOT gain a PhysicsBody2DSmoothing component — the Interpolation field "
                    + "did not reach the creation system's smoothing opt-in."
            );
            Assert.IsFalse(
                noneHas,
                "A None body gained a PhysicsBody2DSmoothing component — the default (no-smoothing) interpolation "
                    + "leaked an unwanted smoothing overlay."
            );
            Assert.AreEqual(
                (byte)PhysicsBody2DInterpolation.Interpolate,
                interpMode,
                "The Interpolate body's stored smoothing mode is not Interpolate — the field was mismapped."
            );
            Assert.AreEqual(
                (byte)PhysicsBody2DInterpolation.Extrapolate,
                extrapMode,
                "The Extrapolate body's stored smoothing mode is not Extrapolate — the field was mismapped."
            );
            yield break;
        }
    }
}
