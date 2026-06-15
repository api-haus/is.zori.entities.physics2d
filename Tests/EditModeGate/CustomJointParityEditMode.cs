using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using Zori.Entities.Physics2D.Tests.Editor;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>CustomJointParityValidation</c> (BEHAVIORAL parity): beyond the bake-level
    /// field-identity the <see cref="CustomJointConvergenceBakeEditMode"/> proves, this gate proves a
    /// custom-authored joint SIMULATES the same as the equivalent built-in <c>*Joint2D</c>. It reuses the
    /// all-nine-kind <c>CustomJointConvergence</c> SubScene (each kind authors a custom owner AND a built-in owner
    /// of identical params, each pinned to its own static anchor), bakes BOTH owners into one ECS Game world, steps
    /// that world under the joint harness driving (<c>FixedStepSimulationSystemGroup.Update()</c> under a swapped
    /// <c>FixedRateSimpleManager(1/60)</c>), and compares each span kind's custom owner's start-relative trajectory
    /// against its built-in twin's. ECS-vs-ECS: no GameObject reference. Constants, <c>Debug.Log</c>, and the
    /// <c>MapOwnerEntitiesByInitialX</c>/<c>CaptureByEntity</c>/<c>AssertPairParity</c>/<c>AngleDelta</c> compare are
    /// copied verbatim.
    /// </summary>
    public sealed class CustomJointParityEditMode : Physics2DEditModeHarness
    {
        const int StepCount = 150;
        const int ExpectedJointOwners = 18; // 9 kinds × (custom + built-in)

        // Mirror of CustomJointConvergenceFixtureBuilder X-keys for the span this gate witnesses.
        const float XHingeCustom = -10f;
        const float XHingeBuiltIn = -8f;
        const float XWheelCustom = -4f;
        const float XWheelBuiltIn = -2f;
        const float XSliderCustom = 20f;
        const float XSliderBuiltIn = 22f;

        // ECS-vs-ECS near-exact POSITION band. Both owners run the SAME v3 solver on field-identical joints, so
        // the only divergence is floating-point non-associativity in the per-body solve order. The worst measured
        // span error is ~1e-4 m.
        const float NearExactMeters = 5e-3f;

        // A jointed owner must actually MOVE over the window, or "both frozen" would pass as parity.
        const float MinTravelMeters = 0.2f;

        // Angle parity caps. A body that holds its orientation (axis-confined Slider) tracks angle tightly; a body
        // that orbits or free-spins (Hinge pendulum, Wheel under a motor) accumulates angle from FP solver-order
        // noise over 150 steps, so its angle gets the generous π cap.
        const float AngleCapHeld = 5e-3f;
        const float AngleCapSpin = PI;

        [Test]
        public void CustomJoint_SimulatesSameAsBuiltIn_AcrossSpan()
        {
            LoadSubScene(Physics2DFixtures.CustomJointConvergence, "CustomJointConvergence");

            // Joint owners carry a PhysicsBody2DDefinition (the body) + a PhysicsJoint2DDefinition (the joint).
            // Key each owner by its baked initial X so a custom owner can be paired with its built-in twin.
            var ownerQuery = Query(
                ComponentType.ReadOnly<PhysicsJoint2DDefinition>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            var ownerCount = ownerQuery.CalculateEntityCount();
            Assert.GreaterOrEqual(
                ownerCount,
                ExpectedJointOwners,
                $"Only {ownerCount} baked joint owners appeared (expected {ExpectedJointOwners}) — the "
                    + "CustomJointConvergence fixture did not bake all nine custom-vs-built-in pairs."
            );

            // First Update runs body + joint creation (no integration on the creation frame), so every owner
            // sits at its authored pose before the first captured step.
            CreateBodies();

            // Pair each owner ENTITY to its kind by its BAKED initial X (PhysicsBody2DDefinition.initialPosition.x
            // — the immutable authored position). Entity identity is then the per-step tracking key.
            var entityByKey = MapOwnerEntitiesByInitialX(ownerQuery);

            var traj = new Dictionary<Entity, (float2 pos, float ang)>[StepCount];
            for (var s = 0; s < StepCount; s++)
            {
                FixedGroup.Update();
                traj[s] = CaptureByEntity(ownerQuery);
            }

            // Compare each span kind's custom owner trajectory against its built-in twin's, start-relative.
            // The span is the three CONSTRAINT kinds whose transient is non-chaotic in this fixture's geometry:
            // Hinge (pendulum orbit), Slider (motor-driven axis slide), Wheel (suspension sag + wheel spin).
            // Position stays tight for every span kind; angle is tight only for the orientation-holding Slider.
            // The other six kinds are pinned at the BAKE level by the convergence gate.
            AssertPairParity("Hinge", XHingeCustom, XHingeBuiltIn, AngleCapSpin, entityByKey, traj);
            AssertPairParity("Slider", XSliderCustom, XSliderBuiltIn, AngleCapHeld, entityByKey, traj);
            AssertPairParity("Wheel", XWheelCustom, XWheelBuiltIn, AngleCapSpin, entityByKey, traj);
        }

        // Map each joint-owner entity to its authored X-key (the even integer it was authored at), read from the
        // BAKED PhysicsBody2DDefinition.initialPosition.x — immutable, so the pairing never shifts under motion.
        static Dictionary<float, Entity> MapOwnerEntitiesByInitialX(EntityQuery ownerQuery)
        {
            using var entities = ownerQuery.ToEntityArray(Allocator.Temp);
            using var defs = ownerQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var map = new Dictionary<float, Entity>();
            for (var i = 0; i < entities.Length; i++)
            {
                var key = (float)System.Math.Round(defs[i].initialPosition.x / 2.0) * 2f;
                map[key] = entities[i];
            }
            return map;
        }

        // Capture every joint-owner's live (position, angle) keyed by ENTITY — stable across steps regardless of
        // how far the owner moves laterally.
        static Dictionary<Entity, (float2 pos, float ang)> CaptureByEntity(EntityQuery ownerQuery)
        {
            using var entities = ownerQuery.ToEntityArray(Allocator.Temp);
            using var ltws = ownerQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            var map = new Dictionary<Entity, (float2, float)>();
            for (var i = 0; i < ltws.Length; i++)
            {
                var m = ltws[i].Value;
                map[entities[i]] = (new float2(m.c3.x, m.c3.y), atan2(m.c0.y, m.c0.x));
            }
            return map;
        }

        static void AssertPairParity(
            string label,
            float xCustom,
            float xBuiltIn,
            float angleCap,
            Dictionary<float, Entity> entityByKey,
            Dictionary<Entity, (float2 pos, float ang)>[] traj
        )
        {
            var customKey = (float)System.Math.Round(xCustom / 2.0) * 2f;
            var builtInKey = (float)System.Math.Round(xBuiltIn / 2.0) * 2f;

            Assert.IsTrue(entityByKey.ContainsKey(customKey), $"{label}: no custom joint owner at X-key {customKey}.");
            Assert.IsTrue(
                entityByKey.ContainsKey(builtInKey),
                $"{label}: no built-in joint owner at X-key {builtInKey}."
            );
            var customEntity = entityByKey[customKey];
            var builtInEntity = entityByKey[builtInKey];

            var custom0 = traj[0][customEntity].pos;
            var builtin0 = traj[0][builtInEntity].pos;

            var worstPos = 0f;
            var worstAng = 0f;
            string posViolation = null;
            string angViolation = null;
            string nanViolation = null;
            for (var s = 0; s < traj.Length; s++)
            {
                var c = traj[s][customEntity];
                var b = traj[s][builtInEntity];
                var customDisp = c.pos - custom0;
                var builtinDisp = b.pos - builtin0;
                var dp = length(customDisp - builtinDisp);
                var da = abs(AngleDelta(c.ang, b.ang));

                if (nanViolation == null && (isnan(dp) || isinf(dp) || isnan(da) || isinf(da)))
                    nanViolation = $"{label}: NaN/Inf at step {s}.";
                worstPos = max(worstPos, dp);
                worstAng = max(worstAng, da);
                if (posViolation == null && dp > NearExactMeters)
                    posViolation =
                        $"{label}: custom-vs-built-in joint motion diverged at step {s}: {dp} m exceeds the "
                        + $"near-exact band {NearExactMeters} m. The custom baker's {label} arm is not "
                        + "producing the same Box2D joint as the built-in baker (a mismapped motion field).";
                if (angViolation == null && da > angleCap)
                    angViolation =
                        $"{label}: custom-vs-built-in joint angle diverged at step {s}: {da} rad exceeds "
                        + $"{angleCap} rad.";
            }

            // Disqualifier: the jointed owner actually moved (a "both frozen / both broken" pair would have a
            // near-zero diff AND near-zero travel, passing the band but proving nothing about the joint).
            var customTravel = length(traj[traj.Length - 1][customEntity].pos - custom0);
            var builtinTravel = length(traj[traj.Length - 1][builtInEntity].pos - builtin0);
            string travelViolation = null;
            if (customTravel < MinTravelMeters || builtinTravel < MinTravelMeters)
                travelViolation =
                    $"{label}: a jointed owner barely moved (customTravel={customTravel} m, "
                    + $"builtinTravel={builtinTravel} m < {MinTravelMeters} m) — the joint did not produce "
                    + "the expected motion, so the parity band proves nothing.";

            Debug.Log(
                $"[PHYSICS2D-PHASEF-BEHAVIOR] {label} worstPosErr={worstPos:E6} worstAngErr={worstAng:E6} "
                    + $"customTravel={customTravel:F4} builtinTravel={builtinTravel:F4} "
                    + $"posBand={NearExactMeters} angCap={angleCap}"
            );

            Assert.IsNull(nanViolation, nanViolation);
            Assert.IsNull(travelViolation, travelViolation);
            Assert.IsNull(posViolation, posViolation);
            Assert.IsNull(angViolation, angViolation);
        }

        static float AngleDelta(float a, float b)
        {
            var d = a - b;
            while (d > PI)
                d -= 2f * PI;
            while (d < -PI)
                d += 2f * PI;
            return d;
        }
    }
}
