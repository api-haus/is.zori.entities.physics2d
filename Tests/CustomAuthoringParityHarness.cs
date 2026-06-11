using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using Zori.Entities.Physics2D.Authoring;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Parity gates for the customisable authoring surface (<see cref="PhysicsBody2DAuthoring"/> +
    /// <see cref="PhysicsShape2DAuthoring"/>). Two gates of differing tightness:
    /// <list type="bullet">
    /// <item><b>Custom-vs-built-in (tight).</b> One subscene authors a custom-authored body AND an equivalent
    /// built-in <c>Rigidbody2D</c>-authored body, distinct only in start X. Both bake to the same runtime
    /// archetype and run the SAME Box2D-v3 solver in one ECS world, so their start-relative trajectories must
    /// agree to a NEAR-EXACT envelope — far tighter than the GameObject v2-vs-v3 band, because there is no
    /// cross-solver difference. This is the hard-to-falsify proof that the dual surface converges: if the
    /// custom baker emitted even a slightly different <see cref="PhysicsBody2DDefinition"/>, the two
    /// trajectories would split.</item>
    /// <item><b>Custom-authored vs GameObject (broad).</b> A custom-authored body run through the same
    /// GameObject oracle the built-in path uses — the reference <c>Rigidbody2D</c>/<c>Collider2D</c> is built
    /// live from the custom authoring component's fields. This proves a custom-authored scene reaches the same
    /// GameObject-physics band a built-in-authored one does.</item>
    /// </list>
    /// </summary>
    public static class CustomAuthoringParityHarness
    {
        const int LoadTimeoutFrames = 600;
        static readonly Vector2 Gravity = new(0f, -9.81f);
        const float ReferenceFriction = 0.4f;
        const float ReferenceRestitution = 0f;

        // ---------------------------------------------------------------------------------------------
        // Tight gate: two ECS-baked bodies (custom + built-in) in one world, same v3 solver, compared.
        // ---------------------------------------------------------------------------------------------

        /// <summary>
        /// Load <paramref name="parentSceneName"/> (a SubScene authoring exactly two dynamic bodies — one via
        /// <see cref="PhysicsBody2DAuthoring"/>/<see cref="PhysicsShape2DAuthoring"/>, one via built-in
        /// <c>Rigidbody2D</c>/collider, identical save for start X), bake both, step the ECS group
        /// <paramref name="stepCount"/> times, and assert the two bodies' start-relative trajectories agree to
        /// within <paramref name="nearExactMeters"/> / <paramref name="nearExactRadians"/>. Bodies are matched
        /// by start X: the custom body is authored at the more-negative X, the built-in at the more-positive
        /// (the test fixture guarantees this).
        /// </summary>
        public static IEnumerator RunCustomVsBuiltInParity(
            string parentSceneName,
            float dt,
            int stepCount,
            float nearExactMeters,
            float nearExactRadians
        )
        {
            SceneManager.LoadScene(parentSceneName, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world — the entities bootstrap did not run.");

            var liveQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var bakedQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            fixedGroup.Enabled = false;

            var framesWaited = 0;
            while (CountDynamic(bakedQuery) < 2 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            Assert.AreEqual(
                2,
                CountDynamic(bakedQuery),
                $"Expected exactly 2 dynamic baked bodies (custom + built-in) after {framesWaited} frames; "
                    + $"got {CountDynamic(bakedQuery)}. Build the fixture via its editor fixture builder."
            );

            var savedRateManager = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(dt);
            fixedGroup.Enabled = true;

            // First update creates the bodies (no step), so both sit at their authored pose.
            fixedGroup.Update();
            Assert.AreEqual(
                2,
                CountDynamic(liveQuery),
                "Body creation did not run for both baked bodies."
            );

            var traj = new float2[stepCount][]; // [step][bodyIndexByStartX]
            var ang = new float[stepCount][];
            for (var s = 0; s < stepCount; s++)
            {
                fixedGroup.Update();
                CaptureByStartX(liveQuery, out traj[s], out ang[s]);
            }

            fixedGroup.RateManager = savedRateManager;

            AssertNearExact(traj, ang, nearExactMeters, nearExactRadians, framesWaited);
        }

        // Capture the two dynamic bodies' (position, angle), ordered by ascending start X (index 0 = custom
        // at the more-negative X, index 1 = built-in). The ordering key is the CURRENT X, which for a pure
        // vertical fall equals the start X — the fixture authors them with no lateral velocity so X is stable.
        static void CaptureByStartX(EntityQuery liveQuery, out float2[] positions, out float[] angles)
        {
            using var ltws = liveQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            using var defs = liveQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var list = new List<(float2 pos, float ang)>();
            for (var i = 0; i < ltws.Length; i++)
            {
                if (defs[i].bodyType != Unity.U2D.Physics.PhysicsBody.BodyType.Dynamic)
                    continue;
                var m = ltws[i].Value;
                list.Add((new float2(m.c3.x, m.c3.y), atan2(m.c0.y, m.c0.x)));
            }
            list.Sort((a, b) => a.pos.x.CompareTo(b.pos.x));
            positions = new float2[list.Count];
            angles = new float[list.Count];
            for (var i = 0; i < list.Count; i++)
            {
                positions[i] = list[i].pos;
                angles[i] = list[i].ang;
            }
        }

        static void AssertNearExact(
            float2[][] traj,
            float[][] ang,
            float posTol,
            float angTol,
            int loadFrames
        )
        {
            var stepCount = traj.Length;
            Assert.AreEqual(2, traj[0].Length, "The tight gate requires exactly two dynamic bodies.");

            // Compare each body's displacement FROM ITS OWN START, cancelling the deliberate start-X offset.
            var custom0 = traj[0][0];
            var builtin0 = traj[0][1];

            var log = new System.Text.StringBuilder();
            log.AppendLine(
                $"[PHYSICS2D-CUSTOM-PARITY] steps={stepCount} loadFrames={loadFrames} "
                    + $"posTol={posTol} angTol={angTol}"
            );
            log.AppendLine("step\tposErr\tangErr");

            var worstPos = 0f;
            var worstAng = 0f;
            string posViolation = null;
            string angViolation = null;
            string nanViolation = null;
            for (var s = 0; s < stepCount; s++)
            {
                var customDisp = traj[s][0] - custom0;
                var builtinDisp = traj[s][1] - builtin0;
                var dp = length(customDisp - builtinDisp);
                var da = abs(AngleDelta(ang[s][0], ang[s][1]));

                if (
                    nanViolation == null
                    && (isnan(dp) || isinf(dp) || isnan(da) || isinf(da))
                )
                    nanViolation = $"NaN/Inf at step {s}.";
                worstPos = max(worstPos, dp);
                worstAng = max(worstAng, da);
                if (posViolation == null && dp > posTol)
                    posViolation =
                        $"Custom-vs-built-in displacement diverged at step {s}: {dp} m exceeds the "
                        + $"near-exact tolerance {posTol} m. The custom baker is not producing the same "
                        + "PhysicsBody2DDefinition/PhysicsShape2D as the built-in baker.";
                if (angViolation == null && da > angTol)
                    angViolation =
                        $"Custom-vs-built-in angle diverged at step {s}: {da} rad exceeds {angTol} rad.";
                log.AppendLine($"{s}\t{dp:E6}\t{da:E6}");
            }
            log.AppendLine(
                $"[PHYSICS2D-CUSTOM-PARITY] WORST_POS_ERR={worstPos:E6} WORST_ANG_ERR={worstAng:E6}"
            );
            if (nanViolation != null)
                log.AppendLine($"[PHYSICS2D-CUSTOM-PARITY] NAN: {nanViolation}");
            if (posViolation != null)
                log.AppendLine($"[PHYSICS2D-CUSTOM-PARITY] POS: {posViolation}");
            if (angViolation != null)
                log.AppendLine($"[PHYSICS2D-CUSTOM-PARITY] ANG: {angViolation}");

            // Disqualifier: both bodies must actually have fallen (a no-op bake would leave them at start).
            var customDrop = abs((traj[stepCount - 1][0] - custom0).y);
            var builtinDrop = abs((traj[stepCount - 1][1] - builtin0).y);
            log.AppendLine(
                $"[PHYSICS2D-CUSTOM-PARITY] customDrop={customDrop:F4} builtinDrop={builtinDrop:F4}"
            );
            string travelViolation = null;
            if (customDrop < 0.5f || builtinDrop < 0.5f)
                travelViolation =
                    $"A body barely moved (customDrop={customDrop}, builtinDrop={builtinDrop}) — a "
                    + "silently-no-op bake.";
            if (travelViolation != null)
                log.AppendLine($"[PHYSICS2D-CUSTOM-PARITY] TRAVEL: {travelViolation}");

            Debug.Log(log.ToString());

            Assert.IsNull(nanViolation, nanViolation);
            Assert.IsNull(travelViolation, travelViolation);
            Assert.IsNull(posViolation, posViolation);
            Assert.IsNull(angViolation, angViolation);
        }

        static int CountDynamic(EntityQuery query)
        {
            using var defs = query.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var n = 0;
            for (var i = 0; i < defs.Length; i++)
                if (defs[i].bodyType == Unity.U2D.Physics.PhysicsBody.BodyType.Dynamic)
                    n++;
            return n;
        }

        // ---------------------------------------------------------------------------------------------
        // Broad gate: custom-authored ECS body vs a GameObject reference built from the custom authoring.
        // ---------------------------------------------------------------------------------------------

        /// <summary>
        /// Like <c>PhysicsParityHarness.RunParity</c>, but the child authoring scene carries
        /// <see cref="PhysicsBody2DAuthoring"/>/<see cref="PhysicsShape2DAuthoring"/> (not built-in
        /// components), so the GameObject reference is built live from those custom fields. This runs a
        /// custom-authored scene through the same GameObject oracle the built-in path uses.
        /// </summary>
        public static IEnumerator RunCustomAuthoredGameObjectParity(
            string parentSceneName,
            string childSceneName,
            float dt,
            int stepCount,
            PhysicsParityHarness.ParityEnvelope envelope
        )
        {
            SceneManager.LoadScene(parentSceneName, LoadSceneMode.Single);
            yield return null;

            var world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(world, "No default ECS world.");

            var bakedQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var liveQuery = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            var fixedGroup = world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.Enabled = false;

            var framesWaited = 0;
            while (CountDynamic(bakedQuery) == 0 && framesWaited < LoadTimeoutFrames)
            {
                framesWaited++;
                yield return null;
            }
            var bakedCount = CountDynamic(bakedQuery);
            Assert.Greater(bakedCount, 0, $"No baked dynamic body after {framesWaited} frames.");

            var prevMode = UnityEngine.Physics2D.simulationMode;
            var prevGravity = UnityEngine.Physics2D.gravity;
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = Gravity;

            var refMaterial = new PhysicsMaterial2D("CustomParityReference")
            {
                friction = ReferenceFriction,
                bounciness = ReferenceRestitution,
            };

            var refLoad = SceneManager.LoadSceneAsync(childSceneName, LoadSceneMode.Additive);
            Assert.IsNotNull(refLoad, $"Child scene '{childSceneName}' is not loadable by name.");
            while (!refLoad.isDone)
                yield return null;

            var childScene = SceneManager.GetSceneByName(childSceneName);
            var bodies = BuildReferenceFromCustomAuthoring(childScene, refMaterial);
            Assert.AreEqual(
                bakedCount,
                bodies.Count,
                $"Body count mismatch: baked {bakedCount}, built {bodies.Count} reference bodies from "
                    + "the custom authoring components."
            );
            UnityEngine.Physics2D.SyncTransforms();

            var savedRateManager = fixedGroup.RateManager;
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(dt);
            fixedGroup.Enabled = true;

            fixedGroup.Update();
            Assert.AreEqual(bakedCount, CountDynamic(liveQuery), "Body creation did not run.");

            var ecsTraj = new PhysicsParityHarness.Pose[stepCount][];
            var refTraj = new PhysicsParityHarness.Pose[stepCount][];
            for (var s = 0; s < stepCount; s++)
            {
                fixedGroup.Update();
                ecsTraj[s] = CaptureEcsPoses(liveQuery);
                UnityEngine.Physics2D.Simulate(dt, UnityEngine.Physics2D.AllLayers);
                refTraj[s] = CaptureReferencePoses(bodies);
            }

            fixedGroup.RateManager = savedRateManager;

            foreach (var rb in bodies)
                if (rb != null)
                    Object.Destroy(rb.gameObject);
            Object.Destroy(refMaterial);
            UnityEngine.Physics2D.gravity = prevGravity;
            UnityEngine.Physics2D.simulationMode = prevMode;
            var refUnload = SceneManager.UnloadSceneAsync(childScene);
            if (refUnload != null)
                while (!refUnload.isDone)
                    yield return null;

            PhysicsParityHarness.AssertParity(ecsTraj, refTraj, dt, envelope, framesWaited);
        }

        // Build live Rigidbody2D + collider reference bodies from the custom authoring components in the
        // child scene — the same fields the package's custom bakers read, so single authoring is preserved:
        // one authored child scene feeds the ECS bake (via the parent SubScene) AND this GameObject reference.
        static List<Rigidbody2D> BuildReferenceFromCustomAuthoring(
            Scene childScene,
            PhysicsMaterial2D fallbackMaterial
        )
        {
            var bodies = new List<Rigidbody2D>();
            foreach (var root in childScene.GetRootGameObjects())
            {
                foreach (var bodyAuthoring in root.GetComponentsInChildren<PhysicsBody2DAuthoring>(true))
                {
                    if (bodyAuthoring.BodyType == PhysicsBody2DMotionType.Static)
                        continue;

                    var go = bodyAuthoring.gameObject;
                    var rb = go.AddComponent<Rigidbody2D>();
                    rb.bodyType =
                        bodyAuthoring.BodyType == PhysicsBody2DMotionType.Dynamic
                            ? RigidbodyType2D.Dynamic
                            : RigidbodyType2D.Kinematic;
                    rb.gravityScale = bodyAuthoring.GravityScale;
                    rb.linearDamping = bodyAuthoring.LinearDamping;
                    rb.angularDamping = bodyAuthoring.AngularDamping;
                    rb.useAutoMass = bodyAuthoring.UseAutoMass;
                    if (!bodyAuthoring.UseAutoMass && bodyAuthoring.Mass > 0f)
                        rb.mass = bodyAuthoring.Mass;
                    rb.sleepMode = RigidbodySleepMode2D.NeverSleep;

                    // Build the matching collider from the shape authoring on the same GameObject.
                    var shapeAuthoring = go.GetComponent<PhysicsShape2DAuthoring>();
                    if (shapeAuthoring != null)
                        AddColliderFor(go, shapeAuthoring, fallbackMaterial);

                    // Apply the authored initial velocity (the custom surface's first-class field).
                    if (bodyAuthoring.HasInitialVelocity)
                    {
                        rb.linearVelocity = bodyAuthoring.InitialLinearVelocity;
                        rb.angularVelocity = bodyAuthoring.InitialAngularVelocity;
                    }
                    bodies.Add(rb);
                }

                // Static floors: a PhysicsShape2DAuthoring with NO PhysicsBody2DAuthoring bakes to a
                // collider-only static body in ECS. The GameObject reference must mirror that, or the dynamic
                // bodies fall through empty space (a Collider2D with no Rigidbody2D is a live static collider).
                // Without this the ECS body rests on its baked floor while the reference free-falls — a
                // divergence that grows without bound, not a parity signal.
                foreach (var shapeAuthoring in root.GetComponentsInChildren<PhysicsShape2DAuthoring>(true))
                {
                    if (shapeAuthoring.GetComponent<PhysicsBody2DAuthoring>() != null)
                        continue; // belongs to a dynamic body, already given a collider above
                    AddColliderFor(shapeAuthoring.gameObject, shapeAuthoring, fallbackMaterial);
                }
            }
            return bodies;
        }

        static void AddColliderFor(
            GameObject go,
            PhysicsShape2DAuthoring shape,
            PhysicsMaterial2D fallbackMaterial
        )
        {
            Collider2D col = shape.Kind switch
            {
                PhysicsShape2DKind.Circle => MakeCircle(go, shape),
                PhysicsShape2DKind.Box => MakeBox(go, shape),
                PhysicsShape2DKind.Capsule => MakeCapsule(go, shape),
                _ => MakeCircle(go, shape),
            };
            col.offset = shape.Offset;
            col.density = shape.Density > 0f ? shape.Density : 1f;
            col.sharedMaterial = fallbackMaterial;
        }

        static Collider2D MakeCircle(GameObject go, PhysicsShape2DAuthoring shape)
        {
            var c = go.AddComponent<CircleCollider2D>();
            c.radius = shape.Radius;
            return c;
        }

        static Collider2D MakeBox(GameObject go, PhysicsShape2DAuthoring shape)
        {
            var c = go.AddComponent<BoxCollider2D>();
            c.size = shape.BoxSize;
            // The box corner-rounding is BoxCornerRadius (the edgeRadius analogue, default 0), NOT the
            // circle/capsule Radius — mirror the baker's field choice so the GameObject reference and the ECS
            // bake agree (reading Radius here re-introduced the doubled-floor divergence the fix removed).
            c.edgeRadius = shape.BoxCornerRadius;
            return c;
        }

        static Collider2D MakeCapsule(GameObject go, PhysicsShape2DAuthoring shape)
        {
            var c = go.AddComponent<CapsuleCollider2D>();
            c.size = shape.CapsuleSize;
            c.direction = shape.CapsuleVertical
                ? CapsuleDirection2D.Vertical
                : CapsuleDirection2D.Horizontal;
            return c;
        }

        static PhysicsParityHarness.Pose[] CaptureReferencePoses(List<Rigidbody2D> bodies)
        {
            var poses = new PhysicsParityHarness.Pose[bodies.Count];
            for (var i = 0; i < bodies.Count; i++)
                poses[i] = new PhysicsParityHarness.Pose
                {
                    position = new float2(bodies[i].position.x, bodies[i].position.y),
                    angleRadians = radians(bodies[i].rotation),
                };
            return poses;
        }

        static PhysicsParityHarness.Pose[] CaptureEcsPoses(EntityQuery liveQuery)
        {
            using var ltws = liveQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            using var defs = liveQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            var poses = new List<PhysicsParityHarness.Pose>(ltws.Length);
            for (var i = 0; i < ltws.Length; i++)
            {
                if (defs[i].bodyType == Unity.U2D.Physics.PhysicsBody.BodyType.Static)
                    continue;
                var m = ltws[i].Value;
                poses.Add(
                    new PhysicsParityHarness.Pose
                    {
                        position = new float2(m.c3.x, m.c3.y),
                        angleRadians = atan2(m.c0.y, m.c0.x),
                    }
                );
            }
            return poses.ToArray();
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
