using Unity.Mathematics;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Phase-11 step-config fixtures: SubScenes carrying a REAL <see cref="PhysicsStep2DAuthoring"/> on a
    /// GameObject, plus a built-in <see cref="Rigidbody2D"/> + <see cref="CircleCollider2D"/> faller, so the
    /// EditMode gate (<c>Phase11StepConfigEditMode</c>) bakes the actual <c>PhysicsStep2DAuthoringBaker</c> and a
    /// GameObject-parity oracle can instantiate the SAME authored faller live. Geometry is the verbatim port of
    /// the PlayMode <c>StepConfigFixtureBuilder</c> (the <c>PhysicsStep2DAuthoring</c> setters and the
    /// Faller position/collider), dropping only its <c>RegisterSceneInBuildSettings</c> /
    /// <c>MoveGameObjectToScene</c> wiring — the EditMode harness owns scene authoring through these populate
    /// methods instead.
    /// </summary>
    public static partial class Physics2DFixtures
    {
        // The authored NON-default gravity — distinct in BOTH axes from the (0,-9.81) default so a dropped-field
        // baker that left gravity at default is caught on either component. Mirrors
        // StepConfigFixtureBuilder.ConfiguredGravity; the gate matches Physics2D.gravity to this.
        public static readonly float2 P11ConfiguredGravity = new float2(5f, -20f);

        // The faller start: high enough that a body drifting/falling under the configured gravity travels well
        // within the scene without contacting anything (these fixtures author no floor — the faller free-falls).
        public static readonly Vector2 P11FallerStart = new Vector2(0f, 50f);
        public const float P11FallerRadius = 0.5f;

        // Authored distinct non-default config values, each read back off the live world by the gate.
        public const int P11SubstepsValue = 1; // default 4
        public const bool P11SleepingAllowedValue = false; // default true
        public const float P11MaximumLinearSpeedValue = 7.5f; // default 400
        public const float P11BounceThresholdValue = 13.5f; // default 1
        public const float P11ContactSpeedValue = 9.25f; // default 3

        // A dynamic faller authored as a built-in Rigidbody2D + CircleCollider2D, so the GameObject-parity oracle
        // can instantiate the SAME body live. gravityScale = 1 so the world gravity is the sole driver.
        static void MakeP11Faller(GameObject root)
        {
            var go = NewChild(root, "Faller", new Vector3(P11FallerStart.x, P11FallerStart.y, 0f));
            var rb = go.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            var circle = go.AddComponent<CircleCollider2D>();
            circle.radius = P11FallerRadius;
        }

        // A GameObject carrying a real PhysicsStep2DAuthoring, configured via the public setters so the baked
        // PhysicsWorld2DConfig carries exactly these values. mutate is the per-fixture knob.
        static GameObject MakeP11StepConfig(GameObject root, string name, System.Action<PhysicsStep2DAuthoring> mutate)
        {
            var go = NewChild(root, name, Vector3.zero);
            var step = go.AddComponent<PhysicsStep2DAuthoring>();
            mutate(step);
            return go;
        }

        // (1) Configured gravity: a NON-default gravity (a horizontal drift + a stronger-than-default fall).
        public static void P11ConfiguredGravityFixture(GameObject root)
        {
            MakeP11StepConfig(root, "StepConfig", s => s.Gravity = P11ConfiguredGravity);
            MakeP11Faller(root);
        }

        // (2) Default fallback: NO PhysicsStep2DAuthoring at all (the backward-compat path). Only the faller. The
        // world must use the Box2D defaultDefinition (g = -9.81), unchanged from before the config surface.
        public static void P11DefaultFallbackFixture(GameObject root)
        {
            MakeP11Faller(root);
        }

        // (3) Substeps: a config that differs from the default ONLY in simulationSubSteps (gravity stays the
        // default so the fall is the familiar -9.81 and the read-back of substeps is the isolated witness).
        public static void P11SubstepsFixture(GameObject root)
        {
            MakeP11StepConfig(root, "StepConfig", s => s.SimulationSubSteps = P11SubstepsValue);
            MakeP11Faller(root);
        }

        // (4) More fields: four more fields at non-default values, each read back off the live world:
        // sleepingAllowed (bool), maximumLinearSpeed (float), bounceThreshold (float), contactSpeed (float).
        // Gravity stays default.
        public static void P11MoreFieldsFixture(GameObject root)
        {
            MakeP11StepConfig(
                root,
                "StepConfig",
                s =>
                {
                    s.SleepingAllowed = P11SleepingAllowedValue;
                    s.MaximumLinearSpeed = P11MaximumLinearSpeedValue;
                    s.BounceThreshold = P11BounceThresholdValue;
                    s.ContactSpeed = P11ContactSpeedValue;
                }
            );
            MakeP11Faller(root);
        }

        // (5) Multiplicity: TWO PhysicsStep2DAuthoring on TWO GameObjects in one SubScene. [DisallowMultipleComponent]
        // only prevents two on ONE GameObject; this is the residual case the design's TryGetSingleton throw catches
        // at world creation.
        public static void P11MultiplicityFixture(GameObject root)
        {
            MakeP11StepConfig(root, "StepConfigA", s => s.Gravity = new float2(0f, -3f));
            MakeP11StepConfig(root, "StepConfigB", s => s.Gravity = new float2(0f, -7f));
            MakeP11Faller(root);
        }
    }
}
