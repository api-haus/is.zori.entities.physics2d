using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using Zori.Entities.Physics2D.Authoring;

namespace Zori.Entities.Physics2D.Tests.Editor
{
    /// <summary>
    /// Populate methods for the custom-authoring EditMode gates — the fixtures the former PlayMode
    /// <c>CustomAuthoring2DSceneBuilder</c>, <c>CustomAuthoringFixtureBuilder</c> and
    /// <c>PhaseAConvergenceFixtureBuilder</c> authored, ported verbatim into the portable
    /// <see cref="Physics2DFixtures"/> populate model. Geometry is identical; only the GameObject creation idiom
    /// (now <see cref="NewChild"/> under the fixture root) and asset persistence (now into
    /// <see cref="CurrentFolder"/>) differ, so the EditMode harness loads + opens them like every other fixture.
    /// </summary>
    public static partial class Physics2DFixtures
    {
        // ---- CustomAuthoring2D (CustomAuthoring2DSceneBuilder) ----------------------------------------------

        const float CaFloorTop = 0f;
        const int CaFilterCategory = 8; // an arbitrary explicit category bit for the filtered pair

        public static void CustomAuthoring2D(GameObject root)
        {
            // The material template MaterialBody inherits its bounciness from.
            var template = new PhysicsMaterial2D("BouncyTemplate") { friction = 0.3f, bounciness = 0.8f };
            AssetDatabase.CreateAsset(template, s_CurrentFolder + "/BouncyTemplate.physicsMaterial2D");

            // A static box floor (no PhysicsBody2DAuthoring → the shape baker's static fallback). Radius 0 so the
            // box has a sharp top edge (a non-zero corner radius would raise the effective floor surface).
            var floor = CaMakeShape(root, "Floor", new Vector3(0f, CaFloorTop, 0f));
            floor.Kind = PhysicsShape2DKind.Box;
            floor.BoxSize = new float2(40f, 1f);
            floor.Radius = 0f;

            // The five shape kinds as dynamic bodies that fall and settle on the floor. (Render-rate
            // Interpolation is a documented authoring field but is intentionally NOT set on a scene body: it is
            // invisible in a still settling scene and, loaded into the persistent default test world via a
            // SubScene, its PhysicsBody2DSmoothing entities leak the smoothing job across tests. It is exercised
            // in isolation by Phase8InterpCcdJointBreakGate and documented in custom-authoring.md.)
            var circle = CaMakeBody(root, "CircleBody", new Vector3(-9f, 6f, 0f), out var circleShape);
            circleShape.Kind = PhysicsShape2DKind.Circle;
            circleShape.Radius = 0.5f;

            var box = CaMakeBody(root, "BoxBody", new Vector3(-6f, 6f, 0f), out var boxShape);
            boxShape.Kind = PhysicsShape2DKind.Box;
            boxShape.BoxSize = new float2(1f, 1f);
            boxShape.BoxAngle = 20f; // the free box orientation a built-in BoxCollider2D cannot author

            var capsule = CaMakeBody(root, "CapsuleBody", new Vector3(-3f, 6f, 0f), out var capsuleShape);
            capsuleShape.Kind = PhysicsShape2DKind.Capsule;
            capsuleShape.CapsuleSize = new float2(0.8f, 1.6f);
            capsuleShape.CapsuleVertical = true;
            capsuleShape.CapsuleAngle = 15f; // the free capsule orientation

            var polygon = CaMakeBody(root, "PolygonBody", new Vector3(0f, 6f, 0f), out var polygonShape);
            polygonShape.Kind = PhysicsShape2DKind.Polygon;
            polygonShape.Vertices = new[]
            {
                new Vector2(-0.5f, -0.5f),
                new Vector2(0.5f, -0.5f),
                new Vector2(0.7f, 0.2f),
                new Vector2(0f, 0.7f),
                new Vector2(-0.7f, 0.2f),
            }; // a 5-vertex convex hull — the single-hull (PolygonDecompose off) path
            polygonShape.PolygonDecompose = false;

            // A static edge/chain wall (the 2D analogue of the 3D "plane"; an open chain, no enclosing body).
            var edge = CaMakeShape(root, "EdgeWall", new Vector3(12f, 1f, 0f));
            edge.Kind = PhysicsShape2DKind.Edge;
            edge.EdgeIsLoop = false;
            edge.Vertices = new[] { new Vector2(-2f, 0f), new Vector2(0f, 1.5f), new Vector2(2f, 0f) };

            // A body driven by a PhysicsMaterial2D template (bounciness inherited) WITH a per-field override
            // (friction overridden inline) — the Phase-B inheritance + override model.
            var material = CaMakeBody(root, "MaterialBody", new Vector3(3f, 7f, 0f), out var materialShape);
            materialShape.Kind = PhysicsShape2DKind.Box;
            materialShape.BoxSize = new float2(1f, 1f);
            materialShape.MaterialTemplate = template; // inherits bounciness 0.8
            materialShape.OverrideFriction = true;
            materialShape.Friction = 0.1f; // overrides the template's friction 0.3 with a slippery 0.1

            // A filtered pair: two circles that share an explicit category bit and collide only with that bit, so
            // they collide with each other but not with the default-filter bodies. OverrideFilterBits bypasses the
            // project layer-collision matrix entirely, so the demonstration is project-independent.
            var filterA = CaMakeBody(root, "FilteredBodyA", new Vector3(7f, 6f, 0f), out var filterAShape);
            CaConfigureFilteredCircle(filterAShape);
            var filterB = CaMakeBody(root, "FilteredBodyB", new Vector3(7f, 9f, 0f), out var filterBShape);
            CaConfigureFilteredCircle(filterBShape);
        }

        static void CaConfigureFilteredCircle(PhysicsShape2DAuthoring shape)
        {
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = 0.4f;
            shape.OverrideFilterBits = true;
            shape.CategoryBits = 1 << CaFilterCategory;
            shape.ContactBits = 1 << CaFilterCategory;
        }

        static PhysicsBody2DAuthoring CaMakeBody(
            GameObject root,
            string name,
            Vector3 position,
            out PhysicsShape2DAuthoring shape
        )
        {
            var go = NewChild(root, name, position);
            var body = go.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.GravityScale = 1f;
            body.UseAutoMass = true; // density-derived mass — no per-body mass tuning
            shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Density = 1f;
            return body;
        }

        static PhysicsShape2DAuthoring CaMakeShape(GameObject root, string name, Vector3 position)
        {
            var go = NewChild(root, name, position);
            return go.AddComponent<PhysicsShape2DAuthoring>();
        }

        // ---- CustomVsBuiltIn (CustomAuthoringFixtureBuilder) -----------------------------------------------

        const float CustomAuthStartY = 10f;
        const float CustomAuthCircleRadius = 0.5f;

        public static void CustomVsBuiltIn(GameObject root)
        {
            // Custom-authored body at the more-negative X (index 0 in the X-sorted compare).
            var customGo = NewChild(root, "CustomBody", new Vector3(-2f, CustomAuthStartY, 0f));
            var body = customGo.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.GravityScale = 1f;
            body.UseAutoMass = true; // density-derived mass — matches the built-in default-collider path
            var shape = customGo.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = CustomAuthCircleRadius;
            shape.Density = 1f;

            // Built-in body at the more-positive X (index 1), authored to match the custom body's params.
            var builtinGo = NewChild(root, "BuiltInBody", new Vector3(2f, CustomAuthStartY, 0f));
            var rb = builtinGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.gravityScale = 1f;
            rb.useAutoMass = true;
            var circle = builtinGo.AddComponent<CircleCollider2D>();
            circle.radius = CustomAuthCircleRadius;
            circle.density = 1f;
        }

        // ---- CustomCircleOnFloor (CustomAuthoringFixtureBuilder) -------------------------------------------

        public static void CustomCircleOnFloor(GameObject root)
        {
            // Custom-authored static box floor (no PhysicsBody2DAuthoring → the shape baker's static fallback).
            var floor = NewChild(root, "CustomFloor", new Vector3(0f, 0f, 0f));
            var floorShape = floor.AddComponent<PhysicsShape2DAuthoring>();
            floorShape.Kind = PhysicsShape2DKind.Box;
            floorShape.BoxSize = new float2(40f, 1f);

            // Custom-authored dynamic circle body that falls onto the floor and rests.
            var bodyGo = NewChild(root, "CustomFallingBody", new Vector3(0f, 5f, 0f));
            var body = bodyGo.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.GravityScale = 1f;
            body.UseAutoMass = true;
            var shape = bodyGo.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = CustomAuthCircleRadius;
            shape.Density = 1f;
        }

        // ---- PhaseAFilterLayer (PhaseAConvergenceFixtureBuilder) -------------------------------------------

        const float PhaseACustomX = -2f;
        const float PhaseABuiltInX = 2f;
        const int PhaseAFilterLayerIndex = 8;

        public static void PhaseAFilterLayer(GameObject root)
        {
            // A custom-authored static floor on the filter layer so the layer-collision matrix governs the
            // contact for both bodies; the floor sits under both drop columns.
            var floor = NewChild(root, "CustomFilterFloor", new Vector3(0f, 0f, 0f));
            floor.layer = PhaseAFilterLayerIndex;
            var floorShape = floor.AddComponent<PhysicsShape2DAuthoring>();
            floorShape.Kind = PhysicsShape2DKind.Box;
            floorShape.BoxSize = new float2(40f, 1f);
            floorShape.Layer = PhaseAFilterLayerIndex;

            // Custom shape with Layer = 8 at X=-2.
            var customGo = NewChild(root, "CustomFilterBody", new Vector3(PhaseACustomX, 5f, 0f));
            customGo.layer = PhaseAFilterLayerIndex;
            var cBody = customGo.AddComponent<PhysicsBody2DAuthoring>();
            cBody.BodyType = PhysicsBody2DMotionType.Dynamic;
            cBody.UseAutoMass = true;
            var cShape = customGo.AddComponent<PhysicsShape2DAuthoring>();
            cShape.Kind = PhysicsShape2DKind.Circle;
            cShape.Radius = 0.5f;
            cShape.Density = 1f;
            cShape.Layer = PhaseAFilterLayerIndex;

            // Built-in collider whose gameObject.layer = 8 at X=+2.
            var builtinGo = NewChild(root, "BuiltInFilterBody", new Vector3(PhaseABuiltInX, 5f, 0f));
            builtinGo.layer = PhaseAFilterLayerIndex;
            var rb = builtinGo.AddComponent<Rigidbody2D>();
            rb.bodyType = RigidbodyType2D.Dynamic;
            rb.useAutoMass = true;
            var circle = builtinGo.AddComponent<CircleCollider2D>();
            circle.radius = 0.5f;
            circle.density = 1f;
        }
    }
}
