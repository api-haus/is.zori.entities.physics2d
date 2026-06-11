# Custom Authoring 2D

This sample is the customisable-authoring entry point: the complete `PhysicsBody2DAuthoring` / `PhysicsShape2DAuthoring` surface, an authored scene that demonstrates the full range, and the worked example of the package's low-level direct-creation path. It is the 2D analogue of the `com.unity.physics` "Custom Physics Authoring" sample (3D, structure-only reference) — a custom body authoring component, a custom shape authoring component, a 2D-native scene-view editor, and a code path that authors physics entities directly without any MonoBehaviour.

The authoring components and their bakers ship in the package's compiled `Authoring/` and `Baking/` assemblies (so they are unit-tested in the package), and this sample references them. Importing the sample copies it into `Assets/Samples/`, where it is yours to edit — which is the point of "customisable". The sample carries no `com.unity.entities.graphics` dependency: the package is renderer-agnostic, so the custom-authoring sample's value is the edit-time authoring experience (the scene-view gizmos draw every shape without a renderer) and the bake-and-simulate path; a runtime-rendered benchmark sample is a separate, later sample.

## The sample scene

`Scenes/CustomAuthoring2DSample.unity` is the parent scene you open; it carries one `SubScene` whose child scene (`CustomAuthoring2DSample_Sub.unity`) holds nine authored GameObjects that exercise the complete body+shape surface across one screen.

- **CircleBody / BoxBody / CapsuleBody / PolygonBody** — the four enclosing shape kinds as dynamic bodies. BoxBody carries a free `BoxAngle` of 20° and CapsuleBody a free `CapsuleAngle` of 15° — the free orientations the built-in `BoxCollider2D` / `CapsuleCollider2D` cannot author (their rotation is their Transform's). PolygonBody is a five-vertex convex hull on the single-hull path.
- **EdgeWall** — a static open three-vertex Edge/chain (the 2D analogue of the 3D sample's plane: an open chain, not an enclosing solid).
- **Floor** — a static box collider, no body, that the dynamic bodies settle on (the shape baker's collider-only static-body fallback). Its corner-rounding `Radius` is 0 so its top surface is a sharp edge.
- **MaterialBody** — a box driven by a `PhysicsMaterial2D` template (`Scenes/BouncyTemplate.physicsMaterial2D`, bounciness 0.8) whose bounciness it inherits, with a per-field friction override (the inline 0.1 wins over the template's 0.3). This is the Phase-B material-template + per-field-override model.
- **FilteredBodyA / FilteredBodyB** — a filtered pair: two circles that author an explicit contact-filter category/contact bitset (`OverrideFilterBits`, category and contact both `1 << 8`), so they collide with each other but the default-filter bodies do not enter their category. The explicit bitset bypasses the project layer-collision matrix entirely, so the demonstration is project-independent.

On import, open the parent scene. At **edit time** you see the custom inspectors (the per-kind geometry, the material override rows, the filter section, the "Fit To…" dropdown) and the scene-view gizmo outlines + draggable handles for every shape — no renderer required. Press **Play** and the SubScene bakes, the bodies simulate, and each body's `Unity.Transforms.LocalToWorld` updates as it falls and settles on the floor.

## The complete authoring surface

- **`PhysicsBody2DAuthoring`** (`Zori.Entities.Physics2D.Authoring`) authors the runtime `PhysicsBody2DDefinition` directly: body type, gravity scale, linear/angular damping, explicit mass / auto-mass, the three per-DOF freeze constraints, an initial linear/angular velocity, render-rate interpolation, continuous collision detection, and a custom mass-distribution override (a `float2` center of mass + a scalar rotational inertia). The initial velocity is the knob a built-in `Rigidbody2D` cannot author at bake time, because `Rigidbody2D.linearVelocity` is runtime-only.
- **`PhysicsShape2DAuthoring`** authors the runtime `PhysicsShape2D` directly: the `PhysicsShape2DKind` union (Circle / Box / Capsule / Polygon / Edge), explicit geometry with free box/capsule orientation, an offset, a `PhysicsMaterial2D` material template with per-field friction / bounciness / combine override, an explicit contact-filter bitset alongside the layer, and the Collide / Sensor collision response. The 2D-native editor draws gizmo outlines, draggable scene handles per kind, and a "Fit To…" dropdown that fits the shape to a Sprite physics shape, a SpriteRenderer's bounds, or a PolygonCollider2D's paths.
- **`BatchSpawnSampleSystem`** demonstrates the bulk-creation path: it enqueues one `PhysicsBody2DBatchRequest`, which `PhysicsBody2DBatchCreationSystem` turns into many identical dynamic bodies in a single `PhysicsWorld.CreateBodyBatch` native call.

The full per-field reference, the material-template / override model, the filter precedence, the auto-fit, the editor handles/gizmos, and the deliberate 2D negative space are documented in the package's `Documentation~/custom-authoring.md`.

## The convergence property

A body authored with `PhysicsBody2DAuthoring` / `PhysicsShape2DAuthoring` and the equivalent body authored with the built-in `Rigidbody2D` / `Collider2D` bake to the same runtime archetype and run the same Box2D solver, so they simulate identically — the package's step and write-back systems never ask which surface created a body. The convergence gate measures this directly: a custom-authored circle and a built-in-authored circle run a bit-identical trajectory (0.0 worst error over 120 steps). This is the same property the `com.unity.physics` custom-authoring sample relies on.

## The low-level direct surface (no MonoBehaviour)

To author physics entities entirely from code, call `DirectPhysics2DAuthoring.Create(entityManager, body, shape)` (or the `EntityCommandBuffer` overload from inside a job). The entity gains a `PhysicsBody2DDefinition`, a `PhysicsShape2D`, and a `LocalToWorld`, and the package's per-entity creation loop turns it into a live Box2D body on the next fixed step — the same loop a baked entity flows through. For many identical bodies in one frame, create one `PhysicsBody2DBatchRequest` entity instead; `PhysicsBody2DBatchCreationSystem` creates every body in a single `CreateBodyBatch` call and scatters their start positions in a single `SetBatchTransform` call.

## The deliberate 2D negative space

The complete surface covers every **2D-expressible** feature, which is not a literal 1:1 with the 3D sample — Box2D-v3 geometry is exactly Circle / Capsule / Polygon (≤8 vertices) / Segment / Chain, so the 3D Cylinder, Plane, and 3D-mesh shape types are designed out (a 2D "cylinder" is a box, a "plane" is a box or an edge, a "mesh collider" is a chain or a decomposed convex polygon set). The 3D sample's perspective scene-handle math and async convex/mesh previews are likewise N/A — 2D handles are flat and synchronous. The full negative-space rationale is in `Documentation~/custom-authoring.md`.

`WorldIndex` and `SolverType` (the 3D sample's two remaining body knobs) are the genuine omissions: world-index sharding needs the multi-world model the package defers, and there is no per-body solver field on the 2D archetype. They are the natural additive extension when that infrastructure lands, not a fork of the current surface.
