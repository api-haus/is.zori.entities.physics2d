# Custom Authoring 2D

This sample is the customisable-authoring entry point and the worked example of the package's low-level direct surface. It mirrors the `com.unity.physics` "Custom Physics Authoring" sample (3D, structure-only reference): a custom body authoring component, a custom shape authoring component, and a code path that authors physics entities directly without any MonoBehaviour.

## What it demonstrates

The customisable authoring components and their bakers ship in the package's compiled `Authoring/` and `Baking/` assemblies (so they are unit-tested in the package), and this sample references them. Importing the sample copies it into `Assets/Samples/`, where it is yours to edit — which is the point of "customisable".

- `PhysicsBody2DAuthoring` (`Zori.Entities.Physics2D.Authoring`) authors the runtime `PhysicsBody2DDefinition` directly, exposing the body type, gravity scale, damping, explicit mass / auto-mass, per-DOF freeze constraints, and an initial linear/angular velocity as first-class inspector fields. The initial velocity is the knob a built-in `Rigidbody2D` cannot author at bake time, because `Rigidbody2D.linearVelocity` is runtime-only.
- `PhysicsShape2DAuthoring` authors the runtime `PhysicsShape2D` directly, carrying the `PhysicsShape2DKind` union (Circle / Box / Capsule / Polygon / Edge), explicit geometry, an offset, and inline friction / bounciness / density — the geometry a built-in collider would round-trip through sprite or mesh resolution.
- `BatchSpawnSampleSystem` demonstrates the bulk-creation optimization: it enqueues one `PhysicsBody2DBatchRequest`, which `PhysicsBody2DBatchCreationSystem` turns into many identical dynamic bodies in a single `PhysicsWorld.CreateBodyBatch` native call.

## The convergence property

A body authored with `PhysicsBody2DAuthoring`/`PhysicsShape2DAuthoring` and the equivalent body authored with the built-in `Rigidbody2D`/`Collider2D` bake to the same runtime archetype and run the same Box2D solver, so they simulate identically. The package's step and write-back systems never ask which surface created a body. This is the same property the `com.unity.physics` custom-authoring sample relies on, where the custom baker emits the same runtime components the built-in `Rigidbody` baker emits.

## Authoring a scene with the custom components

1. Add a `PhysicsBody2DAuthoring` to a GameObject and set its body type and parameters.
2. Add a `PhysicsShape2DAuthoring` to the same GameObject and choose its kind and geometry. A GameObject carrying only a shape (no body) bakes to a collider-only static body, the same rule built-in 2D physics applies to a `Collider2D` with no `Rigidbody2D`.
3. Place the GameObject in a SubScene so the standard `Baker<T>` pipeline runs at bake time. The package's `PhysicsBody2DAuthoringBaker` and `PhysicsShape2DAuthoringBaker` convert them; at runtime the body steps and its pose is written into `Unity.Transforms.LocalToWorld`.

## The low-level direct surface (no MonoBehaviour)

To author physics entities entirely from code, call `DirectPhysics2DAuthoring.Create(entityManager, body, shape)` (or the `EntityCommandBuffer` overload from inside a job). The entity gains a `PhysicsBody2DDefinition`, a `PhysicsShape2D`, and a `LocalToWorld`, and the package's per-entity creation loop turns it into a live Box2D body on the next fixed step — the same loop a baked entity flows through.

For many identical bodies in one frame, create one `PhysicsBody2DBatchRequest` entity instead. It carries the shared definition, the circle radius, a count, and a scatter AABB; `PhysicsBody2DBatchCreationSystem` creates every body in a single `CreateBodyBatch` call and scatters their start positions in a single `SetBatchTransform` call.

## Deliberately omitted knobs

The 3D DOTS sample also exposes `WorldIndex`, `SolverType`, and a custom inertia tensor. This 2D sample omits them: world-index sharding needs the multi-world model the package defers, and a custom 2D solver or inertia override has no field on the current runtime archetype. They are the natural additive extension when that infrastructure lands, not a fork of the current surface.
