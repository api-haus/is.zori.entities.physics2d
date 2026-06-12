# Zori Entities Physics 2D

2D physics for Unity Entities (DOTS / ECS), built on Unity's embedded Box2D v3.

`is.zori.entities.physics2d` binds Unity's embedded Box2D v3 — the `Unity.U2D.Physics` API (`PhysicsWorld`, `PhysicsBody`, `PhysicsShape`, the geometry and query types) that ships in the editor from `6000.3` onward — to Entities. It bakes authoring to ECS components, creates and steps one `PhysicsWorld` per ECS `World` inside `FixedStepSimulationSystemGroup`, and writes each body's pose to `Unity.Transforms.LocalToWorld`. The simulation is Box2D's; this package is the ECS binding, not a new solver. It writes `LocalToWorld` and nothing else, so it doesn't assume a renderer.

The API follows `com.unity.physics`. A `PhysicsWorldSingleton2D` holds the world handle, spatial queries run over it from Burst jobs, and the query surface mirrors Unity Physics where a 2D equivalent exists — for example, `PhysicsQueries2D.ClosestPoint` corresponds to `CollisionWorld.CalculateDistance(PointDistanceInput)`.

It targets feature parity with GameObject 2D physics, checked by GameObject-vs-ECS parity tests (150 / 150 passing), with a few [known gaps](Documentation~/parity-matrix.md).

## Install

Add the package by git URL (Package Manager → Add package from git URL), or in `Packages/manifest.json`:

```json
"dependencies": {
  "is.zori.entities.physics2d": "https://github.com/api-haus/is.zori.entities.physics2d.git"
}
```

You can also drop the package into your project's `Packages/` folder as an embedded package. See [Installation](Documentation~/installation.md) for requirements and how to run the tests.

## Quick start

1. Put a GameObject with a `Rigidbody2D` (Dynamic) and a `CircleCollider2D` in a SubScene.
2. Add a `Collider2D`-only GameObject below it as a floor (no `Rigidbody2D` → static body).
3. Enter Play mode. The body falls, rests on the floor, and its entity's `LocalToWorld` updates every fixed step.

There are no new authoring concepts to learn. Push, move, and query bodies at runtime from your own ECS systems via `PhysicsBody2DCommands` and `PhysicsQueries2D`, and read contacts, triggers, and joint-breaks from the event buffers. See [Getting started](Documentation~/getting-started.md).

## How it binds Box2D to ECS

- **Authoring bakes to ECS components.** Add the built-in `Rigidbody2D` / `Collider2D` / `*Joint2D` / `*Effector2D` / `PhysicsMaterial2D` to GameObjects in a SubScene; the bakers convert them to blittable `IComponentData` (`PhysicsBody2DDefinition`, `PhysicsShape2D`, and the joint and effector definitions). It reuses the built-in components instead of adding new authoring types.
- **One world per ECS `World`, stepped by the package.** `PhysicsWorld2DSystem` creates the `Unity.U2D.Physics` world with `simulationType = Script`, so the engine never auto-steps it, and calls `Simulate(dt)` once per fixed step. Bodies and shapes are created from the baked components on the main thread; the solver runs inside the engine.
- **Poses are written back to transforms.** `PhysicsBody2DWriteBackSystem` reads every live body's pose in one `PhysicsBody.GetBatchTransform` call, and a Burst job scatters each into the entity's `LocalToWorld`. There is no managed `Transform` write.
- **The simulation is read and driven through ECS types.** Your systems use `PhysicsQueries2D` for spatial queries (ray / overlap / cast / closest-point, with 64-bit layer-mask filtering), the contact / trigger / joint-break event buffers on the `PhysicsWorldSingleton2D` entity to observe results, and `PhysicsBody2DCommands` to apply forces, impulses, torque, and kinematic `MovePosition`.
- **A low-level path exposes the Box2D batch surface directly.** Custom authoring MonoBehaviours and a from-code helper write the runtime components directly, and a bulk-creation seam collapses N identical bodies into one `CreateBodyBatch` call — for cases the built-in components don't cover.

## Feature parity

Targets parity with GameObject 2D physics, checked by GameObject-vs-ECS parity tests (150 / 150 passing). Legend: ✅ at parity · ⚠️ shipped with a documented gap · ❌ not yet. Full per-feature detail with measured evidence is in the [parity matrix](Documentation~/parity-matrix.md).

| Area | Status |
|---|---|
| `Rigidbody2D` — body type, mass, constraints, damping, gravity scale | ✅ |
| Colliders — Circle, Box, Capsule, Polygon, Edge (entity scale baked in) | ✅ |
| Composite & Custom colliders | ✅ |
| `PhysicsMaterial2D` — friction, bounciness, density | ✅ |
| All 9 joints (`*Joint2D`) + joint break | ✅ |
| Collision layers & matrix, queries (ray / overlap / cast) | ✅ |
| Contact & trigger events | ✅ |
| Runtime forces / impulses / torque + kinematic `MovePosition` | ✅ |
| Interpolation & extrapolation | ✅ |
| Continuous collision detection | ⚠️ exact vs a dynamic wall; stricter (safer) vs a static wall |
| Effectors — Area, Point, Surface | ✅ |
| Effector — Buoyancy | ⚠️ AABB submersion approximation (~4 cm at an extreme density) |
| Effector — Platform (one-way) | ⚠️ single body faithful; multi-body & `colliderMask` gaps |
| Simulation config (`PhysicsStep2D`), per-entity teardown | ✅ |
| Custom & low-level (`CreateBodyBatch`) authoring | ✅ |
| Not yet — include/exclude layers, `sleepMode`, full `Collision2D` contact geometry, instance-method queries, live joint re-tuning | ❌ |

## Requirements

- Unity 6000.x with the embedded `Unity.U2D.Physics` (Box2D v3) engine — that engine ships in the editor from `6000.3` onward; the package was developed and validated against `6000.6.0a6`.
- Entities 6.5, Collections 6.5, Burst 1.8.29, Mathematics 1.3.2

## Documentation

- [Overview and concepts](Documentation~/index.md) — what the package is and how a body flows from authoring to `LocalToWorld`.
- [Feature-parity matrix](Documentation~/parity-matrix.md) — the per-feature support state vs GameObject 2D physics, with every known gap and its measured evidence.

## License

MIT — see [LICENSE](LICENSE).
