# Zori Entities Physics 2D

**A DOTS-native 2D physics engine for Entities, bound to Unity's embedded Box2D v3.**

`is.zori.entities.physics2d` is a bindings and integration layer: it presents Unity's embedded Box2D-v3 engine — the low-level `Unity.U2D.Physics` surface (`PhysicsWorld`, `PhysicsBody`, `PhysicsShape`, the geometry and query types) that ships inside the Unity editor from `6000.3` onward — to Entities (Unity DOTS / ECS) as a first-class 2D physics engine. The engine itself is Unity's; this package owns the binding: it bakes authoring to ECS runtime components, owns and steps one `PhysicsWorld` per ECS `World` inside `FixedStepSimulationSystemGroup`, and writes every body's pose back into `Unity.Transforms.LocalToWorld`. It is renderer-agnostic — its contract ends at `LocalToWorld`, so whatever draws from that matrix is up to you.

The package is modelled after `com.unity.physics`, Unity's DOTS physics package. It follows that package's authoring → baking → stepping → query shape: a `PhysicsWorldSingleton2D` holds the one world handle the way Unity Physics' `PhysicsWorldSingleton` does, spatial queries run over that world from inside Burst jobs the way the engine's own `PhysicsQueryJob` does, and the query surface mirrors Unity Physics call-for-call where a 2D analogue exists (`PhysicsQueries2D.ClosestPoint` is the 2D form of `CollisionWorld.CalculateDistance(PointDistanceInput)`). It is the 2D, Box2D-backed sibling of Unity Physics rather than a from-scratch solver — the simulation math is Box2D's, exposed to ECS rather than reimplemented.

On top of that binding, the package was built to roughly full feature parity with GameObject 2D physics, validated by GameObject-vs-ECS parity tests (150 / 150), with a few [documented known gaps](Documentation~/parity-matrix.md).

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

That's it — no bespoke authoring concepts. Push, move, and query bodies at runtime from your own ECS systems via `PhysicsBody2DCommands` and `PhysicsQueries2D`, and read contacts/triggers/joint-breaks from the event buffers. See [Getting started](Documentation~/getting-started.md).

## How it binds Box2D to ECS

- **Authoring is baked to ECS components.** Drop the familiar `Rigidbody2D` / `Collider2D` / `*Joint2D` / `*Effector2D` / `PhysicsMaterial2D` on GameObjects in a SubScene; the package's bakers convert them to blittable `IComponentData` (`PhysicsBody2DDefinition`, `PhysicsShape2D`, the joint and effector definitions). No new authoring vocabulary is introduced where a built-in component already expresses the concept.
- **One world is owned and stepped per ECS `World`.** `PhysicsWorld2DSystem` creates the `Unity.U2D.Physics` world with `simulationType = Script` — so the engine never auto-steps it — and issues exactly one `Simulate(dt)` per fixed step, making the ECS system the sole stepper. Box2D bodies and shapes are created from the baked components on the main thread; the simulation math runs inside the engine.
- **Poses are written back to transforms.** `PhysicsBody2DWriteBackSystem` reads every live body's pose in one bulk `PhysicsBody.GetBatchTransform` call and a Burst job scatters each into the entity's `LocalToWorld`. There is no per-body managed `Transform` write; the binding's output is the ECS transform matrix.
- **The simulation is read through public ECS types.** Spatial queries (`PhysicsQueries2D` — ray / overlap / cast / closest-point with 64-bit layer-mask filtering), contact / trigger / joint-break event buffers on the `PhysicsWorldSingleton2D` entity, and a runtime write-in command buffer (`PhysicsBody2DCommands` — forces, impulses, torque, kinematic `MovePosition`) are how a consumer's own ECS systems drive and observe the bound engine.
- **A low-level path exposes the Box2D batch surface directly.** Custom authoring MonoBehaviours and a direct-from-code helper write the runtime components straight, and a bulk-creation seam collapses N identical bodies into one native `CreateBodyBatch` call — the cases the built-in components cannot express.

## Feature parity

Built to roughly full parity with GameObject 2D physics, validated by GameObject-vs-ECS parity tests (150 / 150). Legend: ✅ at parity · ⚠️ shipped with a documented gap · ❌ not yet. Full per-feature detail with measured evidence is in the [parity matrix](Documentation~/parity-matrix.md).

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
- [Feature-parity matrix](Documentation~/parity-matrix.md) — the honest, per-feature support state vs GameObject 2D physics, with every known gap and its measured evidence.

## License

MIT — see [LICENSE](LICENSE).
</content>
</invoke>
