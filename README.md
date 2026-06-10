# Zori Entities Physics 2D

**Author 2D physics with the built-in Unity components you already know — and simulate it in DOTS.**

`is.zori.entities.physics2d` lets you drop the familiar `Rigidbody2D` / `Collider2D` / `*Joint2D` / `*Effector2D` components onto GameObjects in a SubScene, bakes them to ECS, steps a Box2D-v3 world each fixed step, and writes every body's pose into `Unity.Transforms.LocalToWorld`. It is renderer-agnostic — its contract ends at `LocalToWorld`, so whatever draws from that matrix is up to you. The package was built to roughly full feature parity with GameObject 2D physics, validated by GameObject-vs-ECS parity tests (150 / 150), with a few [documented known gaps](Documentation~/parity-matrix.md).

## Install

Add the package by git URL (Package Manager → Add package from git URL), or in `Packages/manifest.json`:

```json
"dependencies": {
  "is.zori.entities.physics2d": "https://github.com/<owner>/<repo>.git"
}
```

You can also drop the package into your project's `Packages/` folder as an embedded package. See [Installation](Documentation~/installation.md) for requirements and how to run the tests.

## Quick start

1. Put a GameObject with a `Rigidbody2D` (Dynamic) and a `CircleCollider2D` in a SubScene.
2. Add a `Collider2D`-only GameObject below it as a floor (no `Rigidbody2D` → static body).
3. Enter Play mode. The body falls, rests on the floor, and its entity's `LocalToWorld` updates every fixed step.

That's it — no bespoke authoring concepts. Push, move, and query bodies at runtime from your own ECS systems via `PhysicsBody2DCommands` and `PhysicsQueries2D`, and read contacts/triggers/joint-breaks from the event buffers. See [Getting started](Documentation~/getting-started.md).

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

- Unity 6000.x (developed and validated against `6000.6.0a6`)
- Entities 6.5, Collections 6.5, Burst 1.8.29, Mathematics 1.3.2

## Documentation

- [Overview and concepts](Documentation~/index.md) — what the package is and how a body flows from authoring to `LocalToWorld`.
- [Feature-parity matrix](Documentation~/parity-matrix.md) — the honest, per-feature support state vs GameObject 2D physics, with every known gap and its measured evidence.

## License

MIT — see [LICENSE](LICENSE).
