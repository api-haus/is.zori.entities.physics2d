# Concepts

This page is the conceptual model of the package: what it is, how a body flows from authoring to a rendered pose, the DOTS design posture it commits to, the single-world model, and the Box2D version reality that defines what "parity" means here. The per-feature verdicts live in the [parity matrix](parity-matrix.md); this page is the shape of the system, not the per-feature support state.

## The authoring → bake → step → `LocalToWorld` model

A body's life has four stages, and the package owns the middle two:

- **Author.** You place a GameObject carrying built-in 2D physics components (`Rigidbody2D`, a `Collider2D`, optionally `*Joint2D` / `*Effector2D` / `PhysicsMaterial2D`) in a SubScene, or you author the package's runtime components directly from code or a custom MonoBehaviour.
- **Bake.** At bake time the package's editor-only `Baker<T>` pipeline converts each built-in component to blittable ECS runtime data: the body parameters to `PhysicsBody2DDefinition`, the collider to `PhysicsShape2D`, a joint to `PhysicsJoint2DDefinition`, an effector to `PhysicsEffector2D`. Baking reads the authoring components and produces components; it does not create Box2D bodies. The exact field mapping is the [bake contract](bake-contract.md).
- **Step.** At runtime the package's systems lazily create one Box2D body (and its shapes and joints) per baked entity, step the world once per fixed step, drain the per-step event spans into buffers, and tear a body down when its entity dies. The full sequence is the [runtime systems](runtime-systems.md) doc.
- **Write back.** After the step the package reads every body's pose in one bulk call and writes it into each entity's `Unity.Transforms.LocalToWorld` with a Burst job. The package writes `LocalToWorld` and stops. Wiring a renderer to draw from that matrix is the consumer's job and is out of the package's scope — which is what "renderer-agnostic" means.

## The DOTS posture

The package commits to a specific set of DOTS design decisions, and several of its properties follow from them. These are intentional and load-bearing — the negative space is as much a part of the design as the positive.

- **The pose is written directly to `LocalToWorld`, not routed through `LocalTransform`.** The physics bodies are flat and unparented, so the post-step pose IS the world matrix; there is no parent to compose against. The write-back system writes `LocalToWorld` under a `[WriteGroup(typeof(LocalToWorld))]`, so `LocalToWorldSystem` yields ownership of these entities' matrices and the direct write avoids the one-fixed-step recompose latency a `LocalTransform` round-trip would add.
- **No managed-`Transform` write-back.** The engine's Box2D managed-Transform tween (`TransformWriteMode.Interpolate`) is never enabled. Interpolation and extrapolation are done in ECS over the stored poses by a render-rate smoothing job, holding the DOTS posture rather than reaching back into a managed `Transform`.
- **Bulk transforms.** Poses are read with one `PhysicsBody.GetBatchTransform` call over all bodies and written with one Burst job, rather than a per-body managed read. The only two Burst jobs in the package are this write-back job and the smoothing job; the per-step force / event / effector / teardown work is managed `Unity.U2D.Physics` calls inlined into the systems on the main thread.
- **The package owns stepping (`SimulationType.Script`).** The world is created with `simulationType = Script`, which stops the engine from auto-stepping it. The ECS system is the sole stepper, calling one explicit `PhysicsWorld.Simulate(dt)` per fixed step. Any auto-stepping mode (`FixedUpdate` / `Update`) would make the engine integrate the world on top of the package's step — a double-step — which is why the simulation type is not a configurable field.

## The single-world model

The package owns exactly one `PhysicsWorld` per ECS `World`, created lazily on the first update and destroyed when the ECS world is destroyed. It is a private world, not the engine's shared `PhysicsWorld.defaultWorld`, so a shared default world cannot couple the package to anything else in the project — which is what holds the standalone contract.

The lazy creation is load-bearing: a `PhysicsWorld` created at system-create time does not survive the scene-load / PlayMode-enter physics-module reset, so the owning system ensures a valid world at each update — creating it on the first update and recreating it if a module reset invalidated the handle.

One consequence is that world-index sharding and a multi-world model are deferred. `WorldIndex` is deliberately not exposed on the runtime ([low-level surface](low-level-surface.md)); it is the natural additive extension if the multi-world infrastructure ever lands.

## The Box2D v2-vs-v3 reality, and what "parity" means

In editor `6000.6.0a6` the GameObject `Physics2D.Simulate` path runs the **Box2D-v2 iteration** solver, while this package runs the **Box2D-v3 sub-stepping** solver. They are two different integrators that converge to the same physics but do not produce bit-identical trajectories — free-fall agrees to a constant integration-convention offset of about `1.5e-3 m/step` with exactly-zero angle error, and the contact phase is bounded but genuinely noisier (restitution is where they differ most).

So "parity" here is a precise, bounded claim, not an exact-trajectory match:

- A parity pass means the package behaves like built-in GameObject 2D physics **within a measured band** and commits none of the disqualifying bugs (it moved when it should, no NaN, correct static-vs-dynamic classification, stays in-plane, settles in the right region). The disqualifiers carry the correctness load; the band absorbs the cross-solver difference.
- **Binary facts are asserted exactly** — does a layer pair collide, does a body tunnel, is a body in a query hit-set, did a joint break, is an off-mask body driven. These do not get a band.
- A few comparisons are v3-vs-v3 and genuinely exact rather than bounded: the custom-authoring-vs-built-in-baker trajectory (the same solver on both sides), the body-teardown native counts, and the composite/custom shape-count witnesses.

The measured worst-error numbers in the parity matrix are alpha-pinned to this editor. The v2-vs-v3 split is also the root cause of three of the documented known gaps (the static-wall CCD divergence, the single-step kinematic-rotation undershoot, and — partly — the contact-event noise). If a future editor migrates the GameObject path onto the v3 solver, the two paths become the same solver, agreement collapses toward bit-level, and the bands must be re-derived. The full discussion is in the parity matrix under "The parity oracle is bounded, not exact."
