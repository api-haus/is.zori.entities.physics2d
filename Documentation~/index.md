# Zori Entities Physics 2D

`is.zori.entities.physics2d` is a bindings layer, not a from-scratch solver: it exposes Unity's embedded Box2D v3 — the low-level `Unity.U2D.Physics` engine (`PhysicsWorld`, `PhysicsBody`, `PhysicsShape`, the geometry and query types) that ships inside the editor from `6000.3` onward — to Entities (Unity DOTS / ECS) as a first-class, DOTS-native 2D physics engine. The simulation math is Box2D's; this package owns the binding and is modelled after `com.unity.physics`, following that package's authoring → baking → stepping → query shape. The package README is the canonical statement of that identity; this document is the conceptual overview of how a body flows through the binding.

Concretely, you author a scene with the built-in Unity 2D physics components you already know — `Rigidbody2D`, the `Collider2D` family, the `*Joint2D` family, the `*Effector2D` family, and `PhysicsMaterial2D` — the package bakes them to ECS runtime data, owns and steps one `Unity.U2D.Physics` world per ECS `World` each fixed step, and writes every body's pose into `Unity.Transforms.LocalToWorld`. It is renderer-agnostic: its contract ends at `LocalToWorld`, and whatever consumer draws from that matrix is out of scope.

The package was built to roughly full feature parity with GameObject 2D physics, validated by threshold-based GameObject-vs-ECS parity tests: the GameObject-parity gate suite passes 150 / 150 PlayMode on two consecutive green runs with bit-identical deterministic witnesses, with a handful of documented known gaps. The complete package test suite is larger — 188 PlayMode / 69 EditMode — because the custom-authoring surface (`PhysicsBody2DAuthoring` / `PhysicsShape2DAuthoring` / `PhysicsJoint2DAuthoring` + the 2D-native editor) adds convergence, editor-math, and behavioral-parity tests that have no GameObject oracle; the [parity matrix](parity-matrix.md) explains which number means what. That matrix is the single source of truth for every per-feature verdict — at-parity, at-parity-with-a-known-gap, and still-not-covered. Read it before assuming any feature behaves identically to its GameObject counterpart.

## What the package gives you

- **The familiar authoring surface.** Drop a `Rigidbody2D` and a `Collider2D` on a GameObject in a SubScene and it bakes and simulates. No bespoke authoring concept is introduced where a built-in one already exists, so the bake path carries zero new vocabulary and the full GameObject-physics familiarity. The two package-shipped authoring components are the two with no built-in equivalent: `InitialVelocity2DAuthoring` (a velocity seed) and `PhysicsStep2DAuthoring` (the per-scene simulation config).
- **A DOTS-native runtime.** Bodies live as entities, the simulation runs as ECS systems in `FixedStepSimulationSystemGroup`, and each body's pose is written into `LocalToWorld` in bulk with Burst — no per-body managed `Transform` write-back. A game's own ECS systems read the simulation through public types: spatial queries, contact / trigger / joint-break event buffers, and a runtime write-in command buffer.
- **A low-level path for the cases the built-in components cannot express.** Custom authoring MonoBehaviours and a direct-from-code authoring helper write the runtime components straight, and a bulk-creation seam turns N identical bodies into one native batch call.

## The two interaction surfaces

The package exposes two ways to create bodies over one runtime, and they converge on the same `IComponentData` archetype, so the step and write-back systems never ask which surface created a body.

- **High-level (declarative bake).** Author with the built-in components in a SubScene; the package's editor-only bakers convert them to the runtime components. This is the parity surface, and its ceiling is exactly what the built-in components can express. The per-component field mapping is the [bake contract](bake-contract.md); how a scene is set up is [high-level authoring](authoring-high-level.md); the per-component option reference is [supported components](authoring-components.md).
- **Low-level (direct ECS).** Two capabilities, both shipped as the `CustomAuthoring2D` sample: custom `PhysicsBody2DAuthoring` / `PhysicsShape2DAuthoring` / `PhysicsJoint2DAuthoring` MonoBehaviours that author the runtime components directly — the complete surface (an initial velocity seed, free box/capsule orientation, a `PhysicsMaterial2D` material template with per-field override, an explicit contact-filter bitset, a custom mass-distribution override, interpolation, CCD, a Collide/Sensor response, and a unified nine-kind joint selector the built-in components cannot all express) with a 2D-native scene-view editor over all three (kind-switched custom inspectors, draggable handles for shapes and joint anchors/axis/limits, gizmo outlines, a "Fit To…" dropdown, and component icons) — and a direct/bulk creation seam onto the `Unity.U2D.Physics` batch API for the many-identical-bodies case. See the [low-level surface](low-level-surface.md) and the [custom-authoring sample](custom-authoring.md).

A scene can mix the two: a built-in-authored body and a custom-authored body coexist in one world because they bake to the same archetype, and the determinism gate measured them bit-identical.

## Documentation map

Conceptual:

- [Concepts](concepts.md) — the authoring → bake → step → `LocalToWorld` model, the DOTS posture, the single-world model, and the Box2D v2-vs-v3 reality and what it means for "parity."
- [Installation and requirements](installation.md) — Unity 6000.x / Entities 6.5, the embedded-package and UPM-git-URL install paths, and the `testables` note for running the package tests.
- [Getting started](getting-started.md) — the minimal path: a SubScene with a `Rigidbody2D` and a `Collider2D` bakes, simulates, and moves its `LocalToWorld`.
- [Angular unit convention](angular-units.md) — radians for body rotation, degrees for angular velocity and joint angles, stated once and referenced everywhere.

Reference — authoring:

- [Supported components](authoring-components.md) — a section per authoring surface documenting each supported component and its options: bodies, colliders, joints, effectors, materials, the initial-velocity seed, and the simulation-config component.
- [High-level authoring](authoring-high-level.md) — how a scene is set up for baking.
- [Bake contract](bake-contract.md) — the canonical per-component field mapping: which fields each baker reads, what they map to in the runtime and in Box2D, and which fields are deliberately ignored.

Reference — runtime:

- [Runtime systems and the step model](runtime-systems.md) — the systems, the per-step sequence, the world lifecycle, body teardown, render-rate smoothing, and the update order.
- [Runtime components](runtime-components.md) — the catalogue of runtime `IComponentData` and `DynamicBuffer` types a consumer reads or writes.
- [Runtime API](runtime-api.md) — the programmatic surfaces: spatial queries, the event buffers, the runtime write-in commands, and the simulation-config read surface.
- [Low-level surface](low-level-surface.md) — direct-from-code authoring and the bulk-creation seam.
- [Custom-authoring sample](custom-authoring.md) — the `Samples~/CustomAuthoring2D` sample.

Parity:

- [Feature-parity matrix](parity-matrix.md) — the single source of truth for per-feature support state, the exercising test, and the measured worst error, including each known gap's measured evidence and root cause.
