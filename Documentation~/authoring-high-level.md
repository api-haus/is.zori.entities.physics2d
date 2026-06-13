# High-level authoring (the bake path)

The high-level surface is the built-in 2D physics components you already know. You author a GameObject with `Rigidbody2D`, a `Collider2D`, a `PhysicsMaterial2D`, and `*Joint2D` components, place it in a subscene, and the package's bakers convert it to the ECS runtime at bake time. The design decision is that no bespoke authoring concept is introduced where a built-in one exists — so the only package-shipped authoring components are the two with no built-in equivalent: `InitialVelocity2DAuthoring` (a velocity seed, because `Rigidbody2D.linearVelocity` is runtime-only and bakes to zero) and `PhysicsStep2DAuthoring` (the per-scene simulation config, because there is no built-in component for the world parameters).

This doc covers how a scene is set up. The per-component option reference (what each knob does) is [supported components](authoring-components.md); the exact per-field mapping (what each baker reads and what it ignores) is the [bake contract](bake-contract.md); the minimal worked example is [getting started](getting-started.md).

## Subscene authoring

The standard Entities path is subscene authoring: the GameObjects live in a subscene so the `Baker<T>` pipeline runs over them at bake time and produces entities in the default `World`. A GameObject carrying a `Rigidbody2D` and a `CircleCollider2D` bakes to one entity that carries both `PhysicsBody2DDefinition` and `PhysicsShape2D` — the natural ECS shape of "one body, one shape." A collider-only GameObject (no `Rigidbody2D`) bakes to a static body through the collider baker's static-body fallback.

## Supported authoring components

- **`Rigidbody2D`** — dynamic, kinematic, and static body types; `gravityScale`, linear/angular damping, the freeze `constraints`, `mass` / `useAutoMass`, `collisionDetectionMode` (continuous), and `interpolation` (Interpolate / Extrapolate render-rate smoothing).
- **Initial velocity** — authored on the package's `InitialVelocity2DAuthoring` component, not on `Rigidbody2D` (whose `linearVelocity` is runtime-only and bakes to zero). Add `InitialVelocity2DAuthoring` alongside the `Rigidbody2D` to seed a starting velocity.
- **All five `Collider2D` shapes** — `CircleCollider2D`, `BoxCollider2D`, `CapsuleCollider2D`, `PolygonCollider2D` (convex, path 0), `EdgeCollider2D` (one-sided static surface) — plus their `isTrigger` (sensor) flag and their layer (the collision filter).
- **`CompositeCollider2D`** (Polygons / Outlines) and **`CustomCollider2D`** — multi-shape bodies. A composite's `usedByComposite` (`compositeOperation`) children are merged into one body; their own bakers emit nothing (`bake-contract.md`).
- **`PhysicsMaterial2D`** — friction and bounciness via `Collider2D.sharedMaterial`; density via `Collider2D.density`.
- **Collision layers + the project layer-collision matrix** — `gameObject.layer` + `Physics2D.GetLayerCollisionMask` bake to the shape's contact filter, so a baked body collides with exactly the set a GameObject would.
- **All nine `*Joint2D` components** — Hinge, Slider, Wheel, Distance, Spring, Fixed, Relative, Friction, Target — plus `breakForce` / `breakTorque` / `breakAction` (joint break). A joint's `connectedBody` should reference a concrete body; the null-connected "joint to a point in space" is faithful only for `TargetJoint2D` (`bake-contract.md`).
- **All five `*Effector2D` components** — `AreaEffector2D`, `BuoyancyEffector2D`, `PointEffector2D` (force fields on a sensor collider), `SurfaceEffector2D`, `PlatformEffector2D` (contact-response on a solid collider). See `parity-matrix.md` for the Platform one-way known gaps.
- **Simulation config** — `PhysicsStep2DAuthoring` (one per SubScene/world, package-shipped) configures the world: gravity, the Box2D-v3 solver sub-step count, the worker count, the sleep / continuous-collision toggles, and the contact-solver knobs. Explicit per-scene config; nothing is read from the project `Physics2D` settings, and a scene with no `PhysicsStep2DAuthoring` keeps the Box2D defaults. The field set and the deliberately-omitted knobs (simulation type, fixed timestep) are in `bake-contract.md` (Simulation configuration).

## Runtime surfaces beyond authoring

Some built-in features are not authored on the GameObject — they are runtime API a consumer's ECS system calls: spatial queries (the `Rigidbody2D.Cast`-style raycast / overlap / cast), the contact / trigger / joint-break event callbacks, and the `Rigidbody2D.AddForce` / `MovePosition` runtime write-in. These are `runtime-api.md`, not bake inputs.

## Not supported on this path

The per-collider `Rigidbody2D.includeLayers` / `excludeLayers` overrides, `sleepMode`, and an explicit centre-of-mass / inertia override are not baked on the built-in path; setting them on a GameObject has no effect. The authoritative coverage and the reason each is deferred is in `parity-matrix.md` (still-not-covered). The centre-of-mass / inertia override does have a custom-path home: the `PhysicsBody2DAuthoring` component exposes a 2D mass-distribution override (a `float2` centre of mass + a scalar rotational inertia), documented in [custom authoring](custom-authoring.md). It is a custom-surface feature, not a built-in `Rigidbody2D` field the bake path reads.

## What the package does with a baked body

A baked entity flows through the runtime systems (`runtime-systems.md`): a creation step makes the Box2D body and shape from the baked definition, the world is stepped once per fixed step, and the body's pose is written into `LocalToWorld`. The package writes `LocalToWorld` and stops — wiring a renderer to draw from that matrix is the consumer's job and is out of the package's scope.
