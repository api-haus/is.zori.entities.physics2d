# Custom authoring (`Samples~/CustomAuthoring2D`)

`CustomAuthoring2D` is the importable sample (registered in `package.json` `samples`, imported through the Package Manager into `Assets/`) that demonstrates the low-level surface and gives you an editable starting point for beyond-built-in authoring. It is shipped as a sample rather than core package code because it is meant to be forked and customised — the "customisable" requirement — so it lands in your `Assets/` where you can edit it. It is the 2D analogue of the DOTS `com.unity.physics` "Custom Physics Authoring" sample, used for its authoring → baker → same-runtime structure only.

## When to use it

Use the custom authoring when the built-in `Rigidbody2D` / `Collider2D` cannot express what you need, or when you want a body authored from explicit geometry without round-tripping through a sprite/mesh-resolved collider. Use the bake path (`authoring-high-level.md`) otherwise — it is the familiar, zero-new-concepts surface.

## What the sample contains

- **`PhysicsBody2DAuthoring`** — a custom body authoring MonoBehaviour exposing, as inspector fields: the package's own `PhysicsBody2DMotionType` body type (so the custom surface depends on no built-in component), gravity scale, linear/angular damping, explicit mass / auto-mass, the three per-DOF freeze toggles, and an **initial linear/angular velocity**. The initial velocity is the knob a built-in `Rigidbody2D` cannot author at bake time, because `Rigidbody2D.linearVelocity` is runtime-only. Its baker emits the same `PhysicsBody2DDefinition` (and the same `PhysicsBody2DInitialVelocity`) the built-in `Rigidbody2DBaker` emits.
- **`PhysicsShape2DAuthoring`** — a custom shape authoring MonoBehaviour carrying the `PhysicsShape2DKind` union directly (Circle / Box / Capsule / Polygon / Edge) with explicit geometry, an offset, and inline friction / bounciness / density. Its baker emits the same `PhysicsShape2D` the built-in collider bakers emit, builds the Polygon/Edge vertex blob the same way, and adds the collider-only static-body fallback when no `PhysicsBody2DAuthoring` is present.
- **`BatchSpawnSampleSystem`** — the worked example of the bulk-creation path (`low-level-surface.md`), spawning a run of identical bodies through `CreateBodyBatch`.

## The convergence property

The load-bearing point of the sample is that the custom MonoBehaviours bake to the *same* runtime archetype the built-in components bake to. A body authored via `PhysicsBody2DAuthoring` and the equivalent `Rigidbody2D`/`Collider2D`-authored body land in one archetype and run one Box2D solver, so the step and write-back systems never ask which surface created a body. The determinism gate measured this directly: a `Baker<PhysicsBody2DAuthoring>` and a `Baker<Rigidbody2D>` fed identical parameters in identical creation order produce a **bit-identical trajectory** (0.0 worst error over 120 steps). Any drift in the custom baker would split the trajectories immediately, which makes the convergence test the cleanest possible falsification gate.

## Authoring-component caveat

The two authoring MonoBehaviours drop `using static Unity.Mathematics.math;` and use `math.max(...)`, deviating from the project's `max()`-over-`math.max()` preference. The static import collides with `float2` in type position inside a MonoBehaviour field initializer (`new float2(...)` parses against the `math.float2` method, CS0119). The runtime ECS structs do not hit this because they do not combine the static import with `new float2(...)` field initializers. This is a deliberate local deviation, justified because these are cold authoring components.
