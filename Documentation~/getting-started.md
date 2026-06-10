# Getting started

This is the minimal path: a SubScene with a `Rigidbody2D` and a `Collider2D` bakes to ECS, simulates over Box2D, and moves the entity's `LocalToWorld`. If you have used GameObject 2D physics, there is nothing new to author here — the difference is what happens at runtime and where the result lands.

## The minimal falling body

1. Create a SubScene (right-click in the Hierarchy → New Sub Scene, or add a `SubScene` component to a GameObject and open it). Authoring inside a SubScene is what makes the package's `Baker<T>` pipeline run over your GameObjects at bake time.
2. Inside the SubScene, add a GameObject with a `Rigidbody2D` (leave `Body Type` at `Dynamic`) and a `CircleCollider2D`. This is the natural ECS shape of "one body, one shape": the GameObject bakes to one entity carrying both a `PhysicsBody2DDefinition` and a `PhysicsShape2D`.
3. Add a second GameObject with only a `BoxCollider2D` and no `Rigidbody2D`, positioned below the first as a floor. A `Collider2D` with no `Rigidbody2D` bakes to a static body, exactly as it is a static body in built-in 2D physics.
4. Enter Play mode. The package creates the Box2D bodies on the first update, steps the world each fixed step, and the dynamic body falls under gravity and rests on the static floor.

## Seeing `LocalToWorld` move

The package's output is `Unity.Transforms.LocalToWorld` on the baked entity, updated every fixed step. The dynamic body's entity has its `LocalToWorld` overwritten with the post-step pose, so a render system or a graphics setup that reads `LocalToWorld` draws the body at its simulated position.

To confirm the simulation is running without a renderer wired up, open the Entities Hierarchy / Inspector (Window → Entities) and watch the falling entity's `LocalToWorld` translation decrease each fixed step until it settles on the floor. Anything that consumes `LocalToWorld` — an `Entities.Graphics` `MaterialMeshInfo` setup, a custom draw system, or your own renderer — sees the same motion.

## Seeding a starting velocity

`Rigidbody2D.linearVelocity` is a runtime-only property and bakes to zero from a saved scene, so a starting velocity is authored on the package's `InitialVelocity2DAuthoring` component instead. Add it alongside the `Rigidbody2D` and set its linear / angular velocity; the body launches at that velocity on creation. See [supported components](authoring-components.md) for the field details.

## Where to go next

- To configure the world (gravity, the solver sub-step count, sleep / continuous-collision toggles), add one `PhysicsStep2DAuthoring` to a GameObject in the SubScene — see [supported components](authoring-components.md) and the [bake contract](bake-contract.md). A scene with no `PhysicsStep2DAuthoring` keeps the Box2D defaults.
- To push, move, or query a body at runtime from your own ECS system, see the [runtime API](runtime-api.md): the `PhysicsBody2DCommands` write-in helpers, the `PhysicsQueries2D` spatial queries, and the contact / trigger / joint-break event buffers.
- To author bodies the built-in components cannot express, or to bulk-spawn many identical bodies in one call, see the [low-level surface](low-level-surface.md) and the [custom-authoring sample](custom-authoring.md).
- Before relying on any feature behaving exactly like its GameObject counterpart, read the [feature-parity matrix](parity-matrix.md).
