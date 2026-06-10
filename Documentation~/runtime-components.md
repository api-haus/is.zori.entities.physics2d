# Runtime components

This is the catalogue of the runtime ECS types — the blittable `IComponentData`, `IBufferElementData`, and `ICleanupComponentData` that the systems consume and a consumer reads or writes. The systems that drive them are the [runtime systems](runtime-systems.md); the static API surfaces (queries, commands, query-hit struct) are the [runtime API](runtime-api.md). All runtime types are in the `Zori.Entities.Physics2D` namespace and carry no `UnityEngine.*` reference, so they reach a player build.

## Body components

These live on a body entity. A baked entity (or a directly-authored one) starts with the definition components; the system adds the live-handle and cleanup components at creation.

- **`PhysicsBody2DDefinition`** — the baked body parameters consumed once at creation: `bodyType`, `gravityScale`, `linearDamping` / `angularDamping`, `mass` / `useAutoMass`, `constraints`, `initialPosition`, `initialRotationRadians` (radians, per the [angular unit convention](angular-units.md)), `fastCollisions` (continuous CD), and `interpolation` (a `PhysicsBody2DInterpolation`: `None` / `Interpolate` / `Extrapolate`).
- **`PhysicsBody2DInitialVelocity`** — the optional starting-velocity seed (linear m/s, angular deg/sec), applied to the body at creation. Authored by `InitialVelocity2DAuthoring` or the custom body authoring; absent on a body with no seed.
- **`PhysicsShape2D`** — the baked geometry, a tagged union over `PhysicsShape2DKind` (`Circle` / `Box` / `Capsule` / `Polygon` / `Edge`) plus the surface fields (`friction` / `bounciness` / `density`), the baked contact-filter `categoryBits` / `contactBits`, the `isTrigger` sensor flag, and the `polygonDecompose` flag for a concave / large polygon. Variable-length kinds (Polygon / Edge) reference a vertex blob.
- **`PhysicsShape2DElement`** — an optional `IBufferElementData` of the EXTRA shapes on a multi-shape (composite / custom) body. A one-shape body carries none, so the historical single-shape archetype is unchanged.
- **`PhysicsBody2D`** — the live association between the entity and its `Unity.U2D.Physics.PhysicsBody` handle, added by the creation system (not a baker). Its presence is also the "already created" marker the creation query filters on with `WithNone<PhysicsBody2D>`.
- **`PhysicsBody2DCleanup`** — an `ICleanupComponentData` holding a copy of the same handle. ECS retains it after the entity is destroyed (when the regular `PhysicsBody2D` is stripped), so it is the surviving witness the cleanup system frees the body from.
- **`PhysicsBody2DSmoothing`** — the per-body interpolation state (previous / current pose as position + `(cos, sin)`, plus the post-step linear and angular velocity) for a body whose `interpolation != None`. The render-rate smoothing job reads it; a `None` body carries none.

## Joint components

These live on a joint-owning entity (the entity carrying the joint is Box2D `bodyB`).

- **`PhysicsJoint2DDefinition`** — the baked joint, a tagged union over `PhysicsJoint2DKind` (the nine built-in joint identities) plus the joint's parameters, the connected `Entity`, and the shared break parameters (`breakForce` / `breakTorque` / `breakAction`, a `PhysicsJointBreakAction2D`).
- **`PhysicsJoint2D`** — the live `Unity.U2D.Physics.PhysicsJoint` handle and the created-marker.
- **`PhysicsJoint2DBroken`** — a zero-size tag added to an entity whose joint broke under a `Destroy` / `Disable` action, so the creation query never re-creates it.

## Effector component

- **`PhysicsEffector2D`** — the baked effector, a tagged union over `PhysicsEffector2DKind` (`Area` / `Buoyancy` / `Point` / `Platform` / `Surface`) plus the per-kind fields. Lives on the effector entity, whose own collider bakes to a `PhysicsShape2D` normally (sensor for the force-field family, solid for the contact-response family).

## Event buffers

These are `DynamicBuffer<…>` streams on the `PhysicsWorldSingleton2D` entity, cleared and refilled each step, valid for the tick after the world system runs. They are the package's analogue of the `OnCollision*2D` / `OnTrigger*2D` / `OnJointBreak2D` callbacks; their fields and read pattern are in the [runtime API](runtime-api.md).

- **`PhysicsContactEvent2D`** — a non-trigger pair beginning or ending touch (`phase`, `entityA` / `entityB`, raw `shapeA` / `shapeB`).
- **`PhysicsTriggerEvent2D`** — a sensor pair beginning or ending overlap (`phase`, `triggerEntity` / `visitorEntity`, raw shapes).
- **`PhysicsJointBreakEvent2D`** — a joint whose reaction exceeded its threshold (`jointEntity`, raw `joint`, `breakAction`).
- **`PhysicsEventPhase2D`** — the `Begin` / `End` enum shared by the contact and trigger events.

## Write-in buffer

- **`DynamicBuffer<PhysicsBody2DCommand>`** — the optional per-body runtime write-in queue, appended through the `PhysicsBody2DCommands` helpers and drained onto the body before each step ([runtime API](runtime-api.md)). A body with no command buffer costs nothing. The per-command kinds are a `PhysicsBody2DCommandKind`; the force mode is a `PhysicsForceMode2D`.

## World and config singletons

These live on per-world singleton entities.

- **`PhysicsWorldSingleton2D`** — holds the one `Unity.U2D.Physics.PhysicsWorld` handle this ECS world owns. Other systems and queries reach the world through it (`SystemAPI.GetSingleton<PhysicsWorldSingleton2D>().world`).
- **`PhysicsWorld2DConfig`** — the optional per-world simulation config (gravity, sub-step count, worker count, sleep / continuous-collision toggles, contact-solver knobs), baked from a `PhysicsStep2DAuthoring`. A consumer-readable singleton; read once at world creation, so it is a read surface, not a runtime-mutation surface ([runtime API](runtime-api.md)). `PhysicsWorld2DConfig.Default` carries the same values the Box2D `defaultDefinition` exposes.
- **`PhysicsFixedStepTime2D`** — the most-recent fixed-step time (elapsed time + delta time), written each stepping frame and read by the render-rate smoothing system. The 2D analogue of `com.unity.physics`'s `MostRecentFixedTime`.

## Bulk-creation request

- **`PhysicsBody2DBatchRequest`** — a one-shot request to bulk-create many identical dynamic circle bodies in a single native `CreateBodyBatch` call (count, shared body parameters, circle radius, density, a deterministic scatter AABB + seed, the shared contact filter, and a sensor flag). The optimisation seam for the many-identical-bodies case — see the [low-level surface](low-level-surface.md).
