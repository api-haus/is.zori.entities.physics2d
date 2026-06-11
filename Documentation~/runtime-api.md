# Runtime API (queries, events, write-in, config)

This is the programmatic surface a game's own ECS systems call against the package's `PhysicsWorld` — the analogues of `Physics2D.Raycast`, the `OnCollision*2D` / `OnTrigger*2D` / `OnJointBreak2D` callbacks, and `Rigidbody2D.AddForce` / `MovePosition`. The systems that own the world and drive the step are the [runtime systems](runtime-systems.md); the runtime types these surfaces read and write are catalogued in [runtime components](runtime-components.md). The [parity matrix](parity-matrix.md) is the canonical home for the parity verdict on each; this doc covers shape and usage, not the verdict.

## Spatial queries — `PhysicsQueries2D`

A `public static class` of synchronous spatial queries over the package's `PhysicsWorld`, the analogue of `UnityEngine.Physics2D.Raycast` / `OverlapCircle` / `OverlapBox` / `OverlapPoint` / `CircleCast` / `BoxCast` / `CapsuleCast`. A query reads the simulation state, so it is valid only against a stepped world — call it after `PhysicsWorld2DSystem` has run for the frame, or from a job that captured the world handle as a `[ReadOnly]` field. Obtain the world with `SystemAPI.GetSingleton<PhysicsWorldSingleton2D>().world`.

The methods, each taking the `PhysicsWorld`, a `ulong hitLayerMask`, and a caller-owned result container:

- `Raycast(world, origin, direction, distance, hitLayerMask, NativeList<PhysicsQueryHit2D> hits)` — every hit, sorted nearest first; returns the count.
- `RaycastClosest(world, origin, direction, distance, hitLayerMask, out PhysicsQueryHit2D hit)` — the closest hit only; returns whether anything was hit.
- `OverlapCircle(world, center, radius, hitLayerMask, hits)`, `OverlapBox(world, center, size, angleRadians, hitLayerMask, hits)`, and `OverlapCapsule(world, center1, center2, radius, hitLayerMask, hits)` — every shape overlapping the region.
- `OverlapPoint(world, point, hitLayerMask)` — the cheap boolean form (`TestOverlapPoint`).
- `CircleCast(world, origin, radius, direction, distance, hitLayerMask, hits)`, `BoxCast(world, origin, size, angleRadians, direction, distance, hitLayerMask, hits)`, and `CapsuleCast(world, center1, center2, radius, direction, distance, hitLayerMask, hits)` — sweep a shape through the world. The capsule is the two world-space end-cap centers plus the end radius (the `CapsuleGeometry.Create` form), the cast a rounded character sweeps so its caps clear edges a box corner would catch on.

A `hitLayerMask` of `0` or `~0ul` means "hit every layer" (the `Physics2D.DefaultRaycastLayers` / `AllLayers` convention), so a caller that does not care about layers passes `0` and gets all hits rather than none; otherwise the mask is written into `PhysicsQuery.QueryFilter.hitCategories`, the same field the GameObject layer-mask raycast honors.

### Closest point — `ClosestPoint`

`ClosestPoint(world, point, maxDistance, hitLayerMask, NativeList<PhysicsQueryHit2D> hits, out ClosestPoint2D result)` returns the nearest world body to a query point, with the closest point on that body, the separation distance, and the surface normal — the 2D analogue of `com.unity.physics`'s `CollisionWorld.CalculateDistance(PointDistanceInput)`, which the 2D world exposes no single call for. It composes the existing overlap broad-phase (a disc of `maxDistance` finds candidate shapes within range) with the per-pair exact distance (`PhysicsQuery.ShapeDistance` measures the point against each candidate), and returns the closest; `hits` is a caller-owned scratch list the broad-phase reuses. It returns `false` when no body is within `maxDistance`.

The result is a `ClosestPoint2D` value struct — `entity` (the nearest body, or `Entity.Null` if not a package body), `point` (the closest point on the body's surface), `normal` (pointing from the query point toward the body), `distance` (zero when the point is inside the body, where `normal` is then degenerate), and the raw `shape`. A caller that needs a push-out direction from inside a body uses the overlap-then-cast-back path, not this query, because Box2D's `ShapeDistance` reports a zero distance and a degenerate normal for an overlap.

### The hit struct — `PhysicsQueryHit2D`

A hit is a `PhysicsQueryHit2D` value struct (not an `IComponentData`): a query returns hits into a caller-owned container, not onto an entity. Its fields:

- **`entity`** — the entity owning the hit shape's body, resolved via the body's packed `userData`. A hit on a body the package did not pack (such as a shapeless joint world-anchor) is `Entity.Null`, which a caller can filter out.
- **`point`** — the world-space contact point. Filled for cast queries; zero for overlap queries (which have no contact point).
- **`normal`** — the world-space surface normal at the contact. Filled for cast queries; zero for overlaps.
- **`fraction`** — the fraction of the cast distance to the contact, in `[0, 1]`. Filled for cast queries; zero otherwise.
- **`shape`** — the raw Box2D `PhysicsShape`, a same-frame convenience for reading `shapeType` etc.

The query helpers are plain `static` methods (no `[BurstCompile]`, per the entry-point-only rule): they are HPC#-clean, so a caller's own `[BurstCompile]` job can call them and they auto-compile from that context, while a main-thread system calls them managed. `PackEntity` / `UnpackEntity` / `ResolveEntity` are the public entity↔`userData` packing helpers the queries and the event collection share.

## Contact / trigger / joint-break events

Box2D-v3 reports a pair beginning and ending touch as two distinct event lists; there is no per-frame "still touching" event. Each step, `PhysicsWorld2DSystem` drains the engine event spans into three `DynamicBuffer<…>` streams on the `PhysicsWorldSingleton2D` entity, cleared and refilled each step. Read them from a system ordered `[UpdateAfter(typeof(PhysicsWorld2DSystem))]` in `FixedStepSimulationSystemGroup` via `SystemAPI.GetSingletonBuffer<T>(isReadOnly: true)`; the buffer is valid for that tick and cleared at the next.

- **`PhysicsContactEvent2D`** — the `OnCollisionEnter2D` / `OnCollisionExit2D` analogue, for a non-trigger pair. Carries `phase` (`Begin` / `End`), the resolved `entityA` / `entityB`, and the raw `shapeA` / `shapeB`. It is a touch SIGNAL — which pair, beginning or ending, this step — NOT the contact manifold: Box2D-v3's begin/end events carry no contact point / normal / relative velocity (that geometry is on the separate threshold-gated `ContactHitEvent` channel the package does not surface — [parity matrix](parity-matrix.md) still-not-covered).
- **`PhysicsTriggerEvent2D`** — the `OnTriggerEnter2D` / `OnTriggerExit2D` analogue, for a pair where one (or both) shapes is a sensor. Carries `phase`, `triggerEntity` (owns the sensor), `visitorEntity` (owns the shape that entered/left), and the raw shapes. Two overlapping sensors DO produce trigger events (one per sensor's perspective), matching GameObject.
- **`PhysicsJointBreakEvent2D`** — the `OnJointBreak2D` analogue, produced when a joint's reaction exceeds its baked threshold. Carries `jointEntity`, the raw `joint` handle (valid only this tick), and the `breakAction`. The structural reaction (destroy / disable) is done by `PhysicsJoint2DBreakSystem`, not the consumer; the event is the notification.

The `phase` enum (`PhysicsEventPhase2D { Begin, End }`) keeps the surface to one buffer per event kind. The GameObject `OnCollisionStay2D` / `OnTriggerStay2D` "currently touching" set is the begin..end interval a consumer derives by applying `Begin` (insert the pair) and `End` (remove it) — the package does NOT synthesise a per-frame Stay element (it would be an O(touching-pairs) write most consumers do not want). The resolved entities are stable; the raw shape handles are volatile beyond the frame, and an entity that could not be resolved (a shape destroyed since the step) is `Entity.Null`.

## Runtime write-in — `PhysicsBody2DCommands`

The analogue of `Rigidbody2D.AddForce` / `AddForceAtPosition` / `AddTorque`, the `linearVelocity` / `angularVelocity` writes, and `MovePosition` / `MoveRotation`. Each helper appends one `PhysicsBody2DCommand` to a body entity's `DynamicBuffer<PhysicsBody2DCommand>`; `PhysicsWorld2DSystem` drains the buffer onto the Box2D body immediately before the step and clears it, so a command authored this frame drives exactly one step — the same one-shot, per-`FixedUpdate` semantics as the GameObject calls.

Add the buffer once (`EntityManager.AddBuffer<PhysicsBody2DCommand>(entity)`), then each frame get it (`SystemAPI.GetBuffer<PhysicsBody2DCommand>(entity)`) and call the helpers:

- `AddForce(buffer, force)` and `AddForce(buffer, force, PhysicsForceMode2D mode)` — continuous force or instantaneous impulse at the centre of mass.
- `AddForceAtPosition(buffer, force, worldPoint, mode)` — at a world point (off-centre → linear + torque).
- `AddTorque(buffer, torque, mode)`.
- `SetLinearVelocity(buffer, velocity)` (m/s) and `SetAngularVelocity(buffer, degreesPerSecond)` (deg/sec) — direct velocity writes that wake the body.
- `MovePosition(buffer, target)`, `MoveRotation(buffer, targetRadians)`, `MovePositionAndRotation(buffer, targetPosition, targetRadians)` — swept, collision-aware kinematic moves over the next step (`SetTransformTarget`), not teleports.

`PhysicsForceMode2D { Force, Impulse }` is a package-local enum mirroring `UnityEngine.ForceMode2D` (the renderer-agnostic runtime carries no built-in dependency). A buffer (not a single command component) is what lets multiple `AddForce` / `AddTorque` in one frame accumulate the way they do on a GameObject: the commands drain in order before one `Simulate`, and Box2D's own force accumulator sums the continuous-force commands. A force / impulse is mass-scaled by the body's resolved mass inside Box2D (the helpers pass the raw vector, never pre-dividing), a write-in on a frozen DOF is cancelled by the solver, and every force / impulse / move wakes a sleeping body — all matching GameObject. The underlying `PhysicsBody2DCommandKind` enum names the exact Box2D call each helper maps to.

Angular units follow the package's [angular unit convention](angular-units.md): a rotation target is in radians (`MoveRotation` / `MovePositionAndRotation` `targetRadians`), while an angular velocity is in deg/sec (`SetAngularVelocity`'s `degreesPerSecond`). A verbatim port of `rb.MoveRotation(90f)` (degrees) must therefore convert: `MoveRotation(math.radians(90f))`. A separate known gap: a single `MoveRotation` to a far target undershoots in one step on Box2D-v3 — sustained re-issue converges ([parity matrix](parity-matrix.md) known-gap).

The helpers are plain `static` HPC#-clean buffer appends (no `[BurstCompile]`), callable from a Burst job or a main-thread system.

## Reading the simulation config — `PhysicsWorld2DConfig`

The world's simulation parameters are authored per scene with `PhysicsStep2DAuthoring` and baked to the `PhysicsWorld2DConfig` singleton ([supported components](authoring-components.md), [bake contract](bake-contract.md)). It is a consumer-readable `IComponentData`: a system that needs the active gravity, sub-step count, or contact-solver settings reads `SystemAPI.GetSingleton<PhysicsWorld2DConfig>()` (or `TryGetSingleton`, which returns false when no `PhysicsStep2DAuthoring` was authored and the world is on the Box2D `defaultDefinition`). `PhysicsWorld2DConfig.Default` carries the same values `defaultDefinition` exposes, so a consumer wanting the effective parameters in the no-config case can fall back to `Default`. The singleton is read once at world creation to build the world; writing it at runtime does NOT re-apply to the live world (the params are create-time), so it is a read surface, not a runtime-mutation surface.

## Direct authoring from code — `DirectPhysics2DAuthoring`

To author a physics entity entirely from code (no MonoBehaviour, no `Baker<T>`), `DirectPhysics2DAuthoring` writes the runtime `PhysicsBody2DDefinition` + `PhysicsShape2D` (and a seeded `LocalToWorld`) straight onto an entity, after which the same creation loop turns it into a live Box2D body. Use the `EntityManager` overload for an immediate structural change and the `EntityCommandBuffer` overload to author inside a job. This and the bulk-creation request are the [low-level surface](low-level-surface.md).
