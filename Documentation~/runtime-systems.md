# Runtime systems

The runtime owns the `PhysicsWorld`, creates Box2D bodies and joints lazily from the baked components, applies per-step write-in and effector forces, steps the world once per fixed step, surfaces contact / trigger / joint-break events, tears down a body when its entity dies, and writes each body's pose into `LocalToWorld`. Five systems live in `FixedStepSimulationSystemGroup` plus one render-rate smoothing system in `TransformSystemGroup`; only two Burst jobs exist (the write-back and the smoothing job). None of the fixed-step systems is `[BurstCompile]` — they call managed `Unity.U2D.Physics` instance methods on the main thread, which is why the per-step force / event / effector work is inlined into the stepper rather than scheduled as jobs.

## Runtime component set

The runtime state is blittable `IComponentData` plus three `DynamicBuffer<…>` event streams and the per-body command / smoothing components.

Core, on each body entity:

- **`PhysicsBody2DDefinition`** — the baked body parameters, consumed once at body creation: body type, gravity scale, damping, mass / auto-mass, constraints, initial pose, `fastCollisions` (continuous CD), and `interpolation`.
- **`PhysicsShape2D`** — the baked geometry, a tagged union over `PhysicsShape2DKind` (Circle / Box / Capsule / Polygon / Edge) plus the surface fields (friction / bounciness / density), the baked contact-filter `categoryBits` / `contactBits`, the `isTrigger` sensor flag, and the `polygonDecompose` flag for a concave / large polygon.
- **`PhysicsShape2DElement`** — an optional `IBufferElementData` of EXTRA shapes for a multi-shape (composite / custom) body. A one-shape body carries none, so the historical single-shape archetype is unchanged.
- **`PhysicsBody2D`** — the live association between an entity and its `Unity.U2D.Physics.PhysicsBody` handle, added by the creation system. It is also the "already created" marker the creation query filters on with `WithNone<PhysicsBody2D>`, and the body's `userData` is packed with the owning entity at creation so a query hit / event shape resolves back to it.
- **`PhysicsBody2DCleanup`** — an `ICleanupComponentData` holding a copy of the same `PhysicsBody` handle, added in the same operation as `PhysicsBody2D`. ECS retains it after the entity is destroyed (when the regular `PhysicsBody2D` is stripped), so it is the surviving witness of the handle the cleanup system frees.

The joint surface adds `PhysicsJoint2DDefinition` (the baked joint) and `PhysicsJoint2D` (the live `PhysicsJoint` handle and created-marker), plus `PhysicsJoint2DBroken` (a zero-size tag stopping a broken joint from re-forming). The effector surface adds `PhysicsEffector2D` (the baked effector definition) on the effector entity.

Optional, per body:

- **`DynamicBuffer<PhysicsBody2DCommand>`** — the runtime write-in queue (`PhysicsBody2DCommands` appends `AddForce` / `AddTorque` / velocity / MovePosition commands); drained onto the body and cleared each step.
- **`PhysicsBody2DSmoothing`** + the **`PhysicsFixedStepTime2D`** singleton — the interpolation state for a body whose `interpolation != None`.

Per-world, on the singleton entity:

- **`PhysicsWorldSingleton2D`** — the `PhysicsWorld` handle.
- **`DynamicBuffer<PhysicsContactEvent2D>`**, **`DynamicBuffer<PhysicsTriggerEvent2D>`**, **`DynamicBuffer<PhysicsJointBreakEvent2D>`** — the per-step event streams, cleared and refilled each step.
- **`PhysicsWorld2DConfig`** — the optional per-world simulation config (gravity, sub-step count, worker count, sleep / continuous-collision toggles, contact-solver knobs), on its own baked config entity (not the world-handle singleton). Read once at world creation; absent → the Box2D `defaultDefinition` is used. See "Simulation configuration" below.

## World lifecycle

`PhysicsWorld2DSystem` owns one `PhysicsWorld` per ECS `World`, created LAZILY at the top of `OnUpdate` (not in `OnCreate`) and destroyed in `OnDestroy`. The lazy creation is load-bearing: a `PhysicsWorld` created at system-create time does not survive the scene-load / PlayMode-enter physics-module reset, so the system ensures a valid world at each update — creating the singleton on the first update and recreating the world if the handle was invalidated by a module reset. Destroying the world destroys all its bodies, shapes, and joints. The world is created with `simulationType = Script`, which stops the engine from auto-stepping it and makes the ECS system the sole stepper, and from `PhysicsWorldDefinition.defaultDefinition` (so the world-level `continuousAllowed` stays on by default — Dynamic-vs-Static CCD is free). The handle is published as `PhysicsWorldSingleton2D`; the three event buffers ride that singleton entity. The package owns a private world rather than the engine's `PhysicsWorld.defaultWorld` so a shared default world cannot couple the package to anything else in the project, holding the standalone contract.

### Simulation configuration

The world's parameters are configurable per scene through the optional `PhysicsWorld2DConfig` singleton, baked from a `PhysicsStep2DAuthoring` in a SubScene (`bake-contract.md`). At each `CreateWorld` call the system resolves the singleton: when present, it overrides the gravity, the Box2D-v3 solver sub-step count, the worker count, the sleep / continuous-collision toggles, and the contact-solver knobs on top of `defaultDefinition`; when absent, the world is built straight from `defaultDefinition` — the exact world shipped before this surface existed, so a scene with no `PhysicsStep2DAuthoring` is unchanged. The config is read fresh at every `CreateWorld` (so a world recreated after a module reset re-reads the current config), and `simulationType` is forced to `Script` regardless of the config — the simulation type is deliberately not a config field, because the package owns stepping with the explicit `Simulate(dt)` and any auto-stepping mode would double-step. The fixed timestep is likewise not configured here: it is the `FixedStepSimulationSystemGroup` rate, an ECS-global group property shared by every system in that group, not a per-physics-world value. The package is single-world, so exactly one `PhysicsWorld2DConfig` is expected per `World`; more than one surfaces as a singleton-query throw at world creation.

## Body and joint creation

Creation is lazy and deferred, driven off the baked components.

- **Bodies** are created in `PhysicsWorld2DSystem.OnUpdate` for any entity that has a `PhysicsBody2DDefinition` and a `PhysicsShape2D` but not yet a `PhysicsBody2D`. The system creates the Box2D body and its primary shape (applying the surface material, contact filter, sensor flag, the explicit/auto mass, the initial-velocity seed, the CCD flag, and the chain-only mass floor), then attaches every `PhysicsShape2DElement` extra shape to the same body via the same `CreateShapeForBody` path (so a multi-shape body's mass is the sum over its shapes), packs the entity into the body's `userData`, adds `PhysicsBody2D` + `PhysicsBody2DCleanup` (and a seeded `PhysicsBody2DSmoothing` when interpolated) through an `EntityCommandBuffer`. The update that first creates a body does not step it, so the body sits at its authored pose for one update; the next update steps it.
- **Joints** are created in `PhysicsJoint2DCreationSystem`, `[UpdateBefore(PhysicsWorld2DSystem)]`. A joint references a second body, so it cannot be created until both bodies have live `PhysicsBody2D` handles; the system re-checks each update (query `WithAll<PhysicsJoint2DDefinition> WithNone<PhysicsJoint2D, PhysicsJoint2DBroken>`) and creates the joint once both exist, so the first step that integrates a jointed body already honours the constraint. At creation it packs the owner entity into `joint.userData` and, when the action is not `Ignore` and the threshold is finite, sets the native `forceThreshold` / `torqueThreshold` for joint break. A null connected body resolves to a lazily-created static world anchor at the origin (`bake-contract.md`).

## The step

`PhysicsWorld2DSystem.OnUpdate` does the whole per-step sequence in one method, in this order, only on a non-creation frame (a frame that created bodies does not step them):

1. **Apply runtime commands** — drain every `WithAll<PhysicsBody2D, PhysicsBody2DCommand>` entity's buffer onto its body (`ApplyForce*` / `ApplyLinearImpulse*` / `ApplyTorque` / velocity set / `SetTransformTarget`), then clear the buffer. One-shot: a command applies exactly once at one step.
2. **Apply effectors** — iterate `WithAll<PhysicsBody2D, PhysicsEffector2D, PhysicsShape2D>`, and per kind: a force-field effector (Area / Buoyancy / Point) overlap-queries its sensor region and applies the force to every dynamic body inside; a Platform effector gates its own body's `enabled` one-way; a Surface effector reads `GetContacts` and drives each contacting body tangentially. All before the step, so an effector force accumulates and is mass-scaled / frozen-axis-cancelled like a command force.
3. **`world.Simulate(dt)`** — exactly one step. Because the system lives in `FixedStepSimulationSystemGroup`, `dt` is the group's fixed timestep and the group's catch-up manager sub-steps the whole group to track wall-clock with a constant `dt` — deterministic, framerate-independent physics with no bespoke sub-stepping; Box2D's own internal sub-steps stay at the world default.
4. **Collect events** — immediately after `Simulate`, while the volatile `ReadOnlySpan` event accessors are still valid, drain `world.contactBeginEvents` / `contactEndEvents` / `triggerBeginEvents` / `triggerEndEvents` / `jointThresholdEvents` into the three singleton buffers, resolving each shape / joint to its owning entity via the packed `userData`. The spans are cleared on the next world mutation, so the read happens in the same method before anything else touches the world. The buffers are cleared at the top of each step, so a creation (no-step) frame leaves them empty.
5. **Record the fixed-step time** into `PhysicsFixedStepTime2D` for the render-rate smoothing system.

## Per-entity body teardown

`PhysicsBody2DCleanupSystem` (`[UpdateBefore(PhysicsWorld2DSystem)]`) frees the Box2D body of a destroyed entity. Its query is `WithAll<PhysicsBody2DCleanup> WithNone<PhysicsBody2D>` — a destroyed-entity "ghost," which ECS leaves addressable through the retained cleanup component after stripping the regular `PhysicsBody2D`. It collects the ghost handles, calls `PhysicsBody.DestroyBatch` (which cascades to the body's shapes AND any attached joints), and removes the last cleanup component so ECS reclaims the entity. Because it runs before the stepper, a body destroyed on frame N is freed at the top of frame N+1, before that step integrates it — so a dead body is simulated for zero further steps and is never written back (write-back queries `PhysicsBody2D`, which a ghost no longer has). `OnDestroy` drains any remaining ghosts at world teardown. This closes the per-despawn body leak; the live body count returns to baseline under churn.

## Joint break

When a joint's reaction force / torque exceeds its baked threshold, Box2D fires a `jointThresholdEvents` entry (it does NOT itself destroy the joint). `PhysicsWorld2DSystem.CollectJointBreaks` reads that volatile span after `Simulate`, resolves each joint to its owner via `joint.userData`, reads the baked `breakAction` (a pure ECS read, span-safe), and appends a `PhysicsJointBreakEvent2D` to the singleton break buffer. The structural reaction — a destroy / disable is illegal mid-span-read — is deferred to `PhysicsJoint2DBreakSystem` (`[UpdateAfter(PhysicsWorld2DSystem)]`): for a `Destroy` / `Disable` action it `DestroyJointBatch`es the handle, removes `PhysicsJoint2D`, and adds the `PhysicsJoint2DBroken` tag (so the joint never re-forms); a `CallbackOnly` action leaves the joint in place. The break-event buffer is consumer-readable for one tick — the `OnJointBreak2D` analogue.

## Write-back

`PhysicsBody2DWriteBackSystem` runs `[UpdateAfter(PhysicsWorld2DSystem)]`, on the just-stepped poses. It makes one pass over the `PhysicsBody2D` query to build two index-aligned arrays — the body handles and the matching entities — reads all poses in one bulk `PhysicsBody.GetBatchTransform` call, and schedules `BatchTransformToLocalToWorldJob` (the first `[BurstCompile]` entry point) to convert each `BatchTransform` to a `float4x4` and scatter it into that entity's `LocalToWorld` via a `ComponentLookup`. The per-step native arrays are `Allocator.TempJob`, disposed after the job completes. For an interpolated body it also captures the cur→prev pose shift and the post-step velocity into `PhysicsBody2DSmoothing` on the main thread (a managed velocity read, not in the Burst job).

The pose is written to `LocalToWorld` directly under a `[WriteGroup(typeof(LocalToWorld))]` so `LocalToWorldSystem` yields ownership of these entities' matrices. For the flat, unparented physics bodies this is cleaner than routing through `LocalTransform`: the matrix is what the pose already is, there is no parent to compose against, and the direct write avoids the one-fixed-step recompose latency. The write-back addressing inherits the POC's assumption that `GetBatchTransform` returns results in input-span order.

## Render-rate interpolation

`PhysicsBody2DSmoothingSystem` (`[UpdateInGroup(TransformSystemGroup)] [UpdateBefore(LocalToWorldSystem)]`) smooths the rendered pose of an interpolated body between fixed steps, at the variable render rate. It reads `PhysicsFixedStepTime2D` and its own `SystemAPI.Time.ElapsedTime` to compute how far the render time is ahead of the last physics step, then a `[BurstCompile] IJobChunk` over `WithAll<PhysicsBody2DSmoothing, LocalToWorld>` overwrites `LocalToWorld` with the interpolated (prev→cur lerp + (cos,sin) nlerp) or extrapolated (cur + velocity·timeAhead) pose. The Box2D managed-Transform tween (`TransformWriteMode.Interpolate`) is never enabled — the smoothing is done in ECS over the stored poses, holding the DOTS posture. The smoothing write survives `LocalToWorldSystem` because the body's `LocalTransform` stays unchanged after baking, so the dirty-check skips it. A `None` body carries no smoothing component and is never touched by this system, keeping its fixed-rate pose. In a fixed-rate batchmode loop (no render headroom) the time-ahead is ≈0 and the smoothing is an identity write — which is why the interpolation parity gate asserts the internal lerp/extrapolate invariant rather than a rendered-frame comparison.

## Update order

In `FixedStepSimulationSystemGroup`:

1. `PhysicsBody2DCleanupSystem` — free the Box2D bodies of destroyed entities (`[UpdateBefore(PhysicsWorld2DSystem)]`).
2. `PhysicsJoint2DCreationSystem` — create joints whose bodies now both exist (`[UpdateBefore(PhysicsWorld2DSystem)]`).
3. `PhysicsWorld2DSystem` — create missing bodies (per-entity, or from a cached body template for identical instantiated forms past the threshold, collapsing same-frame runs through one `CreateBodyBatch`; the optimisation seam, `low-level-surface.md`), apply commands + effectors, `Simulate(dt)`, collect events, collect joint breaks, record the fixed-step time.
4. `PhysicsJoint2DBreakSystem` — apply Destroy / Disable joint breaks (`[UpdateAfter(PhysicsWorld2DSystem)]`).
5. `PhysicsBody2DWriteBackSystem` — bulk-read poses, Burst-write `LocalToWorld`, capture smoothing (`[UpdateAfter(PhysicsWorld2DSystem)]`).

In `TransformSystemGroup`: `PhysicsBody2DSmoothingSystem` (`[UpdateBefore(LocalToWorldSystem)]`) for interpolated bodies. `LocalToWorldSystem` runs later and skips the physics-owned entities because of the write group / unchanged `LocalTransform`. Rendering consumers read `LocalToWorld` after that — out of scope.
