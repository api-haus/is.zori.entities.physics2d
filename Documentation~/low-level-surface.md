# Low-level surface

The low-level surface is the direct ECS path onto the same runtime the bake path produces. It exists because the high-level bake path is deliberately not optimised for the many-identical-bodies case, and because the built-in components cannot express some knobs the runtime can. Both capabilities ship in the `Samples~/CustomAuthoring2D` sample; the runtime they target is the one `runtime-systems.md` describes, unchanged. The verified `Unity.U2D.Physics` signatures below were decoded against the editor's module XML (`UnityEngine.PhysicsCore2DModule.xml`) and used in the shipped systems.

## Direct runtime authoring

A body can be authored by writing the runtime `IComponentData` directly — `PhysicsBody2DDefinition` + `PhysicsShape2D` (+ the optional `PhysicsBody2DInitialVelocity`) — from code or from your own ECS baker, with no built-in component in between. The creation system makes the Box2D body the same way it does for a baked one, because it keys on the components, not on how they were produced. The `CustomAuthoring2D` sample's `PhysicsBody2DAuthoring` / `PhysicsShape2DAuthoring` MonoBehaviours are the worked example of this (`custom-authoring.md`).

## Bulk creation (the optimisation seam)

For spawning many identical bodies in one frame, the per-entity `CreateBody` loop is replaced by one native batch call. The package ships `PhysicsBody2DBatchCreationSystem`, which reads a `PhysicsBody2DBatchRequest` and creates a run of identical bodies in a single `CreateBodyBatch`:

- `PhysicsWorld.CreateBodyBatch(PhysicsBodyDefinition def, int count, Allocator alloc)` → `NativeArray<PhysicsBody>` — creates `count` identical bodies in one call, returning their handles in index order. The entity↔handle association is rebuilt from that index order.
- `PhysicsBody.GetBatchTransform(ReadOnlySpan<PhysicsBody>, Allocator)` — the bulk pose read; this is already the write-back mechanism, and the low-level surface exposes it as a public way to read many poses in one call.
- `PhysicsBody.SetBatchTransform(ReadOnlySpan<PhysicsBody.BatchTransform>)` — bulk teleport.
- `PhysicsBody.DestroyBatch(ReadOnlySpan<PhysicsBody>)` — the **static** per-body batch destroy that cascades to each body's shapes AND attached joints; this is what `PhysicsBody2DCleanupSystem` calls to free a destroyed entity's body (`runtime-systems.md`). It is distinct from `PhysicsWorld.DestroyBodyBatch`, which does NOT document the shape/joint cascade — reaching for the world-scoped call instead would orphan a destroyed body's shapes/joints. `PhysicsWorld.DestroyJointBatch` (also static) is the world-teardown joint drain.

Per-entity despawn needs no manual destroy call: destroying a physics entity (`EntityManager.DestroyEntity`) frees its Box2D body, shapes, and joints automatically through the cleanup-component teardown (`runtime-systems.md`), so the bulk destroy surface above is for explicit low-level batch despawn, not the common entity-death case.

Bulk creation keyed by a shared definition is the headline beyond-built-in optimisation: it turns N `CreateBody` calls into one, which the built-in component path cannot do because each baked entity carries its own definition.

## What is not exposed

`WorldIndex`, `SolverType`, and a custom inertia override are deliberately **not** exposed on the runtime, even though the 3D DOTS custom-authoring sample has them. World-index sharding needs the multi-world model the package defers (one `PhysicsWorld` per ECS `World` today), and a custom solver/inertia override has no field on the current runtime archetype. They are the natural additive extension when that infrastructure lands, recorded here and in the sample so the omission is visible rather than silent.
