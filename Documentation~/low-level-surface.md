# Low-level surface

The low-level surface is the direct ECS path onto the same runtime the bake path produces. It exists because the high-level bake path is deliberately not optimised for the many-identical-bodies case, and because the built-in components cannot express some knobs the runtime can. Both capabilities ship in the `Samples~/CustomAuthoring2D` sample; the runtime they target is the one `runtime-systems.md` describes, unchanged. The verified `Unity.U2D.Physics` signatures below were decoded against the editor's module XML (`UnityEngine.PhysicsCore2DModule.xml`) and used in the shipped systems.

## Direct runtime authoring

A body can be authored by writing the runtime `IComponentData` directly — `PhysicsBody2DDefinition` + `PhysicsShape2D` (+ the optional `PhysicsBody2DInitialVelocity`) — from code or from your own ECS baker, with no built-in component in between. The creation system makes the Box2D body the same way it does for a baked one, because it keys on the components, not on how they were produced. The `CustomAuthoring2D` sample's `PhysicsBody2DAuthoring` / `PhysicsShape2DAuthoring` MonoBehaviours are the worked example of this (`custom-authoring.md`).

## Mass spawning (the cached-template optimisation)

The idiomatic way to spawn many identical bodies is to bake (or direct-author) one prefab carrying the body+shape components and a baked `PhysicsBody2DFormHash`, then `ecb.Instantiate(prefab)` it. The instances are self-describing — `Instantiate` replicates the form hash by value — so `PhysicsWorld2DSystem`'s creation loop recognises the shared form with no side-channel request and serves the instances from a cached body template, which removes the per-entity C# definition construction + mass arithmetic. There is no `PhysicsBody2DBatchRequest` and no request-consuming system: a self-describing instance IS the trigger.

How the optimisation works, and what it can and cannot save:

- The `PhysicsBody2DFormHash` is a baked 128-bit content hash of the body+shape **form** (every field that feeds the Box2D definitions, with pose and initial velocity excluded), computed by `PhysicsBody2DFormHashBakingSystem` after the component bakers. Pose-free, so every instance of a spray shares one hash.
- `PhysicsWorld2DSystem` keeps a cross-frame `NativeHashMap<formHash, template>`. A form's template is built **lazily**, once the form's body count crosses `PhysicsStep2DAuthoring.IdenticalBodyThreshold` (N): below N each body takes the unchanged per-entity path; the body that crosses N becomes the template donor; at/above N each body is created from the cached template. The cache survives a world recreate (it holds pose-free, world-independent data). This is the **cross-frame** mechanism — a 1/frame spray still benefits, because the cache persists the form knowledge across frames.
- Within one frame, a run of K ≥ 2 identical bodies is additionally collapsed into a single `PhysicsWorld.CreateBodyBatch(def, count, Allocator)` (one native body-creation call for all K), then K `CreateShape` calls, then one `PhysicsBody.SetBatchTransform(ReadOnlySpan<PhysicsBody.BatchTransform>)` writing the K individual poses.
- **The floor (stated plainly):** Box2D-v3 has no clone/share/template primitive, so each body owns its own concrete shape — N bodies cost N `CreateBody` + N `CreateShape` native calls, full stop. The cache removes the per-entity *C# definition construction and mass arithmetic*, not the native body/shape allocation. The saving is therefore bounded and grows with form complexity (small for a default-density auto-mass circle; real for a polygon's vertex marshalling or a custom-mass resolution). The optimisation is **transparent**: a cached-template body is bit-identical to a per-entity one, so toggling `CacheIdenticalBodies` or any threshold N never changes the simulation.

The related bulk surface for despawn:

- `PhysicsBody.DestroyBatch(ReadOnlySpan<PhysicsBody>)` — the **static** per-body batch destroy that cascades to each body's shapes AND attached joints; this is what `PhysicsBody2DCleanupSystem` calls to free a destroyed entity's body (`runtime-systems.md`). It is distinct from `PhysicsWorld.DestroyBodyBatch`, which does NOT document the shape/joint cascade — reaching for the world-scoped call instead would orphan a destroyed body's shapes/joints. `PhysicsWorld.DestroyJointBatch` (also static) is the world-teardown joint drain.

Per-entity despawn needs no manual destroy call: destroying a physics entity (`EntityManager.DestroyEntity`) frees its Box2D body, shapes, and joints automatically through the cleanup-component teardown (`runtime-systems.md`).

## What is not exposed

`WorldIndex`, `SolverType`, and a custom inertia override are deliberately **not** exposed on the runtime, even though the 3D DOTS custom-authoring sample has them. World-index sharding needs the multi-world model the package defers (one `PhysicsWorld` per ECS `World` today), and a custom solver/inertia override has no field on the current runtime archetype. They are the natural additive extension when that infrastructure lands, recorded here and in the sample so the omission is visible rather than silent.
