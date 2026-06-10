# Bake contract

This is the contract the high-level surface promises: for each supported built-in component, exactly which fields the baker reads, the runtime `IComponentData` it produces, the Box2D mapping at body/shape/joint creation, and the fields the baker deliberately ignores. A field listed as ignored maps to a Box2D default or has no Box2D equivalent; setting it on the authoring component silently does nothing, which is the surprise this doc exists to prevent. The runtime archetype produced here is identical whether a body is authored with the built-in components or the custom MonoBehaviours (`custom-authoring.md`) — both bake to the same components the step and write-back systems consume.

The bakers all live in the editor-only `Zori.Entities.Physics2D.Baking` assembly, so the `UnityEngine.*` authoring references never reach a player build. Every body / collider / joint / effector baker requests `GetEntity(TransformUsageFlags.Dynamic)`, which gives the entity the `LocalToWorld` the write-back system targets; the one exception is the simulation-config baker (below), whose entity is pure data and uses `TransformUsageFlags.None`.

## Simulation configuration

The package ships one non-built-in authoring component for the world itself: `PhysicsStep2DAuthoring` (in the `Zori.Entities.Physics2D.Authoring` assembly), the 2D analogue of `com.unity.physics`'s `PhysicsStepAuthoring`. Place ONE on a GameObject in a SubScene to configure the simulation; `PhysicsStep2DAuthoringBaker` bakes it to the `PhysicsWorld2DConfig` singleton that `PhysicsWorld2DSystem` reads at world creation (`runtime-systems.md`). The configuration is explicit per scene: nothing is read from the project's `UnityEngine.Physics2D` settings at bake or runtime, and a scene with NO `PhysicsStep2DAuthoring` keeps the Box2D `PhysicsWorldDefinition.defaultDefinition` — the exact world shipped before this surface existed. The component's inspector defaults mirror `defaultDefinition` exactly, so a component left untouched is also behaviourally identical to no component.

The exposed fields are the genuinely configurable subset of `Unity.U2D.Physics.PhysicsWorldDefinition`, verified against the editor `6000.6.0a6` module XML / DLL.

| `PhysicsStep2DAuthoring` field | Runtime field (`PhysicsWorld2DConfig`) | Box2D mapping at creation (`PhysicsWorldDefinition.*`) | Default |
|---|---|---|---|
| `Gravity` (`float2`, m/s²) | `gravity` (`float2`) | `gravity` (`Vector2`) | `(0, -9.81)` |
| `SimulationSubSteps` (`int`, ≥1) | `simulationSubSteps` | `simulationSubSteps` (Box2D-v3 solver sub-steps per `Simulate`) | `4` |
| `SimulationWorkers` (`int`, ≥1) | `simulationWorkers` | `simulationWorkers` (capped to device cores at runtime) | `64` |
| `ContinuousAllowed` (`bool`) | `continuousAllowed` | `continuousAllowed` (Dynamic-vs-Static CCD) | `true` |
| `SleepingAllowed` (`bool`) | `sleepingAllowed` | `sleepingAllowed` | `true` |
| `BounceThreshold` (`float`, m/s) | `bounceThreshold` | `bounceThreshold` | `1` |
| `ContactHitEventThreshold` (`float`, m/s) | `contactHitEventThreshold` | `contactHitEventThreshold` | `1` |
| `ContactFrequency` (`float`, cycles/s) | `contactFrequency` | `contactFrequency` (contact stiffness) | `30` |
| `ContactDamping` (`float`, 1 = critical) | `contactDamping` | `contactDamping` (contact bounciness) | `10` |
| `ContactSpeed` (`float`, m/s) | `contactSpeed` | `contactSpeed` (overlap-resolution speed) | `3` |
| `ContactRecycleDistance` (`float`, m) | `contactRecycleDistance` | `contactRecycleDistance` (0 disables recycling) | `0.05` |
| `MaximumLinearSpeed` (`float`, m/s) | `maximumLinearSpeed` | `maximumLinearSpeed` | `400` |

Deliberately NOT exposed, and why: the world's **simulation type** is locked to `SimulationType.Script` and is not a field — the package owns stepping with an explicit `PhysicsWorld.Simulate(dt)`, and any auto-stepping mode (`FixedUpdate` / `Update`) would make the engine integrate the world on top of the package's step (a double-step). The **fixed timestep** is not configured here either — it is the `FixedStepSimulationSystemGroup` rate, an ECS-global property shared by every system in that group, not a per-physics-world value. The `com.unity.physics` 3D knobs `EnableGyroscopicTorque`, `SolverIterationCount`, `DirectSolverSettings`, `SolverStabilizationHeuristic`, `SynchronizeCollisionWorld`, the incremental-broadphase toggles, the depenetration-velocity clamps, and `CollisionTolerance` have no `PhysicsWorldDefinition` (2D) analogue — the 2D engine's solver model is the single `simulationSubSteps` count, restitution is per-shape (`PhysicsMaterial2D` bounciness, below), and the nearest collision-tolerance knobs are `ContactSpeed` / `ContactRecycleDistance`, which are exposed under their real engine names.

The package is single-world, so exactly one `PhysicsStep2DAuthoring` is expected per baked world. `[DisallowMultipleComponent]` blocks two on one GameObject; two on separate GameObjects in the same SubScene bake successfully but surface as a singleton-query throw when the world is created.

## `Rigidbody2D` → `PhysicsBody2DDefinition`

`Rigidbody2DBaker : Baker<Rigidbody2D>` reads the body parameters and the initial pose from the GameObject `Transform`.

| `Rigidbody2D` field | Runtime field | Box2D mapping at creation |
|---|---|---|
| `bodyType` (`RigidbodyType2D` Dynamic/Kinematic/Static) | `PhysicsBody2DDefinition.bodyType` | `PhysicsBody.BodyType.Dynamic/Kinematic/Static` |
| `gravityScale` | `gravityScale` | `PhysicsBodyDefinition.gravityScale` |
| `linearDamping` | `linearDamping` | `PhysicsBodyDefinition.linearDamping` |
| `angularDamping` | `angularDamping` | `PhysicsBodyDefinition.angularDamping` |
| `constraints` (`RigidbodyConstraints2D` freeze flags) | `constraints` | per-flag OR-fold onto `PhysicsBody.BodyConstraints` (`FreezePositionX` → `PositionX`, `FreezePositionY` → `PositionY`, `FreezeRotation` → `Rotation`) |
| `mass` | `mass` | written into the body's `MassConfiguration` when `useAutoMass` is false |
| `useAutoMass` | `useAutoMass` | true ⇒ mass is density-derived from the shapes; false ⇒ the explicit `mass` is applied |
| `collisionDetectionMode` (`Continuous` / `Discrete`) | `fastCollisions` | `Continuous` ⇒ `PhysicsBodyDefinition.fastCollisionsAllowed = true` (continuous collision against Dynamic / Kinematic bodies); `Discrete` ⇒ false. Dynamic-vs-Static CCD is the world-level `continuousAllowed` (on by default), so a Discrete body still does not tunnel a static wall (`parity-matrix.md` records the v2-vs-v3 static-wall gap). |
| `interpolation` (`None` / `Interpolate` / `Extrapolate`) | `interpolation` (a `PhysicsBody2DInterpolation`) | mapped 1:1; a non-`None` body additionally gets a `PhysicsBody2DSmoothing` component so its `LocalToWorld` is smoothed between fixed steps at render rate (`runtime-systems.md`). `None` carries no smoothing component. |
| `Transform.position` (xy) | `initialPosition` | `PhysicsBodyDefinition.position` |
| `Transform.eulerAngles.z` (degrees) | `initialRotationRadians` (radians) | `PhysicsBodyDefinition.rotation` via the unit-agnostic `PhysicsRotate.FromRadians` |

The body baker (and the collider-only static fallback, and the custom-body authoring) also emits a `PhysicsBody2DRenderScale` carrying `transform.lossyScale.xy`. The collider geometry is baked at this scale (so the Box2D body is unit scale), and the write-back re-applies the scale to `LocalToWorld` so the rendered transform keeps its scale — the collision-and-graphics split documented under [Transform scale](#transform-scale--baked-into-the-collider-geometry-carried-to-graphics).

The initial Z-Euler angle is the body's 2D rotation; X/Y rotation in authoring is ignored because the simulation is planar. The baker converts the GameObject's degrees to radians once (`math.radians`) because the package's rotation-angle convention is radians ([angular unit convention](angular-units.md)); the engine rotor accepts either unit. Angular velocity and joint angular parameters stay in degrees per that same convention — no conversion at bake.

`Rigidbody2D.linearVelocity` / `angularVelocity` are **not** read by this baker. They are runtime-only properties that bake to zero from a saved scene (the value is not serialised), so a velocity seed is authored separately on `InitialVelocity2DAuthoring` (below).

Ignored, mapping to a Box2D default or to no-op: `simulated`, `sleepMode` (and its sleep-tolerance thresholds), `includeLayers` / `excludeLayers` (per-collider/body layer overrides on top of the matrix are not folded into the baked contact filter — `parity-matrix.md` still-not-covered), `useFullKinematicContacts`, `totalForce` / `totalTorque` (runtime force accumulation is not a bake input — runtime force write-in is the `PhysicsBody2DCommand` surface, `runtime-systems.md`). `centerOfMass` / `inertia` are taken from the shapes' mass distribution; an explicit override is not exposed on the built-in path.

## Initial velocity seed → `PhysicsBody2DInitialVelocity`

`InitialVelocity2DAuthoring` is a small package-shipped authoring component (not a built-in one), baked by `InitialVelocity2DBaker`, that carries a serialised `linearVelocity` / `angularVelocity`. It exists because `Rigidbody2D.linearVelocity` is runtime-only and cannot be baked. The creation system reads `PhysicsBody2DInitialVelocity` when the body is created and applies the seed to the live `PhysicsBody`. The custom `PhysicsBody2DAuthoring` body emits the same component, so the seed is one mechanism across both surfaces.

## `Collider2D` shapes → `PhysicsShape2D`

Each built-in collider has its own `Baker<T>` in `Baking/Collider2DBaking.cs`, all producing one tagged-union `PhysicsShape2D` (`kind` byte plus the per-kind geometry fields). Fixed-size kinds (Circle / Box / Capsule) carry their geometry inline; variable-length kinds (Polygon / Edge) reference a `BlobAssetReference<PhysicsShape2DVertices>` blob — only those two ever allocate a blob.

| Built-in collider | `PhysicsShape2D.kind` | Geometry source fields | Box2D geometry at creation |
|---|---|---|---|
| `CircleCollider2D` | `Circle` | `radius`, `offset` | `CircleGeometry { radius, center = offset }` |
| `BoxCollider2D` | `Box` | `size`, `offset`, `edgeRadius` | `PolygonGeometry.CreateBox(size, radius, PhysicsTransform(offset), inscribe: false)` |
| `CapsuleCollider2D` | `Capsule` | `size` + `direction` → two end-cap centers, `radius`, `offset` | `CapsuleGeometry.Create(center1, center2, radius)` with the offset folded into both centers |
| `PolygonCollider2D` | `Polygon` | path-0 vertices (blob), `offset` | `PolygonGeometry.Create(span, radius, PhysicsTransform(offset))` — convex hull, 3..`MaxPolygonVertices` |
| `EdgeCollider2D` | `Edge` | `points` (blob), `edgeRadius`, `edgeIsLoop` | `ChainGeometry(points)` attached via `PhysicsBody.CreateChain` |

`PolygonCollider2D` bakes **path 0 only** and assumes the path is convex; additional paths are ignored and a concave single-path authored polygon fails `PolygonGeometry.Create`'s validation. A concave or multi-path collision surface is authored with a `CompositeCollider2D` (below), whose merged Polygons paths bake with decomposition (`PolygonGeometry.CreatePolygons`).

### Transform scale → baked into the collider geometry, carried to graphics

A Box2D shape carries no per-shape and no per-body scale (the engine geometry structs — `CircleGeometry`, `PolygonGeometry`, `CapsuleGeometry`, `ChainGeometry` — expose only positions, radii, and vertices), so the body GameObject's `transform.lossyScale.xy` is baked INTO every shape's geometry at bake time. This is the "even for static colliders" case: a `BoxCollider2D` with Size (1, 1) on a Transform with Scale X = 18.18 bakes to a box 18.18 m wide, so a body rests across its full rendered width rather than only its centre 1×1. The scale source is the body GameObject's own `lossyScale` (collider and body share one leaf GameObject in this package; a collider with no `Rigidbody2D` IS the body via the static fallback). Box2D has no engine-native body scale to defer to — unlike `com.unity.physics` (3D), which keeps a uniform body scale on `LocalTransform.Scale` for the runtime engine to apply — so in 2D ALL scale (uniform and non-uniform) is baked into the geometry, and the entity scale is carried separately for graphics on `PhysicsBody2DRenderScale` (below).

The per-kind scale rule matches the built-in collider's behaviour under transform scale (`Collider2DBaking.Scale*` helpers):

| Geometry | Scale rule | Why |
|---|---|---|
| Box extents (`size`) | per-axis `abs(size · scale)` | the box gizmo follows X and Y independently; `abs` because a flip mirrors a symmetric box rather than shrinking it |
| Circle radius | `radius · max(abs(scale.x), abs(scale.y))` | a circle cannot become an ellipse, so the LARGER absolute axis is used (the circle gizmo grows to the larger axis) |
| Capsule | the authored `size` is scaled per-axis BEFORE the caps are derived | a non-uniform scale changes both the cap radius and the segment, as the gizmo deforms; scaling precomputed caps uniformly would be wrong |
| Corner-rounding radius (box `edgeRadius`, polygon/composite radius) | isotropic `max(abs(scale.x), abs(scale.y))` | a rounding radius is a circle of curvature; under non-uniform scale this is an approximation (Box2D cannot express elliptical corners) |
| Polygon / edge / composite / custom vertices | per-component signed `v · scale`, with the vertex order REVERSED when `scale.x · scale.y < 0` | a mirror (odd negative-axis count) flips winding; Box2D wants CCW convex hulls and a chain's solid side IS its winding, so the order is reversed to restore it |
| `Collider2D.offset` | per-axis signed `offset · scale` | the offset is a point in the collider's scaled local space; a flip moves it to the mirrored side |

Because the geometry is baked at this scale, the Box2D body runs at unit scale. The entity's scale is re-applied to `LocalToWorld` at write-back from `PhysicsBody2DRenderScale` so the rendered transform is `T · R · S` (collision uses the scaled shape; the sprite keeps its scale). The same rules apply to the custom-authoring shape baker (`PhysicsShape2DAuthoring`) so a shape authored via the custom surface on a scaled GameObject is indistinguishable at runtime from the same shape on a built-in collider. The direct-author (`DirectPhysics2DAuthoring`) and batch (`PhysicsBody2DBatchRequest`) paths have no GameObject transform — the caller supplies final geometry, and the render scale defaults to `(1, 1)`.

A `CustomCollider2D` / custom-authoring capsule stores two explicit end-cap centres (no Vertical/Horizontal axis), so its cap radius uses the same `max(abs(scale))` circle rule rather than the orthogonal-axis rule — a known over-approximation under extreme non-uniform scale, documented for the parity gate.

## Multiple shapes on one body — `CompositeCollider2D` / `CustomCollider2D` → `PhysicsShape2D` + `PhysicsShape2DElement` buffer

A body may carry more than one Box2D shape. The package expresses this as the primary `PhysicsShape2D` (shape 0) plus an optional `DynamicBuffer<PhysicsShape2DElement>` of the remaining shapes (1..K-1) on the same body entity. A one-collider body carries only the primary component and no buffer — the historical single-shape archetype is unchanged. The creation system attaches the primary, then every buffer element, to one Box2D body via the same `CreateShapeForBody` path, so every extra shape gets identical surface/filter/trigger/offset handling; `useAutoMass` mass is the sum over all the body's shapes (Box2D recomputes the mass configuration as each shape is added).

### `CompositeCollider2D` → merged paths

`CompositeCollider2DBaker : Baker<CompositeCollider2D>` calls `GenerateGeometry()` (so the merged paths are current even for a Manual-generation composite), then bakes one `PhysicsShape2D` per merged path (`pathCount` / `GetPath`). The path vertices are local-space and already merged by the engine — the baker reads them, it does not recompute the merge.

| `CompositeCollider2D.geometryType` | `PhysicsShape2D.kind` | Box2D geometry at creation |
|---|---|---|
| `Polygons` | `Polygon` with `polygonDecompose = true`, `radius = edgeRadius` | `PolygonGeometry.CreatePolygons(path, transform, Temp)` decomposes the (possibly concave / over-`MaxPolygonVertices`) merged path into convex fragments, attached in one `CreateShapeBatch` |
| `Outlines` | `Edge` with `edgeIsLoop = true`, `radius = edgeRadius` | `ChainGeometry(path)` attached via `CreateChain` as a CLOSED loop (a composite outline is a closed edge loop — unlike the open `EdgeCollider2D`) |

Friction / bounciness / density and the contact filter are read from the **composite's** own `sharedMaterial` / `density` / layer (after the merge the body interacts with the composite, which carries the merged surface — matching GameObject). The static-body fallback applies (a composite with no `Rigidbody2D` is static level geometry).

### `CustomCollider2D` → custom shape group

`CustomCollider2DBaker : Baker<CustomCollider2D>` reads the collider's `PhysicsShapeGroup2D` (`GetCustomShapes`) and bakes one `PhysicsShape2D` per custom shape, mapping each `PhysicsShapeType2D` 1:1 onto the package kind (vertices are group-local):

| `PhysicsShapeType2D` | `PhysicsShape2D.kind` | Vertices / radius |
|---|---|---|
| `Circle` | `Circle` | 1 vertex = center (folded into `offset`), `radius` |
| `Capsule` | `Capsule` | 2 vertices = the two end-cap centers, `radius` = cap radius |
| `Polygon` | `Polygon` | N convex-hull vertices, `radius` = corner rounding; `polygonDecompose` only if N > `MaxPolygonVertices` |
| `Edges` | `Edge` | N chain vertices, `edgeIsLoop = false` (open one-sided chain — static-surface use) |

Surface / filter come from the custom collider's own material / layer (as a built-in collider), and the static-body fallback applies. The `UnityEngine.PhysicsShape2D` descriptor name-collides with the package's runtime `PhysicsShape2D`; the baker fully-qualifies the built-in struct.

### `usedByComposite` children are excluded from standalone baking

A child collider merged into a composite declares it via `Collider2D.compositeOperation != None` (the `6000.6.0a6` form of the legacy `usedByComposite` bool — `Merge` is the common case). Each built-in collider baker returns early when `Collider2DBaking.IsUsedByComposite(authoring)` is true, baking **nothing** — no `PhysicsShape2D`, no static-body fallback. The child's geometry is already represented inside the composite's merged paths, so baking it again would create a second overlapping shape on its own static body — the double-bake a composite exists to avoid.

`EdgeCollider2D` bakes to a Box2D `ChainGeometry`, which is a **one-sided, non-solid surface** — the faithful use is as static ground/wall geometry, the same way a Noita-like world uses edges. A built-in `EdgeCollider2D` is two-sided and usable on a dynamic body; the chain primitive deliberately does not solve that case, so an edge as a dynamic falling body is in the negative space of the Box2D chain (a dynamic one-sided edge body would need `SegmentGeometry` or a polygon, not a chain).

### Static-body fallback

A `Collider2D` on a GameObject with **no** `Rigidbody2D` is a static body in built-in 2D physics. Each collider baker calls `Collider2DBaking.AddStaticBodyIfNoRigidbody`, which emits a default static `PhysicsBody2DDefinition` when `GetComponent<Rigidbody2D>()` is null — a no-op when a `Rigidbody2D` is present (that path bakes the body definition via `Rigidbody2DBaker`). Without this, a collider-only static floor would never become a body and dynamic bodies would have nothing to rest on.

### Mass floor for chain-only dynamic bodies

A Box2D chain contributes no mass, so a dynamic body whose only shape is a chain would have mass 0 and never move under gravity, whereas a built-in `Rigidbody2D` defaults to mass 1 regardless of collider. After shape creation, a dynamic body with `mass <= 0` is given a unit `MassConfiguration`. This is a correctness floor, gated on `bodyType == Dynamic && mass <= 0`, not the mass feature; it never overrides a solid shape's density-derived or explicitly-authored mass.

## `PhysicsMaterial2D` + density → surface fields on `PhysicsShape2D`

`PhysicsShape2D` carries `friction`, `bounciness`, and `density` as plain floats (so the component stays blittable); the Box2D `PhysicsShape.SurfaceMaterial` struct is built at creation. `Collider2DBaking.ReadSurface` reads friction / bounciness from `Collider2D.sharedMaterial` (a `PhysicsMaterial2D`) when present, else the engine's material-less default of friction 0.4 / bounciness 0; density is read from `Collider2D.density`. At creation, `PhysicsWorld2DSystem` writes these into the shape's `PhysicsShapeDefinition.surfaceMaterial` and sets `PhysicsShapeDefinition.density` when the baked density is positive (leaving Box2D's default otherwise).

Ignored on `Collider2D`: `layerOverridePriority`. (`isTrigger`, the collision layer / category filter, and `compositeOperation` / `usedByComposite` ARE read; `usedByEffector` marks the collider as an effector region, baked by the effector bakers below — the effector GameObject's own collider bakes to its shape normally, sensor or solid per the effector kind.)

## `*Joint2D` → `PhysicsJoint2DDefinition`

Each of the nine built-in 2D joints has a `Baker<T>` producing one tagged-union `PhysicsJoint2DDefinition` (`PhysicsJoint2DKind` plus the joint's parameters and an `Entity connectedBody`). The component carries the **built-in** joint identity, not the Box2D kind, because several built-in joints map to one Box2D kind. The entity carrying the joint component is Box2D `bodyB` (the joint owner); the connected entity is `bodyA`. `Joint2DBaking.ResolveConnectedBody` turns `Joint2D.connectedBody` into the connected `Entity` and registers the bake dependency.

| Built-in joint | `PhysicsJoint2DKind` | Box2D joint definition |
|---|---|---|
| `HingeJoint2D` | `Hinge` | `PhysicsHingeJointDefinition` |
| `SliderJoint2D` | `Slider` | `PhysicsSliderJointDefinition` |
| `WheelJoint2D` | `Wheel` | `PhysicsWheelJointDefinition` |
| `DistanceJoint2D` | `Distance` | `PhysicsDistanceJointDefinition` (spring disabled ⇒ rigid) |
| `SpringJoint2D` | `Spring` | `PhysicsDistanceJointDefinition` (spring enabled) |
| `FixedJoint2D` | `Fixed` | `PhysicsFixedJointDefinition` (one built-in frequency feeds both the linear and angular stiffness) |
| `RelativeJoint2D` | `Relative` | `PhysicsRelativeJointDefinition` (position spring drives the maintained offset) |
| `FrictionJoint2D` | `Friction` | `PhysicsRelativeJointDefinition` (velocity-control caps, no spring) |
| `TargetJoint2D` | `Target` | `PhysicsRelativeJointDefinition` (position spring pulls toward a world target; no connected body — resolves to the package's static world anchor) |

Per-joint field mappings: anchors are body-local `float2`; a slide/suspension axis bakes as `axisAngleDegrees` (degrees); the motor as `enableMotor` / `motorSpeed` (hinge/wheel deg/sec, slider m/s) / `maxMotorEffort`; a limit as `enableLimit` / `lowerLimit` / `upperLimit` (hinge angle in degrees, slider/wheel translation in meters); a spring as `enableSpring` / `springFrequency` / `springDamping`; the distance/spring rest length as `restLength`; the relative joint's maintained pose as `linearOffset` + `angularOffsetDegrees` (degrees) with `maxForce` + `maxTorque` effort caps; `collideConnected` from `Joint2D.enableCollision`. The joint angular parameters are degrees throughout because the engine's joint angle/motor fields are degrees ([angular unit convention](angular-units.md)).

Two measured frame-convention facts the bakers encode to match the built-in reference rather than the package's own intuition:

- A `RelativeJoint2D.linearOffset` is encoded as `localAnchorB = anchor + linearOffset` so the package drives `bodyB` to `bodyA − linearOffset`, matching the built-in sign (no-op for Friction/Target, which bake a zero offset).
- For the relative-family joints the **position spring** (`springLinearFrequency`/`Damping`), not the velocity-control `maxForce`/`maxTorque`, is what drives a body toward its offset; `maxForce`/`maxTorque` alone make a valid but inert friction joint.

### Joint break → native threshold + `PhysicsJointBreakEvent2D`

Every `*Joint2D` shares the base `Joint2D.breakForce` / `breakTorque` / `breakAction`, baked onto the shared `PhysicsJoint2DDefinition`:

| `Joint2D` field | Runtime field | Creation / runtime behaviour |
|---|---|---|
| `breakForce` (Infinity = never break) | `breakForce` | when finite and the action is not `Ignore`, set as the native `PhysicsJoint.forceThreshold` at creation |
| `breakTorque` (Infinity = never break) | `breakTorque` | when finite and the action is not `Ignore`, set as the native `PhysicsJoint.torqueThreshold` |
| `breakAction` (`JointBreakAction2D`) | `breakAction` (a `PhysicsJointBreakAction2D`) | `Ignore` → never set a threshold; `CallbackOnly` → surface the break event, keep the joint; `Destroy` / `Disable` → surface the event and destroy the Box2D joint (the bodies separate) |

The native `forceThreshold` is in the SAME units as the GameObject `breakForce` (newtons) — the Phase-8 force-units gate measured the break-load ratio at exactly 1.000, so no bake scale is applied. Box2D fires a `jointThresholdEvents` entry when a joint's reaction exceeds its threshold but does NOT itself destroy the joint; the runtime reads that event and applies the action (`runtime-systems.md`). `Destroy` and `Disable` both destroy the Box2D constraint (Box2D has no disable-joint primitive); the distinction is preserved in the surfaced `PhysicsJointBreakEvent2D.breakAction` so a consumer can mirror the built-in destroy-the-entity vs disable-the-component semantics — the package never destroys the owner entity itself.

Ignored / handled implicitly: `FixedJoint2D.referenceAngle` (the Box2D fixed joint locks the relative orientation implied by the bodies' authored poses at creation, which carries the same information for axis-aligned bodies); runtime-tunable joint parameters (a motor speed / limit / spring changed per frame is not a public surface — joint break is wired, but live re-tuning of an unbroken joint is not; `parity-matrix.md` still-not-covered).

### World anchor for a null connected body

Built-in 2D physics allows a joint's `connectedBody` to be null ("a joint to a point in space"). Box2D has no implicit ground body, so `PhysicsJoint2DCreationSystem` lazily creates and caches one shapeless static body at the world origin to serve as `bodyA`. This path is exercised and validated only for `TargetJoint2D`, whose world-target semantics map onto it faithfully. It is **not** the right model for a null-connected `HingeJoint2D` / `WheelJoint2D`: built-in `Physics2D` does not interpret their null connected body as a Box2D world anchor, so those joints must be authored against a concrete static anchor body, not a null one. The world-anchor code path is correct and shipped; only the parity fixtures for Hinge/Wheel avoid it because nothing on the built-in side validates that interpretation.

## `*Effector2D` → `PhysicsEffector2D`

Box2D-v3 has no native effector; each built-in effector is reproduced as an ECS apply over a baked definition. Each of the five effector bakers (`AreaEffector2DBaker`, `BuoyancyEffector2DBaker`, `PointEffector2DBaker`, `PlatformEffector2DBaker`, `SurfaceEffector2DBaker`) reads its built-in component and emits one tagged-union `PhysicsEffector2D` (`PhysicsEffector2DKind` plus the per-kind fields). The effector GameObject's own collider bakes to its `PhysicsShape2D` through the normal collider baker, and the collider-only static-body fallback gives the effector entity its body — the effector baker only ADDS the definition. The collider's `isTrigger` is the load-bearing difference between the two families, and it is whatever the author set:

- **Force-field effectors (Area / Buoyancy / Point)** sit on a SENSOR collider (`isTrigger = 1` in every example scene): the region overlaps without a collision response, so a body falls through / floats in it. Each step, `PhysicsWorld2DSystem` overlap-queries the region (honoring `colliderMask`) and applies the per-kind force to every dynamic body inside, before `Simulate`, in the same pre-step window the runtime write-in drains (`runtime-systems.md`) — so an effector force is mass-scaled and frozen-axis-cancelled by the solver exactly like a `Rigidbody2D.AddForce(_, Force)`.
- **Contact-response effectors (Platform / Surface)** sit on a SOLID collider (`isTrigger = 0`): a body rests ON the platform / rides the belt. Platform gates the platform body's collision one-way from the surface arc; Surface drives contacting bodies (`PhysicsBody.GetContacts`) tangentially toward the belt speed.

| Built-in effector | `PhysicsEffector2DKind` | Key baked fields |
|---|---|---|
| `AreaEffector2D` | `Area` | `forceMagnitude`, `forceVariation`, `forceAngleRadians` (degrees→radians), `useGlobalAngle`, `forceTargetIsRigidbody`, `linearDamping`, `angularDamping` |
| `BuoyancyEffector2D` | `Buoyancy` | `surfaceLevel`, `fluidDensity` (= `density`), `flowMagnitude`, `flowVariation`, `flowAngleRadians`, `gravityMagnitude` (= `|Physics2D.gravity|` at bake), `linearDamping`, `angularDamping` |
| `PointEffector2D` | `Point` | `forceMagnitude` (negative attracts, positive repels), `forceVariation`, `distanceScale`, `forceMode` (Constant / InverseLinear / InverseSquared), `forceSourceIsRigidbody`, `linearDamping`, `angularDamping` |
| `PlatformEffector2D` | `Platform` | `surfaceArcRadians` (degrees→radians), `rotationalOffsetRadians`, `useOneWay` |
| `SurfaceEffector2D` | `Surface` | `surfaceSpeed` (= `speed`), `surfaceSpeedVariation`, `forceScale`, `useContactForce`, `surfaceUseFriction` |

`colliderMask` is the shared base (`Effector2DBaking.ReadMask`): `useColliderMask ? colliderMask : Physics2D.GetLayerCollisionMask(layer)`, baked to a raw 64-bit mask passed as the region overlap / drive query's `hitLayerMask`. The drag fields apply as a velocity multiplier `v *= 1/(1 + damping·dt)`, the standard Unity/Box2D damping integrator, not a force. The variation fields apply only when non-zero (a `Unity.Mathematics.Random`-seeded path); zero is the deterministic, parity-asserted case.

`parity-matrix.md` is the canonical home for the effector parity verdicts — Area / Point / Surface and single-body Platform are at-parity; the multi-body Platform one-way and the Platform `colliderMask`-on-collider divergence are documented known gaps; the buoyancy AABB-submersion approximation is a measured known gap. Read it before relying on an effector edge case.
