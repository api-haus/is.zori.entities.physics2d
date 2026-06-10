# Supported components reference

This page documents every authoring surface the package supports, one section per component family, with the options each exposes and a pointer to what it bakes to. The exact field-by-field mapping (which field maps to which runtime/Box2D field, and which fields are deliberately ignored) is the [bake contract](bake-contract.md); this page is the consumer-facing "what can I author and what does each knob do" reference. The per-feature parity verdict for each is in the [parity matrix](parity-matrix.md). Angular units throughout follow the [angular unit convention](angular-units.md).

The supported set is the full built-in 2D physics component surface — `Rigidbody2D`, the five `Collider2D` shapes plus `CompositeCollider2D` and `CustomCollider2D`, `PhysicsMaterial2D`, all nine `*Joint2D` components, and all five `*Effector2D` components — plus two package-shipped authoring components for the cases the built-in surface has no component for: `InitialVelocity2DAuthoring` and `PhysicsStep2DAuthoring`.

## `Rigidbody2D` — the body

A `Rigidbody2D` bakes to a `PhysicsBody2DDefinition`. The supported options:

- **`bodyType`** — `Dynamic`, `Kinematic`, or `Static`, mapped 1:1 to the Box2D body type.
- **`gravityScale`** — the per-body gravity multiplier.
- **`linearDamping` / `angularDamping`** — velocity decay.
- **`mass` / `useAutoMass`** — with `useAutoMass` on, mass is derived from the shapes' density; with it off, the explicit `mass` is applied.
- **`constraints`** — the freeze flags (`FreezePositionX`, `FreezePositionY`, `FreezeRotation`) fold onto the body's constraint mask.
- **`collisionDetectionMode`** — `Continuous` enables continuous collision against Dynamic / Kinematic bodies; `Discrete` disables it. Note that a Discrete body still does not tunnel a STATIC wall, because Dynamic-vs-Static CCD is the world-level default — see the [parity matrix](parity-matrix.md) for the v2-vs-v3 static-wall behaviour.
- **`interpolation`** — `None`, `Interpolate`, or `Extrapolate`. A non-`None` body additionally gets render-rate smoothing of its `LocalToWorld` between fixed steps ([runtime systems](runtime-systems.md)).

The initial pose comes from the GameObject `Transform` (position xy, and the Z-Euler angle as the 2D rotation; X/Y rotation is ignored because the simulation is planar).

`Rigidbody2D.linearVelocity` / `angularVelocity` are NOT read — they are runtime-only properties that bake to zero from a saved scene. Author a starting velocity with `InitialVelocity2DAuthoring` (below). Ignored fields (`simulated`, `sleepMode`, `includeLayers` / `excludeLayers`, `useFullKinematicContacts`, `centerOfMass` / `inertia` override) are enumerated in the [bake contract](bake-contract.md) and the [parity matrix](parity-matrix.md) still-not-covered list.

## `InitialVelocity2DAuthoring` — the velocity seed (package-shipped)

A small package-shipped authoring component, added alongside a `Rigidbody2D`, that carries a serialised starting velocity baked to `PhysicsBody2DInitialVelocity` and applied to the body on creation. It exists because `Rigidbody2D.linearVelocity` is runtime-only and cannot be baked. Fields:

- **linear velocity** (`float2`, m/s).
- **angular velocity** (degrees per second, per the [angular unit convention](angular-units.md)).

## Colliders — the shapes

Each built-in collider bakes to a `PhysicsShape2D` (a tagged union over `PhysicsShape2DKind`). Every collider also reads the shared surface and filter options below.

| Collider | `PhysicsShape2DKind` | Notes |
|---|---|---|
| `CircleCollider2D` | `Circle` | `radius` + `offset`. |
| `BoxCollider2D` | `Box` | `size`, `offset`, `edgeRadius` (corner rounding). |
| `CapsuleCollider2D` | `Capsule` | `size` + `direction` resolve to the two end-cap centres and a radius. |
| `PolygonCollider2D` | `Polygon` | Convex, path 0 only — a concave or multi-path polygon is authored with a `CompositeCollider2D` instead. |
| `EdgeCollider2D` | `Edge` | A one-sided, non-solid chain surface — the faithful use is static ground/wall geometry. Requires at least 3 vertices (a 2-vertex edge throws at creation — see the [parity matrix](parity-matrix.md) still-not-covered). |

Shared options on every collider:

- **`isTrigger`** — bakes to a sensor shape: the region overlaps without a collision response and produces trigger events rather than contact events.
- **`PhysicsMaterial2D` friction / bounciness** — read from `Collider2D.sharedMaterial`; absent, the engine default of friction 0.4 / bounciness 0 applies.
- **`density`** — `Collider2D.density`, the auto-mass source.
- **layer** — `gameObject.layer` plus the project layer-collision matrix bake to the shape's contact filter, so a baked body collides with exactly the set a GameObject would.

A `Collider2D` on a GameObject with no `Rigidbody2D` bakes to a static body (the static-body fallback), exactly as in built-in 2D physics.

### Multi-shape bodies — `CompositeCollider2D` and `CustomCollider2D`

A body can carry more than one shape: the primary `PhysicsShape2D` plus a buffer of extra shapes.

- **`CompositeCollider2D`** (`geometryType` `Polygons` or `Outlines`) — the merged paths bake to multiple shapes on one body. `Polygons` paths decompose into convex fragments (so a concave or large merged surface is supported); `Outlines` paths bake to closed chain loops. The composite's own material / density / layer drive the merged body's surface and filter. The merged child colliders (those with a composite operation) bake nothing on their own — their geometry is already inside the composite's merged paths.
- **`CustomCollider2D`** — its `PhysicsShapeGroup2D` shapes each bake to one shape on the body, mapping each shape type (Circle / Capsule / Polygon / Edges) 1:1 onto the package kind.

The full geometry mapping for both is in the [bake contract](bake-contract.md).

## `PhysicsMaterial2D` — surface material

`PhysicsMaterial2D` is not authored as a standalone entity; its `friction` and `bounciness` are read through `Collider2D.sharedMaterial` and baked into the shape's surface material. Density is read from `Collider2D.density`, not from the material.

## Joints — `*Joint2D`

All nine built-in 2D joints are supported, each baking to one `PhysicsJoint2DDefinition` carrying the built-in joint identity, its parameters, and the connected entity:

| Joint | Box2D mechanism |
|---|---|
| `HingeJoint2D` | Hinge (revolute) — anchors, optional motor and angle limit. |
| `SliderJoint2D` | Slider (prismatic) — axis angle, optional motor and translation limit. |
| `WheelJoint2D` | Wheel — suspension axis, spring, optional motor. |
| `DistanceJoint2D` | Distance (rigid rod at the rest length). |
| `SpringJoint2D` | Distance with the spring enabled (frequency + damping). |
| `FixedJoint2D` | Fixed weld — one frequency feeds both the linear and angular stiffness. |
| `RelativeJoint2D` | Relative — a position spring drives the maintained linear + angular offset. |
| `FrictionJoint2D` | Relative with velocity-control caps (no spring). |
| `TargetJoint2D` | Relative pulling toward a world target; the null connected body resolves to a static world anchor. |

Per-joint options: anchors (body-local), an axis angle (degrees), the motor (`enableMotor` / `motorSpeed` / `maxMotorEffort`), a limit (`enableLimit` / `lowerLimit` / `upperLimit`), a spring (`enableSpring` / `springFrequency` / `springDamping`), the distance/spring rest length, the relative-family offset (`linearOffset` + `angularOffsetDegrees` with `maxForce` / `maxTorque` caps), and `collideConnected`. Joint angles are in degrees ([angular unit convention](angular-units.md)).

Every joint also carries the shared break parameters:

- **`breakForce` / `breakTorque`** (Infinity = never break) — set as the native force / torque thresholds.
- **`breakAction`** — `Ignore` (never break), `CallbackOnly` (surface the break event, keep the joint), or `Destroy` / `Disable` (surface the event and destroy the Box2D joint). A joint break surfaces a `PhysicsJointBreakEvent2D` ([runtime API](runtime-api.md)).

A joint's `connectedBody` should reference a concrete body. The null-connected "joint to a point in space" is faithful only for `TargetJoint2D` — see the [bake contract](bake-contract.md) for why.

## Effectors — `*Effector2D`

Box2D-v3 has no native effector; each of the five built-in effectors is reproduced as a per-step ECS apply over a baked `PhysicsEffector2D`. The effector GameObject's own collider bakes to its shape through the normal collider baker. The collider's `isTrigger` is the load-bearing distinction between the two families:

- **Force-field effectors (`AreaEffector2D`, `BuoyancyEffector2D`, `PointEffector2D`)** sit on a sensor collider (`isTrigger = 1`): a body falls through / floats in the region while the effector applies a force to every dynamic body inside, before the step, the same window the runtime write-in drains — so an effector force is mass-scaled and frozen-axis-cancelled exactly like a `Rigidbody2D.AddForce(_, Force)`.
- **Contact-response effectors (`SurfaceEffector2D`, `PlatformEffector2D`)** sit on a solid collider (`isTrigger = 0`): a body rests on the platform or rides the belt. Surface drives contacting bodies tangentially toward the belt speed; Platform gates the platform body's collision one-way from the surface arc.

| Effector | Key options |
|---|---|
| `AreaEffector2D` | `forceMagnitude`, `forceAngle`, `useGlobalAngle`, `forceTarget`, linear / angular drag. |
| `BuoyancyEffector2D` | `surfaceLevel`, `density` (fluid density), `flowMagnitude` / `flowAngle`, linear / angular drag. |
| `PointEffector2D` | `forceMagnitude` (negative attracts, positive repels), `distanceScale`, `forceMode` (Constant / InverseLinear / InverseSquared), `forceSource`, drag. |
| `SurfaceEffector2D` | `speed` (belt speed), `forceScale`, `useContactForce`, `useFriction`. |
| `PlatformEffector2D` | `surfaceArc`, `rotationalOffset`, `useOneWay`. |

All five share `colliderMask` (which layers the effector acts on). The Platform multi-body one-way and `colliderMask`-on-collider behaviours are documented known gaps, and the buoyancy submersion is an AABB approximation — read the [parity matrix](parity-matrix.md) before relying on an effector edge case.

## `PhysicsStep2DAuthoring` — the simulation config (package-shipped)

The package's one non-built-in world-config component, the 2D analogue of `com.unity.physics`'s `PhysicsStepAuthoring`. Place ONE on a GameObject in a SubScene to configure the simulation; it bakes to the `PhysicsWorld2DConfig` singleton read at world creation. The configuration is explicit per scene: nothing is read from the project's `Physics2D` settings, and a scene with no `PhysicsStep2DAuthoring` keeps the Box2D defaults — the inspector defaults mirror the defaults exactly, so a component left untouched is also behaviourally identical to no component.

The exposed fields (with their defaults):

| Field | Default | Meaning |
|---|---|---|
| `Gravity` (`float2`, m/s²) | `(0, -9.81)` | World gravity. |
| `SimulationSubSteps` (`int`, ≥1) | `4` | Box2D-v3 solver sub-steps per `Simulate`. |
| `SimulationWorkers` (`int`, ≥1) | `64` | Worker count (capped to device cores at runtime). |
| `ContinuousAllowed` (`bool`) | `true` | World-level Dynamic-vs-Static CCD. |
| `SleepingAllowed` (`bool`) | `true` | Whether bodies may sleep. |
| `BounceThreshold` (`float`, m/s) | `1` | Restitution velocity threshold. |
| `ContactHitEventThreshold` (`float`, m/s) | `1` | Hit-event speed threshold. |
| `ContactFrequency` (`float`, cycles/s) | `30` | Contact stiffness. |
| `ContactDamping` (`float`, 1 = critical) | `10` | Contact damping. |
| `ContactSpeed` (`float`, m/s) | `3` | Overlap-resolution speed. |
| `ContactRecycleDistance` (`float`, m) | `0.05` | Contact recycle distance (0 disables recycling). |
| `MaximumLinearSpeed` (`float`, m/s) | `400` | Linear speed cap. |

The simulation TYPE and the fixed TIMESTEP are deliberately not fields: the type is locked to `Script` (the package owns stepping), and the timestep is the `FixedStepSimulationSystemGroup` rate, an ECS-global group property. The [bake contract](bake-contract.md) records the full field set and the reasoning behind the deliberately-omitted knobs.
