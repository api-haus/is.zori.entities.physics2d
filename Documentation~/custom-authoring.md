# Custom authoring

Custom authoring is the package's first-class surface for authoring physics bodies, shapes, and joints directly against the runtime, for when the built-in `Rigidbody2D` / `Collider2D` / `*Joint2D` components cannot express what you need. The authoring components (`PhysicsBody2DAuthoring`, `PhysicsShape2DAuthoring`, `PhysicsJoint2DAuthoring`) and their bakers live in the package's compiled `Authoring/` and `Baking/` assemblies and are unit-tested in the package. The DOTS `com.unity.physics` package ships its equivalent only as an importable "Custom Physics Authoring" sample; here it is a first-class package member. The 2D analogy to that sample is its authoring → baker → same-runtime structure only — the surface is complete for every 2D-expressible feature, not a literal 1:1 with the 3D type count (see [the deliberate 2D negative space](#the-deliberate-2d-negative-space) below).

## When to use it

Use the custom authoring when the built-in `Rigidbody2D` / `Collider2D` / `*Joint2D` cannot express what you need — a bake-time initial velocity, a free box or capsule orientation, an explicit contact-filter bitset, a custom centre-of-mass / inertia override, explicit geometry without round-tripping through a sprite/mesh-resolved collider, or a DOTS-native joint authored directly against the runtime joint definition. Use the bake path ([high-level authoring](authoring-high-level.md)) otherwise — it is the familiar, zero-new-concepts surface.

## A worked example scene

A worked example brings the whole surface together in one scene — the package's authoring bake fixture (`CustomAuthoring2D.unity`, exercised by `CustomAuthoring2DBakeSmoke`): a parent scene carrying one `SubScene` whose child holds nine authored GameObjects — CircleBody, BoxBody (free 20° `BoxAngle`), CapsuleBody (free 15° `CapsuleAngle`), PolygonBody (a five-vertex convex hull), a static EdgeWall (an open three-vertex chain), a static Floor (a collider-only box the bodies settle on), MaterialBody (a `PhysicsMaterial2D` template with a per-field friction override), and a filtered FilteredBodyA / FilteredBodyB pair (an explicit category/contact bitset). The custom inspectors and the scene-view gizmo outlines + draggable handles show at edit time with no renderer, and on Play the `SubScene` bakes and the bodies simulate, each body's `LocalToWorld` updating as it falls and settles.

## Body authoring — `PhysicsBody2DAuthoring`

A custom body authoring MonoBehaviour that authors the runtime `PhysicsBody2DDefinition` directly. Its baker emits the same `PhysicsBody2DDefinition` (and the same `PhysicsBody2DInitialVelocity`) the built-in `Rigidbody2DBaker` emits.

| Field | Type | What it authors |
|---|---|---|
| `BodyType` | `PhysicsBody2DMotionType` | Dynamic / Kinematic / Static — the package's own enum, so the custom surface depends on no built-in component. |
| `GravityScale` | float | Per-body gravity scale (`Rigidbody2D.gravityScale`). |
| `LinearDamping` / `AngularDamping` | float | Linear / angular velocity decay. |
| `UseAutoMass` / `Mass` | bool / float | Density-derived mass when `UseAutoMass`, else the explicit `Mass`. |
| `FreezePositionX` / `FreezePositionY` / `FreezeRotation` | bool | The per-DOF freeze constraints (`Rigidbody2D.constraints`). |
| `InitialLinearVelocity` / `InitialAngularVelocity` | `float2` / float (deg/s) | A bake-time initial velocity seed — the knob a built-in `Rigidbody2D` cannot author, because `Rigidbody2D.linearVelocity` is runtime-only. |
| `Interpolation` | `PhysicsBody2DInterpolation` | None / Interpolate / Extrapolate render-rate pose smoothing (`Rigidbody2D.interpolation`). |
| `CollisionDetection` | `PhysicsCollisionDetection2D` | Discrete / Continuous continuous-collision detection (`Rigidbody2D.collisionDetectionMode`). |
| `OverrideMassDistribution` + `CenterOfMass` + `RotationalInertia` | bool + `float2` + float | A custom mass distribution: when on, the explicit local-space centre of mass is applied, and a positive `RotationalInertia` overrides the shape-derived scalar inertia. A 2D body has one rotational DOF, so its inertia is a single scalar with no orientation to override. |

The mass-distribution override is applied in `PhysicsWorld2DSystem.ApplyMass` by writing `body.massConfiguration`. Because any explicit `massConfiguration` write shifts the Box2D-v3 sub-step solver's free-fall integration relative to the auto-mass path, the override is off by default and a body that does not opt in never enters that write; a body that does opt in lands in a slightly different integration band, which is the documented cost of the override.

## Shape authoring — `PhysicsShape2DAuthoring`

A custom shape authoring MonoBehaviour carrying the `PhysicsShape2DKind` union directly with explicit geometry and the surface material inline. Its baker emits the same `PhysicsShape2D` the built-in collider bakers emit, builds the Polygon/Edge vertex blob the same way, and adds the collider-only static-body fallback when no `PhysicsBody2DAuthoring` is present.

### Geometry by kind

| Kind | Fields | Notes |
|---|---|---|
| Circle | `Radius`, `Offset` | A circle of the given radius at the offset. |
| Box | `BoxSize`, `BoxAngle`, `Radius`, `Offset` | A box of full extents `BoxSize`, with a free local z-rotation `BoxAngle` (degrees) the built-in `BoxCollider2D` cannot express, and `Radius` as the corner-rounding radius. |
| Capsule | `CapsuleSize`, `CapsuleVertical`, `CapsuleAngle`, `Offset` | A capsule of the given size, long axis Y (`CapsuleVertical`) or X, with a free local z-rotation `CapsuleAngle` on top. `GetCapsuleCenters` derives the two end-cap centers exactly as the built-in capsule baker does, then rotates them. |
| Polygon | `Vertices`, `PolygonDecompose`, `Radius`, `Offset` | A convex 3–8-vertex hull (`PolygonDecompose` off), or a concave / over-8-vertex outline the runtime decomposes into convex fragments at creation (`PolygonDecompose` on). `Radius` rounds the corners. |
| Edge | `Vertices`, `EdgeIsLoop`, `Offset` | An open chain (`EdgeIsLoop` off) or a closed loop. An Edge is a one-sided static surface, not an enclosing solid. |

`Offset` is the collider's local offset, folded into the geometry at creation; the unused fields for a given kind are inert.

### Material template + per-field override

The surface coefficients support the 3D sample's override-flag-plus-value-plus-template inheritance model, adapted to 2D's native material asset. The template is an optional `MaterialTemplate` — a `UnityEngine.PhysicsMaterial2D`, **not** a bespoke ScriptableObject, because 2D already ships `PhysicsMaterial2D` as the material asset. Each of friction, bounciness, friction-combine, and bounciness-combine resolves with the precedence

$$\text{resolved} = \text{Override} \;?\; \text{inline} : (\text{template} \neq \text{null} \;?\; \text{template} : \text{inline})$$

— the inline value when its `Override…` flag is set, else the assigned template's value, else the inline value (which doubles as the no-template default). The baker takes a `DependsOn` dependency on the template, so editing the referenced `PhysicsMaterial2D` re-bakes every shape that references it.

Only what a `PhysicsMaterial2D` carries can be inherited: friction, bounciness, and the two combine modes. **Density** is a shape property (`Collider2D.density`), not on `PhysicsMaterial2D`, so it has no template source and stays the inline value with no override flag. **`CollisionResponse`** (Collide / Sensor, the 2D-expressible subset of the 3D four-way collision-response policy) likewise has no `PhysicsMaterial2D` source and stays inline.

### Contact filter

The contact filter resolves with a fixed precedence:

- `OverrideFilterBits` on → the explicit `CategoryBits` / `ContactBits` 32-bit masks are baked directly (widened to 64 bits, upper 32 zero), bypassing the layer matrix.
- else `Layer` in `[0..31]` → `categoryBits = 1 << Layer`, `contactBits = Physics2D.GetLayerCollisionMask(Layer)` from the project layer-collision matrix. The named categories are the project's Unity layer names — 2D reuses the layer system rather than inventing a 32-entry category-names asset.
- else `Layer == -1` (the default) → zero bits, and the creation system applies the collide-with-everything default.

## Auto-fit — `PhysicsShape2DAutoFit`

A static utility (in the `Authoring` assembly, so it is callable and unit-tested without the editor) that fits a shape to a 2D point source. It gathers a point cloud from one of three sources — a Sprite's physics shape, a SpriteRenderer's sprite bounds, or a PolygonCollider2D's paths — and produces best-fit parameters for the four enclosing kinds: an AABB or PCA-oriented box, a Welzl minimum-enclosing circle, a PCA capsule, or a convex hull (Andrew's monotone chain) with the ≤8/decompose decision. It writes the authoring fields directly, so a fitted shape bakes exactly as a hand-authored one (the fit emits unscaled local-space fields; the baker applies the transform scale). Edge is not a fit target — an open chain encloses nothing. The "Fit To…" dropdown in the shape inspector calls `FitToSprite` / `FitToSpriteRenderer` / `FitToPolygonCollider2D`.

## Joint authoring — `PhysicsJoint2DAuthoring`

A single custom joint authoring MonoBehaviour with a `PhysicsJoint2DKind` selector — the DOTS-native alternative to the nine built-in `*Joint2D` components, exactly as `PhysicsBody2DAuthoring` is the alternative to `Rigidbody2D`. It is ONE unified component over a kind enum rather than a per-type MonoBehaviour family, because the 2D package's own convention is unified-with-selector and the runtime `PhysicsJoint2DDefinition` is already a single tagged union over `PhysicsJoint2DKind`. Its baker emits the same `PhysicsJoint2DDefinition` the built-in `*Joint2DBaker`s emit, so a custom joint and the equivalent built-in `*Joint2D` of identical parameters bake field-identical.

The nine kinds (`PhysicsJoint2DKind`): Hinge, Slider, Wheel, Distance, Spring, Fixed, Relative, Friction, Target. Each kind consumes a specific subset of the authored fields, and the baker reproduces each built-in baker's field selection one-for-one; the per-kind editor (below) shows only the fields a given kind consumes, so a user never edits a field the bake drops.

| Field | Type | Kinds that use it | What it authors |
|---|---|---|---|
| `Kind` | `PhysicsJoint2DKind` | all | The joint type — the discriminant the baker switches on. |
| `ConnectedBody` | `PhysicsBody2DAuthoring` | all except Target | The connected body (Box2D `bodyA`; the owner GameObject is `bodyB`). Resolved through `GetEntity(..., Dynamic)`, which is surface-agnostic, so a custom joint can connect to a `PhysicsBody2DAuthoring`- or a `Rigidbody2D`-backed body. Null → a static anchor at the world origin. |
| `Anchor` / `ConnectedAnchor` | `float2` / `float2` | anchored kinds (zeroed at bake for Relative / Friction) | The owner-body-local anchor and the connected-body-local anchor. For Target, `ConnectedAnchor` is the world-space target point. |
| `AxisAngle` | float (deg) | Slider, Wheel | The slide / suspension axis direction. |
| `UseMotor` + `MotorSpeed` + `MaxMotorEffort` | bool + float + float | Hinge, Slider, Wheel | The `JointMotor2D` sub-surface: speed (deg/s for Hinge/Wheel, m/s for Slider) and the max motor torque (Hinge/Wheel) or force (Slider). |
| `UseLimits` + `LowerLimit` + `UpperLimit` | bool + float + float | Hinge (angle°), Slider (translation m) | The limit range. Wheel has no translation limit — the baker forces it off regardless of the toggle. |
| `Frequency` + `DampingRatio` | float + float | Wheel, Spring, Fixed, Target | The spring sub-surface (suspension / spring stiffness; for Fixed it is the weld stiffness, where frequency 0 = a rigid weld). The `enableSpring` flag is derived per kind by the baker to match each built-in baker, not authored. |
| `RestLength` | float | Distance, Spring | The rest length of the distance / spring constraint. |
| `LinearOffset` + `AngularOffset` | `float2` + float (deg) | Relative (Friction zeros offset; only force caps matter) | The relative-pose drive offset. |
| `MaxForce` + `MaxTorque` | float + float | Relative, Friction, Target (Target forces `MaxTorque` to 0) | The force / torque caps for the velocity- or position-control kinds. |
| `CollideConnected` | bool | all except Target | Whether the two connected bodies collide. Target is single-body, so its `collideConnected` is always baked true (forcing it false has no effect). |
| `BreakAction` + `BreakForce` + `BreakTorque` | `PhysicsJointBreakAction2D` + float + float | all | The break surface — the action (`Ignore` / `CallbackOnly` / `Destroy` / `Disable`) and the force / torque thresholds (default ∞ / never break). The custom surface authors the package break-action enum directly, with no built-in-enum mapping. The default action is `Destroy`, matching the built-in `Joint2D` default. |

`enableSpring` is derived per kind, not exposed: Wheel / Spring / Target carry it on, Distance / Fixed / Friction off, and Relative is a hardcoded stiff critically-damped spring (8 Hz / damping 1) that approximates the built-in's rigid relative-pose pull. Exposing the flag would let a user author a state with no built-in equivalent, breaking convergence. The five inert per-kind fields for a given kind (e.g. a Hinge's axis, a Friction's anchors) are dropped at bake, which is why the editor hides them.

## The 2D-native editor

A new `Zori.Entities.Physics2D.Editor` assembly provides custom inspectors, scene-view handles, gizmos, and the Fit dropdown for all three authoring components — a 2D-native rewrite informed by the 3D sample's inspector structure, not a line-port of its perspective handle math (see [the deliberate 2D negative space](#the-deliberate-2d-negative-space)).

- **Inspectors** — `PhysicsBody2DAuthoringEditor` shows the body fields conditionally by motion type (a disabled `∞` mass for non-Dynamic), with an Advanced foldout (collision detection, the mass-distribution override) and status HelpBoxes. `PhysicsShape2DAuthoringEditor` shows a kind selector, the per-kind geometry, a material section with per-coefficient override rows that preview the inherited template value, a filter section with a Unity-layer popup and a layer-named bit-mask, status messages, and the "Fit To…" dropdown. `PhysicsJoint2DAuthoringEditor` shows a kind selector and only the fields each kind consumes (the per-kind field map derived from the baker, the single source of truth), kind-aware unit labels (deg / m), and status HelpBoxes (an inverted limit range, a missing connected body, a sprung kind with frequency 0). For `Kind == Target` it hides the connected-body field, the collide-connected field, and the max-torque field (all three are forced or dropped at bake for a single-body world-anchor joint) and shows an info box explaining the single-body model so the absence is legible.
- **Scene handles** (`OnSceneGUI`, in the GameObject's local XY frame) — for shapes: a circle radius handle; a box's two half-extent sliders plus a rotation ring; a capsule's two end-cap handles plus a perpendicular radius handle; per-vertex drag for polygons and edges; and a common offset handle (vertices are added/removed via the inspector's vertex list). For joints: an owner-anchor handle and a connected-anchor / world-target handle with a connecting line between them; an axis-direction handle for Slider and Wheel; an angle-limit arc with draggable end markers for the Hinge; and a translation-limit segment with endpoint handles for the Slider. Relative and Friction draw only the body-origin connecting line (their anchors are zeroed at bake, so no anchor handle is shown).
- **Gizmos** — a `[DrawGizmo]` outline drawer renders every shape's outline and every joint's anchors / connecting line / axis / limit arc always-on (dim) and brighter when selected, drawn from pure outline and limit math (`PhysicsShape2DGizmos` and `PhysicsJoint2DGizmos` in the `Authoring` assembly). This is what makes the shapes and joints visible at edit time with no renderer.
- **Icons** — the authoring MonoBehaviours carry built-in 2D-physics icons so they are recognizable in the inspector and hierarchy: `PhysicsBody2DAuthoring` borrows the `Rigidbody2D` glyph, `PhysicsShape2DAuthoring` the `BoxCollider2D` glyph (the generic `Collider2D` name does not resolve in this editor, so a candidate list falls through to it), `PhysicsStep2DAuthoring` a settings glyph, and `PhysicsJoint2DAuthoring` the `HingeJoint2D` glyph.

The pure handle / outline / limit math lives in the `Authoring` assembly (`PhysicsShape2DGizmos`, `PhysicsJoint2DGizmos`) so it is unit-testable without the editor; the Editor assembly is a thin `UnityEditor.Handles` / `Gizmos` shell. The interactive drag, the inspector layout, and the gizmo render have no batchmode harness and are manual-QA surfaces.

## The convergence property

The load-bearing point of the custom surface is that the custom MonoBehaviours bake to the *same* runtime archetype the built-in components bake to. A body authored via `PhysicsBody2DAuthoring` and the equivalent `Rigidbody2D` / `Collider2D`-authored body land in one archetype and run one Box2D solver, so the step and write-back systems never ask which surface created a body. The convergence gate measured this directly: a `Baker<PhysicsBody2DAuthoring>` and a `Baker<Rigidbody2D>` fed identical parameters in identical creation order produce a **bit-identical trajectory** (0.0 worst error over 120 steps), and the same gate extends to a template-driven shape (a custom shape inheriting a `PhysicsMaterial2D` bakes bit-identical to a built-in collider carrying the same `sharedMaterial`). The joint surface converges the same way: a custom `PhysicsJoint2DAuthoring` and the equivalent built-in `*Joint2D` of identical parameters bake a **field-identical** `PhysicsJoint2DDefinition` for all nine kinds (the `connectedBody` entity aside, which differs by which GameObject each references), proven by a mutation-probe-verified gate. Any drift in a custom baker would split the trajectories or the structs immediately, which makes convergence the cleanest possible falsification gate.

## The deliberate 2D negative space

The custom surface is complete when it covers every **2D-expressible** feature with a 2D-native editor — not when its type count and handle math match the 3D sample. Box2D-v3 geometry is exactly `CircleGeometry`, `CapsuleGeometry`, `PolygonGeometry` (≤8 vertices), `SegmentGeometry`, and `ChainGeometry`, and a 2D scene-view needs no perspective math, so the following are designed out on purpose — reproducing them would reintroduce exactly what the 2D domain leaves out:

- **No Cylinder, Plane, or 3D-mesh shape type.** Box2D-v3 has none. A 2D "cylinder" is a box or a rounded box, a "plane/ground" is a box or an Edge/Segment, and a "mesh collider" is a Chain (an edge loop) or a decomposed convex Polygon set. The package's kinds are exactly the five Box2D-v3 shapes.
- **No 3D-perspective handle math.** The 3D sample's bevelled-bounds-handle family and its corner-horizon / backfacing utility are perspective-correct 3D wireframe machinery with no 2D need; the 2D handles are flat XY sliders, free-move dots, and wire discs with no backfacing or horizon.
- **No async convex/mesh preview jobs.** The 3D async preview exists because convex-hull / mesh-collider blob creation is expensive; 2D outlines are cheap to draw synchronously from `float2` math.
- **No `PhysicsCategoryNames` / material-tags ScriptableObject.** 2D reuses the project's named Unity layers as the "named categories" and `PhysicsMaterial2D` as the material asset, so the 3D sample's 32-entry category-names asset and bespoke material-template asset are not invented.
- **No inertia-tensor orientation.** A 2D body's rotational inertia is a single scalar with one DOF, so the 3D inertia-tensor orientation field is meaningless and is not authored.
- **No 3D joint / motor types with no 2D analogue.** The 3D sample ships a per-type joint family (`BallAndSocket`, `FreeHinge`, `LimitedHinge`, `LimitDOF`, `Prismatic`, `Ragdoll`, `Rigid`, `LimitedDistance`) plus separate motor MonoBehaviours (`AngularVelocityMotor`, `LinearVelocityMotor`, `PositionMotor`, `RotationalMotor`), each its own `BaseJoint` over a distinct `PhysicsJoint.CreateX` factory. The 2D joint set is exactly the nine `*Joint2D` kinds, and several 3D types collapse into them: a 2D ball-and-socket IS a Hinge (a 2D point pin), a 2D ragdoll cone-twist collapses to a Hinge with angle limits (one rotational DOF), `Prismatic` IS the Slider, and `LimitDOF`'s per-axis locking maps to the body's freeze toggles, not a joint. A 3D motor is its own joint; a 2D motor is the `JointMotor2D` sub-surface riding on Hinge / Slider / Wheel, so there is no standalone position or rotational motor joint, and inventing one would be a non-converging 3D port. The per-type 3D editor family (`BallAndSocketJointEditor`, `LimitedHingeJointEditor`, `RagdollJointEditor`) collapses to the one kind-switched `PhysicsJoint2DAuthoringEditor` for the same reason.

Two 3D body knobs are genuine omissions rather than negative space: **`WorldIndex`** (multi-world sharding, which needs the multi-world model the package defers) and **`SolverType`** (no per-body solver field on the 2D archetype). They are the natural additive extension when that infrastructure lands. The 3D per-joint `MaxImpulse` / `SolverType` / `WorldIndex` knobs are absent for the same reason; the 2D break surface (`BreakForce` / `BreakTorque` / `BreakAction`) is the 2D analogue of the 3D impulse-event threshold, and it IS exposed.

## Known limitations and a manual-QA note

- **Auto-fit silently convex-hulls a concave outline whose hull is small enough.** The Polygon auto-fit decides single-hull-vs-decompose on the convex-hull vertex count, not on whether the source outline is concave. A concave outline whose convex hull is within the Box2D ≤8-vertex cap takes the single-hull path and is filled to its convex hull — the fitted shape over-covers the concave notch. This is reviewable at edit time (the gizmo outline shows the filled hull, not the concave outline) and is not a runtime defect; a concave fit that follows the outline faithfully needs the decompose path, which the auto-fit reaches only when the hull exceeds the cap.
- **Capsule end-cap drag re-centres the capsule (manual QA).** The capsule's authoring model is size + vertical/horizontal + angle, so the scene-view handle derives the capsule from those fields and re-derives the centre as the midpoint of the two end-caps. Dragging one end-cap therefore shifts the other as the capsule re-centres about its midpoint — a symmetric-about-centre model that is always geometrically correct but is not a "pin one end, drag the other" feel. The field-from-drag arithmetic round-trips correctly through `GetCapsuleCenters` at every spine angle; the interaction is an ergonomics call to evaluate by hand.
- **Relative / Target custom-joint behavioral parity is bake-pinned only.** All nine custom joint kinds bake field-identical to their built-in `*Joint2D` twins, and the three constraint kinds with a stable transient (Hinge / Slider / Wheel) are also directly witnessed to simulate identically (Wheel bit-identical). Relative and Target are proven only at the bake level: they are stiff-spring position controllers, and on the convergence fixture's frictionless symmetric body the floating-point per-body solve noise amplifies chaotically, so a tight behavioral band would falsely fail a correct baker. Their simulated parity rests on field-identity (identical struct → identical Box2D joint) plus the built-in `JointParityValidation`. See the [parity matrix](parity-matrix.md#relative--target-custom-joint-behavioral-parity-is-bake-pinned-only).

## Authoring-component caveat

The two authoring MonoBehaviours use `math.max(...)` rather than the project's `using static Unity.Mathematics.math;` + bare `max()` preference, because the static import collides with `float2` in type position inside a MonoBehaviour field initializer (`new float2(...)` parses against the `math.float2` factory method, CS0119). The runtime ECS structs do not hit this because they do not combine the static import with `new float2(...)` field initializers. This is a deliberate local deviation, justified because these are cold authoring components.
