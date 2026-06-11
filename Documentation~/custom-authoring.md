# Custom authoring (`Samples~/CustomAuthoring2D`)

`CustomAuthoring2D` is the importable sample (registered in `package.json` `samples`, imported through the Package Manager into `Assets/`) that demonstrates the complete custom-authoring surface and gives you an editable starting point for beyond-built-in authoring. The authoring components and their bakers live in the package's compiled `Authoring/` and `Baking/` assemblies (so they are unit-tested in the package); the sample is the authored scene + the worked low-level example that reference them. It is the 2D analogue of the DOTS `com.unity.physics` "Custom Physics Authoring" sample, used for its authoring → baker → same-runtime structure only — the surface is complete for every 2D-expressible feature, not a literal 1:1 with the 3D type count (see [the deliberate 2D negative space](#the-deliberate-2d-negative-space) below).

## When to use it

Use the custom authoring when the built-in `Rigidbody2D` / `Collider2D` cannot express what you need — a bake-time initial velocity, a free box or capsule orientation, an explicit contact-filter bitset, a custom centre-of-mass / inertia override, or explicit geometry without round-tripping through a sprite/mesh-resolved collider. Use the bake path ([high-level authoring](authoring-high-level.md)) otherwise — it is the familiar, zero-new-concepts surface.

## The sample scene

The sample ships an authored scene under `Samples~/CustomAuthoring2D/Scenes/`: a parent scene (`CustomAuthoring2DSample.unity`) carrying one `SubScene` whose child holds nine authored GameObjects — CircleBody, BoxBody (free 20° `BoxAngle`), CapsuleBody (free 15° `CapsuleAngle`), PolygonBody (a five-vertex convex hull), a static EdgeWall (an open three-vertex chain), a static Floor (a collider-only box the bodies settle on), MaterialBody (a `PhysicsMaterial2D` template with a per-field friction override), and a filtered FilteredBodyA / FilteredBodyB pair (an explicit category/contact bitset). On import you see the custom inspectors and the scene-view gizmo outlines + draggable handles at edit time (no renderer required), and on Play the SubScene bakes and the bodies simulate, each body's `LocalToWorld` updating as it falls and settles. The sample carries no `com.unity.entities.graphics` dependency: the edit-time gizmos are the visual, and a runtime-rendered view is a separate, later benchmark sample.

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

## The 2D-native editor

A new `Zori.Entities.Physics2D.Editor` assembly provides custom inspectors, scene-view handles, gizmos, and the Fit dropdown — a 2D-native rewrite informed by the 3D sample's inspector structure, not a line-port of its perspective handle math (see [the deliberate 2D negative space](#the-deliberate-2d-negative-space)).

- **Inspectors** — `PhysicsBody2DAuthoringEditor` shows the body fields conditionally by motion type (a disabled `∞` mass for non-Dynamic), with an Advanced foldout (collision detection, the mass-distribution override) and status HelpBoxes. `PhysicsShape2DAuthoringEditor` shows a kind selector, the per-kind geometry, a material section with per-coefficient override rows that preview the inherited template value, a filter section with a Unity-layer popup and a layer-named bit-mask, status messages, and the "Fit To…" dropdown.
- **Scene handles** (`OnSceneGUI`, in the GameObject's local XY frame) — a circle radius handle; a box's two half-extent sliders plus a rotation ring; a capsule's two end-cap handles plus a perpendicular radius handle; per-vertex drag for polygons and edges; and a common offset handle. Vertices are added/removed via the inspector's vertex list.
- **Gizmos** — a `[DrawGizmo]` outline drawer renders every shape's outline always-on (dim) and brighter when selected, drawn from pure outline math (`PhysicsShape2DGizmos` in the `Authoring` assembly). This is what makes the shapes visible at edit time with no renderer.
- **Icons** — the authoring MonoBehaviours carry built-in 2D-physics icons (`Rigidbody2D` / `BoxCollider2D` / settings glyphs) so they are recognizable in the inspector and hierarchy.

## The convergence property

The load-bearing point of the custom surface is that the custom MonoBehaviours bake to the *same* runtime archetype the built-in components bake to. A body authored via `PhysicsBody2DAuthoring` and the equivalent `Rigidbody2D` / `Collider2D`-authored body land in one archetype and run one Box2D solver, so the step and write-back systems never ask which surface created a body. The convergence gate measured this directly: a `Baker<PhysicsBody2DAuthoring>` and a `Baker<Rigidbody2D>` fed identical parameters in identical creation order produce a **bit-identical trajectory** (0.0 worst error over 120 steps), and the same gate extends to a template-driven shape (a custom shape inheriting a `PhysicsMaterial2D` bakes bit-identical to a built-in collider carrying the same `sharedMaterial`). Any drift in the custom baker would split the trajectories immediately, which makes convergence the cleanest possible falsification gate.

## The deliberate 2D negative space

The custom surface is complete when it covers every **2D-expressible** feature with a 2D-native editor — not when its type count and handle math match the 3D sample. Box2D-v3 geometry is exactly `CircleGeometry`, `CapsuleGeometry`, `PolygonGeometry` (≤8 vertices), `SegmentGeometry`, and `ChainGeometry`, and a 2D scene-view needs no perspective math, so the following are designed out on purpose — reproducing them would reintroduce exactly what the 2D domain leaves out:

- **No Cylinder, Plane, or 3D-mesh shape type.** Box2D-v3 has none. A 2D "cylinder" is a box or a rounded box, a "plane/ground" is a box or an Edge/Segment, and a "mesh collider" is a Chain (an edge loop) or a decomposed convex Polygon set. The package's kinds are exactly the five Box2D-v3 shapes.
- **No 3D-perspective handle math.** The 3D sample's bevelled-bounds-handle family and its corner-horizon / backfacing utility are perspective-correct 3D wireframe machinery with no 2D need; the 2D handles are flat XY sliders, free-move dots, and wire discs with no backfacing or horizon.
- **No async convex/mesh preview jobs.** The 3D async preview exists because convex-hull / mesh-collider blob creation is expensive; 2D outlines are cheap to draw synchronously from `float2` math.
- **No `PhysicsCategoryNames` / material-tags ScriptableObject.** 2D reuses the project's named Unity layers as the "named categories" and `PhysicsMaterial2D` as the material asset, so the 3D sample's 32-entry category-names asset and bespoke material-template asset are not invented.
- **No inertia-tensor orientation.** A 2D body's rotational inertia is a single scalar with one DOF, so the 3D inertia-tensor orientation field is meaningless and is not authored.

Two 3D body knobs are genuine omissions rather than negative space: **`WorldIndex`** (multi-world sharding, which needs the multi-world model the package defers) and **`SolverType`** (no per-body solver field on the 2D archetype). They are the natural additive extension when that infrastructure lands.

## Known limitations and a manual-QA note

- **Auto-fit silently convex-hulls a concave outline whose hull is small enough.** The Polygon auto-fit decides single-hull-vs-decompose on the convex-hull vertex count, not on whether the source outline is concave. A concave outline whose convex hull is within the Box2D ≤8-vertex cap takes the single-hull path and is filled to its convex hull — the fitted shape over-covers the concave notch. This is reviewable at edit time (the gizmo outline shows the filled hull, not the concave outline) and is not a runtime defect; a concave fit that follows the outline faithfully needs the decompose path, which the auto-fit reaches only when the hull exceeds the cap.
- **Capsule end-cap drag re-centres the capsule (manual QA).** The capsule's authoring model is size + vertical/horizontal + angle, so the scene-view handle derives the capsule from those fields and re-derives the centre as the midpoint of the two end-caps. Dragging one end-cap therefore shifts the other as the capsule re-centres about its midpoint — a symmetric-about-centre model that is always geometrically correct but is not a "pin one end, drag the other" feel. The field-from-drag arithmetic round-trips correctly through `GetCapsuleCenters` at every spine angle; the interaction is an ergonomics call to evaluate by hand.

## Authoring-component caveat

The two authoring MonoBehaviours use `math.max(...)` rather than the project's `using static Unity.Mathematics.math;` + bare `max()` preference, because the static import collides with `float2` in type position inside a MonoBehaviour field initializer (`new float2(...)` parses against the `math.float2` factory method, CS0119). The runtime ECS structs do not hit this because they do not combine the static import with `new float2(...)` field initializers. This is a deliberate local deviation, justified because these are cold authoring components.
