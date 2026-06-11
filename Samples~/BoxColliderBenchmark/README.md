# Box Collider Creation Benchmark

This sample is the shipped demonstration AND the measurement vehicle for the cached-body-template creation optimisation. It spray-spawns quads carrying the package's **box collider** (`PhysicsShape2DKind.Box`), renders them with the official **Unity.Entities.Graphics** DOTS renderer straight off the `Unity.Transforms.LocalToWorld` the package writes, and times the per-entity body-creation cost so that running the same scene with the optimisation on versus off — across thresholds — is the measurement.

## What it shows

One self-describing quad prefab carries a dynamic box-collider body (its `PhysicsBody2DDefinition` + `PhysicsShape2D`), a `PhysicsBody2DFormHash` (so the runtime recognises the shared form), and the Entities.Graphics render components (a `RenderMeshArray` quad + a `MaterialMeshInfo`). The spawner `ecb.Instantiate(prefab)`s a few copies per frame over many frames — the cross-frame spray a falling-sand world produces, not a single bulk burst — scattering each instance's pose. The instances are self-describing, so `PhysicsWorld2DSystem`'s creation loop recognises the shared form via the replicated form hash and serves them from a cached body template once the form's count crosses the configured threshold.

The rendering integration is the point of the box-collider sample: the prefab carries **no transform glue**. The package's `PhysicsBody2DWriteBackSystem` writes each body's pose into `LocalToWorld` every fixed step, and Unity.Entities.Graphics renders each instance directly from that `LocalToWorld`. The package stays renderer-agnostic — it writes `LocalToWorld` and lets any renderer consume it; this sample is where the official DOTS renderer consumes it.

## This sample requires `com.unity.entities.graphics`

The package itself declares **no** Entities.Graphics dependency (it is renderer-agnostic), so importing this sample requires adding the renderer to the consuming project's manifest. Add this line to your project's `Packages/manifest.json` `dependencies`:

```jsonc
"com.unity.entities.graphics": "6.5.0"
```

The version pairs with `com.unity.entities` `6.5.0`. Without it, the sample's asmdef (which references `Unity.Entities.Graphics`) does not compile and the quads do not render.

## How to run it

1. Add `com.unity.entities.graphics` to your manifest (above).
2. Add a `BoxColliderBenchmarkConfig` singleton to a scene — from a bootstrap system, a tiny authoring MonoBehaviour + baker, or a directly-authored entity — to set the workload shape (`count`, `perFrame`, `boxSize`, the spawn AABB). The sample is inert until this singleton exists.
3. Add a `PhysicsStep2DAuthoring` to the scene (in a SubScene). This is the **control surface** — its `Cache Identical Bodies` (on/off) and `Identical Body Threshold` (N) fields drive the optimisation. This is the package's existing per-world config, not a sample-specific knob.
4. Press Play. The spray spawns, the quads render via Entities.Graphics off the physics `LocalToWorld`, and the timing instrument logs one line per creation frame to the console.

## Reading the timing

`BodyCreationTimingBeginSystem` / `BodyCreationTimingEndSystem` bracket `PhysicsWorld2DSystem`'s update and log, per creation frame:

```
[BoxColliderBenchmark] created=16 live=128 frame=420.0us perBody=26.250us (running: total=128 perBody=24.100us) [...]
```

- `created` — bodies created this frame; `live` — total live bodies after.
- `frame` — the bracketed window in microseconds; `perBody` — `frame / created`.
- `running total` / `running perBody` — cumulative across the spray.

**What the bracket measures, plainly.** `PhysicsWorld2DSystem` does body creation AND the per-step `Simulate(dt)` in one update — they are not separable from outside the package, so the bracket measures creation **plus** the same-frame simulate, not creation alone. The number that isolates the creation-path saving is the **on/off delta at matched body counts**: run the same scene/spray once with `Cache Identical Bodies` ON and once OFF, and compare the per-frame times at equal `live` counts. Simulate cost is identical across the two arms for the same live-body count, so the difference is the creation-path saving. The per-frame log (not just a total) exposes the warm-up tax below the threshold — the frames before the form's count crosses N take the per-entity path even with the cache on.

## The honest, bounded saving

A box is a mid-complexity form — `PolygonGeometry.CreateBox` with a `PhysicsTransform`, more than a circle's two floats, less than a decomposed concave polygon. The cache removes the per-entity **C# definition construction + mass arithmetic**; it does **not** remove the irreducible per-body `CreateBody`/`CreateShape` native calls (Box2D-v3 has no clone/share/template primitive — N bodies cost N of each, full stop). So the on/off delta is **real but bounded**. The instrument reports the measured delta as it is; if it is small, the readout says so rather than overselling. The honest deliverable is a documented "boxes save X% of the creation-side C# cost; the native body/shape allocation is unchanged," not a sample tuned to flatter the feature.

## Meaningful timing is deck-only

Body creation is CPU / managed-marshalling cost. Per the project's benchmarking policy, this is timed on the **target device's CPU** (the Steam Deck), because a faster desktop single thread understates the marshalling cost that motivates the optimisation. The desktop run verifies the sample works — it renders, the toggle takes effect, the instrument logs a number — but the desktop timing numbers are discarded, not reported. The threshold-default-setting sweep (optimisation OFF vs ON, N over {1, 2, 4, 8, 16, 32}, N swept over {1k, 4k, 16k}) is a deck measurement.
