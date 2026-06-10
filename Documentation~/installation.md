# Installation and requirements

## Requirements

The package targets the Unity 6000.x line with the DOTS stack. Its declared package dependencies are:

| Dependency | Version |
|---|---|
| Unity editor | 6000.0 or newer (the `package.json` `unity` field; the package was developed and validated against `6000.6.0a6`) |
| `com.unity.entities` | 6.5.0 |
| `com.unity.collections` | 6.5.0 |
| `com.unity.burst` | 1.8.29 |
| `com.unity.mathematics` | 1.3.2 |

The 2D physics binding the package steps over — `Unity.U2D.Physics` (the engine's low-level Box2D-v3 surface) — ships with the editor's 2D physics module, so it is not a separate package dependency. The package depends on no rendering package: it is renderer-agnostic and its output is `Unity.Transforms.LocalToWorld`.

The declared `6000.0` minimum is the baseline, not the validated editor. The package was built and its parity suite was measured against the alpha `6000.6.0a6`, where the GameObject reference path runs Box2D-v2 and the package runs Box2D-v3 ([Concepts](concepts.md), "The Box2D v2-vs-v3 reality"). On a different editor minor, verify any low-level `Unity.U2D.Physics` signature against that editor rather than trusting a captured doc.

## Installing the package

There are two supported install paths.

- **As a UPM package from a git URL.** Add the repository's git URL to the project through the Package Manager's "Add package from git URL," or add it directly to `Packages/manifest.json`:

  ```json
  {
    "dependencies": {
      "is.zori.entities.physics2d": "https://github.com/<owner>/<repo>.git"
    }
  }
  ```

- **As an embedded package.** Clone or copy the package into the project's `Packages/` directory (so the path is `Packages/is.zori.entities.physics2d/`). Unity auto-discovers a package placed there; an embedded package needs no `manifest.json` dependency line.

## Importing the sample

The package ships one sample, `Custom Authoring 2D`, registered in `package.json`. Import it from the Package Manager's Samples tab; it copies into the project's `Assets/Samples/`, where it is yours to edit. It demonstrates the low-level direct/bulk authoring surface and the custom authoring MonoBehaviours — see the [custom-authoring sample](custom-authoring.md) doc.

## Running the package tests

The package carries a PlayMode parity suite (the `Zori.Entities.Physics2D` assembly) and an EditMode fixture-builder assembly. To run the package's tests from a consuming project, the Unity Test Framework requires the package to be listed under `testables` in the project's `Packages/manifest.json`:

```json
{
  "testables": [
    "is.zori.entities.physics2d"
  ]
}
```

Without that entry, test discovery reports `testcasecount="0"` — the tests are present but hidden. The parity fixtures are authored as SubScenes built by EditMode `*FixtureBuilder` methods (they are not committed scenes), so a fresh checkout builds them via the fixture-builder `-executeMethod` entry points before the PlayMode suite runs.
