# Angular unit convention

This is the single canonical home for the package's angular units; every component, command, and baker doc references this page rather than restating it. The convention follows the engine, which has a mixed angular surface, not a single unit.

## A body's rotation angle is in radians

A body's rotation ANGLE is in radians. The engine's rotation type `Unity.U2D.Physics.PhysicsRotate` is unit-agnostic — it stores a normalized cos/sin direction, not an angle scalar, and constructs from either `FromRadians` or `FromDegrees` — so for body pose the engine demands no particular unit, and the package chooses radians to match the Entities / `Unity.Mathematics` / `LocalToWorld` `float4x4` ecosystem. This covers:

- `PhysicsBody2DDefinition.initialRotationRadians` — the baked initial rotation. The `Rigidbody2DBaker` converts the GameObject's `Transform.eulerAngles.z` (degrees) to radians once with `math.radians`.
- `PhysicsBody2DCommands.MoveRotation` and `MovePositionAndRotation` — their `targetRadians` parameter.
- The effector angles the package evaluates with its own `sincos` / `atan2`: `forceAngleRadians`, `flowAngleRadians`, `surfaceArcRadians`, `rotationalOffsetRadians`.

Because `UnityEngine.Rigidbody2D.MoveRotation` takes DEGREES, a verbatim port of `rb.MoveRotation(90f)` must convert: `MoveRotation(math.radians(90f))`.

## An angular velocity is in degrees per second, and joint angles are in degrees

An angular VELOCITY is in degrees per second, and a joint's angular parameters (hinge angle limits and spring target angle, hinge/wheel motor speed, slide/suspension axis angle, relative angular offset) are in degrees. Here the engine IS built in degrees — `PhysicsBody.angularVelocity` and `PhysicsBodyDefinition.angularVelocity` are documented "in degrees per second," and the hinge `lower/upperAngleLimit` and motor `motorSpeed` are documented "in degrees" / "degrees per second" — and so are the matching `Rigidbody2D` / `Joint2D` fields. The package complies rather than rebase a deg/sec engine onto rad/sec, which would inject a `180/π` conversion at every velocity and joint site and diverge from the GameObject unit. This covers:

- `PhysicsBody2DInitialVelocity.angularVelocity` (the initial-velocity seed).
- `PhysicsBody2DCommands.SetAngularVelocity` — its `degreesPerSecond` parameter.
- The joint definition's `motorSpeed`, `lowerLimit` / `upperLimit` (hinge), `axisAngleDegrees`, and `angularOffsetDegrees`.

## Two corollaries

- An `AddTorque` value is a torque (N·m) or an angular impulse, not an angle, so it carries no radians/degrees unit.
- The smoothing system's internal `angularVelRad` is a private rad/sec representation (converted at capture for the extrapolation trig), not a user-facing API.

This is the package's ratified convention, not a parity gap — the `MoveRotation` radians-vs-degrees question is settled here, and the parity gate proves the package's radians input drives the same physical rotation `Rigidbody2D.MoveRotation`'s degrees input does under conversion.
