using Unity.Collections;
using Unity.Entities;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Creates the Box2D joint for each baked entity carrying a <see cref="PhysicsJoint2DDefinition"/>,
    /// deferred until BOTH the joint-owner entity and its connected entity have a live
    /// <see cref="PhysicsBody2D"/> handle — a joint references two bodies, so unlike a body/shape it cannot
    /// be created the instant it bakes. Stores the resulting <see cref="PhysicsJoint"/> as a
    /// <see cref="PhysicsJoint2D"/> on the owner entity and tears every joint down with
    /// <c>DestroyJointBatch</c> in <c>OnDestroy</c>.
    /// </summary>
    /// <remarks>
    /// <b>Ordering — why before the step.</b> The system runs <c>[UpdateBefore(PhysicsWorld2DSystem)]</c> in
    /// the same <see cref="Physics2DSimulationSystemGroup"/>. The body-creation/step protocol is: on the
    /// group update that first creates the bodies, <see cref="PhysicsWorld2DSystem"/> skips its
    /// <c>Simulate</c> (creation and integration are decoupled), so the bodies sit at their authored pose for
    /// one update; the NEXT update steps them. Running joint creation just before
    /// <see cref="PhysicsWorld2DSystem"/> means that on that next update the joints are created first and the
    /// step that follows already honours the constraint — so a jointed body is never integrated free for even
    /// one step before its joint exists. This matches the GameObject reference, where the built-in joint is
    /// present from body instantiation and the first <c>Physics2D.Simulate</c> already constrains the bodies.
    /// On the very first update no bodies exist yet, so no joint is created; the deferral query handles that
    /// naturally (the bodies lack <see cref="PhysicsBody2D"/>).
    ///
    /// <b>Not <c>[BurstCompile]</c>.</b> <c>CreateJoint</c> is a managed <c>Unity.U2D.Physics</c> instance
    /// method on the main thread, exactly like <c>CreateBody</c>/<c>CreateShape</c> — joint creation joins the
    /// body-creation phase on the managed side, with Burst confined to the write-back job.
    ///
    /// <b>Teardown.</b> Destroying the <see cref="PhysicsWorld"/> already frees every joint it owns (a joint
    /// is owned by the world like a body/shape), so the <c>DestroyJointBatch</c> call here is the explicit,
    /// world-still-valid path for an orderly shutdown and the seam per-joint break/despawn extends;
    /// it is a no-op leak-safety belt when the world is torn down first.
    /// </remarks>
    [UpdateInGroup(typeof(Physics2DSimulationSystemGroup))]
    [UpdateBefore(typeof(PhysicsWorld2DSystem))]
    public partial struct PhysicsJoint2DCreationSystem : ISystem
    {
        // A lazily-created static body at the world origin that serves as bodyA for joints whose built-in
        // connectedBody is null — the built-in "joint to a point in space" (world anchor). Box2D has no
        // implicit ground body, so the package supplies one static anchor per world; null-connected joints'
        // connectedAnchor is a WORLD-space point, which is this anchor's local space (it sits at the origin
        // with identity rotation). Cached so all null-connected joints share one anchor body, and rebuilt
        // whenever the handle goes invalid (a physics-module reset recreates the world and invalidates every
        // handle it owned, including this anchor). PhysicsBody is a blittable 64-bit-ID struct, so it lives
        // directly in the unmanaged ISystem state.
        PhysicsBody _worldAnchorBody;

        public void OnDestroy(ref SystemState state)
        {
            // Collect every live joint handle and destroy them in one batch while the world is still valid.
            // Guard on the world: if PhysicsWorld2DSystem already tore it down, the joints went with it.
            if (!SystemAPI.TryGetSingleton<PhysicsWorldSingleton2D>(out var singleton) || !singleton.world.isValid)
                return;

            var jointQuery = SystemAPI.QueryBuilder().WithAll<PhysicsJoint2D>().Build();
            var count = jointQuery.CalculateEntityCount();
            if (count == 0)
                return;

            using var jointComponents = jointQuery.ToComponentDataArray<PhysicsJoint2D>(Allocator.Temp);
            var handles = new NativeArray<PhysicsJoint>(count, Allocator.Temp);
            for (var i = 0; i < count; i++)
                handles[i] = jointComponents[i].joint;
            // DestroyJointBatch is a static method on PhysicsWorld (the world is implied by the joint handles).
            PhysicsWorld.DestroyJointBatch(handles.AsReadOnlySpan());
            handles.Dispose();
        }

        public void OnUpdate(ref SystemState state)
        {
            // No joints waiting to be created → nothing to do. (RequireForUpdate would also gate the
            // OnDestroy teardown query off, so the gate is an explicit early-out here instead.)
            // WithNone<PhysicsJoint2DBroken>: a joint that has broken (Destroy/Disable) keeps its
            // PhysicsJoint2DDefinition but is tagged broken, so it is never re-created here.
            var pendingQuery = SystemAPI
                .QueryBuilder()
                .WithAll<PhysicsJoint2DDefinition>()
                .WithNone<PhysicsJoint2D, PhysicsJoint2DBroken>()
                .Build();
            if (pendingQuery.IsEmpty)
                return;

            // The world must exist (PhysicsWorld2DSystem creates it). On the first update it may not yet,
            // but then no bodies exist either, so the per-joint readiness check below would skip anyway.
            if (!SystemAPI.TryGetSingleton<PhysicsWorldSingleton2D>(out var singleton) || !singleton.world.isValid)
                return;

            var world = singleton.world;
            var bodyLookup = SystemAPI.GetComponentLookup<PhysicsBody2D>(isReadOnly: true);

            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (
                var (defRO, entity) in SystemAPI
                    .Query<RefRO<PhysicsJoint2DDefinition>>()
                    .WithNone<PhysicsJoint2D, PhysicsJoint2DBroken>()
                    .WithEntityAccess()
            )
            {
                var d = defRO.ValueRO;

                // Defer until the owner's body exists.
                if (!bodyLookup.HasComponent(entity))
                    continue;
                var bodyB = bodyLookup[entity].body; // owner = bodyB
                if (!bodyB.isValid)
                    continue;

                // Resolve bodyA (the connected body). A concrete connectedBody must also have been created;
                // a null connectedBody is the built-in "joint to a point in space" — bodyA is the shared
                // static world anchor, and connectedAnchor is then a world-space point.
                PhysicsBody bodyA;
                if (d.connectedBody == Entity.Null)
                {
                    bodyA = EnsureWorldAnchor(world);
                }
                else
                {
                    if (!bodyLookup.HasComponent(d.connectedBody))
                        continue; // connected body not created yet — defer
                    bodyA = bodyLookup[d.connectedBody].body;
                }
                if (!bodyA.isValid)
                    continue;

                var joint = CreateJoint(world, bodyA, bodyB, d);
                if (joint.isValid)
                {
                    ArmBreakThreshold(joint, entity, d);
                    ecb.AddComponent(entity, new PhysicsJoint2D { joint = joint });
                }
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        // Lazily create (and cache, per world) a static body at the origin to serve as bodyA for joints whose
        // built-in connectedBody is null. Rebuilt when the world handle changes (a physics-module reset
        // recreates the world, invalidating the cached anchor). The anchor carries no shape — a joint
        // attaches to a body, not a shape, so a shapeless static body is a valid, zero-cost world anchor.
        PhysicsBody EnsureWorldAnchor(PhysicsWorld world)
        {
            // Cache hit: a still-valid anchor. An anchor created against a since-recreated world reads invalid
            // (its handle was destroyed with the old world), forcing a rebuild against the current world.
            if (_worldAnchorBody.isValid)
                return _worldAnchorBody;

            var anchorDef = PhysicsBodyDefinition.defaultDefinition;
            anchorDef.type = PhysicsBody.BodyType.Static;
            anchorDef.position = Vector2.zero;
            anchorDef.transformWriteMode = PhysicsBody.TransformWriteMode.Off;
            _worldAnchorBody = world.CreateBody(anchorDef);
            return _worldAnchorBody;
        }

        // Arm the joint's native break threshold and pack its owner entity into the joint userData.
        //
        // Box2D-v3 has native break support: setting forceThreshold/torqueThreshold (base PhysicsJoint, XML
        // P:…PhysicsJoint.forceThreshold/.torqueThreshold) makes the engine produce a jointThresholdEvent the
        // step the joint's reaction force/torque exceeds the threshold — Box2D does NOT destroy the joint, the
        // package does (per breakAction, in PhysicsJoint2DBreakSystem). The threshold is set on the base joint
        // handle (one site, vs nine per-definition arms) AFTER creation; the IL confirms the setter is on the
        // base IPhysicsJoint. It is armed ONLY when the action is not Ignore AND the relevant break value is
        // finite — the built-in Joint2D default breakForce/breakTorque is Infinity (never break), which must NOT
        // become a zero/low threshold that fires every step. userData packs the owner entity (the joint rides on
        // bodyB's entity) so CollectJointBreaks can resolve a threshold event's joint back to its entity, exactly
        // as body userData resolves a query/contact hit. Managed Unity.U2D.Physics calls, main thread — not Burst.
        static void ArmBreakThreshold(PhysicsJoint joint, Entity entity, in PhysicsJoint2DDefinition d)
        {
            joint.userData = PhysicsQueries2D.PackEntity(entity);

            if (d.breakAction == PhysicsJointBreakAction2D.Ignore)
                return;
            if (!float.IsInfinity(d.breakForce) && !float.IsNaN(d.breakForce))
                joint.forceThreshold = d.breakForce;
            if (!float.IsInfinity(d.breakTorque) && !float.IsNaN(d.breakTorque))
                joint.torqueThreshold = d.breakTorque;
        }

        // Build the matching Box2D joint definition for the baked kind and create it. The anchors are already
        // each body's local space (the built-in anchor/connectedAnchor are body-local), so they fold directly
        // into the localAnchorA/B frames; the slide/suspension axis is the rotation of localAnchorA. Managed
        // Unity.U2D.Physics calls, main thread — not Burst.
        static PhysicsJoint CreateJoint(
            PhysicsWorld world,
            PhysicsBody bodyA,
            PhysicsBody bodyB,
            in PhysicsJoint2DDefinition d
        )
        {
            var anchorA = new PhysicsTransform((Vector2)d.connectedAnchor); // on bodyA (connected)
            var anchorB = new PhysicsTransform((Vector2)d.anchor); // on bodyB (owner)

            switch (d.kind)
            {
                case PhysicsJoint2DKind.Hinge:
                {
                    var def = PhysicsHingeJointDefinition.defaultDefinition;
                    def.bodyA = bodyA;
                    def.bodyB = bodyB;
                    def.localAnchorA = anchorA;
                    def.localAnchorB = anchorB;
                    def.collideConnected = d.collideConnected;
                    def.enableMotor = d.enableMotor;
                    def.motorSpeed = d.motorSpeed;
                    def.maxMotorTorque = d.maxMotorEffort;
                    def.enableLimit = d.enableLimit;
                    def.lowerAngleLimit = d.lowerLimit;
                    def.upperAngleLimit = d.upperLimit;
                    return world.CreateJoint(def);
                }

                case PhysicsJoint2DKind.Slider:
                {
                    var def = PhysicsSliderJointDefinition.defaultDefinition;
                    def.bodyA = bodyA;
                    def.bodyB = bodyB;
                    // The slide axis is the local X of localAnchorA's frame: rotate the connected anchor frame
                    // by the authored axis angle (SliderJoint2D.angle, in degrees).
                    def.localAnchorA = new PhysicsTransform(
                        (Vector2)d.connectedAnchor,
                        new PhysicsRotate(radians(d.axisAngleDegrees))
                    );
                    def.localAnchorB = anchorB;
                    def.collideConnected = d.collideConnected;
                    def.enableMotor = d.enableMotor;
                    def.motorSpeed = d.motorSpeed;
                    def.maxMotorForce = d.maxMotorEffort;
                    def.enableLimit = d.enableLimit;
                    def.lowerTranslationLimit = d.lowerLimit;
                    def.upperTranslationLimit = d.upperLimit;
                    return world.CreateJoint(def);
                }

                case PhysicsJoint2DKind.Wheel:
                {
                    var def = PhysicsWheelJointDefinition.defaultDefinition;
                    def.bodyA = bodyA;
                    def.bodyB = bodyB;
                    // The suspension axis is the local X of localAnchorA's frame
                    // (WheelJoint2D.suspension.angle, in degrees), exactly as the WheelJoint Sandbox example
                    // encodes it (it builds the same rotation from the suspension angle).
                    def.localAnchorA = new PhysicsTransform(
                        (Vector2)d.connectedAnchor,
                        new PhysicsRotate(radians(d.axisAngleDegrees))
                    );
                    def.localAnchorB = anchorB;
                    def.collideConnected = d.collideConnected;
                    def.enableMotor = d.enableMotor;
                    def.motorSpeed = d.motorSpeed;
                    def.maxMotorTorque = d.maxMotorEffort;
                    def.enableLimit = d.enableLimit;
                    def.lowerTranslationLimit = d.lowerLimit;
                    def.upperTranslationLimit = d.upperLimit;
                    def.enableSpring = d.enableSpring;
                    def.springFrequency = d.springFrequency;
                    def.springDamping = d.springDamping;
                    return world.CreateJoint(def);
                }

                case PhysicsJoint2DKind.Distance:
                case PhysicsJoint2DKind.Spring:
                {
                    // DistanceJoint2D (rigid) and SpringJoint2D (oscillating) are both the Box2D distance
                    // joint; the spring enable flag (true only for Spring) is what distinguishes them. The
                    // two anchors are held at the baked rest length: rigid keeps it exactly, the spring
                    // oscillates toward it at springFrequency/springDamping.
                    var def = PhysicsDistanceJointDefinition.defaultDefinition;
                    def.bodyA = bodyA;
                    def.bodyB = bodyB;
                    def.localAnchorA = anchorA;
                    def.localAnchorB = anchorB;
                    def.collideConnected = d.collideConnected;
                    def.distance = d.restLength;
                    def.enableSpring = d.enableSpring;
                    def.springFrequency = d.springFrequency;
                    def.springDamping = d.springDamping;
                    return world.CreateJoint(def);
                }

                case PhysicsJoint2DKind.Fixed:
                {
                    // FixedJoint2D locks the two bodies in their relative pose. The built-in single
                    // frequency/dampingRatio feeds BOTH the Box2D linear and angular stiffness; a frequency
                    // of zero is the rigid (maximum-stiffness) lock. springFrequency/springDamping carry the
                    // built-in frequency/dampingRatio (see FixedJoint2DBaker).
                    var def = PhysicsFixedJointDefinition.defaultDefinition;
                    def.bodyA = bodyA;
                    def.bodyB = bodyB;
                    def.localAnchorA = anchorA;
                    def.localAnchorB = anchorB;
                    def.collideConnected = d.collideConnected;
                    def.linearFrequency = d.springFrequency;
                    def.linearDamping = d.springDamping;
                    def.angularFrequency = d.springFrequency;
                    def.angularDamping = d.springDamping;
                    return world.CreateJoint(def);
                }

                case PhysicsJoint2DKind.Relative:
                case PhysicsJoint2DKind.Friction:
                case PhysicsJoint2DKind.Target:
                {
                    // RelativeJoint2D / FrictionJoint2D / TargetJoint2D are all the Box2D relative joint. Per
                    // the module XML, the relative joint has TWO independent controls: a SPRING that drives the
                    // bodies toward a target relative pose (springLinear/AngularFrequency), and a VELOCITY
                    // control (maxForce/maxTorque) that resists relative motion — "simulated friction such as
                    // seen in top-down games". So:
                    //   - Relative/Target want to REACH a pose/point → the spring is the position controller; it
                    //     must be enabled or the joint holds the current pose and never corrects (a relative
                    //     joint with maxForce alone is a friction joint, not an offset tracker).
                    //   - Friction wants only to DAMP relative motion → spring OFF, maxForce/maxTorque on.
                    // The maintained relative offset is folded into the anchor FRAMES: localAnchorB carries the
                    // linear offset in its position and the angular offset in its rotation, so that when the two
                    // anchor frames coincide bodyB sits at (linearOffset, angularOffset) relative to bodyA.
                    // Friction bakes a zero offset; Target uses a null connectedBody → the static world anchor
                    // as bodyA, so the offset is the target's world position relative to the world origin.
                    // The maintained offset is encoded so bodyB rests at (bodyA − linearOffset), matching the
                    // built-in v2 RelativeJoint2D sign (a built-in linearOffset of (2,0) holds bodyB 2 units in
                    // −X of bodyA — measured against the GameObject reference). localAnchorB at
                    // (anchor + linearOffset) places bodyB's anchor frame so the joint pulls bodyB to that pose.
                    var def = PhysicsRelativeJointDefinition.defaultDefinition;
                    def.bodyA = bodyA;
                    def.bodyB = bodyB;
                    def.localAnchorA = anchorA;
                    def.localAnchorB = new PhysicsTransform(
                        (Vector2)(d.anchor + d.linearOffset),
                        new PhysicsRotate(radians(d.angularOffsetDegrees))
                    );
                    def.collideConnected = d.collideConnected;

                    if (d.enableSpring)
                    {
                        // Position/orientation spring drives toward the offset (Relative/Target). The spring's
                        // force/torque are capped by springMaxForce/springMaxTorque from the built-in maxForce/
                        // maxTorque, so the convergence honours the authored effort limit. springFrequency/
                        // springDamping carry the controller stiffness (Target: its built-in frequency/damping;
                        // Relative: a stiff critically-damped spring approximating the built-in's rigid pull).
                        def.springLinearFrequency = d.springFrequency;
                        def.springLinearDamping = d.springDamping;
                        def.springMaxForce = d.maxForce;
                        // Target has no torque cap (a point constraint), so its angular spring stays off; a
                        // Relative joint with an angular offset drives it through the angular spring.
                        if (d.maxTorque > 0f)
                        {
                            def.springAngularFrequency = d.springFrequency;
                            def.springAngularDamping = d.springDamping;
                            def.springMaxTorque = d.maxTorque;
                        }
                    }
                    else
                    {
                        // Friction: no spring, pure velocity-control friction.
                        def.maxForce = d.maxForce;
                        def.maxTorque = d.maxTorque;
                    }
                    return world.CreateJoint(def);
                }

                default:
                    return default;
            }
        }
    }
}
