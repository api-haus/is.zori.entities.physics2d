using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Owns the one <see cref="PhysicsWorld"/> this ECS <c>World</c> uses. Each group update first issues
    /// exactly one <c>Simulate(dt)</c> for the bodies already live, THEN creates a Box2D body+shape for each
    /// baked entity that does not yet have one — so a body never integrates on the frame it is created (it
    /// joins the simulation on the next step) while the existing population keeps stepping on a frame that
    /// also creates new bodies. Lives in <see cref="FixedStepSimulationSystemGroup"/> so the step runs at the
    /// group's fixed timestep with the catch-up manager sub-stepping to wall-clock — deterministic, framerate
    /// independent, no bespoke sub-stepping.
    /// </summary>
    /// <remarks>
    /// Not <c>[BurstCompile]</c>: the world/body calls are managed <c>Unity.U2D.Physics</c> instance
    /// methods on the main thread. <c>simulationType = Script</c> makes this system the sole stepper —
    /// the engine never auto-steps the world. The world is created (and, if the engine's 2D physics
    /// module is reset under it on a scene-load / PlayMode-enter boundary, recreated) lazily at the top
    /// of <c>OnUpdate</c>, not once in <c>OnCreate</c>: a <see cref="PhysicsWorld"/> created at
    /// system-create time does not survive that module reset, while the ECS system and its singleton do,
    /// leaving a stale invalid handle. <c>OnDestroy</c> tears the world down; destroying it frees every
    /// body and shape, so there is no per-body teardown here at world end. Per-entity body teardown during a
    /// session (freeing a body when its entity is despawned) lives in <c>PhysicsBody2DCleanupSystem</c>, which
    /// runs <c>[UpdateBefore]</c> this system so a destroyed entity's body is freed before the next step.
    ///
    /// The creation loop is inline in <c>OnUpdate</c>: the <c>SystemAPI.Query</c>/<c>SystemAPI.Time</c>
    /// surface is source-generated against the system instance and is not available from a static helper,
    /// so the body/shape creation stays in the system method rather than a factored-out function.
    /// </remarks>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public partial struct PhysicsWorld2DSystem : ISystem
    {
        // The cross-frame cached body-template store, keyed by the baked PhysicsBody2DFormHash. An entry is built
        // lazily once a form's seenCount crosses the threshold (PhysicsWorld2DConfig.identicalBodyThreshold) and
        // then serves every subsequent identical body of that form from the prepared definition/geometry/mass —
        // skipping the per-entity definition construction + mass arithmetic. The store holds pose-free,
        // world-independent data, so it SURVIVES a world recreate (only live PhysicsBody handles are invalidated by
        // a module reset, and the store holds none). Disposed on OnDestroy. Created lazily on first use because an
        // ISystem has no OnCreate here and the map must be allocated exactly once.
        NativeHashMap<uint4, CachedBodyTemplate> m_Templates;

        public void OnDestroy(ref SystemState state)
        {
            if (SystemAPI.TryGetSingleton<PhysicsWorldSingleton2D>(out var s) && s.world.isValid)
                s.world.Destroy();
            if (m_Templates.IsCreated)
                m_Templates.Dispose();
        }

        // What the cache keeps for one form: the prepared body definition (pose/velocity overwritten per instance),
        // the prepared shape definition + geometry value, and the resolved mass write — plus the threshold
        // bookkeeping. All members are blittable values (the geometry kinds admitted to the cached arm are
        // Circle/Box/Capsule/simple-Polygon, whose geometry is a fixed-size value; Edge/decompose/multi-shape stay
        // on the per-entity path), so the whole struct lives in a NativeHashMap with no cache-owned allocation.
        struct CachedBodyTemplate
        {
            public PhysicsBodyDefinition bodyDef; // pose + velocity zeroed; overwritten per instance
            public PhysicsShapeDefinition shapeDef; // surface / filter / trigger flags
            public GeometryUnion geometry; // the prepared geometry value + kind tag
            public MassResolution mass; // the pre-decided massConfiguration write (or "leave auto")
            public PhysicsBody2DInterpolation interpolation; // the form's smoothing mode (in the form hash)
            public int seenCount; // how many bodies of this form have been initialised
            public byte isBuilt; // 0 = counting toward threshold; 1 = template built and in use
        }

        // A tagged union of the value-cacheable geometry kinds. The kind tag selects which value the template path
        // feeds to body.CreateShape. Only the kinds whose creation is exactly CreateShape(geometryValue, shapeDef)
        // are admitted — Circle/Box/Capsule (inline structs) and a simple (non-decompose) Polygon (a
        // fixed-max-vertex PolygonGeometry value). Edge (a NativeArray-backed ChainGeometry), a decompose polygon
        // (a multi-fragment CreateShapeBatch), and multi-shape bodies are NOT admitted and never reach here.
        struct GeometryUnion
        {
            public CachedGeometryKind kind;
            public CircleGeometry circle;
            public PolygonGeometry polygon; // also carries the prepared box (PolygonGeometry.CreateBox)
            public CapsuleGeometry capsule;
        }

        enum CachedGeometryKind : byte
        {
            Circle,
            Polygon,
            Capsule,
        }

        // The resolved mass write for a form: whether ApplyMass would write the body's massConfiguration at all,
        // and (if so) the exact MassConfiguration to write. Because the auto-mass Box2D derives from a form's shape
        // is identical across every body of that form, the donor's post-ApplyMass massConfiguration is the value
        // every replay body must end at — so the cache captures the donor's resolved write verbatim.
        struct MassResolution
        {
            public byte write; // 1 = write massConfiguration; 0 = leave the auto-computed mass untouched
            public PhysicsBody.MassConfiguration config;
        }

        // One entity deferred into the same-frame collapse bucket: its identity plus the per-instance pose and
        // velocity overwrite. Accumulated during the creation scan and consumed by DrainSameFrameCollapse.
        struct PendingInit
        {
            public Entity entity;
            public float2 position;
            public float rotationRadians;
            public byte hasVelocity;
            public PhysicsBody2DInitialVelocity velocity;
        }

        // The unchanged per-entity body creation: build the full body definition (with this instance's pose and
        // velocity), CreateBody, pack the entity into userData, attach the primary shape and any extra shapes, then
        // resolve and apply mass. This is the exact path the system used before the cached-template optimisation —
        // the cached arm reproduces its RESULT, so this is also the donor builder and the transparency oracle.
        static PhysicsBody CreatePerEntityBody(
            PhysicsWorld world,
            Entity entity,
            in PhysicsBody2DDefinition d,
            in PhysicsShape2D sh,
            bool hasVelocity,
            in PhysicsBody2DInitialVelocity velocity,
            DynamicBuffer<PhysicsShape2DElement> extra
        )
        {
            var bodyDef = PhysicsBodyDefinition.defaultDefinition;
            bodyDef.type = d.bodyType;
            bodyDef.gravityScale = d.gravityScale;
            bodyDef.linearDamping = d.linearDamping;
            bodyDef.angularDamping = d.angularDamping;
            bodyDef.position = (Vector2)d.initialPosition;
            bodyDef.rotation = PhysicsRotate.FromRadians(d.initialRotationRadians);
            // Initial velocity seed from the optional PhysicsBody2DInitialVelocity (absent → zero). The units match
            // 1:1: XML P:…PhysicsBodyDefinition.linearVelocity is m/s and .angularVelocity is deg/sec, the same units
            // InitialVelocity2DAuthoring carries.
            if (hasVelocity)
            {
                bodyDef.linearVelocity = (Vector2)velocity.linearVelocity;
                bodyDef.angularVelocity = velocity.angularVelocity;
            }
            // DOF constraint locks.
            bodyDef.constraints = d.constraints;
            // Continuous collision detection (Rigidbody2D.collisionDetectionMode = Continuous). The "fast" body flag
            // enables CCD against Dynamic/Kinematic bodies so a fast body does not tunnel them in one step;
            // Dynamic-vs-Static CCD is the world-level continuousAllowed (on by default), so a Discrete body still
            // does not tunnel a static wall, but a Continuous body additionally does not tunnel a fast dynamic body.
            // Discrete bakes false (the Box2D default).
            bodyDef.fastCollisionsAllowed = d.fastCollisions;
            // Locked DOTS posture: never write a managed Transform; poses move via GetBatchTransform.
            bodyDef.transformWriteMode = PhysicsBody.TransformWriteMode.Off;

            var body = world.CreateBody(bodyDef);
            // Pack the owning entity into the body's userData so a spatial query can map a hit shape back to its
            // entity (shape.body.userData.int64Value → Entity). Set once per body; the body's lone shape resolves
            // through .body.
            body.userData = PhysicsQueries2D.PackEntity(entity);

            CreateShapeForBody(body, sh);

            // Multi-shape bodies (CompositeCollider2D merged paths, CustomCollider2D shape groups) carry their extra
            // shapes in an optional DynamicBuffer<PhysicsShape2DElement> alongside the primary PhysicsShape2D. A
            // normal one-shape body has no buffer, so this is a no-op for it (the single-shape archetype is
            // unchanged). Each extra shape attaches to the SAME body via the same CreateShapeForBody path, so it gets
            // identical surface/filter/trigger/offset handling. Box2D accumulates the auto-mass from all attached
            // shapes, so ApplyMass (run once below, after every shape exists) derives the correct multi-shape mass.
            if (extra.IsCreated)
                for (var i = 0; i < extra.Length; i++)
                    CreateShapeForBody(body, extra[i].shape);

            ApplyMass(body, d);
            return body;
        }

        // Add the PhysicsBody2D handle witness, the cleanup witness, and (for an interpolated/extrapolated body) the
        // render-rate smoothing seed to a just-created body's entity, via the deferred ECB. Identical across every
        // creation path (per-entity, template, in-frame collapse) — the body handle is the only varying input.
        static void AddPhysicalBodyComponents(
            ref EntityCommandBuffer ecb,
            Entity entity,
            in PhysicsBody2DDefinition d,
            PhysicsBody body
        )
        {
            ecb.AddComponent(entity, new PhysicsBody2D { body = body });
            // The retained handle witness that survives the entity's destruction, so PhysicsBody2DCleanupSystem can
            // free this body when the entity is despawned.
            ecb.AddComponent(entity, new PhysicsBody2DCleanup { body = body });

            // Render-rate smoothing state for an interpolated/extrapolated body (Rigidbody2D.interpolation). Added
            // only when the mode is not None — a non-interpolated body never carries it and keeps its fixed-rate
            // LocalToWorld. Seeded with prev == cur == the authored pose so the first render-rate pass before any
            // step is a no-op (hasPrev = 0 until the write-back captures a second pose).
            if (d.interpolation != PhysicsBody2DInterpolation.None)
            {
                sincos(d.initialRotationRadians, out var s0, out var c0);
                var cosSin0 = float2(c0, s0);
                ecb.AddComponent(
                    entity,
                    new PhysicsBody2DSmoothing
                    {
                        prevPos = d.initialPosition,
                        prevCosSin = cosSin0,
                        curPos = d.initialPosition,
                        curCosSin = cosSin0,
                        linearVel = Unity.Mathematics.float2.zero,
                        angularVelRad = 0f,
                        mode = (byte)d.interpolation,
                        hasPrev = 0,
                    }
                );
            }
        }

        // The pose-free body definition cached for a form: the per-entity body definition with position, rotation,
        // and velocity zeroed (every instance overwrites them). transformWriteMode is locked Off, as on the
        // per-entity path. The cached value is overwritten per instance with the real pose/velocity at creation.
        static PhysicsBodyDefinition BuildTemplateBodyDef(in PhysicsBody2DDefinition d)
        {
            var bodyDef = PhysicsBodyDefinition.defaultDefinition;
            bodyDef.type = d.bodyType;
            bodyDef.gravityScale = d.gravityScale;
            bodyDef.linearDamping = d.linearDamping;
            bodyDef.angularDamping = d.angularDamping;
            bodyDef.constraints = d.constraints;
            bodyDef.fastCollisionsAllowed = d.fastCollisions;
            bodyDef.transformWriteMode = PhysicsBody.TransformWriteMode.Off;
            // position / rotation / linearVelocity / angularVelocity stay at the default (zero) — overwritten per
            // instance from the entity's own pose + optional velocity seed.
            return bodyDef;
        }

        // Whether a shape's geometry is value-cacheable (Circle/Box/Capsule/simple-Polygon), and if so the prepared
        // geometry value. Edge (a NativeArray-backed ChainGeometry) and a decompose polygon (a multi-fragment
        // CreateShapeBatch) are NOT value-cacheable and return false — those forms stay on the per-entity path.
        static bool TryGetCacheableGeometry(in PhysicsShape2D sh, out GeometryUnion geometry)
        {
            geometry = default;
            switch (sh.kind)
            {
                case PhysicsShape2DKind.Circle:
                    geometry.kind = CachedGeometryKind.Circle;
                    geometry.circle = BuildCircleGeometry(in sh);
                    return true;
                case PhysicsShape2DKind.Box:
                    geometry.kind = CachedGeometryKind.Polygon;
                    geometry.polygon = BuildBoxGeometry(in sh);
                    return true;
                case PhysicsShape2DKind.Capsule:
                    geometry.kind = CachedGeometryKind.Capsule;
                    geometry.capsule = BuildCapsuleGeometry(in sh);
                    return true;
                case PhysicsShape2DKind.Polygon:
                    if (sh.polygonDecompose)
                        return false;

                    // Marshal the blob into the inline-storage PolygonGeometry once; the value is self-contained.
                    {
                        ref var blob = ref sh.vertices.Value.points;
                        var span = new NativeArray<Vector2>(blob.Length, Allocator.Temp);
                        for (var i = 0; i < blob.Length; i++)
                            span[i] = (Vector2)blob[i];
                        geometry.kind = CachedGeometryKind.Polygon;
                        geometry.polygon = BuildPolygonGeometry(in sh, span);
                        span.Dispose();
                    }
                    return true;
                default: // Edge — chain geometry is NativeArray-backed, not cacheable by value.
                    return false;
            }
        }

        // Attach the cached shape to a body from the template's prepared geometry + shapeDef. The kind tag selects
        // the geometry value; the call is the identical CreateShape the per-entity path issues, with cached inputs.
        static void CreateShapeFromTemplate(PhysicsBody body, in CachedBodyTemplate t)
        {
            switch (t.geometry.kind)
            {
                case CachedGeometryKind.Circle:
                    body.CreateShape(t.geometry.circle, t.shapeDef);
                    break;
                case CachedGeometryKind.Polygon:
                    body.CreateShape(t.geometry.polygon, t.shapeDef);
                    break;
                case CachedGeometryKind.Capsule:
                    body.CreateShape(t.geometry.capsule, t.shapeDef);
                    break;
            }
        }

        // Create one body from a cached template: copy the cached body def, overwrite the per-instance pose +
        // velocity, CreateBody, pack userData, attach the cached shape, apply the cached mass resolution. The result
        // is bit-identical to the per-entity path (same definition + same shape → same Box2D body + same mass).
        static PhysicsBody CreateBodyFromTemplate(
            PhysicsWorld world,
            Entity entity,
            in CachedBodyTemplate t,
            float2 position,
            float rotationRadians,
            bool hasVelocity,
            in PhysicsBody2DInitialVelocity velocity
        )
        {
            var bodyDef = t.bodyDef;
            bodyDef.position = (Vector2)position;
            bodyDef.rotation = PhysicsRotate.FromRadians(rotationRadians);
            if (hasVelocity)
            {
                bodyDef.linearVelocity = (Vector2)velocity.linearVelocity;
                bodyDef.angularVelocity = velocity.angularVelocity;
            }

            var body = world.CreateBody(bodyDef);
            body.userData = PhysicsQueries2D.PackEntity(entity);
            CreateShapeFromTemplate(body, in t);
            ApplyResolvedMass(body, in t.mass);
            return body;
        }

        // Drain the same-frame collapse buckets. For each form with K >= 2 members that arrived this frame, issue one
        // CreateBodyBatch from the cached body def, attach the K cached shapes, apply the K cached masses, and one
        // SetBatchTransform writing the K individual poses — the inner routine factored from the removed batch
        // creation system. A lone member (K == 1) takes the plain template path (a 1-body batch buys nothing). Every
        // created body gets the same ECS components through the shared ECB. The K bodies are bit-identical to K
        // per-entity bodies; only the native body-CREATION call count is collapsed (the K CreateShape calls remain).
        void DrainSameFrameCollapse(
            PhysicsWorld world,
            ref EntityCommandBuffer ecb,
            NativeParallelMultiHashMap<uint4, PendingInit> collapse
        )
        {
            var keys = collapse.GetKeyArray(Allocator.Temp);
            var seen = new NativeHashSet<uint4>(keys.Length, Allocator.Temp);
            var members = new NativeList<PendingInit>(16, Allocator.Temp);

            for (var k = 0; k < keys.Length; k++)
            {
                var hash = keys[k];
                if (!seen.Add(hash))
                    continue; // GetKeyArray repeats a key once per member; handle each key's whole run once.

                members.Clear();
                foreach (var m in collapse.GetValuesForKey(hash))
                    members.Add(m);

                var t = m_Templates[hash];

                if (members.Length == 1)
                {
                    var p = members[0];
                    var body = CreateBodyFromTemplate(
                        world,
                        p.entity,
                        in t,
                        p.position,
                        p.rotationRadians,
                        p.hasVelocity != 0,
                        in p.velocity
                    );
                    AddPhysicalBodyComponentsFromTemplate(ref ecb, in t, p, body);
                    continue;
                }

                CreateBodyBatchInto(world, ref ecb, in t, members);
            }

            members.Dispose();
            seen.Dispose();
            keys.Dispose();
        }

        // One CreateBodyBatch for the K same-frame members of an isBuilt form: one native body-creation call for all
        // K bodies from the shared cached def, then the K cached-shape attachments, the K cached-mass applies, one
        // SetBatchTransform writing each member's individual pose, and the per-body userData + ECS components. This
        // is the inner routine of the removed PhysicsBody2DBatchCreationSystem, now fed by the cached template and
        // attaching to the already-instantiated entities (not fresh em.CreateEntity ones).
        void CreateBodyBatchInto(
            PhysicsWorld world,
            ref EntityCommandBuffer ecb,
            in CachedBodyTemplate t,
            NativeList<PendingInit> members
        )
        {
            var count = members.Length;
            var bodies = world.CreateBodyBatch(t.bodyDef, count, Allocator.Persistent);
            var transforms = new NativeArray<PhysicsBody.BatchTransform>(count, Allocator.Temp);

            for (var i = 0; i < count; i++)
            {
                var p = members[i];
                // Local copy because PhysicsBody is a 64-bit-ID handle struct: a NativeArray indexer returns a copy,
                // so setting userData on `body` still routes to the same native body the handle names.
                var body = bodies[i];
                body.userData = PhysicsQueries2D.PackEntity(p.entity);
                CreateShapeFromTemplate(body, in t);
                ApplyResolvedMass(body, in t.mass);

                // Per-instance velocity: CreateBodyBatch fans out the single cached def (zero velocity), so a member
                // carrying an initial-velocity seed has it set on its body after creation, matching the per-entity
                // path where the def carried it. (A pure-pose spray sets nothing here.)
                if (p.hasVelocity != 0)
                {
                    body.linearVelocity = (Vector2)p.velocity.linearVelocity;
                    body.angularVelocity = p.velocity.angularVelocity;
                }

                var rot = PhysicsRotate.FromRadians(p.rotationRadians);
                transforms[i] = new PhysicsBody.BatchTransform(body)
                {
                    position = (Vector2)p.position,
                    rotation = rot,
                };
                AddPhysicalBodyComponentsFromTemplate(ref ecb, in t, p, body);
            }

            PhysicsBody.SetBatchTransform(transforms.AsReadOnlySpan());

            transforms.Dispose();
            bodies.Dispose();
        }

        // The PhysicsBody2D / cleanup / smoothing component adds for a template-created body. Mirrors
        // AddPhysicalBodyComponents, but reads the interpolation mode from the cached template (the form's
        // interpolation is part of the form hash, so every body of the form shares the donor's mode) and seeds the
        // smoothing from the PendingInit pose. Identical to the per-entity smoothing seed for the same pose + mode.
        static void AddPhysicalBodyComponentsFromTemplate(
            ref EntityCommandBuffer ecb,
            in CachedBodyTemplate t,
            in PendingInit p,
            PhysicsBody body
        )
        {
            ecb.AddComponent(p.entity, new PhysicsBody2D { body = body });
            ecb.AddComponent(p.entity, new PhysicsBody2DCleanup { body = body });

            if (t.interpolation != PhysicsBody2DInterpolation.None)
            {
                sincos(p.rotationRadians, out var s0, out var c0);
                var cosSin0 = float2(c0, s0);
                ecb.AddComponent(
                    p.entity,
                    new PhysicsBody2DSmoothing
                    {
                        prevPos = p.position,
                        prevCosSin = cosSin0,
                        curPos = p.position,
                        curCosSin = cosSin0,
                        linearVel = Unity.Mathematics.float2.zero,
                        angularVelRad = 0f,
                        mode = (byte)t.interpolation,
                        hasPrev = 0,
                    }
                );
            }
        }

        // Create the one PhysicsWorld this ECS World owns, and publish its handle as the singleton.
        // A new PhysicsWorld is always invalid until created; this is also the recreation path when the
        // engine's 2D physics module is reset (PlayMode enter / scene load) under it — see remarks.
        //
        // Backward-compatible config fallback: with no PhysicsWorld2DConfig (no PhysicsStep2DAuthoring in the
        // scene) the world is built straight from PhysicsWorldDefinition.defaultDefinition — the exact world
        // shipped before this config surface existed. A config (baked from PhysicsStep2DAuthoring) overrides
        // ONLY the fields it carries; defaultDefinition supplies the rest. simulationType is overwritten to
        // Script in either case: the package owns stepping via the explicit Simulate(dt) below, and Script is
        // the mode where the engine does not also auto-step (defaultDefinition.simulateType is FixedUpdate, so
        // copying it unguarded would double-step). simulationType is therefore not a config field — see
        // PhysicsWorld2DConfig.
        static PhysicsWorld CreateWorld(in PhysicsWorld2DConfig? cfg)
        {
            var def = PhysicsWorldDefinition.defaultDefinition;
            def.drawOptions = PhysicsWorld.DrawOptions.Off;
            if (cfg.HasValue)
            {
                var c = cfg.Value;
                def.gravity = (Vector2)c.gravity;
                def.simulationSubSteps = c.simulationSubSteps;
                def.simulationWorkers = c.simulationWorkers;
                def.continuousAllowed = c.continuousAllowed;
                def.sleepingAllowed = c.sleepingAllowed;
                def.bounceThreshold = c.bounceThreshold;
                def.contactHitEventThreshold = c.contactHitEventThreshold;
                def.contactFrequency = c.contactFrequency;
                def.contactDamping = c.contactDamping;
                def.contactSpeed = c.contactSpeed;
                def.contactRecycleDistance = c.contactRecycleDistance;
                def.maximumLinearSpeed = c.maximumLinearSpeed;
            }
            var world = PhysicsWorld.Create(def);
            world.simulationType = PhysicsWorld.SimulationType.Script;
            return world;
        }

        // The contacts row to bake into a categorized shape's Box2D ContactFilter so its query-visibility does
        // not depend on its collision-matrix row. Box2D's query-vs-shape match is bidirectional and ANDs
        // (shape.categories & query.hitCategories) with (shape.contacts & query.categories); the query surface
        // sets query.categories = All, so a shape with contacts = 0 fails the second clause and is invisible to
        // EVERY query — even the documented "mask 0 = hit everything" path. A shape on a dedicated layer whose 2D
        // collision-matrix row is fully unchecked (GetLayerCollisionMask(layer) = 0, e.g. a non-blocking rope
        // anchor / detection marker) bakes contactBits = 0 and so was unqueryable. Substituting the shape's own
        // categoryBits when the authored row is empty keeps it colliding with nothing on every OTHER category
        // (the real collision surfaces) while restoring (contacts & All) != 0 so a category query finds it. A
        // non-empty authored row passes through unchanged, so a shape that already collides is untouched.
        static ulong QueryVisibleContacts(in PhysicsShape2D sh)
        {
            return sh.contactBits != 0ul ? sh.contactBits : sh.categoryBits;
        }

        // Create the Box2D shape for one baked geometry on a freshly-created body. The Collider2D.offset is
        // folded into the geometry here because none of the CreateShape overloads takes an offset transform:
        // Box/Polygon fold it via a PhysicsTransform, Circle via the geometry center, Capsule by translating
        // both end centers, Edge by translating each chain vertex. Managed Unity.U2D.Physics calls, main
        // thread — not Burst.
        static void CreateShapeForBody(PhysicsBody body, in PhysicsShape2D sh)
        {
            var shapeDef = BuildShapeDef(in sh);

            var offset = (Vector2)sh.offset;

            switch (sh.kind)
            {
                case PhysicsShape2DKind.Circle:
                    body.CreateShape(BuildCircleGeometry(in sh), shapeDef);
                    break;

                case PhysicsShape2DKind.Box:
                    body.CreateShape(BuildBoxGeometry(in sh), shapeDef);
                    break;

                case PhysicsShape2DKind.Capsule:
                    body.CreateShape(BuildCapsuleGeometry(in sh), shapeDef);
                    break;

                case PhysicsShape2DKind.Polygon:
                {
                    ref var blob = ref sh.vertices.Value.points;
                    var span = new NativeArray<Vector2>(blob.Length, Allocator.Temp);
                    for (var i = 0; i < blob.Length; i++)
                        span[i] = (Vector2)blob[i];
                    if (sh.polygonDecompose)
                    {
                        // A closed (possibly concave, possibly over-MaxPolygonVertices) outline — a composite
                        // Polygons path or a concave/large custom polygon. PolygonGeometry.CreatePolygons
                        // decomposes it into convex fragments (each within MaxPolygonVertices) and validates
                        // them; the fragments are attached in one CreateShapeBatch call. The returned array
                        // must be disposed. An empty result (degenerate outline) attaches nothing.
                        var fragments = PolygonGeometry.CreatePolygons(
                            vertices: span.AsReadOnlySpan(),
                            transform: new PhysicsTransform(offset),
                            allocator: Allocator.Temp
                        );
                        if (fragments.Length > 0)
                            body.CreateShapeBatch(
                                fragments.AsReadOnlySpan(),
                                shapeDef,
                                Allocator.Temp
                            );
                        if (fragments.IsCreated)
                            fragments.Dispose();
                    }
                    else
                    {
                        body.CreateShape(BuildPolygonGeometry(in sh, span), shapeDef);
                    }
                    span.Dispose();
                    break;
                }

                case PhysicsShape2DKind.Edge:
                {
                    ref var blob = ref sh.vertices.Value.points;
                    var verts = new NativeArray<Vector2>(blob.Length, Allocator.Temp);
                    for (var i = 0; i < blob.Length; i++)
                        verts[i] = (Vector2)(blob[i] + sh.offset);
                    // A chain carries its OWN definition (no PhysicsShapeDefinition overload): its
                    // surfaceMaterial, contactFilter, triggerEvents, and isLoop ride the chain def, not the
                    // shape def. Propagate the baked surface/filter/trigger so a composite-Outlines (or edge)
                    // surface has the right friction/bounciness/layer the same way a solid shape does. A chain
                    // is non-solid, so it carries no density and no contactEvents flag (contacts on a chain are
                    // implicit); the surface material + filter + trigger are the propagatable knobs.
                    var chainDef = PhysicsChainDefinition.defaultDefinition;
                    chainDef.isLoop = sh.edgeIsLoop;
                    var chainSurface = chainDef.surfaceMaterial;
                    chainSurface.friction = sh.friction;
                    chainSurface.bounciness = sh.bounciness;
                    chainSurface.frictionMixing = MapMixing(sh.frictionMixing);
                    chainSurface.bouncinessMixing = MapMixing(sh.bouncinessMixing);
                    chainDef.surfaceMaterial = chainSurface;
                    if (sh.categoryBits != 0ul)
                    {
                        var chainFilter = PhysicsShape.ContactFilter.defaultFilter;
                        chainFilter.categories = new PhysicsMask { bitMask = sh.categoryBits };
                        chainFilter.contacts = new PhysicsMask { bitMask = QueryVisibleContacts(sh) };
                        chainDef.contactFilter = chainFilter;
                    }
                    chainDef.triggerEvents = true;
                    body.CreateChain(new ChainGeometry(verts), chainDef);
                    verts.Dispose();
                    break;
                }
            }
        }

        // Build the shared PhysicsShapeDefinition (surface material, density, contact filter, trigger + event
        // flags) for a baked shape. Factored out of CreateShapeForBody so the per-entity path and the cached
        // template build the identical definition — the template caches this value and replays it.
        static PhysicsShapeDefinition BuildShapeDef(in PhysicsShape2D sh)
        {
            var shapeDef = PhysicsShapeDefinition.defaultDefinition;

            // Surface material: friction/bounciness from the baked PhysicsMaterial2D values (XML
            // P:…PhysicsShape.SurfaceMaterial.friction/.bounciness) plus their combine (mixing) modes (XML
            // P:…PhysicsShape.SurfaceMaterial.frictionMixing/.bouncinessMixing). The SurfaceMaterial struct is
            // built here (not stored on the blittable component); the other surface knobs keep their defaults.
            var surface = shapeDef.surfaceMaterial;
            surface.friction = sh.friction;
            surface.bounciness = sh.bounciness;
            surface.frictionMixing = MapMixing(sh.frictionMixing);
            surface.bouncinessMixing = MapMixing(sh.bouncinessMixing);
            shapeDef.surfaceMaterial = surface;

            // Density drives the auto-mass-from-shapes path (Collider2D.density). A value <= 0 means the
            // collider author never set it; leave Box2D's default density rather than zero out the mass.
            if (sh.density > 0f)
                shapeDef.density = sh.density;

            // Contact filter from the baked layer-matrix bits (XML F:…PhysicsShapeDefinition.contactFilter).
            // categoryBits = 1<<layer, contactBits = the layer-matrix row. A zero category means the shape was
            // authored without a GameObject layer (direct/custom authoring) — keep the everything-default so
            // such a shape still collides with everything (the dual-surface default). Starting from
            // defaultFilter preserves the default groupIndex (the Box2D group override is out of scope: it has
            // no GameObject 2D-physics analogue to bake from).
            if (sh.categoryBits != 0ul)
            {
                var filter = PhysicsShape.ContactFilter.defaultFilter;
                filter.categories = new PhysicsMask { bitMask = sh.categoryBits };
                filter.contacts = new PhysicsMask { bitMask = QueryVisibleContacts(sh) };
                shapeDef.contactFilter = filter;
            }

            // Trigger (sensor) + event reporting. Collider2D.isTrigger → PhysicsShapeDefinition.isTrigger: a
            // sensor overlaps without a collision response (XML P:…PhysicsShapeDefinition.isTrigger). Contact and
            // trigger events are enabled on EVERY shape so every package pair is event-eligible, matching the
            // GameObject's always-on Enter/Stay/Exit (no per-collider opt-in): a contact event fires if EITHER
            // shape has contactEvents (:11609), a trigger event fires only if BOTH have triggerEvents (:11718),
            // so unconditional-true on both satisfies both rules. startStaticContacts forces a Static body's
            // shape to create contacts at add time (implicitly already true for Trigger/Dynamic/Kinematic,
            // :11699) so a dynamic body landing on a static floor fires a contact-begin. The flags are set on the
            // definition, not post-creation: the XML warns a runtime flag change is expensive and may lose
            // begin/end events (:10061, :11615).
            shapeDef.isTrigger = sh.isTrigger;
            shapeDef.contactEvents = true;
            shapeDef.triggerEvents = true;
            shapeDef.startStaticContacts = true;

            return shapeDef;
        }

        // The folded-offset circle geometry — Collider2D.offset rides CircleGeometry.center (no CreateShape
        // overload takes an offset transform). Pure function of the shape, so it is also the cached value.
        static CircleGeometry BuildCircleGeometry(in PhysicsShape2D sh) =>
            new CircleGeometry { radius = sh.radius, center = (Vector2)sh.offset };

        // The box-as-polygon geometry. The box's free z-rotation (radians) rides the creation PhysicsTransform
        // alongside the folded offset. A 0 angle is PhysicsRotate.FromRadians(0) = identity, the prior offset-only
        // transform, so an un-rotated box is unchanged. Pure function of the shape, so it is the cached value.
        static PolygonGeometry BuildBoxGeometry(in PhysicsShape2D sh) =>
            PolygonGeometry.CreateBox(
                size: (Vector2)sh.size,
                radius: sh.radius,
                transform: new PhysicsTransform(
                    (Vector2)sh.offset,
                    PhysicsRotate.FromRadians(sh.boxAngleRadians)
                ),
                inscribe: false
            );

        // The capsule geometry — the offset is folded into both end centers (no offset transform overload). Pure
        // function of the shape, so it is the cached value.
        static CapsuleGeometry BuildCapsuleGeometry(in PhysicsShape2D sh) =>
            CapsuleGeometry.Create(
                center1: (Vector2)(sh.capsuleCenter1 + sh.offset),
                center2: (Vector2)(sh.capsuleCenter2 + sh.offset),
                radius: sh.radius
            );

        // The simple (convex, non-decompose) polygon geometry from the baked vertex blob, with the offset folded
        // via a PhysicsTransform. The PolygonGeometry has a fixed maximum vertex count, so the returned value is
        // self-contained (no NativeArray) and is the cached value. The caller owns the span; CreateShape copies the
        // vertices into the geometry's inline storage.
        static PolygonGeometry BuildPolygonGeometry(
            in PhysicsShape2D sh,
            NativeArray<Vector2> span
        ) =>
            PolygonGeometry.Create(
                vertices: span.AsReadOnlySpan(),
                radius: sh.radius,
                transform: new PhysicsTransform((Vector2)sh.offset)
            );

        // Map the package-local PhysicsSurfaceMixing2D onto the engine SurfaceMaterial.MixingMode (XML
        // T:…PhysicsShape.SurfaceMaterial.MixingMode). The two share the same five members in the same order, so
        // this is a 1:1 by-name conversion; the package enum is carried package-local so neither the Runtime nor
        // the Authoring assembly references Unity.U2D.Physics, and the engine enum is named only here.
        static PhysicsShape.SurfaceMaterial.MixingMode MapMixing(PhysicsSurfaceMixing2D mixing) =>
            mixing switch
            {
                PhysicsSurfaceMixing2D.Maximum => PhysicsShape.SurfaceMaterial.MixingMode.Maximum,
                PhysicsSurfaceMixing2D.Mean => PhysicsShape.SurfaceMaterial.MixingMode.Mean,
                PhysicsSurfaceMixing2D.Minimum => PhysicsShape.SurfaceMaterial.MixingMode.Minimum,
                PhysicsSurfaceMixing2D.Multiply => PhysicsShape.SurfaceMaterial.MixingMode.Multiply,
                _ => PhysicsShape.SurfaceMaterial.MixingMode.Average,
            };

        // The built-in Rigidbody2D default mass, used to recognise "the author did not set a custom mass".
        const float DefaultRigidbody2DMass = 1f;

        // Resolve a dynamic body's mass after its shapes exist, mirroring Rigidbody2D's mass / useAutoMass.
        // After CreateShape, Box2D has already computed a density-derived MassConfiguration from the attached
        // shapes (XML: the MassConfiguration is recomputed whenever a shape is added). The guiding rule is to
        // touch massConfiguration as LITTLE as possible: a measured side effect of explicitly assigning it is
        // that the v3 sub-stepping solver's free-fall integration shifts relative to the v2 GameObject
        // reference (the cross-solver convention offset roughly 2.6×es, from ~1.7e-3 to ~4.4e-3 m/step on a
        // free-falling circle), independent of the mass value. So every body that does NOT need a custom mass
        // is left on its auto-computed mass — that keeps the falling-body and collider fixtures on the tight
        // free-fall band they are calibrated to. Cases:
        //   - useAutoMass true: keep the density-derived mass (Rigidbody2D.useAutoMass) — except a chain
        //     (EdgeCollider2D) is a non-solid surface contributing NO mass, so a dynamic chain-only body lands
        //     at mass 0 and never integrates; floor it to a unit MassConfiguration (a built-in Rigidbody2D
        //     defaults to mass 1 regardless of collider). This is the Phase-1A chain-only mass floor, now
        //     folded into the mass feature.
        //   - useAutoMass false, default mass on a solid body that already has positive auto mass: a no-op.
        //     The body falls correctly on either mass, so the assignment is skipped to avoid the solver
        //     perturbation — this is what keeps the default-mass fixtures (Circle, all 1A shapes) green.
        //   - useAutoMass false, custom mass (or a body with no auto mass, e.g. a chain): apply the explicit
        //     Rigidbody2D.mass via massConfiguration, scaling the rotational inertia by mass/oldMass so spin
        //     stays consistent. Scenes that set a custom mass carry a band wide enough for the perturbed slope.
        // Non-dynamic bodies (Static/Kinematic) ignore mass entirely. Implemented as a pure RESOLVE (read the
        // body's shape-derived mass, decide whether and what to write) followed by an APPLY, so the cached-template
        // path can capture the resolution from the donor body and replay the identical write on every later body of
        // the form — making a template-path body bit-identical to a per-entity-path one.
        static void ApplyMass(PhysicsBody body, in PhysicsBody2DDefinition d) =>
            ApplyResolvedMass(body, ResolveMass(body, in d));

        // Decide the mass write for a body whose shapes already exist. Pure function of the form (the body
        // definition) and the body's shape-derived auto mass — which is identical for every body of one form — so
        // the returned MassResolution is the same for all bodies of the form and can be cached from the donor.
        static MassResolution ResolveMass(PhysicsBody body, in PhysicsBody2DDefinition d)
        {
            if (d.bodyType != PhysicsBody.BodyType.Dynamic)
                return default; // write = 0; non-dynamic bodies never touch massConfiguration.

            if (d.useAutoMass)
            {
                if (body.mass <= 0f)
                {
                    var massCfg = body.massConfiguration;
                    massCfg.mass = 1f;
                    if (massCfg.rotationalInertia <= 0f)
                        massCfg.rotationalInertia = 1f;
                    return WithOverride(body, massCfg, in d, forceWrite: true);
                }
                return WithOverride(body, body.massConfiguration, in d, forceWrite: false);
            }

            // Explicit mass requested. If the author left the default mass and the shape already produced a
            // positive auto mass, do not perturb the solver — the default-mass body falls correctly as is.
            var isDefaultMass = Mathf.Abs(d.mass - DefaultRigidbody2DMass) < 1e-6f;
            if (isDefaultMass && body.mass > 0f)
                return WithOverride(body, body.massConfiguration, in d, forceWrite: false);

            var cfg = body.massConfiguration;
            var oldMass = cfg.mass;
            var newMass = d.mass > 0f ? d.mass : 1f;
            // Scale the auto-computed rotational inertia to the new mass so spin behaviour stays consistent;
            // when the shape contributed no inertia (e.g. a chain) seed a unit value so rotation is defined.
            if (oldMass > 0f && cfg.rotationalInertia > 0f)
                cfg.rotationalInertia *= newMass / oldMass;
            else if (cfg.rotationalInertia <= 0f)
                cfg.rotationalInertia = newMass;
            cfg.mass = newMass;
            return WithOverride(body, cfg, in d, forceWrite: true);
        }

        // Fold the custom mass-distribution override (PhysicsBody2DAuthoring.OverrideMassDistribution) into a mass
        // resolution: set the body's center of mass and, when an explicit inertia was authored (> 0), its rotational
        // inertia, on top of whatever mass the resolution above produced (the override is orthogonal to the mass
        // magnitude — a body can override COM while keeping its auto mass). When the override is off the resolution
        // is the input cfg with the input write flag, so a body that does not opt in keeps the solver convention the
        // gates are calibrated to (touching massConfiguration shifts the v3 free-fall integration — see ApplyMass
        // remarks). When on, the write flag becomes true because the override always mutates massConfiguration.
        static MassResolution WithOverride(
            PhysicsBody body,
            PhysicsBody.MassConfiguration cfg,
            in PhysicsBody2DDefinition d,
            bool forceWrite
        )
        {
            if (!d.overrideMassDistribution)
                return new MassResolution { write = (byte)(forceWrite ? 1 : 0), config = cfg };

            cfg.center = (Vector2)d.centerOfMass;
            if (d.rotationalInertia > 0f)
                cfg.rotationalInertia = d.rotationalInertia;
            return new MassResolution { write = 1, config = cfg };
        }

        // Apply a resolved mass to a body: write the massConfiguration only when the resolution decided to. A
        // no-op resolution leaves the body's auto-computed mass untouched.
        static void ApplyResolvedMass(PhysicsBody body, in MassResolution res)
        {
            if (res.write != 0)
                body.massConfiguration = res.config;
        }

        public void OnUpdate(ref SystemState state)
        {
            // Ensure a valid world exists, (re)creating it and (re)publishing the singleton if it is
            // missing or has been invalidated by a physics-module reset. This is the load-bearing
            // robustness over a once-only OnCreate: a PhysicsWorld created at system-create time does not
            // survive the scene-load / PlayMode-enter boundary that resets the 2D physics module, so the
            // world is owned here lazily rather than at OnCreate.
            // Resolve the optional simulation config baked from a PhysicsStep2DAuthoring. Absent → the
            // backward-compatible defaultDefinition path; present → it overrides the world params at creation.
            // Resolved fresh here (not cached) so a world recreated after a physics-module reset re-reads the
            // current config. TryGetSingleton throws on more than one config, surfacing the single-world
            // "one PhysicsStep2D per world" rule loudly rather than silently picking one.
            PhysicsWorld2DConfig? config = SystemAPI.TryGetSingleton<PhysicsWorld2DConfig>(
                out var cfg
            )
                ? cfg
                : null;

            if (!SystemAPI.TryGetSingleton<PhysicsWorldSingleton2D>(out var singleton))
            {
                state.EntityManager.CreateSingleton(
                    new PhysicsWorldSingleton2D { world = CreateWorld(config) }
                );
                // The per-frame event streams ride the singleton entity: two DynamicBuffers cleared and refilled
                // each step from the post-Simulate event spans (below). Added once at creation; they survive a
                // world recreate (only the PhysicsWorld handle is reset on a module reset, not the entity).
                var singletonEntity = SystemAPI.GetSingletonEntity<PhysicsWorldSingleton2D>();
                state.EntityManager.AddBuffer<PhysicsContactEvent2D>(singletonEntity);
                state.EntityManager.AddBuffer<PhysicsTriggerEvent2D>(singletonEntity);
                // The per-frame joint-break event stream rides the same singleton entity, like the contact and
                // trigger buffers: cleared and refilled each step from the post-Simulate jointThresholdEvents span.
                state.EntityManager.AddBuffer<PhysicsJointBreakEvent2D>(singletonEntity);
                // The most-recent fixed-step time, written after each Simulate and read at render rate by
                // PhysicsBody2DSmoothingSystem to compute how far render time is ahead of the last physics step.
                state.EntityManager.AddComponent<PhysicsFixedStepTime2D>(singletonEntity);
                singleton = SystemAPI.GetSingleton<PhysicsWorldSingleton2D>();
            }
            else if (!singleton.world.isValid)
            {
                singleton.world = CreateWorld(config);
                SystemAPI.SetSingleton(singleton);
            }

            var world = singleton.world;
            if (!world.isValid)
                return;

            // Create a Box2D body+shape for each baked entity lacking a live PhysicsBody2D handle. Three paths,
            // chosen per entity by the cached-template optimisation (PhysicsWorld2DConfig.cacheIdenticalBodies):
            //   - the unchanged per-entity path (CreatePerEntityBody) — for a heterogeneous bake, a warm-up body
            //     below the threshold, a no-form-hash body, a multi-shape body, or a kind not value-cacheable;
            //   - the cheap cached-template path — for an isBuilt form's body, replaying the prepared
            //     definition/geometry/mass instead of re-constructing them;
            //   - the same-frame in-frame collapse — for K >= 2 isBuilt-form bodies that land in ONE frame, one
            //     CreateBodyBatch instead of K CreateBody calls (the inner routine factored from the removed batch
            //     creation system). A 1/frame cross-frame spray gets K = 1 and the plain template path.
            // The PhysicsBody2D / PhysicsBody2DCleanup / PhysicsBody2DSmoothing component adds are identical across
            // all three paths (AddPhysicalBodyComponents) and are deferred through one ECB played back immediately,
            // so the new bodies are live for the next step. Whatever path created a body, the RESULT is identical —
            // a cached-template body is bit-identical to a per-entity one (same definition + same shape → same Box2D
            // body), which is what keeps the optimisation transparent.
            var cacheEnabled = config.HasValue
                ? config.Value.cacheIdenticalBodies
                : PhysicsWorld2DConfig.Default.cacheIdenticalBodies;
            var threshold = max(
                1,
                config.HasValue
                    ? config.Value.identicalBodyThreshold
                    : PhysicsWorld2DConfig.Default.identicalBodyThreshold
            );
            if (cacheEnabled && !m_Templates.IsCreated)
                m_Templates = new NativeHashMap<uint4, CachedBodyTemplate>(
                    16,
                    Allocator.Persistent
                );

            // The per-frame event buffers on the singleton entity. Cleared every frame and refilled below from the
            // post-Simulate spans, so they always reflect exactly this step's events.
            var contactEvents = SystemAPI.GetSingletonBuffer<PhysicsContactEvent2D>();
            var triggerEvents = SystemAPI.GetSingletonBuffer<PhysicsTriggerEvent2D>();
            var jointBreakEvents = SystemAPI.GetSingletonBuffer<PhysicsJointBreakEvent2D>();
            contactEvents.Clear();
            triggerEvents.Clear();
            jointBreakEvents.Clear();

            // STEP, THEN CREATE — in this order, every frame. The step integrates the bodies that are ALREADY live
            // at the top of this update; the bodies created later in this same update (below) are created AFTER the
            // step, so they first integrate on the NEXT update. This preserves the spawn-frame-no-step rule per BODY
            // (a body never advances on the frame it is created — it sits at its authored pose until the next step),
            // while letting the world keep stepping the existing population on a frame that ALSO creates new bodies.
            // The previous form gated the whole step on "no body was created this frame" (if (!createdAny)), which
            // froze every body for the entire duration of a cross-frame spray (the spawner creates each frame, so no
            // frame ever stepped until the spray ended) — the deferred-simulation bug. The parity harness lockstep
            // still holds: its first group Update has no live body to step (the step is a no-op) and then creates
            // all bodies; each later Update steps the now-live population exactly once before creating nothing, so
            // capture s still reflects (s + 1) integrations, matched against the reference's per-loop Simulate.
            {
                var dt = SystemAPI.Time.DeltaTime;

                // Drain every body entity's runtime write-in command buffer onto its Box2D body BEFORE the step,
                // then clear the buffer. This is the runtime force/impulse/torque/velocity/MovePosition surface
                // (Phase 7): the user appends commands via PhysicsBody2DCommands; the apply happens here, on the
                // step frame, immediately before Simulate, so a continuous Force command is integrated by Box2D
                // during the step (and N such commands sum through Box2D's own force accumulator the way repeated
                // Rigidbody2D.AddForce calls sum within one FixedUpdate), an Impulse command's velocity delta is
                // in place before the step, and a MovePosition/MoveRotation kinematic sweep reaches its target
                // over this dt. The buffer is cleared so each command drives exactly one step (the one-shot,
                // per-step semantics of the GameObject calls). The loop must be inline in OnUpdate — SystemAPI
                // .Query is source-generated against this system instance, not callable from a static helper.
                // Mass-scaling and frozen-axis cancellation are the solver's job: the raw force/impulse is passed
                // through and Box2D divides by the body's resolved mass/inertia and zeroes a frozen DOF, so a
                // write-in matches the equivalent Rigidbody2D call without the package re-deriving either.
                // A SkipInterpolation command resets the entity's PhysicsBody2DSmoothing (read by the render-rate
                // smoothing system) — a per-entity component the body handle alone cannot reach. The lookup is
                // grabbed once here and indexed per command, so a body with no smoothing component (interpolation
                // None) is a cheap HasComponent miss and the command is a no-op for it.
                var smoothingLookup = SystemAPI.GetComponentLookup<PhysicsBody2DSmoothing>(
                    isReadOnly: false
                );
                foreach (
                    var (bodyRO, commands, entity) in SystemAPI
                        .Query<RefRO<PhysicsBody2D>, DynamicBuffer<PhysicsBody2DCommand>>()
                        .WithEntityAccess()
                )
                {
                    var body = bodyRO.ValueRO.body;
                    if (body.isValid)
                        for (var i = 0; i < commands.Length; i++)
                        {
                            var cmd = commands[i];
                            // SkipInterpolation carries no body call — it resets the entity's smoothing to the
                            // body's CURRENT pose (so it must observe any SetTransform already drained this frame,
                            // which is why it is handled here, after ApplyCommand, off the live body).
                            if (cmd.kind == PhysicsBody2DCommandKind.SkipInterpolation)
                                ResetSmoothing(body, entity, ref smoothingLookup);
                            else
                                ApplyCommand(body, cmd, dt);
                        }
                    commands.Clear();
                }

                // Force-field effectors (Phase 10a): Area / Buoyancy / Point. Each effector entity carries a
                // baked PhysicsEffector2D definition alongside its sensor PhysicsShape2D (its trigger collider
                // region) and a static body. Here, BEFORE Simulate (the same pre-step force-accumulation window
                // the command drain above uses), each effector overlap-queries its region for the dynamic bodies
                // inside it and applies the GameObject force formula to each — so the effector force accumulates
                // with the body's gravity/commands and is mass-scaled + frozen-axis-cancelled by the solver
                // during Simulate, exactly like a Rigidbody2D.AddForce(_, Force). The overlap query runs
                // pre-Simulate, so there is no volatile-span concern (that is a post-Simulate concern). The loop
                // is inline because SystemAPI.Query is source-generated against this system instance. The hit
                // list is reused across all effectors this frame.
                var effectorHits = new NativeList<PhysicsQueryHit2D>(16, Allocator.Temp);
                var bodyLookup = SystemAPI.GetComponentLookup<PhysicsBody2D>(isReadOnly: true);
                foreach (
                    var (effBodyRO, effRO, effShapeRO, effEntity) in SystemAPI
                        .Query<
                            RefRO<PhysicsBody2D>,
                            RefRO<PhysicsEffector2D>,
                            RefRO<PhysicsShape2D>
                        >()
                        .WithEntityAccess()
                )
                {
                    var effBody = effBodyRO.ValueRO.body;
                    if (!effBody.isValid)
                        continue;
                    ApplyEffector(
                        world,
                        effBody,
                        effEntity,
                        effRO.ValueRO,
                        effShapeRO.ValueRO,
                        effectorHits,
                        bodyLookup,
                        dt
                    );
                }
                effectorHits.Dispose();

                world.Simulate(dt);
                // Read the post-step event spans into the owned buffers IMMEDIATELY, before any other world
                // call. The spans are ReadOnlySpans into native scratch the XML says is "cleared immediately
                // after being provided" and that "any change to the world state can invalidate" — copying each
                // event (resolve entities + copy the raw handles) into ECS-owned buffers here is the only safe
                // way to expose them. No Unity.U2D.Physics WORLD mutation happens between Simulate and these
                // reads; ResolveEntity / UnpackEntity only READ a shape/joint (isValid/body/userData), which
                // does not invalidate the spans. The actual joint destroy/disable is a structural change, so it
                // is deferred to PhysicsJoint2DBreakSystem ([UpdateAfter] this system) — here we only collect.
                CollectEvents(world, contactEvents, triggerEvents);
                // Resolve each break's owner entity and its baked action. The definition lookup only READS ECS
                // data (it does not mutate the Box2D world), so it is safe inside the volatile-span loop.
                var jointDefLookup = SystemAPI.GetComponentLookup<PhysicsJoint2DDefinition>(
                    isReadOnly: true
                );
                CollectJointBreaks(world, jointBreakEvents, jointDefLookup);

                // Record this step's time so PhysicsBody2DSmoothingSystem (render rate) can compute how far the
                // render time is ahead of the last physics step. ElapsedTime/DeltaTime here are the fixed
                // group's clock (this system runs in FixedStepSimulationSystemGroup).
                SystemAPI.SetSingleton(
                    new PhysicsFixedStepTime2D
                    {
                        elapsedTime = SystemAPI.Time.ElapsedTime,
                        deltaTime = dt,
                    }
                );
            }

            // Create a Box2D body+shape for each baked entity lacking a live PhysicsBody2D handle, AFTER the step
            // above. Three paths, chosen per entity by the cached-template optimisation
            // (PhysicsWorld2DConfig.cacheIdenticalBodies):
            //   - the unchanged per-entity path (CreatePerEntityBody) — for a heterogeneous bake, a warm-up body
            //     below the threshold, a no-form-hash body, a multi-shape body, or a kind not value-cacheable;
            //   - the cheap cached-template path — for an isBuilt form's body, replaying the prepared
            //     definition/geometry/mass instead of re-constructing them;
            //   - the same-frame in-frame collapse — for K >= 2 isBuilt-form bodies that land in ONE frame, one
            //     CreateBodyBatch instead of K CreateBody calls (the inner routine factored from the removed batch
            //     creation system). A 1/frame cross-frame spray gets K = 1 and the plain template path.
            // The PhysicsBody2D / PhysicsBody2DCleanup / PhysicsBody2DSmoothing component adds are identical across
            // all three paths (AddPhysicalBodyComponents) and are deferred through one ECB played back immediately,
            // so the new bodies are live for the NEXT step (the step above already ran for this frame). Whatever path
            // created a body, the RESULT is identical — a cached-template body is bit-identical to a per-entity one
            // (same definition + same shape → same Box2D body), which is what keeps the optimisation transparent.
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            // Same-frame buckets: per form-hash, the entities of an isBuilt form that arrived this frame and are
            // deferred for the in-frame CreateBodyBatch collapse. Allocated lazily only when the optimisation admits
            // at least one such entity.
            var collapse = default(NativeParallelMultiHashMap<uint4, PendingInit>);

            foreach (
                var (defRO, shapeRO, entity) in SystemAPI
                    .Query<RefRO<PhysicsBody2DDefinition>, RefRO<PhysicsShape2D>>()
                    .WithNone<PhysicsBody2D>()
                    .WithEntityAccess()
            )
            {
                var d = defRO.ValueRO;
                var sh = shapeRO.ValueRO;

                var hasVelocity = SystemAPI.HasComponent<PhysicsBody2DInitialVelocity>(entity);
                var velocity = hasVelocity
                    ? SystemAPI.GetComponent<PhysicsBody2DInitialVelocity>(entity)
                    : default;
                var hasExtraShapes = SystemAPI.HasBuffer<PhysicsShape2DElement>(entity);
                var extra = hasExtraShapes
                    ? SystemAPI.GetBuffer<PhysicsShape2DElement>(entity)
                    : default;

                // Eligibility for the cached arm: the optimisation is on, the entity carries a baked form hash, it
                // is single-shape, and its kind has a value-cacheable geometry (Circle/Box/Capsule/simple-Polygon —
                // Edge's chain geometry and a decompose polygon are NOT cacheable, multi-shape is excluded). Anything
                // failing this falls to the unchanged per-entity path, exactly as before this optimisation existed.
                var geometry = default(GeometryUnion);
                var eligible =
                    cacheEnabled
                    && !hasExtraShapes
                    && SystemAPI.HasComponent<PhysicsBody2DFormHash>(entity)
                    && TryGetCacheableGeometry(in sh, out geometry);

                if (!eligible)
                {
                    var bodyA = CreatePerEntityBody(
                        world,
                        entity,
                        in d,
                        in sh,
                        hasVelocity,
                        in velocity,
                        extra
                    );
                    AddPhysicalBodyComponents(ref ecb, entity, in d, bodyA);
                    continue;
                }

                var hash = SystemAPI.GetComponent<PhysicsBody2DFormHash>(entity).value;
                m_Templates.TryGetValue(hash, out var tmpl);
                tmpl.seenCount++;

                if (tmpl.isBuilt != 0)
                {
                    // The form's template already exists. Defer this entity into the same-frame bucket; the K-member
                    // runs are created after the scan (one CreateBodyBatch for K >= 2, the plain template path for
                    // K == 1). The template stays in the map; only the counter advanced.
                    m_Templates[hash] = tmpl;
                    if (!collapse.IsCreated)
                        collapse = new NativeParallelMultiHashMap<uint4, PendingInit>(
                            64,
                            Allocator.Temp
                        );
                    collapse.Add(
                        hash,
                        new PendingInit
                        {
                            entity = entity,
                            position = d.initialPosition,
                            rotationRadians = d.initialRotationRadians,
                            hasVelocity = (byte)(hasVelocity ? 1 : 0),
                            velocity = velocity,
                        }
                    );
                    continue;
                }

                // Below or crossing the threshold. Create this body through the unchanged per-entity path (its
                // result is the donor for the template), and on the crossing build the template from it.
                var body = CreatePerEntityBody(
                    world,
                    entity,
                    in d,
                    in sh,
                    hasVelocity,
                    in velocity,
                    extra
                );
                AddPhysicalBodyComponents(ref ecb, entity, in d, body);

                if (tmpl.seenCount >= threshold)
                {
                    tmpl.bodyDef = BuildTemplateBodyDef(in d); // pose + velocity zeroed; overwritten per instance
                    tmpl.shapeDef = BuildShapeDef(in sh);
                    tmpl.geometry = geometry;
                    tmpl.mass = ResolveMass(body, in d); // the donor's resolved write — bit-identical for the form
                    tmpl.interpolation = d.interpolation; // the form's smoothing mode (part of the form hash)
                    tmpl.isBuilt = 1;
                }
                m_Templates[hash] = tmpl;
            }

            // Drain the same-frame buckets: one CreateBodyBatch per form with K >= 2 members, the plain template
            // path for a lone member. The bodies' ECS components are added through the same ECB.
            if (collapse.IsCreated)
            {
                DrainSameFrameCollapse(world, ref ecb, collapse);
                collapse.Dispose();
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        // Apply one runtime write-in command to a live body. Managed Unity.U2D.Physics instance calls on the main
        // thread — not Burst, like the rest of this system. The Force/Impulse distinction is carried by the kind:
        // a Force/Torque maps to the accumulating ApplyForce*/ApplyTorque (step-integrated by Simulate), an
        // Impulse/AngularImpulse to the instantaneous ApplyLinearImpulse*/ApplyAngularImpulse (modifies velocity
        // now). Every Apply* passes wake:true (Rigidbody2D.AddForce implicitly wakes a sleeping body); a direct
        // velocity set wakes the body explicitly. MovePosition/MoveRotation map to SetTransformTarget(target, dt)
        // — the native swept, collision-aware, sleep-thresholded, auto-waking kinematic move — built from the
        // supplied target plus the body's CURRENT other component (position keeps current rotation, and vice
        // versa) so a position-only move does not also snap the rotation.
        static void ApplyCommand(PhysicsBody body, in PhysicsBody2DCommand cmd, float dt)
        {
            switch (cmd.kind)
            {
                case PhysicsBody2DCommandKind.Force:
                    body.ApplyForceToCenter((Vector2)cmd.linear, true);
                    break;
                case PhysicsBody2DCommandKind.ForceAtPosition:
                    body.ApplyForce((Vector2)cmd.linear, (Vector2)cmd.worldPoint, true);
                    break;
                case PhysicsBody2DCommandKind.Impulse:
                    body.ApplyLinearImpulseToCenter((Vector2)cmd.linear, true);
                    break;
                case PhysicsBody2DCommandKind.ImpulseAtPosition:
                    body.ApplyLinearImpulse((Vector2)cmd.linear, (Vector2)cmd.worldPoint, true);
                    break;
                case PhysicsBody2DCommandKind.Torque:
                    body.ApplyTorque(cmd.angular, true);
                    break;
                case PhysicsBody2DCommandKind.AngularImpulse:
                    body.ApplyAngularImpulse(cmd.angular, true);
                    break;
                case PhysicsBody2DCommandKind.SetLinearVelocity:
                    body.linearVelocity = (Vector2)cmd.linear;
                    body.awake = true;
                    break;
                case PhysicsBody2DCommandKind.SetAngularVelocity:
                    body.angularVelocity = cmd.angular;
                    body.awake = true;
                    break;
                case PhysicsBody2DCommandKind.MovePosition:
                    body.SetTransformTarget(
                        new PhysicsTransform((Vector2)cmd.linear, body.rotation),
                        dt
                    );
                    break;
                case PhysicsBody2DCommandKind.MoveRotation:
                    body.SetTransformTarget(
                        new PhysicsTransform(body.position, PhysicsRotate.FromRadians(cmd.angular)),
                        dt
                    );
                    break;
                case PhysicsBody2DCommandKind.MovePositionAndRotation:
                    body.SetTransformTarget(
                        new PhysicsTransform(
                            (Vector2)cmd.linear,
                            PhysicsRotate.FromRadians(cmd.angular)
                        ),
                        dt
                    );
                    break;
                case PhysicsBody2DCommandKind.SetTransform:
                    // The INSTANTANEOUS pose set — a hard teleport, NOT the swept SetTransformTarget. A direct write
                    // to body.transform (native b2Body_SetTransform): no deltaTime, no velocity, so the world's
                    // maximumLinearSpeed clamp (which gates only the velocity-based swept move) never applies and the
                    // body reaches an arbitrarily far destination in one step. The per-axis flag in worldPoint.x
                    // keeps the unchanged axis at the body's CURRENT value, so a position-only set does not snap the
                    // rotation and vice versa, mirroring the Move* kinds.
                    var setPos = ((int)cmd.worldPoint.x & 1) != 0;
                    var setRot = ((int)cmd.worldPoint.x & 2) != 0;
                    var newPos = setPos ? (Vector2)cmd.linear : body.position;
                    var newRot = setRot ? PhysicsRotate.FromRadians(cmd.angular) : body.rotation;
                    body.transform = new PhysicsTransform(newPos, newRot);
                    break;
                case PhysicsBody2DCommandKind.SkipInterpolation:
                    // Handled in the drain loop (it needs the entity's smoothing component, not the body handle);
                    // never reaches ApplyCommand. Listed so the switch is exhaustive over the command kinds.
                    break;
            }
        }

        // Reset a teleported body's render-rate smoothing to its current (just-set) pose so the next render frame
        // draws the new pose with no interpolation streak from the old one — the 2D analogue of the 3D
        // CharacterInterpolation.SkipNextInterpolation(). Seeds prev == cur == the live pose and clears hasPrev, the
        // same shape PhysicsWorld2DSystem's creation path seeds a fresh body's smoothing with (where prev == cur ==
        // the authored pose and hasPrev = 0 makes the first render pass an identity write of the current pose). A
        // body that carries no smoothing component (interpolation None) has no streak to suppress, so this is a
        // HasComponent no-op for it. Managed Unity.U2D.Physics reads on the main thread, like the rest of this system.
        static void ResetSmoothing(
            PhysicsBody body,
            Entity entity,
            ref ComponentLookup<PhysicsBody2DSmoothing> smoothingLookup
        )
        {
            if (!smoothingLookup.HasComponent(entity))
                return;

            var pos = (float2)(Vector2)body.position;
            var rot = body.rotation;
            var cosSin = float2(rot.cos, rot.sin);

            var s = smoothingLookup[entity];
            s.prevPos = pos;
            s.curPos = pos;
            s.prevCosSin = cosSin;
            s.curCosSin = cosSin;
            s.hasPrev = 0;
            smoothingLookup[entity] = s;
        }

        // Apply one force-field effector to every dynamic body overlapping its region this step. Box2D-v3 has no
        // native effector — an Effector2D is a GameObject-physics abstraction the package reproduces here: the
        // effector's own (sensor) shape defines a region; an overlap query (honoring the effector's colliderMask)
        // returns the bodies inside it; the per-kind force formula is applied to each as a continuous force (the
        // upcoming Simulate accumulates + integrates + mass-scales it) plus drag as a velocity multiplier.
        // Managed Unity.U2D.Physics calls on the main thread — not Burst, like the rest of this system.
        static void ApplyEffector(
            PhysicsWorld world,
            PhysicsBody effectorBody,
            Entity effectorEntity,
            in PhysicsEffector2D eff,
            in PhysicsShape2D effShape,
            NativeList<PhysicsQueryHit2D> hits,
            ComponentLookup<PhysicsBody2D> bodyLookup,
            float dt
        )
        {
            var bodyPos = (float2)(Vector2)effectorBody.position;
            var bodyRot = effectorBody.rotation;
            var bodyAngle = atan2(bodyRot.sin, bodyRot.cos);

            // Surface is a conveyor: it drives the bodies in CONTACT with its (solid) surface, not bodies in a
            // region, so it discovers them via the surface body's live contact list rather than a region overlap.
            if (eff.kind == PhysicsEffector2DKind.Surface)
            {
                ApplySurfaceDrive(effectorBody, effectorEntity, eff, bodyLookup);
                return;
            }

            // Platform is a one-way gate on a SOLID collider: it applies no force to bodies; it classifies each
            // nearby body (blocking vs passing) against the surface arc and toggles its OWN body's participation so
            // a body inside the arc rests on it while a body outside it passes through (an approximation — the
            // faithful per-contact pre-solve veto is unreachable; see the Phase-10b design's negative space). It
            // owns its own EXPANDED region query: a fast body's collision contact forms a step before the tight
            // shape overlap reports it, so an approaching body must be detected a margin out to disable the
            // platform BEFORE the solver forms the contact that would stop it.
            if (eff.kind == PhysicsEffector2DKind.Platform)
            {
                ApplyPlatformOneWay(
                    world,
                    effectorBody,
                    effectorEntity,
                    eff,
                    effShape,
                    bodyPos,
                    bodyAngle,
                    hits,
                    bodyLookup
                );
                return;
            }

            // Region overlap from the effector's baked SENSOR shape (Area/Buoyancy/Point). Box → OverlapBox
            // (center = body pose folded with the shape offset, oriented by the body rotation); Circle →
            // OverlapCircle. The two shapes the query API and the example effector scenes use. For any other shape
            // kind, fall back to the body's world AABB as an axis-aligned box (a coarse region; flagged for a
            // future OverlapCapsule/Polygon).
            switch (effShape.kind)
            {
                case PhysicsShape2DKind.Circle:
                {
                    var center = bodyPos + Rotate(effShape.offset, bodyRot.cos, bodyRot.sin);
                    PhysicsQueries2D.OverlapCircle(
                        world,
                        center,
                        effShape.radius,
                        eff.colliderMask,
                        hits
                    );
                    break;
                }
                case PhysicsShape2DKind.Box:
                {
                    var center = bodyPos + Rotate(effShape.offset, bodyRot.cos, bodyRot.sin);
                    PhysicsQueries2D.OverlapBox(
                        world,
                        center,
                        effShape.size,
                        bodyAngle,
                        eff.colliderMask,
                        hits
                    );
                    break;
                }
                default:
                {
                    var aabb = effectorBody.GetAABB();
                    var lo = (float2)(Vector2)aabb.lowerBound;
                    var hi = (float2)(Vector2)aabb.upperBound;
                    PhysicsQueries2D.OverlapBox(
                        world,
                        (lo + hi) * 0.5f,
                        hi - lo,
                        0f,
                        eff.colliderMask,
                        hits
                    );
                    break;
                }
            }

            for (var i = 0; i < hits.Length; i++)
            {
                var target = hits[i].entity;
                // Skip the effector's own sensor shape (a self-hit) and any body the query did not resolve.
                if (target == effectorEntity || target == Entity.Null)
                    continue;
                if (!bodyLookup.HasComponent(target))
                    continue;
                var body = bodyLookup[target].body;
                // Only dynamic bodies are pushed by an effector. PhysicsBody.type is the (non-obsolete) runtime
                // body-type getter, returning PhysicsBody.BodyType.
                if (!body.isValid || body.type != PhysicsBody.BodyType.Dynamic)
                    continue;

                switch (eff.kind)
                {
                    case PhysicsEffector2DKind.Area:
                        ApplyAreaForce(body, eff, bodyAngle);
                        break;
                    case PhysicsEffector2DKind.Buoyancy:
                        ApplyBuoyancyForce(body, eff, dt);
                        break;
                    case PhysicsEffector2DKind.Point:
                        ApplyPointForce(body, effectorBody, effShape, eff);
                        break;
                }

                // Drag (the per-body extra damping the effector imposes while inside). Buoyancy scales its drag by
                // the submerged fraction inside ApplyBuoyancyForce; Area/Point apply the full drag here.
                if (eff.kind != PhysicsEffector2DKind.Buoyancy)
                    Drag(body, eff.linearDamping, eff.angularDamping, dt);
            }
        }

        // Area: a directional force at forceAngle (world-space if useGlobalAngle, else relative to the effector
        // body rotation), magnitude forceMagnitude (+ variation). forceTarget Rigidbody → at the body centre of
        // mass (pure linear); Collider → at the body collider centroid (worldCenterOfMass for a single-shape
        // body, may add torque off-centre). Applied as a continuous force the upcoming Simulate integrates.
        static void ApplyAreaForce(PhysicsBody body, in PhysicsEffector2D eff, float effectorAngle)
        {
            var angle =
                eff.useGlobalAngle != 0
                    ? eff.forceAngleRadians
                    : eff.forceAngleRadians + effectorAngle;
            sincos(angle, out var s, out var c);
            var f = new float2(c, s) * (eff.forceMagnitude + Variation(eff.forceVariation));
            if (eff.forceTargetIsRigidbody != 0)
                body.ApplyForceToCenter((Vector2)f, true);
            else
                body.ApplyForce((Vector2)f, body.worldCenterOfMass, true);
        }

        // Buoyancy: an upward force scaled by the submerged fraction (from the body's world AABB vs surfaceLevel),
        // plus a submerged-fraction-scaled flow force and fluid drag. The buoyant force is fluidDensity·f·g·mass
        // upward, so after the solver divides by mass the upward acceleration is fluidDensity·f·g — net vertical
        // accel g·(fluidDensity·f − 1) (zero at f = 1/fluidDensity, the float equilibrium). The exact buoyancy
        // constant is the parity target the validating gate confirms against the GameObject oracle.
        static void ApplyBuoyancyForce(PhysicsBody body, in PhysicsEffector2D eff, float dt)
        {
            var aabb = body.GetAABB();
            var bot = ((Vector2)aabb.lowerBound).y;
            var top = ((Vector2)aabb.upperBound).y;
            var h = top - bot;
            if (h <= 0f)
                return;
            var submerged = clamp(eff.surfaceLevel - bot, 0f, h);
            var f = submerged / h;
            if (f <= 0f)
                return;

            var mass = body.mass;
            var fy = eff.fluidDensity * f * eff.gravityMagnitude * mass;
            body.ApplyForceToCenter(new Vector2(0f, fy), true);

            if (eff.flowMagnitude != 0f || eff.flowVariation != 0f)
            {
                sincos(eff.flowAngleRadians, out var fs, out var fc);
                var flow =
                    new float2(fc, fs) * ((eff.flowMagnitude + Variation(eff.flowVariation)) * f);
                body.ApplyForceToCenter((Vector2)flow, true);
            }

            // Fluid drag acts on the submerged part only.
            Drag(body, eff.linearDamping * f, eff.angularDamping * f, dt);
        }

        // Point: a radial force from the source point (the effector collider centroid, or its body centre of mass
        // for forceSource Rigidbody) to the target (the affected body centre of mass), with the forceMode falloff
        // over distance × distanceScale. A negative forceMagnitude attracts toward the point; positive repels.
        static void ApplyPointForce(
            PhysicsBody body,
            PhysicsBody effectorBody,
            in PhysicsShape2D effShape,
            in PhysicsEffector2D eff
        )
        {
            float2 source;
            if (eff.forceSourceIsRigidbody != 0)
            {
                source = (float2)(Vector2)effectorBody.worldCenterOfMass;
            }
            else
            {
                var rot = effectorBody.rotation;
                source =
                    (float2)(Vector2)effectorBody.position
                    + Rotate(effShape.offset, rot.cos, rot.sin);
            }

            var targetPos = (float2)(Vector2)body.worldCenterOfMass;
            var delta = targetPos - source;
            var dist = length(delta) * eff.distanceScale;
            var dir = normalizesafe(delta);

            var mag = eff.forceMagnitude;
            switch (eff.forceMode)
            {
                case 1: // InverseLinear
                    if (dist > 1e-5f)
                        mag /= dist;
                    break;
                case 2: // InverseSquared
                    if (dist > 1e-5f)
                        mag /= dist * dist;
                    break;
                // case 0 (Constant): no falloff.
            }

            var f = dir * (mag + Variation(eff.forceVariation));
            body.ApplyForceToCenter((Vector2)f, true);
        }

        // Platform (one-way): gate the platform's OWN body participation so a body within the surface arc rests on
        // it (solid) while a body outside the arc passes through (transparent). Box2D-v3's faithful per-contact
        // pre-solve veto (IPreSolveCallback.OnPreSolve2D) is unreachable from the package's native-poll posture (it
        // is callback-only, mid-step, any-thread, world-write-locked, managed-callbackTarget-per-shape), so this
        // is a per-step WHOLE-BODY approximation: it is faithful for the single-interacting-body case (the example
        // scenes drop one body at a time), with a documented gap for simultaneous mixed bodies (one above resting +
        // one below passing) — see the Phase-10b design's negative space. The platform applies NO force to bodies.
        // The margin (metres) the platform region is grown by for the one-way detection query. A fast body's
        // collision contact forms a step before the tight platform-shape overlap reports it, so an approaching
        // body must be caught a margin out to disable the platform BEFORE the solver forms the stopping contact.
        // ~2 m catches a body several steps out at typical speeds; larger only widens the detection zone.
        const float PlatformOneWayMargin = 2f;

        static void ApplyPlatformOneWay(
            PhysicsWorld world,
            PhysicsBody platformBody,
            Entity platformEntity,
            in PhysicsEffector2D eff,
            in PhysicsShape2D effShape,
            float2 platformPos,
            float platformAngle,
            NativeList<PhysicsQueryHit2D> hits,
            ComponentLookup<PhysicsBody2D> bodyLookup
        )
        {
            // useOneWay off → a plain solid platform, no gating.
            if (eff.useOneWay == 0)
            {
                platformBody.enabled = true;
                return;
            }

            // EXPANDED region query (the platform shape grown by the detection margin), so an approaching body is
            // seen before the solver forms the contact. Box → grown OverlapBox; Circle → grown OverlapCircle; else
            // the body AABB grown.
            switch (effShape.kind)
            {
                case PhysicsShape2DKind.Circle:
                {
                    var center =
                        platformPos
                        + Rotate(effShape.offset, cos(platformAngle), sin(platformAngle));
                    PhysicsQueries2D.OverlapCircle(
                        world,
                        center,
                        effShape.radius + PlatformOneWayMargin,
                        eff.colliderMask,
                        hits
                    );
                    break;
                }
                case PhysicsShape2DKind.Box:
                {
                    var center =
                        platformPos
                        + Rotate(effShape.offset, cos(platformAngle), sin(platformAngle));
                    PhysicsQueries2D.OverlapBox(
                        world,
                        center,
                        effShape.size + (2f * PlatformOneWayMargin),
                        platformAngle,
                        eff.colliderMask,
                        hits
                    );
                    break;
                }
                default:
                {
                    var aabb = platformBody.GetAABB();
                    var lo = (float2)(Vector2)aabb.lowerBound - PlatformOneWayMargin;
                    var hi = (float2)(Vector2)aabb.upperBound + PlatformOneWayMargin;
                    PhysicsQueries2D.OverlapBox(
                        world,
                        (lo + hi) * 0.5f,
                        hi - lo,
                        0f,
                        eff.colliderMask,
                        hits
                    );
                    break;
                }
            }

            // The platform's world up axis (the surface-arc centre): local +Y rotated by the body angle + the
            // rotational offset. A body "from above" has its direction-from-platform within ±(surfaceArc/2) of
            // this up; a body outside the arc is "from below / the side".
            sincos(platformAngle + eff.rotationalOffsetRadians, out var us, out var uc);
            var up = new float2(-us, uc); // rotate (0,1) by the angle
            var cosHalfArc = cos(eff.surfaceArcRadians * 0.5f);

            var anyBlocking = false;
            var anyPassing = false;
            for (var i = 0; i < hits.Length; i++)
            {
                var target = hits[i].entity;
                if (target == platformEntity || target == Entity.Null)
                    continue;
                if (!bodyLookup.HasComponent(target))
                    continue;
                var body = bodyLookup[target].body;
                if (!body.isValid || body.type != PhysicsBody.BodyType.Dynamic)
                    continue;

                var rel = (float2)(Vector2)body.worldCenterOfMass - platformPos;
                var dir = normalizesafe(rel, up);
                var vel = (float2)(Vector2)body.linearVelocity;
                // Blocking: the body sits within the surface arc (above) and is not moving up through the platform.
                // Passing: outside the arc (below / the side) OR moving up through it (a body launched from below).
                var withinArc = dot(dir, up) >= cosHalfArc;
                var movingThrough = dot(vel, up) > 1e-3f;
                if (withinArc && !movingThrough)
                    anyBlocking = true;
                else
                    anyPassing = true;
            }

            // Solid when a blocking body is present (so it rests); transparent when ONLY passing bodies overlap (so
            // they pass through). Recomputed every step, so a platform left transparent with no passing body next
            // step re-solidifies. The whole-body gate is the approximation's limit: a simultaneous blocking +
            // passing pair keeps the platform solid (the passing body is then wrongly blocked) — the documented gap.
            platformBody.enabled = !(anyPassing && !anyBlocking);
        }

        // Surface (conveyor): drive every dynamic body in CONTACT with the surface body tangentially toward the
        // belt speed, via a velocity-error linear impulse so the tangential velocity converges without overshoot.
        // Contacting bodies are discovered from the surface body's live contact list (PhysicsBody.GetContacts), the
        // CURRENT touching set pre-Simulate (a body riding the belt persists in it every step) — not the Phase-6
        // begin/end edge buffer, which would drive a riding body for only one step. The impulse is mass-scaled
        // (impulse ∝ mass) so every body reaches the same belt speed regardless of mass, matching the GameObject
        // "maintain a speed" semantics. Managed Unity.U2D.Physics calls on the main thread — not Burst.
        static void ApplySurfaceDrive(
            PhysicsBody surfaceBody,
            Entity surfaceEntity,
            in PhysicsEffector2D eff,
            ComponentLookup<PhysicsBody2D> bodyLookup
        )
        {
            // The belt tangent: the surface body's local +X in world space (positive speed drives along it). A
            // curved/multi-segment belt would need a per-contact-segment tangent — a flagged refinement; the
            // example belts are horizontal edges driven along local X.
            var rot = surfaceBody.rotation;
            var tangent = new float2(rot.cos, rot.sin);
            var beltSpeed = eff.surfaceSpeed + Variation(eff.surfaceSpeedVariation);
            var scale = clamp(eff.forceScale, 0f, 1f);

            var contacts = surfaceBody.GetContacts(Allocator.Temp);
            for (var i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                // The contact's two shapes: one is the surface's own shape, the other the riding body's. Resolve the
                // body shape (the one whose entity is NOT the surface) and skip the self / unresolved / non-Dynamic.
                var entityA = PhysicsQueries2D.ResolveEntity(c.shapeA);
                var entityB = PhysicsQueries2D.ResolveEntity(c.shapeB);
                var target = entityA == surfaceEntity ? entityB : entityA;
                if (target == surfaceEntity || target == Entity.Null)
                    continue;
                // Effector-level colliderMask: GetContacts returns every contact regardless of the effector mask,
                // so an off-mask body is filtered here. A 0 / all-ones mask means "every layer" (the query
                // convention). The mask is matched against the body's shape category bit.
                if (!bodyLookup.HasComponent(target))
                    continue;
                var body = bodyLookup[target].body;
                if (!body.isValid || body.type != PhysicsBody.BodyType.Dynamic)
                    continue;
                if (!MaskAllows(eff.colliderMask, body))
                    continue;

                var vel = (float2)(Vector2)body.linearVelocity;
                var vTan = dot(vel, tangent);
                var dv = beltSpeed - vTan;
                var impulse = tangent * (scale * dv * body.mass);
                if (eff.useContactForce != 0)
                {
                    // Apply at the contact point so an off-centre contact also imparts the matching spin.
                    var point = ContactPoint(c, body);
                    body.ApplyLinearImpulse((Vector2)impulse, (Vector2)point, true);
                }
                else
                {
                    body.ApplyLinearImpulseToCenter((Vector2)impulse, true);
                }
            }
            if (contacts.IsCreated)
                contacts.Dispose();
        }

        // Whether an effector colliderMask admits a body's shape category. A 0 or all-ones mask means "every
        // layer" (the PhysicsQueries2D layer-mask convention). Otherwise the body's shape category bit must
        // intersect the mask. The body's category is read from its first shape's contact filter.
        static bool MaskAllows(ulong colliderMask, PhysicsBody body)
        {
            if (colliderMask == 0ul || colliderMask == ~0ul)
                return true;
            var shapes = body.GetShapes(Allocator.Temp);
            var allowed = false;
            for (var i = 0; i < shapes.Length; i++)
            {
                if ((shapes[i].contactFilter.categories.bitMask & colliderMask) != 0ul)
                {
                    allowed = true;
                    break;
                }
            }
            if (shapes.IsCreated)
                shapes.Dispose();
            return allowed;
        }

        // The world-space contact point for a surface contact, for useContactForce. The contact manifold's first
        // point if available, else the body's centre of mass (a degenerate fallback that makes the impulse purely
        // linear — identical to useContactForce off).
        static float2 ContactPoint(PhysicsShape.Contact c, PhysicsBody body)
        {
            var manifold = c.manifold;
            if (manifold.pointCount > 0)
                return (float2)(Vector2)manifold[0].point;
            return (float2)(Vector2)body.worldCenterOfMass;
        }

        // Per-body damping as a velocity multiplier v *= 1/(1 + c·dt), the standard Unity/Box2D damping
        // integrator (NOT a force). A non-positive coefficient is a no-op.
        static void Drag(PhysicsBody body, float linearDamping, float angularDamping, float dt)
        {
            if (linearDamping > 0f)
            {
                var scale = 1f / (1f + linearDamping * dt);
                body.linearVelocity = (Vector2)((float2)(Vector2)body.linearVelocity * scale);
            }
            if (angularDamping > 0f)
                body.angularVelocity *= 1f / (1f + angularDamping * dt);
        }

        // A random in [-variation, +variation], or 0 when variation is 0 (the deterministic, parity-asserted
        // path every example scene authors). The random path is a documented non-deterministic feature.
        static float Variation(float variation)
        {
            if (variation == 0f)
                return 0f;
            return UnityEngine.Random.Range(-variation, variation);
        }

        // Rotate a local 2D vector by a (cos, sin) rotation — folds a shape offset into world space.
        static float2 Rotate(float2 v, float cos, float sin)
        {
            return new float2(v.x * cos - v.y * sin, v.x * sin + v.y * cos);
        }

        // Drain the four post-step event spans into the singleton's owned buffers, resolving every event shape
        // back to its owning entity through the Phase-5 userData packing (shape.body.userData → Entity). Called
        // only right after Simulate, while the spans are valid. Managed Unity.U2D.Physics span reads on the main
        // thread — not Burst, like the rest of this system.
        static void CollectEvents(
            PhysicsWorld world,
            DynamicBuffer<PhysicsContactEvent2D> contactEvents,
            DynamicBuffer<PhysicsTriggerEvent2D> triggerEvents
        )
        {
            var begin = world.contactBeginEvents;
            for (var i = 0; i < begin.Length; i++)
            {
                var e = begin[i];
                contactEvents.Add(
                    new PhysicsContactEvent2D
                    {
                        phase = PhysicsEventPhase2D.Begin,
                        entityA = PhysicsQueries2D.ResolveEntity(e.shapeA),
                        entityB = PhysicsQueries2D.ResolveEntity(e.shapeB),
                        shapeA = e.shapeA,
                        shapeB = e.shapeB,
                    }
                );
            }

            var end = world.contactEndEvents;
            for (var i = 0; i < end.Length; i++)
            {
                var e = end[i];
                contactEvents.Add(
                    new PhysicsContactEvent2D
                    {
                        phase = PhysicsEventPhase2D.End,
                        entityA = PhysicsQueries2D.ResolveEntity(e.shapeA),
                        entityB = PhysicsQueries2D.ResolveEntity(e.shapeB),
                        shapeA = e.shapeA,
                        shapeB = e.shapeB,
                    }
                );
            }

            var triggerBegin = world.triggerBeginEvents;
            for (var i = 0; i < triggerBegin.Length; i++)
            {
                var e = triggerBegin[i];
                triggerEvents.Add(
                    new PhysicsTriggerEvent2D
                    {
                        phase = PhysicsEventPhase2D.Begin,
                        triggerEntity = PhysicsQueries2D.ResolveEntity(e.triggerShape),
                        visitorEntity = PhysicsQueries2D.ResolveEntity(e.visitorShape),
                        triggerShape = e.triggerShape,
                        visitorShape = e.visitorShape,
                    }
                );
            }

            var triggerEnd = world.triggerEndEvents;
            for (var i = 0; i < triggerEnd.Length; i++)
            {
                var e = triggerEnd[i];
                triggerEvents.Add(
                    new PhysicsTriggerEvent2D
                    {
                        phase = PhysicsEventPhase2D.End,
                        triggerEntity = PhysicsQueries2D.ResolveEntity(e.triggerShape),
                        visitorEntity = PhysicsQueries2D.ResolveEntity(e.visitorShape),
                        triggerShape = e.triggerShape,
                        visitorShape = e.visitorShape,
                    }
                );
            }
        }

        // Drain the post-step joint-threshold span into the singleton's break buffer. Box2D produces a
        // jointThresholdEvent for each joint whose reaction force/torque exceeded its armed forceThreshold/
        // torqueThreshold this step, but does NOT destroy the joint — the package destroys/disables it (per the
        // baked breakAction) in PhysicsJoint2DBreakSystem, which runs AFTER this system so the structural change
        // (DestroyJointBatch + RemoveComponent) is legal. Here we only COPY each event out of the volatile span
        // (resolving the owner entity via the joint's packed userData and reading the baked breakAction from the
        // joint's owner-entity definition is deferred — instead the breakAction is read from the joint's owner
        // entity in the apply system). Called only right after Simulate, while the span is valid. Managed
        // Unity.U2D.Physics span reads on the main thread — not Burst, like the rest of this system.
        static void CollectJointBreaks(
            PhysicsWorld world,
            DynamicBuffer<PhysicsJointBreakEvent2D> jointBreakEvents,
            ComponentLookup<PhysicsJoint2DDefinition> jointDefLookup
        )
        {
            var events = world.jointThresholdEvents;
            for (var i = 0; i < events.Length; i++)
            {
                var joint = events[i].joint;
                // A joint in the span "may have been deleted since this event was produced" (module XML) — skip
                // an invalid handle. (A still-valid joint is destroyed by the apply system, not here.)
                if (!joint.isValid)
                    continue;
                var entity = PhysicsQueries2D.UnpackEntity(joint.userData);
                // The baked action decides destroy vs surface-only; read it from the owner entity's definition
                // (a pure ECS read — no world mutation, safe in the span loop). A resolved entity that no longer
                // carries the definition (e.g. despawned the same step) falls back to CallbackOnly (surface
                // only, do not destroy a handle whose owner is gone).
                var action = jointDefLookup.HasComponent(entity)
                    ? jointDefLookup[entity].breakAction
                    : PhysicsJointBreakAction2D.CallbackOnly;
                jointBreakEvents.Add(
                    new PhysicsJointBreakEvent2D
                    {
                        jointEntity = entity,
                        joint = joint,
                        breakAction = action,
                    }
                );
            }
        }
    }
}
