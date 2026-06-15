using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
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
    /// The ECS-facing spatial-query surface over the package's <see cref="PhysicsWorld"/> — the analogue of
    /// <c>UnityEngine.Physics2D.Raycast</c> / <c>OverlapCircle</c> / <c>OverlapBox</c> / <c>CircleCast</c> /
    /// <c>BoxCast</c>. Each helper takes the world handle (obtained from
    /// <see cref="PhysicsWorldSingleton2D"/>), an optional layer mask, and a caller-owned result container, and
    /// returns hits as <see cref="PhysicsQueryHit2D"/> (owning entity, point, normal, fraction, raw shape) —
    /// the same data the GameObject calls expose for the same scene.
    /// </summary>
    /// <remarks>
    /// <b>When valid.</b> A query reads the simulation state, so it is valid only against a stepped world — i.e.
    /// after <see cref="PhysicsWorld2DSystem"/> has run for the frame, or from inside a job that captured the
    /// world handle as a <c>[ReadOnly]</c> field (the engine's own <c>PhysicsQueryJob</c> pattern). Obtain the
    /// world with <c>SystemAPI.GetSingleton&lt;PhysicsWorldSingleton2D&gt;().world</c>.
    ///
    /// <b>Layer mask.</b> Pass a 64-bit <c>hitLayerMask</c> of the categories the query may hit. A mask of
    /// <c>0</c> or <c>~0ul</c> means "hit every layer" (the <c>Physics2D.DefaultRaycastLayers</c> /
    /// <c>AllLayers</c> convention), so a caller that does not care about layers passes <c>0</c> and gets all
    /// hits rather than none. The mask is written into <c>PhysicsQuery.QueryFilter.hitCategories</c>, the same
    /// field the GameObject layer-mask raycast honors.
    ///
    /// <b>Hit→entity.</b> A query returns a Box2D shape; <see cref="ResolveEntity"/> maps it back to the owning
    /// entity through <c>shape.body.userData.int64Value</c>, which the creation systems pack with the entity at
    /// body creation (<see cref="PackEntity"/>). A hit on a body with no packed entity resolves to
    /// <see cref="Entity.Null"/>.
    ///
    /// <b>Burst.</b> These are plain <c>static</c> helpers (no <c>[BurstCompile]</c>, per the entry-point-only
    /// rule): they are HPC#-clean (struct math, native containers, and the Box2D query calls the engine's own
    /// <c>PhysicsQueryJob</c> runs inside a Burst job), so a caller's own <c>[BurstCompile]</c> job can call
    /// them and they auto-compile from that context, while a main-thread system calls them managed.
    /// </remarks>
    public static class PhysicsQueries2D
    {
        // ---- entity <-> userData packing -------------------------------------------------------------------

        /// <summary>
        /// Pack an <see cref="Entity"/> into a <see cref="PhysicsUserData"/> for storage on a body's userData,
        /// so a query hit can recover the owning entity. The 64-bit <c>int64Value</c> holds
        /// <c>(Index &lt;&lt; 32) | (uint)Version</c> — lossless for the two 32-bit halves of an Entity.
        /// </summary>
        public static PhysicsUserData PackEntity(Entity entity)
        {
            var packed = ((ulong)(uint)entity.Index << 32) | (uint)entity.Version;
            return new PhysicsUserData { int64Value = packed };
        }

        /// <summary>Inverse of <see cref="PackEntity"/>. A zero <c>int64Value</c> unpacks to a zero Entity,
        /// which equals <see cref="Entity.Null"/> (Index 0, Version 0).</summary>
        public static Entity UnpackEntity(PhysicsUserData userData)
        {
            var packed = userData.int64Value;
            return new Entity { Index = (int)(packed >> 32), Version = (int)(packed & 0xFFFFFFFFul) };
        }

        /// <summary>Resolve the entity owning a hit shape, via its body's packed userData. Returns
        /// <see cref="Entity.Null"/> for an invalid shape or a body the package did not pack.</summary>
        public static Entity ResolveEntity(PhysicsShape shape)
        {
            if (!shape.isValid)
                return Entity.Null;
            var body = shape.body;
            if (!body.isValid)
                return Entity.Null;
            return UnpackEntity(body.userData);
        }

        // ---- query filter ----------------------------------------------------------------------------------

        // Build a QueryFilter honoring a layer mask. A 0 or all-ones mask means "hit everything".
        //
        // The query matches a shape by WHAT THE SHAPE IS (its categories), not by what it collides with. Box2D's
        // query-vs-shape match is bidirectional: (shape.categories & query.hitCategories) != 0 AND
        // (shape.contacts & query.categories) != 0. We keep query.categories = All (inherited from Everything) so
        // the second clause reduces to shape.contacts != 0, and put the caller's layer mask in hitCategories so the
        // first clause is the category test. The mask therefore selects shapes by category; the shape's own
        // collision-matrix row is decoupled at shape creation, where a categorized "collides with nothing" shape is
        // baked with a non-zero contacts so it stays query-visible (PhysicsWorld2DSystem.QueryVisibleContacts).
        static PhysicsQuery.QueryFilter Filter(ulong hitLayerMask)
        {
            var filter = PhysicsQuery.QueryFilter.Everything;
            if (hitLayerMask != 0ul && hitLayerMask != ~0ul)
                filter.hitCategories = new PhysicsMask { bitMask = hitLayerMask };
            return filter;
        }

        static PhysicsQueryHit2D ToHit(in PhysicsQuery.WorldCastResult r)
        {
            return new PhysicsQueryHit2D
            {
                entity = ResolveEntity(r.shape),
                point = (float2)(Vector2)r.point,
                normal = (float2)(Vector2)r.normal,
                fraction = r.fraction,
                shape = r.shape,
            };
        }

        static PhysicsQueryHit2D ToHit(in PhysicsQuery.WorldOverlapResult r)
        {
            return new PhysicsQueryHit2D
            {
                entity = ResolveEntity(r.shape),
                point = new float2(0f, 0f),
                normal = new float2(0f, 0f),
                fraction = 0f,
                shape = r.shape,
            };
        }

        // ---- raycast ---------------------------------------------------------------------------------------

        /// <summary>
        /// Cast a ray (a finite line segment) from <paramref name="origin"/> along
        /// <paramref name="direction"/> for <paramref name="distance"/>, appending every hit (sorted nearest
        /// first) into <paramref name="hits"/>. Honors <paramref name="hitLayerMask"/>. Returns the hit count.
        /// </summary>
        public static int Raycast(
            PhysicsWorld world,
            float2 origin,
            float2 direction,
            float distance,
            ulong hitLayerMask,
            NativeList<PhysicsQueryHit2D> hits
        )
        {
            hits.Clear();
            if (!world.isValid)
                return 0;
            var dir = normalizesafe(direction);
            var input = new PhysicsQuery.CastRayInput
            {
                origin = (Vector2)origin,
                translation = (Vector2)(dir * distance),
                maxFraction = 1f,
            };
            var results = world.CastRay(
                input,
                Filter(hitLayerMask),
                PhysicsQuery.WorldCastMode.AllSorted,
                Allocator.Temp
            );
            for (var i = 0; i < results.Length; i++)
                hits.Add(ToHit(results[i]));
            results.Dispose();
            return hits.Length;
        }

        /// <summary>Cast a ray and return only the closest hit. Returns false (and a default hit) if nothing
        /// was hit. Honors <paramref name="hitLayerMask"/>.</summary>
        public static bool RaycastClosest(
            PhysicsWorld world,
            float2 origin,
            float2 direction,
            float distance,
            ulong hitLayerMask,
            out PhysicsQueryHit2D hit
        )
        {
            hit = default;
            if (!world.isValid)
                return false;
            var dir = normalizesafe(direction);
            var input = new PhysicsQuery.CastRayInput
            {
                origin = (Vector2)origin,
                translation = (Vector2)(dir * distance),
                maxFraction = 1f,
            };
            var results = world.CastRay(
                input,
                Filter(hitLayerMask),
                PhysicsQuery.WorldCastMode.Closest,
                Allocator.Temp
            );
            var hadHit = results.Length > 0;
            if (hadHit)
            {
                hit = ToHit(results[0]);
                results.Dispose();
            }
            return hadHit;
        }

        // ---- overlap ---------------------------------------------------------------------------------------

        /// <summary>Find every shape overlapping a circle, appending hits into <paramref name="hits"/>. Honors
        /// the layer mask. Returns the hit count.</summary>
        public static int OverlapCircle(
            PhysicsWorld world,
            float2 center,
            float radius,
            ulong hitLayerMask,
            NativeList<PhysicsQueryHit2D> hits
        )
        {
            hits.Clear();
            if (!world.isValid)
                return 0;
            var geometry = new CircleGeometry { radius = radius, center = (Vector2)center };
            var results = world.OverlapGeometry(geometry, Filter(hitLayerMask), Allocator.Temp);
            for (var i = 0; i < results.Length; i++)
                hits.Add(ToHit(results[i]));
            results.Dispose();
            return hits.Length;
        }

        /// <summary>Find every shape overlapping an oriented box (full <paramref name="size"/> extents,
        /// rotated <paramref name="angleRadians"/>), appending hits into <paramref name="hits"/>. Honors the
        /// layer mask. Returns the hit count.</summary>
        public static int OverlapBox(
            PhysicsWorld world,
            float2 center,
            float2 size,
            float angleRadians,
            ulong hitLayerMask,
            NativeList<PhysicsQueryHit2D> hits
        )
        {
            hits.Clear();
            if (!world.isValid)
                return 0;
            var transform = new PhysicsTransform((Vector2)center, new PhysicsRotate(angleRadians));
            var geometry = PolygonGeometry.CreateBox((Vector2)size, 0f, transform, inscribe: false);
            var results = world.OverlapGeometry(geometry, Filter(hitLayerMask), Allocator.Temp);
            for (var i = 0; i < results.Length; i++)
                hits.Add(ToHit(results[i]));
            results.Dispose();
            return hits.Length;
        }

        /// <summary>Test whether any shape overlaps a world-space point (honoring the layer mask). The cheap
        /// boolean form, mapping <c>UnityEngine.Physics2D.OverlapPoint</c> presence to a yes/no.</summary>
        public static bool OverlapPoint(PhysicsWorld world, float2 point, ulong hitLayerMask)
        {
            if (!world.isValid)
                return false;
            return world.TestOverlapPoint((Vector2)point, Filter(hitLayerMask));
        }

        // ---- shape cast ------------------------------------------------------------------------------------

        /// <summary>Cast a circle through the world along <paramref name="direction"/> for
        /// <paramref name="distance"/>, appending hits (nearest first) into <paramref name="hits"/>. Honors the
        /// layer mask. Returns the hit count. The analogue of <c>Physics2D.CircleCast</c>.</summary>
        public static int CircleCast(
            PhysicsWorld world,
            float2 origin,
            float radius,
            float2 direction,
            float distance,
            ulong hitLayerMask,
            NativeList<PhysicsQueryHit2D> hits
        )
        {
            hits.Clear();
            if (!world.isValid)
                return 0;
            var dir = normalizesafe(direction);
            var geometry = new CircleGeometry { radius = radius, center = (Vector2)origin };
            var translation = (Vector2)(dir * distance);
            var results = world.CastGeometry(
                geometry,
                translation,
                Filter(hitLayerMask),
                PhysicsQuery.WorldCastMode.AllSorted,
                Allocator.Temp
            );
            for (var i = 0; i < results.Length; i++)
                hits.Add(ToHit(results[i]));
            results.Dispose();
            return hits.Length;
        }

        /// <summary>Cast an oriented box through the world along <paramref name="direction"/> for
        /// <paramref name="distance"/>, appending hits (nearest first) into <paramref name="hits"/>. Honors the
        /// layer mask. Returns the hit count. The analogue of <c>Physics2D.BoxCast</c>.</summary>
        public static int BoxCast(
            PhysicsWorld world,
            float2 origin,
            float2 size,
            float angleRadians,
            float2 direction,
            float distance,
            ulong hitLayerMask,
            NativeList<PhysicsQueryHit2D> hits
        )
        {
            hits.Clear();
            if (!world.isValid)
                return 0;
            var dir = normalizesafe(direction);
            var transform = new PhysicsTransform((Vector2)origin, new PhysicsRotate(angleRadians));
            var geometry = PolygonGeometry.CreateBox((Vector2)size, 0f, transform, inscribe: false);
            var translation = (Vector2)(dir * distance);
            var results = world.CastGeometry(
                geometry,
                translation,
                Filter(hitLayerMask),
                PhysicsQuery.WorldCastMode.AllSorted,
                Allocator.Temp
            );
            for (var i = 0; i < results.Length; i++)
                hits.Add(ToHit(results[i]));
            results.Dispose();
            return hits.Length;
        }

        /// <summary>
        /// Cast a capsule through the world along <paramref name="direction"/> for <paramref name="distance"/>,
        /// appending hits (nearest first) into <paramref name="hits"/>. Honors the layer mask. Returns the hit
        /// count. The capsule is the two world-space end-cap centers <paramref name="center1"/> /
        /// <paramref name="center2"/> and the end <paramref name="radius"/> — the same
        /// <c>CapsuleGeometry.Create</c> the creation system builds a capsule body's shape from
        /// (<see cref="PhysicsWorld2DSystem"/>). The analogue of <c>CircleCast</c>/<c>BoxCast</c> for a capsule
        /// proxy, the cast a rounded-bottom character sweeps so its caps clear step edges a box corner catches.
        /// </summary>
        public static int CapsuleCast(
            PhysicsWorld world,
            float2 center1,
            float2 center2,
            float radius,
            float2 direction,
            float distance,
            ulong hitLayerMask,
            NativeList<PhysicsQueryHit2D> hits
        )
        {
            hits.Clear();
            if (!world.isValid)
                return 0;
            var dir = normalizesafe(direction);
            var geometry = CapsuleGeometry.Create((Vector2)center1, (Vector2)center2, radius);
            var translation = (Vector2)(dir * distance);
            var results = world.CastGeometry(
                geometry,
                translation,
                Filter(hitLayerMask),
                PhysicsQuery.WorldCastMode.AllSorted,
                Allocator.Temp
            );
            for (var i = 0; i < results.Length; i++)
                hits.Add(ToHit(results[i]));
            results.Dispose();
            return hits.Length;
        }

        /// <summary>Find every shape overlapping a capsule (the two end-cap centers
        /// <paramref name="center1"/> / <paramref name="center2"/> and the end <paramref name="radius"/>),
        /// appending hits into <paramref name="hits"/>. Honors the layer mask. Returns the hit count. The capsule
        /// analogue of <see cref="OverlapCircle"/> / <see cref="OverlapBox"/>.</summary>
        public static int OverlapCapsule(
            PhysicsWorld world,
            float2 center1,
            float2 center2,
            float radius,
            ulong hitLayerMask,
            NativeList<PhysicsQueryHit2D> hits
        )
        {
            hits.Clear();
            if (!world.isValid)
                return 0;
            var geometry = CapsuleGeometry.Create((Vector2)center1, (Vector2)center2, radius);
            var results = world.OverlapGeometry(geometry, Filter(hitLayerMask), Allocator.Temp);
            for (var i = 0; i < results.Length; i++)
                hits.Add(ToHit(results[i]));
            results.Dispose();
            return hits.Length;
        }

        // ---- closest point / distance ----------------------------------------------------------------------

        /// <summary>
        /// The nearest world body to a query point, with the closest point on that body, the separation distance,
        /// and the surface normal — the substrate analogue of <c>com.unity.physics</c>'s
        /// <c>CollisionWorld.CalculateDistance(PointDistanceInput)</c> the 3D character controller uses for
        /// anchor detection and depenetration. There is no world-level distance call on the 2D world; this
        /// composes the existing overlap broad-phase with the per-pair <c>PhysicsQuery.ShapeDistance</c> exact
        /// distance: a circle of <paramref name="maxDistance"/> finds the candidate shapes within range, then
        /// <c>ShapeDistance</c> measures the query point (a zero-radius circle proxy) against each candidate and
        /// the closest is returned. Honors <paramref name="hitLayerMask"/>. Returns false (and a default result)
        /// when no body is within <paramref name="maxDistance"/>.
        /// </summary>
        /// <remarks>
        /// <see cref="ClosestPoint2D.distance"/> is zero when the point is inside a body (Box2D returns a zero
        /// distance for an overlap), and <see cref="ClosestPoint2D.normal"/> is then degenerate — a caller that
        /// needs a push-out direction from inside a body uses the overlap+cast-back path, not this query.
        /// <paramref name="hits"/> is a caller-owned scratch list the broad-phase overlap reuses (cleared on
        /// entry), so a hot caller avoids a per-call allocation. Like the rest of the surface this is a plain
        /// <c>static</c> helper (HPC#-clean, no <c>[BurstCompile]</c>), valid only against a stepped world.
        /// </remarks>
        public static bool ClosestPoint(
            PhysicsWorld world,
            float2 point,
            float maxDistance,
            ulong hitLayerMask,
            NativeList<PhysicsQueryHit2D> hits,
            out ClosestPoint2D result
        )
        {
            result = default;
            if (!world.isValid)
                return false;

            // Broad phase: every shape whose body is within maxDistance of the point (a generous over-set — the
            // overlap is against a disc of that radius, so it includes any body whose nearest surface is within
            // range). The exact ShapeDistance below culls the rest.
            OverlapCircle(world, point, max(maxDistance, 0f), hitLayerMask, hits);
            if (hits.Length == 0)
                return false;

            // A single-point shape proxy (the ShapeProxy(Vector2) form — radius 0, no validity check, unlike the
            // CircleGeometry ctor which rejects a zero radius). The world position rides on transformA, so the
            // local proxy point is the origin.
            var queryProxy = new PhysicsShape.ShapeProxy(Vector2.zero);
            var queryTransform = new PhysicsTransform((Vector2)point);

            var found = false;
            var best = float.MaxValue;
            for (var i = 0; i < hits.Length; i++)
            {
                var shape = hits[i].shape;
                if (!shape.isValid)
                    continue;
                var d = PhysicsQuery.ShapeDistance(
                    new PhysicsQuery.DistanceInput
                    {
                        shapeProxyA = queryProxy,
                        transformA = queryTransform,
                        shapeProxyB = shape.CreateShapeProxy(false), // PhysicsShape -> ShapeProxy (6000.6's implicit conversion is exactly CreateShapeProxy(false))
                        transformB = shape.body.transform,
                        useRadii = true,
                    }
                );
                if (d.distance > maxDistance || d.distance >= best)
                    continue;
                best = d.distance;
                result = new ClosestPoint2D
                {
                    entity = ResolveEntity(shape),
                    point = (float2)(Vector2)d.pointB,
                    normal = (float2)(Vector2)d.normal,
                    distance = d.distance,
                    shape = shape,
                };
                found = true;
            }
            return found;
        }
    }
}
