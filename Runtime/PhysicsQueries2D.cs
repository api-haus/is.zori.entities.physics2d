using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
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
            var transform = new PhysicsTransform((Vector2)center, PhysicsRotate.FromRadians(angleRadians));
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
            var transform = new PhysicsTransform((Vector2)origin, PhysicsRotate.FromRadians(angleRadians));
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
    }
}
