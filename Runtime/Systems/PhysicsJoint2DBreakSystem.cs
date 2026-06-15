using Unity.Collections;
using Unity.Entities;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// Applies the structural reaction to a joint break that <c>PhysicsWorld2DSystem</c> only collected. For
    /// each <see cref="PhysicsJointBreakEvent2D"/> whose action is <see cref="PhysicsJointBreakAction2D.Destroy"/>
    /// or <see cref="PhysicsJointBreakAction2D.Disable"/>, destroys the Box2D joint (the bodies then separate),
    /// removes <see cref="PhysicsJoint2D"/> from the owner entity, and adds the sticky
    /// <see cref="PhysicsJoint2DBroken"/> tag so the creation system never re-forms it. A
    /// <see cref="PhysicsJointBreakAction2D.CallbackOnly"/> event leaves the joint in place (the constraint
    /// still holds; only the surfaced event fires).
    /// </summary>
    /// <remarks>
    /// <b>Why a separate system, after the world system.</b> The break events are read from Box2D's volatile
    /// post-step joint-threshold span inside <c>PhysicsWorld2DSystem</c>, where a destroy (a world mutation)
    /// and a <c>RemoveComponent</c> (a structural change) are both illegal mid-span-read. The collect/apply
    /// split mirrors the contact/trigger event collection and the body cleanup: collect from the
    /// volatile span in the world system, do the structural change in a sibling system. Running
    /// <c>[UpdateAfter(PhysicsWorld2DSystem)]</c> in the same <see cref="Physics2DSimulationSystemGroup"/> tick
    /// means a joint loaded past its threshold this step is gone before the next step integrates the (now free)
    /// bodies.
    ///
    /// <b>Teardown.</b> Destroying the joint frees the Box2D constraint; the bodies are NOT destroyed (the
    /// package never owns entity lifetime — a consumer reads the break event and decides whether to destroy or
    /// disable on its side, mirroring the built-in <c>Destroy</c> vs <c>Disable</c> distinction the action
    /// carries). <c>PhysicsBody2DCleanupSystem</c> still frees a body when its entity is destroyed.
    ///
    /// <b>Not <c>[BurstCompile]</c>.</b> <c>DestroyJointBatch</c> is a managed <c>Unity.U2D.Physics</c> call on
    /// the main thread, exactly like the joint creation in the sibling system — Burst is confined to the
    /// write-back / smoothing jobs.
    /// </remarks>
    [UpdateInGroup(typeof(Physics2DSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsWorld2DSystem))]
    public partial struct PhysicsJoint2DBreakSystem : ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            // The break events live on the world singleton, refilled each step by PhysicsWorld2DSystem. No
            // singleton yet (no step has run) → nothing to do.
            if (!SystemAPI.TryGetSingletonBuffer<PhysicsJointBreakEvent2D>(out var breaks, isReadOnly: true))
                return;
            if (breaks.Length == 0)
                return;

            // Collect the joints to actually destroy (Destroy/Disable) and the entities to mark broken. Done in
            // one pass into temp arrays, then the destroy + structural changes are applied outside any span.
            var handles = new NativeList<PhysicsJoint>(breaks.Length, Allocator.Temp);
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            for (var i = 0; i < breaks.Length; i++)
            {
                var e = breaks[i];
                // CallbackOnly (and any Ignore that slipped through) leaves the joint intact — surface only.
                if (
                    e.breakAction != PhysicsJointBreakAction2D.Destroy
                    && e.breakAction != PhysicsJointBreakAction2D.Disable
                )
                    continue;
                if (e.joint.isValid)
                    handles.Add(e.joint);
                // Remove the live-handle marker and tag the entity broken so the creation query never re-forms
                // the joint. Guard on the entity still existing + still carrying the marker.
                if (e.jointEntity != Entity.Null && SystemAPI.HasComponent<PhysicsJoint2D>(e.jointEntity))
                {
                    ecb.RemoveComponent<PhysicsJoint2D>(e.jointEntity);
                    ecb.AddComponent<PhysicsJoint2DBroken>(e.jointEntity);
                }
            }

            // Destroy every broken joint in one batch (the world is implied by the handles); ignores any handle
            // already invalid. Then play back the component changes.
            if (handles.Length > 0)
                PhysicsWorld.DestroyJointBatch(handles.AsArray().AsReadOnlySpan());
            ecb.Playback(state.EntityManager);

            handles.Dispose();
            ecb.Dispose();
        }
    }
}
