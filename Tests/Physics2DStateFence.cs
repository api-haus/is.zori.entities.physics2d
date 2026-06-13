using UnityEngine;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Snapshot of the two global <c>UnityEngine.Physics2D</c> knobs a PlayMode gate mutates to drive its
    /// GameObject Box2D-v2 oracle deterministically — <c>simulationMode</c> and <c>gravity</c>. A gate captures
    /// the live values and switches to <c>Script</c> mode plus its chosen gravity in <c>[SetUp]</c> (or a
    /// medium's constructor), then <see cref="Restore"/>s them in <c>[TearDown]</c> (or <c>Dispose</c>), so no
    /// test leaks the global mode or gravity into the next one. The package namespace shadows <c>Physics2D</c>,
    /// so the engine type is fully qualified here. The layer-collision matrix and the query flags are NOT part
    /// of this fence — a gate that mutates those snapshots them alongside its own state.
    /// </summary>
    readonly struct Physics2DStateFence
    {
        readonly SimulationMode2D m_Mode;
        readonly Vector2 m_Gravity;

        Physics2DStateFence(SimulationMode2D mode, Vector2 gravity)
        {
            m_Mode = mode;
            m_Gravity = gravity;
        }

        public static Physics2DStateFence EnterScriptMode(Vector2 gravity)
        {
            var saved = new Physics2DStateFence(UnityEngine.Physics2D.simulationMode, UnityEngine.Physics2D.gravity);
            UnityEngine.Physics2D.simulationMode = SimulationMode2D.Script;
            UnityEngine.Physics2D.gravity = gravity;
            return saved;
        }

        public readonly void Restore()
        {
            UnityEngine.Physics2D.gravity = m_Gravity;
            UnityEngine.Physics2D.simulationMode = m_Mode;
        }
    }
}
