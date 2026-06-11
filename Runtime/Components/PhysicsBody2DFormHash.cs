using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.Physics2D
{
    /// <summary>
    /// A stable 128-bit content hash of a body's <em>form</em> — every field that feeds the Box2D body and
    /// shape definitions, with <b>pose and initial velocity excluded</b> — computed once at bake time and
    /// carried as a blittable component so the runtime never recomputes it. It is the grouping/lookup key the
    /// cached-template creation path (<c>PhysicsWorld2DSystem</c>) uses to recognise that N instantiated bodies
    /// share one form and serve them from a single prepared template instead of re-constructing the body+shape
    /// definitions per entity.
    /// </summary>
    /// <remarks>
    /// <para><b>Why pose is excluded.</b> <c>initialPosition</c> / <c>initialRotationRadians</c> and the optional
    /// <c>PhysicsBody2DInitialVelocity</c> seed are per-instance launch parameters applied individually at
    /// creation; folding them into the hash would split every spray into singleton forms and defeat the whole
    /// mechanism. The fields that DO enter the hash are the body-form fields of <see cref="PhysicsBody2DDefinition"/>
    /// (type, damping, gravity scale, constraints, mass policy, CCD, interpolation, mass-distribution override)
    /// and the shape fields of <see cref="PhysicsShape2D"/> (kind, offset, geometry, surface, filter, trigger),
    /// plus — for Polygon/Edge — the vertex blob content, and every extra <see cref="PhysicsShape2DElement"/> in
    /// buffer order.</para>
    ///
    /// <para><b>Why a baking SYSTEM, not a baker.</b> The body definition and the shape come from DIFFERENT bakers
    /// (<c>Rigidbody2DBaker</c> vs the collider bakers), and a <c>Baker&lt;T&gt;</c> never sees another baker's
    /// output on the same entity. The hash is therefore computed by <c>PhysicsBody2DFormHashBakingSystem</c>, a
    /// baking-world system ordered after the component bakers, as a pure function of the already-baked runtime
    /// components — so the built-in, custom-authoring, and direct paths all produce the same hash for the same
    /// form for free.</para>
    ///
    /// <para><b>How an instance carries it for free.</b> <c>ecb.Instantiate(prefab)</c> replicates every blittable
    /// component by value, so a baked prefab carrying this lands the identical 16-byte hash on every instance at
    /// zero runtime cost — the instance is self-describing.</para>
    /// </remarks>
    public struct PhysicsBody2DFormHash : IComponentData
    {
        /// <summary>The 128-bit form hash as a <c>uint4</c> (a <c>Hash128</c>-equivalent value). Equal hashes
        /// mean equal forms (a collision is astronomically unlikely, and the template path additionally rebuilds
        /// from the first donor entity's real components, so a collision at worst routes to a verified-by-field
        /// donor — see <c>PhysicsWorld2DSystem</c>).</summary>
        public uint4 value;
    }
}
