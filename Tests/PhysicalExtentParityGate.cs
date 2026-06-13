using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.TestTools;
using static Unity.Mathematics.math;

namespace Zori.Entities.Physics2D.Tests
{
    /// <summary>
    /// Adversarial cross-path gate for the <see cref="PhysicsShape2D"/> → physical Box2D shape conversion: a shape
    /// authored at dimension <c>D</c> must produce a PHYSICAL Box2D extent of exactly <c>D</c> — equal to the
    /// authored dimension AND equal to the Unity built-in collider of the same authored dimension. Built from the
    /// PHYSICAL decision point (the live <c>PhysicsBody</c>'s created <c>PhysicsShape</c> geometry AABB), not from
    /// the baked ECS field, because the field-vs-(field×scale) comparison is exactly the blind spot that let a
    /// half-extent/size-doubling bug pass undetected through <see cref="Phase12ColliderScaleGate"/> (which asserts
    /// <c>shape.size == authored × scale</c> and creates NO Box2D body) and through the convergence gates (which
    /// use a box only as a tolerant STATIC contact floor, never pinning a custom-authored box's extent against a
    /// built-in <c>BoxCollider2D</c>'s extent head-to-head). The McIlroy posture: verify the decision point the
    /// system actually reaches at creation, not the input the author imagined.
    /// </summary>
    /// <remarks>
    /// Each test creates a body in a DEDICATED disposable <see cref="World"/> (the
    /// <see cref="DirectAndBatchPathValidation"/> pattern — three FixedStep systems, one <c>group.Update()</c>
    /// creates the body without stepping), reads the live <c>PhysicsShape</c> off
    /// <c>PhysicsBody2D.body.GetShapes</c>, and computes the EXACT geometry AABB via the per-kind
    /// <c>CalculateAABB</c> (not <c>PhysicsShape.aabb</c>, which is inflated by the speculative-collision margin).
    /// The Unity ground truth is a throwaway GameObject collider's <c>bounds.size</c>. No <c>WaitForEndOfFrame</c>
    /// (it does not tick in batchmode); coroutines yield <c>null</c>.
    /// </remarks>
    public sealed class PhysicalExtentParityGate
    {
        const float Dt = 1f / 60f;
        const float Tol = 1e-4f;
        static readonly List<BlobAssetReference<PhysicsShape2DVertices>> s_Blobs = new();

        static BlobAssetReference<PhysicsShape2DVertices> Blob(float2[] pts)
        {
            var b = new BlobBuilder(Allocator.Temp);
            ref var root = ref b.ConstructRoot<PhysicsShape2DVertices>();
            var arr = b.Allocate(ref root.points, pts.Length);
            for (var i = 0; i < pts.Length; i++)
                arr[i] = pts[i];
            var blob = b.CreateBlobAssetReference<PhysicsShape2DVertices>(Allocator.Persistent);
            b.Dispose();
            s_Blobs.Add(blob);
            return blob;
        }

        [TearDown]
        public void TearDown()
        {
            foreach (var b in s_Blobs)
                if (b.IsCreated)
                    b.Dispose();
            s_Blobs.Clear();
        }

        static World MakeWorld(out FixedStepSimulationSystemGroup group) =>
            PhysicsTestWorld.Create("PhysicalExtentTestWorld", out group, Dt);

        // The EXACT physical extent (width, height) of the body's created shape(s): create the body, step once to
        // create it, union every shape's exact geometry AABB (per-kind CalculateAABB at identity — the shape's
        // local geometry already carries the folded offset/rotation, and the body sits at the origin).
        static float2 MeasurePhysicalExtent(PhysicsShape2D shape)
        {
            var world = MakeWorld(out var group);
            try
            {
                var em = world.EntityManager;
                var body = new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Dynamic,
                    gravityScale = 0f,
                    useAutoMass = true,
                };
                var e = DirectPhysics2DAuthoring.Create(em, body, shape);
                group.Update(); // first update creates the body (no step)

                var pb = em.GetComponentData<PhysicsBody2D>(e).body;
                Assert.IsTrue(pb.isValid, "Body was not created.");
                var shapes = pb.GetShapes(Allocator.Temp);
                Assert.Greater(shapes.Length, 0, "Body has no shapes.");

                var lo = new float2(float.MaxValue, float.MaxValue);
                var hi = new float2(float.MinValue, float.MinValue);
                var identity = new PhysicsTransform(Vector2.zero);
                for (var i = 0; i < shapes.Length; i++)
                {
                    PhysicsAABB ab;
                    switch (shapes[i].shapeType)
                    {
                        case PhysicsShape.ShapeType.Circle:
                            ab = shapes[i].circleGeometry.CalculateAABB(identity);
                            break;
                        case PhysicsShape.ShapeType.Capsule:
                            ab = shapes[i].capsuleGeometry.CalculateAABB(identity);
                            break;
                        case PhysicsShape.ShapeType.Polygon:
                            ab = shapes[i].polygonGeometry.CalculateAABB(identity);
                            break;
                        default:
                            ab = shapes[i].aabb; // segment/chain — exact-enough for the kinds this gate covers
                            break;
                    }
                    Vector2 l = ab.lowerBound,
                        u = ab.upperBound;
                    lo = min(lo, new float2(l.x, l.y));
                    hi = max(hi, new float2(u.x, u.y));
                }
                shapes.Dispose();
                return hi - lo;
            }
            finally
            {
                world.Dispose();
            }
        }

        static float2 GameObjectBounds<T>(System.Action<T> configure)
            where T : Collider2D
        {
            var go = new GameObject("ExtentGroundTruth");
            try
            {
                var col = go.AddComponent<T>();
                configure(col);
                Vector3 s = col.bounds.size;
                return new float2(s.x, s.y);
            }
            finally
            {
                Object.DestroyImmediate(go);
            }
        }

        static void AssertExtent(string kind, float2 measured, float2 expectedAuthored, float2 groundTruth)
        {
            Debug.Log(
                $"[PHYSICS2D-EXTENT-{kind}] physical=({measured.x:F4},{measured.y:F4}) "
                    + $"authored=({expectedAuthored.x:F4},{expectedAuthored.y:F4}) "
                    + $"unityGT=({groundTruth.x:F4},{groundTruth.y:F4})"
            );
            Assert.AreEqual(
                expectedAuthored.x,
                measured.x,
                Tol,
                $"{kind}: physical width must equal the AUTHORED dimension (1 unit == 1 unit, not 2)."
            );
            Assert.AreEqual(
                expectedAuthored.y,
                measured.y,
                Tol,
                $"{kind}: physical height must equal the AUTHORED dimension."
            );
            Assert.AreEqual(
                groundTruth.x,
                measured.x,
                Tol,
                $"{kind}: physical width must equal the Unity built-in collider's bounds (cross-path parity)."
            );
            Assert.AreEqual(
                groundTruth.y,
                measured.y,
                Tol,
                $"{kind}: physical height must equal the Unity built-in collider's bounds (cross-path parity)."
            );
        }

        // ---------------------------------------------------------------------------------------------------
        // (1) BOX — the reported case. size=(1,1) → physical 1×1 == authored == BoxCollider2D(size=(1,1)).

        [UnityTest]
        public IEnumerator Box_PhysicalExtentEqualsAuthoredAndBoxCollider2D()
        {
            var measured = MeasurePhysicalExtent(
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(1f, 1f),
                    density = 1f,
                    friction = 0.4f,
                }
            );
            var gt = GameObjectBounds<BoxCollider2D>(c => c.size = new Vector2(1f, 1f));
            AssertExtent("BOX", measured, new float2(1f, 1f), gt);
            yield return null;
        }

        // (2) CIRCLE — radius=0.5 → physical diameter 1×1 == CircleCollider2D(radius=0.5).

        [UnityTest]
        public IEnumerator Circle_PhysicalExtentEqualsAuthoredAndCircleCollider2D()
        {
            var measured = MeasurePhysicalExtent(
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            var gt = GameObjectBounds<CircleCollider2D>(c => c.radius = 0.5f);
            AssertExtent("CIRCLE", measured, new float2(1f, 1f), gt);
            yield return null;
        }

        // (3) CAPSULE — the package stores explicit end-cap centres. The equivalent of a Unity vertical
        //     CapsuleCollider2D size=(1,2) is radius=0.5, caps at y=±0.5: physical 1×2.

        [UnityTest]
        public IEnumerator Capsule_PhysicalExtentEqualsAuthoredAndCapsuleCollider2D()
        {
            var measured = MeasurePhysicalExtent(
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Capsule,
                    capsuleCenter1 = new float2(0f, -0.5f),
                    capsuleCenter2 = new float2(0f, 0.5f),
                    radius = 0.5f,
                    density = 1f,
                    friction = 0.4f,
                }
            );
            var gt = GameObjectBounds<CapsuleCollider2D>(c =>
            {
                c.direction = CapsuleDirection2D.Vertical;
                c.size = new Vector2(1f, 2f);
            });
            AssertExtent("CAPSULE", measured, new float2(1f, 2f), gt);
            yield return null;
        }

        // (4) SIMPLE POLYGON — a unit quad spanning ±0.5 → physical 1×1 == PolygonCollider2D of the same points.

        [UnityTest]
        public IEnumerator Polygon_PhysicalExtentEqualsAuthoredAndPolygonCollider2D()
        {
            var quad = new[]
            {
                new float2(-0.5f, -0.5f),
                new float2(0.5f, -0.5f),
                new float2(0.5f, 0.5f),
                new float2(-0.5f, 0.5f),
            };
            var measured = MeasurePhysicalExtent(
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Polygon,
                    polygonDecompose = false,
                    density = 1f,
                    friction = 0.4f,
                    vertices = Blob(quad),
                }
            );
            var gt = GameObjectBounds<PolygonCollider2D>(c =>
                c.SetPath(
                    0,
                    new[]
                    {
                        new Vector2(-0.5f, -0.5f),
                        new Vector2(0.5f, -0.5f),
                        new Vector2(0.5f, 0.5f),
                        new Vector2(-0.5f, 0.5f),
                    }
                )
            );
            AssertExtent("POLYGON", measured, new float2(1f, 1f), gt);
            yield return null;
        }

        // (5) ORIENTED BOX — the custom-authoring-only free z-rotation a BoxCollider2D cannot express. A unit box
        //     rotated 45° has a physical AABB of √2 on each axis: pins boxAngleRadians feeding the geometry, and
        //     that the rotation does NOT double the extent.

        [UnityTest]
        public IEnumerator OrientedBox_PhysicalExtentMatchesRotatedUnitBox()
        {
            var measured = MeasurePhysicalExtent(
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Box,
                    size = new float2(1f, 1f),
                    boxAngleRadians = radians(45f),
                    density = 1f,
                    friction = 0.4f,
                }
            );
            var diag = sqrt(2f); // a unit box rotated 45° spans its diagonal on each axis
            Debug.Log(
                $"[PHYSICS2D-EXTENT-OBOX] physical=({measured.x:F4},{measured.y:F4}) expected=({diag:F4},{diag:F4})"
            );
            Assert.AreEqual(diag, measured.x, 1e-3f, "Oriented unit box width = √2 (rotation, not doubling).");
            Assert.AreEqual(diag, measured.y, 1e-3f, "Oriented unit box height = √2 (rotation, not doubling).");
            yield return null;
        }
    }
}
