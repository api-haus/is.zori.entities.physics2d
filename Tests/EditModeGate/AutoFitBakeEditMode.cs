using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using Zori.Entities.Physics2D.Tests.Editor;
using static Unity.Mathematics.math;
#if UNITY_6000_6_OR_NEWER
using Unity.U2D.Physics;
#else
using UnityEngine.LowLevelPhysics2D;
#endif

namespace Zori.Entities.Physics2D.Tests.EditModeGate
{
    /// <summary>
    /// EditMode port of <c>AutoFitBakeGate</c>: a fitted Box bakes bit-identical to a hand-authored Box of the
    /// same local geometry, and a fitted falling Circle rests at <c>floorTop + fittedRadius</c>. Geometry is set
    /// by the REAL <c>PhysicsShape2DAutoFit</c> at fixture build time. Assertions, tolerances and step count
    /// copied verbatim from the PlayMode gate.
    /// </summary>
    public sealed class AutoFitBakeEditMode : Physics2DEditModeHarness
    {
        const float BoxW = Physics2DFixtures.AutoFitBoxW;
        const float BoxH = Physics2DFixtures.AutoFitBoxH;
        const float XFitted = Physics2DFixtures.AutoFitXFitted;
        const float XHand = Physics2DFixtures.AutoFitXHand;
        const float CircleRadius = Physics2DFixtures.AutoFitCircleRadius;
        const float FloorTop = Physics2DFixtures.AutoFitFloorTop;
        const float Eps = 1e-5f;

        [Test]
        public void FittedBox_BakesBitIdenticalToHandAuthored_AndFittedCircleRestsAtFittedRadius()
        {
            LoadSubScene(Physics2DFixtures.AutoFitBake, "AutoFitBake");

            var shapeQuery = Query(
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>()
            );
            Assert.GreaterOrEqual(shapeQuery.CalculateEntityCount(), 4, "Expected >= 4 baked shapes.");

            using var shapes = shapeQuery.ToComponentDataArray<PhysicsShape2D>(Allocator.Temp);
            using var defs = shapeQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);

            var haveFitted = false;
            var haveHand = false;
            PhysicsShape2D fittedBox = default;
            PhysicsShape2D handBox = default;
            for (var i = 0; i < shapes.Length; i++)
            {
                if (shapes[i].kind != PhysicsShape2DKind.Box)
                    continue;
                var x = defs[i].initialPosition.x;
                if (abs(x - XFitted) < 0.25f)
                {
                    fittedBox = shapes[i];
                    haveFitted = true;
                }
                else if (abs(x - XHand) < 0.25f)
                {
                    handBox = shapes[i];
                    haveHand = true;
                }
            }
            Assert.IsTrue(haveFitted && haveHand, $"Missing a baked box body (fitted={haveFitted}, hand={haveHand}).");

            Assert.AreEqual(handBox.size.x, fittedBox.size.x, 0f, "Fitted box width != hand-authored.");
            Assert.AreEqual(handBox.size.y, fittedBox.size.y, 0f, "Fitted box height != hand-authored.");
            Assert.AreEqual(handBox.boxAngleRadians, fittedBox.boxAngleRadians, Eps, "Fitted box angle != hand.");
            Assert.AreEqual(handBox.radius, fittedBox.radius, 0f, "Fitted box corner radius != hand 0.");
            Assert.AreEqual(handBox.offset.x, fittedBox.offset.x, Eps, "Fitted box offset.x != hand.");
            Assert.AreEqual(handBox.offset.y, fittedBox.offset.y, Eps, "Fitted box offset.y != hand.");

            Assert.AreEqual(BoxW, fittedBox.size.x, Eps, "Fitted box width != the 4-unit source extent.");
            Assert.AreEqual(BoxH, fittedBox.size.y, Eps, "Fitted box height != the 2-unit source extent.");

            var haveCircle = false;
            PhysicsShape2D fittedCircle = default;
            for (var i = 0; i < shapes.Length; i++)
                if (shapes[i].kind == PhysicsShape2DKind.Circle)
                {
                    fittedCircle = shapes[i];
                    haveCircle = true;
                }
            Assert.IsTrue(haveCircle, "No baked Circle shape — the fitted falling circle is missing.");
            Assert.AreEqual(
                CircleRadius,
                fittedCircle.radius,
                1e-3f,
                "Fitted circle radius != the ring source radius."
            );

            // First update creates the bodies (no integration), then step until the circle settles.
            CreateBodies();

            var dynQuery = Query(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            var restY = float.NaN;
            for (var s = 0; s < 300; s++)
            {
                Step(1);
                restY = ReadDynamicY(dynQuery);
            }

            Debug.Log(
                $"[PHYSICS2D-PHASEC-AUTOFIT-BAKE-EDITMODE] fitted circle rest Y = {restY}, expected ~"
                    + $"{FloorTop + CircleRadius}."
            );

            Assert.IsFalse(float.IsNaN(restY), "No dynamic body found — the fitted circle did not bake/create.");

            var expectedY = FloorTop + CircleRadius;
            Assert.AreEqual(
                expectedY,
                restY,
                0.06f,
                $"Fitted circle rested at Y={restY}, not at floorTop+fittedRadius={expectedY}."
            );
            Assert.Less(restY, 3f, $"Fitted circle never fell (rest Y={restY}) — a no-op bake/create.");
        }

        static float ReadDynamicY(EntityQuery dynQuery)
        {
            using var ltws = dynQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            using var defs = dynQuery.ToComponentDataArray<PhysicsBody2DDefinition>(Allocator.Temp);
            for (var i = 0; i < ltws.Length; i++)
            {
                if (defs[i].bodyType == PhysicsBody.BodyType.Static)
                    continue;
                return ltws[i].Value.c3.y;
            }
            return float.NaN;
        }
    }
}
