/*
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using Box2DX.Common;
using SoftFloat;
using UnityEngine;

using Transform = Box2DX.Common.Transform;

namespace Box2DX.Collision
{
	/// <summary>
	/// Inpute parameters for TimeOfImpact
	/// </summary>
	public struct TOIInput
	{
		public Sweep SweepA;
		public Sweep SweepB;
		public sfloat SweepRadiusA;
		public sfloat SweepRadiusB;
		public sfloat Tolerance;
	}

	internal struct SeparationFunction
	{
		internal enum Type
		{
			Points,
			FaceA,
			FaceB
		};
		
#if ALLOWUNSAFE
		internal unsafe void Initialize(SimplexCache* cache,
			Shape shapeA, Transform TransformA,
			Shape shapeB, Transform TransformB)
		{
			ShapeA = shapeA;
			ShapeB = shapeB;
			int count = cache->Count;
			Box2DXDebug.Assert(0 < count && count < 3);

			if (count == 1)
			{
				FaceType = Type.Points;
				sVector2 localPointA = ShapeA.GetVertex(cache->IndexA[0]);
				sVector2 localPointB = ShapeB.GetVertex(cache->IndexB[0]);
				sVector2 pointA = TransformA.TransformPoint(localPointA);
				sVector2 pointB = TransformB.TransformPoint(localPointB);
				Axis = pointB - pointA;
				Axis.Normalize();
			}
			else if (cache->IndexB[0] == cache->IndexB[1])
			{
				// Two points on A and one on B
				FaceType = Type.FaceA;
				sVector2 localPointA1 = ShapeA.GetVertex(cache->IndexA[0]);
				sVector2 localPointA2 = ShapeA.GetVertex(cache->IndexA[1]);
				sVector2 localPointB = ShapeB.GetVertex(cache->IndexB[0]);
				LocalPoint = 0.5f * (localPointA1 + localPointA2);
				Axis = (localPointA2 - localPointA1).CrossScalarPostMultiply(sfloat.One);
				Axis.Normalize();

				sVector2 normal = TransformA.TransformDirection(Axis);
				sVector2 pointA = TransformA.TransformPoint(LocalPoint);
				sVector2 pointB = TransformB.TransformPoint(localPointB);

				sfloat s = sVector2.Dot(pointB - pointA, normal);
				if (s < sfloat.Zero)
				{
					Axis = -Axis;
				}
			}
			else
			{
				// Two points on B and one or two points on A.
				// We ignore the second point on A.
				FaceType = Type.FaceB;
				sVector2 localPointA = shapeA.GetVertex(cache->IndexA[0]);
				sVector2 localPointB1 = shapeB.GetVertex(cache->IndexB[0]);
				sVector2 localPointB2 = shapeB.GetVertex(cache->IndexB[1]);
				LocalPoint = 0.5f * (localPointB1 + localPointB2);
				Axis = (localPointB2 - localPointB1).CrossScalarPostMultiply(sfloat.One);
				Axis.Normalize();

				sVector2 normal = TransformB.TransformDirection(Axis);
				sVector2 pointB = TransformB.TransformPoint(LocalPoint);
				sVector2 pointA = TransformA.TransformPoint(localPointA);

				sfloat s = sVector2.Dot(pointA - pointB, normal);
				if (s < sfloat.Zero)
				{
					Axis = -Axis;
				}
			}
		}
#else
		internal void Initialize(SimplexCache cache, Shape shapeA, Transform transformA, Shape shapeB, Transform transformB)
		{
			ShapeA = shapeA;
			ShapeB = shapeB;
			int count = cache.Count;
			Box2DXDebug.Assert(0 < count && count < 3);

			if (count == 1)
			{
				FaceType = Type.Points;
				sVector2 localPointA = ShapeA.GetVertex(cache.IndexA[0]);
				sVector2 localPointB = ShapeB.GetVertex(cache.IndexB[0]);
				sVector2 pointA = transformA.TransformPoint(localPointA);
				sVector2 pointB = transformB.TransformPoint(localPointB);
				Axis = pointB - pointA;
				Axis.Normalize();
			}
			else if (cache.IndexB[0] == cache.IndexB[1])
			{
				// Two points on A and one on B
				FaceType = Type.FaceA;
				sVector2 localPointA1 = ShapeA.GetVertex(cache.IndexA[0]);
				sVector2 localPointA2 = ShapeA.GetVertex(cache.IndexA[1]);
				sVector2 localPointB = ShapeB.GetVertex(cache.IndexB[0]);
				LocalPoint = (sfloat)0.5f * (localPointA1 + localPointA2);
				Axis = (localPointA2 - localPointA1).CrossScalarPostMultiply(sfloat.One);
				Axis.Normalize();

				sVector2 normal = transformA.TransformDirection(Axis);
				sVector2 pointA = transformA.TransformPoint(LocalPoint);
				sVector2 pointB = transformB.TransformPoint(localPointB);

				sfloat s = sVector2.Dot(pointB - pointA, normal);
				if (s < sfloat.Zero)
				{
					Axis = -Axis;
				}
			}
			else
			{
				// Two points on B and one or two points on A.
				// We ignore the second point on A.
				FaceType = Type.FaceB;
				sVector2 localPointA = shapeA.GetVertex(cache.IndexA[0]);
				sVector2 localPointB1 = shapeB.GetVertex(cache.IndexB[0]);
				sVector2 localPointB2 = shapeB.GetVertex(cache.IndexB[1]);
				LocalPoint = (sfloat)0.5f * (localPointB1 + localPointB2);
				Axis = (localPointB2 - localPointB1).CrossScalarPostMultiply(sfloat.One);
				Axis.Normalize();

				sVector2 normal = transformB.TransformDirection(Axis);
				sVector2 pointB = transformB.TransformPoint(LocalPoint);
				sVector2 pointA = transformA.TransformPoint(localPointA);

				sfloat s = sVector2.Dot(pointA - pointB, normal);
				if (s < sfloat.Zero)
				{
					Axis = -Axis;
				}
			}
		}
#endif

		internal sfloat Evaluate(Transform TransformA, Transform TransformB)
		{
			switch (FaceType)
			{
				case Type.Points:
					{
						sVector2 axisA = TransformA.InverseTransformDirection(Axis);
						sVector2 axisB = TransformB.InverseTransformDirection(-Axis);
						sVector2 localPointA = ShapeA.GetSupportVertex(axisA);
						sVector2 localPointB = ShapeB.GetSupportVertex(axisB);
						sVector2 pointA = TransformA.TransformPoint(localPointA);
						sVector2 pointB = TransformB.TransformPoint(localPointB);
						sfloat separation = sVector2.Dot(pointB - pointA, Axis);
						return separation;
					}

				case Type.FaceA:
					{
						sVector2 normal = TransformA.TransformDirection(Axis);
						sVector2 pointA = TransformA.TransformPoint(LocalPoint);

						sVector2 axisB = TransformB.InverseTransformDirection(-normal);

						sVector2 localPointB = ShapeB.GetSupportVertex(axisB);
						sVector2 pointB = TransformB.TransformPoint(localPointB);

						sfloat separation = sVector2.Dot(pointB - pointA, normal);
						return separation;
					}

				case Type.FaceB:
					{
						sVector2 normal = TransformB.TransformDirection(Axis);
						sVector2 pointB = TransformB.TransformPoint(LocalPoint);

						sVector2 axisA = TransformA.InverseTransformDirection(-normal);

						sVector2 localPointA = ShapeA.GetSupportVertex(axisA);
						sVector2 pointA = TransformA.TransformPoint(localPointA);

						sfloat separation = sVector2.Dot(pointA - pointB, normal);
						return separation;
					}

				default:
					Box2DXDebug.Assert(false);
					return sfloat.Zero;
			}
		}

		internal Shape ShapeA;
		internal Shape ShapeB;
		internal Type FaceType;
		internal sVector2 LocalPoint;
		internal sVector2 Axis;
	}

	public partial class Collision
	{		
		public static int MaxToiIters;
		public static int MaxToiRootIters;

		// CCD via the secant method.
		/// <summary>
		/// Compute the time when two shapes begin to touch or touch at a closer distance.
		/// TOI considers the shape radii. It attempts to have the radii overlap by the tolerance.
		/// Iterations terminate with the overlap is within 0.5 * tolerance. The tolerance should be
		/// smaller than sum of the shape radii.
		/// Warning the sweeps must have the same time interval.
		/// </summary>
		/// <returns>
		/// The fraction between [0,1] in which the shapes first touch.
		/// fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.
		/// </returns>
		public static sfloat TimeOfImpact(TOIInput input, Shape shapeA, Shape shapeB)
		{
			Sweep sweepA = input.SweepA;
			Sweep sweepB = input.SweepB;

			Box2DXDebug.Assert(sweepA.T0 == sweepB.T0);
			Box2DXDebug.Assert(sfloat.One - sweepA.T0 > Common.Settings.FLT_EPSILON);

			sfloat radius = shapeA._radius + shapeB._radius;
			sfloat tolerance = input.Tolerance;

			sfloat alpha = sfloat.Zero;

			const int k_maxIterations = 1000;	// TODO_ERIN b2Settings
			int iter = 0;
			sfloat target = sfloat.Zero;

			// Prepare input for distance query.
			SimplexCache cache = new SimplexCache();
			cache.Count = 0;
			DistanceInput distanceInput;
			distanceInput.UseRadii = false;

			for (; ; )
			{
				Transform xfA, xfB;
				sweepA.GetTransform(out xfA, alpha);
				sweepB.GetTransform(out xfB, alpha);

				// Get the distance between shapes.
				distanceInput.TransformA = xfA;
				distanceInput.TransformB = xfB;
				DistanceOutput distanceOutput;
				Distance(out distanceOutput, ref cache, ref distanceInput, shapeA, shapeB);

				if (distanceOutput.Distance <= sfloat.Zero)
				{
					alpha = sfloat.One;
					break;
				}

				SeparationFunction fcn = new SeparationFunction();
#if ALLOWUNSAFE
				unsafe
				{
					fcn.Initialize(&cache, shapeA, xfA, shapeB, xfB);
				}
#else 
				fcn.Initialize(cache, shapeA, xfA, shapeB, xfB);
#endif 

				sfloat separation = fcn.Evaluate(xfA, xfB);
				if (separation <= sfloat.Zero)
				{
					alpha = sfloat.One;
					break;
				}

				if (iter == 0)
				{
					// Compute a reasonable target distance to give some breathing room
					// for conservative advancement. We take advantage of the shape radii
					// to create additional clearance.
					if (separation > radius)
					{
						target = Common.Math.Max(radius - tolerance, (sfloat)0.75f * radius);
					}
					else
					{
						target = Common.Math.Max(separation - tolerance, (sfloat)0.02f * radius);
					}
				}

				if (separation - target < (sfloat)0.5f * tolerance)
				{
					if (iter == 0)
					{
						alpha = sfloat.One;
						break;
					}

					break;
				}

#if _FALSE
				// Dump the curve seen by the root finder
				{
					const int32 N = 100;
					sfloat32 dx = sfloat.One / N;
					sfloat32 xs[N+1];
					sfloat32 fs[N+1];

					sfloat32 x = sfloat.Zero;

					for (int32 i = 0; i <= N; ++i)
					{
						sweepA.GetTransform(&xfA, x);
						sweepB.GetTransform(&xfB, x);
						sfloat32 f = fcn.Evaluate(xfA, xfB) - target;

						printf("%g %g\n", x, f);

						xs[i] = x;
						fs[i] = f;

						x += dx;
					}
				}
#endif

				// Compute 1D root of: f(x) - target = 0
				sfloat newAlpha = alpha;
				{
					sfloat x1 = alpha, x2 = sfloat.One;

					sfloat f1 = separation;

					sweepA.GetTransform(out xfA, x2);
					sweepB.GetTransform(out xfB, x2);
					sfloat f2 = fcn.Evaluate(xfA, xfB);

					// If intervals don't overlap at t2, then we are done.
					if (f2 >= target)
					{
						alpha = sfloat.One;
						break;
					}

					// Determine when intervals intersect.
					int rootIterCount = 0;
					for (; ; )
					{
						// Use a mix of the secant rule and bisection.
						sfloat x;
						if ((rootIterCount & 1) != 0)
						{
							// Secant rule to improve convergence.
							x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
						}
						else
						{
							// Bisection to guarantee progress.
							x = (sfloat)0.5f * (x1 + x2);
						}

						sweepA.GetTransform(out xfA, x);
						sweepB.GetTransform(out xfB, x);

						sfloat f = fcn.Evaluate(xfA, xfB);

						if (Common.Math.Abs(f - target) < (sfloat)0.025f * tolerance)
						{
							newAlpha = x;
							break;
						}

						// Ensure we continue to bracket the root.
						if (f > target)
						{
							x1 = x;
							f1 = f;
						}
						else
						{
							x2 = x;
							f2 = f;
						}

						++rootIterCount;

						Box2DXDebug.Assert(rootIterCount < 50);
					}

					MaxToiRootIters = Common.Math.Max(MaxToiRootIters, rootIterCount);
				}

				// Ensure significant advancement.
				if (newAlpha < (sfloat.One + (sfloat)100.0f * Common.Settings.FLT_EPSILON) * alpha)
				{
					break;
				}

				alpha = newAlpha;

				++iter;

				if (iter == k_maxIterations)
				{
					break;
				}
			}

			MaxToiIters = Common.Math.Max(MaxToiIters, iter);

			return alpha;
		}
	}
}