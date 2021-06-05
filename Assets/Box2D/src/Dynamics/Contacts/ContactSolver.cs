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

//#define B2_DEBUG_SOLVER

using System;
using Box2DX.Collision;
using Box2DX.Common;
using SoftFloat;
using UnityEngine;
using Transform = Box2DX.Common.Transform;

namespace Box2DX.Dynamics
{
#if ALLOWUNSAFE
	public struct ContactConstraintPoint
#else
	public class ContactConstraintPoint
#endif
	{
		public sVector2 LocalPoint;
		public sVector2 RA;
		public sVector2 RB;
		public sfloat NormalImpulse;
		public sfloat TangentImpulse;
		public sfloat NormalMass;
		public sfloat TangentMass;
		public sfloat EqualizedMass;
		public sfloat VelocityBias;
	}

	public class ContactConstraint
	{
		public ContactConstraintPoint[] Points = new ContactConstraintPoint[Settings.MaxManifoldPoints];
		public sVector2 LocalPlaneNormal;
		public sVector2 LocalPoint;
		public sVector2 Normal;
		public Mat22 NormalMass;
		public Mat22 K;
		public Body BodyA;
		public Body BodyB;
		public ManifoldType Type;
		public sfloat Radius;
		public sfloat Friction;
		public sfloat Restitution;
		public int PointCount;
		public Manifold Manifold;
		
#if !ALLOWUNSAFE
		public ContactConstraint()
		{
			for (int i = 0; i < Settings.MaxManifoldPoints; i++)
				Points[i] = new ContactConstraintPoint();
		}
#endif 
	}

	public class ContactSolver : IDisposable
	{
		public TimeStep _step;
		public ContactConstraint[] _constraints;
		public int _constraintCount;

		public ContactSolver(TimeStep step, Contact[] contacts, int contactCount)
		{
			_step = step;
			_constraintCount = contactCount;

			_constraints = new ContactConstraint[_constraintCount];
			for (int i = 0; i < _constraintCount; i++)
			{
				_constraints[i] = new ContactConstraint();
			}

			for (int i = 0; i < _constraintCount; ++i)
			{
				Contact contact = contacts[i];

				Fixture fixtureA = contact._fixtureA;
				Fixture fixtureB = contact._fixtureB;
				Shape shapeA = fixtureA.Shape;
				Shape shapeB = fixtureB.Shape;
				sfloat radiusA = shapeA._radius;
				sfloat radiusB = shapeB._radius;
				Body bodyA = fixtureA.Body;
				Body bodyB = fixtureB.Body;
				Manifold manifold = contact.Manifold;

				sfloat friction = Settings.MixFriction(fixtureA.Friction, fixtureB.Friction);
				sfloat restitution = Settings.MixRestitution(fixtureA.Restitution, fixtureB.Restitution);

				Box2DXDebug.Assert(manifold.PointCount > 0);

				WorldManifold worldManifold = new WorldManifold();
				worldManifold.Initialize(manifold, bodyA._xf, radiusA, bodyB._xf, radiusB);

				ContactConstraint cc = _constraints[i];
				cc.BodyA = bodyA;
				cc.BodyB = bodyB;
				cc.Manifold = manifold;
				cc.Normal = worldManifold.Normal;
				cc.PointCount = manifold.PointCount;
				cc.Friction = friction;
				cc.Restitution = restitution;

				cc.LocalPlaneNormal = manifold.LocalPlaneNormal;
				cc.LocalPoint = manifold.LocalPoint;
				cc.Radius = radiusA + radiusB;
				cc.Type = manifold.Type;

				ContactSolverSetup(manifold, worldManifold, cc);
			}
		}
		
#if ALLOWUNSAFE
		internal unsafe void ContactSolverSetup(Manifold manifold, WorldManifold worldManifold, ContactConstraint cc) 
		{
			// this is kind of yucky but we do know these were setup before entry to this method
			var bodyA = cc.BodyA;
			var bodyB = cc.BodyB;
				
			sVector2 vA = bodyA._linearVelocity;
			sVector2 vB = bodyB._linearVelocity;
			sfloat wA = bodyA._angularVelocity;
			sfloat wB = bodyB._angularVelocity;
			
			fixed (ContactConstraintPoint* ccPointsPtr = cc.Points)
			{
				for (int j = 0; j < cc.PointCount; ++j)
				{
					ManifoldPoint cp = manifold.Points[j];
					ContactConstraintPoint* ccp = &ccPointsPtr[j];

					ccp->NormalImpulse = cp.NormalImpulse;
					ccp->TangentImpulse = cp.TangentImpulse;

					ccp->LocalPoint = cp.LocalPoint;

					ccp->RA = worldManifold.Points[j] - bodyA._sweep.C;
					ccp->RB = worldManifold.Points[j] - bodyB._sweep.C;

					sfloat rnA = ccp->RA.Cross(cc.Normal);
					sfloat rnB = ccp->RB.Cross(cc.Normal);
					rnA *= rnA;
					rnB *= rnB;

					sfloat kNormal = bodyA._invMass + bodyB._invMass + bodyA._invI * rnA + bodyB._invI * rnB;

					Box2DXDebug.Assert(kNormal > Common.Settings.FLT_EPSILON);
					ccp->NormalMass = sfloat.One / kNormal;

					sfloat kEqualized = bodyA._mass * bodyA._invMass + bodyB._mass * bodyB._invMass;
					kEqualized += bodyA._mass * bodyA._invI * rnA + bodyB._mass * bodyB._invI * rnB;

					Box2DXDebug.Assert(kEqualized > Common.Settings.FLT_EPSILON);
					ccp->EqualizedMass = sfloat.One / kEqualized;

					sVector2 tangent = cc.Normal.CrossScalarPostMultiply(sfloat.One);

					sfloat rtA = ccp->RA.Cross(tangent);
					sfloat rtB = ccp->RB.Cross(tangent);
					rtA *= rtA;
					rtB *= rtB;

					sfloat kTangent = bodyA._invMass + bodyB._invMass + bodyA._invI * rtA + bodyB._invI * rtB;

					Box2DXDebug.Assert(kTangent > Common.Settings.FLT_EPSILON);
					ccp->TangentMass = sfloat.One / kTangent;

					// Setup a velocity bias for restitution.
					ccp->VelocityBias = sfloat.Zero;
					sfloat vRel = sVector2.Dot(cc.Normal, vB + ccp->RB.CrossScalarPreMultiply(wB) - vA - ccp->RA.CrossScalarPreMultiply(wA));
					if (vRel < -Common.Settings.VelocityThreshold)
					{
						ccp->VelocityBias = -cc.Restitution * vRel;
					}
				}

				// If we have two points, then prepare the block solver.
				if (cc.PointCount == 2)
				{
					ContactConstraintPoint* ccp1 = &ccPointsPtr[0];
					ContactConstraintPoint* ccp2 = &ccPointsPtr[1];

					sfloat invMassA = bodyA._invMass;
					sfloat invIA = bodyA._invI;
					sfloat invMassB = bodyB._invMass;
					sfloat invIB = bodyB._invI;

					sfloat rn1A = ccp1->RA.Cross(cc.Normal);
					sfloat rn1B = ccp1->RB.Cross(cc.Normal);
					sfloat rn2A = ccp2->RA.Cross(cc.Normal);
					sfloat rn2B = ccp2->RB.Cross(cc.Normal);

					sfloat k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
					sfloat k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
					sfloat k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

					// Ensure a reasonable condition number.
					const sfloat k_maxConditionNumber = 10sfloat.Zero;
					if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
					{
						// K is safe to invert.
						cc.K.Col1 = new sVector2(k11, k12);
						cc.K.Col2 = new sVector2(k12, k22);
						cc.NormalMass = cc.K.GetInverse();
					}
					else
					{
						// The constraints are redundant, just use one.
						// TODO_ERIN use deepest?
						cc.PointCount = 1;
					}
				}
			}
		}
#else
		internal void ContactSolverSetup(Manifold manifold, WorldManifold worldManifold, ContactConstraint cc) 
		{
			// this is kind of yucky but we do know these were setup before entry to this method
			var bodyA = cc.BodyA;
			var bodyB = cc.BodyB;
				
			sVector2 vA = bodyA._linearVelocity;
			sVector2 vB = bodyB._linearVelocity;
			sfloat wA = bodyA._angularVelocity;
			sfloat wB = bodyB._angularVelocity;
			
			ContactConstraintPoint[] ccPointsPtr = cc.Points;
			for (int j = 0; j < cc.PointCount; ++j)
			{
				ManifoldPoint cp = manifold.Points[j];
				ContactConstraintPoint ccp = ccPointsPtr[j];

				ccp.NormalImpulse = cp.NormalImpulse;
				ccp.TangentImpulse = cp.TangentImpulse;

				ccp.LocalPoint = cp.LocalPoint;
							
				ccp.RA = worldManifold.Points[j] - bodyA._sweep.C;
				ccp.RB = worldManifold.Points[j] - bodyB._sweep.C;

				sfloat rnA = ccp.RA.Cross(cc.Normal);
				sfloat rnB = ccp.RB.Cross(cc.Normal);
				rnA *= rnA;
				rnB *= rnB;

				sfloat kNormal = bodyA._invMass + bodyB._invMass + bodyA._invI * rnA + bodyB._invI * rnB;

				Box2DXDebug.Assert(kNormal > Common.Settings.FLT_EPSILON);
				ccp.NormalMass = sfloat.One / kNormal;

				sfloat kEqualized = bodyA._mass * bodyA._invMass + bodyB._mass * bodyB._invMass;
				kEqualized += bodyA._mass * bodyA._invI * rnA + bodyB._mass * bodyB._invI * rnB;

				Box2DXDebug.Assert(kEqualized > Common.Settings.FLT_EPSILON);
				ccp.EqualizedMass = sfloat.One / kEqualized;

				sVector2 tangent = cc.Normal.CrossScalarPostMultiply(sfloat.One);

				sfloat rtA = ccp.RA.Cross(tangent);
				sfloat rtB = ccp.RB.Cross(tangent);
				rtA *= rtA;
				rtB *= rtB;

				sfloat kTangent = bodyA._invMass + bodyB._invMass + bodyA._invI * rtA + bodyB._invI * rtB;

				Box2DXDebug.Assert(kTangent > Common.Settings.FLT_EPSILON);
				ccp.TangentMass = sfloat.One / kTangent;

				// Setup a velocity bias for restitution.
				ccp.VelocityBias = sfloat.Zero;
				sfloat vRel = sVector2.Dot(cc.Normal, vB + ccp.RB.CrossScalarPreMultiply(wB) - vA - ccp.RA.CrossScalarPreMultiply(wA));
				if (vRel < -Common.Settings.VelocityThreshold)
				{
					ccp.VelocityBias = -cc.Restitution * vRel;
				}
			}

			// If we have two points, then prepare the block solver.
			if (cc.PointCount == 2)
			{
				ContactConstraintPoint ccp1 = ccPointsPtr[0];
				ContactConstraintPoint ccp2 = ccPointsPtr[1];

				sfloat invMassA = bodyA._invMass;
				sfloat invIA = bodyA._invI;
				sfloat invMassB = bodyB._invMass;
				sfloat invIB = bodyB._invI;

				sfloat rn1A = ccp1.RA.Cross(cc.Normal);
				sfloat rn1B = ccp1.RB.Cross(cc.Normal);
				sfloat rn2A = ccp2.RA.Cross(cc.Normal);
				sfloat rn2B = ccp2.RB.Cross(cc.Normal);

				sfloat k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
				sfloat k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
				sfloat k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				sfloat k_maxConditionNumber = (sfloat)100.0f;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
				{
					// K is safe to invert.
					cc.K.Col1 = new sVector2(k11, k12);
					cc.K.Col2 = new sVector2(k12, k22);
					cc.NormalMass = cc.K.GetInverse();
				}
				else
				{
					// The constraints are redundant, just use one.
					// TODO_ERIN use deepest?
					cc.PointCount = 1;
				}
			}
		}
#endif

		public void Dispose()
		{
			_constraints = null;
		}
		

		public void InitVelocityConstraints(TimeStep step)
		{
#if ALLOWUNSAFE
			unsafe
			{
				// Warm start.
				for (int i = 0; i < _constraintCount; ++i)
				{
					ContactConstraint c = _constraints[i];

					Body bodyA = c.BodyA;
					Body bodyB = c.BodyB;
					sfloat invMassA = bodyA._invMass;
					sfloat invIA = bodyA._invI;
					sfloat invMassB = bodyB._invMass;
					sfloat invIB = bodyB._invI;
					sVector2 normal = c.Normal;
					sVector2 tangent = normal.CrossScalarPostMultiply(sfloat.One);

					fixed (ContactConstraintPoint* pointsPtr = c.Points)
					{
						if (step.WarmStarting)
						{
							for (int j = 0; j < c.PointCount; ++j)
							{
								ContactConstraintPoint* ccp = &pointsPtr[j];
								ccp->NormalImpulse *= step.DtRatio;
								ccp->TangentImpulse *= step.DtRatio;
								sVector2 P = ccp->NormalImpulse * normal + ccp->TangentImpulse * tangent;
								bodyA._angularVelocity -= invIA * ccp->RA.Cross(P);
								bodyA._linearVelocity -= invMassA * P;
								bodyB._angularVelocity += invIB * ccp->RB.Cross(P);
								bodyB._linearVelocity += invMassB * P;
							}
						}
						else
						{
							for (int j = 0; j < c.PointCount; ++j)
							{
								ContactConstraintPoint* ccp = &pointsPtr[j];
								ccp->NormalImpulse = sfloat.Zero;
								ccp->TangentImpulse = sfloat.Zero;
							}
						}
					}
				}
			}
#else
			// Warm start.
			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];

				Body bodyA = c.BodyA;
				Body bodyB = c.BodyB;
				sfloat invMassA = bodyA._invMass;
				sfloat invIA = bodyA._invI;
				sfloat invMassB = bodyB._invMass;
				sfloat invIB = bodyB._invI;
				sVector2 normal = c.Normal;
				sVector2 tangent = normal.CrossScalarPostMultiply(sfloat.One);

				ContactConstraintPoint[] points = c.Points;
				if (step.WarmStarting)
				{
					for (int j = 0; j < c.PointCount; ++j)
					{
						ContactConstraintPoint ccp = points[j];
						ccp.NormalImpulse *= step.DtRatio;
						ccp.TangentImpulse *= step.DtRatio;
						sVector2 P = ccp.NormalImpulse * normal + ccp.TangentImpulse * tangent;
						bodyA._angularVelocity -= invIA * ccp.RA.Cross(P);
						bodyA._linearVelocity -= invMassA * P;
						bodyB._angularVelocity += invIB * ccp.RB.Cross(P);
						bodyB._linearVelocity += invMassB * P;
					}
				}
				else
				{
					for (int j = 0; j < c.PointCount; ++j)
					{
						ContactConstraintPoint ccp = points[j];
						ccp.NormalImpulse = sfloat.Zero;
						ccp.TangentImpulse = sfloat.Zero;
					}
				}
			}
#endif
		}

		public void SolveVelocityConstraints()
		{
			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Body bodyA = c.BodyA;
				Body bodyB = c.BodyB;
				sfloat wA = bodyA._angularVelocity;
				sfloat wB = bodyB._angularVelocity;
				sVector2 vA = bodyA._linearVelocity;
				sVector2 vB = bodyB._linearVelocity;
				sfloat invMassA = bodyA._invMass;
				sfloat invIA = bodyA._invI;
				sfloat invMassB = bodyB._invMass;
				sfloat invIB = bodyB._invI;
				sVector2 normal = c.Normal;
				sVector2 tangent = normal.CrossScalarPostMultiply(sfloat.One);
				sfloat friction = c.Friction;

				Box2DXDebug.Assert(c.PointCount == 1 || c.PointCount == 2);

#if ALLOWUNSAFE
				unsafe
				{
					fixed (ContactConstraintPoint* pointsPtr = c.Points)
					{
						// Solve tangent constraints
						for (int j = 0; j < c.PointCount; ++j)
						{
							ContactConstraintPoint* ccp = &pointsPtr[j];

							// Relative velocity at contact
							sVector2 dv = vB + ccp->RB.CrossScalarPreMultiply(wB) - vA -  ccp->RA.CrossScalarPreMultiply(wA);

							// Compute tangent force
							sfloat vt = sVector2.Dot(dv, tangent);
							sfloat lambda = ccp->TangentMass * (-vt);

							// b2Clamp the accumulated force
							sfloat maxFriction = friction * ccp->NormalImpulse;
							sfloat newImpulse = Mathf.Clamp(ccp->TangentImpulse + lambda, -maxFriction, maxFriction);
							lambda = newImpulse - ccp->TangentImpulse;

							// Apply contact impulse
							sVector2 P = lambda * tangent;

							vA -= invMassA * P;
							wA -= invIA * ccp->RA.Cross(P);

							vB += invMassB * P;
							wB += invIB * ccp->RB.Cross(P);

							ccp->TangentImpulse = newImpulse;
						}

						// Solve normal constraints
						if (c.PointCount == 1)
						{
							ContactConstraintPoint ccp = c.Points[0];

							// Relative velocity at contact
							sVector2 dv = vB + ccp.RB.CrossScalarPreMultiply(wB) - vA - ccp.RA.CrossScalarPreMultiply(wA);

							// Compute normal impulse
							sfloat vn = sVector2.Dot(dv, normal);
							sfloat lambda = -ccp.NormalMass * (vn - ccp.VelocityBias);

							// Clamp the accumulated impulse
							sfloat newImpulse = Common.Math.Max(ccp.NormalImpulse + lambda, sfloat.Zero);
							lambda = newImpulse - ccp.NormalImpulse;

							// Apply contact impulse
							sVector2 P = lambda * normal;
							vA -= invMassA * P;
							wA -= invIA * ccp.RA.Cross(P);

							vB += invMassB * P;
							wB += invIB * ccp.RB.Cross(P);
							ccp.NormalImpulse = newImpulse;
						}
						else
						{
							// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
							// Build the mini LCP for this contact patch
							//
							// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
							//
							// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
							// b = vn_0 - velocityBias
							//
							// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
							// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
							// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
							// solution that satisfies the problem is chosen.
							// 
							// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
							// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
							//
							// Substitute:
							// 
							// x = x' - a
							// 
							// Plug into above equation:
							//
							// vn = A * x + b
							//    = A * (x' - a) + b
							//    = A * x' + b - A * a
							//    = A * x' + b'
							// b' = b - A * a;

							ContactConstraintPoint* cp1 = &pointsPtr[0];
							ContactConstraintPoint* cp2 = &pointsPtr[1];

							sVector2 a = new sVector2(cp1->NormalImpulse, cp2->NormalImpulse);
							Box2DXDebug.Assert(a.x >= sfloat.Zero && a.y >= sfloat.Zero);

							// Relative velocity at contact
							sVector2 dv1 = vB + cp1->RB.CrossScalarPreMultiply(wB) - vA - cp1->RA.CrossScalarPreMultiply(wA);
							sVector2 dv2 = vB + cp2->RB.CrossScalarPreMultiply(wB) - vA - cp2->RA.CrossScalarPreMultiply(wA);

							// Compute normal velocity
							sfloat vn1 = sVector2.Dot(dv1, normal);
							sfloat vn2 = sVector2.Dot(dv2, normal);

							sVector2 b = new sVector2(vn1 - cp1->VelocityBias, vn2 - cp2->VelocityBias);
							b -= c.K.Multiply(a);

							const sfloat k_errorTol = 1e-3f;
							//B2_NOT_USED(k_errorTol);

							for (; ; )
							{
								//
								// Case 1: vn = 0
								//
								// 0 = A * x' + b'
								//
								// Solve for x':
								//
								// x' = - inv(A) * b'
								//
								sVector2 x = -c.NormalMass.Multiply(b);

								if (x.x >= sfloat.Zero && x.y >= sfloat.Zero)
								{
									// Resubstitute for the incremental impulse
									sVector2 d = x - a;

									// Apply incremental impulse
									sVector2 P1 = d.x * normal;
									sVector2 P2 = d.y * normal;
									vA -= invMassA * (P1 + P2);
									wA -= invIA * (cp1->RA.Cross(P1) + cp2->RA.Cross(P2));

									vB += invMassB * (P1 + P2);
									wB += invIB * (cp1->RB.Cross(P1) + cp2->RB.Cross(P2));

									// Accumulate
									cp1->NormalImpulse = x.x;
									cp2->NormalImpulse = x.y;

#if DEBUG_SOLVER
									// Postconditions
									dv1 = vB + Vec2.Cross(wB, cp1->RB) - vA - Vec2.Cross(wA, cp1->RA);
									dv2 = vB + Vec2.Cross(wB, cp2->RB) - vA - Vec2.Cross(wA, cp2->RA);

									// Compute normal velocity
									vn1 = Vec2.Dot(dv1, normal);
									vn2 = Vec2.Dot(dv2, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
									Box2DXDebug.Assert(Common.Math.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
									break;
								}

								//
								// Case 2: vn1 = 0 and x2 = 0
								//
								//   0 = a11 * x1' + a12 * 0 + b1' 
								// vn2 = a21 * x1' + a22 * 0 + b2'
								//
								x.x = -cp1->NormalMass * b.x;
								x.y = sfloat.Zero;
								vn1 = sfloat.Zero;
								vn2 = c.K.Col1.y * x.x + b.y;

								if (x.x >= sfloat.Zero && vn2 >= sfloat.Zero)
								{
									// Resubstitute for the incremental impulse
									sVector2 d = x - a;

									// Apply incremental impulse
									sVector2 P1 = d.x * normal;
									sVector2 P2 = d.y * normal;
									vA -= invMassA * (P1 + P2);
									wA -= invIA * (cp1->RA.Cross(P1) + cp2->RA.Cross(P2));

									vB += invMassB * (P1 + P2);
									wB += invIB * (cp1->RB.Cross(P1) + cp2->RB.Cross(P2));

									// Accumulate
									cp1->NormalImpulse = x.x;
									cp2->NormalImpulse = x.y;

#if DEBUG_SOLVER
									// Postconditions
									dv1 = vB + Vec2.Cross(wB, cp1->RB) - vA - Vec2.Cross(wA, cp1->RA);

									// Compute normal velocity
									vn1 = Vec2.Dot(dv1, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
#endif
									break;
								}


								//
								// Case 3: w2 = 0 and x1 = 0
								//
								// vn1 = a11 * 0 + a12 * x2' + b1' 
								//   0 = a21 * 0 + a22 * x2' + b2'
								//
								x.x = sfloat.Zero;
								x.y = -cp2->NormalMass * b.y;
								vn1 = c.K.Col2.x * x.y + b.x;
								vn2 = sfloat.Zero;

								if (x.y >= sfloat.Zero && vn1 >= sfloat.Zero)
								{
									// Resubstitute for the incremental impulse
									sVector2 d = x - a;

									// Apply incremental impulse
									sVector2 P1 = d.x * normal;
									sVector2 P2 = d.y * normal;
									vA -= invMassA * (P1 + P2);
									wA -= invIA * (cp1->RA.Cross(P1) + cp2->RA.Cross(P2));

									vB += invMassB * (P1 + P2);
									wB += invIB * (cp1->RB.Cross(P1) + cp2->RB.Cross(P2));

									// Accumulate
									cp1->NormalImpulse = x.x;
									cp2->NormalImpulse = x.y;

#if DEBUG_SOLVER
									// Postconditions
									dv2 = vB + Vec2.Cross(wB, cp2->RB) - vA - Vec2.Cross(wA, cp2->RA);

									// Compute normal velocity
									vn2 = Vec2.Dot(dv2, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
									break;
								}

								//
								// Case 4: x1 = 0 and x2 = 0
								// 
								// vn1 = b1
								// vn2 = b2;
								x.x = sfloat.Zero;
								x.y = sfloat.Zero;
								vn1 = b.x;
								vn2 = b.y;

								if (vn1 >= sfloat.Zero && vn2 >= sfloat.Zero)
								{
									// Resubstitute for the incremental impulse
									sVector2 d = x - a;

									// Apply incremental impulse
									sVector2 P1 = d.x * normal;
									sVector2 P2 = d.y * normal;
									vA -= invMassA * (P1 + P2);
									wA -= invIA * (cp1->RA.Cross(P1) + cp2->RA.Cross(P2));

									vB += invMassB * (P1 + P2);
									wB += invIB * (cp1->RB.Cross(P1) + cp2->RB.Cross(P2));

									// Accumulate
									cp1->NormalImpulse = x.x;
									cp2->NormalImpulse = x.y;

									break;
								}

								// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
								break;
							}
						}

						bodyA._linearVelocity = vA;
						bodyA._angularVelocity = wA;
						bodyB._linearVelocity = vB;
						bodyB._angularVelocity = wB;
					}
				}
#else
				ContactConstraintPoint[] pointsPtr = c.Points;
			
				// Solve tangent constraints
				for (int j = 0; j < c.PointCount; ++j)
				{
					ContactConstraintPoint ccp = pointsPtr[j];

					// Relative velocity at contact
					sVector2 dv = vB + ccp.RB.CrossScalarPreMultiply(wB) - vA -  ccp.RA.CrossScalarPreMultiply(wA);

					// Compute tangent force
					sfloat vt = sVector2.Dot(dv, tangent);
					sfloat lambda = ccp.TangentMass * (-vt);

					// b2Clamp the accumulated force
					sfloat maxFriction = friction * ccp.NormalImpulse;
					sfloat newImpulse = libm.Clamp(ccp.TangentImpulse + lambda, -maxFriction, maxFriction);
					lambda = newImpulse - ccp.TangentImpulse;

					// Apply contact impulse
					sVector2 P = lambda * tangent;

					vA -= invMassA * P;
					wA -= invIA * ccp.RA.Cross(P);

					vB += invMassB * P;
					wB += invIB * ccp.RB.Cross(P);

					ccp.TangentImpulse = newImpulse;
				}

				// Solve normal constraints
				if (c.PointCount == 1)
				{
					ContactConstraintPoint ccp = c.Points[0];

					// Relative velocity at contact
					sVector2 dv = vB + ccp.RB.CrossScalarPreMultiply(wB) - vA - ccp.RA.CrossScalarPreMultiply(wA);

					// Compute normal impulse
					sfloat vn = sVector2.Dot(dv, normal);
					sfloat lambda = -ccp.NormalMass * (vn - ccp.VelocityBias);

					// Clamp the accumulated impulse
					sfloat newImpulse = Common.Math.Max(ccp.NormalImpulse + lambda, sfloat.Zero);
					lambda = newImpulse - ccp.NormalImpulse;

					// Apply contact impulse
					sVector2 P = lambda * normal;
					vA -= invMassA * P;
					wA -= invIA * ccp.RA.Cross(P);

					vB += invMassB * P;
					wB += invIB * ccp.RB.Cross(P);
					ccp.NormalImpulse = newImpulse;
				}
				else
				{
					// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
					// Build the mini LCP for this contact patch
					//
					// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
					//
					// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
					// b = vn_0 - velocityBias
					//
					// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
					// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
					// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
					// solution that satisfies the problem is chosen.
					// 
					// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
					// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
					//
					// Substitute:
					// 
					// x = x' - a
					// 
					// Plug into above equation:
					//
					// vn = A * x + b
					//    = A * (x' - a) + b
					//    = A * x' + b - A * a
					//    = A * x' + b'
					// b' = b - A * a;
					
					ContactConstraintPoint cp1 = pointsPtr[0];
					ContactConstraintPoint cp2 = pointsPtr[1];

					sVector2 a = new sVector2(cp1.NormalImpulse, cp2.NormalImpulse);
					Box2DXDebug.Assert(a.x >= sfloat.Zero && a.y >= sfloat.Zero);

					// Relative velocity at contact
					sVector2 dv1 = vB + cp1.RB.CrossScalarPreMultiply(wB) - vA - cp1.RA.CrossScalarPreMultiply(wA);
					sVector2 dv2 = vB + cp2.RB.CrossScalarPreMultiply(wB) - vA - cp2.RA.CrossScalarPreMultiply(wA);

					// Compute normal velocity
					sfloat vn1 = sVector2.Dot(dv1, normal);
					sfloat vn2 = sVector2.Dot(dv2, normal);

					sVector2 b = new sVector2(vn1 - cp1.VelocityBias, vn2 - cp2.VelocityBias);
					b -= c.K.Multiply(a);

					sfloat k_errorTol = (sfloat)1e-3f;
					//B2_NOT_USED(k_errorTol);

					for (; ; )
					{
						//
						// Case 1: vn = 0
						//
						// 0 = A * x' + b'
						//
						// Solve for x':
						//
						// x' = - inv(A) * b'
						//
						sVector2 x = -c.NormalMass.Multiply(b);

						if (x.x >= sfloat.Zero && x.y >= sfloat.Zero)
						{
							// Resubstitute for the incremental impulse
							sVector2 d = x - a;

							// Apply incremental impulse
							sVector2 P1 = d.x * normal;
							sVector2 P2 = d.y * normal;
							vA -= invMassA * (P1 + P2);
							wA -= invIA * (cp1.RA.Cross(P1) + cp2.RA.Cross(P2));

							vB += invMassB * (P1 + P2);
							wB += invIB * (cp1.RB.Cross(P1) + cp2.RB.Cross(P2));

							// Accumulate
							cp1.NormalImpulse = x.x;
							cp2.NormalImpulse = x.y;

#if DEBUG_SOLVER
							// Postconditions
							dv1 = vB + Vec2.Cross(wB, cp1->RB) - vA - Vec2.Cross(wA, cp1->RA);
							dv2 = vB + Vec2.Cross(wB, cp2->RB) - vA - Vec2.Cross(wA, cp2->RA);

							// Compute normal velocity
							vn1 = Vec2.Dot(dv1, normal);
							vn2 = Vec2.Dot(dv2, normal);

							Box2DXDebug.Assert(Common.Math.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
							Box2DXDebug.Assert(Common.Math.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
							break;
						}

						//
						// Case 2: vn1 = 0 and x2 = 0
						//
						//   0 = a11 * x1' + a12 * 0 + b1' 
						// vn2 = a21 * x1' + a22 * 0 + b2'
						//
						x.x = -cp1.NormalMass * b.x;
						x.y = sfloat.Zero;
						vn1 = sfloat.Zero;
						vn2 = c.K.Col1.y * x.x + b.y;

						if (x.x >= sfloat.Zero && vn2 >= sfloat.Zero)
						{
							// Resubstitute for the incremental impulse
							sVector2 d = x - a;

							// Apply incremental impulse
							sVector2 P1 = d.x * normal;
							sVector2 P2 = d.y * normal;
							vA -= invMassA * (P1 + P2);
							wA -= invIA * (cp1.RA.Cross(P1) + cp2.RA.Cross(P2));

							vB += invMassB * (P1 + P2);
							wB += invIB * (cp1.RB.Cross(P1) + cp2.RB.Cross(P2));

							// Accumulate
							cp1.NormalImpulse = x.x;
							cp2.NormalImpulse = x.y;

#if DEBUG_SOLVER
							// Postconditions
							dv1 = vB + Vec2.Cross(wB, cp1->RB) - vA - Vec2.Cross(wA, cp1->RA);

							// Compute normal velocity
							vn1 = Vec2.Dot(dv1, normal);

							Box2DXDebug.Assert(Common.Math.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
#endif
							break;
						}


						//
						// Case 3: w2 = 0 and x1 = 0
						//
						// vn1 = a11 * 0 + a12 * x2' + b1' 
						//   0 = a21 * 0 + a22 * x2' + b2'
						//
						x.x = sfloat.Zero;
						x.y = -cp2.NormalMass * b.y;
						vn1 = c.K.Col2.x * x.y + b.x;
						vn2 = sfloat.Zero;

						if (x.y >= sfloat.Zero && vn1 >= sfloat.Zero)
						{
							// Resubstitute for the incremental impulse
							sVector2 d = x - a;

							// Apply incremental impulse
							sVector2 P1 = d.x * normal;
							sVector2 P2 = d.y * normal;
							vA -= invMassA * (P1 + P2);
							wA -= invIA * (cp1.RA.Cross(P1) + cp2.RA.Cross(P2));

							vB += invMassB * (P1 + P2);
							wB += invIB * (cp1.RB.Cross(P1) + cp2.RB.Cross(P2));

							// Accumulate
							cp1.NormalImpulse = x.x;
							cp2.NormalImpulse = x.y;

#if DEBUG_SOLVER
							// Postconditions
							dv2 = vB + Vec2.Cross(wB, cp2->RB) - vA - Vec2.Cross(wA, cp2->RA);

							// Compute normal velocity
							vn2 = Vec2.Dot(dv2, normal);

							Box2DXDebug.Assert(Common.Math.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
							break;
						}

						//
						// Case 4: x1 = 0 and x2 = 0
						// 
						// vn1 = b1
						// vn2 = b2;
						x.x = sfloat.Zero;
						x.y = sfloat.Zero;
						vn1 = b.x;
						vn2 = b.y;

						if (vn1 >= sfloat.Zero && vn2 >= sfloat.Zero)
						{
							// Resubstitute for the incremental impulse
							sVector2 d = x - a;

							// Apply incremental impulse
							sVector2 P1 = d.x * normal;
							sVector2 P2 = d.y * normal;
							vA -= invMassA * (P1 + P2);
							wA -= invIA * (cp1.RA.Cross(P1) + cp2.RA.Cross(P2));

							vB += invMassB * (P1 + P2);
							wB += invIB * (cp1.RB.Cross(P1) + cp2.RB.Cross(P2));

							// Accumulate
							cp1.NormalImpulse = x.x;
							cp2.NormalImpulse = x.y;

							break;
						}

						// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
						break;
					}
				}

				bodyA._linearVelocity = vA;
				bodyA._angularVelocity = wA;
				bodyB._linearVelocity = vB;
				bodyB._angularVelocity = wB;
#endif // ALLOWUNSAFE
			}
		}

		public void FinalizeVelocityConstraints()
		{
			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Manifold m = c.Manifold;

				for (int j = 0; j < c.PointCount; ++j)
				{
					m.Points[j].NormalImpulse = c.Points[j].NormalImpulse;
					m.Points[j].TangentImpulse = c.Points[j].TangentImpulse;
				}
			}
		}

		internal class PositionSolverManifold
		{
			internal sVector2 Normal;
			internal sVector2[] Points = new sVector2[Settings.MaxManifoldPoints];
			internal sfloat[] Separations = new sfloat[Settings.MaxManifoldPoints];

			internal void Initialize(ContactConstraint cc)
			{
				Box2DXDebug.Assert(cc.PointCount > 0);

				switch (cc.Type)
				{
					case ManifoldType.Circles:
						{
							sVector2 pointA = cc.BodyA.GetWorldPoint(cc.LocalPoint);
							sVector2 pointB = cc.BodyB.GetWorldPoint(cc.Points[0].LocalPoint);
							if ((pointA - pointB).sqrMagnitude > (sfloat.Epsilon * sfloat.Epsilon))
							{
								Normal = pointB - pointA;
								Normal.Normalize();
							}
							else
							{
								Normal = new sVector2(sfloat.One, sfloat.Zero);
							}

							Points[0] = (sfloat)0.5f * (pointA + pointB);
							Separations[0] = sVector2.Dot(pointB - pointA, Normal) - cc.Radius;
						}
						break;

					case ManifoldType.FaceA:
						{
							Normal = cc.BodyA.GetWorldVector(cc.LocalPlaneNormal);
							sVector2 planePoint = cc.BodyA.GetWorldPoint(cc.LocalPoint);

							for (int i = 0; i < cc.PointCount; ++i)
							{
								sVector2 clipPoint = cc.BodyB.GetWorldPoint(cc.Points[i].LocalPoint);
								Separations[i] = sVector2.Dot(clipPoint - planePoint, Normal) - cc.Radius;
								Points[i] = clipPoint;
							}
						}
						break;

					case ManifoldType.FaceB:
						{
							Normal = cc.BodyB.GetWorldVector(cc.LocalPlaneNormal);
							sVector2 planePoint = cc.BodyB.GetWorldPoint(cc.LocalPoint);

							for (int i = 0; i < cc.PointCount; ++i)
							{
								sVector2 clipPoint = cc.BodyA.GetWorldPoint(cc.Points[i].LocalPoint);
								Separations[i] = sVector2.Dot(clipPoint - planePoint, Normal) - cc.Radius;
								Points[i] = clipPoint;
							}

							// Ensure normal points from A to B
							Normal = -Normal;
						}
						break;
				}
			}
		}

		private static PositionSolverManifold s_PositionSolverManifold = new PositionSolverManifold();

		public bool SolvePositionConstraints(sfloat baumgarte)
		{
			sfloat minSeparation = sfloat.Zero;

			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Body bodyA = c.BodyA;
				Body bodyB = c.BodyB;

				sfloat invMassA = bodyA._mass * bodyA._invMass;
				sfloat invIA = bodyA._mass * bodyA._invI;
				sfloat invMassB = bodyB._mass * bodyB._invMass;
				sfloat invIB = bodyB._mass * bodyB._invI;

				s_PositionSolverManifold.Initialize(c);
				sVector2 normal = s_PositionSolverManifold.Normal;

				// Solver normal constraints
				for (int j = 0; j < c.PointCount; ++j)
				{
					sVector2 point = s_PositionSolverManifold.Points[j];
					sfloat separation = s_PositionSolverManifold.Separations[j];

					sVector2 rA = point - bodyA._sweep.C;
					sVector2 rB = point - bodyB._sweep.C;

					// Track max constraint error.
					minSeparation = Common.Math.Min(minSeparation, separation);

					// Prevent large corrections and allow slop.
					sfloat C = baumgarte * libm.Clamp(separation + Settings.LinearSlop, -Settings.MaxLinearCorrection, sfloat.Zero);

					// Compute normal impulse
					sfloat impulse = -c.Points[j].EqualizedMass * C;

					sVector2 P = impulse * normal;

					bodyA._sweep.C -= invMassA * P;
					bodyA._sweep.A -= invIA * rA.Cross(P);
					bodyA.SynchronizeTransform();

					bodyB._sweep.C += invMassB * P;
					bodyB._sweep.A += invIB * rB.Cross(P);
					bodyB.SynchronizeTransform();
				}
			}

			// We can't expect minSpeparation >= -Settings.LinearSlop because we don't
			// push the separation above -Settings.LinearSlop.
			return minSeparation >= -(sfloat)1.5f * Settings.LinearSlop;
		}
	}
}
