/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

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

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)


// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(2) = max(f2(2), 0)
// upper: f2(2) = min(f2(2), 0)
//
// Solve for correct f2(1)
// K(1,1) * f2(1) = -Cdot(1) - K(1,2) * f2(2) + K(1,1:2) * f1
//                = -Cdot(1) - K(1,2) * f2(2) + K(1,1) * f1(1) + K(1,2) * f1(2)
// K(1,1) * f2(1) = -Cdot(1) - K(1,2) * (f2(2) - f1(2)) + K(1,1) * f1(1)
// f2(1) = invK(1,1) * (-Cdot(1) - K(1,2) * (f2(2) - f1(2))) + f1(1)
//
// Now compute impulse to be applied:
// df = f2 - f1

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;
using SoftFloat;
using UnityEngine;

using Transform = Box2DX.Common.Transform;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// Line joint definition. This requires defining a line of
	/// motion using an axis and an anchor point. The definition uses local
	/// anchor points and a local axis so that the initial configuration
	/// can violate the constraint slightly. The joint translation is zero
	/// when the local anchor points coincide in world space. Using local
	/// anchors and a local axis helps when saving and loading a game.
	/// </summary>
	public class LineJointDef : JointDef
	{
		public LineJointDef()
		{
			Type = JointType.LineJoint;
			localAnchor1 = sVector2.zero;
			localAnchor2 = sVector2.zero;
			localAxis1 = new sVector2(sfloat.One, sfloat.Zero);
			enableLimit = false;
			lowerTranslation = sfloat.Zero;
			upperTranslation = sfloat.Zero;
			enableMotor = false;
			maxMotorForce = sfloat.Zero;
			motorSpeed = sfloat.Zero;
		}

		/// <summary>
		/// Initialize the bodies, anchors, axis, and reference angle using the world
		/// anchor and world axis.
		/// </summary>
		public void Initialize(Body body1, Body body2, sVector2 anchor, sVector2 axis)
		{
			Body1 = body1;
			Body2 = body2;
			localAnchor1 = body1.GetLocalPoint(anchor);
			localAnchor2 = body2.GetLocalPoint(anchor);
			localAxis1 = body1.GetLocalVector(axis);
		}

		/// <summary>
		/// The local anchor point relative to body1's origin.
		/// </summary>
		public sVector2 localAnchor1;

		/// <summary>
		/// The local anchor point relative to body2's origin.
		/// </summary>
		public sVector2 localAnchor2;

		/// <summary>
		/// The local translation axis in body1.
		/// </summary>
		public sVector2 localAxis1;

		/// <summary>
		/// Enable/disable the joint limit.
		/// </summary>
		public bool enableLimit;

		/// <summary>
		/// The lower translation limit, usually in meters.
		/// </summary>
		public sfloat lowerTranslation;

		/// <summary>
		/// The upper translation limit, usually in meters.
		/// </summary>
		public sfloat upperTranslation;

		/// <summary>
		/// Enable/disable the joint motor.
		/// </summary>
		public bool enableMotor;

		/// <summary>
		/// The maximum motor torque, usually in N-m.
		/// </summary>
		public sfloat maxMotorForce;

		/// <summary>
		/// The desired motor speed in radians per second.
		/// </summary>
		public sfloat motorSpeed;
	}

	/// <summary>
	/// A line joint. This joint provides one degree of freedom: translation
	/// along an axis fixed in body1. You can use a joint limit to restrict
	/// the range of motion and a joint motor to drive the motion or to
	/// model joint friction.
	/// </summary>
	public class LineJoint : Joint
	{
		public sVector2 _localAnchor1;
		public sVector2 _localAnchor2;
		public sVector2 _localXAxis1;
		public sVector2 _localYAxis1;

		public sVector2 _axis, _perp;
		public sfloat _s1, _s2;
		public sfloat _a1, _a2;

		public Mat22 _K;
		public sVector2 _impulse;

		public sfloat _motorMass;			// effective mass for motor/limit translational constraint.
		public sfloat _motorImpulse;

		public sfloat _lowerTranslation;
		public sfloat _upperTranslation;
		public sfloat _maxMotorForce;
		public sfloat _motorSpeed;

		public bool _enableLimit;
		public bool _enableMotor;
		public LimitState _limitState;

		public LineJoint(LineJointDef def)
			: base(def)
		{
			_localAnchor1 = def.localAnchor1;
			_localAnchor2 = def.localAnchor2;
			_localXAxis1 = def.localAxis1;
			_localYAxis1 = _localXAxis1.CrossScalarPreMultiply(sfloat.One);
			
			_impulse = sVector2.zero;
			_motorMass = sfloat.Zero;
			_motorImpulse = sfloat.Zero;

			_lowerTranslation = def.lowerTranslation;
			_upperTranslation = def.upperTranslation;
			_maxMotorForce = Settings.FORCE_INV_SCALE(def.maxMotorForce);
			_motorSpeed = def.motorSpeed;
			_enableLimit = def.enableLimit;
			_enableMotor = def.enableMotor;
			_limitState = LimitState.InactiveLimit;

			_axis = sVector2.zero;
			_perp = sVector2.zero;
		}

		public override sVector2 Anchor1
		{
			get { return _body1.GetWorldPoint(_localAnchor1); }
		}

		public override sVector2 Anchor2
		{
			get { return _body2.GetWorldPoint(_localAnchor2); }
		}

		public override sVector2 GetReactionForce(sfloat inv_dt)
		{
			return inv_dt * (_impulse.x * _perp + (_motorImpulse + _impulse.y) * _axis);
		}

		public override sfloat GetReactionTorque(sfloat inv_dt)
		{
			return sfloat.Zero;
		}

		/// <summary>
		/// Get the current joint translation, usually in meters.
		/// </summary>
		public sfloat GetJointTranslation()
		{
			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 p1 = b1.GetWorldPoint(_localAnchor1);
			sVector2 p2 = b2.GetWorldPoint(_localAnchor2);
			sVector2 d = p2 - p1;
			sVector2 axis = b1.GetWorldVector(_localXAxis1);

			sfloat translation = sVector2.Dot(d, axis);
			return translation;
		}

		/// <summary>
		/// Get the current joint translation speed, usually in meters per second.
		/// </summary>
		public sfloat GetJointSpeed()
		{
			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
			sVector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());
			sVector2 p1 = b1._sweep.C + r1;
			sVector2 p2 = b2._sweep.C + r2;
			sVector2 d = p2 - p1;
			sVector2 axis = b1.GetWorldVector(_localXAxis1);

			sVector2 v1 = b1._linearVelocity;
			sVector2 v2 = b2._linearVelocity;
			sfloat w1 = b1._angularVelocity;
			sfloat w2 = b2._angularVelocity;

			sfloat speed = sVector2.Dot(d, axis.CrossScalarPreMultiply(w1)) + sVector2.Dot(axis, v2 + r2.CrossScalarPreMultiply(w2) - v1 - r1.CrossScalarPreMultiply(w1));
			return speed;
		}

		/// <summary>
		/// Is the joint limit enabled?
		/// </summary>
		public bool IsLimitEnabled()
		{
			return _enableLimit;
		}

		/// <summary>
		/// Enable/disable the joint limit.
		/// </summary>
		public void EnableLimit(bool flag)
		{
			_body1.WakeUp();
			_body2.WakeUp();
			_enableLimit = flag;
		}

		/// <summary>
		/// Get the lower joint limit, usually in meters.
		/// </summary>
		public sfloat GetLowerLimit()
		{
			return _lowerTranslation;
		}

		/// <summary>
		/// Get the upper joint limit, usually in meters.
		/// </summary>
		public sfloat GetUpperLimit()
		{
			return _upperTranslation;
		}

		/// <summary>
		/// Set the joint limits, usually in meters.
		/// </summary>
		public void SetLimits(sfloat lower, sfloat upper)
		{
			Box2DXDebug.Assert(lower <= upper);
			_body1.WakeUp();
			_body2.WakeUp();
			_lowerTranslation = lower;
			_upperTranslation = upper;
		}

		/// <summary>
		/// Is the joint motor enabled?
		/// </summary>
		public bool IsMotorEnabled()
		{
			return _enableMotor;
		}

		/// <summary>
		/// Enable/disable the joint motor.
		/// </summary>
		public void EnableMotor(bool flag)
		{
			_body1.WakeUp();
			_body2.WakeUp();
			_enableMotor = flag;
		}

		/// <summary>
		/// Set the motor speed, usually in meters per second.
		/// </summary>
		public void SetMotorSpeed(sfloat speed)
		{
			_body1.WakeUp();
			_body2.WakeUp();
			_motorSpeed = speed;
		}

		/// <summary>
		/// Set the maximum motor force, usually in N.
		/// </summary>
		public void SetMaxMotorForce(sfloat force)
		{
			_body1.WakeUp();
			_body2.WakeUp();
			_maxMotorForce = Settings.FORCE_SCALE(sfloat.One) * force;
		}

		/// <summary>
		/// Get the current motor force, usually in N.
		/// </summary>
		public sfloat GetMotorForce()
		{
			return _motorImpulse;
		}

		/// <summary>
		/// Get the motor speed, usually in meters per second.
		/// </summary>
		public sfloat GetMotorSpeed()
		{
			return _motorSpeed;
		}

		internal override void InitVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			_localCenter1 = b1.GetLocalCenter();
			_localCenter2 = b2.GetLocalCenter();

			Transform xf1 = b1.GetTransform();
			Transform xf2 = b2.GetTransform();

			// Compute the effective masses.
			sVector2 r1 = xf1.TransformDirection(_localAnchor1 - _localCenter1);
			sVector2 r2 = xf2.TransformDirection(_localAnchor2 - _localCenter2);
			sVector2 d = b2._sweep.C + r2 - b1._sweep.C - r1;

			_invMass1 = b1._invMass;
			_invI1 = b1._invI;
			_invMass2 = b2._invMass;
			_invI2 = b2._invI;

			// Compute motor Jacobian and effective mass.
			{
				_axis = xf1.TransformDirection(_localXAxis1);
				_a1 = (d + r1).Cross(_axis);
				_a2 = r2.Cross(_axis);

				_motorMass = _invMass1 + _invMass2 + _invI1 * _a1 * _a1 + _invI2 * _a2 * _a2;
				Box2DXDebug.Assert(_motorMass > Settings.FLT_EPSILON);
				_motorMass = sfloat.One / _motorMass;
			}

			// Prismatic constraint.
			{
				_perp = xf1.TransformDirection(_localYAxis1);

				_s1 = (d + r1).Cross(_perp);
				_s2 = r2.Cross(_perp);

				sfloat m1 = _invMass1, m2 = _invMass2;
				sfloat i1 = _invI1, i2 = _invI2;

				sfloat k11 = m1 + m2 + i1 * _s1 * _s1 + i2 * _s2 * _s2;
				sfloat k12 = i1 * _s1 * _a1 + i2 * _s2 * _a2;
				sfloat k22 = m1 + m2 + i1 * _a1 * _a1 + i2 * _a2 * _a2;

				_K.Col1 = new sVector2(k11, k12);
				_K.Col2 = new sVector2(k12, k22);
			}

			// Compute motor and limit terms.
			if (_enableLimit)
			{
				sfloat jointTranslation = sVector2.Dot(_axis, d);
				if (Box2DX.Common.Math.Abs(_upperTranslation - _lowerTranslation) < (sfloat)2.0f * Settings.LinearSlop)
				{
					_limitState = LimitState.EqualLimits;
				}
				else if (jointTranslation <= _lowerTranslation)
				{
					if (_limitState != LimitState.AtLowerLimit)
					{
						_limitState = LimitState.AtLowerLimit;
						_impulse.y = sfloat.Zero;
					}
				}
				else if (jointTranslation >= _upperTranslation)
				{
					if (_limitState != LimitState.AtUpperLimit)
					{
						_limitState = LimitState.AtUpperLimit;
						_impulse.y = sfloat.Zero;
					}
				}
				else
				{
					_limitState = LimitState.InactiveLimit;
					_impulse.y = sfloat.Zero;
				}
			}
			else
			{
				_limitState = LimitState.InactiveLimit;
			}

			if (_enableMotor == false)
			{
				_motorImpulse = sfloat.Zero;
			}

			if (step.WarmStarting)
			{
				// Account for variable time step.
				_impulse *= step.DtRatio;
				_motorImpulse *= step.DtRatio;

				sVector2 P = _impulse.x * _perp + (_motorImpulse + _impulse.y) * _axis;
				sfloat L1 = _impulse.x * _s1 + (_motorImpulse + _impulse.y) * _a1;
				sfloat L2 = _impulse.x * _s2 + (_motorImpulse + _impulse.y) * _a2;

				b1._linearVelocity -= _invMass1 * P;
				b1._angularVelocity -= _invI1 * L1;

				b2._linearVelocity += _invMass2 * P;
				b2._angularVelocity += _invI2 * L2;
			}
			else
			{
				_impulse = sVector2.zero;
				_motorImpulse = sfloat.Zero;
			}
		}
		
		internal override void SolveVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 v1 = b1._linearVelocity;
			sfloat w1 = b1._angularVelocity;
			sVector2 v2 = b2._linearVelocity;
			sfloat w2 = b2._angularVelocity;

			// Solve linear motor constraint.
			if (_enableMotor && _limitState != LimitState.EqualLimits)
			{
				sfloat Cdot = sVector2.Dot(_axis, v2 - v1) + _a2 * w2 - _a1 * w1;
				sfloat impulse = _motorMass * (_motorSpeed - Cdot);
				sfloat oldImpulse = _motorImpulse;
				sfloat maxImpulse = step.Dt * _maxMotorForce;
				_motorImpulse = libm.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
				impulse = _motorImpulse - oldImpulse;

				sVector2 P = impulse * _axis;
				sfloat L1 = impulse * _a1;
				sfloat L2 = impulse * _a2;

				v1 -= _invMass1 * P;
				w1 -= _invI1 * L1;

				v2 += _invMass2 * P;
				w2 += _invI2 * L2;
			}

			sfloat Cdot1 = sVector2.Dot(_perp, v2 - v1) + _s2 * w2 - _s1 * w1;

			if (_enableLimit && _limitState != LimitState.InactiveLimit)
			{
				// Solve prismatic and limit constraint in block form.
				sfloat Cdot2 = sVector2.Dot(_axis, v2 - v1) + _a2 * w2 - _a1 * w1;
				sVector2 Cdot = new sVector2(Cdot1, Cdot2);

				sVector2 f1 = _impulse;
				sVector2 df =  _K.Solve(-Cdot);
				_impulse += df;

				if (_limitState == LimitState.AtLowerLimit)
				{
					_impulse.y = sfloat.Max(_impulse.y, sfloat.Zero);
				}
				else if (_limitState == LimitState.AtUpperLimit)
				{
					_impulse.y = sfloat.Min(_impulse.y, sfloat.Zero);
				}

				// f2(1) = invK(1,1) * (-Cdot(1) - K(1,2) * (f2(2) - f1(2))) + f1(1)
				sfloat b = -Cdot1 - (_impulse.y - f1.y) * _K.Col2.x;
				sfloat f2r = b / _K.Col1.x + f1.x;
				_impulse.x = f2r;

				df = _impulse - f1;

				sVector2 P = df.x * _perp + df.y * _axis;
				sfloat L1 = df.x * _s1 + df.y * _a1;
				sfloat L2 = df.x * _s2 + df.y * _a2;

				v1 -= _invMass1 * P;
				w1 -= _invI1 * L1;

				v2 += _invMass2 * P;
				w2 += _invI2 * L2;
			}
			else
			{
				// Limit is inactive, just solve the prismatic constraint in block form.
				sfloat df = (-Cdot1) / _K.Col1.x;
				_impulse.x += df;

				sVector2 P = df * _perp;
				sfloat L1 = df * _s1;
				sfloat L2 = df * _s2;

				v1 -= _invMass1 * P;
				w1 -= _invI1 * L1;

				v2 += _invMass2 * P;
				w2 += _invI2 * L2;
			}

			b1._linearVelocity = v1;
			b1._angularVelocity = w1;
			b2._linearVelocity = v2;
			b2._angularVelocity = w2;
		}

		internal override bool SolvePositionConstraints(sfloat baumgarte)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 c1 = b1._sweep.C;
			sfloat a1 = b1._sweep.A;

			sVector2 c2 = b2._sweep.C;
			sfloat a2 = b2._sweep.A;

			// Solve linear limit constraint.
			sfloat linearError = sfloat.Zero, angularError = sfloat.Zero;
			bool active = false;
			sfloat C2 = sfloat.Zero;

			Mat22 R1 = new Mat22(a1), R2 = new Mat22(a2);

			sVector2 r1 = R1.Multiply(_localAnchor1 - _localCenter1);
			sVector2 r2 = R2.Multiply(_localAnchor2 - _localCenter2);
			sVector2 d = c2 + r2 - c1 - r1;

			if (_enableLimit)
			{
				_axis = R1.Multiply(_localXAxis1);

				_a1 = (d + r1).Cross(_axis);
				_a2 = r2.Cross(_axis);

				sfloat translation = sVector2.Dot(_axis, d);
				if (sfloat.Abs(_upperTranslation - _lowerTranslation) < (sfloat)2.0f * Settings.LinearSlop)
				{
					// Prevent large angular corrections
					C2 = libm.Clamp(translation, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);
					linearError = Box2DX.Common.Math.Abs(translation);
					active = true;
				}
				else if (translation <= _lowerTranslation)
				{
					// Prevent large linear corrections and allow some slop.
					C2 = libm.Clamp(translation - _lowerTranslation + Settings.LinearSlop, -Settings.MaxLinearCorrection, sfloat.Zero);
					linearError = _lowerTranslation - translation;
					active = true;
				}
				else if (translation >= _upperTranslation)
				{
					// Prevent large linear corrections and allow some slop.
					C2 = libm.Clamp(translation - _upperTranslation - Settings.LinearSlop, sfloat.Zero, Settings.MaxLinearCorrection);
					linearError = translation - _upperTranslation;
					active = true;
				}
			}

			_perp = R1.Multiply(_localYAxis1);

			_s1 = (d + r1).Cross(_perp);
			_s2 = r2.Cross(_perp);

			sVector2 impulse;
			sfloat C1;
			C1 = sVector2.Dot(_perp, d);

			linearError = Box2DX.Common.Math.Max(linearError, Box2DX.Common.Math.Abs(C1));
			angularError = sfloat.Zero;

			if (active)
			{
				sfloat m1 = _invMass1, m2 = _invMass2;
				sfloat i1 = _invI1, i2 = _invI2;

				sfloat k11 = m1 + m2 + i1 * _s1 * _s1 + i2 * _s2 * _s2;
				sfloat k12 = i1 * _s1 * _a1 + i2 * _s2 * _a2;
				sfloat k22 = m1 + m2 + i1 * _a1 * _a1 + i2 * _a2 * _a2;

				_K.Col1 = new sVector2(k11, k12);
				_K.Col2 = new sVector2(k12, k22);

				sVector2 C = new sVector2();
				C.x = C1;
				C.y = C2;

				impulse = _K.Solve(-C);
			}
			else
			{
				sfloat m1 = _invMass1, m2 = _invMass2;
				sfloat i1 = _invI1, i2 = _invI2;

				sfloat k11 = m1 + m2 + i1 * _s1 * _s1 + i2 * _s2 * _s2;

				sfloat impulse1 = (-C1) / k11;
				impulse.x = impulse1;
				impulse.y = sfloat.Zero;
			}

			sVector2 P = impulse.x * _perp + impulse.y * _axis;
			sfloat L1 = impulse.x * _s1 + impulse.y * _a1;
			sfloat L2 = impulse.x * _s2 + impulse.y * _a2;

			c1 -= _invMass1 * P;
			a1 -= _invI1 * L1;
			c2 += _invMass2 * P;
			a2 += _invI2 * L2;

			// TODO_ERIN remove need for this.
			b1._sweep.C = c1;
			b1._sweep.A = a1;
			b2._sweep.C = c2;
			b2._sweep.A = a2;
			b1.SynchronizeTransform();
			b2.SynchronizeTransform();

			return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
		}
	}
}