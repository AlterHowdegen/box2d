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

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;
using UnityEngine;

namespace Box2DX.Dynamics
{
	using Box2DXMath = Box2DX.Common.Math;
	using SystemMath = System.Math;

	/// <summary>
	/// Revolute joint definition. This requires defining an
	/// anchor point where the bodies are joined. The definition
	/// uses local anchor points so that the initial configuration
	/// can violate the constraint slightly. You also need to
	/// specify the initial relative angle for joint limits. This
	/// helps when saving and loading a game.
	/// The local anchor points are measured from the body's origin
	/// rather than the center of mass because:
	/// 1. you might not know where the center of mass will be.
	/// 2. if you add/remove shapes from a body and recompute the mass,
	///    the joints will be broken.
	/// </summary>
	public class RevoluteJointDef : JointDef
	{
		public RevoluteJointDef()
		{
			Type = JointType.RevoluteJoint;
			LocalAnchor1 = new Vector2(0.0f, 0.0f);
			LocalAnchor2 = new Vector2(0.0f, 0.0f);
			ReferenceAngle = 0.0f;
			LowerAngle = 0.0f;
			UpperAngle = 0.0f;
			MaxMotorTorque = 0.0f;
			MotorSpeed = 0.0f;
			EnableLimit = false;
			EnableMotor = false;
		}

		/// <summary>
		/// Initialize the bodies, anchors, and reference angle using the world
		/// anchor.
		/// </summary>
		public void Initialize(Body body1, Body body2, Vector2 anchor)
		{
			Body1 = body1;
			Body2 = body2;
			LocalAnchor1 = body1.GetLocalPoint(anchor);
			LocalAnchor2 = body2.GetLocalPoint(anchor);
			ReferenceAngle = body2.GetAngle() - body1.GetAngle();
		}

		/// <summary>
		/// The local anchor point relative to body1's origin.
		/// </summary>
		public Vector2 LocalAnchor1;

		/// <summary>
		/// The local anchor point relative to body2's origin.
		/// </summary>
		public Vector2 LocalAnchor2;

		/// <summary>
		/// The body2 angle minus body1 angle in the reference state (radians).
		/// </summary>
		public float ReferenceAngle;

		/// <summary>
		/// A flag to enable joint limits.
		/// </summary>
		public bool EnableLimit;

		/// <summary>
		/// The lower angle for the joint limit (radians).
		/// </summary>
		public float LowerAngle;

		/// <summary>
		/// The upper angle for the joint limit (radians).
		/// </summary>
		public float UpperAngle;

		/// <summary>
		/// A flag to enable the joint motor.
		/// </summary>
		public bool EnableMotor;

		/// <summary>
		/// The desired motor speed. Usually in radians per second.
		/// </summary>
		public float MotorSpeed;

		/// <summary>
		/// The maximum motor torque used to achieve the desired motor speed.
		/// Usually in N-m.
		/// </summary>
		public float MaxMotorTorque;
	}

	/// <summary>
	/// A revolute joint constrains to bodies to share a common point while they
	/// are free to rotate about the point. The relative rotation about the shared
	/// point is the joint angle. You can limit the relative rotation with
	/// a joint limit that specifies a lower and upper angle. You can use a motor
	/// to drive the relative rotation about the shared point. A maximum motor torque
	/// is provided so that infinite forces are not generated.
	/// </summary>
	public class RevoluteJoint : Joint
	{
		public Vector2 _localAnchor1;	// relative
		public Vector2 _localAnchor2;
		public Vector3 _impulse;
		public float _motorImpulse;
		public Mat33 _mass; //effective mass for p2p constraint.
		public float _motorMass;	// effective mass for motor/limit angular constraint.

		public bool _enableMotor;
		public float _maxMotorTorque;
		public float _motorSpeed;

		public bool _enableLimit;
		public float _referenceAngle;
		public float _lowerAngle;
		public float _upperAngle;
		public LimitState _limitState;

		public override Vector2 Anchor1
		{
			get { return _body1.GetWorldPoint(_localAnchor1); }
		}

		public override Vector2 Anchor2
		{
			get { return _body2.GetWorldPoint(_localAnchor2); }
		}

		public override Vector2 GetReactionForce(float inv_dt)
		{
			Vector2 P = _impulse.ToVector2();
			return inv_dt * P;
		}

		public override float GetReactionTorque(float inv_dt)
		{
			return inv_dt * _impulse.z;
		}

		/// <summary>
		/// Get the current joint angle in radians.
		/// </summary>
		public float JointAngle
		{
			get
			{
				Body b1 = _body1;
				Body b2 = _body2;
				return b2._sweep.A - b1._sweep.A - _referenceAngle;
			}
		}


		/// <summary>
		/// Get the current joint angle speed in radians per second.
		/// </summary>
		public float JointSpeed
		{
			get
			{
				Body b1 = _body1;
				Body b2 = _body2;
				return b2._angularVelocity - b1._angularVelocity;
			}
		}

		/// <summary>
		/// Is the joint limit enabled?
		/// </summary>
		public bool IsLimitEnabled
		{
			get { return _enableLimit; }
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
		/// Get the lower joint limit in radians.
		/// </summary>
		public float LowerLimit
		{
			get { return _lowerAngle; }
		}

		/// <summary>
		/// Get the upper joint limit in radians.
		/// </summary>
		public float UpperLimit
		{
			get { return _upperAngle; }
		}

		/// <summary>
		/// Set the joint limits in radians.
		/// </summary>
		public void SetLimits(float lower, float upper)
		{
			Box2DXDebug.Assert(lower <= upper);
			_body1.WakeUp();
			_body2.WakeUp();
			_lowerAngle = lower;
			_upperAngle = upper;
		}

		/// <summary>
		/// Is the joint motor enabled?
		/// </summary>
		public bool IsMotorEnabled
		{
			get { return _enableMotor; }
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
		/// Get\Set the motor speed in radians per second.
		/// </summary>
		public float MotorSpeed
		{
			get { return _motorSpeed; }
			set
			{
				_body1.WakeUp();
				_body2.WakeUp();
				_motorSpeed = value;
			}
		}

		/// <summary>
		/// Set the maximum motor torque, usually in N-m.
		/// </summary>
		public void SetMaxMotorTorque(float torque)
		{
			_body1.WakeUp();
			_body2.WakeUp();
			_maxMotorTorque = torque;
		}

		/// <summary>
		/// Get the current motor torque, usually in N-m.
		/// </summary>
		public float MotorTorque
		{
			get { return _motorImpulse; }
		}

		public RevoluteJoint(RevoluteJointDef def)
			: base(def)
		{
			_localAnchor1 = def.LocalAnchor1;
			_localAnchor2 = def.LocalAnchor2;
			_referenceAngle = def.ReferenceAngle;

			_impulse = Vector3.zero;
			_motorImpulse = 0.0f;

			_lowerAngle = def.LowerAngle;
			_upperAngle = def.UpperAngle;
			_maxMotorTorque = def.MaxMotorTorque;
			_motorSpeed = def.MotorSpeed;
			_enableLimit = def.EnableLimit;
			_enableMotor = def.EnableMotor;
			_limitState = LimitState.InactiveLimit;
		}

		internal override void InitVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			if (_enableMotor || _enableLimit)
			{
				// You cannot create a rotation limit between bodies that
				// both have fixed rotation.
				Box2DXDebug.Assert(b1._invI > 0.0f || b2._invI > 0.0f);
			}

			// Compute the effective mass matrix.
			Vector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
			Vector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

			// J = [-I -r1_skew I r2_skew]
			//     [ 0       -1 0       1]
			// r_skew = [-ry; rx]

			// Matlab
			// K = [ m1+r1y^2*i1+m2+r2y^2*i2,  -r1y*i1*r1x-r2y*i2*r2x,          -r1y*i1-r2y*i2]
			//     [  -r1y*i1*r1x-r2y*i2*r2x, m1+r1x^2*i1+m2+r2x^2*i2,           r1x*i1+r2x*i2]
			//     [          -r1y*i1-r2y*i2,           r1x*i1+r2x*i2,                   i1+i2]

			float m1 = b1._invMass, m2 = b2._invMass;
			float i1 = b1._invI, i2 = b2._invI;

			_mass.Col1.x = m1 + m2 + r1.y * r1.y * i1 + r2.y * r2.y * i2;
			_mass.Col2.x = -r1.y * r1.x * i1 - r2.y * r2.x * i2;
			_mass.Col3.x = -r1.y * i1 - r2.y * i2;
			_mass.Col1.y = _mass.Col2.x;
			_mass.Col2.y = m1 + m2 + r1.x * r1.x * i1 + r2.x * r2.x * i2;
			_mass.Col3.y = r1.x * i1 + r2.x * i2;
			_mass.Col1.z = _mass.Col3.x;
			_mass.Col2.z = _mass.Col3.y;
			_mass.Col3.z = i1 + i2;

			_motorMass = 1.0f / (i1 + i2);

			if (_enableMotor == false)
			{
				_motorImpulse = 0.0f;
			}

			if (_enableLimit)
			{
				float jointAngle = b2._sweep.A - b1._sweep.A - _referenceAngle;
				if (Box2DXMath.Abs(_upperAngle - _lowerAngle) < 2.0f * Settings.AngularSlop)
				{
					_limitState = LimitState.EqualLimits;
				}
				else if (jointAngle <= _lowerAngle)
				{
					if (_limitState != LimitState.AtLowerLimit)
					{
						_impulse.z = 0.0f;
					}
					_limitState = LimitState.AtLowerLimit;
				}
				else if (jointAngle >= _upperAngle)
				{
					if (_limitState != LimitState.AtUpperLimit)
					{
						_impulse.z = 0.0f;
					}
					_limitState = LimitState.AtUpperLimit;
				}
				else
				{
					_limitState = LimitState.InactiveLimit;
					_impulse.z = 0.0f;
				}
			}
			else
			{
				_limitState = LimitState.InactiveLimit;
			}

			if (step.WarmStarting)
			{
				// Scale impulses to support a variable time step.
				_impulse *= step.DtRatio;
				_motorImpulse *= step.DtRatio;

				Vector2 P = _impulse.ToVector2();

				b1._linearVelocity -= m1 * P;
				b1._angularVelocity -= i1 * (r1.Cross(P) + _motorImpulse + _impulse.z);

				b2._linearVelocity += m2 * P;
				b2._angularVelocity += i2 * (r2.Cross(P) + _motorImpulse + _impulse.z);
			}
			else
			{
				_impulse = Vector3.zero;
				_motorImpulse = 0.0f;
			}
		}

		internal override void SolveVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			Vector2 v1 = b1._linearVelocity;
			float w1 = b1._angularVelocity;
			Vector2 v2 = b2._linearVelocity;
			float w2 = b2._angularVelocity;

			float m1 = b1._invMass, m2 = b2._invMass;
			float i1 = b1._invI, i2 = b2._invI;

			//Solve motor constraint.
			if (_enableMotor && _limitState != LimitState.EqualLimits)
			{
				float Cdot = w2 - w1 - _motorSpeed;
				float impulse = _motorMass * (-Cdot);
				float oldImpulse = _motorImpulse;
				float maxImpulse = step.Dt * _maxMotorTorque;
				_motorImpulse = Mathf.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
				impulse = _motorImpulse - oldImpulse;

				w1 -= i1 * impulse;
				w2 += i2 * impulse;
			}

			//Solve limit constraint.
			if (_enableLimit && _limitState != LimitState.InactiveLimit)
			{
				Vector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
				Vector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

				// Solve point-to-point constraint
				Vector2 Cdot1 = v2 + r2.CrossScalarPreMultiply(w2) - v1 - r1.CrossScalarPreMultiply(w1);
				float Cdot2 = w2 - w1;
				Vector3 Cdot = new Vector3(Cdot1.x, Cdot1.y, Cdot2);

				Vector3 impulse = _mass.Solve33(-Cdot);

				if (_limitState == LimitState.EqualLimits)
				{
					_impulse += impulse;
				}
				else if (_limitState == LimitState.AtLowerLimit)
				{
					float newImpulse = _impulse.z + impulse.z;
					if (newImpulse < 0.0f)
					{
						Vector2 reduced = _mass.Solve22(-Cdot1);
						impulse.x = reduced.x;
						impulse.y = reduced.y;
						impulse.z = -_impulse.z;
						_impulse.x += reduced.x;
						_impulse.y += reduced.y;
						_impulse.z = 0.0f;
					}
				}
				else if (_limitState == LimitState.AtUpperLimit)
				{
					float newImpulse = _impulse.z + impulse.z;
					if (newImpulse > 0.0f)
					{
						Vector2 reduced = _mass.Solve22(-Cdot1);
						impulse.x = reduced.x;
						impulse.y = reduced.y;
						impulse.z = -_impulse.z;
						_impulse.x += reduced.x;
						_impulse.y += reduced.y;
						_impulse.z = 0.0f;
					}
				}

				Vector2 P = impulse.ToVector2();

				v1 -= m1 * P;
				w1 -= i1 * (r1.Cross(P) + impulse.z);

				v2 += m2 * P;
				w2 += i2 * (r2.Cross(P) + impulse.z);
			}
			else
			{
				Vector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
				Vector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

				// Solve point-to-point constraint
				Vector2 Cdot = v2 + r2.CrossScalarPreMultiply(w2) - v1 - r1.CrossScalarPreMultiply(w1);
				Vector2 impulse = _mass.Solve22(-Cdot);

				_impulse.x += impulse.x;
				_impulse.y += impulse.y;

				v1 -= m1 * impulse;
				w1 -= i1 * r1.Cross(impulse);

				v2 += m2 * impulse;
				w2 += i2 * r2.Cross(impulse);
			}

			b1._linearVelocity = v1;
			b1._angularVelocity = w1;
			b2._linearVelocity = v2;
			b2._angularVelocity = w2;
		}

		internal override bool SolvePositionConstraints(float baumgarte)
		{
			// TODO_ERIN block solve with limit.

			Body b1 = _body1;
			Body b2 = _body2;

			float angularError = 0.0f;
			float positionError = 0.0f;

			// Solve angular limit constraint.
			if (_enableLimit && _limitState !=  LimitState.InactiveLimit)
			{
				float angle = b2._sweep.A - b1._sweep.A - _referenceAngle;
				float limitImpulse = 0.0f;

				if (_limitState == LimitState.EqualLimits)
				{
					// Prevent large angular corrections
					float C = Mathf.Clamp(angle, -Settings.MaxAngularCorrection, Settings.MaxAngularCorrection);
					limitImpulse = -_motorMass * C;
					angularError = Box2DXMath.Abs(C);
				}
				else if (_limitState == LimitState.AtLowerLimit)
				{
					float C = angle - _lowerAngle;
					angularError = -C;

					// Prevent large angular corrections and allow some slop.
					C = Mathf.Clamp(C + Settings.AngularSlop, -Settings.MaxAngularCorrection, 0.0f);
					limitImpulse = -_motorMass * C;
				}
				else if (_limitState == LimitState.AtUpperLimit)
				{
					float C = angle - _upperAngle;
					angularError = C;

					// Prevent large angular corrections and allow some slop.
					C = Mathf.Clamp(C - Settings.AngularSlop, 0.0f, Settings.MaxAngularCorrection);
					limitImpulse = -_motorMass * C;
				}

				b1._sweep.A -= b1._invI * limitImpulse;
				b2._sweep.A += b2._invI * limitImpulse;

				b1.SynchronizeTransform();
				b2.SynchronizeTransform();
			}

			// Solve point-to-point constraint.
			{
				Vector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
				Vector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

				Vector2 C = b2._sweep.C + r2 - b1._sweep.C - r1;
				positionError = C.magnitude;

				float invMass1 = b1._invMass, invMass2 = b2._invMass;
				float invI1 = b1._invI, invI2 = b2._invI;

				// Handle large detachment.
				float k_allowedStretch = 10.0f * Settings.LinearSlop;
				if (C.sqrMagnitude > k_allowedStretch * k_allowedStretch)
				{
					// Use a particle solution (no rotation).
					Vector2 u = C; u.Normalize();
					float k = invMass1 + invMass2;
					Box2DXDebug.Assert(k > Settings.FLT_EPSILON);
					float m = 1.0f / k;
					Vector2 impulse = m * (-C);
					float k_beta = 0.5f;
					b1._sweep.C -= k_beta * invMass1 * impulse;
					b2._sweep.C += k_beta * invMass2 * impulse;

					C = b2._sweep.C + r2 - b1._sweep.C - r1;
				}

				Mat22 K1 = new Mat22();
				K1.Col1.x = invMass1 + invMass2; K1.Col2.x = 0.0f;
				K1.Col1.y = 0.0f; K1.Col2.y = invMass1 + invMass2;

				Mat22 K2 = new Mat22();
				K2.Col1.x = invI1 * r1.y * r1.y; K2.Col2.x = -invI1 * r1.x * r1.y;
				K2.Col1.y = -invI1 * r1.x * r1.y; K2.Col2.y = invI1 * r1.x * r1.x;

				Mat22 K3 = new Mat22();
				K3.Col1.x = invI2 * r2.y * r2.y; K3.Col2.x = -invI2 * r2.x * r2.y;
				K3.Col1.y = -invI2 * r2.x * r2.y; K3.Col2.y = invI2 * r2.x * r2.x;

				Mat22 K = K1 + K2 + K3;
				Vector2 impulse_ = K.Solve(-C);

				b1._sweep.C -= b1._invMass * impulse_;
				b1._sweep.A -= b1._invI * r1.Cross(impulse_);

				b2._sweep.C += b2._invMass * impulse_;
				b2._sweep.A += b2._invI * r2.Cross(impulse_);

				b1.SynchronizeTransform();
				b2.SynchronizeTransform();
			}

			return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
		}
	}
}
