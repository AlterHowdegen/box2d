﻿/*
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

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;
using SoftFloat;
using UnityEngine;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// Mouse joint definition. This requires a world target point,
	/// tuning parameters, and the time step.
	/// </summary>
	public class MouseJointDef : JointDef
	{
		public MouseJointDef()
		{
			Type = JointType.MouseJoint;
			Target = sVector2.zero;
			MaxForce = sfloat.Zero;
			FrequencyHz = (sfloat)5.0f;
			DampingRatio = (sfloat)0.7f;
		}

		/// <summary>
		/// The initial world target point. This is assumed
		/// to coincide with the body anchor initially.
		/// </summary>
		public sVector2 Target;

		/// <summary>
		/// The maximum constraint force that can be exerted
		/// to move the candidate body. Usually you will express
		/// as some multiple of the weight (multiplier * mass * gravity).
		/// </summary>
		public sfloat MaxForce;

		/// <summary>
		/// The response speed.
		/// </summary>
		public sfloat FrequencyHz;

		/// <summary>
		/// The damping ratio. 0 = no damping, 1 = critical damping.
		/// </summary>
		public sfloat DampingRatio;
	}

	/// <summary>
	/// A mouse joint is used to make a point on a body track a
	/// specified world point. This a soft constraint with a maximum
	/// force. This allows the constraint to stretch and without
	/// applying huge forces.
	/// </summary>
	public class MouseJoint : Joint
	{
		public sVector2 _localAnchor;
		public sVector2 _target;
		public sVector2 _impulse;

		public Mat22 _mass;		// effective mass for point-to-point constraint.
		public sVector2 _C;				// position error
		public sfloat _maxForce;
		public sfloat _frequencyHz;
		public sfloat _dampingRatio;
		public sfloat _beta;
		public sfloat _gamma;

		public override sVector2 Anchor1
		{
			get { return _target; }
		}

		public override sVector2 Anchor2
		{
			get { return _body2.GetWorldPoint(_localAnchor); }
		}

		public override sVector2 GetReactionForce(sfloat inv_dt)
		{
			return inv_dt * _impulse;
		}

		public override sfloat GetReactionTorque(sfloat inv_dt)
		{
			return inv_dt * sfloat.Zero;
		}

		/// <summary>
		/// Use this to update the target point.
		/// </summary>
		public void SetTarget(sVector2 target)
		{
			if (_body2.IsSleeping())
			{
				_body2.WakeUp();
			}
			_target = target;
		}

		public MouseJoint(MouseJointDef def)
			: base(def)
		{
			_target = def.Target;
			_localAnchor = _body2.GetTransform().InverseTransformPoint(_target);

			_maxForce = def.MaxForce;
			_impulse = sVector2.zero;

			_frequencyHz = def.FrequencyHz;
			_dampingRatio = def.DampingRatio;

			_beta = sfloat.Zero;
			_gamma = sfloat.Zero;
		}

		internal override void InitVelocityConstraints(TimeStep step)
		{
			Body b = _body2;

			sfloat mass = b.GetMass();

			// Frequency
			sfloat omega = (sfloat)2.0f * Settings.Pi * _frequencyHz;

			// Damping coefficient
			sfloat d = (sfloat)2.0f * mass * _dampingRatio * omega;

			// Spring stiffness
			sfloat k = mass * (omega * omega);

			// magic formulas
			// gamma has units of inverse mass.
			// beta has units of inverse time.
			Box2DXDebug.Assert(d + step.Dt * k > Settings.FLT_EPSILON);
			_gamma = sfloat.One / (step.Dt * (d + step.Dt * k));
			_beta = step.Dt * k * _gamma;

			// Compute the effective mass matrix.
			sVector2 r = b.GetTransform().TransformDirection(_localAnchor - b.GetLocalCenter());

			// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
			//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
			//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
			sfloat invMass = b._invMass;
			sfloat invI = b._invI;

			Mat22 K1 = new Mat22();
			K1.Col1.x = invMass; K1.Col2.x = sfloat.Zero;
			K1.Col1.y = sfloat.Zero; K1.Col2.y = invMass;

			Mat22 K2 = new Mat22();
			K2.Col1.x = invI * r.y * r.y; K2.Col2.x = -invI * r.x * r.y;
			K2.Col1.y = -invI * r.x * r.y; K2.Col2.y = invI * r.x * r.x;

			Mat22 K = K1 + K2;
			K.Col1.x += _gamma;
			K.Col2.y += _gamma;

			_mass = K.GetInverse();

			_C = b._sweep.C + r - _target;

			// Cheat with some damping
			b._angularVelocity *= (sfloat)0.98f;

			// Warm starting.
			_impulse *= step.DtRatio;
			b._linearVelocity += invMass * _impulse;
			b._angularVelocity += invI * r.Cross(_impulse);
		}

		internal override void SolveVelocityConstraints(TimeStep step)
		{
			Body b = _body2;

			sVector2 r = b.GetTransform().TransformDirection(_localAnchor - b.GetLocalCenter());

			// Cdot = v + cross(w, r)
			sVector2 Cdot = b._linearVelocity + r.CrossScalarPreMultiply(b._angularVelocity);
			sVector2 impulse = _mass.Multiply(-(Cdot + _beta * _C + _gamma * _impulse));

			sVector2 oldImpulse = _impulse;
			_impulse += impulse;
			sfloat maxImpulse = step.Dt * _maxForce;
			if (_impulse.sqrMagnitude > maxImpulse * maxImpulse)
			{
				_impulse *= maxImpulse / _impulse.magnitude;
			}
			impulse = _impulse - oldImpulse;

			b._linearVelocity += b._invMass * impulse;
			b._angularVelocity += b._invI * r.Cross(impulse);
		}

		internal override bool SolvePositionConstraints(sfloat baumgarte)
		{
			return true;
		}
	}
}
