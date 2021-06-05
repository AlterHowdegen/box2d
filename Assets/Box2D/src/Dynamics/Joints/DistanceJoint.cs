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

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;
using SoftFloat;
using UnityEngine;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// Distance joint definition. This requires defining an
	/// anchor point on both bodies and the non-zero length of the
	/// distance joint. The definition uses local anchor points
	/// so that the initial configuration can violate the constraint
	/// slightly. This helps when saving and loading a game.
	/// @warning Do not use a zero or short length.
	/// </summary>
	public class DistanceJointDef : JointDef
	{
		public DistanceJointDef()
		{
			Type = JointType.DistanceJoint;
			LocalAnchor1 = sVector2.zero;
			LocalAnchor2 = sVector2.zero;
			Length = sfloat.One;
			FrequencyHz = sfloat.Zero;
			DampingRatio = sfloat.Zero;
		}

		/// <summary>
		/// Initialize the bodies, anchors, and length using the world anchors.
		/// </summary>
		public void Initialize(Body body1, Body body2, sVector2 anchor1, sVector2 anchor2)
		{
			Body1 = body1;
			Body2 = body2;
			LocalAnchor1 = body1.GetLocalPoint(anchor1);
			LocalAnchor2 = body2.GetLocalPoint(anchor2);
			sVector2 d = anchor2 - anchor1;
			Length = d.magnitude;
		}

		/// <summary>
		/// The local anchor point relative to body1's origin.
		/// </summary>
		public sVector2 LocalAnchor1;

		/// <summary>
		/// The local anchor point relative to body2's origin.
		/// </summary>
		public sVector2 LocalAnchor2;

		/// <summary>
		/// The equilibrium length between the anchor points.
		/// </summary>
		public sfloat Length;

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
	/// A distance joint constrains two points on two bodies
	/// to remain at a fixed distance from each other. You can view
	/// this as a massless, rigid rod.
	/// </summary>
	public class DistanceJoint : Joint
	{
		public sVector2 _localAnchor1;
		public sVector2 _localAnchor2;
		public sVector2 _u;
		public sfloat _frequencyHz;
		public sfloat _dampingRatio;
		public sfloat _gamma;
		public sfloat _bias;
		public sfloat _impulse;
		public sfloat _mass;		// effective mass for the constraint.
		public sfloat _length;

		public override sVector2 Anchor1
		{
			get { return _body1.GetWorldPoint(_localAnchor1);}
		}

		public override sVector2 Anchor2
		{
			get { return _body2.GetWorldPoint(_localAnchor2);}
		}

		public override sVector2 GetReactionForce(sfloat inv_dt)
		{
			return (inv_dt * _impulse) * _u;
		}

		public override sfloat GetReactionTorque(sfloat inv_dt)
		{
			return sfloat.Zero;
		}

		public DistanceJoint(DistanceJointDef def)
			: base(def)
		{
			_localAnchor1 = def.LocalAnchor1;
			_localAnchor2 = def.LocalAnchor2;
			_length = def.Length;
			_frequencyHz = def.FrequencyHz;
			_dampingRatio = def.DampingRatio;
			_impulse = sfloat.Zero;
			_gamma = sfloat.Zero;
			_bias = sfloat.Zero;
		}

		internal override void InitVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			// Compute the effective mass matrix.
			sVector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
			sVector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());
			_u = b2._sweep.C + r2 - b1._sweep.C - r1;

			// Handle singularity.
			sfloat length = _u.magnitude;
			if (length > Settings.LinearSlop)
			{
				_u *= sfloat.One / length;
			}
			else
			{
				_u = sVector2.zero;
			}

			sfloat cr1u = r1.Cross(_u);
			sfloat cr2u = r2.Cross(_u);
			sfloat invMass = b1._invMass + b1._invI * cr1u * cr1u + b2._invMass + b2._invI * cr2u * cr2u;
			Box2DXDebug.Assert(invMass > Settings.FLT_EPSILON);
			_mass = sfloat.One / invMass;

			if (_frequencyHz > sfloat.Zero)
			{
				sfloat C = length - _length;

				// Frequency
				sfloat omega = (sfloat)2.0f * Settings.Pi * _frequencyHz;

				// Damping coefficient
				sfloat d = (sfloat)2.0f * _mass * _dampingRatio * omega;

				// Spring stiffness
				sfloat k = _mass * omega * omega;

				// magic formulas
				_gamma = sfloat.One / (step.Dt * (d + step.Dt * k));
				_bias = C * step.Dt * k * _gamma;

				_mass = sfloat.One / (invMass + _gamma);
			}

			if (step.WarmStarting)
			{
				//Scale the inpulse to support a variable timestep.
				_impulse *= step.DtRatio;
				sVector2 P = _impulse * _u;
				b1._linearVelocity -= b1._invMass * P;
				b1._angularVelocity -= b1._invI * r1.Cross(P);
				b2._linearVelocity += b2._invMass * P;
				b2._angularVelocity += b2._invI * r2.Cross(P);
			}
			else
			{
				_impulse = sfloat.Zero;
			}
		}

		internal override bool SolvePositionConstraints(sfloat baumgarte)
		{
			if (_frequencyHz > sfloat.Zero)
			{
				//There is no possition correction for soft distace constraint.
				return true;
			}

			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
			sVector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

			sVector2 d = b2._sweep.C + r2 - b1._sweep.C - r1;

			sfloat length = d.magnitude;
			d.Normalize();
			sfloat C = length - _length;
			C = libm.Clamp(C, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);

			sfloat impulse = -_mass * C;
			_u = d;
			sVector2 P = impulse * _u;

			b1._sweep.C -= b1._invMass * P;
			b1._sweep.A -= b1._invI * r1.Cross(P);
			b2._sweep.C += b2._invMass * P;
			b2._sweep.A += b2._invI * r2.Cross(P);

			b1.SynchronizeTransform();
			b2.SynchronizeTransform();

			return sfloat.Abs(C) < Settings.LinearSlop;
		}

		internal override void SolveVelocityConstraints(TimeStep step)
		{
			//B2_NOT_USED(step);

			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 r1 = b1.GetTransform().TransformDirection( _localAnchor1 - b1.GetLocalCenter());
			sVector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

			// Cdot = dot(u, v + cross(w, r))
			sVector2 v1 = b1._linearVelocity + r1.CrossScalarPreMultiply(b1._angularVelocity);
			sVector2 v2 = b2._linearVelocity + r2.CrossScalarPreMultiply(b2._angularVelocity);
			sfloat Cdot = sVector2.Dot(_u, v2 - v1);
			sfloat impulse = -_mass * (Cdot + _bias + _gamma * _impulse);
			_impulse += impulse;

			sVector2 P = impulse * _u;
			b1._linearVelocity -= b1._invMass * P;
			b1._angularVelocity -= b1._invI * r1.Cross(P);
			b2._linearVelocity += b2._invMass * P;
			b2._angularVelocity += b2._invI * r2.Cross(P);
		}
	}
}
