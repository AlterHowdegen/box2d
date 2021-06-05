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

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2) >= 0
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)
//
// Limit:
// C = maxLength - length
// u = (p - s) / norm(p - s)
// Cdot = -dot(u, v + cross(w, r))
// K = invMass + invI * cross(r, u)^2
// 0 <= impulse

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;
using SoftFloat;
using UnityEngine;

namespace Box2DX.Dynamics
{
	using Box2DXMath = Box2DX.Common.Math;
	using SystemMath = System.Math;

	/// <summary>
	/// Pulley joint definition. This requires two ground anchors,
	/// two dynamic body anchor points, max lengths for each side,
	/// and a pulley ratio.
	/// </summary>
	public class PulleyJointDef : JointDef
	{
		public PulleyJointDef()
		{
			Type = JointType.PulleyJoint;
			GroundAnchor1 = new sVector2(-sfloat.One, sfloat.One);
			GroundAnchor2 = new sVector2(sfloat.One, sfloat.One);
			LocalAnchor1 = new sVector2(-sfloat.One, sfloat.Zero);
			LocalAnchor2 = new sVector2(sfloat.One, sfloat.Zero);
			Length1 = sfloat.Zero;
			MaxLength1 = sfloat.Zero;
			Length2 = sfloat.Zero;
			MaxLength2 = sfloat.Zero;
			Ratio = sfloat.One;
			CollideConnected = true;
		}

		/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
		public void Initialize(Body body1, Body body2,
						sVector2 groundAnchor1, sVector2 groundAnchor2,
						sVector2 anchor1, sVector2 anchor2,
						sfloat ratio)
		{
			Body1 = body1;
			Body2 = body2;
			GroundAnchor1 = groundAnchor1;
			GroundAnchor2 = groundAnchor2;
			LocalAnchor1 = body1.GetLocalPoint(anchor1);
			LocalAnchor2 = body2.GetLocalPoint(anchor2);
			sVector2 d1 = anchor1 - groundAnchor1;
			Length1 = d1.magnitude;
			sVector2 d2 = anchor2 - groundAnchor2;
			Length2 = d2.magnitude;
			Ratio = ratio;
			Box2DXDebug.Assert(ratio > Settings.FLT_EPSILON);
			sfloat C = Length1 + ratio * Length2;
			MaxLength1 = C - ratio * PulleyJoint.MinPulleyLength;
			MaxLength2 = (C - PulleyJoint.MinPulleyLength) / ratio;
		}

		/// <summary>
		/// The first ground anchor in world coordinates. This point never moves.
		/// </summary>
		public sVector2 GroundAnchor1;

		/// <summary>
		/// The second ground anchor in world coordinates. This point never moves.
		/// </summary>
		public sVector2 GroundAnchor2;

		/// <summary>
		/// The local anchor point relative to body1's origin.
		/// </summary>
		public sVector2 LocalAnchor1;

		/// <summary>
		/// The local anchor point relative to body2's origin.
		/// </summary>
		public sVector2 LocalAnchor2;

		/// <summary>
		/// The a reference length for the segment attached to body1.
		/// </summary>
		public sfloat Length1;

		/// <summary>
		/// The maximum length of the segment attached to body1.
		/// </summary>
		public sfloat MaxLength1;

		/// <summary>
		/// The a reference length for the segment attached to body2.
		/// </summary>
		public sfloat Length2;

		/// <summary>
		/// The maximum length of the segment attached to body2.
		/// </summary>
		public sfloat MaxLength2;

		/// <summary>
		/// The pulley ratio, used to simulate a block-and-tackle.
		/// </summary>
		public sfloat Ratio;
	}

	/// <summary>
	/// The pulley joint is connected to two bodies and two fixed ground points.
	/// The pulley supports a ratio such that:
	/// length1 + ratio * length2 <= constant
	/// Yes, the force transmitted is scaled by the ratio.
	/// The pulley also enforces a maximum length limit on both sides. This is
	/// useful to prevent one side of the pulley hitting the top.
	/// </summary>
	public class PulleyJoint : Joint
	{
		public static readonly sfloat MinPulleyLength = (sfloat)2.0f;

		public Body _ground;
		public sVector2 _groundAnchor1;
		public sVector2 _groundAnchor2;
		public sVector2 _localAnchor1;
		public sVector2 _localAnchor2;

		public sVector2 _u1;
		public sVector2 _u2;

		public sfloat _constant;
		public sfloat _ratio;

		public sfloat _maxLength1;
		public sfloat _maxLength2;

		// Effective masses
		public sfloat _pulleyMass;
		public sfloat _limitMass1;
		public sfloat _limitMass2;

		// Impulses for accumulation/warm starting.
		public sfloat _impulse;
		public sfloat _limitImpulse1;
		public sfloat _limitImpulse2;

		public LimitState _state;
		public LimitState _limitState1;
		public LimitState _limitState2;

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
			sVector2 P = _impulse * _u2;
			return inv_dt * P;
		}

		public override sfloat GetReactionTorque(sfloat inv_dt)
		{
			return sfloat.Zero;
		}

		/// <summary>
		/// Get the first ground anchor.
		/// </summary>
		public sVector2 GroundAnchor1
		{
			get { return _ground.GetTransform().position + _groundAnchor1; }
		}

		/// <summary>
		/// Get the second ground anchor.
		/// </summary>
		public sVector2 GroundAnchor2
		{
			get { return _ground.GetTransform().position + _groundAnchor2; }
		}

		/// <summary>
		/// Get the current length of the segment attached to body1.
		/// </summary>
		public sfloat Length1
		{
			get
			{
				sVector2 p = _body1.GetWorldPoint(_localAnchor1);
				sVector2 s = _ground.GetTransform().position + _groundAnchor1;
				sVector2 d = p - s;
				return d.magnitude;
			}
		}

		/// <summary>
		/// Get the current length of the segment attached to body2.
		/// </summary>
		public sfloat Length2
		{
			get
			{
				sVector2 p = _body2.GetWorldPoint(_localAnchor2);
				sVector2 s = _ground.GetTransform().position + _groundAnchor2;
				sVector2 d = p - s;
				return d.magnitude;
			}
		}

		/// <summary>
		/// Get the pulley ratio.
		/// </summary>
		public sfloat Ratio
		{
			get { return _ratio; }
		}

		public PulleyJoint(PulleyJointDef def)
			: base(def)
		{
			_ground = _body1.GetWorld().GetGroundBody();
			_groundAnchor1 = def.GroundAnchor1 - _ground.GetTransform().position;
			_groundAnchor2 = def.GroundAnchor2 - _ground.GetTransform().position;
			_localAnchor1 = def.LocalAnchor1;
			_localAnchor2 = def.LocalAnchor2;

			Box2DXDebug.Assert(def.Ratio != sfloat.Zero);
			_ratio = def.Ratio;

			_constant = def.Length1 + _ratio * def.Length2;

			_maxLength1 = Common.Math.Min(def.MaxLength1, _constant - _ratio * PulleyJoint.MinPulleyLength);
			_maxLength2 = Common.Math.Min(def.MaxLength2, (_constant - PulleyJoint.MinPulleyLength) / _ratio);

			_impulse = sfloat.Zero;
			_limitImpulse1 = sfloat.Zero;
			_limitImpulse2 = sfloat.Zero;
		}

		internal override void InitVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
			sVector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

			sVector2 p1 = b1._sweep.C + r1;
			sVector2 p2 = b2._sweep.C + r2;

			sVector2 s1 = _ground.GetTransform().position + _groundAnchor1;
			sVector2 s2 = _ground.GetTransform().position + _groundAnchor2;

			// Get the pulley axes.
			_u1 = p1 - s1;
			_u2 = p2 - s2;

			sfloat length1 = _u1.magnitude;
			sfloat length2 = _u2.magnitude;

			if (length1 > Settings.LinearSlop)
			{
				_u1 *= sfloat.One / length1;
			}
			else
			{
				_u1 = sVector2.zero;
			}

			if (length2 > Settings.LinearSlop)
			{
				_u2 *= sfloat.One / length2;
			}
			else
			{
				_u2 = sVector2.zero;
			}

			sfloat C = _constant - length1 - _ratio * length2;
			if (C > sfloat.Zero)
			{
				_state = LimitState.InactiveLimit;
				_impulse = sfloat.Zero;
			}
			else
			{
				_state = LimitState.AtUpperLimit;
			}

			if (length1 < _maxLength1)
			{
				_limitState1 = LimitState.InactiveLimit;
				_limitImpulse1 = sfloat.Zero;
			}
			else
			{
				_limitState1 = LimitState.AtUpperLimit;
			}

			if (length2 < _maxLength2)
			{
				_limitState2 = LimitState.InactiveLimit;
				_limitImpulse2 = sfloat.Zero;
			}
			else
			{
				_limitState2 = LimitState.AtUpperLimit;
			}

			// Compute effective mass.
			sfloat cr1u1 = r1.Cross(_u1);
			sfloat cr2u2 = r2.Cross(_u2);

			_limitMass1 = b1._invMass + b1._invI * cr1u1 * cr1u1;
			_limitMass2 = b2._invMass + b2._invI * cr2u2 * cr2u2;
			_pulleyMass = _limitMass1 + _ratio * _ratio * _limitMass2;
			Box2DXDebug.Assert(_limitMass1 > Settings.FLT_EPSILON);
			Box2DXDebug.Assert(_limitMass2 > Settings.FLT_EPSILON);
			Box2DXDebug.Assert(_pulleyMass > Settings.FLT_EPSILON);
			_limitMass1 = sfloat.One / _limitMass1;
			_limitMass2 = sfloat.One / _limitMass2;
			_pulleyMass = sfloat.One / _pulleyMass;

			if (step.WarmStarting)
			{
				// Scale impulses to support variable time steps.
				_impulse *= step.DtRatio;
				_limitImpulse1 *= step.DtRatio;
				_limitImpulse2 *= step.DtRatio;

				// Warm starting.
				sVector2 P1 = -(_impulse + _limitImpulse1) * _u1;
				sVector2 P2 = (-_ratio * _impulse - _limitImpulse2) * _u2;
				b1._linearVelocity += b1._invMass * P1;
				b1._angularVelocity += b1._invI * r1.Cross(P1);
				b2._linearVelocity += b2._invMass * P2;
				b2._angularVelocity += b2._invI * r2.Cross(P2);
			}
			else
			{
				_impulse = sfloat.Zero;
				_limitImpulse1 = sfloat.Zero;
				_limitImpulse2 = sfloat.Zero;
			}
		}

		internal override void SolveVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
			sVector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

			if (_state == LimitState.AtUpperLimit)
			{
				sVector2 v1 = b1._linearVelocity + r1.CrossScalarPreMultiply(b1._angularVelocity);
				sVector2 v2 = b2._linearVelocity + r1.CrossScalarPreMultiply(b2._angularVelocity);

				sfloat Cdot = -sVector2.Dot(_u1, v1) - _ratio * sVector2.Dot(_u2, v2);
				sfloat impulse = _pulleyMass * (-Cdot);
				sfloat oldImpulse = _impulse;
				_impulse = Box2DX.Common.Math.Max(sfloat.Zero, _impulse + impulse);
				impulse = _impulse - oldImpulse;

				sVector2 P1 = -impulse * _u1;
				sVector2 P2 = -_ratio * impulse * _u2;
				b1._linearVelocity += b1._invMass * P1;
				b1._angularVelocity += b1._invI * r1.Cross(P1);
				b2._linearVelocity += b2._invMass * P2;
				b2._angularVelocity += b2._invI * r2.Cross(P2);
			}

			if (_limitState1 == LimitState.AtUpperLimit)
			{
				sVector2 v1 = b1._linearVelocity + r1.CrossScalarPreMultiply(b1._angularVelocity);

				sfloat Cdot = -sVector2.Dot(_u1, v1);
				sfloat impulse = -_limitMass1 * Cdot;
				sfloat oldImpulse = _limitImpulse1;
				_limitImpulse1 = Box2DX.Common.Math.Max(sfloat.Zero, _limitImpulse1 + impulse);
				impulse = _limitImpulse1 - oldImpulse;

				sVector2 P1 = -impulse * _u1;
				b1._linearVelocity += b1._invMass * P1;
				b1._angularVelocity += b1._invI * r1.Cross(P1);
			}

			if (_limitState2 == LimitState.AtUpperLimit)
			{
				sVector2 v2 = b2._linearVelocity + r2.CrossScalarPreMultiply(b2._angularVelocity);

				sfloat Cdot = -sVector2.Dot(_u2, v2);
				sfloat impulse = -_limitMass2 * Cdot;
				sfloat oldImpulse = _limitImpulse2;
				_limitImpulse2 = Box2DX.Common.Math.Max(sfloat.Zero, _limitImpulse2 + impulse);
				impulse = _limitImpulse2 - oldImpulse;

				sVector2 P2 = -impulse * _u2;
				b2._linearVelocity += b2._invMass * P2;
				b2._angularVelocity += b2._invI * r2.Cross(P2);
			}
		}

		internal override bool SolvePositionConstraints(sfloat baumgarte)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			sVector2 s1 = _ground.GetTransform().position + _groundAnchor1;
			sVector2 s2 = _ground.GetTransform().position + _groundAnchor2;

			sfloat linearError = sfloat.Zero;

			if (_state == LimitState.AtUpperLimit)
			{
				sVector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
				sVector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());

				sVector2 p1 = b1._sweep.C + r1;
				sVector2 p2 = b2._sweep.C + r2;

				// Get the pulley axes.
				_u1 = p1 - s1;
				_u2 = p2 - s2;

				sfloat length1 = _u1.magnitude;
				sfloat length2 = _u2.magnitude;

				if (length1 > Settings.LinearSlop)
				{
					_u1 *= sfloat.One / length1;
				}
				else
				{
					_u1 = sVector2.zero;
				}

				if (length2 > Settings.LinearSlop)
				{
					_u2 *= sfloat.One / length2;
				}
				else
				{
					_u2 = sVector2.zero;
				}

				sfloat C = _constant - length1 - _ratio * length2;
				linearError = Box2DXMath.Max(linearError, -C);

				C = libm.Clamp(C + Settings.LinearSlop, -Settings.MaxLinearCorrection, sfloat.Zero);
				sfloat impulse = -_pulleyMass * C;

				sVector2 P1 = -impulse * _u1;
				sVector2 P2 = -_ratio * impulse * _u2;

				b1._sweep.C += b1._invMass * P1;
				b1._sweep.A += b1._invI * r1.Cross(P1);
				b2._sweep.C += b2._invMass * P2;
				b2._sweep.A += b2._invI * r2.Cross(P2);

				b1.SynchronizeTransform();
				b2.SynchronizeTransform();
			}

			if (_limitState1 == LimitState.AtUpperLimit)
			{
				sVector2 r1 = b1.GetTransform().TransformDirection(_localAnchor1 - b1.GetLocalCenter());
				sVector2 p1 = b1._sweep.C + r1;

				_u1 = p1 - s1;
				sfloat length1 = _u1.magnitude;

				if (length1 > Settings.LinearSlop)
				{
					_u1 *= sfloat.One / length1;
				}
				else
				{
					_u1 = sVector2.zero;
				}

				sfloat C = _maxLength1 - length1;
				linearError = sfloat.Max(linearError, -C);
				C = libm.Clamp(C + Settings.LinearSlop, -Settings.MaxLinearCorrection, sfloat.Zero);
				sfloat impulse = -_limitMass1 * C;

				sVector2 P1 = -impulse * _u1;
				b1._sweep.C += b1._invMass * P1;
				b1._sweep.A += b1._invI * r1.Cross(P1);

				b1.SynchronizeTransform();
			}

			if (_limitState2 == LimitState.AtUpperLimit)
			{
				sVector2 r2 = b2.GetTransform().TransformDirection(_localAnchor2 - b2.GetLocalCenter());
				sVector2 p2 = b2._sweep.C + r2;

				_u2 = p2 - s2;
				sfloat length2 = _u2.magnitude;

				if (length2 > Settings.LinearSlop)
				{
					_u2 *= sfloat.One / length2;
				}
				else
				{
					_u2 = sVector2.zero;
				}

				sfloat C = _maxLength2 - length2;
				linearError = Box2DXMath.Max(linearError, -C);
				C = libm.Clamp(C + Settings.LinearSlop, -Settings.MaxLinearCorrection, sfloat.Zero);
				sfloat impulse = -_limitMass2 * C;

				sVector2 P2 = -impulse * _u2;
				b2._sweep.C += b2._invMass * P2;
				b2._sweep.A += b2._invI * r2.Cross(P2);

				b2.SynchronizeTransform();
			}

			return linearError < Settings.LinearSlop;
		}
	}
}
