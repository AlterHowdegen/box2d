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
	/// A circle shape.
	/// </summary>
	public class CircleShape : Shape
	{
		// Position
		internal sVector2 _position;

		public CircleShape()			
		{
			_type = ShapeType.CircleShape;
		}

		public override bool TestPoint(Transform xf, sVector2 p)
		{
			sVector2 center = xf.position + xf.TransformDirection(_position);
			sVector2 d = p - center;
			return sVector2.Dot(d, d) <= _radius * _radius;
		}

		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.1.2
		// x = s + a * r
		// norm(x) = radius
		public override SegmentCollide TestSegment(Transform xf, out sfloat lambda, out sVector2 normal, Segment segment, sfloat maxLambda)
		{
			lambda = sfloat.Zero;
			normal = sVector2.zero;

			sVector2 position = xf.position + xf.TransformDirection(_position);
			sVector2 s = segment.P1 - position;
			sfloat b = sVector2.Dot(s, s) - _radius * _radius;

			// Does the segment start inside the circle?
			if (b < sfloat.Zero)
			{
				lambda = sfloat.Zero;
				return SegmentCollide.StartInsideCollide;
			}

			// Solve quadratic equation.
			sVector2 r = segment.P2 - segment.P1;
			sfloat c = sVector2.Dot(s, r);
			sfloat rr = sVector2.Dot(r, r);
			sfloat sigma = c * c - rr * b;

			// Check for negative discriminant and short segment.
			if (sigma < sfloat.Zero || rr < Common.Settings.FLT_EPSILON)
			{
				return SegmentCollide.MissCollide;
			}

			// Find the point of intersection of the line with the circle.
			sfloat a = -(c + Common.Math.Sqrt(sigma));

			// Is the intersection point on the segment?
			if (sfloat.Zero <= a && a <= maxLambda * rr)
			{
				a /= rr;
				lambda = a;
				normal = s + a * r;
				normal.Normalize();
				return SegmentCollide.HitCollide;
			}

			return SegmentCollide.MissCollide;
		}

		public override void ComputeAABB(out AABB aabb, Transform xf)
		{
			aabb = new AABB();

			sVector2 p = xf.position + xf.TransformDirection(_position);
			aabb.LowerBound = new sVector2(p.x - _radius, p.y - _radius);
			aabb.UpperBound = new sVector2(p.x + _radius, p.y + _radius);
		}

		public override void ComputeMass(out MassData massData, sfloat density)
		{
			massData = new MassData();

			massData.Mass = density * (sfloat)libm.pi * _radius * _radius;
			massData.Center = _position;

			// inertia about the local origin
			massData.I = massData.Mass * ((sfloat)0.5f * _radius * _radius + sVector2.Dot(_position, _position));
		}		

		public override sfloat ComputeSubmergedArea(sVector2 normal, sfloat offset, Transform xf, out sVector2 c)
		{
			sVector2 p = xf.TransformPoint(_position);
			sfloat l = -(sVector2.Dot(normal, p) - offset);
			if (l < -_radius + Box2DX.Common.Settings.FLT_EPSILON)
			{
				//Completely dry
				c = new sVector2();
				return sfloat.Zero;
			}
			if (l > _radius)
			{
				//Completely wet
				c = p;
				return Box2DX.Common.Settings.Pi * _radius * _radius;
			}

			//Magic
			sfloat r2 = _radius * _radius;
			sfloat l2 = l * l;
			sfloat area = r2 * ((sfloat)libm.asinf(l / _radius) + Box2DX.Common.Settings.Pi / (sfloat)2) +
				l * Box2DX.Common.Math.Sqrt(r2 - l2);
			sfloat com = -(sfloat)2.0f / (sfloat)3.0f * (sfloat)libm.powf(r2 - l2, (sfloat)1.5f) / area;

			c.x = p.x + normal.x * com;
			c.y = p.y + normal.y * com;

			return area;
		}

		/// <summary>
		/// Get the supporting vertex index in the given direction.
		/// </summary>
		public override int GetSupport(sVector2 d)
		{
			return 0;
		}

		/// <summary>
		/// Get the supporting vertex in the given direction.
		/// </summary>
		public override sVector2 GetSupportVertex(sVector2 d)
		{
			return _position;
		}

		/// <summary>
		/// Get a vertex by index. Used by Distance.
		/// </summary>
		public override sVector2 GetVertex(int index)
		{
			Box2DXDebug.Assert(index == 0);
			return _position;
		}

		public override sfloat ComputeSweepRadius(sVector2 pivot)
		{
			return sVector2.Distance(_position, pivot);
		}

		/// <summary>
		/// Get the vertex count.
		/// </summary>
		public int VertexCount { get { return 1; } }
	}
}