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

using System;
using System.Collections.Generic;
using System.Text;
using Box2DX.Common;
using SoftFloat;
using UnityEngine;

using Transform = Box2DX.Common.Transform;

namespace Box2DX.Collision
{
	public class EdgeShape : Shape
	{
		public sVector2 _v1;
		public sVector2 _v2;

		public sfloat _length;

		public sVector2 _normal;

		public sVector2 _direction;

		// Unit vector halfway between m_direction and m_prevEdge.m_direction:
		public sVector2 _cornerDir1;

		// Unit vector halfway between m_direction and m_nextEdge.m_direction:
		public sVector2 _cornerDir2;

		public bool _cornerConvex1;
		public bool _cornerConvex2;

		public EdgeShape _nextEdge;
		public EdgeShape _prevEdge;

		public EdgeShape()
		{
			_type = ShapeType.EdgeShape;
			_radius = Settings.PolygonRadius;
		}

		public override void Dispose()
		{
			if (_prevEdge != null)
			{
				_prevEdge._nextEdge = null;
			}

			if (_nextEdge != null)
			{
				_nextEdge._prevEdge = null;
			}
		}

		public void Set(sVector2 v1, sVector2 v2)
		{
			_v1 = v1;
			_v2 = v2;

			_direction = _v2 - _v1;
			_length = _direction.magnitude;
			_direction.Normalize();
			_normal = _direction.CrossScalarPostMultiply(sfloat.One);

			_cornerDir1 = _normal;
			_cornerDir2 = -sfloat.One * _normal;
		}

		public override bool TestPoint(Transform xf, sVector2 p)
		{
			return false;
		}

		public override SegmentCollide TestSegment(Transform xf, out sfloat lambda, out sVector2 normal, Segment segment, sfloat maxLambda)
		{
			sVector2 r = segment.P2 - segment.P1;
			sVector2 v1 = xf.TransformPoint(_v1);
			sVector2 d = ((sVector2)xf.TransformPoint(_v2)) - v1;
			sVector2 n = d.CrossScalarPostMultiply(sfloat.One);

			sfloat k_slop = (sfloat)100.0f * Common.Settings.FLT_EPSILON;
			sfloat denom = -sVector2.Dot(r, n);

			// Cull back facing collision and ignore parallel segments.
			if (denom > k_slop)
			{
				// Does the segment intersect the infinite line associated with this segment?
				sVector2 b = segment.P1 - v1;
				sfloat a = sVector2.Dot(b, n);

				if (sfloat.Zero <= a && a <= maxLambda * denom)
				{
					sfloat mu2 = -r.x * b.y + r.y * b.x;

					// Does the segment intersect this segment?
					if (-k_slop * denom <= mu2 && mu2 <= denom * (sfloat.One + k_slop))
					{
						a /= denom;
						n.Normalize();
						lambda = a;
						normal = n;
						return SegmentCollide.HitCollide;
					}
				}
			}

			lambda = sfloat.Zero;
			normal = new sVector2();
			return SegmentCollide.MissCollide;
		}

		public override void ComputeAABB(out AABB aabb, Transform xf)
		{
			sVector2 v1 = xf.TransformPoint(_v1);
			sVector2 v2 = xf.TransformPoint(_v2);

			sVector2 r = new sVector2(_radius, _radius);
			aabb.LowerBound = sVector2.Min(v1, v2) - r;
			aabb.UpperBound = sVector2.Max(v1, v2) + r;
		}

		public override void ComputeMass(out MassData massData, sfloat density)
		{
			massData.Mass = sfloat.Zero;
			massData.Center = _v1;
			massData.I = sfloat.Zero;
		}

		public void SetPrevEdge(EdgeShape edge, sVector2 cornerDir, bool convex)
		{
			_prevEdge = edge;
			_cornerDir1 = cornerDir;
			_cornerConvex1 = convex;
		}

		public void SetNextEdge(EdgeShape edge, sVector2 cornerDir, bool convex)
		{
			_nextEdge = edge;
			_cornerDir2 = cornerDir;
			_cornerConvex2 = convex;
		}

		public override sfloat ComputeSubmergedArea(sVector2 normal, sfloat offset, Transform xf, out sVector2 c)
		{
			//Note that v0 is independent of any details of the specific edge
			//We are relying on v0 being consistent between multiple edges of the same body
			sVector2 v0 = offset * normal;
			//b2Vec2 v0 = xf.position + (offset - b2Dot(normal, xf.position)) * normal;

			sVector2 v1 = xf.TransformPoint(_v1);
			sVector2 v2 = xf.TransformPoint(_v2);

			sfloat d1 = sVector2.Dot(normal, v1) - offset;
			sfloat d2 = sVector2.Dot(normal, v2) - offset;

			if (d1 > sfloat.Zero)
			{
				if (d2 > sfloat.Zero)
				{
					c = new sVector2();
					return sfloat.Zero;
				}
				else
				{
					v1 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
				}
			}
			else
			{
				if (d2 > sfloat.Zero)
				{
					v2 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
				}
				else
				{
					//Nothing
				}
			}

			// v0,v1,v2 represents a fully submerged triangle
			sfloat k_inv3 = sfloat.One / (sfloat)3.0f;

			// Area weighted centroid
			c = k_inv3 * (v0 + v1 + v2);

			sVector2 e1 = v1 - v0;
			sVector2 e2 = v2 - v0;

			return (sfloat)0.5f * e1.Cross(e2);
		}

		public sfloat Length
		{
			get { return _length; }
		}

		public sVector2 Vertex1
		{
			get { return _v1; }
		}

		public sVector2 Vertex2
		{
			get { return _v2; }
		}

		public sVector2 NormalVector
		{
			get { return _normal; }
		}

		public sVector2 DirectionVector
		{
			get { return _direction; }
		}

		public sVector2 Corner1Vector
		{
			get { return _cornerDir1; }
		}

		public sVector2 Corner2Vector
		{
			get { return _cornerDir2; }
		}

		public override int GetSupport(sVector2 d)
		{
			return sVector2.Dot(_v1, d) > sVector2.Dot(_v2, d) ? 0 : 1;
		}

		public override sVector2 GetSupportVertex(sVector2 d)
		{
			return sVector2.Dot(_v1, d) > sVector2.Dot(_v2, d) ? _v1 : _v2;
		}

		public override sVector2 GetVertex(int index)
		{
			Box2DXDebug.Assert(0 <= index && index < 2);
			if (index == 0) return _v1;
			else return _v2;
		}

		public bool Corner1IsConvex
		{
			get { return _cornerConvex1; }
		}

		public bool Corner2IsConvex
		{
			get { return _cornerConvex2; }
		}

		public override sfloat ComputeSweepRadius(sVector2 pivot)
		{
			sfloat ds1 = (_v1 - pivot).sqrMagnitude;
			sfloat ds2 = (_v2 - pivot).sqrMagnitude;
			return libm.sqrtf(sfloat.Max(ds1, ds2));
		}
	}
}
