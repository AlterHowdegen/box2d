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

using System;
using System.Collections.Generic;
using System.Text;
using SoftFloat;
using UnityEngine;

namespace Box2DX.Common
{
	/// <summary>
	/// A 2-by-2 matrix. Stored in column-major order.
	/// </summary>
	public struct Mat22
	{
		public sVector2 Col1;
		public sVector2 Col2;

		/// <summary>
		/// Construct this matrix using columns.
		/// </summary>
		public Mat22(sVector2 c1, sVector2 c2)
		{
			Col1 = c1;
			Col2 = c2;
		}

		/// <summary>
		/// Construct this matrix using scalars.
		/// </summary>
		public Mat22(sfloat a11, sfloat a12, sfloat a21, sfloat a22)
		{
			Col1.x = a11; Col1.y = a21;
			Col2.x = a12; Col2.y = a22;
		}

		/// <summary>
		/// Construct this matrix using an angle. 
		/// This matrix becomes an orthonormal rotation matrix.
		/// </summary>
		public Mat22(sfloat angle)
		{
			sfloat c = libm.cosf(angle);
			sfloat s = libm.sinf(angle);
			Col1.x = c; Col2.x = -s;
			Col1.y = s; Col2.y = c;
		}

		/// <summary>
		/// Initialize this matrix using columns.
		/// </summary>
		public void Set(sVector2 c1, sVector2 c2)
		{
			Col1 = c1;
			Col2 = c2;
		}

		/// <summary>
		/// Initialize this matrix using an angle.
		/// This matrix becomes an orthonormal rotation matrix.
		/// </summary>
		public void Set(sfloat angle)
		{
			sfloat c = libm.cosf(angle);
			sfloat s = libm.sinf(angle);
			Col1.x = c; Col2.x = -s;
			Col1.y = s; Col2.y = c;
		}

		/// <summary>
		/// Extract the angle from this matrix (assumed to be a rotation matrix).
		/// </summary>
		public sfloat GetAngle()
		{
			return libm.atan2f(Col1.y, Col1.x);
		}
		
		public sVector2 Multiply(sVector2 vector) { 
			return new sVector2(Col1.x * vector.x + Col2.x * vector.y, Col1.y * vector.x + Col2.y * vector.y);
		}
		
		/// <summary>
		/// Compute the inverse of this matrix, such that inv(A) * A = identity.
		/// </summary>
		public Mat22 GetInverse()
		{
			sfloat a = Col1.x, b = Col2.x, c = Col1.y, d = Col2.y;
			Mat22 B = new Mat22();
			sfloat det = a * d - b * c;
			Box2DXDebug.Assert(det != sfloat.Zero);
			det = sfloat.One / det;
			B.Col1.x = det * d; B.Col2.x = -det * b;
			B.Col1.y = -det * c; B.Col2.y = det * a;
			return B;
		}

		/// <summary>
		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		/// </summary>
		public sVector2 Solve(sVector2 b)
		{
			sfloat a11 = Col1.x, a12 = Col2.x, a21 = Col1.y, a22 = Col2.y;
			sfloat det = a11 * a22 - a12 * a21;
			Box2DXDebug.Assert(det != sfloat.Zero);
			det = sfloat.One / det;
			sVector2 x = new sVector2();
			x.x = det * (a22 * b.x - a12 * b.y);
			x.y = det * (a11 * b.y - a21 * b.x);
			return x;
		}

		public static Mat22 Identity { get { return new Mat22(sfloat.One, sfloat.Zero, sfloat.Zero, sfloat.One); } }

		public static Mat22 operator +(Mat22 A, Mat22 B)
		{
			Mat22 C = new Mat22();
			C.Set(A.Col1 + B.Col1, A.Col2 + B.Col2);
			return C;
		}
	}
}
