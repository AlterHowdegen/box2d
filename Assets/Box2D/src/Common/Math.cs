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

using Random = UnityEngine.Random;

namespace Box2DX.Common
{
	public static class sVector2Extension 
	{	
		// public static Vector3 ToVector3(this sVector2 vector) 
		// { 
		// 	return new Vector3(vector.x, vector.y, 0.0f);
		// }
		
		public static bool IsValid(this sVector2 vector)
		{
			return Math.IsValid(vector.x) && Math.IsValid(vector.y);
		}
		
		public static sfloat Cross(this sVector2 vector, sVector2 other) 
		{ 
			return vector.x * other.y - vector.y * other.x;
		}
		
		public static sVector2 CrossScalarPostMultiply(this sVector2 vector, sfloat s) 
		{ 
			return new sVector2(s * vector.y, -s * vector.x);
		}
		
		public static sVector2 CrossScalarPreMultiply(this sVector2 vector, sfloat s)
		{
			return new sVector2(-s * vector.y, s * vector.x);
		}
		
		public static sVector2 Abs(this sVector2 vector) { 
			return new sVector2(sfloat.Abs(vector.x), sfloat.Abs(vector.y));
		}
	}
	
	public static class Vector3Extension 
	{ 
		public static sVector2 TosVector2(this Vector3 vector) 
		{
			return new sVector2(vector.x, vector.y);
		}
	}
	
	public static class QuaternionExtension 
	{
		// public static Quaternion FromAngle2D(sfloat radians) 
		// { 
		// 	return Quaternion.AngleAxis(radians * libm.Rad2Deg, Vector3.forward);
		// }
	}
	
	public class Math
	{
		public static readonly ushort USHRT_MAX = 0xffff;
		public static readonly byte UCHAR_MAX = 0xff;
		public static readonly int RAND_LIMIT = 32767;

		/// <summary>
		/// This function is used to ensure that a sfloating point number is
		/// not a NaN or infinity.
		/// </summary>
		public static bool IsValid(sfloat x)
		{
			return !(x.IsNaN() || x.IsNegativeInfinity() || x.IsPositiveInfinity());
		}

		[System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
		public struct Convert
		{
			[System.Runtime.InteropServices.FieldOffset(0)]
			public sfloat x;

			[System.Runtime.InteropServices.FieldOffset(0)]
			public int i;
		}
	

#if USE_MATRIX_FOR_ROTATION
	 	public static Mat22 AngleToRotation(sfloat angle) 
		{
			return new Mat22(angle);
		}
#else
		public static Quaternion AngleToRotation(sfloat angle)
		{
			return QuaternionExtension.FromAngle2D(angle);
		}
#endif 

		/// <summary>
		/// This is a approximate yet fast inverse square-root.
		/// </summary>
		public static sfloat InvSqrt(sfloat x)
		{
			Convert convert = new Convert();
			convert.x = x;
			sfloat xhalf = (sfloat)0.5f * x;
			convert.i = 0x5f3759df - (convert.i >> 1);
			x = convert.x;
			x = x * ((sfloat)1.5f - xhalf * x * x);
			return x;
		}

		public static sfloat Sqrt(sfloat x)
		{
			return libm.sqrtf(x);
		}
		
		/// <summary>
		/// Random sfloating point number in range [lo, hi]
		/// </summary>

		/// <summary>
		/// "Next Largest Power of 2
		/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
		/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
		/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
		/// largest power of 2. For a 32-bit value:"
		/// </summary>
		public static uint NextPowerOfTwo(uint x)
		{
			x |= (x >> 1);
			x |= (x >> 2);
			x |= (x >> 4);
			x |= (x >> 8);
			x |= (x >> 16);
			return x + 1;
		}

		public static bool IsPowerOfTwo(uint x)
		{
			bool result = x > 0 && (x & (x - 1)) == 0;
			return result;
		}

		public static sfloat Abs(sfloat a)
		{
			return a > sfloat.Zero ? a : -a;
		}

		public static sVector2 Abs(sVector2 a)
		{
			return new sVector2(sfloat.Abs(a.x), sfloat.Abs(a.y));
		}

		public static Mat22 Abs(Mat22 A)
		{
			Mat22 B = new Mat22();
			B.Set(Math.Abs(A.Col1), Math.Abs(A.Col2));
			return B;
		}

		public static sfloat Min(sfloat a, sfloat b)
		{
			return a < b ? a : b;
		}

		public static int Min(int a, int b)
		{
			return a < b ? a : b;
		}

		public static sfloat Max(sfloat a, sfloat b)
		{
			return a > b ? a : b;
		}

		public static int Max(int a, int b)
		{
			return a > b ? a : b;
		}

		public static sVector2 Clamp(sVector2 a, sVector2 low, sVector2 high)
		{
			return sVector2.Max(low, sVector2.Min(a, high));
		}

		public static void Swap<T>(ref T a, ref T b)
		{
			T tmp = a;
			a = b;
			b = tmp;
		}

		/// <summary>
		/// Multiply a matrix times a vector. If a rotation matrix is provided,
		/// then this Transforms the vector from one frame to another.
		/// </summary>
		public static sVector2 Mul(Mat22 A, sVector2 v)
		{
			return new sVector2(A.Col1.x * v.x + A.Col2.x * v.y, A.Col1.y * v.x + A.Col2.y * v.y);
		}

		/// <summary>
		/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
		/// then this Transforms the vector from one frame to another (inverse Transform).
		/// </summary>
		public static sVector2 MulT(Mat22 A, sVector2 v)
		{
			return new sVector2(sVector2.Dot(v, A.Col1), sVector2.Dot(v, A.Col2));
		}

		/// <summary>
		/// A * B
		/// </summary>
		public static Mat22 Mul(Mat22 A, Mat22 B)
		{
			Mat22 C = new Mat22();
			C.Set(Math.Mul(A, B.Col1), Math.Mul(A, B.Col2));
			return C;
		}

		/// <summary>
		/// A^T * B
		/// </summary>
		public static Mat22 MulT(Mat22 A, Mat22 B)
		{
			sVector2 c1 = new sVector2(sVector2.Dot(A.Col1, B.Col1), sVector2.Dot(A.Col2, B.Col1));
			sVector2 c2 = new sVector2(sVector2.Dot(A.Col1, B.Col2), sVector2.Dot(A.Col2, B.Col2));
			return new Mat22(c1, c2);
		}
	
		public static sVector2 Mul(Transform T, sVector2 v)
		{
#if USE_MATRIX_FOR_ROTATION
			return T.position + Math.Mul(T.rotation, v);
#else
			return T.position + T.TransformDirection(v);
#endif
		}

		public static sVector2 MulT(Transform T, sVector2 v)
		{
#if USE_MATRIX_FOR_ROTATION
			return Math.MulT(T.rotation, v - T.position);
#else
			return T.InverseTransformDirection(v - T.position);
#endif
		}
	
		/// <summary>
		/// Multiply a matrix times a vector.
		/// </summary>
		public static Vector3 Mul(Mat33 A, Vector3 v)
		{
			return v.x * A.Col1 + v.y * A.Col2 + v.z * A.Col3;
		}

		public static sfloat Atan2(sfloat y, sfloat x)
		{
			return (sfloat)libm.atan2f(y, x);
		}
	}
}
