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

using System;
using System.Collections.Generic;
using System.Text;
using SoftFloat;
using UnityEngine;

namespace Box2DX.Common
{
	/// <summary>
	/// A Transform contains translation and rotation.
	/// It is used to represent the position and orientation of rigid frames.
	/// </summary>
	[System.Serializable]
	public struct Transform
	{
		public sVector2 		position;
#if USE_MATRIX_FOR_ROTATION
		public Mat22 		rotation;
#else	
		public Quaternion 	rotation;
#endif 
		
#if USE_MATRIX_FOR_ROTATION
		/// <summary>
		/// Initialize using a position vector and a rotation matrix.
		/// </summary>
		/// <param name="position"></param>
		/// <param name="R"></param>
		public Transform(sVector2 position, Mat22 rotation)
		{
			this.position = position;
			this.rotation = rotation;
		}
#else 
		/// <summary>
		/// Initialize using a position vector and a rotation matrix.
		/// </summary>
		/// <param name="position"></param>
		/// <param name="R"></param>
		public Transform(sVector2 position, Quaternion rotation)
		{
			this.position = position;
			this.rotation = rotation;
		}
#endif 
		
		public sVector2 InverseTransformPoint(sVector2 vector) 
		{	
#if USE_MATRIX_FOR_ROTATION
			return Math.MulT(rotation, vector - position);
#else
			return Quaternion.Inverse(rotation) * (vector - position);
#endif
		}
		
		public sVector2 InverseTransformDirection(sVector2 vector)
		{
#if USE_MATRIX_FOR_ROTATION
			return Math.MulT(rotation, vector);
#else
			return Quaternion.Inverse(rotation) * vector;
#endif
		}
		
		public sVector2 TransformPoint(sVector2 vector)
		{	
#if USE_MATRIX_FOR_ROTATION
			return position + Math.Mul(rotation, vector);
#else
			return position + (rotation * vector.ToVector3()).TosVector2();
#endif
			
		}
	
		// <summary>
		// Transforms direction from local space to world space.
		// </summary>
		public sVector2 TransformDirection(sVector2 vector) 
		{ 
#if USE_MATRIX_FOR_ROTATION
			return Math.Mul(rotation, vector);
#else
			return (rotation * vector.ToVector3()).TosVector2();
#endif
		}
		
#if USE_MATRIX_FOR_ROTATION
		public static readonly Transform identify = new Transform(sVector2.zero, Mat22.Identity);
#else 
		public static readonly Transform identity = new Transform(sVector2.zero, Quaternion.identity);
#endif 
	}
}
