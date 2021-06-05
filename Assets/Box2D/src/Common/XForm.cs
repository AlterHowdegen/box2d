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
	/// A transform contains translation and rotation.
	/// It is used to represent the position and orientation of rigid frames.
	/// </summary>
	public struct XForm
	{
		public sVector2 	position;
		public Mat22 	R;

		/// <summary>
		/// Initialize using a position vector and a rotation matrix.
		/// </summary>
		/// <param name="position"></param>
		/// <param name="R"></param>
		public XForm(sVector2 p, Mat22 rotation)
		{
			position = p;
			R = rotation;
		}

		/// <summary>
		/// Set this to the identity transform.
		/// </summary>
		public void SetIdentity()
		{
			position = sVector2.zero;
			R = Mat22.Identity;
		}

		/// Set this based on the position and angle.
		public void Set(sVector2 p, sfloat angle)
		{
			position = p;
			R = new Mat22(angle);
		}

		/// Calculate the angle that the rotation matrix represents.
		public sfloat GetAngle()
		{
			return libm.atan2f(R.Col1.y, R.Col1.x);
		}
		
		public sVector2 TransformDirection(sVector2 vector) 
		{
			return Math.Mul(R, vector);
		}
		
		public sVector2 InverseTransformDirection(sVector2 vector)
		{
			return Math.MulT(R, vector);
		}
		
		public sVector2 TransformPoint(sVector2 vector)
		{	
			return position + Math.Mul(R, vector);
		}
		
		public sVector2 InverseTransformPoint(sVector2 vector)
		{
			return Math.MulT(R, vector - position);
		}

		public static XForm Identity { get { return new XForm(sVector2.zero, Mat22.Identity); } }
	}
}
