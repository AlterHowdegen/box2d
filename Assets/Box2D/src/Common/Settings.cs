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

namespace Box2DX.Common
{
	public class Settings
	{
#if TARGET_sfloat32_IS_FIXED
		public static readonly sfloat FLT_EPSILON = FIXED_EPSILON;
		public static readonly sfloat FLT_MAX = FIXED_MAX;
		public static sfloat	FORCE_SCALE2(x){ return x<<7;}
		public static sfloat FORCE_INV_SCALE2(x)	{return x>>7;}
#else
		public static readonly sfloat FLT_EPSILON = (sfloat)1.192092896e-07F;//smallest such that 1.0f+FLT_EPSILON != 1.0f
		public static readonly sfloat FLT_EPSILON_SQUARED = FLT_EPSILON * FLT_EPSILON;//smallest such that 1.0f+FLT_EPSILON != 1.0f
		public static readonly sfloat FLT_MAX = (sfloat)3.402823466e+38F;
		public static sfloat FORCE_SCALE(sfloat x) { return x; }
		public static sfloat FORCE_INV_SCALE(sfloat x) { return x; }
#endif

		public static readonly sfloat Pi = (sfloat)libm.pi;

		// Global tuning constants based on meters-kilograms-seconds (MKS) units.

		// Collision
		public static readonly int MaxManifoldPoints = 2;
		public static readonly int MaxPolygonVertices = 8;
		public static readonly int MaxProxies = 512; // this must be a power of two
		public static readonly int MaxPairs = 8 * MaxProxies; // this must be a power of two

		// Dynamics

		/// <summary>
		/// A small length used as a collision and constraint tolerance. Usually it is
		/// chosen to be numerically significant, but visually insignificant.
		/// </summary>
		public static readonly sfloat LinearSlop = (sfloat)0.005f;	// 0.5 cm

		/// <summary>
		/// A small angle used as a collision and constraint tolerance. Usually it is
		/// chosen to be numerically significant, but visually insignificant.
		/// </summary>
		public static readonly sfloat AngularSlop = (sfloat)2.0f / (sfloat)180.0f * Pi; // 2 degrees

		/// <summary>
		/// The radius of the polygon/edge shape skin. This should not be modified. Making
		/// this smaller means polygons will have and insufficient for continuous collision.
		/// Making it larger may create artifacts for vertex collision.
		/// </summary>
		public static readonly sfloat PolygonRadius = (sfloat)2.0f * LinearSlop;

		/// <summary>
		/// Continuous collision detection (CCD) works with core, shrunken shapes. This is amount
		/// by which shapes are automatically shrunk to work with CCD. 
		/// This must be larger than LinearSlop.
		/// </summary>
		public static readonly sfloat ToiSlop = (sfloat)8.0f * LinearSlop;

		/// <summary>
		/// Maximum number of contacts to be handled to solve a TOI island.
		/// </summary>
		public static readonly int MaxTOIContactsPerIsland = 32;

		/// <summary>
		/// Maximum number of joints to be handled to solve a TOI island.
		/// </summary>
		public static readonly int MaxTOIJointsPerIsland = 32;

		/// <summary>
		/// A velocity threshold for elastic collisions. Any collision with a relative linear
		/// velocity below this threshold will be treated as inelastic.
		/// </summary>
		public static readonly sfloat VelocityThreshold = sfloat.One; // 1 m/s

		/// <summary>
		/// The maximum linear position correction used when solving constraints.
		/// This helps to prevent overshoot.
		/// </summary>
		public static readonly sfloat MaxLinearCorrection = (sfloat)0.2f; // 20 cm

		/// <summary>
		/// The maximum angular position correction used when solving constraints.
		/// This helps to prevent overshoot.
		/// </summary>
		public static readonly sfloat MaxAngularCorrection = (sfloat)8.0f / (sfloat)180.0f * Pi; // 8 degrees

		/// <summary>
		/// The maximum linear velocity of a body. This limit is very large and is used
		/// to prevent numerical problems. You shouldn't need to adjust this.
		/// </summary>
#if TARGET_sfloat32_IS_FIXED
		public static readonly sfloat MaxLinearVelocity = (sfloat)100.0f;
#else
		public static readonly sfloat MaxLinearVelocity = (sfloat)200.0f;
		public static readonly sfloat MaxLinearVelocitySquared = MaxLinearVelocity * MaxLinearVelocity;
#endif
		/// <summary>
		/// The maximum angular velocity of a body. This limit is very large and is used
		/// to prevent numerical problems. You shouldn't need to adjust this.
		/// </summary>
		public static readonly sfloat MaxAngularVelocity = (sfloat)250.0f;
#if !TARGET_sfloat32_IS_FIXED
		public static readonly sfloat MaxAngularVelocitySquared = MaxAngularVelocity * MaxAngularVelocity;
#endif

		/// <summary>
		/// The maximum linear velocity of a body. This limit is very large and is used
		/// to prevent numerical problems. You shouldn't need to adjust this.
		/// </summary>
		public static readonly sfloat MaxTranslation = (sfloat)2.0f;
		public static readonly sfloat MaxTranslationSquared = (MaxTranslation * MaxTranslation);

		/// <summary>
		/// The maximum angular velocity of a body. This limit is very large and is used
		/// to prevent numerical problems. You shouldn't need to adjust this.
		/// </summary>
		public static readonly sfloat MaxRotation = ((sfloat)0.5f * Pi);
		public static readonly sfloat MaxRotationSquared = (MaxRotation * MaxRotation);

		/// <summary>
		/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
		/// that overlap is removed in one time step. However using values close to 1 often lead to overshoot.
		/// </summary>
		public static readonly sfloat ContactBaumgarte = (sfloat)0.2f;

		// Sleep

		/// <summary>
		/// The time that a body must be still before it will go to sleep.
		/// </summary>
		public static readonly sfloat TimeToSleep = (sfloat)0.5f; // half a second

		/// <summary>
		/// A body cannot sleep if its linear velocity is above this tolerance.
		/// </summary>
		public static readonly sfloat LinearSleepTolerance = (sfloat)0.01f; // 1 cm/s

		/// <summary>
		/// A body cannot sleep if its angular velocity is above this tolerance.
		/// </summary>
		public static readonly sfloat AngularSleepTolerance = (sfloat)2.0f / (sfloat)180.0f; // 2 degrees/s

		/// <summary>
		/// Friction mixing law. Feel free to customize this.
		/// </summary>
		public static sfloat MixFriction(sfloat friction1, sfloat friction2)
		{
			return (sfloat)libm.sqrtf(friction1 * friction2);
		}

		/// <summary>
		/// Restitution mixing law. Feel free to customize this.
		/// </summary>
		public static sfloat MixRestitution(sfloat restitution1, sfloat restitution2)
		{
			return restitution1 > restitution2 ? restitution1 : restitution2;
		}
	}
}
