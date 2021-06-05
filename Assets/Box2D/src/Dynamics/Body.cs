﻿/*
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

using Box2DX.Common;
using Box2DX.Collision;

using UnityEngine;
using Transform = Box2DX.Common.Transform;
using SoftFloat;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// A body definition holds all the data needed to construct a rigid body.
	/// You can safely re-use body definitions.
	/// </summary>
	public struct BodyDef
	{
		/// <summary>
		/// This constructor sets the body definition default values.
		/// </summary>
		public BodyDef(byte init)
		{
			MassData = new MassData();
			MassData.Center = sVector2.zero;
			MassData.Mass = (sfloat)sfloat.Zero;
			MassData.I =(sfloat) sfloat.Zero;
			UserData = null;
			Position = sVector2.zero;
			Angle = (sfloat)sfloat.Zero;
			LinearVelocity = sVector2.zero; 
			AngularVelocity = (sfloat)sfloat.Zero;
			LinearDamping = (sfloat)sfloat.Zero;
			AngularDamping = (sfloat)sfloat.Zero;
			AllowSleep = true;
			IsSleeping = false;
			FixedRotation = false;
			IsBullet = false;
		}

		/// <summary>
		/// You can use this to initialized the mass properties of the body.
		/// If you prefer, you can set the mass properties after the shapes
		/// have been added using Body.SetMassFromShapes.
		/// </summary>
		public MassData MassData;

		/// <summary>
		/// Use this to store application specific body data.
		/// </summary>
		public object UserData;

		/// <summary>
		/// The world position of the body. Avoid creating bodies at the origin
		/// since this can lead to many overlapping shapes.
		/// </summary>
		public sVector2 Position;

		/// <summary>
		/// The world angle of the body in radians.
		/// </summary>
		public sfloat Angle;

		/// The linear velocity of the body in world co-ordinates.
		public sVector2 LinearVelocity;

		// The angular velocity of the body.
		public sfloat AngularVelocity;

		/// <summary>
		/// Linear damping is use to reduce the linear velocity. The damping parameter
		/// can be larger than sfloat.One but the damping effect becomes sensitive to the
		/// time step when the damping parameter is large.
		/// </summary>
		public sfloat LinearDamping;

		/// <summary>
		/// Angular damping is use to reduce the angular velocity. The damping parameter
		/// can be larger than sfloat.One but the damping effect becomes sensitive to the
		/// time step when the damping parameter is large.
		/// </summary>
		public sfloat AngularDamping;

		/// <summary>
		/// Set this flag to false if this body should never fall asleep. Note that
		/// this increases CPU usage.
		/// </summary>
		public bool AllowSleep;

		/// <summary>
		/// Is this body initially sleeping?
		/// </summary>
		public bool IsSleeping;

		/// <summary>
		/// Should this body be prevented from rotating? Useful for characters.
		/// </summary>
		public bool FixedRotation;

		/// <summary>
		/// Is this a fast moving body that should be prevented from tunneling through
		/// other moving bodies? Note that all bodies are prevented from tunneling through
		/// static bodies.
		/// @warning You should use this flag sparingly since it increases processing time.
		/// </summary>
		public bool IsBullet;
	}
	
	[System.Serializable]
	/// <summary>
	/// A rigid body. These are created via World.CreateBody.
	/// </summary>
	public class Body : IDisposable
	{
		[Flags]
		public enum BodyFlags
		{
			Frozen = 0x0002,
			Island = 0x0004,
			Sleep = 0x0008,
			AllowSleep = 0x0010,
			Bullet = 0x0020,
			FixedRotation = 0x0040
		}

		public enum BodyType
		{
			Static,
			Dynamic,
			MaxTypes
		}

		internal BodyFlags _flags;
		private BodyType _type;

		internal int _islandIndex;

		public Transform _xf;		// the body origin Transform

		public Sweep _sweep;	// the swept motion for CCD

		internal sVector2 _linearVelocity;
		internal sfloat _angularVelocity;

		internal sVector2 _force;
		internal sfloat _torque;

		private World _world;
		internal Body _prev;
		internal Body _next;

		internal Fixture _fixtureList;
		internal int _fixtureCount;

		internal JointEdge _jointList;
		internal ContactEdge _contactList;

		internal Controllers.ControllerEdge _controllerList;

		internal sfloat _mass;
		internal sfloat _invMass;
		internal sfloat _I;
		internal sfloat _invI;

		internal sfloat _linearDamping;
		internal sfloat _angularDamping;

		internal sfloat _sleepTime;

		private object _userData;
		
		public int bodyID;
		
		static int bodyCount = 0;
        private Box2DRigidbody _box2DRigidbody;

        public Box2DRigidbody Box2DRigidbody { get => _box2DRigidbody; }

        internal Body(BodyDef bd, World world)
		{
			Box2DXDebug.Assert(world._lock == false);

			_flags = 0;

			if (bd.IsBullet)
			{
				_flags |= BodyFlags.Bullet;
			}
			if (bd.FixedRotation)
			{
				_flags |= BodyFlags.FixedRotation;
			}
			if (bd.AllowSleep)
			{
				_flags |= BodyFlags.AllowSleep;
			}
			if (bd.IsSleeping)
			{
				_flags |= BodyFlags.Sleep;
			}

			_world = world;

			_xf.position = bd.Position;
			_xf.rotation = Box2DX.Common.Math.AngleToRotation(bd.Angle);
			//_xf.R = new Mat22(bd.Angle);

			_sweep.LocalCenter = bd.MassData.Center;
			_sweep.T0 = sfloat.One;
			_sweep.A0 = _sweep.A = bd.Angle;
			_sweep.C0 = _sweep.C = _xf.TransformPoint(_sweep.LocalCenter);

			//_jointList = null;
			//_contactList = null;
			//_controllerList = null;
			//_prev = null;
			//_next = null;

			_linearVelocity = bd.LinearVelocity;
			_angularVelocity = bd.AngularVelocity;

			_linearDamping = bd.LinearDamping;
			_angularDamping = bd.AngularDamping;

			//_force.Set(sfloat.Zero, sfloat.Zero);
			//_torque = sfloat.Zero;

			//_linearVelocity.SetZero();
			//_angularVelocity = sfloat.Zero;

			//_sleepTime = sfloat.Zero;

			//_invMass = sfloat.Zero;
			//_I = sfloat.Zero;
			//_invI = sfloat.Zero;

			_mass = bd.MassData.Mass;

			if (_mass > sfloat.Zero)
			{
				_invMass = sfloat.One / _mass;
			}

			_I = bd.MassData.I;

			if (_I > sfloat.Zero && (_flags & BodyFlags.FixedRotation) == 0)
			{
				_invI = sfloat.One / _I;
			}

			if (_invMass == sfloat.Zero && _invI == sfloat.Zero)
			{
				_type = BodyType.Static;
			}
			else
			{
				_type = BodyType.Dynamic;
			}

			_userData = bd.UserData;

			//_fixtureList = null;
			//_fixtureCount = 0;
			
			bodyID = bodyCount++;
		}

		public void Dispose()
		{
			Box2DXDebug.Assert(_world._lock == false);
			// shapes and joints are destroyed in World.Destroy
		}

		internal bool SynchronizeFixtures()
		{
			Transform xf1 = new Transform();
			xf1.rotation = Box2DX.Common.Math.AngleToRotation(_sweep.A0);
			//xf1.R = new Mat22(_sweep.A0);
			xf1.position = _sweep.C0 - xf1.TransformDirection(_sweep.LocalCenter);

			bool inRange = true;
			for (Fixture f = _fixtureList; f != null; f = f.Next)
			{
				inRange = f.Synchronize(_world._broadPhase, xf1, _xf);
				if (inRange == false)
				{
					break;
				}
			}

			if (inRange == false)
			{
				_flags |= BodyFlags.Frozen;
				_linearVelocity = sVector2.zero;
				_angularVelocity = sfloat.Zero;

				// Failure
				return false;
			}

			// Success
			return true;
		}

		// This is used to prevent connected bodies from colliding.
		// It may lie, depending on the collideConnected flag.
		internal bool IsConnected(Body other)
		{
			for (JointEdge jn = _jointList; jn != null; jn = jn.Next)
			{
				if (jn.Other == other)
					return jn.Joint._collideConnected == false;
			}

			return false;
		}

		/// <summary>
		/// Creates a fixture and attach it to this body.
		/// @warning This function is locked during callbacks.
		/// </summary>
		/// <param name="def">The fixture definition.</param>
		public Fixture CreateFixture(FixtureDef def)
		{
			Box2DXDebug.Assert(_world._lock == false);
			if (_world._lock == true)
			{
				return null;
			}

			BroadPhase broadPhase = _world._broadPhase;

			Fixture fixture = new Fixture();
			fixture.Create(broadPhase, this, _xf, def);

			fixture._next = _fixtureList;
			_fixtureList = fixture;
			++_fixtureCount;

			fixture._body = this;

			return fixture;
		}

		/// <summary>
		/// Destroy a fixture. This removes the fixture from the broad-phase and
		/// therefore destroys any contacts associated with this fixture. All fixtures
		/// attached to a body are implicitly destroyed when the body is destroyed.
		/// @warning This function is locked during callbacks.
		/// </summary>
		/// <param name="fixture">The fixture to be removed.</param>
		public void DestroyFixture(Fixture fixture)
		{
			Box2DXDebug.Assert(_world._lock == false);
			if (_world._lock == true)
			{
				return;
			}

			Box2DXDebug.Assert(fixture.Body == this);

			// Remove the fixture from this body's singly linked list.
			Box2DXDebug.Assert(_fixtureCount > 0);
			Fixture node = _fixtureList;
			bool found = false;
			while (node != null)
			{
				if (node == fixture)
				{
					//*node = fixture->m_next;
					_fixtureList = fixture.Next;
					found = true;
					break;
				}

				node = node.Next;
			}

			// You tried to remove a shape that is not attached to this body.
			Box2DXDebug.Assert(found);

			BroadPhase broadPhase = _world._broadPhase;

			fixture.Destroy(broadPhase);
			fixture._body = null;
			fixture._next = null;

			--_fixtureCount;
		}

		// TODO_ERIN adjust linear velocity and torque to account for movement of center.
		/// <summary>
		/// Set the mass properties. Note that this changes the center of mass position.
		/// If you are not sure how to compute mass properties, use SetMassFromShapes.
		/// The inertia tensor is assumed to be relative to the center of mass.
		/// </summary>
		/// <param name="massData">The mass properties.</param>
		public void SetMass(MassData massData)
		{
			Box2DXDebug.Assert(_world._lock == false);
			if (_world._lock == true)
			{
				return;
			}

			_invMass = sfloat.Zero;
			_I = sfloat.Zero;
			_invI = sfloat.Zero;

			_mass = massData.Mass;

			if (_mass > sfloat.Zero)
			{
				_invMass = sfloat.One / _mass;
			}

			_I = massData.I;

			if (_I > sfloat.Zero && (_flags & BodyFlags.FixedRotation) == 0)
			{
				_invI = sfloat.One / _I;
			}

			// Move center of mass.
			_sweep.LocalCenter = massData.Center;
			_sweep.C0 = _sweep.C = _xf.TransformPoint(_sweep.LocalCenter);

			BodyType oldType = _type;
			if (_invMass == sfloat.Zero && _invI == sfloat.Zero)
			{
				_type = BodyType.Static;
			}
			else
			{
				_type = BodyType.Dynamic;
			}

			// If the body type changed, we need to refilter the broad-phase proxies.
			if (oldType != _type)
			{
				for (Fixture f = _fixtureList; f != null; f = f.Next)
				{
					f.RefilterProxy(_world._broadPhase, _xf);
				}
			}
		}

		// TODO_ERIN adjust linear velocity and torque to account for movement of center.
		/// <summary>
		/// Compute the mass properties from the attached shapes. You typically call this
		/// after adding all the shapes. If you add or remove shapes later, you may want
		/// to call this again. Note that this changes the center of mass position.
		/// </summary>
		public void SetMassFromShapes()
		{	
			Box2DXDebug.Assert(_world._lock == false);
			if (_world._lock == true)
			{
				return;
			}

			// Compute mass data from shapes. Each shape has its own density.
			_mass = sfloat.Zero;
			_invMass = sfloat.Zero;
			_I = sfloat.Zero;
			_invI = sfloat.Zero;

			sVector2 center = sVector2.zero;
			for (Fixture f = _fixtureList; f != null; f = f.Next)
			{
				MassData massData;
				f.ComputeMass(out massData);
				_mass += massData.Mass;
				center += massData.Mass * massData.Center;
				_I += massData.I;
			}

			// Compute center of mass, and shift the origin to the COM.
			if (_mass > sfloat.Zero)
			{
				_invMass = sfloat.One / _mass;
				center *= _invMass;
			}

			if (_I > sfloat.Zero && (_flags & BodyFlags.FixedRotation) == 0)
			{
				// Center the inertia about the center of mass.
				_I -= _mass * sVector2.Dot(center, center);
				Box2DXDebug.Assert(_I > sfloat.Zero);
				_invI = sfloat.One / _I;
			}
			else
			{
				_I = sfloat.Zero;
				_invI = sfloat.Zero;
			}

			// Move center of mass.
			_sweep.LocalCenter = center;
			_sweep.C0 = _sweep.C = _xf.TransformPoint(_sweep.LocalCenter);

			BodyType oldType = _type;
			if (_invMass == sfloat.Zero && _invI == sfloat.Zero)
			{
				_type = BodyType.Static;
			}
			else
			{
				_type = BodyType.Dynamic;
			}

			// If the body type changed, we need to refilter the broad-phase proxies.
			if (oldType != _type)
			{
				for (Fixture f = _fixtureList; f != null; f = f.Next)
				{
					f.RefilterProxy(_world._broadPhase, _xf);
				}
			}
		}
		
		public bool SetTransform(sVector2 position, sfloat angle) 
		{
			return SetTransform(position, Box2DX.Common.Math.AngleToRotation(angle));
		}
		
#if USE_MATRIX_FOR_ROTATION
		/// <summary>
		/// Set the position of the body's origin and rotation (radians).
		/// This breaks any contacts and wakes the other bodies.
		/// </summary>
		/// <param name="position">The new world position of the body's origin (not necessarily
		/// the center of mass).</param>
		/// <param name="angle">The new world rotation angle of the body in radians.</param>
		/// <returns>Return false if the movement put a shape outside the world. In this case the
		/// body is automatically frozen.</returns>
		public bool SetTransform(sVector2 position, Mat22 rotation)
#else
		public bool SetTransform(sVector2 position, Quaternion rotation)
#endif
		{
			Box2DXDebug.Assert(_world._lock == false);
			if (_world._lock == true)
			{
				return true;
			}

			if (IsFrozen())
			{
				return false;
			}
			
			_xf.rotation = rotation;
			//_xf.R = rotation;
			_xf.position = position;

			_sweep.C0 = _sweep.C = _xf.TransformPoint(_sweep.LocalCenter);
#if USE_MATRIX_FOR_ROTATION
			_sweep.A0 = _sweep.A = rotation.GetAngle();
#else
			_sweep.A0 = _sweep.A = rotation.eulerAngles.z * Mathf.Deg2Rad;
#endif

			bool freeze = false;
			for (Fixture f = _fixtureList; f != null; f = f.Next)
			{
				bool inRange = f.Synchronize(_world._broadPhase, _xf, _xf);

				if (inRange == false)
				{
					freeze = true;
					break;
				}
			}

			if (freeze == true)
			{
				_flags |= BodyFlags.Frozen;
				_linearVelocity = sVector2.zero;
				_angularVelocity = sfloat.Zero;

				// Failure
				return false;
			}

			// Success
			_world._broadPhase.Commit();
			return true;
		}

		/// <summary>
		/// Set the position of the body's origin and rotation (radians).
		/// This breaks any contacts and wakes the other bodies.
		/// Note this is less efficient than the other overload - you should use that
		/// if the angle is available.
		/// </summary>
		/// <param name="xf">The Transform of position and angle to set the body to.</param>
		/// <returns>False if the movement put a shape outside the world. In this case the
		/// body is automatically frozen.</returns>
		public bool SetTransform(Transform xf)
		{
			return SetTransform(xf.position, xf.rotation);
		}

		/// <summary>
		/// Get the body Transform for the body's origin.
		/// </summary>
		/// <returns>Return the world Transform of the body's origin.</returns>
		public Transform GetTransform()
		{
			return _xf;
		}

		/// <summary>
		/// Set the world body origin position.
		/// </summary>
		/// <param name="position">The new position of the body.</param>
		public void SetPosition(sVector2 position)
		{
#if USE_MATRIX_FOR_ROTATION
			SetTransform(position, new Mat22(GetAngle()));
#else
			SetTransform(position, Box2DX.Common.Math.AngleToRotation(GetAngle()));
#endif
		}

		/// <summary>
		/// Set the world body angle.
		/// </summary>
		/// <param name="angle">The new angle of the body in radians</param>
		public void SetAngle(sfloat angle)
		{
#if USE_MATRIX_FOR_ROTATION
			SetTransform(GetPosition(), new Mat22(angle));
#else
			SetTransform(GetPosition(), Box2DX.Common.Math.AngleToRotation(angle));
#endif
		}

		/// <summary>
		/// Get the world body origin position.
		/// </summary>
		/// <returns>Return the world position of the body's origin.</returns>
		public sVector2 GetPosition()
		{
			return _xf.position;
		}

		/// <summary>
		/// Get the angle in radians.
		/// </summary>
		/// <returns>Return the current world rotation angle in radians.</returns>
		public sfloat GetAngle()
		{
			return _sweep.A;
		}

		/// <summary>
		/// Get the world position of the center of mass.
		/// </summary>
		/// <returns></returns>
		public sVector2 GetWorldCenter()
		{
			return _sweep.C;
		}

		/// <summary>
		/// Get the local position of the center of mass.
		/// </summary>
		/// <returns></returns>
		public sVector2 GetLocalCenter()
		{
			return _sweep.LocalCenter;
		}

		/// <summary>
		/// Set the linear velocity of the center of mass.
		/// </summary>
		/// <param name="v">The new linear velocity of the center of mass.</param>
		public void SetLinearVelocity(sVector2 v)
		{
			_linearVelocity = v;
		}

		/// <summary>
		/// Get the linear velocity of the center of mass.
		/// </summary>
		/// <returns>Return the linear velocity of the center of mass.</returns>
		public sVector2 GetLinearVelocity()
		{
			return _linearVelocity;
		}

		/// <summary>
		/// Set the angular velocity.
		/// </summary>
		/// <param name="omega">The new angular velocity in radians/second.</param>
		public void SetAngularVelocity(sfloat w)
		{
			_angularVelocity = w;
		}

		/// <summary>
		/// Get the angular velocity.
		/// </summary>
		/// <returns>Return the angular velocity in radians/second.</returns>
		public sfloat GetAngularVelocity()
		{
			return _angularVelocity;
		}

		/// <summary>
		/// Apply a force at a world point. If the force is not
		/// applied at the center of mass, it will generate a torque and
		/// affect the angular velocity. This wakes up the body.
		/// </summary>
		/// <param name="force">The world force vector, usually in Newtons (N).</param>
		/// <param name="point">The world position of the point of application.</param>
		public void ApplyForce(sVector2 force, sVector2 point)
		{
			if (IsSleeping())
			{
				WakeUp();
			}
			
			_force += force;
			_torque += (point - _sweep.C).Cross(force);
		}

		/// <summary>
		/// Apply a torque. This affects the angular velocity
		/// without affecting the linear velocity of the center of mass.
		/// This wakes up the body.
		/// </summary>
		/// <param name="torque">Torque about the z-axis (out of the screen), usually in N-m.</param>
		public void ApplyTorque(sfloat torque)
		{
			if (IsSleeping())
			{
				WakeUp();
			}
			_torque += torque;
		}

		/// <summary>
		/// Apply an impulse at a point. This immediately modifies the velocity.
		/// It also modifies the angular velocity if the point of application
		/// is not at the center of mass. This wakes up the body.
		/// </summary>
		/// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
		/// <param name="point">The world position of the point of application.</param>
		public void ApplyImpulse(sVector2 impulse, sVector2 point)
		{
			if (IsSleeping())
			{
				WakeUp();
			}
		
			_linearVelocity += _invMass * impulse;
			_angularVelocity += _invI * (point - _sweep.C).Cross(impulse);
		}

		/// <summary>
		/// Get the total mass of the body.
		/// </summary>
		/// <returns>Return the mass, usually in kilograms (kg).</returns>
		public sfloat GetMass()
		{
			return _mass;
		}

		/// <summary>
		/// Get the central rotational inertia of the body.
		/// </summary>
		/// <returns>Return the rotational inertia, usually in kg-m^2.</returns>
		public sfloat GetInertia()
		{
			return _I;
		}

		/// <summary>
		/// Get the mass data of the body.
		/// </summary>
		/// <returns>A struct containing the mass, inertia and center of the body.</returns>
		public MassData GetMassData()
		{
			MassData massData = new MassData();
			massData.Mass = _mass;
			massData.I = _I;
			massData.Center = GetWorldCenter();
			return massData;
		}

		/// <summary>
		/// Get the world coordinates of a point given the local coordinates.
		/// </summary>
		/// <param name="localPoint">A point on the body measured relative the the body's origin.</param>
		/// <returns>Return the same point expressed in world coordinates.</returns>
		public sVector2 GetWorldPoint(sVector2 localPoint)
		{
			return _xf.TransformPoint(localPoint);
		}

		/// <summary>
		/// Get the world coordinates of a vector given the local coordinates.
		/// </summary>
		/// <param name="localVector">A vector fixed in the body.</param>
		/// <returns>Return the same vector expressed in world coordinates.</returns>
		public sVector2 GetWorldVector(sVector2 localVector)
		{
			return _xf.TransformDirection(localVector);
		}

		/// <summary>
		/// Gets a local point relative to the body's origin given a world point.
		/// </summary>
		/// <param name="worldPoint">A point in world coordinates.</param>
		/// <returns>Return the corresponding local point relative to the body's origin.</returns>
		public sVector2 GetLocalPoint(sVector2 worldPoint)
		{
			return _xf.InverseTransformPoint(worldPoint);
		}

		/// <summary>
		/// Gets a local vector given a world vector.
		/// </summary>
		/// <param name="worldVector">A vector in world coordinates.</param>
		/// <returns>Return the corresponding local vector.</returns>
		public sVector2 GetLocalVector(sVector2 worldVector)
		{
			return _xf.InverseTransformDirection(worldVector);
		}

		/// <summary>
		/// Get the world linear velocity of a world point attached to this body.
		/// </summary>
		/// <param name="worldPoint">A point in world coordinates.</param>
		/// <returns>The world velocity of a point.</returns>
		public sVector2 GetLinearVelocityFromWorldPoint(sVector2 worldPoint)
		{
			return _linearVelocity + (worldPoint - _sweep.C).CrossScalarPreMultiply(_angularVelocity);
		}

		/// <summary>
		/// Get the world velocity of a local point.
		/// </summary>
		/// <param name="localPoint">A point in local coordinates.</param>
		/// <returns>The world velocity of a point.</returns>
		public sVector2 GetLinearVelocityFromLocalPoint(sVector2 localPoint)
		{
			return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
		}

		public sfloat GetLinearDamping()
		{
			return _linearDamping;
		}

		public void SetLinearDamping(sfloat linearDamping)
		{
			_linearDamping = linearDamping;
		}

		public sfloat GetAngularDamping()
		{
			return _angularDamping;
		}

		public void SetAngularDamping(sfloat angularDamping)
		{
			_angularDamping = angularDamping;
		}

		/// <summary>
		/// Is this body treated like a bullet for continuous collision detection?
		/// </summary>
		/// <returns></returns>
		public bool IsBullet()
		{
			return (_flags & BodyFlags.Bullet) == BodyFlags.Bullet;
		}

		/// <summary>
		/// Should this body be treated like a bullet for continuous collision detection?
		/// </summary>
		/// <param name="flag"></param>
		public void SetBullet(bool flag)
		{
			if (flag)
			{
				_flags |= BodyFlags.Bullet;
			}
			else
			{
				_flags &= ~BodyFlags.Bullet;
			}
		}

		public bool IsFixedRotation()
		{
			return (_flags & BodyFlags.FixedRotation) == BodyFlags.FixedRotation;
		}

		public void SetFixedRotation(bool fixedr)
		{
			if (fixedr)
			{
				_angularVelocity = sfloat.Zero;
				_invI = sfloat.Zero;
				_flags |= BodyFlags.FixedRotation;
			}
			else
			{
				if (_I > sfloat.Zero)
				{
					// Recover _invI from _I.
					_invI = sfloat.One / _I;
					_flags &= BodyFlags.FixedRotation;
				}
				// TODO: Else what?
			}
		}

		/// <summary>
		/// Is this body static (immovable)?
		/// </summary>
		/// <returns></returns>
		public bool IsStatic()
		{
			return _type == BodyType.Static;
		}

		public void SetStatic()
		{
			if (_type == BodyType.Static)
				return;
			_mass = sfloat.Zero;
			_invMass = sfloat.Zero;
			_I = sfloat.Zero;
			_invI = sfloat.Zero;
			_type = BodyType.Static;

			for (Fixture f = _fixtureList; f != null; f = f.Next)
			{
				f.RefilterProxy(_world._broadPhase, _xf);
			}
		}

		/// <summary>
		/// Is this body dynamic (movable)?
		/// </summary>
		/// <returns></returns>
		public bool IsDynamic()
		{
			return _type == BodyType.Dynamic;
		}

		/// <summary>
		/// Is this body frozen?
		/// </summary>
		/// <returns></returns>
		public bool IsFrozen()
		{
			return (_flags & BodyFlags.Frozen) == BodyFlags.Frozen;
		}

		/// <summary>
		/// Is this body sleeping (not simulating).
		/// </summary>
		/// <returns></returns>
		public bool IsSleeping()
		{
			return (_flags & BodyFlags.Sleep) == BodyFlags.Sleep;
		}

		public bool IsAllowSleeping()
		{
			return (_flags & BodyFlags.AllowSleep) == BodyFlags.AllowSleep;
		}

		/// <summary>
		/// You can disable sleeping on this body.
		/// </summary>
		/// <param name="flag"></param>
		public void AllowSleeping(bool flag)
		{
			if (flag)
			{
				_flags |= BodyFlags.AllowSleep;
			}
			else
			{
				_flags &= ~BodyFlags.AllowSleep;
				WakeUp();
			}
		}

		/// <summary>
		/// Wake up this body so it will begin simulating.
		/// </summary>
		public void WakeUp()
		{
			_flags &= ~BodyFlags.Sleep;
			_sleepTime = sfloat.Zero;
		}

		/// <summary>
		/// Put this body to sleep so it will stop simulating.
		/// This also sets the velocity to zero.
		/// </summary>
		public void PutToSleep()
		{
			_flags |= BodyFlags.Sleep;
			_sleepTime = sfloat.Zero;
			_linearVelocity = sVector2.zero;
			_angularVelocity = sfloat.Zero;
			_force = sVector2.zero;
			_torque = sfloat.Zero;
		}

		/// <summary>
		/// Get the list of all fixtures attached to this body.
		/// </summary>
		/// <returns></returns>
		public Fixture GetFixtureList()
		{
			return _fixtureList;
		}

		/// <summary>
		/// Get the list of all joints attached to this body.
		/// </summary>
		/// <returns></returns>
		public JointEdge GetJointList()
		{
			return _jointList;
		}

		public Controllers.ControllerEdge GetControllerList()
		{
			return _controllerList;
		}

		/// <summary>
		/// Get the next body in the world's body list.
		/// </summary>
		/// <returns></returns>
		public Body GetNext()
		{
			return _next;
		}

		/// <summary>
		/// Get the user data pointer that was provided in the body definition.
		/// </summary>
		/// <returns></returns>
		public object GetUserData()
		{
			return _userData;
		}

		/// <summary>
		/// Set the user data. Use this to store your application specific data.
		/// </summary>
		/// <param name="data"></param>
		public void SetUserData(object data) { _userData = data; }

		/// <summary>
		/// Get the parent world of this body.
		/// </summary>
		/// <returns></returns>
		public World GetWorld() { return _world; }

		internal void SynchronizeTransform()
		{
			//_xf.R = new Mat22(_sweep.A);
			_xf.rotation = Box2DX.Common.Math.AngleToRotation(_sweep.A);
			_xf.position = _sweep.C - _xf.TransformDirection(_sweep.LocalCenter);//Common.Math.Mul(_xf.R, _sweep.LocalCenter);
		}

		internal void Advance(sfloat t)
		{
			// Advance to the new safe time.
			_sweep.Advance(t);
			_sweep.C = _sweep.C0;
			_sweep.A = _sweep.A0;
			SynchronizeTransform();
		}

        internal void SetBehavior(Box2DRigidbody box2DRigidbody)
        {
            _box2DRigidbody = box2DRigidbody;
        }
    }
}
