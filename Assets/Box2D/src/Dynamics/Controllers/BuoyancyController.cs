﻿using Box2DX.Common;
using SoftFloat;
using UnityEngine;

namespace Box2DX.Dynamics.Controllers
{
    /// <summary>
    /// This class is used to build buoyancy controllers
    /// </summary>
    public class BuoyancyControllerDef
    {
        /// The outer surface normal
        public sVector2 Normal;
        /// The height of the fluid surface along the normal
        public sfloat Offset;
        /// The fluid density
        public sfloat Density;
        /// Fluid velocity, for drag calculations
        public sVector2 Velocity;
        /// Linear drag co-efficient
        public sfloat LinearDrag;
        /// Linear drag co-efficient
        public sfloat AngularDrag;
        /// If false, bodies are assumed to be uniformly dense, otherwise use the shapes densities
        public bool UseDensity; //False by default to prevent a gotcha
        /// If true, gravity is taken from the world instead of the gravity parameter.
        public bool UseWorldGravity;
        /// Gravity vector, if the world's gravity is not used
        public sVector2 Gravity;

        public BuoyancyControllerDef()
        {
            Normal = new sVector2(sfloat.Zero, sfloat.One);
            Offset = sfloat.Zero;
            Density = sfloat.Zero;
            Velocity = sVector2.zero;
            LinearDrag = sfloat.Zero;
            AngularDrag = sfloat.Zero;
            UseDensity = false;
            UseWorldGravity = true;
            Gravity =  sVector2.zero;
        }
    }

    /// <summary>
    /// Calculates buoyancy forces for fluids in the form of a half plane.
    /// </summary>
    public class BuoyancyController : Controller
    {
        /// The outer surface normal
        public sVector2 Normal;
        /// The height of the fluid surface along the normal
        public sfloat Offset;
        /// The fluid density
        public sfloat Density;
        /// Fluid velocity, for drag calculations
        public sVector2 Velocity;
        /// Linear drag co-efficient
        public sfloat LinearDrag;
        /// Linear drag co-efficient
        public sfloat AngularDrag;
        /// If false, bodies are assumed to be uniformly dense, otherwise use the shapes densities
        public bool UseDensity; //False by default to prevent a gotcha
        /// If true, gravity is taken from the world instead of the gravity parameter.
        public bool UseWorldGravity;
        /// Gravity vector, if the world's gravity is not used
        public sVector2 Gravity;

        public BuoyancyController(BuoyancyControllerDef buoyancyControllerDef)
        {
            Normal = buoyancyControllerDef.Normal;
            Offset = buoyancyControllerDef.Offset;
            Density = buoyancyControllerDef.Density;
            Velocity = buoyancyControllerDef.Velocity;
            LinearDrag = buoyancyControllerDef.LinearDrag;
            AngularDrag = buoyancyControllerDef.AngularDrag;
            UseDensity = buoyancyControllerDef.UseDensity;
            UseWorldGravity = buoyancyControllerDef.UseWorldGravity;
            Gravity = buoyancyControllerDef.Gravity;
        }

        public override void Step(TimeStep step)
        {
            //B2_NOT_USED(step);
            if (_bodyList == null)
                return;

            if (UseWorldGravity)
            {
                Gravity = _world.Gravity;
            }
            for (ControllerEdge i = _bodyList; i != null; i = i.nextBody)
            {
                Body body = i.body;
                if (body.IsSleeping())
                {
                    //Buoyancy force is just a function of position,
                    //so unlike most forces, it is safe to ignore sleeping bodes
                    continue;
                }
                sVector2 areac = sVector2.zero; 
                sVector2 massc = sVector2.zero;
                sfloat area = sfloat.Zero;
                sfloat mass = sfloat.Zero;
                for (Fixture shape = body.GetFixtureList(); shape != null; shape = shape.Next)
                {
                    sVector2 sc;
                    sfloat sarea = shape.ComputeSubmergedArea(Normal, Offset, out sc);
                    area += sarea;
                    areac.x += sarea * sc.x;
                    areac.y += sarea * sc.y;
                    sfloat shapeDensity = sfloat.Zero;
                    if (UseDensity)
                    {
                        //TODO: Expose density publicly
                        shapeDensity = shape.Density;
                    }
                    else
                    {
                        shapeDensity = sfloat.One;
                    }
                    mass += sarea * shapeDensity;
                    massc.x += sarea * sc.x * shapeDensity;
                    massc.y += sarea * sc.y * shapeDensity;
                }
                areac.x /= area;
                areac.y /= area;
                //Vec2 localCentroid = Math.MulT(body.GetTransform(), areac);
                massc.x /= mass;
                massc.y /= mass;
                if (area < Settings.FLT_EPSILON)
                    continue;
                //Buoyancy
                sVector2 buoyancyForce = -Density * area * Gravity;
                body.ApplyForce(buoyancyForce, massc);
                //Linear drag
                sVector2 dragForce = body.GetLinearVelocityFromWorldPoint(areac) - Velocity;
                dragForce *= -LinearDrag * area;
                body.ApplyForce(dragForce, areac);
                //Angular drag
                //TODO: Something that makes more physical sense?
                body.ApplyTorque(-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * AngularDrag);
            
            }
        }

        public override void Draw(DebugDraw debugDraw)
        {
            sfloat r = (sfloat)1000;
            sVector2 p1 = Offset * Normal + Normal.CrossScalarPostMultiply(r);
            sVector2 p2 = Offset * Normal - Normal.CrossScalarPostMultiply(r);

            Color color = new Color(0.0f, 0.0f, 0.8f);

            debugDraw.DrawSegment(p1, p2, color);
        }
    }
}