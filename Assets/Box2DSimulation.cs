using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Box2DX;
using Box2DX.Dynamics;
using Box2DX.Collision;

public class Box2DSimulation : MonoBehaviour
{
    public static Box2DSimulation instance;
    public bool useCustomBox2D;
    public World world;
    public Vector2 gravity;
    public bool doSleep;
    public Vector2 center;
    public Vector2 lowerBound;
    public Vector2 upperBound;
    private int velocityIteration = 6;
    private int positionIteration = 2;

    public ContactListener contactListener;

    private void Awake(){
        instance = this;

        if(!useCustomBox2D){
            return;
        }

        var aabb = new AABB();
        aabb.LowerBound = lowerBound;
        aabb.UpperBound = upperBound;
        world = new World(aabb, gravity, doSleep);
        // world.GetGroundBody().SetPosition(Vector2.zero);

        contactListener = world._contactListener;
    }

    private void Start(){

    }

    private void FixedUpdate(){
        if(!useCustomBox2D){
            return;
        }
        // Debug.Log("Stepping");
        // Debug.Log(Time.fixedDeltaTime);
        // Debug.Log(world._broadPhase._worldAABB.Center);
        // Debug.Log(world._broadPhase._worldAABB.Extents);
        // Debug.Log(world.GetBodyCount());
        world.Step(Time.fixedDeltaTime, velocityIteration, positionIteration);

        // Debug.Log(world.GetContactCount());

        // foreach (var contact in world._contactList.)
        // {
            
        // }

        // Debug.Log(world._contactList);
    }

    
    private void OnDisable(){
        world.Dispose();
    }

}
