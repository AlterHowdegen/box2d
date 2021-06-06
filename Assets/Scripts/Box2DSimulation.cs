using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Box2DX;
using Box2DX.Dynamics;
using Box2DX.Collision;
using SoftFloat;
using System.Threading;
using System;

public class Box2DSimulation : MonoBehaviour
{
    public static Box2DSimulation instance;
    public bool useCustomBox2D;
    public bool useMultiThreading;
    public World world;
    public sVector2 gravity = new sVector2(0f, -10f);
    public bool doSleep;
    public sVector2 center = sVector2.zero;
    public sVector2 lowerBound = new sVector2(-80f, -80f);
    public sVector2 upperBound = new sVector2(80f, 80f);
    private int velocityIteration = 6;
    private int positionIteration = 2;
    public sfloat timestep = (sfloat)0.02f;
    [SerializeField] private Box2DRigidbody[] _box2DRigidbodies;
    internal bool _initialized;
    private ThreadStart childref;
    private Thread childThread;
    private bool _threadRunning;
    private List<Box2DRigidbody> _bodiesToAdd;
    private float _lastUpdateTimestamp;

    public float LastUpdateTimestamp { get => _lastUpdateTimestamp; }

    private void Awake(){
        instance = this;

        if(!useCustomBox2D){
            return;
        }

        var aabb = new AABB();
        aabb.LowerBound = lowerBound;
        aabb.UpperBound = upperBound;
        world = new World(aabb, gravity, doSleep);
        // world.GetGroundBody().SetPosition(sVector2.zero);

        Time.fixedDeltaTime = (float)timestep;

        _bodiesToAdd = new List<Box2DRigidbody>();

        SetBodies();
    }

    // Execute at design time
    public void GatherBodies(){
        _box2DRigidbodies = FindObjectsOfType<Box2DRigidbody>();
    }

    // Execute at runtime
    private void SetBodies(){
        foreach (var box2DRigidbody in _box2DRigidbodies)
        {
            box2DRigidbody.GatherParts();
            SetBody(box2DRigidbody);
        }
    }

    // Only use this to add bodies to the simulation after it has already been initialized
    public void AddBody(Box2DRigidbody box2DRigidbody){
        if(!Box2DSimulation.instance.useCustomBox2D){
            return;
        }

        box2DRigidbody.GatherParts();
        _bodiesToAdd.Add(box2DRigidbody);
    }

    // Execute at runtime
    private void SetBody(Box2DRigidbody box2DRigidbody){
        if(!Box2DSimulation.instance.useCustomBox2D){
            return;
        }

        var originalRigidbody = box2DRigidbody._originalRigidbody;
        var originalCollider = box2DRigidbody._originalCollider;

        Debug.Log(originalRigidbody);
        Debug.Log(originalCollider);

        // Create body

        var bodyDefinition = new BodyDef();
        bodyDefinition.MassData = new MassData();
        bodyDefinition.MassData.Mass = (sfloat)originalRigidbody.mass < (sfloat)0.1f ? sfloat.Zero : (sfloat)originalRigidbody.mass;
        bodyDefinition.MassData.I = (sfloat)originalRigidbody.mass < (sfloat)0.1f ? sfloat.Zero : (sfloat)originalRigidbody.mass;
        bodyDefinition.Position = new sVector2(box2DRigidbody.transform.position.x, box2DRigidbody.transform.position.y);
        bodyDefinition.Angle = (sfloat)box2DRigidbody.transform.eulerAngles.z * libm.Deg2Rad;
        // bodyDefinition.FixedRotation = _originalRigidbody.freezeRotation;
        // bodyDefinition.AngularDamping = _originalRigidbody.angularDrag;
        bodyDefinition.FixedRotation = false;
        bodyDefinition.LinearDamping = (sfloat)originalRigidbody.drag;
        bodyDefinition.AngularDamping = (sfloat)originalRigidbody.angularDrag;
        // bodyDefinition.AngularVelocity= 1f;
        // bodyDefinition.
        bodyDefinition.AllowSleep = true;
        // bodyDefinition.AngularVelocity = 10f;

        // bodyDefinition.
        
        var body = Box2DSimulation.instance.world.CreateBody(bodyDefinition);

        // Create fixture

        if(originalCollider is BoxCollider2D){
            Debug.Log("Creating polygon fixture");
            var fixtureDefinition = new PolygonDef();
            fixtureDefinition.Density = (sfloat)originalCollider.density;
            fixtureDefinition.Friction = (sfloat)0.6f;
            fixtureDefinition.Restitution = (sfloat)0.5f; 
            fixtureDefinition.IsSensor = originalCollider.isTrigger;
            fixtureDefinition.SetAsBox((sfloat)(((BoxCollider2D)originalCollider).size.x) / (sfloat)2f, (sfloat)(((BoxCollider2D)originalCollider).size.y) / (sfloat)2f);
            body.SetBehavior(box2DRigidbody);
            body.CreateFixture(fixtureDefinition);
        }else if(originalCollider is CircleCollider2D){
            Debug.Log("Creating circle fixture");
            var fixtureDefinition = new CircleDef();
            fixtureDefinition.Density = (sfloat)originalCollider.density;
            fixtureDefinition.Friction = (sfloat)0.6f;
            fixtureDefinition.Restitution = (sfloat)0.5f;
            fixtureDefinition.IsSensor = originalCollider.isTrigger;
            fixtureDefinition.Radius = (sfloat)((CircleCollider2D)originalCollider).radius;
            body.SetBehavior(box2DRigidbody);
            body.CreateFixture(fixtureDefinition);
        }else{
            Debug.Log("Creating generic fixture");
            var fixtureDefinition = new FixtureDef();
            body.CreateFixture(fixtureDefinition);
        }

        box2DRigidbody.SetBody(body);

        // Set contact listener

        Box2DSimulation.instance.world.SetContactListener(box2DRigidbody);

        // Destroy unity 2D physics components
        
        Destroy(originalRigidbody);
        Destroy(originalCollider);
    }

    public void RemoveBody(Box2DRigidbody box2DRigidbody){
        world.DestroyBody(box2DRigidbody.Body);
        // box2DRigidbody.Body
    }

    private void Start(){

    }

    private void FixedUpdate(){
        if(!useCustomBox2D){
            return;
        }

        _lastUpdateTimestamp = Time.timeSinceLevelLoad;

        // Debug.Log("Stepping");
        // Debug.Log(Time.fixedDeltaTime);
        // Debug.Log(world._broadPhase._worldAABB.Center);
        // Debug.Log(world._broadPhase._worldAABB.Extents);
        // Debug.Log(world.GetBodyCount());

        // childref = new ThreadStart(CallToChildThread);

        // Make sure bodies are only added when the world is not locked.

        foreach (var body in _bodiesToAdd)
        {
            SetBody(body);
        }

        if(_bodiesToAdd.Count > 0){
            _bodiesToAdd.Clear();
        }


        // Debug.Log("In Main: Creating the Child thread");

        if(useMultiThreading){
            if(_threadRunning){
                Debug.Log("Previous thread not finished");
                return;
            }
            ThreadPool.QueueUserWorkItem(StepThread);
        }else{
            world.Step(timestep, velocityIteration, positionIteration);
            _initialized = true;
            _threadRunning = false;
        }


        // Loop over contacts

        // Debug.Log(world._contactCount);

        var counter = 0;

        var contactList = world._contactList;

        while(contactList != null && counter < world._contactCount){
            // Debug.Log(contactList.FixtureA.Body.Box2DRigidbody.gameObject);
            contactList = world._contactList.GetNext();
            counter++;
        }


        // if(childThread != null && childThread.IsAlive){
        //     Debug.Log("Previous thread not finished");
        //     return;
        // }
         
        // childThread = new Thread(childref);
        // childThread.Start();

        // Debug.Log(world.GetContactCount());

        // foreach (var contact in world._contactList.)
        // {
            
        // }

        // Debug.Log(world._contactList);
    }

    private void StepThread(object stateInfo)
    {
        _threadRunning = true;
        // Debug.Log("Starting thread");
        world.Step(timestep, velocityIteration, positionIteration);
        _initialized = true;
        // Debug.Log("Thread done");
        _threadRunning = false;
    }

    private void OnDisable(){
        world.Dispose();
    }

}
