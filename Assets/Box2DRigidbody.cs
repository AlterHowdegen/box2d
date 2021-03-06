using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Box2DX;
using Box2DX.Dynamics;
using Box2DX.Collision;
using System;

public class Box2DRigidbody : MonoBehaviour, ContactListener
{
    public Body body;
    public Rigidbody2D _originalRigidbody;
    public Collider2D _originalCollider;
    private ContactListener _contactListener;

    private void Awake(){
        _originalRigidbody = GetComponent<Rigidbody2D>();
        _originalCollider = GetComponent<Collider2D>();
    }

    private void Start(){
        if(!Box2DSimulation.instance.useCustomBox2D){
            return;
        }

        // Create body

        var bodyDefinition = new BodyDef();
        bodyDefinition.MassData = new MassData();
        bodyDefinition.MassData.Mass = _originalRigidbody.mass < 0.1f ? 0f : _originalRigidbody.mass;
        bodyDefinition.MassData.I = _originalRigidbody.mass < 0.1f ? 0f : _originalRigidbody.mass;
        bodyDefinition.Position = _originalRigidbody.position;
        bodyDefinition.Angle = transform.eulerAngles.z * Mathf.Deg2Rad;
        // bodyDefinition.FixedRotation = _originalRigidbody.freezeRotation;
        // bodyDefinition.AngularDamping = _originalRigidbody.angularDrag;
        bodyDefinition.FixedRotation = false;
        bodyDefinition.LinearDamping = _originalRigidbody.drag;
        bodyDefinition.AngularDamping = _originalRigidbody.angularDrag;
        // bodyDefinition.AngularVelocity= 1f;
        // bodyDefinition.
        bodyDefinition.AllowSleep = true;
        // bodyDefinition.AngularVelocity = 10f;

        // bodyDefinition.
        
        body = Box2DSimulation.instance.world.CreateBody(bodyDefinition);

        // Create fixture

        if(_originalCollider is BoxCollider2D){
            Debug.Log("Creating polygon fixture");
            var fixtureDefinition = new PolygonDef();
            fixtureDefinition.SetAsBox(((BoxCollider2D)_originalCollider).size.x / 2f, ((BoxCollider2D)_originalCollider).size.y / 2f);
            fixtureDefinition.Type = ShapeType.PolygonShape;
            fixtureDefinition.Density = _originalCollider.density;
            fixtureDefinition.Friction = 0.6f;
            fixtureDefinition.Restitution = 0.5f; 
            body.CreateFixture(fixtureDefinition);
        }else if(_originalCollider is CircleCollider2D){
            Debug.Log("Creating circle fixture");
            var fixtureDefinition = new CircleDef();
            fixtureDefinition.Density = _originalCollider.density;
            fixtureDefinition.Friction = 0.6f;
            fixtureDefinition.Restitution = 0.5f; 
            fixtureDefinition.Radius = ((CircleCollider2D)_originalCollider).radius;
            body.CreateFixture(fixtureDefinition);
        }else{
            Debug.Log("Creating generic fixture");
            var fixtureDefinition = new FixtureDef();
            body.CreateFixture(fixtureDefinition);
        }

        // Set contact listener

        Box2DSimulation.instance.world.SetContactListener(this);

        // Destroy unity 2D physics components
        
        Destroy(_originalRigidbody);
        Destroy(_originalCollider);
    }

    private void FixedUpdate(){
        if(!Box2DSimulation.instance.useCustomBox2D){
            return;
        }
        transform.position = body.GetPosition();
        transform.eulerAngles = new Vector3(0f, 0f, body.GetAngle() * Mathf.Rad2Deg);
        // Debug.Log(body.IsDynamic());
        // Debug.Log(body._xf.position);
        // Debug.Log(body.GetAngle());
        // Debug.Log(body.GetFixtureList().ShapeType);
        // Debug.Log(body.GetFixtureList().Density);
        // Debug.Log(body.GetType());
        // Debug.Log(body.GetPosition());

        // Box2DSimulation.instance.contactListener.
    }

    public void BeginContact(Contact contact)
    {
        Debug.Log(contact);
    }

    public void EndContact(Contact contact)
    {
    }

    public void PreSolve(Contact contact, Manifold oldManifold)
    {
    }

    public void PostSolve(Contact contact, ContactImpulse impulse)
    {
    }

    // #if UNITY_EDITOR
    // private void OnDrawGizmosSelected(){
    //     Gizmos.color = UnityEngine.Color.green;
    //     var angle = body.GetAngle();

    //     var fixture = body._fixtureList;


    //     Gizmos.DrawWireCube(body.GetWorldCenter(), Vector3.one);

    //     for (int i = 0; i < body._fixtureList.Shape.; i++)
    //     {

    //     }
    // }

    // #endif
}