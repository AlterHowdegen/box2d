using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Box2DX;
using Box2DX.Dynamics;
using Box2DX.Collision;
using System;
using SoftFloat;
using UnityEngine.Events;

public class Box2DRigidbody : MonoBehaviour, ContactListener
{
    [SerializeField] private Body body;
    internal Rigidbody2D _originalRigidbody;
    internal Collider2D _originalCollider;
    public UnityEventContact onBeginContactEvent;
    [SerializeField] private bool logContact;

    public Body Body { get => body; }

    private void Awake(){
        if(Box2DSimulation.instance == null)
            return;
            
        if(Box2DSimulation.instance._initialized){
            Box2DSimulation.instance.AddBody(this);
        }
    }

    public void GatherParts(){
        _originalRigidbody = GetComponent<Rigidbody2D>();
        _originalCollider = GetComponent<Collider2D>();
    }

    internal void SetBody(Body body){
        this.body = body;
    }

    private void FixedUpdate(){
        if(!Box2DSimulation.instance.useCustomBox2D){
            return;
        }
        transform.position = new Vector2((float)body.GetPosition().x, (float)body.GetPosition().y);
        transform.eulerAngles = new Vector3(0f, 0f, (float)body.GetAngle() * Mathf.Rad2Deg);
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
        // Debug.Log($"{gameObject}: {logContact}");
        if(logContact)
            LogContact(contact);
        onBeginContactEvent.Invoke(contact);
        // Effects.instance.Impact(contact);
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
    public void LogContact(Contact contact){
        Debug.Log(contact);

        // Debug.Log(contact.FixtureA.Body.Box2DRigidbody);
        // Debug.Log(contact.FixtureB.Body.Box2DRigidbody);

        Effects.instance.Impact(contact);
    }

    private void OnDisable(){
        Box2DSimulation.instance.RemoveBody(this);
    }
}

[System.Serializable]
public class UnityEventContact: UnityEvent<Contact>
{
}