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
    private float _lastUpdateTimestamp;
    private Vector2 _currentPosition;
    private Vector2 _previousPosition;
    private Quaternion _currentRotation = Quaternion.identity;
    private Quaternion _previousRotation = Quaternion.identity;

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

        _previousPosition = _currentPosition;
        _currentPosition = new Vector2((float)body.GetPosition().x, (float)body.GetPosition().y);

        _previousRotation = _currentRotation;
        var euler = new Vector3(0f, 0f, (float)body.GetAngle() * Mathf.Rad2Deg);
        _currentRotation = Quaternion.Euler(euler);

        // var newPosition = new Vector2((float)body.GetPosition().x, (float)body.GetPosition().y);
        // var newRotation = Quaternion.Euler(new Vector3(0f, 0f, (float)body.GetAngle() * Mathf.Rad2Deg));

        // transform.position = newPosition;
        // transform.rotation = newRotation;

        _lastUpdateTimestamp = Time.timeSinceLevelLoad;
    }

    private void Update(){
        if(!Box2DSimulation.instance.useCustomBox2D){
            return;
        }

        // var deltaTime = (Time.timeSinceLevelLoad - Box2DSimulation.instance.LastUpdateTimestamp) / Time.fixedDeltaTime;
        var deltaTime = (Time.timeSinceLevelLoad - _lastUpdateTimestamp) / Time.fixedDeltaTime;
        // Debug.Log(deltaTime);
        //   new Vector2((float)body._linearVelocity.x, (float)body._linearVelocity.y);
        // var translation = positionDelta * deltaTime;
        // position = position + translation;


        transform.position = Vector2.Lerp(_previousPosition, _currentPosition, deltaTime);

        transform.rotation = Quaternion.Lerp(_previousRotation, _currentRotation, deltaTime);


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
        // if(logContact)
        //     Debug.Log($"{gameObject}: {logContact}");
        // onBeginContactEvent.Invoke(contact);
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

    #if UNITY_EDITOR
    private void OnDrawGizmosSelected(){
        Gizmos.color = UnityEngine.Color.red;
        Gizmos.DrawCube(_previousPosition, Vector3.one * 0.1f);
        Gizmos.color = UnityEngine.Color.green;
        Gizmos.DrawCube(_currentPosition, Vector3.one * 0.1f);

        if(body == null)
            return;

        if(body._fixtureList == null)
            return;

        if(body._fixtureList.ShapeType == ShapeType.PolygonShape){
            var polygonShape = (PolygonShape)body._fixtureList.Shape;
            var g = 0f;
            for (int i = 0; i < polygonShape.VertexCount; i++)
            {
                g = (float)i / (float)polygonShape.VertexCount;
                // Debug.Log(g);
                Gizmos.color = new UnityEngine.Color(g, 1f - g, 0f);
                var vertexA = polygonShape._vertices[i];
                var iPlusOneOrZero = i >= polygonShape.VertexCount - 1 ? 0 : i + 1;
                var vertexB = polygonShape._vertices[iPlusOneOrZero];
                Debug.Log($"{i}: {vertexA}");
                Debug.Log($"{iPlusOneOrZero}: {vertexB}");
                var positionA = transform.position + new Vector3((float)vertexA.x, (float)vertexA.y);
                var positionB = transform.position + new Vector3((float)vertexB.x, (float)vertexB.y);

                positionA = transform.rotation * (positionA - transform.position) + transform.position;
                positionB = transform.rotation * (positionB - transform.position) + transform.position;

                
                Gizmos.DrawLine(positionA, positionB);
            }    
        }
    }
    #endif
}

[System.Serializable]
public class UnityEventContact: UnityEvent<Contact>
{
}