using System.Collections;
using System.Collections.Generic;
using Box2DX.Dynamics;
using SoftFloat;
using UnityEngine;

public class MoveTrigger : MonoBehaviour
{
    private Body _body;
    private sfloat translationAmount = (sfloat)0.1f;

    // Update is called once per frame
    private void Awake(){
        Init();
    }

    private void Init(){
        _body = GetComponent<Box2DRigidbody>().Body;
    }
    void FixedUpdate()
    {
        if(!Box2DSimulation.instance._initialized){
            Init();
            return;
        }

        var transform = _body.GetTransform();
        
        if(Input.GetKey(KeyCode.A)){
            transform.position.x -= translationAmount;
        }

        if(Input.GetKey(KeyCode.D)){
            transform.position.x += translationAmount;
        }

        if(Input.GetKey(KeyCode.S)){
            transform.position.y -= translationAmount;
        }

        if(Input.GetKey(KeyCode.W)){
            transform.position.y += translationAmount;
        }

        _body.SetTransform(transform);
    }
}
