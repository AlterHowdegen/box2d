using System.Collections;
using System.Collections.Generic;
using Box2DX.Dynamics;
using UnityEngine;

public class Effects : MonoBehaviour
{
    public static Effects instance;
    public ParticleSystem particleSystem;
    private void Awake(){
        instance = this;
    }
    public void Impact(Contact contact){
        // Debug.Log($"Impact: {contact.Manifold.LocalPoint}");
        transform.position = (Vector2)contact.FixtureB.Body.Box2DRigidbody.transform.position + new Vector2((float)contact.Manifold.LocalPoint.x, (float)contact.Manifold.LocalPoint.y);
        particleSystem.Emit(10);
    }
}
