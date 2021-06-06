using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class Example : MonoBehaviour
{
    public void Reload(){
        SceneManager.LoadScene("SampleScene");
    }

    public void SetUseDeterministic(bool active){
        Box2DSimulation.instance.useCustomBox2D = active;
    }

    public void SetUseThreading(bool active){
        Box2DSimulation.instance.useMultiThreading = active;
    }

}
