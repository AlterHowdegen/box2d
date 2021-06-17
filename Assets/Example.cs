using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class Example : MonoBehaviour
{
    [SerializeField] private float timeScale = 1f;
    [SerializeField] private InputField verificationInputField;

    private void Start(){
        Time.timeScale = timeScale;

    }

    [ContextMenu("Set Time 10%")]
    public void SetTime10(){
        Time.timeScale = 0.1f;
    }

    [ContextMenu("Set Time 100%")]
    public void SetTime100(){
        Time.timeScale = 1f;
    }

    private void FixedUpdate(){
        verificationInputField.text = Box2DSimulation.instance.verification;
    }


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
