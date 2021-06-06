using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(Box2DSimulation))]
public class Box2DSimulationEditor : Editor
{
    private Box2DSimulation box2DSimulation;
    private void OnEnable(){
        box2DSimulation = (Box2DSimulation)target;
    }
    public override void OnInspectorGUI(){
        
        DrawDefaultInspector();
        
        if(GUILayout.Button("Gather bodies"))
        {
            box2DSimulation.GatherBodies();
        }
    }
}
