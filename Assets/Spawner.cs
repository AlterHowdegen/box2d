using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawner : MonoBehaviour
{
    public List<GameObject> prefabs;

    public void Spawn(){
        var randomIndex = UnityEngine.Random.Range(0, prefabs.Count);
        var randomPrefab = prefabs[randomIndex];
        var instance = Instantiate(randomPrefab, new Vector3(0f, 10f, 0f), Quaternion.identity);
    }
}
