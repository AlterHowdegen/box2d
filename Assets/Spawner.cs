using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawner : MonoBehaviour
{
    public List<GameObject> prefabs;

    public void Spawn(){
        var randomIndex = UnityEngine.Random.Range(0, prefabs.Count);
        var randomPrefab = prefabs[randomIndex];
        var randomPosition = new Vector3(UnityEngine.Random.Range(-1f, 1f), UnityEngine.Random.Range(-1f, 1f), 0f);
        var instance = Instantiate(randomPrefab, new Vector3(0f, 10f, 0f) + randomPosition, Quaternion.identity);
    }

    public void Spawn10(){
        for (int i = 0; i < 10; i++)
        {
            Spawn();
        }
    }
}
