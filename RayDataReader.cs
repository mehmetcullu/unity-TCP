using UnityEngine;

public class RayDataReader : MonoBehaviour
{
    public GameObject targetObject; // Bu alana SonarRayCast olan objeyi sürükle
    private SonarRayCast sonar;

    void Start()
    {
        sonar = targetObject.GetComponent<SonarRayCast>();
    }

    void Update()
    {
        float[] hits = sonar.Hits;
        for (int i = 0; i < hits.Length; i++)
        {
            Debug.Log("Ray " + i + ": " + hits[i]);
        }
    }
}