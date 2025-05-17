using UnityEngine;

public class SonarRayCast : MonoBehaviour
{
    // Public variables
    public int rayNumber = 5; // Number of rays
    public float view = 90f; // Degree of view for rays to be in
    public float rayLength = 10f; // Length of each ray

    // Private variables
    private float[] rayHits; // Array that keeps track of ray hits
    private Transform vehicle; // Transform from which rays will start

    // Public property to expose rayHits
    public float[] Hits => rayHits;

    void Start()
    {
        // Initialize the rayHits array
        rayHits = new float[rayNumber];

        // Assuming the script is attached to the vehicle, we can use this.transform
        vehicle = this.transform;
    }

    void FixedUpdate()
    {
        // Update rays every frame
        DrawRays();
    }

    private void DrawRays()
    {
        // Calculate the angle between each ray
        float angleBetweenRays = view / (rayNumber - 1);

        // Start angle is half of the view to the left
        float startAngle = -view / 2;

        // Loop through each ray
        for (int i = 0; i < rayNumber; i++)
        {
            // Calculate the current angle for this ray
            float currentAngle = startAngle + angleBetweenRays * i;

            // Calculate the direction of the ray
            Vector3 direction = Quaternion.Euler(0, currentAngle, 0) * vehicle.forward;

            // Perform the raycast
            RaycastHit hit;
            if (Physics.Raycast(vehicle.position, direction, out hit, rayLength))
            {
                // If the ray hits something, store the distance in rayHits
                rayHits[i] = hit.distance;

                // Draw the ray in the editor for debugging (red if it hits something)
                Debug.DrawRay(vehicle.position, direction * hit.distance, Color.red);
            }
            else
            {
                // If the ray doesn't hit anything, store -1
                rayHits[i] = -1;

                // Draw the ray in the editor for debugging (green if it doesn't hit anything)
                Debug.DrawRay(vehicle.position, direction * rayLength, Color.green);
            }
        }
    }
}