using UnityEngine;
using System.IO;

public class Vehicle : MonoBehaviour
{
    // Camera and transform variables
    private float offsetZ = 0.7f;
    public float rotationOffsetX = 60f;
    public Camera vehicleCamera;
    private Transform capsule;
    private Vector3 currentRotation;
    private Rigidbody rb;
    private ForceSimulation forceSimulation;

    private Vector3 linearDamping;
    private Vector3 angularDamping;
    private Vector3 coriolisForce;
    private Vector3 restoringForces;
    private Vector3 restoringMoments;


    // Physics properties from JSON
    private AssemblyMassProperties physicsConfig;

    // Damping parameters
    private float D_u, D_v, D_w, D_p, D_q, D_r;

    private float _W;                     // Weight force
    private float _B;                     // Buoyancy force
    private float _W_minus_B;             // Net vertical force

    private float fluidDensity;    // kg/m³ (sea water)
    private float displacedVolume;    // m³
    private Vector3 buoyancyCenter;        // Center of buoyancy (in local coordinates)

    [Header("Custom Vehicle Dynamics")]
    public bool loadDynamics = false;
    // JSON file name (place in Assets/Resources)
    public string fullPath = "physics_config";

    [System.Serializable]
    public class RootObject
    {
        public AssemblyMassProperties assembly_mass_properties;
    }

    // Nested classes to mirror JSON structure
    [System.Serializable]
    public class AssemblyMassProperties
    {
        public Dynamics dynamics;
        public Mass mass;
        public CenterOfMass center_of_mass;
        public PrincipalMoments principal_axes_and_moments_of_inertia;
        public BuoancyCenter center_of_buoancy_relative_to_cg;
    }

    [System.Serializable]
    public class BuoancyCenter
    {
        public float X;
        public float Y;
        public float Z;
    }

    [System.Serializable]
    public class Dynamics
    {
        public Dampling dampling;
        public FluidDensity fluid_density;
        public DisplacedVolume displaced_volume;
    }

    [System.Serializable]
    public class FluidDensity
    {
        public float value;
        public float unit;
    }

    [System.Serializable]
    public class DisplacedVolume
    {
        public float value;
        public float unit;
    }

    [System.Serializable]
    public class Dampling
    {
        public DampingAxis lineer; // Typo "lineer" matches JSON
        public DampingAxis angular;
    }

    [System.Serializable]
    public class DampingAxis
    {
        public float D_u, D_v, D_w, D_p, D_q, D_r;
    }

    [System.Serializable]
    public class Mass
    {
        public float value;
    }

    [System.Serializable]
    public class CenterOfMass
    {
        public float X, Y, Z;
    }

    [System.Serializable]
    public class PrincipalMoments
    {
        public InertiaAxis Ix, Iy, Iz;
    }

    [System.Serializable]
    public class InertiaAxis
    {
        public float[] axis;
        public float moment;
    }

    void Start()
    {
        capsule = transform; // Initialize capsule transform
        rb = GetComponent<Rigidbody>();
        forceSimulation = GetComponent<ForceSimulation>();
        if (loadDynamics) { LoadPhysicsConfig();ConfigurePhysics();}

        _W = rb.mass * Physics.gravity.magnitude;
        _B = fluidDensity * displacedVolume * Physics.gravity.magnitude;
        _W_minus_B = _W - _B;
    }

    void Update()
    {
        CameraPositioning();
    }

    void LoadPhysicsConfig()
    {
        if (File.Exists(fullPath))
        {
            string jsonContent = File.ReadAllText(fullPath);
            RootObject rootObject = JsonUtility.FromJson<RootObject>(jsonContent);
            physicsConfig = rootObject.assembly_mass_properties; // Extract the nested object
            Debug.Log("JSON loaded successfully from: " + fullPath);
        }
        else
        {
            Debug.LogError("JSON file not found at: " + fullPath);
        }
    }

    void ConfigurePhysics()
    {
        // Set mass
        rb.mass = physicsConfig.mass.value;

        // Set center of mass (convert Z-up to Y-up)
        Vector3 com = new Vector3(
            physicsConfig.center_of_mass.X,
            physicsConfig.center_of_mass.Z, // Z becomes Y in Unity
            physicsConfig.center_of_mass.Y
        );
        rb.centerOfMass = com;

        // Set inertia tensor and rotation
        Vector3 inertiaTensor = new Vector3(
            physicsConfig.principal_axes_and_moments_of_inertia.Ix.moment,
            physicsConfig.principal_axes_and_moments_of_inertia.Iy.moment,
            physicsConfig.principal_axes_and_moments_of_inertia.Iz.moment
        );
        rb.inertiaTensor = inertiaTensor;

        // Convert principal axes to Unity's coordinate system
        Vector3 principalX = ConvertAxis(physicsConfig.principal_axes_and_moments_of_inertia.Ix.axis);
        Vector3 principalZ = ConvertAxis(physicsConfig.principal_axes_and_moments_of_inertia.Iz.axis);
        rb.inertiaTensorRotation = Quaternion.LookRotation(principalZ, principalX);
        D_u = physicsConfig.dynamics.dampling.lineer.D_u;
        D_v = physicsConfig.dynamics.dampling.lineer.D_v;
        D_w = physicsConfig.dynamics.dampling.lineer.D_w;
        D_p = physicsConfig.dynamics.dampling.angular.D_p;
        D_q = physicsConfig.dynamics.dampling.angular.D_q;
        D_r = physicsConfig.dynamics.dampling.angular.D_r;

        fluidDensity = physicsConfig.dynamics.fluid_density.value;
        displacedVolume = physicsConfig.dynamics.displaced_volume.value;

        buoyancyCenter = new Vector3(physicsConfig.center_of_buoancy_relative_to_cg.X, physicsConfig.center_of_buoancy_relative_to_cg.Z, physicsConfig.center_of_buoancy_relative_to_cg.Y);
    }

    Vector3 ConvertAxis(float[] jsonAxis)
    {
        // Convert JSON's Z-up axis to Unity's Y-up
        return new Vector3(jsonAxis[0], jsonAxis[2], jsonAxis[1]);
    }

    void FixedUpdate()
    {
        if (loadDynamics)
        {
            ApplyDamping();ApplyCoriolis();ApplyRestoringForces();

            Debug.Log("Force: " + forceSimulation.thrust + "  " + forceSimulation.yaw);
            Debug.Log("Damping Force: " + linearDamping + "  " + angularDamping);
            Debug.Log("Coriolis Force: " + coriolisForce);
            Debug.Log("Restoring Force: " + restoringForces + "  " + restoringMoments);
        
        }
        
    }

    void ApplyDamping()
    {
        // Linear damping (local space)
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        linearDamping = new Vector3(
            -D_u * localVel.x,
            -D_v * localVel.y,
            -D_w * localVel.z
        );
        rb.AddForce(transform.TransformDirection(linearDamping));


        // Angular damping (local space)
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        angularDamping = new Vector3(
            -D_p * localAngVel.x,
            -D_q * localAngVel.y,
            -D_r * localAngVel.z
        );
        rb.AddTorque(transform.TransformDirection(angularDamping));
    }

    void ApplyCoriolis()
    {
        // Coriolis force: F_coriolis = -m * (ω × v)
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        coriolisForce = -rb.mass * Vector3.Cross(localAngVel, localVel);
        Debug.Log($"Local Velocity: {localVel}, Local Angular Velocity: {localAngVel}");

        rb.AddForce(transform.TransformDirection(coriolisForce), ForceMode.Force);
    }

    void ApplyRestoringForces()
    {
        // Get Euler angles in radians
        Vector3 eulerAngles = rb.rotation.eulerAngles * Mathf.Deg2Rad;
        float roll = eulerAngles.z;  // Unity's z-axis is roll
        float pitch = eulerAngles.x; // Unity's x-axis is pitch

        // Calculate trigonometric terms
        float sin_phi = Mathf.Sin(roll);
        float cos_phi = Mathf.Cos(roll);
        float sin_theta = Mathf.Sin(pitch);
        float cos_theta = Mathf.Cos(pitch);

        // Calculate restoring forces in LOCAL coordinates
        restoringForces = new Vector3(
            _W_minus_B * sin_theta,
            -_W_minus_B * cos_theta * sin_phi,
            -_W_minus_B * cos_theta * cos_phi
        );

        // Convert to world space and apply force
        rb.AddForce(transform.TransformDirection(restoringForces));

        // Calculate restoring moments (torques)
        restoringMoments = new Vector3(
            buoyancyCenter.y * _B * cos_theta * cos_phi - buoyancyCenter.z * _B * cos_theta * sin_phi,
            -buoyancyCenter.z * _B * sin_theta - buoyancyCenter.x * _B * cos_theta * cos_phi,
            buoyancyCenter.x * _B * cos_theta * sin_phi + buoyancyCenter.y * _B * sin_theta
        );

        // Apply torques
        rb.AddTorque(restoringMoments);

    }

    void CameraPositioning()
    {
        if (capsule != null)
        {
            // Lock capsule rotation on X/Z axes
            // currentRotation = capsule.rotation.eulerAngles;
            // currentRotation.z = 0f;
            // currentRotation.x = 0f;
            // capsule.rotation = Quaternion.Euler(currentRotation);

            // Position camera
            vehicleCamera.transform.position = capsule.position + capsule.forward * offsetZ;

            // Calculate camera rotation with offset
            Quaternion capsuleRotation = Quaternion.LookRotation(capsule.forward);
            Quaternion rotationWithOffset = capsuleRotation * Quaternion.Euler(rotationOffsetX, 0, 0);
            vehicleCamera.transform.rotation = rotationWithOffset;
        }
    }
}