/*
 * ImportRobot.cs
 * Module 2: The Digital Twin - Chapter 2
 *
 * Helper script for importing URDF robot models into Unity.
 * Uses Unity Robotics Hub's URDF Importer package.
 *
 * Prerequisites:
 * - Unity Robotics Hub packages installed (see manifest.json)
 * - URDF file in Assets/URDF/ folder
 */

using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
using Unity.Robotics.UrdfImporter;
#endif

namespace DigitalTwin.Import
{
    /// <summary>
    /// Provides utilities for importing and configuring URDF robot models.
    /// </summary>
    public class ImportRobot : MonoBehaviour
    {
        [Header("Import Settings")]
        [Tooltip("Path to URDF file relative to Assets folder")]
        public string urdfPath = "URDF/humanoid_robot.urdf";

        [Tooltip("Scale factor for imported model (1.0 = meters)")]
        public float importScale = 1.0f;

        [Tooltip("Use mesh colliders for accurate collision")]
        public bool useMeshColliders = true;

        [Header("Physics Settings")]
        [Tooltip("Enable physics simulation on joints")]
        public bool enablePhysics = true;

        [Tooltip("Default joint damping")]
        public float jointDamping = 10f;

        [Tooltip("Default joint stiffness")]
        public float jointStiffness = 0f;

        [Header("Runtime Reference")]
        [Tooltip("Reference to imported robot root")]
        public GameObject importedRobot;

        /// <summary>
        /// Import robot from URDF file at runtime.
        /// Note: Editor import is preferred for complex models.
        /// </summary>
        public void ImportFromUrdf()
        {
#if UNITY_EDITOR
            string fullPath = System.IO.Path.Combine(Application.dataPath, urdfPath);

            if (!System.IO.File.Exists(fullPath))
            {
                Debug.LogError($"URDF file not found: {fullPath}");
                return;
            }

            // Configure import settings
            ImportSettings settings = new ImportSettings
            {
                chosenAxis = ImportSettings.axisType.yAxis,
                convexMethod = ImportSettings.convexDecomposer.vHACD
            };

            // Import the robot
            importedRobot = UrdfRobotExtensions.Create(fullPath, settings);

            if (importedRobot != null)
            {
                // Apply scale
                importedRobot.transform.localScale = Vector3.one * importScale;

                // Configure physics
                if (enablePhysics)
                {
                    ConfigureJointPhysics(importedRobot);
                }

                Debug.Log($"Successfully imported robot from {urdfPath}");
            }
#else
            Debug.LogWarning("URDF import is only available in Unity Editor");
#endif
        }

        /// <summary>
        /// Configure physics properties for all articulation joints.
        /// </summary>
        private void ConfigureJointPhysics(GameObject robot)
        {
            // Find all ArticulationBody components
            ArticulationBody[] bodies = robot.GetComponentsInChildren<ArticulationBody>();

            foreach (ArticulationBody body in bodies)
            {
                // Configure joint drive
                if (body.jointType != ArticulationJointType.FixedJoint)
                {
                    ArticulationDrive drive = body.xDrive;
                    drive.damping = jointDamping;
                    drive.stiffness = jointStiffness;
                    body.xDrive = drive;
                }

                // Enable collision detection
                body.detectCollisions = true;
            }

            Debug.Log($"Configured physics for {bodies.Length} articulation bodies");
        }

        /// <summary>
        /// Get all joint names from the imported robot.
        /// </summary>
        public List<string> GetJointNames()
        {
            List<string> jointNames = new List<string>();

            if (importedRobot == null)
            {
                Debug.LogWarning("No robot imported");
                return jointNames;
            }

            ArticulationBody[] bodies = importedRobot.GetComponentsInChildren<ArticulationBody>();

            foreach (ArticulationBody body in bodies)
            {
                if (body.jointType != ArticulationJointType.FixedJoint)
                {
                    jointNames.Add(body.gameObject.name);
                }
            }

            return jointNames;
        }

        /// <summary>
        /// Set joint position by name.
        /// </summary>
        public void SetJointPosition(string jointName, float position)
        {
            if (importedRobot == null) return;

            Transform jointTransform = importedRobot.transform.Find(jointName);
            if (jointTransform == null)
            {
                // Search recursively
                jointTransform = FindChildRecursive(importedRobot.transform, jointName);
            }

            if (jointTransform != null)
            {
                ArticulationBody body = jointTransform.GetComponent<ArticulationBody>();
                if (body != null)
                {
                    ArticulationDrive drive = body.xDrive;
                    drive.target = position * Mathf.Rad2Deg; // Convert to degrees
                    body.xDrive = drive;
                }
            }
        }

        /// <summary>
        /// Get current joint positions as dictionary.
        /// </summary>
        public Dictionary<string, float> GetJointPositions()
        {
            Dictionary<string, float> positions = new Dictionary<string, float>();

            if (importedRobot == null) return positions;

            ArticulationBody[] bodies = importedRobot.GetComponentsInChildren<ArticulationBody>();

            foreach (ArticulationBody body in bodies)
            {
                if (body.jointType != ArticulationJointType.FixedJoint)
                {
                    // Get current position in radians
                    float position = body.jointPosition[0] * Mathf.Deg2Rad;
                    positions[body.gameObject.name] = position;
                }
            }

            return positions;
        }

        private Transform FindChildRecursive(Transform parent, string name)
        {
            foreach (Transform child in parent)
            {
                if (child.name == name)
                    return child;

                Transform found = FindChildRecursive(child, name);
                if (found != null)
                    return found;
            }
            return null;
        }

#if UNITY_EDITOR
        /// <summary>
        /// Editor button to trigger import.
        /// </summary>
        [ContextMenu("Import Robot from URDF")]
        private void EditorImport()
        {
            ImportFromUrdf();
        }
#endif
    }
}
