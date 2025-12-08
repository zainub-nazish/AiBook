/*
 * JointStatePublisher.cs
 * Module 2: The Digital Twin - Chapter 2
 *
 * Publishes joint state information from Unity ArticulationBody joints
 * to ROS 2 as sensor_msgs/JointState messages.
 *
 * This enables bidirectional communication between Unity simulation
 * and ROS 2 control/planning systems.
 */

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

namespace DigitalTwin.ROS
{
    /// <summary>
    /// Publishes joint states from Unity to ROS 2.
    /// </summary>
    public class JointStatePublisher : MonoBehaviour
    {
        [Header("Robot Configuration")]
        [Tooltip("Root GameObject of the robot (with ArticulationBody hierarchy)")]
        public GameObject robotRoot;

        [Tooltip("Automatically find robot if not specified")]
        public bool autoFindRobot = true;

        [Header("ROS Configuration")]
        [Tooltip("Topic name for joint state publishing")]
        public string topicName = "/joint_states";

        [Tooltip("Publishing rate in Hz")]
        [Range(1, 100)]
        public float publishRate = 30f;

        [Tooltip("Frame ID for joint state messages")]
        public string frameId = "base_link";

        [Header("Joint Configuration")]
        [Tooltip("Only include joints with these names (empty = all joints)")]
        public List<string> includeJoints = new List<string>();

        [Tooltip("Exclude joints with these names")]
        public List<string> excludeJoints = new List<string>();

        [Header("Debug")]
        [Tooltip("Show debug information")]
        public bool showDebug = false;

        // Internal state
        private ROSConnection rosConnection;
        private List<ArticulationBody> joints = new List<ArticulationBody>();
        private List<string> jointNames = new List<string>();
        private float lastPublishTime;
        private JointStateMsg jointStateMsg;
        private uint sequenceNumber = 0;

        private void Start()
        {
            // Get ROS connection
            rosConnection = ROSConnection.GetOrCreateInstance();

            // Register publisher
            rosConnection.RegisterPublisher<JointStateMsg>(topicName);

            // Find robot if not specified
            if (robotRoot == null && autoFindRobot)
            {
                robotRoot = FindRobotRoot();
            }

            // Discover joints
            if (robotRoot != null)
            {
                DiscoverJoints();
            }
            else
            {
                Debug.LogError("JointStatePublisher: No robot root specified or found");
            }

            // Pre-allocate message
            InitializeMessage();
        }

        /// <summary>
        /// Find the robot root by looking for the first ArticulationBody root.
        /// </summary>
        private GameObject FindRobotRoot()
        {
            ArticulationBody[] bodies = FindObjectsOfType<ArticulationBody>();

            foreach (ArticulationBody body in bodies)
            {
                if (body.isRoot)
                {
                    Debug.Log($"Found robot root: {body.gameObject.name}");
                    return body.gameObject;
                }
            }

            return null;
        }

        /// <summary>
        /// Discover all movable joints in the robot hierarchy.
        /// </summary>
        private void DiscoverJoints()
        {
            joints.Clear();
            jointNames.Clear();

            ArticulationBody[] allBodies = robotRoot.GetComponentsInChildren<ArticulationBody>();

            foreach (ArticulationBody body in allBodies)
            {
                // Skip fixed joints and root
                if (body.jointType == ArticulationJointType.FixedJoint)
                    continue;

                string jointName = body.gameObject.name;

                // Check include list
                if (includeJoints.Count > 0 && !includeJoints.Contains(jointName))
                    continue;

                // Check exclude list
                if (excludeJoints.Contains(jointName))
                    continue;

                joints.Add(body);
                jointNames.Add(jointName);
            }

            Debug.Log($"JointStatePublisher: Discovered {joints.Count} joints");

            if (showDebug)
            {
                foreach (string name in jointNames)
                {
                    Debug.Log($"  - {name}");
                }
            }
        }

        /// <summary>
        /// Initialize the joint state message with proper sizes.
        /// </summary>
        private void InitializeMessage()
        {
            jointStateMsg = new JointStateMsg
            {
                header = new HeaderMsg
                {
                    frame_id = frameId
                },
                name = jointNames.ToArray(),
                position = new double[joints.Count],
                velocity = new double[joints.Count],
                effort = new double[joints.Count]
            };
        }

        private void Update()
        {
            // Check if it's time to publish
            float interval = 1f / publishRate;
            if (Time.time - lastPublishTime < interval)
                return;

            lastPublishTime = Time.time;

            // Update and publish joint states
            if (joints.Count > 0)
            {
                UpdateJointStates();
                PublishJointStates();
            }
        }

        /// <summary>
        /// Update joint state arrays with current values.
        /// </summary>
        private void UpdateJointStates()
        {
            for (int i = 0; i < joints.Count; i++)
            {
                ArticulationBody joint = joints[i];

                if (joint == null) continue;

                // Get position (convert to radians)
                if (joint.jointPosition.dofCount > 0)
                {
                    jointStateMsg.position[i] = joint.jointPosition[0] * Mathf.Deg2Rad;
                }

                // Get velocity (convert to rad/s)
                if (joint.jointVelocity.dofCount > 0)
                {
                    jointStateMsg.velocity[i] = joint.jointVelocity[0] * Mathf.Deg2Rad;
                }

                // Get effort (force/torque)
                if (joint.jointForce.dofCount > 0)
                {
                    jointStateMsg.effort[i] = joint.jointForce[0];
                }
            }
        }

        /// <summary>
        /// Publish the joint state message to ROS.
        /// </summary>
        private void PublishJointStates()
        {
            // Update header timestamp
            double timeSeconds = Time.timeAsDouble;
            int seconds = (int)timeSeconds;
            uint nanoseconds = (uint)((timeSeconds - seconds) * 1e9);

            jointStateMsg.header.stamp = new TimeMsg
            {
                sec = seconds,
                nanosec = nanoseconds
            };

            // Increment sequence number (deprecated in ROS 2 but still used)
            sequenceNumber++;

            // Publish
            rosConnection.Publish(topicName, jointStateMsg);

            if (showDebug)
            {
                Debug.Log($"Published joint states: seq={sequenceNumber}");
            }
        }

        /// <summary>
        /// Manually add a joint to the publisher.
        /// </summary>
        public void AddJoint(ArticulationBody joint)
        {
            if (joint == null || joints.Contains(joint))
                return;

            joints.Add(joint);
            jointNames.Add(joint.gameObject.name);
            InitializeMessage(); // Reallocate message arrays
        }

        /// <summary>
        /// Get current joint positions as a dictionary.
        /// </summary>
        public Dictionary<string, float> GetJointPositions()
        {
            Dictionary<string, float> positions = new Dictionary<string, float>();

            for (int i = 0; i < joints.Count; i++)
            {
                if (joints[i] != null && joints[i].jointPosition.dofCount > 0)
                {
                    positions[jointNames[i]] = joints[i].jointPosition[0] * Mathf.Deg2Rad;
                }
            }

            return positions;
        }

#if UNITY_EDITOR
        [ContextMenu("Refresh Joints")]
        private void EditorRefreshJoints()
        {
            if (robotRoot != null)
            {
                DiscoverJoints();
                InitializeMessage();
            }
        }
#endif
    }
}
