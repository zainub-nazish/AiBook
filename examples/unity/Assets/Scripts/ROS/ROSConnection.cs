/*
 * ROSConnection.cs
 * Module 2: The Digital Twin - Chapter 2
 *
 * Manages the connection between Unity and ROS 2 using ROS-TCP-Connector.
 * Provides a singleton interface for registering publishers and subscribers.
 *
 * Prerequisites:
 * - ROS-TCP-Connector package installed
 * - ros_tcp_endpoint running on ROS 2 side
 */

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace DigitalTwin.ROS
{
    /// <summary>
    /// Connection status enumeration.
    /// </summary>
    public enum ROSConnectionStatus
    {
        Disconnected,
        Connecting,
        Connected,
        Error
    }

    /// <summary>
    /// Manages ROS 2 connection and provides utilities for robotics communication.
    /// </summary>
    public class ROSConnectionManager : MonoBehaviour
    {
        #region Singleton
        private static ROSConnectionManager _instance;
        public static ROSConnectionManager Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = FindObjectOfType<ROSConnectionManager>();
                    if (_instance == null)
                    {
                        GameObject go = new GameObject("ROSConnectionManager");
                        _instance = go.AddComponent<ROSConnectionManager>();
                        DontDestroyOnLoad(go);
                    }
                }
                return _instance;
            }
        }
        #endregion

        [Header("Connection Settings")]
        [Tooltip("IP address of the ROS TCP endpoint")]
        public string rosIPAddress = "127.0.0.1";

        [Tooltip("Port of the ROS TCP endpoint")]
        public int rosPort = 10000;

        [Tooltip("Automatically connect on start")]
        public bool connectOnStart = true;

        [Tooltip("Reconnect interval in seconds")]
        public float reconnectInterval = 5f;

        [Header("Status")]
        [SerializeField] private ROSConnectionStatus connectionStatus = ROSConnectionStatus.Disconnected;

        public ROSConnectionStatus Status => connectionStatus;
        public bool IsConnected => connectionStatus == ROSConnectionStatus.Connected;

        // Events
        public event System.Action OnConnected;
        public event System.Action OnDisconnected;
        public event System.Action<string> OnError;

        private ROSConnection rosConnection;
        private float lastReconnectAttempt;

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }
            _instance = this;
            DontDestroyOnLoad(gameObject);
        }

        private void Start()
        {
            // Get or create ROS Connection component
            rosConnection = ROSConnection.GetOrCreateInstance();

            if (connectOnStart)
            {
                Connect();
            }
        }

        private void Update()
        {
            // Monitor connection status
            UpdateConnectionStatus();

            // Auto-reconnect logic
            if (connectionStatus == ROSConnectionStatus.Disconnected && connectOnStart)
            {
                if (Time.time - lastReconnectAttempt > reconnectInterval)
                {
                    Debug.Log("Attempting to reconnect to ROS...");
                    Connect();
                    lastReconnectAttempt = Time.time;
                }
            }
        }

        /// <summary>
        /// Connect to the ROS TCP endpoint.
        /// </summary>
        public void Connect()
        {
            if (rosConnection == null)
            {
                rosConnection = ROSConnection.GetOrCreateInstance();
            }

            connectionStatus = ROSConnectionStatus.Connecting;

            // Configure connection
            rosConnection.RosIPAddress = rosIPAddress;
            rosConnection.RosPort = rosPort;

            // Connect
            rosConnection.Connect();

            Debug.Log($"Connecting to ROS at {rosIPAddress}:{rosPort}...");
        }

        /// <summary>
        /// Disconnect from ROS.
        /// </summary>
        public void Disconnect()
        {
            if (rosConnection != null)
            {
                rosConnection.Disconnect();
            }

            connectionStatus = ROSConnectionStatus.Disconnected;
            OnDisconnected?.Invoke();

            Debug.Log("Disconnected from ROS");
        }

        /// <summary>
        /// Update connection status based on ROS connection state.
        /// </summary>
        private void UpdateConnectionStatus()
        {
            if (rosConnection == null) return;

            ROSConnectionStatus newStatus = connectionStatus;

            // Check if connected (this is simplified - actual implementation
            // would check the internal connection state)
            if (rosConnection.HasConnectionThread)
            {
                if (connectionStatus != ROSConnectionStatus.Connected)
                {
                    newStatus = ROSConnectionStatus.Connected;
                    OnConnected?.Invoke();
                    Debug.Log("Connected to ROS successfully");
                }
            }

            connectionStatus = newStatus;
        }

        /// <summary>
        /// Register a publisher for a topic.
        /// </summary>
        public void RegisterPublisher<T>(string topic) where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message
        {
            if (rosConnection == null)
            {
                Debug.LogError("ROS Connection not initialized");
                return;
            }

            rosConnection.RegisterPublisher<T>(topic);
            Debug.Log($"Registered publisher for topic: {topic}");
        }

        /// <summary>
        /// Publish a message to a topic.
        /// </summary>
        public void Publish<T>(string topic, T message) where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message
        {
            if (!IsConnected)
            {
                Debug.LogWarning($"Cannot publish to {topic}: Not connected to ROS");
                return;
            }

            rosConnection.Publish(topic, message);
        }

        /// <summary>
        /// Subscribe to a topic.
        /// </summary>
        public void Subscribe<T>(string topic, System.Action<T> callback) where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message
        {
            if (rosConnection == null)
            {
                Debug.LogError("ROS Connection not initialized");
                return;
            }

            rosConnection.Subscribe<T>(topic, callback);
            Debug.Log($"Subscribed to topic: {topic}");
        }

        /// <summary>
        /// Convert Unity Vector3 to ROS coordinate system.
        /// Unity: Y-up, left-handed
        /// ROS: Z-up, right-handed
        /// </summary>
        public static Vector3 UnityToROS(Vector3 unityVector)
        {
            return new Vector3(unityVector.z, -unityVector.x, unityVector.y);
        }

        /// <summary>
        /// Convert ROS Vector3 to Unity coordinate system.
        /// </summary>
        public static Vector3 ROSToUnity(Vector3 rosVector)
        {
            return new Vector3(-rosVector.y, rosVector.z, rosVector.x);
        }

        /// <summary>
        /// Convert Unity Quaternion to ROS coordinate system.
        /// </summary>
        public static Quaternion UnityToROS(Quaternion unityQuat)
        {
            return new Quaternion(unityQuat.z, -unityQuat.x, unityQuat.y, -unityQuat.w);
        }

        /// <summary>
        /// Convert ROS Quaternion to Unity coordinate system.
        /// </summary>
        public static Quaternion ROSToUnity(Quaternion rosQuat)
        {
            return new Quaternion(-rosQuat.y, rosQuat.z, rosQuat.x, -rosQuat.w);
        }
    }
}
