# Chapter 1 Outline: ROS 2 Basics

**File**: `book/docs/module-01-ros2/chapter-01-basics.mdx`
**Estimated Length**: ~15-20 pages
**Estimated Time**: 2-3 hours

## Learning Objectives

By the end of this chapter, students will be able to:

1. Explain what a ROS 2 node is and why modularity matters in robotics
2. Create publisher and subscriber nodes using rclpy
3. Implement a service server and client for request/response communication
4. Use ROS 2 CLI tools to inspect running nodes, topics, and services

## Prerequisites

- Python 3.10+ installed
- ROS 2 Humble installed and sourced
- Basic Python knowledge (functions, classes, imports)
- Terminal/command-line familiarity

## Section Outline

### 1. Introduction to ROS 2 (h2)
- What is ROS 2?
- Why ROS 2 for robotics (modularity, real-time, cross-platform)
- ROS 2 vs ROS 1 (brief comparison, not dwelling on legacy)
- The concept of the ROS 2 graph

**Diagram**: ROS 2 system architecture overview (Mermaid)

### 2. What is a Node? (h2)
- Definition: independent computational unit
- Why nodes? (separation of concerns, fault isolation)
- Node naming and namespaces
- Node discovery (DDS underneath)

**Diagram**: Multiple nodes in a robot system (publisher, subscriber, service)

### 3. Topics: Publish/Subscribe Communication (h2)
- What is a topic?
- Publishers and subscribers
- Message types (std_msgs, geometry_msgs, etc.)
- Quality of Service (QoS) basics

**Diagram**: Publisher → Topic → Subscriber flow

#### 3.1 Creating a Publisher Node (h3)

**Code Example**: `publisher_node.py`
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run Instructions**:
```bash
source /opt/ros/humble/setup.bash
python3 publisher_node.py
```

**Expected Output**:
```
[INFO] [simple_publisher]: Publishing: "Hello, ROS 2! Count: 0"
[INFO] [simple_publisher]: Publishing: "Hello, ROS 2! Count: 1"
...
```

#### 3.2 Creating a Subscriber Node (h3)

**Code Example**: `subscriber_node.py`
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Services: Request/Response Communication (h2)
- What is a service?
- When to use services vs topics
- Service types and definitions
- Synchronous nature of services

**Diagram**: Client → Service Server → Response flow

#### 4.1 Creating a Service Server (h3)

**Code Example**: `service_server.py` (AddTwoInts)

#### 4.2 Creating a Service Client (h3)

**Code Example**: `service_client.py`

### 5. ROS 2 Command-Line Tools (h2)
- `ros2 node list` — List active nodes
- `ros2 topic list` — List active topics
- `ros2 topic echo <topic>` — Print messages
- `ros2 topic info <topic>` — Topic details
- `ros2 service list` — List active services
- `ros2 service call` — Call a service

**Hands-on**: Using CLI to debug publisher/subscriber

### 6. Summary (h2)
- Key concepts recap
- What we learned
- Preview of Chapter 2

## Exercises

### Exercise 1: Create a ROS 2 Workspace
**Objective**: Set up a proper colcon workspace
**Steps**:
1. Create directory structure: `~/ros2_ws/src`
2. Initialize with `colcon build`
3. Source the workspace
**Expected Outcome**: Workspace builds with no errors

### Exercise 2: Build Your First Publisher
**Objective**: Create a publisher that sends custom messages
**Steps**:
1. Modify the publisher to send Float64 messages
2. Publish a sine wave value
3. Run and verify with `ros2 topic echo`
**Expected Outcome**: See oscillating values in terminal

### Exercise 3: Connect Publisher and Subscriber
**Objective**: Run both nodes and observe communication
**Steps**:
1. Run publisher in terminal 1
2. Run subscriber in terminal 2
3. Observe message flow
**Expected Outcome**: Subscriber prints publisher's messages

### Exercise 4: Implement Add Two Ints Service
**Objective**: Create a complete service server and client
**Steps**:
1. Implement service server
2. Implement service client
3. Call service with different values
**Expected Outcome**: Correct sum returned for any inputs

## Assessment Questions

1. What is the difference between a topic and a service in ROS 2?
2. Why would you use multiple nodes instead of one large program?
3. What does `rclpy.spin()` do?
4. How do you find out what topics are currently active?

## Citations

- [1] ROS 2 Documentation - Nodes: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes.html
- [2] ROS 2 Documentation - Topics: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics.html
- [3] ROS 2 Documentation - Services: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services.html
