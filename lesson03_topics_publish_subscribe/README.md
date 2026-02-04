# Lesson 03: ROS2 Topics and Publish-Subscribe Pattern

## Learning Objectives

After completing this lesson, you should be able to:

- Understand ROS2 Topics and their role in asynchronous communication
- Grasp the Publisher-Subscriber (Pub-Sub) design pattern
- Create Publisher nodes that send data to topics
- Create Subscriber nodes that receive data from topics
- Implement practical examples with the Waiter and Chef nodes
- Use ROS2 command-line tools to inspect and debug topics

## Key Concepts

### What are ROS2 Topics?

**Topics** are named buses for asynchronous, many-to-many communication between ROS2 nodes.

**Key Characteristics:**
- **Named Communication Channels**: Each topic has a unique name (e.g., `/orders`, `/sensor_data`)
- **Asynchronous**: Publishers don't wait for subscribers to receive messages
- **Many-to-Many**: Multiple publishers and multiple subscribers can communicate on the same topic
- **Decoupled**: Nodes don't need to know about each other's existence
- **Continuous**: Best for streaming data like sensors, cameras, or continuous sensor streams

### Publisher-Subscriber Pattern

**Publishers** send messages to a topic without knowing who (if anyone) is listening.

**Subscribers** listen to a topic and receive all messages published there.

**Real-World Analogy - Restaurant:**
- **Waiter (Publisher)**: Takes customer orders and announces them loudly in the kitchen
- **/orders (Topic)**: The communication channel where orders are announced
- **Chef (Subscriber)**: Listens for orders on the kitchen counter and starts cooking
- **Decoupling**: The waiter doesn't need to know if the chef is in the kitchen or how many chefs are there. They just announce orders to the topic.

### Why Use the Pub-Sub Pattern?

1. **Decoupling**: Nodes are independent and don't need direct connections
2. **Scalability**: Add more subscribers without modifying the publisher
3. **Flexibility**: One publisher can have zero or many subscribers
4. **Real-time Data**: Perfect for continuous sensor data streams
5. **Asynchronous Communication**: Non-blocking, fire-and-forget messaging

## Creating Publisher and Subscriber Nodes

### Part 1: Understanding Topics

When you create a publisher or subscriber for a topic, ROS2 automatically creates the topic if it doesn't exist.

### Creating a Publisher Node (waiter.py)

Create a new file: `src/test2_py_pkg/test2_py_pkg/waiter.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WaiterNode(Node):
    def __init__(self):
        super().__init__('waiter')
        
        # Create a publisher that sends String messages to the '/orders' topic
        # Arguments: Message type, topic name, queue size (10)
        self.publisher = self.create_publisher(String, 'orders', 10)
        
        # Create a timer that calls publish_order every 1.0 second (1Hz frequency)
        self.timer = self.create_timer(1.0, self.publish_order)
        
        self.order_count = 0
        self.get_logger().info('Waiter node started!')
    
    def publish_order(self):
        msg = String()
        self.order_count += 1
        msg.data = f'Order #{self.order_count}: Burger'
        self.publisher.publish(msg)
        self.get_logger().info(f'Announcing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = WaiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Subscriber Node (chef.py)

Create a new file: `src/test2_py_pkg/test2_py_pkg/chef.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ChefNode(Node):
    def __init__(self):
        super().__init__('chef')
        
        # Create a subscription to listen to the '/orders' topic
        # Arguments: Message type, topic name, callback function, queue size (10)
        self.subscription = self.create_subscription(
            String,
            'orders',
            self.order_callback,
            10
        )
        
        # Prevent unused variable warning
        self.subscription
        
        self.get_logger().info('Chef node started! Listening for orders...')
    
    def order_callback(self, msg):
        """This callback is triggered whenever a message arrives on /orders"""
        self.get_logger().info(f'Received order: {msg.data}')
        self.get_logger().info('Starting to cook...')


def main(args=None):
    rclpy.init(args=args)
    node = ChefNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Key Concepts in Publisher Code

**Creating a Publisher**:
```python
self.publisher = self.create_publisher(String, 'orders', 10)
# Arguments:
# - String: Message type (std_msgs/msg/String)
# - 'orders': Topic name
# - 10: Queue size (max messages to buffer if subscriber is slow)
```

**Publishing a Message**:
```python
msg = String()
msg.data = 'Order #1: Burger'
self.publisher.publish(msg)
```

### Key Concepts in Subscriber Code

**Creating a Subscription**:
```python
self.subscription = self.create_subscription(
    String,          # Message type
    'orders',        # Topic name
    self.order_callback,  # Callback function
    10               # Queue size
)
```

**Callback Function**:
```python
def order_callback(self, msg):
    # This runs automatically when a message arrives
    print(f'Received: {msg.data}')
```

## Registering Your Nodes

Update `setup.py` with entry points for both nodes:

```python
entry_points={
    'console_scripts': [
        'waiter=test2_py_pkg.waiter:main',
        'chef=test2_py_pkg.chef:main',
    ],
},
```

## Building and Running

### Step 1: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select test2_py_pkg
```

### Step 2: Source the Workspace

```bash
source install/setup.bash
```

⚠️ **Important**: Source in EVERY new terminal before running nodes!

### Step 3: Run the Nodes (In Separate Terminals)

**Terminal 1 - Run the Waiter (Publisher)**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run test2_py_pkg waiter
```

**Terminal 2 - Run the Chef (Subscriber)**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run test2_py_pkg chef
```

### Expected Output

**Waiter Terminal**:
```
[INFO] [Waiter node started!]
[INFO] [Announcing: Order #1: Burger]
[INFO] [Announcing: Order #2: Burger]
[INFO] [Announcing: Order #3: Burger]
...
```

**Chef Terminal**:
```
[INFO] [Chef node started! Listening for orders...]
[INFO] [Received order: Order #1: Burger]
[INFO] [Starting to cook...]
[INFO] [Received order: Order #2: Burger]
[INFO] [Starting to cook...]
...
```

## Essential ROS2 Topic Commands

### List All Topics

```bash
ros2 topic list
```

**Output**:
```
/orders
/parameter_events
/rosout
```

### Echo Topic Messages in Real-Time

```bash
ros2 topic echo /orders
```

**Output**:
```
data: 'Order #1: Burger'
---
data: 'Order #2: Burger'
---
data: 'Order #3: Burger'
---
```

### Get Topic Information

```bash
ros2 topic info /orders
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Get Topic Rate and Bandwidth

```bash
ros2 topic hz /orders  # Shows publish frequency
ros2 topic bw /orders  # Shows bandwidth usage
```

### View Topic Data Type

```bash
ros2 interface show std_msgs/msg/String
```

**Output**:
```
string data
```

## Understanding Message Types

### Common Message Types

- **std_msgs/msg/String**: Text messages
- **std_msgs/msg/Int32**: Integer values
- **std_msgs/msg/Float32**: Float values
- **geometry_msgs/msg/Twist**: Robot velocity (linear + angular)
- **sensor_msgs/msg/Image**: Camera images
- **sensor_msgs/msg/LaserScan**: LIDAR data

### Using Different Message Types

**Example with Int32**:
```python
from std_msgs.msg import Int32

self.publisher = self.create_publisher(Int32, 'counter', 10)
msg = Int32()
msg.data = 42
self.publisher.publish(msg)
```

## Advanced: Multiple Publishers and Subscribers

### Multiple Subscribers on One Topic

You can have as many subscribers as you want listening to the same topic:

```bash
# Terminal 1: Publisher
ros2 run test2_py_pkg waiter

# Terminal 2: First Subscriber
ros2 run test2_py_pkg chef

# Terminal 3: Second Subscriber
ros2 run test2_py_pkg chef  # Run chef again

# Terminal 4: Monitor with echo
ros2 topic echo /orders
```

All subscribers receive the same messages independently!

### Multiple Publishers on One Topic

You can also have multiple publishers sending to the same topic:

```python
# Both nodes publish to 'orders'
publisher1 = node1.create_publisher(String, 'orders', 10)
publisher2 = node2.create_publisher(String, 'orders', 10)
```

## Common Issues and Debugging

### Issue: "Topic not created"

Topics are created automatically when the first publisher or subscriber is created. If you don't see a topic:
1. Ensure both publisher and subscriber are running
2. Check node names with `ros2 node list`
3. Verify topic name spelling

### Issue: "Subscriber not receiving messages"

Check:
1. Publisher is running and publishing
2. Topic names match exactly (case-sensitive)
3. Message types match
4. Use `ros2 topic echo /topic_name` to verify messages are being published

### Issue: "No messages from subscriber"

Add logging:
```python
def callback(self, msg):
    self.get_logger().info(f'Message received: {msg.data}')
    # Without logging, you won't see if callback is called
```

### Debugging Commands

```bash
# List all active nodes
ros2 node list

# Show information about a node
ros2 node info /waiter

# List all topics
ros2 topic list

# Show topic details
ros2 topic info /orders

# View topic messages in real-time
ros2 topic echo /orders

# Show publish rate
ros2 topic hz /orders
```

## Practice Exercise

### Create a Temperature Sensor Publisher

1. Create a new file: `src/test2_py_pkg/test2_py_pkg/temperature_sensor.py`
2. Implement a node that:
   - Publishes temperature data to a topic `/temperature`
   - Uses Float32 message type
   - Simulates temperature readings (e.g., 20.0°C starting, increasing by 0.1 each second)
   - Logs each published value
3. Create a temperature monitor subscriber that listens and logs temperature
4. Run both and verify they communicate

## Comparison: Topics vs Services vs Actions

| Feature | Topics | Services | Actions |
|---------|--------|----------|----------|
| **Pattern** | Pub-Sub | Request-Response | Asynchronous Goal |
| **Communication** | Asynchronous | Synchronous | Asynchronous with feedback |
| **Type** | Many-to-Many | One-to-One | One-to-One |
| **Best for** | Continuous data (sensors) | Occasional requests | Long-running tasks |
| **Example** | Temperature streaming | Request calculation | Robot navigation |

## Summary

- **Topics** enable asynchronous, many-to-many communication
- **Publishers** send data to topics without knowing about subscribers
- **Subscribers** listen to topics and react to arriving messages
- Topics are created automatically by ROS2
- The Pub-Sub pattern decouples nodes, making systems flexible and scalable
- Use `ros2 topic` commands to inspect and debug topics
- Message types define the structure of data (String, Int32, custom types, etc.)

## Next Lesson

In the next lesson, we'll explore **ROS2 Services** and implement the **Request-Response pattern** for synchronous communication between nodes.
