# Lesson 05: ROS2 Services - Request/Response Communication

## Quick Summary

Services provide **synchronous, blocking** communication for occasional requests, unlike Topics which are asynchronous and continuous.

## Key Difference: Topics vs Services

| Aspect | Topics | Services |
|--------|--------|----------|
| **Pattern** | Publish-Subscribe (async) | Request-Response (sync) |
| **Participants** | Many-to-Many | Client-Server |
| **Blocking** | No | Yes - client waits |
| **Use Case** | Sensor streams | Commands, calculations |

## Service Definition (.srv file)

```
int64 a
int64 b
---
int64 sum
```

**Format**: Request fields above `---`, response fields below

## Server Implementation (Python)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Server(Node):
    def __init__(self):
        super().__init__('server')
        self.srv = self.create_service(
            AddTwoInts,
            'add',
            self.handle
        )
    
    def handle(self, request, response):
        response.sum = request.a + request.b
        return response

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Server())
    rclpy.shutdown()
```

## Client Implementation (Python)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Client(Node):
    def __init__(self):
        super().__init__('client')
        self.cli = self.create_client(AddTwoInts, 'add')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
    
    def call(self, a, b):
        req = AddTwoInts.Request()
        req.a, req.b = a, b
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = Client()
    result = client.call(5, 3)
    print(f'5 + 3 = {result.sum}')
    rclpy.shutdown()
```

## CLI Commands

```bash
# List services
ros2 service list

# Call service from command line
ros2 service call /add example_interfaces/srv/AddTwoInts '{a: 10, b: 5}'

# Get service info
ros2 service info /add

# Find service type
ros2 service type /add
```

## Next: Lesson 06 - Launch Files

Use launch files to start multiple nodes with one command!
