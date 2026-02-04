# Lesson 07: Smart Restaurant System - Complete Project

## Project Overview

Build a complete ROS2 robotics system that integrates ALL previous lessons:

- **Lesson 04**: Publisher/Subscriber nodes (Waiter, Chef)
- **Lesson 05**: Custom services (Restaurant Manager)
- **Lesson 06**: Launch files (Orchestrate the system)

## System Architecture

```
Waiter (Publisher)
    ↓
    └→ /orders topic
        ↓
        └→ Chef (Subscriber)

Manager (Service Server)
    ↓
    └→ /add_order service
        ↓
        └→ Restaurant System

Launch File
    ↓
    └→ Starts all nodes with one command!
```

## Project Structure

```
ros2_ws/src/
├── restaurant_interfaces/  (CMake package)
│   ├── srv/
│   │   └── AddOrder.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
└── restaurant_system/  (Python package)
    ├── restaurant_system/
    │   ├── __init__.py
    │   ├── waiter.py        (Publisher)
    │   ├── chef.py          (Subscriber)
    │   └── manager.py       (Service Server)
    ├── launch/
    │   └── restaurant.launch.py
    ├── setup.py
    └── package.xml
```

## Assignment Tasks

### Task 1: Define Custom Service

File: `restaurant_interfaces/srv/AddOrder.srv`

```
string item_name
int32 quantity
---
bool accepted
string message
```

### Task 2: Implement Waiter (Publisher)

Publishes orders every 2 seconds to `/orders` topic
- Announce `"Order #X: Burger"`
- Log each order

### Task 3: Implement Chef (Subscriber)

Listens to `/orders` topic
- Receives orders
- Logs receipt
- Logs "Cooking..."

### Task 4: Implement Manager (Service Server)

Handles `/add_order` service requests
- Accept specific item requests (burger, pizza, salad)
- Reject unknown items
- Return response message

### Task 5: Create Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='restaurant_system', executable='chef'),
        Node(package='restaurant_system', executable='manager'),
        Node(package='restaurant_system', executable='waiter'),
    ])
```

## Build & Run

```bash
# Build interfaces first
colcon build --packages-select restaurant_interfaces

# Build system package
colcon build --packages-select restaurant_system
source install/setup.bash

# Run with launch file
ros2 launch restaurant_system restaurant.launch.py
```

## Verification

In separate terminals:

```bash
# List active topics
ros2 topic list

# Echo orders
ros2 topic echo /orders

# Call service
ros2 service call /add_order restaurant_interfaces/srv/AddOrder '{item_name: "burger", quantity: 2}'
```

## Learning Outcomes

After completing this project, you will have:

✓ Implemented publisher and subscriber nodes
✓ Created custom service definitions  
✓ Built a service server and client
✓ Orchestrated multiple nodes with launch files
✓ Integrated ROS2 communication patterns
✓ Built a complete robotics system

## Expected Output

```
[waiter]: Order #1: Burger
[manager]: Restaurant Server started
[chef]: Received order: Order #1: Burger
[chef]: Starting to cook...
[manager]: Service call received
```

## Extensions (Optional)

1. Add more food items to the menu
2. Implement cooking time simulation
3. Add multiple chefs subscribing to orders
4. Create a CLI client to submit orders via service
5. Add logging to a file
6. Create a node that monitors restaurant status

## Troubleshooting

**Nodes don't communicate**: Verify topic/service names match
**Build fails**: Ensure interfaces package is built first
**Service not responding**: Check manager node is running

## Congratulations!

You've completed the ROS2 MSc Applied AI course!

### Next Steps:

- Explore [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- Build your own robot projects
- Learn about advanced topics:
  - Timers and callbacks
  - Parameter server
  - Actions
  - TF (Transform Framework)
  - Navigation Stack
