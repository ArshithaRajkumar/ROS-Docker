### **Two Python ROS 2 nodes** 

inside your package `lab_pkg`:

| Node        | Role                                                                                                 |
| ----------- | ---------------------------------------------------------------------------------------------------- |
| `talker.py` | Reads parameters `v` and `d`, publishes them as `AckermannDriveStamped` messages to `/drive`         |
| `relay.py`  | Subscribes to `/drive`, multiplies speed and steering\_angle by 3, and republishes to `/drive_relay` |



## **Concepts Involved**

| Concept          | What it does                                                          |
| ---------------- | --------------------------------------------------------------------- |
| ROS 2 Parameter  | Stored config value (e.g., speed `v`, steering angle `d`)             |
| Publisher        | Sends out messages to a topic                                         |
| Subscriber       | Listens to messages on a topic                                        |
| Topic            | Named channel like `/drive`, `/drive_relay`                           |
| `ackermann_msgs` | ROS message used to describe vehicle control (speed + steering angle) |

---

## 📁 Folder Structure (inside package)

```
lab_pkg/
├── lab_pkg/
│   ├── __init__.py
│   ├── talker.py        <-- Your first node
│   └── relay.py         <-- Your second node
├── package.xml
├── setup.py
└── resource/
    └── lab_pkg
```


##  Step-by-Step Plan

### 1. Create your Python node files

Inside your package’s Python directory (`lab_pkg/lab_pkg/`):

#### `talker.py`:

```python
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Declare and get parameters
        self.declare_parameter('v', 1.0)
        self.declare_parameter('d', 0.5)

        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        v = self.get_parameter('v').value
        d = self.get_parameter('d').value

        msg = AckermannDriveStamped()
        msg.drive.speed = float(v)
        msg.drive.steering_angle = float(d)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: speed={v}, steering_angle={d}')


def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

#### `relay.py`:

```python
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay')

        self.subscriber = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def callback(self, msg):
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = msg.drive.speed * 3
        new_msg.drive.steering_angle = msg.drive.steering_angle * 3
        self.publisher.publish(new_msg)
        self.get_logger().info(f'Relaying: speed={new_msg.drive.speed}, steering_angle={new_msg.drive.steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 2. Update `setup.py` to include the new Python entry points

Make sure `setup.py` includes:

```python
entry_points={
    'console_scripts': [
        'talker = lab_pkg.talker:main',
        'relay = lab_pkg.relay:main',
    ],
},
```

---

###  3. Build Your Package

In your Docker container:

```bash
cd /my_ros_project
colcon build
```

Then source it:

```bash
source install/setup.bash
```

---

###  4. Set ROS Parameters (from terminal)

You can set parameters while running the node:

```bash
ros2 run lab_pkg talker --ros-args -p v:=2.0 -p d:=0.8
```

Or set them via a YAML or launch file later.

---

###  5. Run the Nodes and Observe Topics

In separate terminals:

```bash
# Terminal 1
ros2 run lab_pkg talker --ros-args -p v:=1.5 -p d:=0.6

# Terminal 2
ros2 run lab_pkg relay
```

You can monitor messages like:

```bash
ros2 topic echo /drive
ros2 topic echo /drive_relay
```

---
