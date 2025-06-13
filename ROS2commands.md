## **clear explanation of each ROS 2 CLI command**

### `ros2 topic list`

**Purpose**:
Shows all active topics in the ROS 2 system.

**Example output**:

```
/drive
/tf
/parameter_events
```

**Use it when**:
You want to see which topics are currently being published or subscribed to by nodes.

---

### `ros2 topic info drive`

⚠️ This is **missing a leading slash**. You should use:

```bash
ros2 topic info /drive
```

**Purpose**:
Displays information about the `/drive` topic, including:

* Type of message being published
* Number of publishers and subscribers

**Example output**:

```
Type: ackermann_msgs/msg/AckermannDriveStamped
Publisher count: 1
Subscription count: 2
```

**Use it when**:
You want to understand what's being communicated on the topic, and which nodes are involved.

---

### `ros2 topic echo drive`

⚠️ Again, better to write:

```bash
ros2 topic echo /drive
```

**Purpose**:
Prints the actual **message data** being published on the `/drive` topic in real-time.

**Example output**:

```yaml
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
drive:
  steering_angle: 1.0
  speed: 3.0
  ...
```

**Use it when**:
You want to debug or inspect the values being transmitted over a topic.

---

### `ros2 node list`

**Purpose**:
Shows all currently running nodes.

**Example output**:

```
/talker
/relay
```

**Use it when**:
You want to check which nodes are alive and active in your ROS graph.

---

### `ros2 node info talker`

⚠️ Should be written as:

```bash
ros2 node info /talker
```

**Purpose**:
Displays detailed info about a specific node (`/talker`), such as:

* Topics it publishes
* Topics it subscribes to
* Services and parameters used

**Use it when**:
You want to inspect how a node is connected and what its role is in the system.

---

### `ros2 node info relay`

Same as above, for the `/relay` node:

```bash
ros2 node info /relay
```

**Use it when**:
You want to check how the `relay` node processes data — for example, whether it's subscribing to `/drive` and republishing modified data.

---

### TL;DR Summary

| Command                  | What It Does                                 | Use Case Example                      |
| ------------------------ | -------------------------------------------- | ------------------------------------- |
| `ros2 topic list`        | Lists all current topics                     | See what data is flowing in ROS       |
| `ros2 topic info /drive` | Shows type, publishers, subscribers of topic | Debugging or inspecting `/drive`      |
| `ros2 topic echo /drive` | Shows live message data from topic           | Monitor message content in real time  |
| `ros2 node list`         | Lists all running nodes                      | See what nodes are active             |
| `ros2 node info /talker` | Shows details about `/talker` node           | Check what `/talker` is publishing    |
| `ros2 node info /relay`  | Shows details about `/relay` node            | Understand how `/relay` modifies data |
