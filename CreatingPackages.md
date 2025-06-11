#### Create a ROS 2 package named `lab_pkg` 
inside your workspace that:

* Supports  Python 
* Depends on `ackermann_msgs` (used in autonomous driving)
* Follows clean folder structure


###  Step 1: **Start and Access Docker Container** (Inside VS code if needed)

In terminal

```bash
docker start my_ros_project
docker exec -it my_ros_project bash
```

* This **starts** your already-created container (`my_ros_project`) and **enters** it.
* Containers run Linux — where your ROS 2 environment lives.



### Step 2: **Source ROS 2**

```bash
source /opt/ros/foxy/setup.bash
```

* This loads ROS 2 into your current terminal session.
* It’s like saying: *“I want to start using ROS 2 commands.”*

# Automate It
You can add this line to your Docker container’s .bashrc so it's loaded automatically:

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
That way, every new terminal inside your container automatically has ROS sourced.
```



### Step 3: **Go to the ROS Workspace `src` Folder**

```bash
cd /my_ros_project/src
```

* You navigate into the `src` folder inside your workspace.
* ROS packages **must** live inside a `src` folder to be found and built.



### Step 4: **Create the ROS 2 Package**



```bash
ros2 pkg create lab_pkg --build-type ament_python --dependencies rclpy ackermann_msgs
```

This command:

* Creates a folder called `lab1_pkg`
* Uses `ament_cmake` (standard build system for ROS 2)
* Automatically sets up dependencies:

  * `rclcpp`: ROS Client Library for C++
  * `rclpy`: ROS Client Library for Python
  * `ackermann_msgs`: Message type for controlling robot movement in a car-like way

So you get a starting template for both C++ and Python ROS code.



### Step 5: **Edit `package.xml` (Optional but important)**

You make sure it lists your dependencies so ROS knows what packages this one relies on.

Inside `lab1_pkg/package.xml`, make sure these are present:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>ackermann_msgs</exec_depend>
```



### Step 6: **Keep Folder Clean**

Make sure:

* You don't create **another `src` inside `lab1_pkg`**
* Don't move or copy in `build/`, `install/` folders — they’ll be generated



### Step 7: **Build the Workspace**

```bash
cd /my_ros_project
colcon build
```

* This compiles all packages in your workspace.
* `colcon` is the standard build tool for ROS 2 (like `make` for ROS 1).



### Step 8: **Install Missing Dependencies**

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

* `rosdep` automatically installs system-level packages required by your ROS packages.
* Saves you from installing each thing manually (e.g., `sudo apt install ros-foxy-ackermann-msgs`)

---

## Summary:

| Command                           | Why You Use It                           |
| --------------------------------- | ---------------------------------------- |
| `ros2 pkg create ...`             | Make a new ROS 2 package                 |
| `source /opt/ros/foxy/setup.bash` | Load ROS 2 tools in terminal             |
| `colcon build`                    | Compile the whole workspace              |
| `rosdep install`                  | Auto-install missing dependencies        |
| `docker exec -it`                 | Get inside your container                |
| `-v ...:/my_ros_project/src`      | Sync code from Windows into Docker Linux |

---

Example File Structure:

``` bash
/my_ros_project/
├── src/
│   └── lab_pkg/         ← You will create this here
├── build/               ← auto-generated
├── install/             ← auto-generated
└── log/                 ← auto-generated
```

```bash
/my_ros_project/src/lab_pkg/
├── package.xml
├── setup.py
├── lab_pkg/
│   └── __init__.py
├── resource/
│   └── lab_pkg
└── test/
    └── test_lab_pkg.py
```    
