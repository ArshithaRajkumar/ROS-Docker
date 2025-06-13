### Launch and parameter file 

1. A **parameter file** to hold `v` and `d` values
2. A **launch file** that starts both `talker` and `relay` nodes and uses that parameter file

---

## File 1: `params.yaml`

Create this file inside your package (e.g., `lab_pkg/config/params.yaml`):

```yaml
talker:
  ros__parameters:
    v: 3.0
    d: 1.0
```

The `talker` name must match the name used in `Node(name='talker', ...)` in your launch file.

---

## File 2: `lab1_launch.py`

Create this in `lab_pkg/launch/lab1_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lab_pkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='lab_pkg',
            executable='talker',
            name='talker',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='lab_pkg',
            executable='relay',
            name='relay',
            output='screen'
        )
    ])
```

---

##  `setup.py` Changes

Make sure you include the `launch` and `config` folders in your package:

```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), ['launch/lab1_launch.py']),
    (os.path.join('share', package_name, 'config'), ['config/params.yaml']),
],
```

Also ensure `import os` is at the top of your `setup.py`.

---

##  Rebuild the workspace

```bash
cd /my_ros_project
colcon build --packages-select lab_pkg
source install/setup.bash
```

---

##  Run the Launch File

```bash
ros2 launch lab_pkg lab1_launch.py
```

You should now see both nodes (`talker`, `relay`) running, and the `talker` publishing using the `v` and `d` values from `params.yaml`.

---

Absolutely! Here's the **correct file structure** for your ROS 2 package `lab_pkg` for Deliverable 3 (Launch + Params) in ROS 2 **Foxy**.

---

##  Final Folder Structure for `lab_pkg`

```
my_ros_project/
├── install/
├── build/
├── src/
│   └── lab_pkg/
│       ├── lab_pkg/
│       │   ├── __init__.py
│       │   ├── talker.py
│       │   └── relay.py
│       ├── launch/
│       │   └── lab1_launch.py        ← Launch file here
│       ├── config/
│       │   └── params.yaml           ← Parameter file here
│       ├── package.xml
│       └── setup.py
```


##  Where to Place Each File:

| File             | Location                  |
| ---------------- | ------------------------- |
| `talker.py`      | `src/lab_pkg/lab_pkg/`    |
| `relay.py`       | `src/lab_pkg/lab_pkg/`    |
| `lab1_launch.py` | `src/lab_pkg/launch/`     |
| `params.yaml`    | `src/lab_pkg/config/`     |
| `setup.py`       | `src/lab_pkg/setup.py`    |
| `package.xml`    | `src/lab_pkg/package.xml` |

---

```bash
Changing parameters Live on the terminal
ros2 param set /talker v 4.0
ros2 param set /talker d 1.2
```