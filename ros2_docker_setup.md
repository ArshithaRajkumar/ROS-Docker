**summary of Docker + ROS 2 setup**

---

## Goal:

**set up ROS 2 Foxy** in a **Docker container**, while keeping the code **editable from Windows**, and follow a clean workspace structure.

---

### 1. **Installation Of Docker**

* installed Docker Desktop on Windows.
* Purpose: Run Linux containers (like ROS 2) on Windows machine.
* Checking the installation and Creating the First Container. Run this command:
   ```bash
  docker run -d -p 8080:80 docker/welcome-to-docker
  ```

---

### 2. **Understanding Docker Image vs Container**

* **Image**: A frozen template (e.g., `ros:foxy`) — like a program installer.
* **Container**: A running instance of that image — like a live program session.
* You use `docker run` to create and start a container from an image.

---

### 3. **Workspace Location**

* chose:
  `C:\Users\DOPPS\Desktop\my_ros_project`
* Created subfolder:
  `C:\Users\DOPPS\Desktop\my_ros_project\src`
* Why? ROS 2 workspaces **must** contain a `src` folder where packages are stored and built using `colcon`.

---

### 4. **Created the Directory**

* Command used:

  ```bash
  mkdir -p "C:\Users\DOPPS\Desktop\my_ros_project\src"
  ```

---

### 5. **Running the Docker Container**

* Full command:

  ```bash
  docker run -it -v "C:\Users\DOPPS\Desktop\my_ros_project\src:/my_ros_project/src" --name my_ros_project ros:foxy
  ```
* What it does:

  * `-it`: Interactively opens a terminal in the container.
  * `-v ...:/my_ros_project/src`: Mounts your Windows `src` folder into the container.
  * `--name my_ros_project`: Names the container.
  * `ros:foxy`: Uses the ROS 2 Foxy image.

What this did:
🔸 Starts a container using the official ROS 2 Foxy image

🔸 Mounts your local folder into the container at /my_ros_project/src

🔸 Names the container my_ros_project

🔸 Opens an interactive terminal into that container

This lets you work with ROS 2 inside Linux without needing Ubuntu installed.  

---

### 6. **Why Mounting Was Important**

* This lets  **edit files on Windows** (VS Code, GitHub, etc.) while still **building and running them in Linux** (inside the container).
* The container sees your ROS packages inside `/my_ros_project/src`.

---

### 7. **Optional Tmux Use**

* You can launch or enter a `tmux` session.
* You learned to:
  * **Add New Window** `ctrl+b` , then `c`
  * **Move around** `ctrl+b`, then `n` or `p`
  * **Detach** using `Ctrl + b`, then `d`
  * **Reattach** with `tmux attach`
  * **Exit** with `exit` or `Ctrl + d`

---
