
# ROS Noetic Setup in Docker

This repository contains the steps and configurations used to set up **ROS Noetic** in a **Docker** container, as part of an assignment. Below, you'll find instructions on how to install Docker, pull the ROS Noetic Docker image, configure display forwarding, and test the ROS environment.

## Prerequisites

- A machine running Ubuntu 20.04 (either natively or in a virtual machine).
- Docker installed on your system.

## Step 1: Install Docker

To install Docker on your Ubuntu system, follow these steps:

1. Update your package lists:
   ```bash
   sudo apt update
   ```

2. Install Docker:
   ```bash
   sudo apt install docker.io
   ```

3. Verify that Docker is installed correctly:
   ```bash
   sudo docker --version
   ```

## Step 2: Pull the ROS Noetic Docker Image

The next step is to pull the official ROS Noetic Docker image, which is based on Ubuntu 20.04 (Focal Fossa). Use the following command to do this:

```bash
sudo docker pull ros:noetic-ros-base-focal
```

This will download the base image for ROS Noetic.

## Step 3: Start the Docker Container

Now, start a Docker container using the ROS Noetic base image. Run the following command:

```bash
sudo docker run -it ros:noetic-ros-base-focal bash
```

This will start the container and open an interactive shell within it.

## Step 4: Map the X11 Display for GUI Applications

If you wish to run graphical applications like `turtlesim`, you'll need to map the X11 display from your host system to the Docker container. This will allow GUI-based applications to display on your host system.

Run the following command:

```bash
docker run -it --rm \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  ros:noetic-ros-base-focal
```

This ensures that your graphical applications can be displayed on the host system.

## Step 5: Set Up ROS in the Container

Once inside the Docker container, you'll need to source the ROS setup file to configure your ROS environment:

```bash
source /opt/ros/noetic/setup.bash
```

This step must be done every time you start a new terminal session in the Docker container.

## Step 6: Test ROS Installation

To test the ROS installation and make sure everything is set up correctly, start the **ROS Master** by running:

```bash
roscore
```

![image](https://github.com/user-attachments/assets/c3804b1e-9657-4eb7-b4c3-43870f8c1539)


This will start the central server (ROS Master) needed for your ROS nodes to communicate. You should see output like:

```bash
... logging to /root/.ros/log/...-roscore-xxx.log
started roslaunch server http://localhost:11311/
ros_comm version 1.15.10
```

Leave this terminal running because **roscore** needs to be active for your ROS nodes to communicate.

## Step 7: Run the Turtlesim Node

In a new terminal (or a new tab), open the Docker container shell again, source the ROS setup file, and then run the Turtlesim node:

```bash
rosrun turtlesim turtlesim_node
```

This should open the Turtlesim window, assuming that display forwarding is set up correctly.

![image](https://github.com/user-attachments/assets/382f8813-9e2d-4504-a711-8c08b57e2fe8)


Here’s the continuation of the **README.md** with the steps you’ve outlined:

---

## Step 8: Access the Running Docker Container

If you're not already inside the running Docker container where ROS is set up, open a new terminal and access the container by running:

```bash
sudo docker exec -it <container_name_or_id> bash
```

Replace `<container_name_or_id>` with the name or ID of your running container. This command will give you a new terminal session inside the running container.

## Step 9: Create and Set Up Your ROS Workspace

Now, inside the Docker container terminal, follow these steps to set up your **catkin workspace**.

### 1. Create the catkin workspace (if not already created):

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```

This will create the `catkin_ws` workspace with a `src` folder.

### 2. Clone the provided ROS package:

Navigate to the `src` directory inside your `catkin_ws` and clone the provided repository.

```bash
cd ~/catkin_ws/src
git clone https://github.com/topguns837/ros_session.git
```

This will download the repository containing the ROS package.

### 3. Install catkin build dependencies:

Before you can build the workspace, you need to install the necessary dependencies to use `catkin_make`.

Run the following commands to install the ROS build dependencies:

```bash
sudo apt-get install ros-noetic-catkin
```

Also, install the **catkin-tools** to manage your build process:

```bash
sudo apt-get install python3-catkin-tools
```

This will install the `catkin_make` tool and other necessary utilities to help you build your workspace.

### 4. Source the ROS setup files:

After installing the dependencies, make sure to source the ROS environment setup file before you run the build process:

```bash
source /opt/ros/noetic/setup.bash
```

### 5. Build the workspace using `catkin_make`:

Now that everything is set up, navigate back to your workspace and build it using `catkin_make`:

```bash
cd ~/catkin_ws
catkin_make
```

This will compile the package and generate the necessary files.

![image](https://github.com/user-attachments/assets/85996bd2-3554-4e69-999c-a9d27420d22f)


### 6. Check if `catkin_make` is available:

If you face issues with `catkin_make`, ensure that it's properly installed. You can check if `catkin_make` is available by running:

```bash
which catkin_make
```

If it returns the path to `catkin_make` (e.g., `/opt/ros/noetic/bin/catkin_make`), then the command is available. Otherwise, there might be a problem with the installation.

### 7. Source the workspace setup file:

Once the build process is complete, you need to source the workspace's setup file so that ROS can recognize your new packages:

```bash
source devel/setup.bash
```

## Step 10: Look for Movement Scripts

Next, let's locate the scripts that control the turtle’s movement.

### 1. List the contents of the `scripts` directory:

Navigate to the `scripts` directory inside the cloned package:

```bash
cd ~/catkin_ws/src/ros_session/scripts
ls
```

You should see scripts that control the turtle’s movement, such as `move_circle.py` and `move_straight.py`.

### 2. Ensure the scripts are executable:

Make sure that the scripts are executable by running the following commands:

```bash
chmod +x ~/catkin_ws/src/ros_session/scripts/move_circle.py
chmod +x ~/catkin_ws/src/ros_session/scripts/move_straight.py
```

This will grant execute permissions to the Python scripts.

## Step 11: Run the ROS Node Manually

Now, you can manually run the ROS nodes to see if they control the turtle's movement.

### 1. For circular movement:

```bash
rosrun ros_session move_circle.py
```

This will make the turtle move in a circular path. If everything is working correctly, you should see the turtle moving in the turtlesim window.

![image](https://github.com/user-attachments/assets/18c0c8c0-33a7-4591-8c04-eeb1980a6537)


### 2. For straight-line movement:

```bash
rosrun ros_session move_straight.py
```

This will make the turtle move in a straight line. Again, check the turtlesim window to confirm the movement.

### 3. Check for errors:

If the turtle does not move as expected, check the terminal output for any error messages. If there are issues, make sure that:

- The scripts are properly executed.
- ROS is sourced correctly.
- The required dependencies are installed.


