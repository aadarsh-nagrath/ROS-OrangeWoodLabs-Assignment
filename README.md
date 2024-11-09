
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
