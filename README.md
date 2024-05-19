# Robot localization with GTSAM
Project for course "Artificial Intelligence in Robotics"

## Table of Contents
- [Robot localization with GTSAM](#robot-localization-with-gtsam)
  - [Table of Contents](#table-of-contents)
  - [1. About the project](#1-about-the-project)
  - [2. Installing dependencies](#2-installing-dependencies)
    - [Create a workspace](#create-a-workspace)
    - [a.) Using Ubuntu 22.04 and ROS2 Humble (without Docker)](#a-using-ubuntu-2204-and-ros2-humble-without-docker)
      - [Install the following modules](#install-the-following-modules)
    - [b.) With Docker](#b-with-docker)
      - [Building the container](#building-the-container)
      - [Starting the container](#starting-the-container)
        - [Opening more terminals in the container](#opening-more-terminals-in-the-container)
  - [3. Running the simulation](#3-running-the-simulation)
    - [Workspace preparation](#workspace-preparation)
    - [Build the package](#build-the-package)
    - [Start the simulation](#start-the-simulation)

## 1. About the project

## 2. Installing dependencies

### Create a workspace

```
mkdir ~/turtlebot_localization_ws && cd ~/turtlebot_localization_ws
```

```
git clone https://github.com/mmcza/Robot-localization-with-GTSAM
```

### a.) Using Ubuntu 22.04 and ROS2 Humble (without Docker)

#### Install the following modules

```
sudo apt-get install -y ros-humble-turtlebot3-gazebo && sudo apt-get install -y ros-humble-nav2-bringup && sudo apt-get install -y ros-humble-navigation2 && sudo apt-get install -y ros-humble-gtsam && sudo apt-get install -y python3-pip
```
```
pip install open3d && pip install gtsam && pip install numpy==1.24.4
```

### b.) With Docker

#### Building the container

```
cd Robot-localization-with-GTSAM && docker build -t robot_localization .
```
#### Starting the container
```
bash start_container.sh 
```

##### Opening more terminals in the container
```
docker exec -ti robot_localization bash
```

> [!NOTE]
> The `robot_localization_ws` directory is shared between the host and container. In the result files inside of it might require sudo privileges to save any changes.

> [!NOTE]
> Dockerfile and script for running container are based on [Rafał Staszak's repository](https://github.com/RafalStaszak/NIMPRA_Docker/)

## 3. Running the simulation

### Workspace preparation

To make the project work we need to prepare the Gazebo simulator properly

```
source /opt/ros/humble/setup.bash
```
```
export TURTLEBOT3_MODEL=waffle
```
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```
```
source /usr/share/gazebo/setup.bash
```

### Build the package
Inside of your workspace (NOT inside your src)

```
colcon build --symlink-install
```
```
source install/setup.bash
```

### Start the simulation

In first terminal start the simulation using the following command
```
ros2 launch nav2_bringup tb3_simulation_launch.py
```

In second terminal (make sure to source the local setup using `source install/setup.bash`) start the pose estimator
```
ros2 run robot_localization estimator
```