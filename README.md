# Webots Environment for Semantic Mapping

This folder contains a Webots simulation project named "WebotsLidarSim" developed using Webots **2023b**. The simulation of a controller to allow access to the robot sensors.

## Description

This project was implemented using the Jazzy Jalisco ROS2 release and the Webots 2023b release. It is recommend to use this specific version to run the project.

Installation instructions for the Jazzy Jalisco can be found at: https://docs.ros.org/en/jazzy/Installation.html
The Webots simulator can be downloaded and installed from: https://cyberbotics.com/#download

This projects assumes that the default install paths are used for the Webots simulator

The simulation showcases the use of a LiDAR and other sensors for perception and navigation tasks. The main controller of the robot is implemented in the `robot_controller.py` script. This script is responsible for initializing the devices that the simulated robot has, as well as creating the required ros Publisher and Subscribers required for it to function.

## How-to build

The build system used in this project is based on the colcon toolkit, so ensure that it is installed first:

    sudo apt install python3-colcon-common-extensions

All the next commands assume that the project files were extracted on the "src" folder of "ros2_ws", created during the steps of the Jazzy Jalisco tutorial, the paths should be adjusted
if this is not your case:

- Install all the dependencies of the project by running:

        sudo rosdep init
        rosdep update
        rosdep install --from-paths src -y --ignore-src

- Build the package:

        colcon build --packages-select webots_ros2_sem_map

## Running the Simulation

Once built, the project can be run by the following commands:

    source install/local_setup.bash
    ros2 launch wall_following_robots robot_launch.py

This should automatically launch the Webots installation and connect the controllers to the robots. Besides the Webots controller, the Slam Toolbox package is also started. This package is responsible for providing the /map messages with the occupancy data required for the semantic mapping of the environment.

### Controlling the bot

The robot is compatible with the Telop Twist Keyboard ros package and is declared as a dependency on the `package.xml` file. To run teleop, simply run

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

on a new terminal and it should be able to control the robot with the keyboard.

### Robot data

In the `robot_controller.py` script, the robot data is printed to the console. This provides a simple example of how to access and process the data from the robot's sensors. The following sensors are used in the simulation:

- **LiDAR Sensor:** The LiDAR sensor provides distance measurements in the form of a 3D array.
- **Camera Sensor:** The camera sensor provides RGB images of the environment, it is positioned in the front of the robot.
- **GPS Sensor:** The GPS sensor provides the position of the robot in the environment.
- **IMU Sensor:** The IMU sensor provides the orientation of the robot.

## Requirements

- **Webots 2023b:** This simulation was developed and tested using Webots 2023b. While it might work with other versions, compatibility is not guaranteed. Download and install Webots 2023b from the official website: [https://cyberbotics.com/](https://cyberbotics.com/)

- **Python (using conda):**
  - It is recommended to use Python version 3.11. You can manage your Python environment using conda.
  - Create a conda environment (e.g., `webots_env`) with Python 3.11:
  ```bash
  conda create -n webots_env python=3.11
  ```
  - Activate the environment:
  ```bash
    conda activate webots_env
  ```
  - Install the required dependencies:
  ```bash
    pip install -r requirements.txt
  ```

## Project Structure

  .
  └── launch # ROS2 launch file for the package
      └── robot_launch.py

  └── resource # Contains .urdf files for the robot used in the simulation and configurations for the Slam Toolbox package
      └── mapper_params_online_async.yaml
      └── robot_webots.urdf
      └── webots_ros2_sem_map

  └── webots_ros2_sem_map # ROS2 Python scripts for the robot and sensor
      └── __init__.py
      └── camera_img_handler.py
      └── map_info.py
      └── robot_controller.py

  └── worlds # Resource files used by the Webots simulator
      └── apartment.wbt
