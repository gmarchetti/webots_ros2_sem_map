# Webots Environment for Semantic Mapping

This folder contains a Webots simulation project named "WebotsLidarSim" developed using Webots **2023b**. The simulation of a controller to allow access to the robot sensors.

## Description

This Webots simulation showcases the use of a LiDAR and other sensors for perception and navigation tasks. The `main_controller.py` script handles the acquisition and processing of LiDAR data, allowing the robot to perceive its surroundings. The specific functionality and behavior of the robot depend on the implementation within the controller script. The current map built in the simulation contains some rooms and furniture, which can be used to test the robot's mapping and navigation capabilities.

## Running the Simulation

First, open the Webots application, navigate to the project folder, and open the `worlds` directory. Then, open the desired world file (e.g., `lidar_world.wbt`) to load the simulation environment. After loading the world, you can run the simulation by clicking the "Play" button in the Webots interface. However, this will not start the Python controller script; you need to run that separately.

To run the Python controller script, follow these steps:

1. **Navigate to the controller directory:**

   ```bash
   cd controllers/main_controller
   ```

2. **Execute the Python controller script:**

   ```bash
   python main_controller.py
   ```

This will start the Webots simulation environment and start the `main_controller.py` script, which controls the behavior of the robot equipped with the LiDAR sensor.

### Controlling the bot

By default the `main_controller.py` script allow the user to control the robot using the keyboard. The following keys can be used to control the robot:

- `W` : Move the robot forward
- `S` : Move the robot backward
- `A` : Turn the robot left
- `D` : Turn the robot right

### Robot data

In the `main_controller.py` script, the robot data is printed to the console. This provides a simple example of how to access and process the data from the robot's sensors. The following sensors are used in the simulation:

- **LiDAR Sensor:** The LiDAR sensor provides distance measurements in the form of a 3D array.
- **Camera Sensor:** The camera sensor provides RGB images of the environment, it is positioned in the front of the robot.
- **GPS Sensor:** The GPS sensor provides the position of the robot in the environment.
- **IMU Sensor:** The IMU sensor provides the orientation of the robot.

### ROS2 Compatibility

The `main_controller.py` script is also compatible with ROS2. If you have ROS2 installed on your system, you can enable the ROS2 functionality in the script. To do this, uncomment the following lines in the `main_controller.py` script:

```python
  # Uncomment the following code to run ROS2 nodes in parallel with the Webots controller.
  # rclpy.init(args=None)

  # ros2_publisher = ROS2Publisher()
  # ros2_subscriber = ROS2Subscriber()

  # try:
  #     rclpy.spin(ros2_publisher)
  # except KeyboardInterrupt:
  #     ros2_publisher.destroy_node()
  #     rclpy.shutdown()
```

The current implementation of the controller has two classes that can serve as examples for ROS2 publishers and subscribers. The `ROS2Publisher` class publishes a message to a topic, and the `ROS2Subscriber` class subscribes to a topic and prints the received message. You can modify these classes to suit your specific needs.

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

- **`controllers/main_controller`:** This folder contains the Python script (`main_controller.py`) responsible for controlling the robot and processing LiDAR data.
- **`worlds`:** This folder contains the Webots world file defining the simulation environment, including the robot, LiDAR sensor, and other objects. In this folder it is also possible to find the textures used in the simulation.

## Important Note: Webots Path in `main_controller.py`

**Please remember to modify line 13 in the `main_controller.py` file to reflect the correct installation path of Webots on your computer.** For example, if your Webots installation is located at `/usr/local/webots`, the line should look like this:

```python
# ... other code ... (Line 12)
sys.path.append('[WEBOTS INSTALLATION FOLDER]/lib/controller/python') # Line 13 (Adjust this path)
# ... rest of the code ...
```

**Adapt this path according to your specific Webots installation directory.**

## ROS Error

If you encounter an error related to ROS when running the simulation, you can disable ROS by following these steps:

- Comment the import of the ROS package in the `main_controller.py` file:

  ```python
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String
  from geometry_msgs.msg import Pose
  ```

- Comment the ROS-related clases in the `main_controller.py` file:

  ```python
  class ROS2Subscriber(Node):
    def __init__(self):
        super().__init__("ros2_subscriber")
        self.subscription = self.create_subscription(
            String, "webots", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

  class ROS2Publisher(Node):
    def **init**(self):
    super().**init**("ros2*publisher")
    self.publisher* = self.create_publisher(String, "chatter", 10)
    timer_period = 0.5
    self.timer = self.create_timer(timer_period, self.timer_callback)

        def timer_callback(self):
            msg = String()
            msg.data = "Hello Webots"

            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
  ```
