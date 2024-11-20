"""main_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from typing import Callable

import numpy as np
import threading
import traceback
import keyboard
import time
import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose

sys.path.append(
    os.path.join("C:", "Program Files", "Webots", "lib", "controller", "python")
)  # !Change the path to the Webots python controller library

from controller import Robot
from controller.motor import Motor
from controller.camera import Camera
from controller.lidar import Lidar


def parse_image(camera: Camera) -> np.ndarray:
    """Parses an image from a camera and returns it as a NumPy array.

    :param camera: Object representing the camera with methods getImage(), getWidth(), and getHeight().
    :type camera: Camera
    :return: Parsed image data reshaped to (height, width, 4).
    :rtype: numpy.ndarray
    :raises AttributeError: If the camera object does not have required methods.
    :raises ValueError: If the image data cannot be reshaped correctly.
    """
    try:
        image = camera.getImage()
        width = camera.getWidth()
        height = camera.getHeight()
        image_array = np.frombuffer(image, dtype=np.uint8)
        return image_array.reshape((height, width, 4))
    except AttributeError:
        print("An error occurred due to missing methods in the camera object.")
        traceback.print_exc()
    except ValueError:
        print("An error occurred while reshaping the image data.")
        traceback.print_exc()


def parse_lidar(lidar: Lidar) -> np.ndarray:
    """Parses a range image from a lidar and returns it as a NumPy array.

    :param lidar: Object representing the lidar with methods getRangeImage(), getHorizontalResolution(), and getNumberOfLayers().
    :type lidar: Lidar
    :return: Parsed range data reshaped to (height, width).
    :rtype: numpy.ndarray
    :raises AttributeError: If the lidar object does not have required methods.
    :raises ValueError: If the range image data cannot be reshaped correctly.
    """
    try:
        range_data = np.array(lidar.getRangeImage())
        width = lidar.getHorizontalResolution()
        height = lidar.getNumberOfLayers()
        return range_data.reshape((height, width))
    except AttributeError:
        print("An error occurred due to missing methods in the lidar object.")
        traceback.print_exc()
    except ValueError:
        print("An error occurred while reshaping the range image data.")
        traceback.print_exc()


def command_thread_factory(
    l1_motor: Motor, r1_motor: Motor, max_speed: float
) -> Callable[[], None]:
    """Creates a thread that controls the robot's movement based on keyboard input.

    :param l1_motor: The left motor of the robot.
    :type l1_motor: Motor
    :param r1_motor: The right motor of the robot.
    :type r1_motor: Motor
    :param max_speed: The maximum speed of the motors.
    :type max_speed: float
    :param break_command: A flag indicating whether the thread should break.
    :type break_command: bool
    :return: A thread that controls the robot's movement.
    :rtype: threading.Thread
    """

    def command_thread():
        """Controls the robot's movement based on keyboard input."""
        while True:
            try:
                angular_speed_reduction = 5
                linear_speed_reduction = 2
                if keyboard.is_pressed("w"):
                    l1_motor.setVelocity(-max_speed / linear_speed_reduction)
                    r1_motor.setVelocity(-max_speed / linear_speed_reduction)
                elif keyboard.is_pressed("s"):
                    l1_motor.setVelocity(max_speed / linear_speed_reduction)
                    r1_motor.setVelocity(max_speed / linear_speed_reduction)
                elif keyboard.is_pressed("a"):
                    l1_motor.setVelocity(max_speed / angular_speed_reduction)
                    r1_motor.setVelocity(-max_speed / angular_speed_reduction)
                elif keyboard.is_pressed("d"):
                    l1_motor.setVelocity(-max_speed / angular_speed_reduction)
                    r1_motor.setVelocity(max_speed / angular_speed_reduction)
                elif keyboard.is_pressed("q"):
                    break
                else:
                    l1_motor.setVelocity(0.0)
                    r1_motor.setVelocity(0.0)
            except Exception:
                # traceback.print_exc()
                pass

            time.sleep(1 / 30)

    return command_thread


def run_robot(robot: Robot) -> None:
    """Controls the robot's movement based on keyboard input.
    :param robot: The robot instance.
    :type robot: Robot
    """
    # get the time step of the current world.
    time_step = 32
    max_speed = 6.28

    # Motors
    l1_motor = robot.getDevice("motor_l1")
    r1_motor = robot.getDevice("motor_r1")

    l1_motor.setPosition(float("inf"))
    r1_motor.setPosition(float("inf"))

    l1_motor.setVelocity(0.0)
    r1_motor.setVelocity(0.0)

    # Sensors
    lidar = robot.getDevice("lidar_sensor")
    camera = robot.getDevice("camera")
    imu = robot.getDevice("imu")
    gps = robot.getDevice("gps")

    camera.enable(time_step)
    imu.enable(time_step)
    gps.enable(time_step)
    lidar.enable(time_step)
    lidar.enablePointCloud()

    # Keyboard Control

    command_thread = command_thread_factory(l1_motor, r1_motor, max_speed)
    thread = threading.Thread(target=command_thread)
    thread.start()

    while robot.step(time_step) != -1:
        robot_orientation = imu.getRollPitchYaw()
        robot_position = gps.getValues()
        image = parse_image(camera)
        ranges = parse_lidar(lidar)

        print(
            f"Orientation: {robot_orientation} | "
            + f"Position: {robot_position} | "
            + f"Image: {image.shape} | "
            + f"Ranges: {ranges.shape}"
        )


class ROS2Subscriber(Node):
    def __init__(self):
        super().__init__("ros2_subscriber")
        self.subscription = self.create_subscription(
            String, "webots", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


class ROS2Publisher(Node):
    def __init__(self):
        super().__init__("ros2_publisher")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello Webots"

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


if __name__ == "__main__":
    robot = Robot()
    run_robot(robot)

    # Uncomment the following code to run ROS2 nodes in parallel with the Webots controller.
    # rclpy.init(args=None)

    # ros2_publisher = ROS2Publisher()
    # ros2_subscriber = ROS2Subscriber()

    # try:
    #     rclpy.spin(ros2_publisher)
    # except KeyboardInterrupt:
    #     ros2_publisher.destroy_node()
    #     rclpy.shutdown()
