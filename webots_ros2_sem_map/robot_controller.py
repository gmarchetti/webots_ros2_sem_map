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
from tf2_ros import TransformBroadcaster, TransformStamped
import rclpy
from math import sin, cos, pi

from rclpy.node import Node
import rclpy.time
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from controller import Robot
from controller.motor import Motor
from controller.camera import Camera
from controller.lidar import Lidar

from PIL import Image

from webots_ros2_sem_map.img_object_detection_node import ImgDetection
HALF_DISTANCE_BETWEEN_WHEELS = 0.45
WHEEL_RADIUS = 0.25

def parse_image(img_detector: ImgDetection, camera: Camera) -> np.ndarray:
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
        img_detector.parse_img_from_array(image_array.reshape((height, width, 4))[:, :, :3])

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

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class RobotController:
    def __publish_odom(self):
        current_time = self.__node.get_clock().now()

        # compute odometry in a typical way given the velocities of the robot
        # dt = (current_time - self.__last_time).to_sec()
        delta_x = 0.0
        delta_y = 0.0
        delta_th = 0.0

        x = delta_x
        y = delta_y
        th = delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = euler_to_quaternion(0, 0, 0)
        # odom_trans = TransformStamped()
        # odom_trans.header.frame_id = 'base_link'
        # odom_trans.child_frame_id = 'odom'
        # odom_trans.header.stamp = current_time.to_msg()
        # odom_trans.transform.translation.x = x
        # odom_trans.transform.translation.y = y
        # odom_trans.transform.translation.z = 0.0
        # odom_trans.transform.rotation = euler_to_quaternion(0, 0, 0)
        # # first, we'll publish the transform over tf
        # self.__odom_broadcaster.sendTransform(
        #     odom_trans
        # )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        # odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        pose = Pose()
        point = Point()
        point.x = point.y = point.z = 0.0
        pose.position = point
        pose.orientation = odom_quat
        odom.pose.pose = pose
    
        # odom.pose.

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist()

            # publish the message
        self.__odom_pub.publish(odom)
        self.__last_time = current_time

    def __save_img_callback(self, message):
        try:
            cam_image = self.__camera.getImage()
            width = self.__camera.getWidth()
            height = self.__camera.getHeight()
            image_array = np.frombuffer(cam_image, dtype=np.uint8)
            img = Image.fromarray(image_array.reshape((height, width, 4))[:, :, :3])
            img.save("screenshot.jpg")
            
            self.__node.get_logger().info(f"Image saved to {os.path.abspath(os.path.join(os.curdir, 'screenshot.jpg'))}")
        except AttributeError:
            print("An error occurred due to missing methods in the camera object.")
            traceback.print_exc()
        except ValueError:
            print("An error occurred while reshaping the image data.")
            traceback.print_exc()

    def init(self, webots_node, properties):
        rclpy.init(args=None)
        
        self.__target_twist = Twist()

        self.__node = rclpy.create_node('robot')
        self.__robot = webots_node.robot

        self.__last_time = self.__robot.getTime()
        # get the time step of the current world.
        time_step = 32
        max_speed = 6.28

        # Motors
        self.__l1_motor = self.__robot.getDevice("motor_l1")
        self.__r1_motor = self.__robot.getDevice("motor_r1")

        self.__l1_motor.setPosition(float("inf"))
        self.__r1_motor.setPosition(float("inf"))

        self.__l1_motor.setVelocity(0.0)
        self.__r1_motor.setVelocity(0.0)

        # Sensors
        self.__lidar = self.__robot.getDevice("lidar_sensor")
        self.__camera = self.__robot.getDevice("camera")
        self.__imu = self.__robot.getDevice("imu")
        self.__gps = self.__robot.getDevice("gps")

        self.__camera.enable(time_step)
        self.__imu.enable(time_step)
        self.__gps.enable(time_step)
        self.__lidar.enable(time_step)
        self.__lidar.enablePointCloud()

        # Ros2 TeleOp Control
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Bool, 'save_img', self.__save_img_callback, 1)

        # self.__iteration_reset_publisher = self.__node.create_publisher(Bool, "iteration_reset", 1)
        self.__odom_pub = self.__node.create_publisher(Odometry, "demo/odom", 1)
        self.__odom_broadcaster = TransformBroadcaster(self.__node)
        print(self.__odom_broadcaster)
     
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
    
    def adjust_target_speed(self):
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__l1_motor.setVelocity(command_motor_left)
        self.__r1_motor.setVelocity(command_motor_right)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.adjust_target_speed()
        self.__publish_odom()
        # robot_orientation = self.__imu.getRollPitchYaw()
        # robot_position = self.__gps.getValues()
        # image = parse_image(self.__imager, self.__camera)
        # ranges = parse_lidar(self.__lidar)

        # print(
        #     f"Orientation: {robot_orientation} | "
        #     + f"Position: {robot_position} | "
        #     + f"Image: {image.shape} | "
        #     + f"Ranges: {ranges.shape}"
        # )
