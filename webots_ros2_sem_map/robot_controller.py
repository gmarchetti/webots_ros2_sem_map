"""main_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from typing import Callable

import numpy as np
import traceback
import os
from tf2_ros import TransformBroadcaster, TransformStamped
import rclpy
from math import sin, cos, pi

import rclpy.time
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PointStamped
from controller import Robot
from controller.motor import Motor
from controller.camera import Camera
from controller.lidar import Lidar

from PIL import Image

from webots_ros2_sem_map.img_object_detection_node import ImgDetection

import logging
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
        image_array = np.frombuffer(image, dtype=np.uint16)
        img_detector.parse_img_from_array(image_array.reshape((height, width, 4)))

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

def swap_cols(arr, frm, to):
    arr[:,[frm, to]] = arr[:,[to, frm]]

class RobotController:
        
    def __gps_to_odom(self, message : PointStamped):
        # self.__logger.debug(message)
        time_stamp = message.header.stamp

        gps_point = message.point

        tf_odom = TransformStamped()

        tf_odom.header.stamp = time_stamp
        tf_odom.header.frame_id = "odom"
        tf_odom.child_frame_id = "base_link"

        tf_odom.transform.translation.x = gps_point.x
        tf_odom.transform.translation.y = gps_point.y
        tf_odom.transform.translation.z = gps_point.z

        self.__odom_tf_broadcaster.sendTransform(tf_odom)

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = time_stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # set the position
        pose = Pose()
        pose.position = gps_point
        odom.pose.pose = pose

        # publish the message
        self.__odom_pub.publish(odom)

    def __save_img_callback(self, message):
        try:
            cam_image = self.__camera.getImage()
            width = self.__camera.getWidth()
            height = self.__camera.getHeight()

            np_image_array = np.frombuffer(cam_image, dtype=np.uint8)
            bgr_array = np_image_array.reshape((height, width, 4))[:, :, :3]
            rgb_array = bgr_array[:,:,::-1]                        
            img = Image.fromarray(rgb_array)
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
        
        self.__logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.DEBUG)
        
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
        self.__node.create_subscription(PointStamped, 'robot/gps', self.__gps_to_odom, 1)

        # self.__iteration_reset_publisher = self.__node.create_publisher(Bool, "iteration_reset", 1)
        self.__odom_pub = self.__node.create_publisher(Odometry, "robot/odom", 1)
        self.__odom_tf_broadcaster = TransformBroadcaster(self.__node)
     
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
