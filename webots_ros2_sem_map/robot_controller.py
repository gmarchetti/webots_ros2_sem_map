"""main_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from typing import Callable

import numpy as np
import traceback
import os
import rclpy
import rclpy.time

from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos, pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PointStamped, Twist
from controller import Robot, Display
from controller.motor import Motor
from controller.camera import Camera
from controller.lidar import Lidar

from .map_info import MapInfo
from .camera_img_handler import CameraImgHandler

import logging
logging.basicConfig(level=logging.INFO)

HALF_DISTANCE_BETWEEN_WHEELS = 0.45
WHEEL_RADIUS = 0.25
OBJ_MAP_SIZE = 270

def prob_to_color(prob: int):
    #RGB
    if prob == 0:
        # return 0xACB5AE 
        return [172, 181, 174]
    elif prob == 100:
        # return 0x000000
        return [0, 0, 0]
    elif prob == -1:
        # return 0x838A84
        return [131, 138, 132]
    else:
        # return 0xED671F 
        return [237, 103, 31]

def pol2cart(rho, phi, current_position=[0,0,0], current_orientation=[0,0,0]):

    x = (rho * np.cos(phi + current_orientation[2])) + current_position[0]
    y = (rho * np.sin(phi + current_orientation[2])) + current_position[1]
    return(x, y)

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
        
        imu_reading = self.__imu.getQuaternion()
        tf_odom.transform.rotation.x = imu_reading[0]
        tf_odom.transform.rotation.y = imu_reading[1]
        tf_odom.transform.rotation.z = imu_reading[2]
        tf_odom.transform.rotation.w = imu_reading[3]

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

    def __cam_angle_to_index(self, angle, num_points):

        a = num_points/pi
        b = 360

        index = int((a*angle) + b)
        index = max(index, 0)
        index = min(index, num_points-1)
        return index

    def __parse_current_camera(self, message):
        try:
            items_in_sight = self.__img_handler.parse_current_camera()
            self.__logger.info(f"found these items in camera {items_in_sight}")

            range_data = self.__lidar.getRangeImage()
            num_range_points = len(range_data)
            
            self.__logger.info(f"Lidar has {num_range_points} number of points")

            angle_between_points = pi/num_range_points

            current_orientation = self.__imu.getRollPitchYaw()
            current_pose = self.__gps.getValues()

            self.__logger.info(f"Current orientation: {current_orientation}")
            self.__logger.info(f"Current Position: {current_pose}")
            
            for item in items_in_sight:
                min_angle, max_angle = item["angle_pos"][0], item["angle_pos"][1]
                min_index = self.__cam_angle_to_index(min_angle, num_range_points)
                max_index = self.__cam_angle_to_index(max_angle, num_range_points)

                self.__logger.info(f"Range data for {item["label"]} is between indexes: {min_index} {max_index}")

                item_range_data = range_data[min_index:max_index]
                angle_point = min_angle                                
                
                for range_point in item_range_data:
                    item_x, item_y = pol2cart(range_point, angle_point, current_pose, current_orientation)
                    
                    angle_point += angle_between_points
                    
                    occupied_prob = self.__map_info.get_prob_is_xy_occupied(item_x, item_y)
                    
                    if occupied_prob > 0:
                        self.__logger.debug(f"Grid has a {occupied_prob}, marking as {item["label"]}")
                        # self.__obj_map[grid_coord[0]][grid_coord[1]] = 1
            
        except AttributeError:
            self.__logger.error("An error occurred due to missing methods in the camera object.")
            traceback.print_exc()
        except ValueError:
            self.__logger.error("An error occurred while reshaping the image data.")
            traceback.print_exc()

    def __save_img_callback(self, message):
        try:
            self.__img_handler.save_camera_to_file()
        except AttributeError:
            self.__logger.error("An error occurred due to missing methods in the camera object.")
            traceback.print_exc()
        except ValueError:
            self.__logger.error("An error occurred while reshaping the image data.")
            traceback.print_exc()
    
    def __read_map_message(self, message):

        m_info = message.info
        m_height = m_info.height
        m_width = m_info.width

        m_data = message.data
        
        m_color_matrix = np.array([prob_to_color(point) for point in m_data], dtype=np.uint8)
        m_color_array = np.reshape(m_color_matrix, m_height*m_width*3)

        self.__map_info.process_new_map_message(message)

        self.__logger.debug(f"Received map data: {m_height}h x {m_width}w {len(m_data)} items")
        ir = self.__display.imageNew(bytes(m_color_array), Display.RGB, m_width, m_height)
        self.__display.imagePaste(ir, 0, 0, False)
        self.__display.imageDelete(ir)

        obj_color_array = self.__map_info.get_obj_map_for_display()
        ir = self.__obj_display.imageNew(bytes(obj_color_array), Display.RGB, m_width, m_height)
        self.__obj_display.imagePaste(ir, 0, 0, False)
        self.__obj_display.imageDelete(ir)

    def init(self, webots_node, properties):
        rclpy.init(args=None)
        
        self.__logger = logging.getLogger(__name__)

        self.__target_twist = Twist()

        self.__node = rclpy.create_node('robot')
        self.__robot = webots_node.robot

        # get the time step of the current world.
        time_step = 32

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
        self.__display = self.__robot.getDevice("display")
        self.__obj_display = self.__robot.getDevice("obj_display")

        self.__camera.enable(time_step)
        self.__imu.enable(time_step)
        self.__gps.enable(time_step)
        self.__lidar.enable(time_step)
        # self.__lidar.enablePointCloud()

        self.__img_handler = CameraImgHandler(self.__camera)

        # Ros2 TeleOp Control
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Bool, 'save_img', self.__save_img_callback, 1)
        self.__node.create_subscription(Bool, 'parse_camera', self.__parse_current_camera, 1)
        self.__node.create_subscription(PointStamped, 'robot/gps', self.__gps_to_odom, 1)
        self.__node.create_subscription(OccupancyGrid, '/map', self.__read_map_message, 1)

        self.__odom_pub = self.__node.create_publisher(Odometry, "robot/odom", 1)
        self.__imu_pub = self.__node.create_publisher(Imu, "robot/imu", 1)
        self.__odom_tf_broadcaster = TransformBroadcaster(self.__node)

        self.__map_info = MapInfo()

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

