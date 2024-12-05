import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_sem_map')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot.urdf')
    
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'lidar_world.wbt'),
        ros2_supervisor=False
    )

    robot_controller = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    image_recognition = Node(
        package='webots_ros2_sem_map',
        executable='img_recognition'
    )

    return LaunchDescription([
        webots,
        robot_controller,
        image_recognition,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])