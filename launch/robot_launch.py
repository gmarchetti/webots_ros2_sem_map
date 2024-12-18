import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_sem_map')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot_webots.urdf')
    
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'lidar_world.wbt'),
        ros2_supervisor=False
    )

    robot_controller = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        remappings=[
            ('/robot/lidar_sensor', '/scan')
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', robot_description_path])}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', robot_description_path])}] # add other parameters here if required
    )

    image_recognition = Node(
        package='webots_ros2_sem_map',
        executable='img_recognition'
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(package_dir, 'resource/ekf.yaml'), {'use_sim_time': True}]
)

    return LaunchDescription([
        webots,
        robot_controller,
        image_recognition,
        robot_state_publisher,
        # robot_localization_node,
        joint_state_publisher_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])