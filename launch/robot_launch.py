import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
def generate_launch_description():
    
    package_dir = get_package_share_directory('bot_controller')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    use_rviz = LaunchConfiguration('rviz', default=False)
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'spot2.wbt')
    )

    
    my_robot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    tool_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'robot_tools_launch.py')
        ),
        launch_arguments={
            'rviz': use_rviz
        }.items(),
    )
    waiting_nodes = WaitForControllerConnection(
        target_driver=my_robot_driver,
        nodes_to_start=tool_nodes
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        footprint_publisher,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])