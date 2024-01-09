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
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes

def generate_launch_description():
    
    package_dir = get_package_share_directory('bot_controller')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_rviz = LaunchConfiguration('rviz', default=False)
    use_slam = LaunchConfiguration('slam', default=False)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'spot2.wbt')
    )

    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')

    
    my_robot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params,
            
        ],
        remappings=mappings,
        respawn=True
    )
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    navigation_nodes = []
    if 'turtlebot3_cartographer' in get_packages_with_prefixes():
        turtlebot_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_slam))
        navigation_nodes.append(turtlebot_slam)

    tool_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'robot_tools_launch.py')
        ),
        launch_arguments={
            'rviz': use_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=my_robot_driver,
        nodes_to_start=navigation_nodes
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