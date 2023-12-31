import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory('bot_controller')

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_rviz = LaunchConfiguration('rviz', default=True)

    # Rviz node
    rviz_config = os.path.join(package_dir, 'resource', 'all.rviz')
    launch_description_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=['--display-config=' + rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=launch.conditions.IfCondition(use_rviz)
        )
    )


    # Launch descriptor
    return LaunchDescription(launch_description_nodes)
