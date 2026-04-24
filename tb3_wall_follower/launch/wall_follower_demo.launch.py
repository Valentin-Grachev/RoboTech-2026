from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    use_rviz_argument = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz together with the Gazebo demo.',
    )

    tb3_wall_follower_share = get_package_share_directory('tb3_wall_follower')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    rviz_config_file = tb3_wall_follower_share + '/rviz/odometry_only.rviz'
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            turtlebot3_gazebo_share + '/launch/turtlebot3_world.launch.py'
        )
    )

    wall_follower_node = Node(
        package='tb3_wall_follower',
        executable='wall_follower',
        name='wall_follower',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='wall_follower_rviz',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        [
            use_rviz_argument,
            world_launch,
            wall_follower_node,
            rviz_node,
        ]
    )
