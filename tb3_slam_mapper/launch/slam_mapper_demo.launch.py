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
        default_value='true',
        description='Start RViz together with Gazebo and SLAM mapper.',
    )

    slam_mapper_share = get_package_share_directory('tb3_slam_mapper')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    rviz_config_file = slam_mapper_share + '/rviz/slam_map.rviz'

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            turtlebot3_gazebo_share + '/launch/turtlebot3_world.launch.py'
        )
    )

    slam_mapper_node = Node(
        package='tb3_slam_mapper',
        executable='slam_mapper',
        name='slam_mapper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    wall_follower_node = Node(
        package='tb3_slam_mapper',
        executable='wall_follower',
        name='wall_follower',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='slam_mapper_rviz',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        [
            use_rviz_argument,
            world_launch,
            slam_mapper_node,
            wall_follower_node,
            rviz_node,
        ]
    )
