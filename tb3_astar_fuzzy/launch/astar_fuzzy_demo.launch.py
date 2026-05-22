from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    use_rviz_argument = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz together with Gazebo, SLAM, fuzzy controller and A* planner.',
    )
    use_planner_argument = DeclareLaunchArgument(
        'use_planner',
        default_value='true',
        description='Start the A* path planner service and /global_plan publisher.',
    )

    package_share = get_package_share_directory('tb3_astar_fuzzy')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    rviz_config_file = package_share + '/rviz/astar_fuzzy.rviz'

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            turtlebot3_gazebo_share + '/launch/turtlebot3_world.launch.py'
        )
    )

    slam_mapper_node = Node(
        package='tb3_astar_fuzzy',
        executable='slam_mapper',
        name='slam_mapper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    fuzzy_controller_node = Node(
        package='tb3_astar_fuzzy',
        executable='fuzzy_controller',
        name='fuzzy_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    astar_planner_node = Node(
        package='tb3_astar_fuzzy',
        executable='astar_planner',
        name='astar_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_planner')),
    )
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='astar_fuzzy_rviz',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        [
            use_rviz_argument,
            use_planner_argument,
            SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
            world_launch,
            slam_mapper_node,
            fuzzy_controller_node,
            astar_planner_node,
            rviz_node,
        ]
    )
