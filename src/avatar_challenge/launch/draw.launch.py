from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz alongside MoveIt (set to false for headless environments).',
    ))

    # xarm_moveit_config's launch chain reads a 'show_rviz' launch configuration
    # to decide whether to start rviz2; bridge our 'use_rviz' arg to it.
    ld.add_action(SetLaunchConfiguration('show_rviz', LaunchConfiguration('use_rviz')))

    # include xarm moveit fake launch
    xarm_pkg = get_package_share_directory('xarm_moveit_config')
    xarm_launch = os.path.join(xarm_pkg, 'launch', 'xarm7_moveit_fake.launch.py')
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(xarm_launch)))

    # draw node action server
    draw_node = Node(
        package='avatar_challenge',
        executable='draw_node',
        name='draw_node',
        output='screen',
    )



    ld.add_action(draw_node)
    return ld


