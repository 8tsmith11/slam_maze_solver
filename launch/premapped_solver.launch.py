from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_path = os.path.join(os.path.dirname(__file__), 'maze_map.yaml')
    goal_path = os.path.join(os.path.dirname(__file__), 'goal.yaml')

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_components'), 'launch', 'nav2_localization_demo.launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'namespace': 'tb4'
        }.items()
    )

    solver = Node(
        package='slam_maze_solver',
        executable='premapped_nav_node',
        name='premapped_nav_node',
        namespace='tb4',
        parameters=[goal_path],
        output='screen',
    )

    return LaunchDescription([
        nav2_localization,
        solver,
    ])