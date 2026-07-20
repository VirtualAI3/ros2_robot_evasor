from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Gazebo + robot_state_publisher + bridge + spawn
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_evasor_gazebo'),
                    'launch', 'gazebo.launch.py'
                ])
            ]),
        ),
        # Nav2 + SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_evasor_control'),
                    'launch', 'nav2_launch.py'
                ])
            ]),
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('robot_evasor_description'),
                'config', 'rviz_config.rviz'
            ])],
        ),
    ])
