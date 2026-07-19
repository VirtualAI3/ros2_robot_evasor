from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_evasor_gazebo'), 'launch', 'gazebo.launch.py'
                ])
            ]),
        ),
        Node(
            package='robot_evasor_control',
            executable='vfh_controller.py',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('robot_evasor_control'),
                    'config', 'vfh_params.yaml'
                ]),
                {'use_sim_time': True},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('robot_evasor_description'), 'config', 'rviz_config.rviz'
            ])],
        ),
    ])
