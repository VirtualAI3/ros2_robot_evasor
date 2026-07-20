import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def check_bt_xml(context):
    bt_path = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'
    )
    exists = os.path.exists(bt_path)
    print(f'[DEBUG] BT XML path: {bt_path}')
    print(f'[DEBUG] BT XML exists: {exists}')
    if not exists:
        print(f'[ERROR] BT XML NOT FOUND! Navigation will NOT work.')
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='true')
    diagnostics = LaunchConfiguration('diagnostics', default='false')

    pkg_control = FindPackageShare('robot_evasor_control')
    default_params = PathJoinSubstitution([pkg_control, 'config', 'nav2_params.yaml'])
    default_slam_params = PathJoinSubstitution([pkg_control, 'config', 'slam_params.yaml'])

    bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Run SLAM if true'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to Nav2 param file'),
        DeclareLaunchArgument(
            'diagnostics', default_value='false',
            description='Run diagnostics node if true'),

        OpaqueFunction(function=check_bt_xml),

        # --- Nav2 Servers ---
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_navi')]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': use_sim_time,
                         'default_bt_xml_filename': bt_xml}],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_navi'),
                        ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother',
                ]
            }]
        ),

        # --- Lifecycle Manager for SLAM (independiente para evitar deadlock con Nav2) ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='slam_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
            condition=IfCondition(slam),
        ),

        # --- SLAM Toolbox ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[default_slam_params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(slam),
        ),

        Node(
            package='robot_evasor_control',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='robot_evasor_control',
            executable='diagnostics_node.py',
            name='diagnostics_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(diagnostics),
        ),
    ])
