import tempfile
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import subprocess


def evaluate_xacro(context):
    xacro_path = PathJoinSubstitution([
        FindPackageShare('robot_evasor_description'), 'urdf', 'evasor_bot.urdf.xacro'
    ]).perform(context)
    xacro_cmd = [FindExecutable(name='xacro').perform(context), xacro_path]
    result = subprocess.check_output(xacro_cmd).decode('utf-8')
    return result


def launch_setup(context, *args, **kwargs):
    robot_description = evaluate_xacro(context)

    world_file = PathJoinSubstitution([
        FindPackageShare('robot_evasor_gazebo'), 'worlds', 'room.world'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        remappings=[
            ('/joint_states', '/world/room_world/model/evasor_bot/joint_state'),
        ],
    )

    tf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    tf.write(robot_description)
    tf.close()
    model_path = tf.name

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', model_path,
            '-name', 'evasor_bot',
            '-allow_renaming', 'true',
            '-x', '1.0',
            '-y', '1.0',
            '-z', '0.1',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/evasor_bot/joint/rl_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/evasor_bot/joint/rr_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/evasor_bot/joint/fl_steering_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/evasor_bot/joint/fr_steering_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/world/room_world/model/evasor_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
    )

    wait = ExecuteProcess(
        cmd=['sleep', '3'],
        output='log',
    )

    return [
        gz_sim,
        robot_state_publisher,
        bridge,
        wait,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait,
                on_exit=[spawn_robot],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
