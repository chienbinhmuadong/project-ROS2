import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

venv_python = os.path.expanduser("~/workspace/rosvenv/bin/python3")


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_lab = get_package_share_directory('ros_project_scene')
    xacro_file = os.path.join(pkg_lab, 'urdf', 'robot_model.xacro')

    world_path = PathJoinSubstitution([
        FindPackageShare('ros_project_scene'),
        'worlds','simple.world'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [world_path, ' -r'],
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])},
        ]
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       parameters=[{'use_sim_time': True}],
       arguments=['-d', os.path.join(pkg_lab, 'config', 'rviz.rviz')],
    
    )

    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=[ '-name', 'diff_drive', '-topic', 'robot_description', '-x', '1', '-y', '1', '-z', '0.5'], 
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_lab, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_controller_left_wheel_controller",
        arguments=["velocity_controller"],
        output="screen"
    )

    image_proc = Node(
        package='ros_project_scene',
        executable='sensor_data_proc',
        name='camera_sensor_handler',
        output='screen',
        prefix=[venv_python, " "],
    )
    
    move_straight = Node(
        package="ros_project_scene",
        executable="move_straight",
        name="move_straight",
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    control_robot = Node(
        package="ros_project_scene",
        executable="control_robot",
        name="control_robot",
        output="screen",
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument(
            'urdf_model',
            default_value=xacro_file,
            description='Full path to the Xacro file'
        ),
        bridge,
        spawn,
        robot_state_publisher,
        rviz,
        spawn_controller,
        # image_proc,
        # move_straight,
        control_robot,
    ])
