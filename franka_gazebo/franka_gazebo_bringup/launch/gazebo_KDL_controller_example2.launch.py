import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,      # <-- make sure this is here
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(
    context: LaunchContext,
    arm_id,
    load_gripper,
    franka_hand,
    base_xyz,
    base_rpy,
    ee_xyz,
    ee_rpy
):
    # Perform substitutions
    arm_id_str       = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str  = context.perform_substitution(franka_hand)
    base_xyz_str     = context.perform_substitution(base_xyz)
    base_rpy_str     = context.perform_substitution(base_rpy)
    ee_xyz_str       = context.perform_substitution(ee_xyz)
    ee_rpy_str       = context.perform_substitution(ee_rpy)

    # Path to Xacro
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    # Process Xacro with mappings for initial pose
    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id':       arm_id_str,
            'hand':         load_gripper_str,
            'ros2_control': 'true',
            'gazebo':       'true',
            'ee_id':        franka_hand_str,
            'xyz':          base_xyz_str,
            'rpy':          base_rpy_str,
            'xyz_ee':       ee_xyz_str,
            'rpy_ee':       ee_rpy_str,
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[robot_description]
        )
    ]


def generate_launch_description():
    # Launch arguments
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper', default_value='false',
        description='true/false for activating the gripper'
    )
    franka_hand_arg = DeclareLaunchArgument(
        'franka_hand', default_value='franka_hand',
        description='Default value: franka_hand'
    )
    arm_id_arg = DeclareLaunchArgument(
        'arm_id', default_value='fr3',
        description='Available values: fr3, fp3, fer'
    )
    base_xyz_arg = DeclareLaunchArgument(
        'base_xyz', default_value='0 0 0',
        description='Initial base position (x y z) in meters'
    )
    base_rpy_arg = DeclareLaunchArgument(
        'base_rpy', default_value='0 0 0',
        description='Initial base orientation (roll pitch yaw) in radians'
    )
    ee_xyz_arg = DeclareLaunchArgument(
        'ee_xyz', default_value='0 0 0',
        description='Initial end-effector offset position (x y z)'
    )
    ee_rpy_arg = DeclareLaunchArgument(
        'ee_rpy', default_value='0 0 0',
        description='Initial end-effector offset orientation (r p y)'
    )

    # LaunchConfigurations
    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand  = LaunchConfiguration('franka_hand')
    arm_id       = LaunchConfiguration('arm_id')
    base_xyz     = LaunchConfiguration('base_xyz')
    base_rpy     = LaunchConfiguration('base_rpy')
    ee_xyz       = LaunchConfiguration('ee_xyz')
    ee_rpy       = LaunchConfiguration('ee_rpy')

    # Robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand,
              base_xyz, base_rpy, ee_xyz, ee_rpy]
    )

    # Gazebo simulation launch
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description')
    )
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/robot_description'], output='screen'
    )

    # RViz visualization
    rviz_config = os.path.join(
        get_package_share_directory('franka_description'),
        'rviz', 'visualize_franka.rviz'
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['--display-config', rviz_config, '-f', 'world']
    )

    # Controllers
    load_jst = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    force_pd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'force_pd_controller'],
        output='screen'
    )

    return LaunchDescription([
        load_gripper_arg,
        franka_hand_arg,
        arm_id_arg,
        base_xyz_arg,
        base_rpy_arg,
        ee_xyz_arg,
        ee_rpy_arg,
        gazebo_launch,
        robot_state_publisher,
        rviz,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn, on_exit=[load_jst])
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=load_jst, on_exit=[force_pd])
        ),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher',
            parameters=[{'source_list': ['joint_states'], 'rate': 30}]
        )
    ])
