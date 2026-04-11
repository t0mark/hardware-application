import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


robot_ip = LaunchConfiguration('robot_ip')
use_fake_hardware = LaunchConfiguration('use_fake_hardware')
fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
model_id = LaunchConfiguration('model_id')
cb_simulation = LaunchConfiguration('cb_simulation')


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('robot_ip', default_value='10.0.2.7'),
        DeclareLaunchArgument('model_id', default_value='rb5_850e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false'),
        DeclareLaunchArgument('cb_simulation', default_value='Real'),
        DeclareLaunchArgument('rviz_config', default_value='moveit.rviz'),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    mappings = {
        'robot_ip': robot_ip,
        'use_fake_hardware': use_fake_hardware,
        'fake_sensor_commands': fake_sensor_commands,
        'model_id': model_id,
        'cb_simulation': cb_simulation,
    }

    moveit_config = (
        MoveItConfigsBuilder('rbpodo')
        .robot_description(file_path='config/rbpodo.urdf.xacro', mappings=mappings)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner']
        )
        .to_moveit_configs()
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('rbpodo_moveit_config'), 'config',
         LaunchConfiguration('rviz_config')]
    )

    params_file = os.path.join(
        get_package_share_directory('p_zone'), 'config', 'params.yaml'
    )

    behavior_dir = os.path.join(
        get_package_share_directory('p_zone'), 'config', 'behavior'
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('rbpodo_bringup'), 'config', 'controllers.yaml'
    )

    nodes = [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='log',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'link0'],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[moveit_config.robot_description],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[moveit_config.to_dict()],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[moveit_config.robot_description, ros2_controllers_path],
            output='both',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager-timeout', '300',
                '--controller-manager', '/controller_manager',
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
        ),
        Node(
            package='p_zone',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='p_zone',
            executable='behavior_executor',
            name='behavior_executor',
            output='screen',
            parameters=[params_file, {'behavior_dir': behavior_dir}],
        ),
    ]

    return nodes
