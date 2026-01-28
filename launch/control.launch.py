from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch 파일 생성."""

    # Launch Arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/hand/joint_angles',
        description='Input joint angles topic from perception node'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='set_angle_data',
        description='Output topic for Inspire Hand control'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Control command publish rate (Hz)'
    )

    smoothing_factor_arg = DeclareLaunchArgument(
        'smoothing_factor',
        default_value='0.3',
        description='EMA smoothing factor (0.0-1.0, higher = less smoothing)'
    )

    enable_thumb_abd_arg = DeclareLaunchArgument(
        'enable_thumb_abd',
        default_value='true',
        description='Enable thumb abduction control'
    )

    angle_min_arg = DeclareLaunchArgument(
        'angle_min',
        default_value='0',
        description='Minimum angle value for Inspire Hand (0-1000)'
    )

    angle_max_arg = DeclareLaunchArgument(
        'angle_max',
        default_value='1000',
        description='Maximum angle value for Inspire Hand (0-1000)'
    )

    target_hand_arg = DeclareLaunchArgument(
        'target_hand',
        default_value='left',
        description='Target hand to control: left, right, or any'
    )

    # Node
    control_node = Node(
        package='vteleop',
        executable='hand_control_node',
        name='inspire_hand_control_node',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'smoothing_factor': LaunchConfiguration('smoothing_factor'),
            'enable_thumb_abd': LaunchConfiguration('enable_thumb_abd'),
            'angle_min': LaunchConfiguration('angle_min'),
            'angle_max': LaunchConfiguration('angle_max'),
            'target_hand': LaunchConfiguration('target_hand'),
        }],
        output='screen'
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        publish_rate_arg,
        smoothing_factor_arg,
        enable_thumb_abd_arg,
        angle_min_arg,
        angle_max_arg,
        target_hand_arg,
        control_node,
    ])
