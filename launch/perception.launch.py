from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch 파일 생성."""

    # Launch Arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='Input image topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/hand/joint_angles',
        description='Output joint angles topic'
    )

    detection_confidence_arg = DeclareLaunchArgument(
        'detection_confidence',
        default_value='0.5',
        description='Minimum detection confidence'
    )

    tracking_confidence_arg = DeclareLaunchArgument(
        'tracking_confidence',
        default_value='0.5',
        description='Minimum tracking confidence'
    )

    max_hands_arg = DeclareLaunchArgument(
        'max_hands',
        default_value='1',
        description='Maximum number of hands to detect'
    )

    viz_topic_arg = DeclareLaunchArgument(
        'viz_topic',
        default_value='/hand/viz_image',
        description='Visualization image topic'
    )

    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz',
        default_value='true',
        description='Enable visualization'
    )

    # Node
    hand_perception_node = Node(
        package='vteleop',
        executable='hand_perception_node',
        name='hand_perception_node',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'viz_topic': LaunchConfiguration('viz_topic'),
            'detection_confidence': LaunchConfiguration('detection_confidence'),
            'tracking_confidence': LaunchConfiguration('tracking_confidence'),
            'max_hands': LaunchConfiguration('max_hands'),
            'enable_viz': LaunchConfiguration('enable_viz'),
        }],
        output='screen'
    )

    return LaunchDescription([
        image_topic_arg,
        output_topic_arg,
        viz_topic_arg,
        detection_confidence_arg,
        tracking_confidence_arg,
        max_hands_arg,
        enable_viz_arg,
        hand_perception_node,
    ])
