from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Perception + Control 통합 Launch 파일."""

    pkg_dir = get_package_share_directory('vteleop')

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'perception.launch.py')
        )
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'output_topic': 'set_angle_data',
        }.items()
    )

    rqt_image_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/hand/viz_image'],
        output='screen'
    )

    return LaunchDescription([
        perception_launch,
        control_launch,
        rqt_image_view,
    ])
