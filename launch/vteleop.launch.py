from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    return LaunchDescription([
        perception_launch,
        control_launch,
    ])
