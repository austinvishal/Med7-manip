from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


class DentaqtSimBringUpMixin:
    @staticmethod
    def include_robot(**kwargs) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("dentaqt_bringup"),
                        "launch",
                        "sim.launch.py"
                    ]
                )
            ),
            **kwargs,
        )
