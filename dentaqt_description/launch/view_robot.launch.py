from launch import LaunchDescription
from launch_ros.actions import Node

from lbr_description import RVizMixin
from dentaqt_description import SimRobotDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(SimRobotDescriptionMixin.arg_robot_name())
    robot_description = SimRobotDescriptionMixin.param_robot_description()
    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        )
    )
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )
    )
    ld.add_action(
        RVizMixin.node_rviz(
            rviz_config_pkg="lbr_description",
            rviz_config="config/config.rviz",
        )
    )
    return ld
