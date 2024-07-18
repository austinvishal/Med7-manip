
from typing import List
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from dentaqt_description import SimRobotDescriptionMixin
from lbr_ros2_control import LBRROS2ControlMixin
from dentaqt_bringup import DentaqtSimBringUpMixin


def launch_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    sim_fixture = DentaqtSimBringUpMixin.include_robot()
    ld.add_action(sim_fixture)

    # No need to launch the ros2 control node
    # separately since dentaqt_ign_ros2_control plugin
    # is automatically launched while loading the robot model

    # spawn the lbr torque forwarder controller
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller="lbr_chained_torque_forwarder"
    )

    ld.add_action(controller)
    gravity_compensation_params_file = os.path.join(
        get_package_share_directory("dentaqt_controllers"),
        "config",
        "lbr_gravity_compensation.yaml"
    )

    robot_name = LaunchConfiguration("robot_name")
    # directly spawn instead of using mixin method
    # since mixin method does not accept parameter argument
    gravity_compensator = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
                "lbr_gravity_compensation_controller",
                "--controller-manager",
                "controller_manager",
                "--param-file",
                gravity_compensation_params_file
        ],
        namespace=robot_name,
    )

    controller_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=controller,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[gravity_compensator],
                ),
            ]
        )
    )

    ld.add_action(controller_event_handler)

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
