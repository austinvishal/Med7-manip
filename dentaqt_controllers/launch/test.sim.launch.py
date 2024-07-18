from typing import List

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node


from dentaqt_bringup import DentaqtSimBringUpMixin
from lbr_ros2_control import LBRROS2ControlMixin


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

    # publish to /lbr_chained_torque_forwarder/commands topic
    torque_sine_reference = Node(
        package="dentaqt_controllers",
        executable="lbr_torque_sine_reference",
        output="screen",
        namespace="lbr",
    )

    controller_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=controller,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[torque_sine_reference],
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
