from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from lbr_description import IgnitionGazeboMixin, RVizMixin
from dentaqt_description import SimRobotDescriptionMixin
from lbr_ros2_control import LBRROS2ControlMixin


def launch_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()
    robot_description = SimRobotDescriptionMixin.param_robot_description()
    ld.add_action(IgnitionGazeboMixin.include_gazebo())
    create = IgnitionGazeboMixin.node_create()
    ld.add_action(create)
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=True)
    ld.add_action(robot_state_publisher)
    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    ld.add_action(joint_state_broadcaster)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    ld.add_action(
        SimRobotDescriptionMixin.node_static_tf(
            tf=[0, 0, 0, 0, 0, 0],  # keep zero
            parent="world",
            child=PathJoinSubstitution(
                [
                    robot_name,
                    "world",
                ]  # results in robot_name/world
            ),
            parameters=[{"use_sim_time": True}],
        )
    )

    # RViz no MoveIt
    rviz = RVizMixin.node_rviz(
        rviz_config_pkg="lbr_bringup",
        rviz_config="config/config.rviz",
        condition=IfCondition(
            LaunchConfiguration("rviz")
        ),
    )

    # RViz event handler
    rviz_event_handler = RegisterEventHandler(
        OnProcessExit(target_action=create, on_exit=[rviz])
    )
    ld.add_action(rviz_event_handler)

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(SimRobotDescriptionMixin.arg_robot_name())
    ld.add_action(
        DeclareLaunchArgument(
            name="rviz", default_value="true", description="Whether to launch RViz."
        )
    )
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
