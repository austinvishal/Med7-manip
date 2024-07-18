// dentaqt Headers
#include <dentaqt_controllers/lbr_gravity_compensation_controller.hpp>

// cpp headers
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>

// pinocchio headers
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace dt {

namespace {
constexpr int c0i{0};
}

/////////////////////////////////////////////////////////////////
auto LBRGravityCompensationController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
  // claim the reference interface exported by lbr_torque_forwarder
  // and add it as command interface to this controller
  controller_interface::InterfaceConfiguration iface_cfg{};
  iface_cfg.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  std::for_each(m_joint_names.begin(), m_joint_names.end(),
                [&iface_cfg, this](const auto &joint_name) {
                  iface_cfg.names.push_back(this->m_chained_controller_prefix +
                                            "/" + joint_name + "/" +
                                            hardware_interface::HW_IF_EFFORT);
                });

  return iface_cfg;
}

/////////////////////////////////////////////////////////////////
auto LBRGravityCompensationController::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
  // get the joint positions through state interface
  controller_interface::InterfaceConfiguration iface_cfg{};
  iface_cfg.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  std::for_each(m_joint_names.begin(), m_joint_names.end(),
                [&iface_cfg](const auto &joint_name) {
                  iface_cfg.names.push_back(joint_name + "/" +
                                            hardware_interface::HW_IF_POSITION);
                });
  return iface_cfg;
}

/////////////////////////////////////////////////////////////////
auto LBRGravityCompensationController::on_init() -> CallbackReturn {
  // init parameter handler

  m_param_handler =
      std::make_shared<lbr_gravity_compensation_controller::ParamListener>(
          this->get_node());
  if (!m_param_handler) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "[LBRGravityCompensationController::on_init] "
                 "Unable to initialize parameter handler. ");
    return CallbackReturn::ERROR;
  }

  const auto params = m_param_handler->get_params();
  if (!m_urdf_param_client.initialize<rclcpp_lifecycle::LifecycleNode>(
          this->get_node(), params.robot_state_publisher_node_name,
          params.robot_description_parameter_name)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "[LBRGravityCompensationController::on_init] "
                 "Unable to initialize urdf parameter client to get "
                 "robot_description. ");
    return CallbackReturn::ERROR;
  }

  // initialize with nan so unwanted values are not sent to the robot
  m_joint_pos.setZero();
  m_gravity_trqs.setZero();
  return CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////
auto LBRGravityCompensationController::on_configure(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    -> CallbackReturn {
  // configure pinocchio model and data
  // build the model from robot description obtained as urdf xml

  const auto &robot_description = m_urdf_param_client.get_urdf_string();
  pinocchio::urdf::buildModelFromXML(robot_description, m_robot_model);

  // if the number of joints in the model is not equal to KUKA LBR joints
  // or if the requested frames do not exist in the model, then fail
  if (m_robot_model.nq != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "[LBRGravityCompensationController::on_configure] "
                 "Unable to load robot model.");
    return CallbackReturn::ERROR;
  }

  // create the cache of the rigid body dynamics algorithm
  m_rbd_data = pinocchio::Data(m_robot_model);

  return CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////
auto LBRGravityCompensationController::update(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period)
    -> controller_interface::return_type {

  // get joint position feedback
  get_joint_feedback();

  // compute generalized gravity torques
  m_gravity_trqs = pinocchio::computeGeneralizedGravity(
      m_robot_model, m_rbd_data, m_joint_pos);

  // set joint torque command
  set_joint_cmd();

  return controller_interface::return_type::OK;
}

/////////////////////////////////////////////////////////////////
auto LBRGravityCompensationController::get_joint_feedback() -> void {
  for (int idx = c0i; idx < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++idx) {
    const auto &joint_name{m_joint_names[idx]};
    auto position_state = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&joint_name](
            const hardware_interface::LoanedStateInterface &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() ==
                     hardware_interface::HW_IF_POSITION;
        });
    m_joint_pos(idx) = position_state->get_value();
  }
}

/////////////////////////////////////////////////////////////////
auto LBRGravityCompensationController::set_joint_cmd() -> void {
  for (int idx = c0i; idx < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++idx) {
    const auto &joint_name{m_joint_names[idx]};
    auto effort_cmd = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name,
         this](const hardware_interface::LoanedCommandInterface &interface) {
          return interface.get_prefix_name() ==
                     this->m_chained_controller_prefix &&
                 interface.get_interface_name() ==
                     joint_name + "/" + hardware_interface::HW_IF_EFFORT;
        });

    effort_cmd->set_value(m_gravity_trqs[idx]);
  }
}

} // namespace dt

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(dt::LBRGravityCompensationController,
                            controller_interface::ControllerInterface)
