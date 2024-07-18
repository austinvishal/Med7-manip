#ifndef DENTAQT_CONTROLLERS_LBR_GRAVITY_COMPENSATION_CONTROLLER_HPP
#define DENTAQT_CONTROLLERS_LBR_GRAVITY_COMPENSATION_CONTROLLER_HPP

#include "visibility_control.hpp"

// CPP Headers
#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ROS 2 Headers
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

// LBR FRI Headers
#include "friLBRState.h"

#include <dentaqt_controllers/urdf_param_client.hpp>

// include generated parameter header
#include <lbr_gravity_compensation_controller_parameters.hpp>

// pinocchio headers
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

// Eigen headers
#include <Eigen/Dense>

namespace dt {

class LBRGravityCompensationController
    : public controller_interface::ControllerInterface {
public:
  DENTAQT_CONTROLLERS_PUBLIC
  CallbackReturn on_init() override;

  DENTAQT_CONTROLLERS_PUBLIC
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  auto get_joint_feedback() -> void;
  auto set_joint_cmd() -> void;

  std::shared_ptr<lbr_gravity_compensation_controller::ParamListener>
      m_param_handler{nullptr};
  URDFParameterClient m_urdf_param_client{};

  // List of KUKA LBR Joints
  std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS> m_joint_names{
      "A1", "A2", "A3", "A4", "A5", "A6", "A7"};

  /// @brief Joint position received from state interface
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1> m_joint_pos{};
  /// @brief Gravity compensation torques to be sent to the command interface
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1>
      m_gravity_trqs{};

  std::string m_chained_controller_prefix{"lbr_chained_torque_forwarder"};

  /// @brief rigid body dynamics model
  pinocchio::Model m_robot_model{};

  /// @brief cache of rigid body dynamics algorithm
  pinocchio::Data m_rbd_data{};
};

} // namespace dt

#endif // DENTAQT_CONTROLLERS_LBR_GRAVITY_COMPENSATION_CONTROLLER_HPP
