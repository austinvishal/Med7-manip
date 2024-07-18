#ifndef DENTAQT_CONTROLLERS_LBR_CHAINED_TORQUE_FORWARDER_HPP
#define DENTAQT_CONTROLLERS_LBR_CHAINED_TORQUE_FORWARDER_HPP

#include "visibility_control.hpp"

// CPP Headers
#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ROS 2 Headers
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// ROS 2 Control Headers
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <realtime_tools/realtime_buffer.h>

// LBR FRI Headers
#include "friLBRState.h"

#include <lbr_fri_msgs/msg/lbr_torque_command.hpp>

namespace dt {

class LBRChainedTorqueForwarder
    : public controller_interface::ChainableControllerInterface {
public:
  using InterfaceRefs = std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
  using TorqueCmd = lbr_fri_msgs::msg::LBRTorqueCommand;
  using TorqueCmdPtr = TorqueCmd::SharedPtr;

  DENTAQT_CONTROLLERS_PUBLIC
  LBRChainedTorqueForwarder();

  /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////
  //////// Override methods for Lifecycle node interface //////////
  /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////
  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /// @brief Callback for the transition state `Configuring`
  /// This method is called once in the node's life time
  /// in order to load its configuration and setting up
  /// resources that will be held throughout it's life time.
  /// Used mainly for initialization and configuration.
  /// @param prevState previous lifecycle state
  /// @return TRANSITION_CALLBACK_SUCCESS transitions to Inactive
  ///         TRANSITION_CALLBACK_FAILURE transitions to Unconfigured
  ///         TRANSITION_CALLBACK_ERROR transitions to ErrorProcessing
  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /// @brief Callback for the transition state `Activating`
  /// This method is called does final preparations before
  /// beginnning to execute the actual node behavior.
  /// Might include resource acquistion that is
  /// possible only when node is active
  /// should obey real-time constraints
  /// @param prevState previous lifecycle state
  /// @return TRANSITION_CALLBACK_SUCCESS transitions to Active
  ///         TRANSITION_CALLBACK_FAILURE transitions to InActive
  ///         TRANSITION_CALLBACK_ERROR transitions to ErrorProcessing
  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /// @brief Callback for the transition state `Deactivating`
  /// This method is called to reverse the changes caused by `on_activate`.
  /// should obey real-time constraints
  /// @param prevState previous lifecycle state
  /// @return TRANSITION_CALLBACK_SUCCESS transitions to Active
  ///         TRANSITION_CALLBACK_FAILURE transitions to InActive
  ///         TRANSITION_CALLBACK_ERROR transitions to ErrorProcessing
  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /// @brief Callback for the transition state `CleaningUp`
  /// This method is called to clear all state and return the node
  /// to a functionally equivalent state as it was first created
  /// @param prevState previous lifecycle state
  /// @return TRANSITION_CALLBACK_SUCCESS transitions to UnConfigured
  ///         TRANSITION_CALLBACK_FAILURE transitions to InActive
  ///         TRANSITION_CALLBACK_ERROR transitions to ErrorProcessing
  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////
  ////////// Override methods for controller interface ////////////
  /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////

  /// @brief Get the configuration for controller's
  /// required command interfaces
  /// @return configuration of command interfaces
  /// which is used to check if controller can be
  /// activated and claim interfaces from hardware
  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  /// @brief Get the configuration for controller's
  /// required state interfaces
  /// @return configuration of state interfaces
  /// which is used to check if controller can be
  /// acivated and claim interfaces from hardware
  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  /// @brief Perform controller computations and update command interfaces
  /// method called after update_reference_from_subscribers()
  /// when not in chained mode
  /// @param time current time
  /// @param period time step
  /// @return OK if control step update was successful, ERROR if not
  DENTAQT_CONTROLLERS_PUBLIC
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

protected:
  /// @brief Input interfaces exported by the chainable controller
  /// @return vector of command interface handles
  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() override;

  /// @brief Update reference for input topics from subscribers
  ///  when not in chained mode
  /// @return OK if update is successful, ERROR if not
  controller_interface::return_type
  update_reference_from_subscribers() override;

  /// @brief Updates for switching to chained mode from external mode
  /// note that in chained mode, all external interfaces
  /// such as subscribers and service servers are disabled to
  /// avoid potential concurrency issues in input commands
  /// @param chainedMode
  /// @return true is switch to chained mode was successful
  bool on_set_chained_mode(bool chainedMode) override;

  auto reference_command_interfaces() -> bool;
  auto clear_command_interfaces() -> void;

  auto configure_subscribers() -> void;

  // List of KUKA LBR Joints
  std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS> m_joint_names{
      "A1", "A2", "A3", "A4", "A5", "A6", "A7"};
  InterfaceRefs m_torque_cmd_interfaces{};

  realtime_tools::RealtimeBuffer<TorqueCmdPtr> m_rt_lbr_trq_cmd_ptr{nullptr};
  std::shared_ptr<TorqueCmd> m_trq_cmd_msg{nullptr};
  rclcpp::Subscription<TorqueCmd>::SharedPtr m_lbr_trq_cmd_subscription_ptr{
      nullptr};
  // turn this into a parameter in the future
  std::string m_trqcmd_subscription_topic{"command/torque"};
};

} // namespace dt

#endif // DENTAQT_CONTROLLERS_LBR_CHAINED_TORQUE_FORWARDER_HPP
