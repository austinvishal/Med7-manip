// CPP Headers
#include <exception>
#include <limits>

// ROS 2 Control Headers
#include <dentaqt_controllers/lbr_chained_torque_forwarder.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace dt {

namespace {
constexpr double c0d{0.0};
} // namespace

/////////////////////////////////////////////////////////////////
LBRChainedTorqueForwarder::LBRChainedTorqueForwarder()
    : controller_interface::ChainableControllerInterface(),
      m_rt_lbr_trq_cmd_ptr{nullptr}, m_trq_cmd_msg{nullptr},
      m_lbr_trq_cmd_subscription_ptr{nullptr} {}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
  controller_interface::InterfaceConfiguration iface_cfg{};
  iface_cfg.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  std::for_each(m_joint_names.begin(), m_joint_names.end(),
                [&iface_cfg](const auto &joint_name) {
                  iface_cfg.names.push_back(joint_name + "/" +
                                            hardware_interface::HW_IF_EFFORT);
                });

  return iface_cfg;
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {

  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::on_export_reference_interfaces()
    -> std::vector<hardware_interface::CommandInterface> {
  // export all input interfaces
  const auto nr_joints = m_joint_names.size();
  const auto nr_chainable_interfaces{nr_joints};

  std::vector<hardware_interface::CommandInterface> chainable_cmd_interfaces{};
  chainable_cmd_interfaces.reserve(nr_chainable_interfaces);
  reference_interfaces_.resize(nr_chainable_interfaces, c0d);

  std::string controller_prefix{this->get_node()->get_name()};
  for (std::size_t idx = 0; idx < nr_joints; ++idx) {
    auto interface_name =
        m_joint_names[idx] + "/" + hardware_interface::HW_IF_EFFORT;
    chainable_cmd_interfaces.emplace_back(controller_prefix, interface_name,
                                          &reference_interfaces_[idx]);
  }

  return chainable_cmd_interfaces;
}

//////////////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::on_set_chained_mode(bool chainedMode) -> bool {
  return true || chainedMode;
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::configure_subscribers() -> void {
  // create a callback lambda for writing incoming msg
  // into the torque command pointer
  auto joint_cmd_callback =
      [this](const lbr_fri_msgs::msg::LBRTorqueCommand::SharedPtr msg) {
        m_rt_lbr_trq_cmd_ptr.writeFromNonRT(msg);
      };

  m_lbr_trq_cmd_subscription_ptr =
      this->get_node()->create_subscription<TorqueCmd>(
          m_trqcmd_subscription_topic, rclcpp::SystemDefaultsQoS(),
          joint_cmd_callback);
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::on_init()
    -> controller_interface::CallbackReturn {
  m_torque_cmd_interfaces.reserve(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

  return controller_interface::CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::on_configure(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    -> controller_interface::CallbackReturn {

  try {
    configure_subscribers();
  } catch (const std::exception &msg) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Failed to initialize LBR Chained Torque Forwarder with error: %s",
        msg.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::on_activate(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    -> controller_interface::CallbackReturn {
  if (!reference_command_interfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::update_and_write_commands(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period)
    -> controller_interface::return_type {

  // forward values from reference interfaces
  // to the loaned command interfaces
  for (std::size_t idx = 0; idx < KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
       ++idx) {
    m_torque_cmd_interfaces[idx].get().set_value(reference_interfaces_[idx]);
  }

  return controller_interface::return_type::OK;
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::update_reference_from_subscribers()
    -> controller_interface::return_type {

  // update reference inputs from subscribers when not in chained mode
  m_trq_cmd_msg = *m_rt_lbr_trq_cmd_ptr.readFromRT();
  if (m_trq_cmd_msg.get()) {
    for (std::size_t idx = 0; idx < KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
         ++idx) {
      reference_interfaces_[idx] = m_trq_cmd_msg->torque[idx];
    }
  }

  return controller_interface::return_type::OK;
}

//////////////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::reference_command_interfaces() -> bool {
  // get command interfaces and check the number of joints for activation
  for (auto &command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() ==
        hardware_interface::HW_IF_EFFORT) {
      m_torque_cmd_interfaces.emplace_back(std::ref(command_interface));
    }
  }

  if (m_torque_cmd_interfaces.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of torque command interfaces: (%ld) does not "
                 "match the number of robot joints: (%d)",
                 m_torque_cmd_interfaces.size(),
                 KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::clear_command_interfaces() -> void {
  m_torque_cmd_interfaces.clear();
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::on_deactivate(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    -> controller_interface::CallbackReturn {
  clear_command_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////
auto LBRChainedTorqueForwarder::on_cleanup(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    -> controller_interface::CallbackReturn {
  // reset controller
  // if unable to reset, throw error

  m_rt_lbr_trq_cmd_ptr.reset();
  m_trq_cmd_msg.reset();
  m_lbr_trq_cmd_subscription_ptr.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace dt

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(dt::LBRChainedTorqueForwarder,
                            controller_interface::ChainableControllerInterface)
