#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <lbr_fri_msgs/msg/lbr_torque_command.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class LBRTorqueSineReference : public rclcpp::Node {
  static constexpr double c0d{0.};
  static constexpr double c1d{1.};
  static constexpr double c2d{2.};
  static constexpr double k_pos_amplitude{0.57}; // rad
  static constexpr double k_amplitude{15.};      // Nm
  static constexpr double k_frequency{2.0};      // Hz
  static constexpr double k_sample_time{0.01};
  static constexpr double k_buffer_size{1};
  static constexpr int k_controlled_joint_idx{5};

public:
  LBRTorqueSineReference(const std::string &node_name)
      : rclcpp::Node(node_name) {
    // create a publisher
    m_trq_cmd_pub = this->create_publisher<lbr_fri_msgs::msg::LBRTorqueCommand>(
        "command/torque", k_buffer_size);

    m_jstate_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", k_buffer_size,
        std::bind(&LBRTorqueSineReference::on_lbr_state, this,
                  std::placeholders::_1));
  }

protected:
  auto on_lbr_state(
      [[maybe_unused]] const sensor_msgs::msg::JointState::SharedPtr msg)
      -> void {
    if (msg->position.size() == m_trq_cmd.joint_position.size()) {
      std::copy(msg->position.begin(), msg->position.end(),
                m_trq_cmd.joint_position.begin());
    }

    m_sample_time += k_sample_time;
    if (m_sample_time > c1d) {
      m_sample_time = c0d;
    }
    m_phase = c2d * M_PI * k_frequency * m_sample_time;

    std::fill(m_trq_cmd.torque.begin(), m_trq_cmd.torque.end(), c0d);
    m_trq_cmd.torque[k_controlled_joint_idx] = k_amplitude * std::sin(m_phase);
    m_trq_cmd_pub->publish(m_trq_cmd);
  }

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRTorqueCommand>::SharedPtr
      m_trq_cmd_pub{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jstate_sub{
      nullptr};
  lbr_fri_msgs::msg::LBRTorqueCommand m_trq_cmd{};
  double m_phase{0.0};
  double m_sample_time{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<LBRTorqueSineReference>("lbr_torque_sine_reference"));
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
