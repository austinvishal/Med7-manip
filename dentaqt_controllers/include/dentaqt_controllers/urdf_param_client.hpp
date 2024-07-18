#ifndef DENTAQT_CONTROLLERS_URDF_PARAMETER_CLIENT_HPP
#define DENTAQT_CONTROLLERS_URDF_PARAMETER_CLIENT_HPP

#include "visibility_control.hpp"

// CPP Headers
#include <memory>
#include <string>
#include <type_traits>

// ROS 2 Headers
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace dt {

class URDFParameterClient {
public:
  template <typename NodeType>
  DENTAQT_CONTROLLERS_PUBLIC auto
  initialize(typename NodeType::SharedPtr node,
             const std::string &robot_publisher_node,
             const std::string &robot_desc_param) -> bool {
    static_assert(
        std::is_same_v<NodeType, rclcpp::Node> ||
            std::is_same_v<NodeType, rclcpp_lifecycle::LifecycleNode>,
        "NodeType must be either rclcpp::Node or rclcpp::LifeCycleNode.");

    if (node == nullptr) {
      return false;
    }

    auto params_client = std::make_shared<rclcpp::SyncParametersClient>(
        node, robot_publisher_node);

    constexpr int wait_time_1s{2};
    while (
        !params_client->wait_for_service(std::chrono::seconds(wait_time_1s))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "[URDFParameterClient::initialize] "
                                         "Interrupted while waiting for the "
                                         "parameter client service. Exiting.");
        return false;
      }

      RCLCPP_INFO(node->get_logger(),
                  "[URDFParameterClient::initialize] "
                  "Parameter client service not available, "
                  "waiting for the service to become active...");
    }

    if (!params_client->has_parameter(robot_desc_param)) {
      RCLCPP_ERROR(node->get_logger(),
                   "[URDFParameterClient::initialize] "
                   "Parameter with name [%s] to retreive urdf string does not "
                   "exist in the list of parameters provided by the [%s] node",
                   robot_desc_param.c_str(), robot_publisher_node.c_str());
      return false;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "[URDFParameterClient::initialize] "
        "Parameter client service of [%s] node "
        "exists and provides [%s] parameter. Use get_urdf_string(str) to "
        "get the urdf parameter as a string.",
        robot_publisher_node.c_str(), robot_desc_param.c_str());

    auto parameters = params_client->get_parameters({robot_desc_param});
    bool broken{false};
    for (const auto &param : parameters) {
      if (param.get_name() == robot_desc_param) {
        m_urdf_string = param.value_to_string();
        broken = true;
        break;
      }
    }

    if (!broken) {
      RCLCPP_ERROR(node->get_logger(),
                   "[URDFParameterClient::initialize] "
                   "Unable to read urdf string from parameter [%s]",
                   robot_desc_param.c_str());
      return false;
    }

    return true;
  }

  DENTAQT_CONTROLLERS_PUBLIC
  auto get_urdf_string() const -> const std::string & { return m_urdf_string; }

protected:
  std::string m_urdf_string{""};
};

} // namespace dt

#endif // DENTAQT_CONTROLLERS_URDF_PARAMETER_CLIENT_HPP
