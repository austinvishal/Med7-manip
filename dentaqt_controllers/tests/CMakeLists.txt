# torque sine reference
add_executable(lbr_torque_sine_reference
  lbr_torque_sine_reference.cpp
)

ament_target_dependencies(lbr_torque_sine_reference
  fri_vendor
  lbr_fri_msgs
  rclcpp
  sensor_msgs
)

target_link_libraries(lbr_torque_sine_reference
  FRIClient::FRIClient
)

install(TARGETS
lbr_torque_sine_reference
DESTINATION lib/${PROJECT_NAME}
)
