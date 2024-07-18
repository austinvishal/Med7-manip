cmake_minimum_required(VERSION 3.22)
project(dentaqt_ign_ros2_control VERSION 0.1.0
                                 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

add_library(
  ${PROJECT_NAME}-system SHARED
  src/dentaqt_ign_ros2_control_plugin.cpp
)
target_include_directories(${PROJECT_NAME}-system PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(
  ${PROJECT_NAME}-system
    ignition-gazebo${GZ_SIM_VER}::core
    ignition-plugin${GZ_PLUGIN_VER}::register
)
ament_target_dependencies(${PROJECT_NAME}-system
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  rclcpp_lifecycle
)

add_library(dentaqt_ign_hardware_plugins SHARED
  src/dentaqt_ign_system.cpp)
target_include_directories(dentaqt_ign_hardware_plugins PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>"
                                                               "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
ament_target_dependencies(dentaqt_ign_hardware_plugins
  rclcpp_lifecycle
  hardware_interface
  rclcpp
)
target_link_libraries(
    dentaqt_ign_hardware_plugins
    ignition-gazebo${GZ_SIM_VER}::core
)

# Install
install(TARGETS
dentaqt_ign_hardware_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} dentaqt_ign_hardware_plugins)

install(TARGETS ${PROJECT_NAME}-system
        DESTINATION lib)

pluginlib_export_plugin_description_file(dentaqt_ign_ros2_control dentaqt_ign_hardware_plugins.xml)

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
