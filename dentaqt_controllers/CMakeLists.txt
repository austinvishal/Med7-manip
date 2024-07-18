cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME dentaqt_controllers)
set(PKG_NAME ${PROJECT_NAME})
# Follow semantic versioning for project version
# See https://semver.org/
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

project(${PROJECT_NAME} VERSION ${PROJECT_VERSION}
                        LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify a build type - default build type is Release
if (NOT CMAKE_CONFIGURATION_TYPES)
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release"
            CACHE STRING "Build Types: Debug, Release, MinSizeRel, RelWithDebInfo" FORCE)
    endif()
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# find dependencies
set(ROS2_PKG_DEPENDS
generate_parameter_library
controller_interface
hardware_interface
pluginlib
rclcpp
rclcpp_lifecycle
realtime_tools

fri_vendor
FRIClient
lbr_fri_msgs
lbr_fri_ros2

visualization_msgs
interactive_markers
)

set (THIS_PACKAGE_INCLUDE_DEPENDS
  ${LBR_ROS2_DEPENDS}
  ${ROS2_PKG_DEPENDS}
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)

# generate controller parameters
generate_parameter_library(lbr_gravity_compensation_controller_parameters
  parameter_generators/lbr_gravity_compensation_controller.yaml
)

set(PARAMETER_TARGETS lbr_gravity_compensation_controller_parameters)

set(CONTROLLER_SOURCES  src/lbr_chained_torque_forwarder.cpp
                        src/lbr_gravity_compensation_controller.cpp)

add_library(${PKG_NAME} SHARED ${CONTROLLER_SOURCES})
target_compile_features(${PKG_NAME} PUBLIC cxx_std_17)
target_include_directories(${PKG_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/lbr_gravity_compensation_controller_parameters>
)
target_link_libraries(${PKG_NAME} PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  ${PARAMETER_TARGETS}
)
ament_target_dependencies(${PKG_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PKG_NAME} PRIVATE "DENTAQT_CONTROLLERS_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface plugin_description_files/dentaqt_controllers.xml)

add_subdirectory(tests)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PKG_NAME} ${PARAMETER_TARGETS}
  EXPORT export_${PKG_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PKG_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
