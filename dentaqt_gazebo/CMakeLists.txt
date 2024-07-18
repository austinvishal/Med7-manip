cmake_minimum_required(VERSION 3.22)
project(dentaqt_gazebo VERSION 0.1.0
                       LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-common4 REQUIRED COMPONENTS profiler)
find_package(ignition-gazebo6 REQUIRED COMPONENTS gui)

set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

add_subdirectory(plugins)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch urdf models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
