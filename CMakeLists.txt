cmake_minimum_required(VERSION 3.8)
project(pi3hat_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

add_library(pi3hat SHARED
  include/pi3hat/pi3hat.cpp
)

# Create a library target for ActuatorBase
add_library(actuator_base SHARED
  src/actuators/actuator_base.cpp
)
# If actuator_base needs to link against other libraries, do it here with target_link_libraries

# Create a library target for ODriveActuator, which depends on ActuatorBase
add_library(odrive_actuator SHARED
  src/actuators/odrive_actuator.cpp
)
# Link odrive_actuator against actuator_base
target_link_libraries(odrive_actuator
  PRIVATE
  actuator_base
)

# add compile directive for shared library
add_library(pi3hat_hardware_interface SHARED
  src/actuators/actuator_base.cpp
  src/actuators/odrive_actuator.cpp  
  src/pi3hat_hardware_interface.cpp
)

target_link_libraries(pi3hat_hardware_interface PUBLIC
  pi3hat
  bcm_host
  odrive_actuator
)

target_compile_features(pi3hat_hardware_interface PUBLIC cxx_std_17)

target_include_directories(pi3hat_hardware_interface  PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
$<INSTALL_INTERFACE:include/pi3hat_hardware_interface>
)

# # add include directory
# target_include_directories(pi3hat_hardware_interface
#   PRIVATE
#     include
# )

ament_target_dependencies(
  pi3hat_hardware_interface PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "PI3HAT_HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface pi3hat_hardware_interface.xml)

# INSTALL
install(
  TARGETS pi3hat
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include/pi3hat_hardware_interface
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/pi3hat_hardware_interface
)
install(
  DIRECTORY description/urdf
  DESTINATION share/pi3hat_hardware_interface
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_pi3hat_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()