cmake_minimum_required(VERSION 3.8)
project(robot_simple)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)



include_directories(include)

set(node_plugins "")

add_library(robot_component SHARED
  src/battery.cpp
  src/robot.cpp)
target_include_directories(robot_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(robot_component
  PRIVATE "robot_simple_BUILDING_DLL")
ament_target_dependencies(robot_component
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "robot_interfaces"
)
rclcpp_components_register_node(robot_component PLUGIN "robot_simple::Battery" EXECUTABLE "battery")
rclcpp_components_register_node(robot_component PLUGIN "robot_simple::Robot" EXECUTABLE "robot")

set(node_plugins "${node_plugins}robot_simple::Battery;$<TARGET_FILE:battery_component>\n")
set(node_plugins "${node_plugins}robot_simple::Robot;$<TARGET_FILE:robot_component>\n")


install(TARGETS
  robot_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)



add_executable(robot_system
src/robot_system.cpp)

target_link_libraries(robot_system
  robot_component
)
ament_target_dependencies(robot_system
  "rclcpp"
)
install(TARGETS
  robot_system
  DESTINATION lib/${PROJECT_NAME})


# add_executable(robot_node 
#   src/robot.cpp)
# target_include_directories(robot_node PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(robot_node
#   PRIVATE "robot_simple_BUILDING_DLL")
# ament_target_dependencies(robot_node
#   "rclcpp"
#   "rclcpp_action"
#   "geometry_msgs"
#   "tf2"
#   "tf2_ros"
#   "robot_interfaces"
# )

# add_executable(battery_node 
#   src/battery.cpp)
# target_include_directories(battery_node PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(battery_node
#   PRIVATE "robot_simple_BUILDING_DLL")
# ament_target_dependencies(battery_node
#   "rclcpp"
#   "rclcpp_action"
#   "geometry_msgs"
#   "tf2"
#   "tf2_ros"
#   "robot_interfaces"
# )

# install(TARGETS
#   robot_node
#   battery_node
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)


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

ament_package()


