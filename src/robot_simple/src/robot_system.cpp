#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_simple/battery.hpp"
#include "robot_simple/robot.hpp"



int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize any global resources needed by the middleware and the client library.
  // This will also parse command line arguments one day (as of Beta 1 they are not used).
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;  

  auto robot = std::make_shared<robot_simple::Robot>();
  exec.add_node(robot);
  auto battery = std::make_shared<robot_simple::Battery>();
  exec.add_node(battery);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  exec.spin();

  rclcpp::shutdown();

  return 0;
}