#include "ydlidar_ros2_driver/ydlidar_ros2_driver_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component =
      std::make_shared<ydlidar_ros2_driver::YdlidarRos2DriverComponent>(
          options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
