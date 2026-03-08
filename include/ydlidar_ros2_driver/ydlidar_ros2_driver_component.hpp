#ifndef YDLIDAR_ROS2_DRIVER_COMPONENT_HPP_
#define YDLIDAR_ROS2_DRIVER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "src/CYdLidar.h"
#include <memory>

namespace ydlidar_ros2_driver
{

class YdlidarRos2DriverComponent : public rclcpp::Node
{
public:
  explicit YdlidarRos2DriverComponent(const rclcpp::NodeOptions & options);
  ~YdlidarRos2DriverComponent() override;

private:
  void timer_callback();
  bool stop_scan_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  bool start_scan_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  std::unique_ptr<CYdLidar> laser_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  float angle_min_limit_;
  float angle_max_limit_;
};

}  // namespace ydlidar_ros2_driver

#endif  // YDLIDAR_ROS2_DRIVER_COMPONENT_HPP_
