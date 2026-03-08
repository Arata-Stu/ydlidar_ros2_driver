#include "ydlidar_ros2_driver/ydlidar_ros2_driver_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <math.h>

using namespace std::chrono_literals;

namespace ydlidar_ros2_driver {

YdlidarRos2DriverComponent::YdlidarRos2DriverComponent(
    const rclcpp::NodeOptions &options)
    : Node("ydlidar_ros2_driver_node", options) {
  RCLCPP_INFO(this->get_logger(),
              "[YDLIDAR INFO] Starting customized Tmini Plus Driver Component");

  laser_ = std::make_unique<CYdLidar>();

  // --- Node Parameters (User Configurable) ---
  std::string port =
      this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "laser_frame");

  // Field of View (FOV) Software Limits [rad]
  angle_min_limit_ = this->declare_parameter<float>("angle_min", -M_PI);
  angle_max_limit_ = this->declare_parameter<float>("angle_max", M_PI);

  // --- Tmini Plus Hardcoded Parameters ---
  std::string ignore_array = "";
  int baudrate = 230400;
  int lidar_type = TYPE_TRIANGLE;
  int device_type = YDLIDAR_TYPE_SERIAL;
  int sample_rate = 4;
  int abnormal_check_count = 4;
  int intensity_bit = 16;

  bool fixed_resolution = true;
  bool reversion = true;
  bool inverted = true;
  bool auto_reconnect = true;
  bool isSingleChannel = false;
  bool intensity = true;
  bool support_motor_dtr = false;

  float angle_max_hw = 180.0f;
  float angle_min_hw = -180.0f;
  float range_max = 12.0f;
  float range_min = 0.03f;
  float frequency = 10.0f;

  // --- Apply to SDK ---
  laser_->setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  laser_->setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                      ignore_array.size());
  laser_->setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  laser_->setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));
  laser_->setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));
  laser_->setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));
  laser_->setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check_count,
                      sizeof(int));
  laser_->setlidaropt(LidarPropIntenstiyBit, &intensity_bit, sizeof(int));

  laser_->setlidaropt(LidarPropFixedResolution, &fixed_resolution,
                      sizeof(bool));
  laser_->setlidaropt(LidarPropReversion, &reversion, sizeof(bool));
  laser_->setlidaropt(LidarPropInverted, &inverted, sizeof(bool));
  laser_->setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));
  laser_->setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
  laser_->setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));
  laser_->setlidaropt(LidarPropSupportMotorDtrCtrl, &support_motor_dtr,
                      sizeof(bool));

  laser_->setlidaropt(LidarPropMaxAngle, &angle_max_hw, sizeof(float));
  laser_->setlidaropt(LidarPropMinAngle, &angle_min_hw, sizeof(float));
  laser_->setlidaropt(LidarPropMaxRange, &range_max, sizeof(float));
  laser_->setlidaropt(LidarPropMinRange, &range_min, sizeof(float));
  laser_->setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  bool ret = laser_->initialize();
  if (ret) {
    ret = laser_->turnOn();
    if (!ret) {
      RCLCPP_ERROR(this->get_logger(), "Failed to turn on laser: %s",
                   laser_->DescribeError());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize: %s",
                 laser_->DescribeError());
  }

  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS());

  stop_service_ = this->create_service<std_srvs::srv::Empty>(
      "stop_scan", std::bind(&YdlidarRos2DriverComponent::stop_scan_callback,
                             this, std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3));

  start_service_ = this->create_service<std_srvs::srv::Empty>(
      "start_scan", std::bind(&YdlidarRos2DriverComponent::start_scan_callback,
                              this, std::placeholders::_1,
                              std::placeholders::_2, std::placeholders::_3));

  // 20Hz loop rate
  timer_ = this->create_wall_timer(
      50ms, std::bind(&YdlidarRos2DriverComponent::timer_callback, this));
}

YdlidarRos2DriverComponent::~YdlidarRos2DriverComponent() {
  RCLCPP_INFO(this->get_logger(), "[YDLIDAR INFO] Stopping YDLIDAR Tmini Plus");
  laser_->turnOff();
  laser_->disconnecting();
}

bool YdlidarRos2DriverComponent::stop_scan_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  (void)request_header;
  (void)req;
  (void)response;
  return laser_->turnOff();
}

bool YdlidarRos2DriverComponent::start_scan_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  (void)request_header;
  (void)req;
  (void)response;
  return laser_->turnOn();
}

void YdlidarRos2DriverComponent::timer_callback() {
  LaserScan scan;
  if (laser_->doProcessSimple(scan)) {
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

    scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
    scan_msg->header.stamp.nanosec =
        scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
    scan_msg->header.frame_id = frame_id_;

    scan_msg->angle_min =
        std::max((float)angle_min_limit_, scan.config.min_angle);
    scan_msg->angle_max =
        std::min((float)angle_max_limit_, scan.config.max_angle);
    scan_msg->angle_increment = scan.config.angle_increment;
    scan_msg->scan_time = scan.config.scan_time;
    scan_msg->time_increment = scan.config.time_increment;
    scan_msg->range_min = scan.config.min_range;
    scan_msg->range_max = scan.config.max_range;

    int total_size = (scan.config.max_angle - scan.config.min_angle) /
                         scan.config.angle_increment +
                     1;
    int publish_size = (scan_msg->angle_max - scan_msg->angle_min) /
                           scan.config.angle_increment +
                       1;

    if (publish_size > 0) {
      scan_msg->ranges.resize(publish_size,
                              std::numeric_limits<float>::infinity());
      scan_msg->intensities.resize(publish_size, 0.0);

      for (size_t i = 0; i < scan.points.size(); ++i) {
        float point_angle = scan.points[i].angle;
        float point_range = scan.points[i].range;
        float point_intensity = scan.points[i].intensity;

        if (point_angle >= scan_msg->angle_min &&
            point_angle <= scan_msg->angle_max) {
          int index = std::ceil((point_angle - scan_msg->angle_min) /
                                scan.config.angle_increment);
          if (index >= 0 && index < publish_size) {
            if (point_range >= scan.config.min_range &&
                point_range <= scan.config.max_range) {
              scan_msg->ranges[index] = point_range;
              scan_msg->intensities[index] = point_intensity;
            }
          }
        }
      }
      laser_pub_->publish(std::move(scan_msg));
    }
  } else {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Failed to get scan");
  }
}

} // namespace ydlidar_ros2_driver

RCLCPP_COMPONENTS_REGISTER_NODE(ydlidar_ros2_driver::YdlidarRos2DriverComponent)
