#include "ydlidar_ros2_driver/ydlidar_ros2_driver_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
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
  rotate_angle_ = this->declare_parameter<float>("rotate_angle", 0.0f);
  publish_point_cloud_ =
      this->declare_parameter<bool>("publish_point_cloud", true);

  // --- Tmini Plus Hardcoded Parameters ---
  std::string ignore_array = "";
  int baudrate = 230400;
  int lidar_type = TYPE_TRIANGLE;
  int device_type = YDLIDAR_TYPE_SERIAL;
  int sample_rate = 4;
  int abnormal_check_count = 4;
  int intensity_bit = 16;

  bool fixed_resolution = true;
  bool reversion = this->declare_parameter<bool>("reversion", true);
  bool inverted = this->declare_parameter<bool>("inverted", true);
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
  if (publish_point_cloud_) {
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", rclcpp::SensorDataQoS());
  }

  RCLCPP_INFO(this->get_logger(), "point_cloud publishing: %s",
              publish_point_cloud_ ? "enabled" : "disabled");

  stop_service_ = this->create_service<std_srvs::srv::Empty>(
      "stop_scan", std::bind(&YdlidarRos2DriverComponent::stop_scan_callback,
                             this, std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3));

  start_service_ = this->create_service<std_srvs::srv::Empty>(
      "start_scan", std::bind(&YdlidarRos2DriverComponent::start_scan_callback,
                              this, std::placeholders::_1,
                              std::placeholders::_2, std::placeholders::_3));

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

float YdlidarRos2DriverComponent::normalize_angle(float angle) {
  while (angle > static_cast<float>(M_PI)) {
    angle -= 2.0f * static_cast<float>(M_PI);
  }
  while (angle < -static_cast<float>(M_PI)) {
    angle += 2.0f * static_cast<float>(M_PI);
  }
  return angle;
}

std::uint32_t YdlidarRos2DriverComponent::color_from_index(std::size_t index,
                                                           std::size_t size) {
  const float normalized =
      size > 1 ? static_cast<float>(index) / static_cast<float>(size - 1) : 0.0f;
  const std::uint8_t red = static_cast<std::uint8_t>(255.0f * normalized);
  const std::uint8_t green = static_cast<std::uint8_t>(
      255.0f * (1.0f - std::fabs((normalized * 2.0f) - 1.0f)));
  const std::uint8_t blue = static_cast<std::uint8_t>(255.0f * (1.0f - normalized));

  return (static_cast<std::uint32_t>(red) << 16) |
         (static_cast<std::uint32_t>(green) << 8) |
         static_cast<std::uint32_t>(blue);
}

sensor_msgs::msg::PointCloud2
YdlidarRos2DriverComponent::create_point_cloud_message(
    const LaserScan &scan, const std_msgs::msg::Header &header,
    float angle_min, float angle_max) const {
  sensor_msgs::msg::PointCloud2 point_cloud_msg;
  point_cloud_msg.header = header;
  point_cloud_msg.height = 1;
  point_cloud_msg.is_bigendian = false;
  point_cloud_msg.is_dense = false;

  std::size_t valid_points = 0;
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    const auto &point = scan.points[i];
    const float point_angle = normalize_angle(point.angle + rotate_angle_);
    if (point_angle < angle_min || point_angle > angle_max) {
      continue;
    }
    if (point.range < scan.config.min_range || point.range > scan.config.max_range) {
      continue;
    }
    ++valid_points;
  }

  sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(valid_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_rgb(point_cloud_msg, "rgb");

  const std::size_t color_size = valid_points > 0 ? valid_points : 1;
  std::size_t color_index = 0;
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    const auto &point = scan.points[i];
    const float point_angle = normalize_angle(point.angle + rotate_angle_);
    if (point_angle < angle_min || point_angle > angle_max) {
      continue;
    }
    if (point.range < scan.config.min_range || point.range > scan.config.max_range) {
      continue;
    }

    *iter_x = point.range * std::cos(point_angle);
    *iter_y = point.range * std::sin(point_angle);
    *iter_z = 0.0f;

    const std::uint32_t rgb = color_from_index(color_index, color_size);
    float packed_rgb = 0.0f;
    std::memcpy(&packed_rgb, &rgb, sizeof(rgb));
    *iter_rgb = packed_rgb;

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_rgb;
    ++color_index;
  }

  return point_cloud_msg;
}

void YdlidarRos2DriverComponent::timer_callback() {
  LaserScan scan;
  if (laser_->doProcessSimple(scan)) {
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

    scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
    scan_msg->header.stamp.nanosec =
        scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
    scan_msg->header.frame_id = frame_id_;

    scan_msg->angle_min = std::max(angle_min_limit_, scan.config.min_angle);
    scan_msg->angle_max = std::min(angle_max_limit_, scan.config.max_angle);
    scan_msg->angle_increment = scan.config.angle_increment;
    scan_msg->scan_time = scan.config.scan_time;
    scan_msg->time_increment = scan.config.time_increment;
    scan_msg->range_min = scan.config.min_range;
    scan_msg->range_max = scan.config.max_range;

    const int total_size =
        std::ceil((scan.config.max_angle - scan.config.min_angle) /
                  scan.config.angle_increment) +
        1;
    const int publish_size =
        std::ceil((scan_msg->angle_max - scan_msg->angle_min) /
                  scan.config.angle_increment) +
        1;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[YDLIDAR FOV] Data Size: Original=%d -> Published=%d",
                         total_size, publish_size);

    if (publish_size > 0) {
      scan_msg->ranges.resize(publish_size,
                              std::numeric_limits<float>::infinity());
      scan_msg->intensities.resize(publish_size, 0.0);

      for (std::size_t i = 0; i < scan.points.size(); ++i) {
        const float point_angle =
            normalize_angle(scan.points[i].angle + rotate_angle_);
        const float point_range = scan.points[i].range;
        const float point_intensity = scan.points[i].intensity;

        if (point_angle < scan_msg->angle_min || point_angle > scan_msg->angle_max) {
          continue;
        }

        const int index = std::round((point_angle - scan_msg->angle_min) /
                                     scan.config.angle_increment);
        if (index < 0 || index >= publish_size) {
          continue;
        }

        if (point_range >= scan.config.min_range &&
            point_range <= scan.config.max_range) {
          scan_msg->ranges[index] = point_range;
          scan_msg->intensities[index] = point_intensity;
        }
      }

      if (publish_point_cloud_ && point_cloud_pub_) {
        auto point_cloud_msg = create_point_cloud_message(
            scan, scan_msg->header, scan_msg->angle_min, scan_msg->angle_max);
        point_cloud_pub_->publish(point_cloud_msg);
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
