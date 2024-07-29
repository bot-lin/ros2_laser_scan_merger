//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <cmath>

#include <string>
#include <vector>
#include <array>
#include <iostream>

using Matrix4x4 = std::array<std::array<double, 4>, 4>;
using Vector4 = std::array<double, 4>;

struct Quaternion {
    double w, x, y, z;
};

Matrix4x4 quaternionToMatrix(const Quaternion& q, const Vector4& v) {
    Matrix4x4 matrix = {0};

    matrix[0][0] = 1 - 2 * q.y * q.y - 2 * q.z * q.z;
    matrix[0][1] = 2 * q.x * q.y - 2 * q.z * q.w;
    matrix[0][2] = 2 * q.x * q.z + 2 * q.y * q.w;
    matrix[0][3] = v[0];

    matrix[1][0] = 2 * q.x * q.y + 2 * q.z * q.w;
    matrix[1][1] = 1 - 2 * q.x * q.x - 2 * q.z * q.z;
    matrix[1][2] = 2 * q.y * q.z - 2 * q.x * q.w;
    matrix[1][3] = v[1];

    matrix[2][0] = 2 * q.x * q.z - 2 * q.y * q.w;
    matrix[2][1] = 2 * q.y * q.z + 2 * q.x * q.w;
    matrix[2][2] = 1 - 2 * q.x * q.x - 2 * q.y * q.y;
    matrix[2][3] = v[2];

    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;

    return matrix;
}

Vector4 transformPoint(const Matrix4x4 &transformMatrix, const Vector4 &point) {
  Vector4 transformedPoint = {0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
          transformedPoint[i] += transformMatrix[i][j] * point[j];
      }
  }
  return transformedPoint;
}



class scanMerger : public rclcpp::Node
{
public:
  scanMerger() : Node("ros2_laser_scan_merger")
  {

    laser1_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    laser2_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    found_tf_ = false;

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan1", default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
    sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan2", default_qos, std::bind(&scanMerger::scan_callback2, this, std::placeholders::_1));

    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Hello");
  }

private:
  void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    laser1_ = _msg;
    update_merged_scan();
    // update_point_cloud_rgb();
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
    //         _msg->ranges[100]);
  }
  void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    laser2_ = _msg;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
    //         _msg->ranges[100]);
  }

    void update_merged_scan()
  {
    if (!laser1_ || !laser2_) {
      return;
    }

    auto merged_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    merged_scan->header.stamp = this->now();
    merged_scan->header.frame_id = "laser_merge";
    merged_scan->angle_min = 0.0;
    merged_scan->angle_max = M_PI * 2;
    merged_scan->angle_increment = std::min(laser1_->angle_increment, laser2_->angle_increment);
    merged_scan->time_increment = std::min(laser1_->time_increment, laser2_->time_increment);
    merged_scan->scan_time = std::max(laser1_->scan_time, laser2_->scan_time);
    merged_scan->range_min = std::min(laser1_->range_min, laser2_->range_min);
    merged_scan->range_max = std::max(laser1_->range_max, laser2_->range_max);

    size_t total_ranges = static_cast<size_t>((merged_scan->angle_max - merged_scan->angle_min) / merged_scan->angle_increment) +1;
    merged_scan->ranges.resize(total_ranges, std::numeric_limits<float>::infinity());
    merged_scan->intensities.resize(total_ranges, 0.0);

    if (!found_tf_){
      std::string fromFrameRel ="laser_merge";
      std::string toFrameRel ="laser";
      std::string toFrameRel2 = "laser2";
          try{
        geometry_msgs::msg::TransformStamped transformStamped;
        Quaternion q = {0, 0, 0, 0};
        //scan1
        transformStamped = tf_buffer_->lookupTransform(
          fromFrameRel, toFrameRel,
          tf2::TimePointZero);
        q = {transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z};
        scan1_tf_matrix_ = quaternionToMatrix(q, {transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z, 1.0});
        ///scan2
        transformStamped = tf_buffer_->lookupTransform(
          fromFrameRel, toFrameRel2,
          tf2::TimePointZero);
        q = {transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z};
        scan2_tf_matrix_ = quaternionToMatrix(q, {transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z, 1.0});
       

        found_tf_ = true;
      } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          fromFrameRel.c_str() , toFrameRel.c_str(), ex.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return;
      }
    }

    merge_laser_scan(laser1_, merged_scan, scan1_tf_matrix_);
    merge_laser_scan(laser2_, merged_scan, scan2_tf_matrix_);

    laser_scan_pub_->publish(*merged_scan);
  }

  void merge_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr& input_scan, const sensor_msgs::msg::LaserScan::SharedPtr& output_scan, const std::array<std::array<double, 4>, 4> tranform_matrix)
  {

    float angle = input_scan->angle_min;
    for (size_t i = 0; i < input_scan->ranges.size(); ++i) {
      angle = input_scan->angle_min + i * input_scan->angle_increment;
      if (input_scan->ranges[i] < input_scan->range_min || input_scan->ranges[i] > input_scan->range_max) {
        continue;
      }

      Vector4 point = {input_scan->ranges[i] * std::cos(angle), input_scan->ranges[i] * std::sin(angle), 0.0, 1.0};
      point = transformPoint(tranform_matrix, point);

      float transformed_angle = std::atan2(point[1], point[0]);
      float transformed_range = sqrt(pow(point[0], 2) + pow(point[1], 2));
      if (transformed_angle < 0) {
        transformed_angle += 2 * M_PI;
      }

      if (transformed_angle < output_scan->angle_min || transformed_angle > output_scan->angle_max) {
        continue;
      }

      size_t index = static_cast<size_t>(std::round((transformed_angle - output_scan->angle_min) / output_scan->angle_increment));
      if (index < output_scan->ranges.size()) {
        if (transformed_range < output_scan->ranges[index]) {
          output_scan->ranges[index] = transformed_range;
          output_scan->intensities[index] = input_scan->intensities[i];
        }
      }
    }
  }



  std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
  bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_, found_tf_;
  float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
  uint8_t laser1R_, laser1G_, laser1B_;

  float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
  uint8_t laser2R_, laser2G_, laser2B_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::array<std::array<double, 4>, 4> scan1_tf_matrix_, scan2_tf_matrix_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  

  sensor_msgs::msg::LaserScan::SharedPtr laser1_;
  sensor_msgs::msg::LaserScan::SharedPtr laser2_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanMerger>());
  rclcpp::shutdown();
  return 0;
}
