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
    matrix[0][3] = 0;

    matrix[1][0] = 2 * q.x * q.y + 2 * q.z * q.w;
    matrix[1][1] = 1 - 2 * q.x * q.x - 2 * q.z * q.z;
    matrix[1][2] = 2 * q.y * q.z - 2 * q.x * q.w;
    matrix[1][3] = 0;

    matrix[2][0] = 2 * q.x * q.z - 2 * q.y * q.w;
    matrix[2][1] = 2 * q.y * q.z + 2 * q.x * q.w;
    matrix[2][2] = 1 - 2 * q.x * q.x - 2 * q.y * q.y;
    matrix[2][3] = 0;

    matrix[3][0] = v[0];
    matrix[3][1] = v[1];
    matrix[3][2] = v[2];
    matrix[3][3] = v[3];

    return matrix;
}

Vector4 transformPoint(const Matrix4x4 &transformMatrix, const Vector4 &point) {
  Vector4 transformedPoint = {0, 0, 0, 0};
  RCLCPP_INFO(this->get_logger(), "-----------------------");
  RCLCPP_INFO(this->get_logger(), "point: '%f' '%f' '%f' '%f'", point[0], point[1], point[2], point[3]);
  RCLCPP_INFO(this->get_logger(), "matrix: '%f' '%f' '%f' '%f'", transformMatrix[0][0], transformMatrix[0][1], transformMatrix[0][2], transformMatrix[0][3]);
  RCLCPP_INFO(this->get_logger(), "matrix: '%f' '%f' '%f' '%f'", transformMatrix[1][0], transformMatrix[1][1], transformMatrix[1][2], transformMatrix[1][3]);
  RCLCPP_INFO(this->get_logger(), "matrix: '%f' '%f' '%f' '%f'", transformMatrix[2][0], transformMatrix[2][1], transformMatrix[2][2], transformMatrix[2][3]);
  RCLCPP_INFO(this->get_logger(), "matrix: '%f' '%f' '%f' '%f'", transformMatrix[3][0], transformMatrix[3][1], transformMatrix[3][2], transformMatrix[3][3]);

  for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
          transformedPoint[i] += transformMatrix[i][j] * point[j];
      }
  }
  RCLCPP_INFO(this->get_logger(), "transformed point: '%f' '%f' '%f' '%f'", transformedPoint[0], transformedPoint[1], transformedPoint[2], transformedPoint[3]);

  return transformedPoint;
}

class scanMerger : public rclcpp::Node
{
public:
  scanMerger() : Node("ros2_laser_scan_merger")
  {
    initialize_params();
    refresh_params();

    laser1_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    laser2_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    found_tf_ = false;

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic1_, default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
    sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic2_, default_qos, std::bind(&scanMerger::scan_callback2, this, std::placeholders::_1));

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS());
    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan/merge", rclcpp::SensorDataQoS());
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
    refresh_params();
    if (!laser1_ || !laser2_) {
      return;
    }

    auto merged_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    merged_scan->header.stamp = this->now();
    merged_scan->header.frame_id = "base_link";
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

    merge_laser_scan(laser1_, merged_scan, 0.364160, -0.074248, 2.35619);
    // merge_laser_scan(laser2_, merged_scan, -0.72, 0.324, -0.785398);

    laser_scan_pub_->publish(*merged_scan);
  }

  void merge_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr& input_scan, const sensor_msgs::msg::LaserScan::SharedPtr& output_scan, float x_offset, float y_offset, float yaw_offset)
  {
    if (!found_tf_){
      std::string fromFrameRel ="base_link";
      std::string toFrameRel ="laser";
          try{
        geometry_msgs::msg::TransformStamped transformStamped;
        //scan1
        
        transformStamped = tf_buffer_->lookupTransform(
          fromFrameRel, toFrameRel,
          tf2::TimePointZero);
        Quaternion q = {transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z};
        scan1_tf_matrix_ = quaternionToMatrix(q, {transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z, 1.0});
         RCLCPP_INFO(this->get_logger(), "translation: '%f' '%f'", transformStamped.transform.translation.x,
            transformStamped.transform.translation.y);
          RCLCPP_INFO(this->get_logger(), "rotation: '%f' '%f' '%f' '%f'", q.w, q.x, q.y, q.z);
        // //scan2
        // std::string fromFrameRel2 ="laser2";
        // transformStamped = tf_buffer_->lookupTransform(
        //   fromFrameRel2, target_frame_,
        //   tf2::TimePointZero);
        // q = {transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z};
        // scan2_tf_matrix_ = quaternionToMatrix(q);

        found_tf_ = true;
      } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          fromFrameRel.c_str() , toFrameRel.c_str(), ex.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return;
      }
    }
            
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x_offset, y_offset, 0.0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_offset);
    transform.setRotation(q);

    float angle = input_scan->angle_min;
    for (size_t i = 0; i < input_scan->ranges.size(); ++i) {
      angle = input_scan->angle_min + i * input_scan->angle_increment;
      if (input_scan->ranges[i] < input_scan->range_min || input_scan->ranges[i] > input_scan->range_max) {
        // angle += input_scan->angle_increment;
        continue;
      }

      // tf2::Vector3 point(input_scan->ranges[i] * std::cos(angle), input_scan->ranges[i] * std::sin(angle), 0.0);
      Vector4 point = {input_scan->ranges[i] * std::cos(angle), input_scan->ranges[i] * std::sin(angle), 0.0, 1.0};
      point = transformPoint(scan1_tf_matrix_, point);
      // point = transform * point;

      float transformed_angle = std::atan2(point[1], point[0]);
      float transformed_range = sqrt(pow(point[0], 2) + pow(point[1], 2));
      if (transformed_angle < 0) {
        transformed_angle += 2 * M_PI;
      }

      if (transformed_angle < output_scan->angle_min || transformed_angle > output_scan->angle_max) {
        // angle += input_scan->angle_increment;
        continue;
      }

      size_t index = static_cast<size_t>(std::round((transformed_angle - output_scan->angle_min) / output_scan->angle_increment));
      if (index < output_scan->ranges.size()) {
        if (transformed_range < output_scan->ranges[index]) {
          output_scan->ranges[index] = transformed_range;
          output_scan->intensities[index] = input_scan->intensities[i];
        }
      }
      // angle += input_scan->angle_increment;
    }
  }



  void update_point_cloud_rgb()
  {
    refresh_params();
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    std::vector<std::array<float, 2>> scan_data;
    int count = 0;
    float min_theta = 0;
    float max_theta = 0;
    if (show1_ && laser1_)
    {
      float temp_min_, temp_max_;
      if( laser1_->angle_min < laser1_->angle_max){
        temp_min_ = laser1_->angle_min;
        temp_max_ = laser1_->angle_max;
      } else{
        temp_min_ = laser1_->angle_max;
        temp_max_ = laser1_->angle_min;
      }
      for (float i = temp_min_; i <= temp_max_ && count < laser1_->ranges.size();
           i += laser1_->angle_increment)
      {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(laser1R_, laser1G_, laser1B_);
        int used_count_ = count;
        if (flip1_)
        {
          used_count_ = (int)laser1_->ranges.size() - 1 - count;
        }
        float temp_x = laser1_->ranges[used_count_] * std::cos(i);
        float temp_y = laser1_->ranges[used_count_] * std::sin(i);
        pt.x =
            temp_x * std::cos(laser1Alpha_ * M_PI / 180) - temp_y * std::sin(laser1Alpha_ * M_PI / 180) + laser1XOff_;
        pt.y =
            temp_x * std::sin(laser1Alpha_ * M_PI / 180) + temp_y * std::cos(laser1Alpha_ * M_PI / 180) + laser1YOff_;
        pt.z = laser1ZOff_;
        if ((i < (laser1AngleMin_ * M_PI / 180)) || (i > (laser1AngleMax_ * M_PI / 180)))
        {
          if (inverse1_)
          {
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        else
        {
          if (!inverse1_)
          {
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        count++;
      }
    }

    count = 0;
    if (show2_ && laser2_)
    {
      float temp_min_, temp_max_;
      if( laser2_->angle_min < laser2_->angle_max){
        temp_min_ = laser2_->angle_min;
        temp_max_ = laser2_->angle_max;
      } else{
        temp_min_ = laser2_->angle_max;
        temp_max_ = laser2_->angle_min;
      }
      for (float i = temp_min_; i <= temp_max_ && count < laser2_->ranges.size();
           i += laser2_->angle_increment)
      {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(laser2R_, laser2G_, laser2B_);

        int used_count_ = count;
        if (flip2_)
        {
          used_count_ = (int)laser2_->ranges.size() - 1 - count;
        }

        float temp_x = laser2_->ranges[used_count_] * std::cos(i);
        float temp_y = laser2_->ranges[used_count_] * std::sin(i);
        pt.x =
            temp_x * std::cos(laser2Alpha_ * M_PI / 180) - temp_y * std::sin(laser2Alpha_ * M_PI / 180) + laser2XOff_;
        pt.y =
            temp_x * std::sin(laser2Alpha_ * M_PI / 180) + temp_y * std::cos(laser2Alpha_ * M_PI / 180) + laser2YOff_;
        pt.z = laser2ZOff_;
        if ((i < (laser2AngleMin_ * M_PI / 180)) || (i > (laser2AngleMax_ * M_PI / 180)))
        {
          if (inverse2_)
          {
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        else
        {
          if (!inverse2_)
          {
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        count++;
      }
    }

    auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *pc2_msg_);
    pc2_msg_->header.frame_id = cloudFrameId_;
    pc2_msg_->header.stamp = now();
    pc2_msg_->is_dense = false;
    point_cloud_pub_->publish(*pc2_msg_);
  }

  float GET_R(float x, float y)
  {
    return sqrt(x * x + y * y);
  }
  float GET_THETA(float x, float y)
  {
    float temp_res;
    if ((x != 0))
    {
      temp_res = atan(y / x);
    }
    else
    {
      if (y >= 0)
      {
        temp_res = M_PI / 2;
      }
      else
      {
        temp_res = -M_PI / 2;
      }
    }
    if (temp_res > 0)
    {
      if (y < 0)
      {
        temp_res -= M_PI;
      }
    }
    else if (temp_res < 0)
    {
      if (x < 0)
      {
        temp_res += M_PI;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "x: '%f', y: '%f', a: '%f'", x, y, temp_res);

    return temp_res;
  }
  float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle)
  {
    return (magnitude_1 + current_angle * ((magnitude_2 - magnitude_1) / (angle_2 - angle_1)));
  }
  void initialize_params()
  {
    this->declare_parameter("pointCloudTopic", "base/custom_cloud");
    this->declare_parameter("pointCloutFrameId", "laser");

    this->declare_parameter("scanTopic1", "lidar_front_right/scan");
    this->declare_parameter("laser1XOff", -0.45);
    this->declare_parameter("laser1YOff", 0.24);
    this->declare_parameter("laser1ZOff", 0.0);
    this->declare_parameter("laser1Alpha", 45.0);
    this->declare_parameter("laser1AngleMin", -181.0);
    this->declare_parameter("laser1AngleMax", 181.0);
    this->declare_parameter("laser1R", 255);
    this->declare_parameter("laser1G", 0);
    this->declare_parameter("laser1B", 0);
    this->declare_parameter("show1", true);
    this->declare_parameter("flip1", false);
    this->declare_parameter("inverse1", false);

    this->declare_parameter("scanTopic2", "lidar_rear_left/scan");
    this->declare_parameter("laser2XOff", 0.315);
    this->declare_parameter("laser2YOff", -0.24);
    this->declare_parameter("laser2ZOff", 0.0);
    this->declare_parameter("laser2Alpha", 225.0);
    this->declare_parameter("laser2AngleMin", -181.0);
    this->declare_parameter("laser2AngleMax", 181.0);
    this->declare_parameter("laser2R", 0);
    this->declare_parameter("laser2G", 0);
    this->declare_parameter("laser2B", 255);
    this->declare_parameter("show2", true);
    this->declare_parameter("flip2", false);
    this->declare_parameter("inverse2", false);
  }
  void refresh_params()
  {
    this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "pointCloud");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloudFrameId_, "laser");
    this->get_parameter_or<std::string>("scanTopic1", topic1_, "scan");
    this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.0);
    this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser1ZOff", laser1ZOff_, 0.0);
    this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 0.0);
    this->get_parameter_or<float>("laser1AngleMin", laser1AngleMin_, -181.0);
    this->get_parameter_or<float>("laser1AngleMax", laser1AngleMax_, 181.0);
    this->get_parameter_or<uint8_t>("laser1R", laser1R_, 0);
    this->get_parameter_or<uint8_t>("laser1G", laser1G_, 0);
    this->get_parameter_or<uint8_t>("laser1B", laser1B_, 0);
    this->get_parameter_or<bool>("show1", show1_, true);
    this->get_parameter_or<bool>("flip1", flip1_, false);
    this->get_parameter_or<bool>("inverse1", inverse1_, false);
    this->get_parameter_or<std::string>("scanTopic2", topic2_, "lidar_rear_left/scan");
    this->get_parameter_or<float>("laser2XOff", laser2XOff_, 0.0);
    this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser2ZOff", laser2ZOff_, 0.0);
    this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
    this->get_parameter_or<float>("laser2AngleMin", laser2AngleMin_, -181.0);
    this->get_parameter_or<float>("laser2AngleMax", laser2AngleMax_, 181.0);
    this->get_parameter_or<uint8_t>("laser2R", laser2R_, 0);
    this->get_parameter_or<uint8_t>("laser2G", laser2G_, 0);
    this->get_parameter_or<uint8_t>("laser2B", laser2B_, 0);
    this->get_parameter_or<bool>("show2", show2_, false);
    this->get_parameter_or<bool>("flip2", flip2_, false);
    this->get_parameter_or<bool>("inverse2", inverse2_, false);
  }
  std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
  bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_, found_tf_;
  float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
  uint8_t laser1R_, laser1G_, laser1B_;

  float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
  uint8_t laser2R_, laser2G_, laser2B_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::array<std::array<double, 4>, 4> scan1_tf_matrix_;

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
