#ifndef ROSWRAPPER_DUAL_HPP_
#define ROSWRAPPER_DUAL_HPP_

#include "ros/ROSWrapper.h"
#include "basic/alias.h"
#include "basic/Manifold.h"

#include <deque>
#include <cmath>

namespace LI2Sup {

class ROSWrapperDual : public ROSWrapper {
public:
  explicit ROSWrapperDual(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ROSWrapperDual() = default;
  using Ptr = std::shared_ptr<ROSWrapperDual>;

private:
  // ---- overrides ----
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg) override;
  void pub_cloud_world_rear(const BASIC::CloudPtr& pc, double time) override;
  void pub_cloud_body_rear(const BASIC::CloudPtr& pc, double time) override;
  bool getRearToFront(BASIC::M3& R, BASIC::V3& t) const override;

  void rearImuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
  void rearLidarHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void setupDualParams();
  void setupDualIO();

  // temporal fusion: find nearest rear IMU and average
  bool tryFuseWithRearIMU(double front_time, BASIC::V3& fused_acc, BASIC::V3& fused_gyr);

  // rear → front transform
  BASIC::SE3 T_rear_to_front_;

  // rear IMU history for temporal fusion
  std::deque<IMUData> rear_imu_history_;
  static constexpr double kImuFusionWindow = 0.002;  // 2ms maximum gap for fusion

  // fusion enable flag
  bool imu_fusion_enable_ = true;

  // rear subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_rear_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_rear_;

  // rear output
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world_rear_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_body_rear_;
};

} // namespace LI2Sup

#endif
