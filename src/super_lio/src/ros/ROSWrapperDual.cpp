#include "ros/ROSWrapperDual.h"

#include <deque>
#include <cmath>

using namespace BASIC;

namespace LI2Sup {

static bool validPoint(float x, float y, float z) {
  double dis = x * x + y * y + z * z;
  return dis > g_blind2 && dis < g_maxrange2;
}


ROSWrapperDual::ROSWrapperDual(const rclcpp::NodeOptions& options)
  : ROSWrapper(options, "super_lio_dual_node")
{
  setupDualParams();
  setupDualIO();
}


void ROSWrapperDual::setupDualParams()
{
  // ---- rear-to-front extrinsic ----
  declare_parameter("lio.dual.rear_to_front",
                    std::vector<double>(6, 0.0));
  std::vector<double> rear_to_front;
  get_parameter("lio.dual.rear_to_front", rear_to_front);

  V3 t(rear_to_front[0], rear_to_front[1], rear_to_front[2]);
  auto R =
      Eigen::AngleAxisd(rear_to_front[5] * M_PI / 180.0,
                         Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rear_to_front[4] * M_PI / 180.0,
                         Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rear_to_front[3] * M_PI / 180.0,
                         Eigen::Vector3d::UnitX());
  T_rear_to_front_ = SE3(R.cast<scalar>(), t.cast<scalar>());

  LOG(INFO) << GREEN << " ---> [Dual] rear_to_front t: "
            << t.transpose() << RESET;

  // ---- IMU fusion config ----
  declare_parameter("lio.dual.imu_fusion", true);
  imu_fusion_enable_ = get_parameter("lio.dual.imu_fusion").as_bool();
  LOG(INFO) << GREEN << " ---> [Dual] imu_fusion: "
            << (imu_fusion_enable_ ? "true" : "false") << RESET;

  // ---- rear topics ----
  declare_parameter("lio.dual.rear_lidar_topic", std::string("/rear_lidar"));
  declare_parameter("lio.dual.rear_imu_topic", std::string("/rear_lidar/imu"));
}


void ROSWrapperDual::setupDualIO()
{
  rclcpp::QoS imu_qos(100);
  if (!g_imu_qos_reliable) {
    imu_qos = rclcpp::QoS(rclcpp::KeepLast(100))
        .best_effort()
        .durability_volatile();
  }

  std::string rear_imu_topic, rear_lidar_topic;
  get_parameter("lio.dual.rear_imu_topic", rear_imu_topic);
  get_parameter("lio.dual.rear_lidar_topic", rear_lidar_topic);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = getSensorCallbackGroup();

  sub_imu_rear_ = create_subscription<sensor_msgs::msg::Imu>(
      rear_imu_topic, imu_qos,
      std::bind(&ROSWrapperDual::rearImuHandler, this, std::placeholders::_1),
      sub_opt);

  rclcpp::QoS lidar_qos(100);
  if (!g_lidar_qos_reliable) {
    lidar_qos = rclcpp::QoS(rclcpp::KeepLast(100))
        .best_effort()
        .durability_volatile();
  }

  sub_lidar_rear_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      rear_lidar_topic, lidar_qos,
      std::bind(&ROSWrapperDual::rearLidarHandler, this, std::placeholders::_1),
      sub_opt);

  auto pointcloud_qos = rclcpp::QoS(rclcpp::KeepLast(2))
      .best_effort()
      .durability_volatile();

  pub_cloud_world_rear_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "rear_lidar/cloud_world", pointcloud_qos);
  pub_cloud_body_rear_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "rear_lidar/body/cloud", pointcloud_qos);

  LOG(INFO) << GREEN << " ---> [Dual] Subscribed rear IMU: "
            << rear_imu_topic << RESET;
  LOG(INFO) << GREEN << " ---> [Dual] Subscribed rear LiDAR: "
            << rear_lidar_topic << RESET;
}


// =================== IMU Fusion ============================================
// Two IMUs rigidly mounted see the SAME angular velocity.
// Gyro: rotate rear gyr to front frame, then average.
// Accel: rotate rear acc to front frame, account for lever-arm if desired,
//        then average (lever-arm correction omitted here - for a ground robot
//        the centripetal terms are typically << noise level).
// ==========================================================================

bool ROSWrapperDual::tryFuseWithRearIMU(double front_time,
                                        V3& fused_acc, V3& fused_gyr)
{
  if (!imu_fusion_enable_ || rear_imu_history_.empty())
    return false;

  // Remove stale rear samples (> kImuFusionWindow from front_time)
  while (!rear_imu_history_.empty() &&
         rear_imu_history_.front().secs < front_time - kImuFusionWindow)
    rear_imu_history_.pop_front();

  if (rear_imu_history_.empty())
    return false;

  // Find nearest rear sample within window
  const IMUData* best = &rear_imu_history_.front();
  double best_gap = std::abs(best->secs - front_time);

  for (const auto& r : rear_imu_history_) {
    double gap = std::abs(r.secs - front_time);
    if (gap < best_gap) {
      best_gap = gap;
      best = &r;
    }
  }

  if (best_gap > kImuFusionWindow)
    return false;

  // ---- Fuse ----
  // Both the front IMU data (fused_acc/gyr) and the rear IMU data
  // (best->acc/gyr) are already in the front LiDAR coordinate frame,
  // rotated in their respective handlers. Direct 50/50 average.
  fused_gyr = (fused_gyr + best->gyr) * 0.5;
  fused_acc = (fused_acc + best->acc) * 0.5;

  return true;
}


void ROSWrapperDual::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  IMUData data;
  data.secs = this->now().seconds();

  V3 acc_raw(msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z);
  V3 gyr_raw(msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z);

  M3 R_IMU_to_Lidar = g_lidar_imu.R_.transpose();
  data.acc = R_IMU_to_Lidar * acc_raw;
  data.gyr = R_IMU_to_Lidar * gyr_raw;

  // ---- Dual-IMU fusion: average with nearby rear sample ----
  V3 fused_acc = data.acc;
  V3 fused_gyr = data.gyr;
  bool fused = tryFuseWithRearIMU(data.secs, fused_acc, fused_gyr);

  if (data.secs < last_timestamp_imu_) {
    LOG(WARNING) << "imu loop back, clear buffer";
    imu_buffer_.clear();
    imu_buffer_.push_back(data);   // raw front, fallback
    last_timestamp_imu_ = data.secs;
    return;
  }

  // Push to LIO buffer (use fused data for better accuracy)
  IMUData fused_data = data;
  if (fused) {
    fused_data.acc = fused_acc;
    fused_data.gyr = fused_gyr;
  }
  imu_buffer_.push_back(fused_data);
  last_timestamp_imu_ = data.secs;

  // ---- IMU-rate odometry: run Predict with fused data ----
  DynamicState imu_state, robo_state;
  if (eskf_->Predict(fused_data, imu_state, robo_state)) {
    nav_msgs::msg::Odometry odom_imu, odom_robo;

    {
      odom_imu.pose.pose.position.x = imu_state.p(0);
      odom_imu.pose.pose.position.y = imu_state.p(1);
      odom_imu.pose.pose.position.z = imu_state.p(2);

      Quat q(imu_state.R);
      q.normalize();

      odom_imu.pose.pose.orientation.x = q.x();
      odom_imu.pose.pose.orientation.y = q.y();
      odom_imu.pose.pose.orientation.z = q.z();
      odom_imu.pose.pose.orientation.w = q.w();

      odom_imu.twist.twist.linear.x = imu_state.v(0);
      odom_imu.twist.twist.linear.y = imu_state.v(1);
      odom_imu.twist.twist.linear.z = imu_state.v(2);

      odom_imu.twist.twist.angular.x = imu_state.w(0);
      odom_imu.twist.twist.angular.y = imu_state.w(1);
      odom_imu.twist.twist.angular.z = imu_state.w(2);
    }

    {
      odom_robo.pose.pose.position.x = robo_state.p(0);
      odom_robo.pose.pose.position.y = robo_state.p(1);
      odom_robo.pose.pose.position.z = robo_state.p(2);

      Quat q(robo_state.R);
      q.normalize();

      odom_robo.pose.pose.orientation.x = q.x();
      odom_robo.pose.pose.orientation.y = q.y();
      odom_robo.pose.pose.orientation.z = q.z();
      odom_robo.pose.pose.orientation.w = q.w();
    }

    odom_imu.header.stamp = this->now();
    odom_robo.header.stamp = this->now();
    odom_imu.header.frame_id = g_world_frame;
    odom_imu.child_frame_id = g_imu_frame;
    odom_robo.header.frame_id = g_world_frame;
    odom_robo.child_frame_id = "base_link";
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);

    // ---- fast_tf (duplicated from parent, uses fused IMU state) ----
    if (g_fast_tf) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = this->now();
      tf_msg.header.frame_id = g_world_frame;
      tf_msg.child_frame_id = g_imu_frame;

      tf_msg.transform.translation.x = imu_state.p(0);
      tf_msg.transform.translation.y = imu_state.p(1);
      tf_msg.transform.translation.z = imu_state.p(2);

      Quat q_imu(imu_state.R);
      tf_msg.transform.rotation.x = q_imu.x();
      tf_msg.transform.rotation.y = q_imu.y();
      tf_msg.transform.rotation.z = q_imu.z();
      tf_msg.transform.rotation.w = q_imu.w();

      tf_broadcaster_->sendTransform(tf_msg);

      if (g_footprint_pub_en) {
        geometry_msgs::msg::TransformStamped tf_footprint;
        tf_footprint.header.stamp = this->now();
        tf_footprint.header.frame_id = g_world_frame;
        tf_footprint.child_frame_id = g_tf_base_footprint_frame;

        tf_footprint.transform.translation.x = imu_state.p(0);
        tf_footprint.transform.translation.y = imu_state.p(1);
        tf_footprint.transform.translation.z = imu_state.p(2);

        Eigen::Vector3f world_up;
        if (g_ref_gravity_axis == 0)      world_up = Eigen::Vector3f(-1, 0, 0);
        else if (g_ref_gravity_axis == 1) world_up = Eigen::Vector3f(0, -1, 0);
        else                              world_up = Eigen::Vector3f(0, 0, 1);

        Eigen::Vector3f lidar_fwd_local = Eigen::Vector3f::UnitZ();
        Eigen::Vector3f lidar_fwd_world = Eigen::Quaternionf(imu_state.R) * lidar_fwd_local;

        Eigen::Vector3f fwd_proj = lidar_fwd_world - (lidar_fwd_world.dot(world_up)) * world_up;
        fwd_proj.normalize();

        Eigen::Vector3f foot_x = fwd_proj;
        Eigen::Vector3f foot_z = world_up;
        Eigen::Vector3f foot_y = foot_z.cross(foot_x);

        Eigen::Matrix3f foot_mat;
        foot_mat.col(0) = foot_x;
        foot_mat.col(1) = foot_y;
        foot_mat.col(2) = foot_z;

        Eigen::Quaternionf q_foot(foot_mat);
        q_foot.normalize();

        tf_footprint.transform.rotation.x = q_foot.x();
        tf_footprint.transform.rotation.y = q_foot.y();
        tf_footprint.transform.rotation.z = q_foot.z();
        tf_footprint.transform.rotation.w = q_foot.w();

        tf_broadcaster_->sendTransform(tf_footprint);
      }
    }
  }
}


// =================== Rear IMU Handler (store only, no Predict) ==============

void ROSWrapperDual::rearImuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  IMUData data;
  data.secs = this->now().seconds();

  V3 acc_raw(msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z);
  V3 gyr_raw(msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z);

  // Rotate to front-lidar frame for consistent storage
  M3 R_rearIMU_to_frontLidar = T_rear_to_front_.R_ * g_lidar_imu.R_.transpose();
  data.acc = R_rearIMU_to_frontLidar * acc_raw;
  data.gyr = R_rearIMU_to_frontLidar * gyr_raw;

  // Store in history for fusion; keep at most 20 samples (200ms @ 100Hz)
  rear_imu_history_.push_back(data);
  if (rear_imu_history_.size() > 20)
    rear_imu_history_.pop_front();
}


// =================== Rear LiDAR Handler =====================================

void ROSWrapperDual::rearLidarHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  LidarData lidar_data;

  switch (static_cast<LID_TYPE>(g_lidar_type))
  {
  case LID_TYPE::ROBOSENSE_AIRY:
  {
    bool has_ring = false;
    bool has_timestamp = false;
    for (const auto& field : msg->fields) {
      if (field.name == "ring") has_ring = true;
      if (field.name == "timestamp") has_timestamp = true;
    }

    if (!has_ring || !has_timestamp) return;

    pcl::PointCloud<robosenseM1_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    if (plsize == 0) return;

    double min_time = std::numeric_limits<double>::max();
    double max_time = std::numeric_limits<double>::lowest();
    for (int i = 0; i < plsize; ++i) {
      double ts = pl_orig.points[i].timestamp;
      if (ts < min_time) min_time = ts;
      if (ts > max_time) max_time = ts;
    }
    lidar_data.pc.reset(new pcl::PointCloud<PointXTZIT>());
    lidar_data.pc->reserve(plsize / g_filter_rate + 1);
    lidar_data.start_time = this->now().seconds();

    double offset_time = 0.0;

    for (int i = 0; i < plsize; i += g_filter_rate) {
      auto& pt = pl_orig.points[i];
      float ros_x = pt.x;
      float ros_y = pt.y;
      float ros_z = pt.z;
      if (!validPoint(ros_x, ros_y, ros_z)) continue;
      if (g_intensity_filter_en && pt.intensity < g_intensity_min) continue;

      // Store in rear frame; transform to front frame deferred to after
      // voxel downsampling in SuperLIO::stateProcess() to save compute.
      offset_time = pt.timestamp - min_time;
      lidar_data.pc->emplace_back(
          ros_x, ros_y, ros_z,
          pt.intensity, offset_time);
    }

    lidar_data.end_time = lidar_data.start_time + offset_time;
    lidar_data.frame_id = "__rear__";

    if (!lidar_data.pc->empty()) {
      lidar_buffer_.push_back(std::move(lidar_data));
    }
    break;
  }
  default:
    LOG(WARNING) << "[Dual] Unsupported lidar type for rear: "
                 << g_lidar_type << " (only Airy=9 supported)";
    break;
  }
}

// =================== Rear Cloud Publishers ==================================

bool ROSWrapperDual::getRearToFront(M3& R, V3& t) const
{
  R = T_rear_to_front_.R_;
  t = T_rear_to_front_.t_;
  return true;
}

void ROSWrapperDual::pub_cloud_world_rear(const CloudPtr& pc, double time)
{
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_world_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_rear_->publish(cloud);
}

void ROSWrapperDual::pub_cloud_body_rear(const CloudPtr& pc, double time)
{
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_imu_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_body_rear_->publish(cloud);
}

} // namespace LI2Sup
