
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <sched.h>
#include <pthread.h>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>


using namespace BASIC;

namespace LI2Sup{

bool SuperLIO::set_realtime_priority(int priority)
{
  struct sched_param param;
  param.sched_priority = priority;
  
  int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (ret != 0) {
    LOG(WARNING) << "Failed to set SCHED_FIFO priority " << priority 
               << ", error: " << std::strerror(ret);
    return false;
  }
  
  LOG(INFO) << GREEN << " ---> [SuperLIO]: Set SCHED_FIFO priority to " << priority << RESET;
  return true;
}



inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  output_running_ = true;
  output_thread_ = std::thread(&SuperLIO::OutputThread, this);

  if(g_save_map){
    namespace fs = std::filesystem;
    
    std::string save_map_dir = g_save_map_dir;
    if (!save_map_dir.empty() && save_map_dir[0] != '/') {
      save_map_dir = g_root_dir + save_map_dir;
    }
    
    std::string pcd_folder = save_map_dir + "/PCD";
    
    if(fs::exists(pcd_folder)){
      int deleted_count = 0;
      for(const auto& entry : fs::directory_iterator(pcd_folder)){
        std::string filename = entry.path().filename().string();
        if(entry.path().extension() == ".pcd" &&
           filename.find(g_pcd_prefix + "scans_") != std::string::npos){
          try{
            fs::remove(entry.path());
            deleted_count++;
            LOG(INFO) << GREEN << " ---> Deleted old PCD fragment: " << filename << RESET;
          } catch(const std::exception& e){
            LOG(WARNING) << RED << " ---> Failed to delete " << filename 
                        << ": " << e.what() << RESET;
          }
        }
        if(entry.path().extension() == ".txt" &&
           filename.find(g_pcd_prefix + "scans_") != std::string::npos){
          try{
            fs::remove(entry.path());
            deleted_count++;
            LOG(INFO) << GREEN << " ---> Deleted old pose file: " << filename << RESET;
          } catch(const std::exception& e){
            LOG(WARNING) << RED << " ---> Failed to delete " << filename 
                        << ": " << e.what() << RESET;
          }
        }
      }
      if(deleted_count > 0){
        LOG(INFO) << YELLOW << " ---> Deleted " << deleted_count 
                  << " old PCD fragments with prefix '" << g_pcd_prefix << "'" << RESET;
      }
    }
    
    save_running_ = true;
    save_thread_ = std::thread(&SuperLIO::SaveThread, this);
  }

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}

SuperLIO::~SuperLIO(){
  output_running_ = false;
  output_cv_.notify_all();
  if(output_thread_.joinable()){
    output_thread_.join();
  }

  save_running_ = false;
  save_cv_.notify_all();
  if(save_thread_.joinable()){
    save_thread_.join();
  }
}


void SuperLIO::reinitLIO(){
  LOG(INFO) << YELLOW << " ---> [SuperLIO]: Reinitializing map (keeping KF pose)..." << RESET;

  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }

  LOG(INFO) << GREEN << " ---> [SuperLIO]: Map reinitialized, world pose preserved. pcd_index=" << pcd_index_ << RESET;
}


void SuperLIO::stateWaitKFInit()
{
  // downsample_only mode: skip KF and map initialization
  if(g_downsample_only){
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Downsample-only mode, skip KF and map init" << RESET;
    return;
  }
  
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  // downsample_only mode should not reach here, but add check for safety
  if(g_downsample_only){
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Downsample-only mode, skip map init" << RESET;
    return;
  }
  
  if(g_lio_only_undistort){
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Undistort-only mode, skip map init" << RESET;
    return;
  }
  if (map_init()) {
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

void SuperLIO::process(){
  static bool priority_set = false;
  if (!priority_set) {
    set_realtime_priority(98);
    priority_set = true;
  }
  
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }
  current_lidar_frame_ = measures_.lidar.frame_id;
  (this->*state_fn_)();
}


bool SuperLIO::kf_init(){
  for(auto& imu: measures_.imu){
    kf_init_imu_count_++;
    kf_init_mean_gyro_ += (imu.gyr - kf_init_mean_gyro_) / kf_init_imu_count_;
    kf_init_mean_acce_ += (imu.acc - kf_init_mean_acce_) / kf_init_imu_count_;
  }

  /// 100 Hz for 1 second.
  if(kf_init_imu_count_ < 50){
    return false;
  }

  V3 gravity = - kf_init_mean_acce_ * g_gravity_norm / kf_init_mean_acce_.norm();
  V3 ref_gravity;
  switch(g_ref_gravity_axis) {
    case 0:  ref_gravity = V3(g_gravity_norm, 0, 0); break;   // +X
    case 1:  ref_gravity = V3(0, g_gravity_norm, 0); break;   // +Y
    case 2:  
    default: ref_gravity = V3(0, 0, -g_gravity_norm); break;  // -Z (default)
  }
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  LOG(INFO) << GREEN << " ---> [SuperLIO]: Gravity Alignment Results:" << RESET;
  LOG(INFO) << GREEN << "      Mean Acceleration: [" << kf_init_mean_acce_.transpose() << "]" << RESET;
  LOG(INFO) << GREEN << "      Gravity Norm: " << g_gravity_norm << RESET;
  LOG(INFO) << GREEN << "      Measured Gravity: [" << gravity.transpose() << "]" << RESET;
  LOG(INFO) << GREEN << "      Reference Gravity: [" << ref_gravity.transpose() << "]" << RESET;
  LOG(INFO) << GREEN << "      Yaw Angle: " << yaw * 180.0 / M_PI << " degrees" << RESET;
  LOG(INFO) << GREEN << "      IMU Scale: " << g_gravity_norm / kf_init_mean_acce_.norm() << RESET;

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / kf_init_mean_acce_.norm();
  kf_->SetInitialConditions(options, kf_init_mean_gyro_, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  
  if(pending_save_cleanup_){
    pending_save_cleanup_ = false;
    saveMap();
    reinitLIO();
  }
  
  // pcd save-only mode: highest priority, skip all LIO/IMU
  if(g_pcd_save_mode.load()){
    PCDSaveOnly();
    return;
  }
  
  // downsample_only mode has highest priority
  if(g_downsample_only){
    if(g_time_eva){
      time_record_.Evaluate([this]() { DownSampleOnly(); }, "[DownSampleOnly]");
    }else{
      DownSampleOnly();
    }
    Output();
    caceData();
    return;
  }
  
  if(g_lio_only_undistort){
    if(g_time_eva){
      time_record_.Evaluate([this](){Propagation_Undistort();}, "[Undistort]");
      time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
      time_record_.Evaluate([this]() { Observe(); }, "[Observe]");
      time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
    }else{
      Propagation_Undistort();
      DownSample();
      Observe();
      UpdateMap();
    }
    Output();
    caceData();
    return;
  }
  if(g_time_eva){
    time_record_.Evaluate([this](){Propagation_Undistort();}, "[Undistort]");
    time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
    time_record_.Evaluate([this]() { Observe(); }, "[Observe]");
    time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
  }else{
    Propagation_Undistort();
    DownSample();
    Observe();
    UpdateMap();
  }
  Output();
  caceData();
}


void SuperLIO::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if(g_lio_only_undistort){
    if(g_if_filter){
      *world_pc_ = *ds_undistort_;
    }else{
      *world_pc_ = *scan_undistort_full_;
    }
  }else{
    if(g_if_filter){
      pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
    }else{
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
    }
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
  }

  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    std::string save_map_dir = g_save_map_dir;
    if (!save_map_dir.empty() && save_map_dir[0] != '/') {
      save_map_dir = g_root_dir + save_map_dir;
    }
    std::string cmd = "rm -rf " + save_map_dir + "/PCD";
    [[maybe_unused]] int res;
    res = system(cmd.c_str());
    cmd = "mkdir -p " + save_map_dir + "/PCD";
    res = system(cmd.c_str());
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    
    SaveData save_data;
    save_data.cloud_to_save.reset(new PointCloudType(*point_map_));
    save_data.pcd_index = pcd_index_;
    save_data.timestamp = state.timestamp;
    save_data.position = state.p.cast<float>();
    save_data.orientation = BASIC::Quat(state.R.R_.cast<float>());
    
    {
      std::lock_guard<std::mutex> lock(save_mutex_);
      if(save_queue_.size() > 3){
        save_queue_.pop();
      }
      save_queue_.push(std::move(save_data));
    }
    save_cv_.notify_one();
    
    point_map_->clear();
    scan_wait_num = 0;
  }
}

void SuperLIO::PCDSaveOnly(){
  DownSampleOnly();

  if(g_save_map && !ds_undistort_->empty()){
    pcd_index_++;

    SaveData save_data;
    save_data.cloud_to_save.reset(new PointCloudType(*ds_undistort_));
    save_data.pcd_index = pcd_index_;
    save_data.timestamp = measures_.lidar.start_time;
    save_data.position = V3(0, 0, 0);
    save_data.orientation = Quat(1, 0, 0, 0);

    {
      std::lock_guard<std::mutex> lock(save_mutex_);
      if(save_queue_.size() > 3){
        save_queue_.pop();
      }
      save_queue_.push(std::move(save_data));
    }
    save_cv_.notify_one();
  }

  pcd_save_frame_count_++;
  if(g_pcd_save_frames > 0 && pcd_save_frame_count_ >= g_pcd_save_frames){
    LOG(INFO) << YELLOW << " ---> [SuperLIO]: PCD save complete (" << pcd_save_frame_count_
              << " frames), auto-exiting save mode..." << RESET;
    pcd_save_frame_count_ = 0;
    g_pcd_save_mode.store(false);
    pending_save_cleanup_ = true;
  }
}

void SuperLIO::SaveThread(){
  while(save_running_){
    SaveData data;
    {
      std::unique_lock<std::mutex> lock(save_mutex_);
      save_cv_.wait(lock, [this]{
        return !save_queue_.empty() || !save_running_;
      });
      
      if(!save_running_ && save_queue_.empty()){
        break;
      }
      
      if(save_queue_.empty()){
        continue;
      }
      
      data = std::move(save_queue_.front());
      save_queue_.pop();
      save_cv_.notify_one();
    }
    
    if(data.cloud_to_save && !data.cloud_to_save->empty()){
      std::string save_map_dir = g_save_map_dir;
      if (!save_map_dir.empty() && save_map_dir[0] != '/') {
        save_map_dir = g_root_dir + save_map_dir;
      }
      std::string map_name(std::string(save_map_dir + "/PCD/" + g_pcd_prefix + "scans_") + std::to_string(data.pcd_index) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/" << g_pcd_prefix << "scans_" << data.pcd_index 
                << "  size:  " << data.cloud_to_save->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *data.cloud_to_save);
      
      std::string odom_name(std::string(save_map_dir + "/PCD/" + g_pcd_prefix + "scans_") + std::to_string(data.pcd_index) +
                                 std::string(".txt"));
      std::ofstream odom_file(odom_name);
      if(odom_file.is_open()){
        odom_file << std::fixed << std::setprecision(6);
        odom_file << data.timestamp << " "
                  << data.position.x() << " " << data.position.y() << " " << data.position.z() << " "
                  << data.orientation.x() << " " << data.orientation.y() << " " 
                  << data.orientation.z() << " " << data.orientation.w() << std::endl;
        odom_file.close();
      }
    }
  }
}


void SuperLIO::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string save_map_dir = g_save_map_dir;
  if (!save_map_dir.empty() && save_map_dir[0] != '/') {
    save_map_dir = g_root_dir + save_map_dir;
  }

  std::string pcd_folder = save_map_dir + "/PCD";
  std::string output_map_name;
  if(g_dynamic_removal_enable){
    size_t dot_pos = g_map_name.find_last_of('.');
    if(dot_pos != std::string::npos){
      output_map_name = save_map_dir + "/" + g_map_name.substr(0, dot_pos) + "_ori" + g_map_name.substr(dot_pos);
    } else {
      output_map_name = save_map_dir + "/" + g_map_name + "_ori";
    }
  } else {
    output_map_name = save_map_dir + "/" + g_map_name;
  }

  LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

  PointCloudType::Ptr merged_map(new PointCloudType());

  int count = 0;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
      entry.path().filename().string().find(g_pcd_prefix + "scans_") != std::string::npos) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0) {
        *merged_map += *tmp_cloud;
        count++;
        // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string() 
        //           << "   size: " << tmp_cloud->size() << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
      }
    }
  }

  LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

  PointCloudType filtered_map;

  if(g_if_filter){
    LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    
    voxel_filter.setInputCloud(merged_map);
    voxel_filter.filter(filtered_map);
  }else{
    LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
    filtered_map = *merged_map;
  }
  
  if (filtered_map.size() > 0) {
    filtered_map.width = filtered_map.size();
    filtered_map.height = 1;
    filtered_map.is_dense = false;
  }

  pcl::io::savePCDFileBinary(output_map_name, filtered_map);

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;
  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      
      auto state = kf_->GetNavState();
      SaveData save_data;
      save_data.cloud_to_save.reset(new PointCloudType(*point_map_));
      save_data.pcd_index = pcd_index_;
      save_data.timestamp = state.timestamp;
      save_data.position = state.p.cast<float>();
      save_data.orientation = BASIC::Quat(state.R.R_.cast<float>());
      
      {
        std::lock_guard<std::mutex> lock(save_mutex_);
        save_queue_.push(std::move(save_data));
      }
      save_cv_.notify_one();
      
      point_map_->clear();
    }
    
    {
      std::unique_lock<std::mutex> lock(save_mutex_);
      save_cv_.wait(lock, [this]{
        return save_queue_.empty();
      });
    }
    
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
    
    if(g_dynamic_removal_enable){
      LOG(INFO) << YELLOW << " ---> Running dynamic point removal ... " << RESET;
      std::string save_map_dir = g_save_map_dir;
      if (!save_map_dir.empty() && save_map_dir[0] != '/') {
        save_map_dir = g_root_dir + save_map_dir;
      }
      std::string pcd_folder = save_map_dir + "/PCD";
      std::string filtered_output = save_map_dir + "/" + g_map_name;
      
      std::stringstream cmd;
      cmd << "taskset -c 0,1,2,3,4,5,6 ros2 run super_lio dynamic_remove_node"
          << " --input_dir " << pcd_folder
          << " --output_file " << filtered_output
          << " --grid_size " << g_dynamic_removal_grid_size
          << " --min_neighbors " << g_dynamic_removal_min_neighbors
          << " --method " << g_dynamic_removal_method;
      
      if(!g_pcd_prefix.empty()) {
        cmd << " --scans_prefix " << g_pcd_prefix;
      }
      
      if(g_dynamic_removal_method == 0) {
        cmd << " --frame_window " << g_dynamic_removal_frame_window;
      } else {
        cmd << " --raycast_min_hits " << g_dynamic_removal_raycast_min_hits;
      }
      
      if(!g_dynamic_removal_isolated_removal) {
        cmd << " --disable_isolated";
      }
      
      LOG(INFO) << YELLOW << " ---> Executing: " << cmd.str() << RESET;
      int ret = system(cmd.str().c_str());
      if(ret == 0) {
        LOG(INFO) << GREEN << " ---> Dynamic point removal success. Output: " << filtered_output << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Dynamic point removal failed with code: " << ret << RESET;
      }
    }
    
    LOG(INFO) << YELLOW << " ---> Process cace map ... " << RESET;
    ProcessCaceMap();
    LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string save_map_dir = g_save_map_dir;
    if (!save_map_dir.empty() && save_map_dir[0] != '/') {
      save_map_dir = g_root_dir + save_map_dir;
    }
    std::string map_name = save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
  }
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}


void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;
  auto& raw_pc = measures_.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize); 

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t; V3 p_h, v_h, acc_t, w_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.offset_time;
      if (query_time > propagate_states_.back().time) {
        pt_full.x = pt.x;
        pt_full.y = pt.y;
        pt_full.z = pt.z;
        continue;
      }
      auto match_iter = propagate_states_.begin();
      for (auto iter = propagate_states_.begin(); iter != propagate_states_.end(); ++iter) {
        auto next_iter = std::next(iter);
        if (iter->time < query_time && next_iter->time >= query_time) {
          match_iter = iter;
          break;
        }
      }
      auto match_iter_n = std::next(match_iter);
      double dt = match_iter_n->time - match_iter->time;
      double s = (query_time - match_iter->time) / dt;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      w_t = match_iter_n->w;
      M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
      V3 t_ei(p_h + v_h * dt + 0.5 * acc_t * dt * dt - T_end_t);
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = R_inv * (R_i * raw + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


void SuperLIO::DownSampleOnly(){
  // Directly process raw point cloud without IMU propagation
  // Apply filter_rate, intensity_filter, and voxel_filter
  
  scan_undistort_full_->clear();
  ds_undistort_->clear();
  
  auto& raw_pc = measures_.lidar.pc;
  std::size_t ptsize = raw_pc->size();
  
  // Apply filter_rate and intensity_filter
  for(std::size_t i = 0; i < ptsize; i += g_filter_rate){
    const auto& pt = raw_pc->points[i];
    
    // Apply intensity filter if enabled
    if(g_intensity_filter_en && pt.intensity < g_intensity_min){
      continue;
    }
    
    // Apply range filter
    double dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if(dis > g_blind2 && dis < g_maxrange2){
      pcl::PointXYZI pt_out;
      pt_out.x = pt.x;
      pt_out.y = pt.y;
      pt_out.z = pt.z;
      pt_out.intensity = pt.intensity;
      scan_undistort_full_->push_back(pt_out);
    }
  }
  
  // Apply voxel grid filter if enabled
  if(g_enable_downsample && !scan_undistort_full_->empty()){
    voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
    voxel_grid_fliter_.filter(ds_undistort_);
  }else{
    *ds_undistort_ = *scan_undistort_full_;
  }
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};


void SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();
  
  static std::vector<float> _lengths;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              effect_mask_[idx] = false;
              effect_knn_mask_[idx] = false;
              continue;
            }
            effect_knn_mask_[idx] = true;
            effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
          }

          if(!effect_mask_[idx]) continue;

          auto& abcd = abcd_vec_[idx];
          scalar error;
          effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
          if(!effect_mask_[idx]) continue;
          
          {
            V3d normvec(abcd[0], abcd[1], abcd[2]);
            V3d nb = R_transpose * normvec;
            V3d point_body_d = point_body.cast<double>();
            V6d J;
            J.head<3>() = point_body_d.cross(nb);
            J.tail<3>() = normvec;
      
            local_acc.HTVH += J * 1000 * J.transpose();
            local_acc.HTVr -= J * 1000 * error;
          }
        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  });

  frame_num_++;
}


void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  
  OutputData output_data;
  output_data.state = state;
  output_data.is_undistort_only = g_lio_only_undistort || g_downsample_only;
  output_data.lidar_frame = current_lidar_frame_;

  // downsample_only mode: output without transformation
  if(g_downsample_only){
    output_data.state.timestamp = measures_.lidar.start_time;
    if(g_visual_map){
      static int count = -1;
      count++;
      if(count % g_pub_step == 0){
        count = 0;
        output_data.world_pc.reset(new PointCloudType());
        if(g_visual_dense){
          *output_data.world_pc = *scan_undistort_full_;
        }else{
          *output_data.world_pc = *ds_undistort_;
        }
        output_data.has_world_pc = true;
      }
    }

    if(g_visual_map_body){
      static int count_body = -1;
      count_body++;
      if(count_body % g_pub_step == 0){
        count_body = 0;
        output_data.body_pc.reset(new PointCloudType());
        if(g_visual_dense_body){
          *output_data.body_pc = *scan_undistort_full_;
        }else{
          *output_data.body_pc = *ds_undistort_;
        }
        output_data.has_body_pc = true;
      }
    }
  }
  else if(g_lio_only_undistort){
    if(g_visual_map){
      static int count = -1;
      count++;
      if(count % g_pub_step == 0){
        count = 0;
        output_data.world_pc.reset(new PointCloudType());
        if(g_visual_dense){
          *output_data.world_pc = *scan_undistort_full_;
        }else{
          *output_data.world_pc = *ds_undistort_;
        }
        output_data.has_world_pc = true;
      }
    }

    if(g_visual_map_body){
      static int count_body = -1;
      count_body++;
      if(count_body % g_pub_step == 0){
        count_body = 0;
        output_data.body_pc.reset(new PointCloudType());
        if(g_visual_dense_body){
          *output_data.body_pc = *scan_undistort_full_;
        }else{
          *output_data.body_pc = *ds_undistort_;
        }
        output_data.has_body_pc = true;
      }
    }
  }else{
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
    transformation.block<3, 1>(0, 3) = state.p.cast<float>();

    if(g_visual_map){
      static int count = -1;
      count++;
      if(count % g_pub_step == 0){
        count = 0;
        output_data.world_pc.reset(new PointCloudType());
        if(g_visual_dense){
          pcl::transformPointCloud(*scan_undistort_full_, *output_data.world_pc, transformation);
        }else{
          pcl::transformPointCloud(*ds_undistort_, *output_data.world_pc, transformation);
        }
        output_data.has_world_pc = true;
      }
    }

    if(g_visual_map_body){
      static int count_body = -1;
      count_body++;
      if(count_body % g_pub_step == 0){
        count_body = 0;
        output_data.body_pc.reset(new PointCloudType());
        if(g_visual_dense_body){
          *output_data.body_pc = *scan_undistort_full_;
        }else{
          *output_data.body_pc = *ds_undistort_;
        }
        output_data.has_body_pc = true;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if(output_queue_.size() > 5){
      output_queue_.pop();
    }
    output_queue_.push(std::move(output_data));
  }
  output_cv_.notify_one();
}

void SuperLIO::OutputThread(){
  while(output_running_){
    OutputData data;
    {
      std::unique_lock<std::mutex> lock(output_mutex_);
      output_cv_.wait(lock, [this]{
        return !output_queue_.empty() || !output_running_;
      });
      
      if(!output_running_ && output_queue_.empty()){
        break;
      }
      
      if(output_queue_.empty()){
        continue;
      }
      
      data = std::move(output_queue_.front());
      output_queue_.pop();
    }
    
    if(!data.is_undistort_only){
      data_wrapper_->pub_odom(data.state);
    }
    
    if(data.has_world_pc && data.world_pc){
      if(data.is_undistort_only){
        data_wrapper_->pub_cloud_world_undistort_only(data.world_pc, data.state.timestamp, data.lidar_frame);
      }else{
        data_wrapper_->pub_cloud_world(data.world_pc, data.state.timestamp);
      }
    }
    
    if(data.has_body_pc && data.body_pc){
      if(data.is_undistort_only){
        data_wrapper_->pub_cloud_undistort_only(data.body_pc, data.state.timestamp, data.lidar_frame);
      }else{
        data_wrapper_->pub_cloud_body(data.body_pc, data.state.timestamp);
      }
    }
  }
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}

} // namespace END.