/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;

  extern bool g_save_map;
  extern bool g_if_filter;
  extern std::string g_map_name;
  extern std::string g_save_map_dir;
  extern float g_map_ds_size;
  extern int   g_pcd_save_interval;
  extern std::string g_pcd_prefix;
  
  extern std::string g_imu_topic;
  extern std::string g_lidar_topic;
  
  /// QoS settings
  extern bool g_imu_qos_reliable;
  extern bool g_lidar_qos_reliable;

  extern int   g_lidar_type;       // 1: mid360, 2: hesai16, 3: velo16, 4: velo32, 5: vel_nclt, 6: ls16 
  extern float g_blind2;
  extern float g_maxrange2;
  extern int   g_filter_rate;
  extern int   g_filter_offset; // 余数偏移，用于下次采样的起始点
  extern bool  g_enable_filter_offset;
  extern bool  g_enable_downsample;
  extern float g_voxel_fliter_size;
  extern bool  g_intensity_filter_en;
  extern float g_intensity_min;

  extern int    g_imu_type;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;

  extern BASIC::SE3 g_lidar_imu;      // lidar in imu frame
  extern BASIC::SE3 g_odom_robo;      // lidar in robot frame
  extern BASIC::M3  g_lidar_robo_yaw; // lidar in robot frame rotation only yaw

  /// hash_map
  extern std::size_t g_ivox_capacity;
  extern float       g_ivox_resolution;
  
  /// kf
  extern int g_kf_type;            // 1: ESKF, 2: InESKF.
  extern int g_kf_max_iterations;
  extern bool g_kf_align_gravity;
  extern int g_ref_gravity_axis;   // 0: +X, 1: +Y, 2: -Z (default)
  extern double g_kf_quit_eps;

  /// submaps
  extern double g_submap_resolution;
  extern int    g_submap_capacity;
  
  /// output  
  extern bool g_2_robot;
  extern bool g_2_plan_env_world;
  extern bool g_2_plan_env_body;
  extern bool g_2_ml_map;
  extern bool g_visual_map;
  extern bool g_visual_dense;
  extern bool g_visual_map_body;
  extern bool g_visual_dense_body;
  extern int  g_pub_step;
  extern bool g_footprint_pub_en;
  extern std::string g_tf_base_footprint_frame;
  extern std::string g_world_frame;
  extern std::string g_imu_frame;

  /// map save service topic
  extern std::string g_map_save_service_topic;

  /// for planner
  extern bool g_planner_enable;

  /// lio only undistort mode
  extern bool g_lio_only_undistort;

  /// downsample only mode (highest priority)
  extern bool g_downsample_only;

  /// Define the hybrid residual formulation.
  enum ResidualType{
    PROB = 1,     // Probabilistic residual
    P2P  = 2,     // Point-to-plane residual
    MIX  = 3      // Hybrid residual (probabilistic + point-to-plane)
  };
  extern ResidualType g_residual_type;


  /// for relocation
  extern bool g_update_map;
  extern double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// for dynamic point removal
  extern bool g_dynamic_removal_enable;
  extern int  g_dynamic_removal_method;     // 0: Temporal, 1: Raycast
  extern float g_dynamic_removal_grid_size;
  extern int   g_dynamic_removal_min_neighbors;
  extern int   g_dynamic_removal_frame_window;
  extern int   g_dynamic_removal_raycast_min_hits;
  extern bool  g_dynamic_removal_isolated_removal;

  /// for SC-PGO offline processing output
  extern bool  g_sc_pgo_enable;
  extern float g_sc_pgo_keyframe_gap;
  extern float g_sc_pgo_keyframe_deg_gap;

  /// Single-core mode: disables all TBB parallelism in LIO pipeline,
   /// independent threads (OutputThread, SaveThread) remain unaffected.
   extern bool g_single_core;

   /// Fast tf: publish IMU-rate transforms (world->imu, world->base_footprint)
   extern bool g_fast_tf;

   /// Degeneracy detection + Tikhonov regularization on observation Hessian
   extern bool   g_degeneracy_detect_en;
   extern double g_degeneracy_threshold;
   extern double g_tikhonov_lambda;
  extern int    g_min_effect_pts;

   /// Adaptive observation weight: replace flat ×1000 with distance-dependent weight
   extern bool   g_adaptive_weight_en;
   extern double g_adaptive_weight_sigma;

   /// Max consecutive Observe-SKIP frames before resetting IMU bias
   extern int    g_bias_reset_skip_threshold;

   /// Share IVox map between front and rear lidar in dual mode
   extern bool   g_share_ivox;

}

#endif
