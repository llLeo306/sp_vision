#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp vision tracker
constructor_args:
  - cfg:
      debug_view: true
      tracker:
        enemy_color: blue
        min_detect_count: 5
        max_temp_lost_count: 15
        outpost_max_temp_lost_count: 75
        max_match_distance: 0.15
        max_match_yaw_diff: 1.0
        outpost_extra_match_yaw_diff: 0.7
      solver:
        camera_matrix: [1818.3669452465165, 0, 751.06226574703498, 0, 1822.494494078506, 530.43671556112133, 0, 0, 1]
        distort_coeffs: [-0.077944626599568856, 0.15447826031486889, -0.0025714394278524674, 0.00083016311301273629, 0]
        R_gimbal2imubody: [-1, 0, 0, 0, -1, 0, 0, 0, 1]
        R_camera2gimbal: [-0.0083195760046954614, 0.010498791137270739, 0.99991027599468041, -0.99960756138647755, -0.026835747568381807, -0.0080352891314148939, 0.026748978935305992, -0.99958472279097077, 0.010717933047771133]
        t_camera2gimbal: [0.094969301833534511, 0.095006290298006682, 0.050987066291756609]
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <chrono>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "Modules/SpVisionCommon/SpVisionMessages.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "mutex.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"

class SpVisionTracker : public LibXR::Application
{
public:
  struct TrackerConfig
  {
    std::string enemy_color = "blue";
    int min_detect_count = 5;
    int max_temp_lost_count = 15;
    int outpost_max_temp_lost_count = 75;
    double max_match_distance = 0.15;
    double max_match_yaw_diff = 1.0;
    double outpost_extra_match_yaw_diff = 0.7;
  };

  struct SolverConfig
  {
    std::vector<double> camera_matrix{
      1818.3669452465165, 0.0, 751.06226574703498, 0.0, 1822.494494078506, 530.43671556112133,
      0.0, 0.0, 1.0};
    std::vector<double> distort_coeffs{
      -0.077944626599568856, 0.15447826031486889, -0.0025714394278524674,
      0.00083016311301273629, 0.0};
    std::vector<double> R_gimbal2imubody{-1, 0, 0, 0, -1, 0, 0, 0, 1};
    std::vector<double> R_camera2gimbal{
      -0.0083195760046954614, 0.010498791137270739, 0.99991027599468041,
      -0.99960756138647755, -0.026835747568381807, -0.0080352891314148939,
      0.026748978935305992, -0.99958472279097077, 0.010717933047771133};
    std::vector<double> t_camera2gimbal{
      0.094969301833534511, 0.095006290298006682, 0.050987066291756609};
  };

  struct Config
  {
    bool debug_view = true;
    bool enable_outpost_debug_log = true;
    std::string outpost_debug_log_dir = "logs/analysis";
    bool save_debug_video = true;
    std::string debug_video_path = "logs/analysis/reprojection_debug.avi";
    double debug_video_fps = 30.0;
    TrackerConfig tracker{};
    SolverConfig solver{};
    bool save_outpost_bad_frames = true;
    std::string outpost_bad_frame_dir = "logs/analysis/bad_frames";
    double outpost_bad_pre_reproj_threshold = 80.0;
    double outpost_bad_match_reproj_threshold = 24.0;
    int outpost_bad_min_interval = 8;
    bool save_outpost_sample_frames = true;
    std::string outpost_sample_frame_dir = "logs/analysis/outpost_100ms";
    int outpost_sample_interval_ms = 100;
  };

  SpVisionTracker(LibXR::HardwareContainer & hw, LibXR::ApplicationManager & app, Config cfg);

  void OnMonitor() override {}

private:
  static std::string WriteSolverConfig(const SolverConfig & cfg, const void * tag);
  static std::string WriteTrackerConfig(const TrackerConfig & cfg, const void * tag);
  void OnImage(const sp_xr::ImageFrame & frame);
  void OnAimCommand(const sp_xr::AimCommand & command);
  void OnGimbalState(const sp_xr::GimbalState & state);
  void OnArmors(const sp_xr::ArmorFrame & armors);
  void ShowReprojection(
    const sp_xr::ArmorFrame & armors_msg, const std::list<auto_aim::Target> & targets,
    const std::list<auto_aim::Armor> & solved_armors);
  void InitDebugVideoWriter(const cv::Mat & frame);
  void InitOutpostCsvLog();
  void AppendOutpostCsvLog(
    const sp_xr::ArmorFrame & armors_msg, const std::list<auto_aim::Target> & targets,
    double min_reproj_err_px, double min_center_err_px, double min_bbox_iou, double min_center_dy_px,
    double obs_z_best, double pred_z_best, double set_iou_norm, double set_iou_visible,
    double set_iou_match_mean, double set_coverage, double set_visible_recall,
    int set_pred_count, int set_obs_count, int set_match_count);

  Config cfg_;
  std::string solver_config_path_;
  std::string tracker_config_path_;
  auto_aim::Solver solver_;
  auto_aim::Tracker tracker_;

  LibXR::Topic::Domain vision_domain_{"sp_vision"};
  LibXR::Topic track_topic_{"tracked_targets", sizeof(sp_xr::TrackFrame), &vision_domain_};

  LibXR::Mutex state_lock_;
  sp_xr::GimbalState latest_state_;
  std::atomic<bool> has_state_{false};

  LibXR::Mutex image_lock_;
  cv::Mat latest_image_;
  uint64_t latest_image_seq_{0};
  std::atomic<bool> has_image_{false};

  LibXR::Mutex aim_lock_;
  sp_xr::AimCommand latest_aim_command_{};
  std::atomic<bool> has_aim_command_{false};

  std::ofstream outpost_csv_file_;
  std::string outpost_csv_path_;
  bool outpost_csv_header_written_{false};
  bool has_outpost_t0_{false};
  std::chrono::steady_clock::time_point outpost_t0_{};
  uint64_t outpost_rows_{0};

  cv::VideoWriter debug_video_writer_;
  bool debug_video_ready_{false};
  uint64_t debug_video_frames_{0};
  uint64_t last_bad_frame_seq_{0};
  uint64_t bad_frames_saved_{0};
  bool has_sample_frame_ts_{false};
  std::chrono::steady_clock::time_point last_sample_frame_ts_{};
  uint64_t sample_frames_saved_{0};
};
