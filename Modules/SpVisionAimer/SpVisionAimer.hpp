#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp vision aimer and shooter
constructor_args:
  - cfg:
      to_now: false
      enable_csv_log: true
      csv_log_dir: logs/analysis
      aimer:
        yaw_offset_deg: -1
        left_yaw_offset_deg: -1
        right_yaw_offset_deg: -1
        enable_dual_yaw_offset: false
        pitch_offset_deg: -1.4
        comming_angle_deg: 60
        leaving_angle_deg: 20
        high_speed_delay_time: 0.03
        low_speed_delay_time: 0.015
        decision_speed: 8
      shooter:
        first_tolerance_deg: 3
        second_tolerance_deg: 2
        judge_distance: 2
        auto_fire: true
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <fstream>
#include <string>

#include "Modules/SpVisionCommon/SpVisionMessages.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "mutex.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tools/math_tools.hpp"

class SpVisionAimer : public LibXR::Application
{
public:
  struct AimerConfig
  {
    double yaw_offset_deg = -1.0;
    double left_yaw_offset_deg = -1.0;
    double right_yaw_offset_deg = -1.0;
    bool enable_dual_yaw_offset = false;
    double pitch_offset_deg = -1.4;
    double comming_angle_deg = 60.0;
    double leaving_angle_deg = 20.0;
    double high_speed_delay_time = 0.03;
    double low_speed_delay_time = 0.015;
    double decision_speed = 8.0;
  };

  struct ShooterConfig
  {
    double first_tolerance_deg = 3.0;
    double second_tolerance_deg = 2.0;
    double judge_distance = 2.0;
    bool auto_fire = true;
  };

  struct Config
  {
    bool to_now = false;
    AimerConfig aimer{};
    ShooterConfig shooter{};
    bool enable_csv_log = true;
    std::string csv_log_dir = "logs/analysis";
  };

  SpVisionAimer(LibXR::HardwareContainer & hw, LibXR::ApplicationManager & app, Config cfg);

  void OnMonitor() override {}

private:
  static std::string WriteAimerConfig(const AimerConfig & cfg, const void * tag);
  static std::string WriteShooterConfig(const ShooterConfig & cfg, const void * tag);
  void InitCsvLog();
  void AppendCsvLog(
    const sp_xr::TrackFrame & tracks, const sp_xr::GimbalState & state, const io::Command & command,
    const Eigen::Vector3d & gimbal_ypr);
  void OnGimbalState(const sp_xr::GimbalState & state);
  void OnTracks(const sp_xr::TrackFrame & tracks);

  Config cfg_;
  std::string aimer_config_path_;
  std::string shooter_config_path_;
  auto_aim::Aimer aimer_;
  auto_aim::Shooter shooter_;

  LibXR::Topic::Domain vision_domain_{"sp_vision"};
  LibXR::Topic command_topic_{"aim_command", sizeof(sp_xr::AimCommand), &vision_domain_};
  LibXR::Topic aim_share_topic_{"aim_share", sizeof(sp_xr::AimShareData)};

  LibXR::Mutex state_lock_;
  sp_xr::GimbalState latest_state_;
  std::atomic<bool> has_state_{false};

  LibXR::Mutex csv_lock_;
  std::ofstream csv_log_file_;
  std::string csv_log_path_;
  bool csv_header_written_{false};
  bool has_csv_t0_{false};
  std::chrono::steady_clock::time_point csv_t0_{};
  uint64_t csv_rows_{0};
};
