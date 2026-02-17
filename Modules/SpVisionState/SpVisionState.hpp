#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp vision gimbal state source
constructor_args:
  - cfg:
      publish_interval_ms: 2
      mock_mode: true
      mock_bullet_speed: 30.0
      use_topic_quaternion: false
      quaternion_topic_name: ahrs_quaternion
      cboard:
        can_interface: can0
        quaternion_canid: 0x100
        bullet_speed_canid: 0x101
        send_canid: 0xff
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <memory>
#include <string>

#include "Modules/SpVisionCommon/SpVisionMessages.hpp"
#include "app_framework.hpp"
#include "io/cboard.hpp"
#include "libxr.hpp"
#include "mutex.hpp"
#include "thread.hpp"

class SpVisionState : public LibXR::Application
{
public:
  struct CBoardConfig
  {
    std::string can_interface = "can0";
    int quaternion_canid = 0x100;
    int bullet_speed_canid = 0x101;
    int send_canid = 0xff;
  };

  struct Config
  {
    int publish_interval_ms = 2;
    bool mock_mode = true;
    double mock_bullet_speed = 30.0;
    bool use_topic_quaternion = false;
    std::string quaternion_topic_name = "ahrs_quaternion";
    CBoardConfig cboard{};
  };

  SpVisionState(LibXR::HardwareContainer & hw, LibXR::ApplicationManager & app, Config cfg);

  ~SpVisionState() override;

  void OnMonitor() override {}

private:
  static void PublishThreadFun(SpVisionState * self);
  void OnQuaternion(const Eigen::Quaternionf & q);

  static std::string WriteCBoardConfig(const CBoardConfig & cfg, const void * tag);

  Config cfg_;
  std::string cboard_config_path_;
  std::unique_ptr<io::CBoard> cboard_;
  LibXR::Thread publish_thread_;
  std::atomic<bool> running_{false};
  LibXR::Mutex quat_lock_;
  Eigen::Quaternionf latest_topic_q_{1.0f, 0.0f, 0.0f, 0.0f};
  std::atomic<bool> has_topic_q_{false};
  std::atomic<bool> q_first_log_printed_{false};

  LibXR::Topic::Domain vision_domain_{"sp_vision"};
  LibXR::Topic gimbal_state_topic_{"gimbal_state", sizeof(sp_xr::GimbalState), &vision_domain_};
};
