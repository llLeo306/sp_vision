#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp vision camera publisher
constructor_args:
  - cfg:
      camera:
        camera_name: hikrobot
        exposure_ms: 2.0
        gamma: 1.0
        gain: 16.0
        vid_pid: "2bdf:0001"
      use_video: true
      video_path: Modules/SpVisionCommon/assets/demo/demo.avi
      loop_video: true
      preview: false
      publish_interval_ms: 0
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "Modules/SpVisionCommon/SpVisionMessages.hpp"
#include "app_framework.hpp"
#include "io/camera.hpp"
#include "libxr.hpp"
#include "thread.hpp"

class SpVisionCamera : public LibXR::Application
{
public:
  struct CameraConfig
  {
    std::string camera_name = "hikrobot";
    double exposure_ms = 2.0;
    double gamma = 1.0;
    double gain = 16.0;
    std::string vid_pid = "2bdf:0001";
  };

  struct Config
  {
    CameraConfig camera{};
    bool use_video = true;
    std::string video_path = "Modules/SpVisionCommon/assets/demo/demo.avi";
    bool loop_video = true;
    bool preview = false;
    int publish_interval_ms = 0;
  };

  SpVisionCamera(LibXR::HardwareContainer & hw, LibXR::ApplicationManager & app, Config cfg);

  ~SpVisionCamera() override;

  void OnMonitor() override {}

private:
  static void CaptureThreadFun(SpVisionCamera * self);

  std::unique_ptr<io::Camera> camera_;
  std::string camera_config_path_;
  Config cfg_;
  LibXR::Thread capture_thread_;
  std::atomic<bool> running_{false};
  uint64_t seq_{0};

  LibXR::Topic::Domain vision_domain_{"sp_vision"};
  LibXR::Topic image_topic_{"image_raw", sizeof(sp_xr::ImageFrame), &vision_domain_};
};
