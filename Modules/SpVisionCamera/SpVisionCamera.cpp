#include "SpVisionCamera.hpp"

#include <pthread.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "logger.hpp"

namespace
{
std::string WriteCameraConfig(const SpVisionCamera::CameraConfig & cfg, const void * tag)
{
  const auto dir = std::filesystem::temp_directory_path() / "sp_vision_xr_cfg";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  std::ostringstream name;
  name << "camera_" << reinterpret_cast<uintptr_t>(tag) << ".yaml";
  const auto path = dir / name.str();

  std::ofstream ofs(path);
  ofs << "camera_name: \"" << cfg.camera_name << "\"\n";
  ofs << "exposure_ms: " << cfg.exposure_ms << "\n";
  ofs << "gamma: " << cfg.gamma << "\n";
  ofs << "gain: " << cfg.gain << "\n";
  ofs << "vid_pid: \"" << cfg.vid_pid << "\"\n";
  return path.string();
}
}  // namespace

SpVisionCamera::SpVisionCamera(
  LibXR::HardwareContainer &, LibXR::ApplicationManager & app, Config cfg)
: cfg_(std::move(cfg))
{
  if (!cfg_.use_video) {
    camera_config_path_ = WriteCameraConfig(cfg_.camera, this);
    camera_ = std::make_unique<io::Camera>(camera_config_path_);
  }

  running_.store(true);
  capture_thread_.Create(
    this, SpVisionCamera::CaptureThreadFun, "sp_cam", 1024 * 1024,
    LibXR::Thread::Priority::REALTIME);

  app.Register(*this);
  XR_LOG_PASS("[SpVisionCamera] initialized. source=%s", cfg_.use_video ? "video" : "camera");
}

SpVisionCamera::~SpVisionCamera()
{
  running_.store(false);
  pthread_join(static_cast<LibXR::libxr_thread_handle>(capture_thread_), nullptr);
}

void SpVisionCamera::CaptureThreadFun(SpVisionCamera * self)
{
  cv::VideoCapture cap;
  if (self->cfg_.use_video) {
    cap.open(self->cfg_.video_path);
    if (!cap.isOpened()) {
      XR_LOG_ERROR("[SpVisionCamera] failed to open video: %s", self->cfg_.video_path.c_str());
      self->running_.store(false);
      return;
    }
    XR_LOG_PASS("[SpVisionCamera] video opened: %s", self->cfg_.video_path.c_str());
  }

  bool first_frame_logged = false;
  while (self->running_.load()) {
    sp_xr::ImageFrame frame;
    if (self->cfg_.use_video) {
      if (!cap.read(frame.img) || frame.img.empty()) {
        if (!self->cfg_.loop_video) {
          XR_LOG_WARN("[SpVisionCamera] video ended.");
          self->running_.store(false);
          break;
        }
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        XR_LOG_WARN("[SpVisionCamera] video loop restart.");
        continue;
      }
      frame.timestamp = std::chrono::steady_clock::now();
    } else {
      self->camera_->read(frame.img, frame.timestamp);
    }

    frame.seq = ++self->seq_;
    self->image_topic_.Publish(frame);

    if (!first_frame_logged) {
      XR_LOG_PASS(
        "[SpVisionCamera] first frame published. seq=%llu size=%dx%d",
        static_cast<unsigned long long>(frame.seq), frame.img.cols, frame.img.rows);
      first_frame_logged = true;
    }

    if (self->cfg_.preview) {
      cv::imshow("sp_vision_image_raw", frame.img);
      cv::waitKey(1);
    }

    if (self->cfg_.publish_interval_ms > 0) {
      LibXR::Thread::Sleep(static_cast<uint32_t>(self->cfg_.publish_interval_ms));
    }
  }
}
