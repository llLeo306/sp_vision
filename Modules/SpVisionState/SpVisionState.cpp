#include "SpVisionState.hpp"

#include <pthread.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

#include "logger.hpp"

using namespace std::chrono_literals;

std::string SpVisionState::WriteCBoardConfig(const CBoardConfig & cfg, const void * tag)
{
  const auto dir = std::filesystem::temp_directory_path() / "sp_vision_xr_cfg";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  std::ostringstream name;
  name << "cboard_" << reinterpret_cast<uintptr_t>(tag) << ".yaml";
  const auto path = dir / name.str();

  std::ofstream ofs(path);
  ofs << "can_interface: \"" << cfg.can_interface << "\"\n";
  ofs << "quaternion_canid: " << cfg.quaternion_canid << "\n";
  ofs << "bullet_speed_canid: " << cfg.bullet_speed_canid << "\n";
  ofs << "send_canid: " << cfg.send_canid << "\n";
  return path.string();
}

SpVisionState::SpVisionState(
  LibXR::HardwareContainer &, LibXR::ApplicationManager & app, Config cfg)
: cfg_(std::move(cfg))
{
  if (cfg_.use_topic_quaternion) {
    LibXR::Topic q_topic(LibXR::Topic::FindOrCreate<Eigen::Quaternionf>(
      cfg_.quaternion_topic_name.c_str()));
    auto q_cb = LibXR::Topic::Callback::Create(
      [](bool, SpVisionState * self, LibXR::RawData & data) {
        auto * q = reinterpret_cast<Eigen::Quaternionf *>(data.addr_);
        self->OnQuaternion(*q);
      },
      this);
    q_topic.RegisterCallback(q_cb);
  }

  if (!cfg_.mock_mode) {
    cboard_config_path_ = WriteCBoardConfig(cfg_.cboard, this);
    cboard_ = std::make_unique<io::CBoard>(cboard_config_path_);
  }

  running_.store(true);
  publish_thread_.Create(
    this, PublishThreadFun, "sp_state", 256 * 1024, LibXR::Thread::Priority::REALTIME);

  app.Register(*this);
  XR_LOG_PASS("[SpVisionState] initialized. mode=%s", cfg_.mock_mode ? "mock" : "cboard");
}

SpVisionState::~SpVisionState()
{
  running_.store(false);
  pthread_join(static_cast<LibXR::libxr_thread_handle>(publish_thread_), nullptr);
}

void SpVisionState::PublishThreadFun(SpVisionState * self)
{
  bool first_publish_logged = false;
  while (self->running_.load()) {
    sp_xr::GimbalState state;
    state.timestamp = std::chrono::steady_clock::now();

    if (self->cfg_.use_topic_quaternion && self->has_topic_q_.load()) {
      LibXR::Mutex::LockGuard lock(self->quat_lock_);
      state.q = self->latest_topic_q_;
      state.bullet_speed = self->cfg_.mock_bullet_speed;
      state.mode = io::Mode::auto_aim;
      state.shoot_mode = io::ShootMode::left_shoot;
      state.ft_angle = 0.0;
    } else if (self->cboard_) {
      auto q = self->cboard_->imu_at(state.timestamp - 1ms);
      state.q = Eigen::Quaternionf(
        static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()),
        static_cast<float>(q.z()));
      state.bullet_speed = self->cboard_->bullet_speed;
      state.mode = self->cboard_->mode;
      state.shoot_mode = self->cboard_->shoot_mode;
      state.ft_angle = self->cboard_->ft_angle;
    } else {
      state.q = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
      state.bullet_speed = self->cfg_.mock_bullet_speed;
      state.mode = io::Mode::auto_aim;
      state.shoot_mode = io::ShootMode::left_shoot;
      state.ft_angle = 0.0;
    }

    self->gimbal_state_topic_.Publish(state);

    if (!first_publish_logged) {
      XR_LOG_PASS("[SpVisionState] first gimbal_state published.");
      first_publish_logged = true;
    }

    if (self->cfg_.publish_interval_ms > 0) {
      LibXR::Thread::Sleep(static_cast<uint32_t>(self->cfg_.publish_interval_ms));
    }
  }
}

void SpVisionState::OnQuaternion(const Eigen::Quaternionf & q)
{
  LibXR::Mutex::LockGuard lock(quat_lock_);
  latest_topic_q_ = q;
  has_topic_q_.store(true);
  if (!q_first_log_printed_.exchange(true)) {
    XR_LOG_PASS(
      "[SpVisionState] first topic quaternion received: w=%.4f x=%.4f y=%.4f z=%.4f", q.w(),
      q.x(), q.y(), q.z());
  }
}
