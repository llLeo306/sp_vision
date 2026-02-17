#include "SpVisionAimer.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "logger.hpp"

std::string SpVisionAimer::WriteAimerConfig(const AimerConfig & cfg, const void * tag)
{
  const auto dir = std::filesystem::temp_directory_path() / "sp_vision_xr_cfg";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  std::ostringstream name;
  name << "aimer_" << reinterpret_cast<uintptr_t>(tag) << ".yaml";
  const auto path = dir / name.str();

  std::ofstream ofs(path);
  ofs << "yaw_offset: " << cfg.yaw_offset_deg << "\n";
  ofs << "pitch_offset: " << cfg.pitch_offset_deg << "\n";
  ofs << "comming_angle: " << cfg.comming_angle_deg << "\n";
  ofs << "leaving_angle: " << cfg.leaving_angle_deg << "\n";
  ofs << "high_speed_delay_time: " << cfg.high_speed_delay_time << "\n";
  ofs << "low_speed_delay_time: " << cfg.low_speed_delay_time << "\n";
  ofs << "decision_speed: " << cfg.decision_speed << "\n";
  if (cfg.enable_dual_yaw_offset) {
    ofs << "left_yaw_offset: " << cfg.left_yaw_offset_deg << "\n";
    ofs << "right_yaw_offset: " << cfg.right_yaw_offset_deg << "\n";
  }
  return path.string();
}

std::string SpVisionAimer::WriteShooterConfig(const ShooterConfig & cfg, const void * tag)
{
  const auto dir = std::filesystem::temp_directory_path() / "sp_vision_xr_cfg";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  std::ostringstream name;
  name << "shooter_" << reinterpret_cast<uintptr_t>(tag) << ".yaml";
  const auto path = dir / name.str();

  std::ofstream ofs(path);
  ofs << "first_tolerance: " << cfg.first_tolerance_deg << "\n";
  ofs << "second_tolerance: " << cfg.second_tolerance_deg << "\n";
  ofs << "judge_distance: " << cfg.judge_distance << "\n";
  ofs << "auto_fire: " << (cfg.auto_fire ? "true" : "false") << "\n";
  return path.string();
}

void SpVisionAimer::InitCsvLog()
{
  if (!cfg_.enable_csv_log) {
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(cfg_.csv_log_dir, ec);
  if (ec) {
    XR_LOG_WARN("[SpVisionAimer] create csv log dir failed: %s", cfg_.csv_log_dir.c_str());
    cfg_.enable_csv_log = false;
    return;
  }

  const auto now = std::chrono::system_clock::now();
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);

  std::ostringstream name;
  name << "aim_trace_" << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S") << ".csv";
  csv_log_path_ = (std::filesystem::path(cfg_.csv_log_dir) / name.str()).string();
  csv_log_file_.open(csv_log_path_, std::ios::out | std::ios::trunc);
  if (!csv_log_file_.is_open()) {
    XR_LOG_WARN("[SpVisionAimer] open csv log failed: %s", csv_log_path_.c_str());
    cfg_.enable_csv_log = false;
    return;
  }

  csv_log_file_ << "seq,time_s,mode,bullet_speed,target_count,target_distance,target_vyaw,"
                  "gimbal_yaw,gimbal_pitch,cmd_yaw,cmd_pitch,yaw_err,pitch_err,control,shoot,"
                  "aim_valid,aim_x,aim_y,aim_z,aim_a,aim_armor_name,aim_armor_type\n";
  csv_log_file_.flush();
  csv_header_written_ = true;
  XR_LOG_PASS("[SpVisionAimer] csv log: %s", csv_log_path_.c_str());
}

void SpVisionAimer::AppendCsvLog(
  const sp_xr::TrackFrame & tracks, const sp_xr::GimbalState & state, const io::Command & command,
  const Eigen::Vector3d & gimbal_ypr)
{
  if (!cfg_.enable_csv_log || !csv_log_file_.is_open() || !csv_header_written_) {
    return;
  }

  LibXR::Mutex::LockGuard lock(csv_lock_);
  if (!has_csv_t0_) {
    csv_t0_ = tracks.timestamp;
    has_csv_t0_ = true;
  }
  const double time_s = tools::delta_time(tracks.timestamp, csv_t0_);

  double target_distance = std::numeric_limits<double>::quiet_NaN();
  double target_vyaw = std::numeric_limits<double>::quiet_NaN();
  if (!tracks.targets.empty()) {
    const auto x = tracks.targets.front().ekf_x();
    target_distance = std::sqrt(tools::square(x[0]) + tools::square(x[2]));
    target_vyaw = x[7];
  }

  const bool aim_valid = aimer_.debug_aim_point.valid;
  const double aim_x = aim_valid ? aimer_.debug_aim_point.xyza.x() : std::numeric_limits<double>::quiet_NaN();
  const double aim_y = aim_valid ? aimer_.debug_aim_point.xyza.y() : std::numeric_limits<double>::quiet_NaN();
  const double aim_z = aim_valid ? aimer_.debug_aim_point.xyza.z() : std::numeric_limits<double>::quiet_NaN();
  const double aim_a = aim_valid ? aimer_.debug_aim_point.xyza.w() : std::numeric_limits<double>::quiet_NaN();

  const double yaw_err = tools::limit_rad(command.yaw - gimbal_ypr[0]);
  const double pitch_err = tools::limit_rad(command.pitch - gimbal_ypr[1]);

  csv_log_file_ << tracks.seq << "," << std::fixed << std::setprecision(6) << time_s << ","
                << static_cast<int>(state.mode) << "," << state.bullet_speed << "," << tracks.targets.size()
                << "," << target_distance << "," << target_vyaw << "," << gimbal_ypr[0] << "," << gimbal_ypr[1]
                << "," << command.yaw << "," << command.pitch << "," << yaw_err << "," << pitch_err << ","
                << (command.control ? 1 : 0) << "," << (command.shoot ? 1 : 0) << "," << (aim_valid ? 1 : 0)
                << "," << aim_x << "," << aim_y << "," << aim_z << "," << aim_a << ","
                << static_cast<int>(aimer_.debug_target_name) << ","
                << static_cast<int>(aimer_.debug_target_armor_type) << "\n";
  ++csv_rows_;
  if ((csv_rows_ % 20) == 0) {
    csv_log_file_.flush();
  }
}

SpVisionAimer::SpVisionAimer(
  LibXR::HardwareContainer &, LibXR::ApplicationManager & app, Config cfg)
: cfg_(std::move(cfg)),
  aimer_config_path_(WriteAimerConfig(cfg_.aimer, this)),
  shooter_config_path_(WriteShooterConfig(cfg_.shooter, this)),
  aimer_(aimer_config_path_),
  shooter_(shooter_config_path_)
{
  LibXR::Topic state_topic(
    LibXR::Topic::FindOrCreate<sp_xr::GimbalState>("gimbal_state", &vision_domain_));
  auto state_cb = LibXR::Topic::Callback::Create(
    [](bool, SpVisionAimer * self, LibXR::RawData & data) {
      auto * state = reinterpret_cast<sp_xr::GimbalState *>(data.addr_);
      self->OnGimbalState(*state);
    },
    this);
  state_topic.RegisterCallback(state_cb);

  LibXR::Topic track_topic(
    LibXR::Topic::FindOrCreate<sp_xr::TrackFrame>("tracked_targets", &vision_domain_));
  auto track_cb = LibXR::Topic::Callback::Create(
    [](bool, SpVisionAimer * self, LibXR::RawData & data) {
      auto * tracks = reinterpret_cast<sp_xr::TrackFrame *>(data.addr_);
      self->OnTracks(*tracks);
    },
    this);
  track_topic.RegisterCallback(track_cb);

  InitCsvLog();

  app.Register(*this);
  XR_LOG_PASS("[SpVisionAimer] initialized.");
}

void SpVisionAimer::OnGimbalState(const sp_xr::GimbalState & state)
{
  LibXR::Mutex::LockGuard lock(state_lock_);
  latest_state_ = state;
  has_state_.store(true);
}

void SpVisionAimer::OnTracks(const sp_xr::TrackFrame & tracks)
{
  if (!has_state_.load()) {
    return;
  }

  sp_xr::GimbalState state;
  {
    LibXR::Mutex::LockGuard lock(state_lock_);
    state = latest_state_;
  }

  auto command = aimer_.aim(tracks.targets, tracks.timestamp, state.bullet_speed, cfg_.to_now);

  Eigen::Quaterniond q(state.q.w(), state.q.x(), state.q.y(), state.q.z());
  Eigen::Vector3d gimbal_ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);

  if (state.mode != io::Mode::auto_aim) {
    command = {false, false, 0.0, 0.0};
  } else {
    command.shoot = shooter_.shoot(command, aimer_, tracks.targets, gimbal_ypr);
  }

  sp_xr::AimCommand output;
  output.command = command;
  output.aim_point_valid = aimer_.debug_aim_point.valid ? 1U : 0U;
  if (aimer_.debug_aim_point.valid) {
    output.aim_x = static_cast<float>(aimer_.debug_aim_point.xyza.x());
    output.aim_y = static_cast<float>(aimer_.debug_aim_point.xyza.y());
    output.aim_z = static_cast<float>(aimer_.debug_aim_point.xyza.z());
    output.aim_a = static_cast<float>(aimer_.debug_aim_point.xyza.w());
    output.aim_armor_type = static_cast<uint8_t>(aimer_.debug_target_armor_type);
    output.aim_armor_name = static_cast<uint8_t>(aimer_.debug_target_name);
  }
  output.timestamp = tracks.timestamp;
  output.seq = tracks.seq;
  command_topic_.Publish(output);

  sp_xr::AimShareData share{};
  share.seq = static_cast<uint32_t>(tracks.seq);
  share.yaw = static_cast<float>(command.yaw);
  share.pitch = static_cast<float>(command.pitch);
  share.tracking = command.control ? 1U : 0U;
  share.shoot = command.shoot ? 1U : 0U;
  aim_share_topic_.Publish(share);

  AppendCsvLog(tracks, state, command, gimbal_ypr);

  if (command.shoot) {
    XR_LOG_INFO(
      "[SpVisionAimer] FIRE seq=%llu yaw=%.4f pitch=%.4f", static_cast<unsigned long long>(tracks.seq),
      command.yaw, command.pitch);
  }
}
