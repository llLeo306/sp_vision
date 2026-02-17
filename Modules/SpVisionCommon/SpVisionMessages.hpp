#pragma once

#include <chrono>
#include <cstdint>
#include <list>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "Modules/SpVisionAutoAimCore/tasks/auto_aim/armor.hpp"
#include "Modules/SpVisionAutoAimCore/tasks/auto_aim/target.hpp"
#include "Modules/SpVisionCommon/command.hpp"
#include "Modules/SpVisionState/io/cboard.hpp"

namespace sp_xr
{
struct ImageFrame
{
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  uint64_t seq = 0;
};

struct ArmorFrame
{
  std::list<auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  uint64_t seq = 0;
};

struct TrackFrame
{
  std::list<auto_aim::Target> targets;
  std::chrono::steady_clock::time_point timestamp;
  uint64_t seq = 0;
};

struct GimbalState
{
  Eigen::Quaternionf q{1.0f, 0.0f, 0.0f, 0.0f};
  double bullet_speed = 0.0;
  io::Mode mode = io::Mode::idle;
  io::ShootMode shoot_mode = io::ShootMode::left_shoot;
  double ft_angle = 0.0;
  std::chrono::steady_clock::time_point timestamp;
};

struct AimCommand
{
  io::Command command{false, false, 0.0, 0.0};
  uint8_t aim_point_valid = 0;
  float aim_x = 0.0f;
  float aim_y = 0.0f;
  float aim_z = 0.0f;
  float aim_a = 0.0f;
  uint8_t aim_armor_type = 0;
  uint8_t aim_armor_name = 0;
  std::chrono::steady_clock::time_point timestamp;
  uint64_t seq = 0;
};

struct AimShareData
{
  uint32_t seq = 0;
  float yaw = 0.0f;
  float pitch = 0.0f;
  uint8_t tracking = 0;
  uint8_t shoot = 0;
};
}  // namespace sp_xr
