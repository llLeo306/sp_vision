#include "target.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
// Outpost CAD (unit: m): diameter 550, armor center heights low/mid/high = 1446/1490/1554.
constexpr double kOutpostRadius = 0.275;
constexpr std::array<double, 3> kOutpostLayerOffsets = {-0.044, 0.0, 0.064};  // L/M/H
constexpr double kOutpostDir = -1.0;  // outpost plate indexing direction along rotation
constexpr double kOutpostHalfSpan =
  0.5 * (kOutpostLayerOffsets[2] - kOutpostLayerOffsets[0]);  // for debug only
constexpr double kOutpostCastThreshold = 0.020;
constexpr double kOutpostVelocityDamp = 0.5;

int WrapOutpostId(int id)
{
  int v = id % 3;
  if (v < 0) v += 3;
  return v;
}

double OutpostLayerOffsetByMappedId(int mapped_id)
{
  return kOutpostLayerOffsets[static_cast<size_t>(WrapOutpostId(mapped_id))];
}
}  // namespace

double Target::outpost_dz_ = kOutpostHalfSpan;
double Target::outpost_r_ = kOutpostRadius;
int Target::outpost_idx_ = 0;
double Target::outpost_cast_threshold_ = kOutpostCastThreshold;
double Target::last_outpost_z_diff_ = 0.0;
bool Target::last_outpost_switch_ = false;
int Target::outpost_z_level_ = 0;
double Target::outpost_z_mid_ = 0.0;
double Target::outpost_last_high_z_ = 0.0;
double Target::outpost_last_low_z_ = 0.0;
int Target::outpost_switch_cooldown_ = 0;
bool Target::outpost_seen_high_ = false;
int Target::outpost_trend_dir_ = 0;
bool Target::outpost_model_locked_ = false;
double Target::outpost_last_switch_yaw_ = 0.0;

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  has_last_measure_z_(false),
  last_measure_z_(0.0),
  switch_count_(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）

  if (name == ArmorName::outpost && armor_num_ == 3) {
    outpost_r_ = kOutpostRadius;
    outpost_dz_ = kOutpostHalfSpan;
    outpost_cast_threshold_ = kOutpostCastThreshold;
    outpost_idx_ = 0;
    outpost_z_mid_ = xyz[2];
    outpost_last_low_z_ = xyz[2];
    outpost_last_high_z_ = xyz[2];
    outpost_z_level_ = 0;
    outpost_switch_cooldown_ = 0;
    outpost_seen_high_ = false;
    outpost_trend_dir_ = 1;  // Fixed cycle: H->L->M->H
    outpost_model_locked_ = false;  // must observe H->L jump before lock
    outpost_last_switch_yaw_ = ypr[0];
    ekf_.x[8] = outpost_r_;
  }
}

Target::Target(double x, double vyaw, double radius, double h)
: last_id(0),
  armor_num_(4),
  has_last_measure_z_(false),
  last_measure_z_(0.0)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 4.0;   // 前哨站平移过程噪声（减小抖动）
    v2 = 0.04;  // 前哨站角速度过程噪声（减小抖动）
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);

  if (name == ArmorName::outpost && armor_num_ == 3) {
    ekf_.x[8] = outpost_r_;
  }
}

void Target::update(const Armor & armor, bool force_switch, int forced_id)
{
  int id = forced_id;
  if (id < 0 || id >= armor_num_) {
    if (name == ArmorName::outpost && armor_num_ == 3) {
      const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();
      int best_id = 0;
      double best_cost = std::numeric_limits<double>::infinity();
      for (int i = 0; i < armor_num_; i++) {
        const auto & xyza = xyza_list[i];
        const Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
        const double z_diff = std::abs(armor.xyz_in_world[2] - xyza[2]);
        const double ypr_diff = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3]));
        const double los_yaw_diff = std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));
        const double cost = 1.00 * ypr_diff + 0.85 * los_yaw_diff + 1.30 * z_diff;
        if (cost < best_cost) {
          best_cost = cost;
          best_id = i;
        }
      }
      id = best_id;
    } else {
    // 装甲板匹配
      auto min_angle_error = 1e10;
      const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();
      std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
      for (int i = 0; i < armor_num_; i++) {
        xyza_i_list.push_back({xyza_list[i], i});
      }

      std::sort(
        xyza_i_list.begin(), xyza_i_list.end(),
        [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
          Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
          Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
          return ypd1[2] < ypd2[2];
        });

      const int candidate_num = std::min(3, armor_num_);
      for (int i = 0; i < candidate_num; i++) {
        const auto & xyza = xyza_i_list[i].first;
        Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
        auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                           std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

        if (std::abs(angle_error) < std::abs(min_angle_error)) {
          id = xyza_i_list[i].second;
          min_angle_error = angle_error;
        }
      }
    }
  }

  if (id != 0) jumped = true;

  bool inferred_switch = (id != last_id);
  if (force_switch && name == ArmorName::outpost && armor_num_ == 3 && has_last_measure_z_) {
    inferred_switch =
      inferred_switch || (std::abs(last_measure_z_ - armor.xyz_in_world[2]) > outpost_cast_threshold_);
  }
  is_switch_ = inferred_switch;

  if (is_switch_) switch_count_++;

  update_count_++;
  update_outpost_jump(armor, id, inferred_switch);
  last_id = id;

  // Keep EKF update id consistent with tracker association to avoid destabilizing 4-armor path.
  update_ypda(armor, id);
}

int Target::update_outpost_jump(const Armor & armor, int id, bool inferred_switch)
{
  last_outpost_switch_ = false;
  if (!(name == ArmorName::outpost && armor_num_ == 3)) {
    last_measure_z_ = armor.xyz_in_world[2];
    has_last_measure_z_ = true;
    return id;
  }

  const double z = armor.xyz_in_world[2];
  if (has_last_measure_z_) {
    last_outpost_z_diff_ = last_measure_z_ - z;
  } else {
    last_outpost_z_diff_ = 0.0;
  }
  if (!has_last_measure_z_) {
    outpost_idx_ = 0;
  } else {
    if (last_outpost_z_diff_ > outpost_cast_threshold_) {
      outpost_idx_ = 0;
    } else {
      outpost_idx_ = WrapOutpostId(outpost_idx_ + 1);
    }
    last_outpost_switch_ = true;
  }

  // Follow the tuned outpost model: jump handling damps linear velocity to improve EKF stability.
  ekf_.x[1] *= kOutpostVelocityDamp;
  ekf_.x[3] *= kOutpostVelocityDamp;
  ekf_.x[5] *= kOutpostVelocityDamp;

  const int mapped_id = WrapOutpostId(outpost_idx_);
  const double layer_offset = OutpostLayerOffsetByMappedId(mapped_id);
  const double mid_meas = z - layer_offset;
  if (!has_last_measure_z_) {
    outpost_z_mid_ = mid_meas;
  } else {
    const double alpha = 0.18;
    outpost_z_mid_ = (1.0 - alpha) * outpost_z_mid_ + alpha * mid_meas;
  }

  outpost_z_level_ = mapped_id - 1;
  outpost_last_low_z_ = outpost_z_mid_ + kOutpostLayerOffsets[0];
  outpost_last_high_z_ = outpost_z_mid_ + kOutpostLayerOffsets[2];
  outpost_r_ = kOutpostRadius;
  outpost_dz_ = kOutpostHalfSpan;
  outpost_cast_threshold_ = kOutpostCastThreshold;
  ekf_.x[8] = outpost_r_;

  has_last_measure_z_ = true;
  last_measure_z_ = z;
  (void)inferred_switch;
  return id;
}

void Target::update_ypda(const Armor & armor, int id)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};
  if (name == ArmorName::outpost && armor_num_ == 3) {
    R_dig[0] *= 1.4;
    R_dig[1] *= 1.4;
    R_dig[3] *= 1.3;
  }

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    if (name == ArmorName::outpost && armor_num_ == 3) {
      angle = tools::limit_rad(x[6] + kOutpostDir * id * 2 * CV_PI / armor_num_);
    }
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);

}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    if (name == ArmorName::outpost && armor_num_ == 3) {
      angle = tools::limit_rad(ekf_.x[6] + kOutpostDir * i * 2 * CV_PI / armor_num_);
    }
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  if (name == ArmorName::outpost && armor_num_ == 3) {
    angle = tools::limit_rad(x[6] + kOutpostDir * id * 2 * CV_PI / armor_num_);
  }
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];
  if (name == ArmorName::outpost && armor_num_ == 3) {
    // Outpost tuned model: rotate layer assignment with current outpost_idx phase.
    const int mapped_id = WrapOutpostId(id + outpost_idx_);
    armor_z = x[4] + kOutpostLayerOffsets[static_cast<size_t>(mapped_id)];
  }

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  if (name == ArmorName::outpost && armor_num_ == 3) {
    angle = tools::limit_rad(x[6] + kOutpostDir * id * 2 * CV_PI / armor_num_);
  }
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

int Target::debug_outpost_idx() { return outpost_idx_; }

double Target::debug_outpost_dz() { return outpost_dz_; }

double Target::debug_last_outpost_z_diff() { return last_outpost_z_diff_; }

bool Target::debug_last_outpost_switch()
{
  const bool latched = last_outpost_switch_;
  last_outpost_switch_ = false;
  return latched;
}

}  // namespace auto_aim
