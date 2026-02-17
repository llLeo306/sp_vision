#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor, bool force_switch = false, int forced_id = -1);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  static int debug_outpost_idx();
  static double debug_outpost_dz();
  static double debug_last_outpost_z_diff();
  static bool debug_last_outpost_switch();

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;
  bool has_last_measure_z_;
  double last_measure_z_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  int update_outpost_jump(const Armor & armor, int id, bool inferred_switch);

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  // Shared outpost model state (kept static to mimic global tracker behavior).
  static double outpost_dz_;
  static double outpost_r_;
  static int outpost_idx_;
  static double outpost_cast_threshold_;
  static double last_outpost_z_diff_;
  static bool last_outpost_switch_;
  static int outpost_z_level_;
  static double outpost_z_mid_;
  static double outpost_last_high_z_;
  static double outpost_last_low_z_;
  static int outpost_switch_cooldown_;
  static bool outpost_seen_high_;
  static int outpost_trend_dir_;
  static bool outpost_model_locked_;
  static double outpost_last_switch_yaw_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
