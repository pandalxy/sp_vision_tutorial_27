#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <tf2/utils.h>

#include <rclcpp/exceptions.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

// 带默认值的参数读取：yaml 里没写就用默认值。
// 注意：launch 传的参数在节点里以 override 形式存在（has_parameter 可能为
// false），declare 后必须回读——否则 override 值被忽略、一直用默认值。
template<typename T>
T param_with_default(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  node->declare_parameter<T>(name, def);
  return node->get_parameter(name).get_value<T>();
}

// 角度差归一化到 [-pi, pi]
double angleDiff(double a, double b)
{
  double d = a - b;
  while (d > M_PI) d -= 2.0 * M_PI;
  while (d < -M_PI) d += 2.0 * M_PI;
  return d;
}

}

namespace pid_controller {

void PidController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;
  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }

  // base_frame_id 为框架要求必填参数
  base_frame_id_ = require_param<std::string>(node_, plugin_name_ + ".base_frame_id");

  // 控制器参数（未在 yaml 中填写的使用默认值）
  const std::string ns = plugin_name_ + ".";
  RCLCPP_INFO(node_->get_logger(),
    "[%s] param probe: has=%d node_name=%s",
    plugin_name_.c_str(),
    node_->has_parameter(ns + "kp") ? 1 : 0, node_->get_name());
  kp_               = param_with_default(node_, ns + "kp", kp_);
  ki_               = param_with_default(node_, ns + "ki", ki_);
  kd_               = param_with_default(node_, ns + "kd", kd_);
  ki_max_           = param_with_default(node_, ns + "ki_max", ki_max_);
  lookahead_base_   = param_with_default(node_, ns + "lookahead_base", lookahead_base_);
  lookahead_gain_   = param_with_default(node_, ns + "lookahead_gain", lookahead_gain_);
  max_vel_          = param_with_default(node_, ns + "max_vel", max_vel_);
  corner_lat_accel_ = param_with_default(node_, ns + "corner_lat_accel", corner_lat_accel_);
  speed_horizon_    = param_with_default(node_, ns + "speed_horizon", speed_horizon_);
  decel_accel_      = param_with_default(node_, ns + "decel_accel", decel_accel_);
  goal_gain_        = param_with_default(node_, ns + "goal_gain", goal_gain_);
  goal_end_vel_     = param_with_default(node_, ns + "goal_end_vel", goal_end_vel_);
  goal_decel_       = param_with_default(node_, ns + "goal_decel", goal_decel_);
  goal_lag_         = param_with_default(node_, ns + "goal_lag", goal_lag_);
  stop_dist_        = param_with_default(node_, ns + "stop_dist", stop_dist_);
  shortcut_dev_     = param_with_default(node_, ns + "shortcut_dev", shortcut_dev_);
  smooth_window_    = param_with_default(node_, ns + "smooth_window", smooth_window_);
  resample_step_    = param_with_default(node_, ns + "resample_step", resample_step_);
  pred_latency_     = param_with_default(node_, ns + "pred_latency", pred_latency_);

  // 发布预处理后的路径，供 RViz 观察控制器实际跟踪的轨迹
  local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/local_path", 1);

  // 调试用：发布 map 系期望速度（linear）与所用 yaw（angular.z）
  debug_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/controller/intended_vel", 1);

  // ESDF 代价地图：净空栅格，用于贴墙紧急限速与路径净空约束
  wall_brake_dist_ = param_with_default(node_, ns + "wall_brake_dist", wall_brake_dist_);
  wall_speed_gain_ = param_with_default(node_, ns + "wall_speed_gain", wall_speed_gain_);
  min_path_clearance_ = param_with_default(node_, ns + "min_path_clearance", min_path_clearance_);
  gap_crawl_dist_ = param_with_default(node_, ns + "gap_crawl_dist", gap_crawl_dist_);
  gap_crawl_speed_ = param_with_default(node_, ns + "gap_crawl_speed", gap_crawl_speed_);
  decel_lag_dist_ = param_with_default(node_, ns + "decel_lag_dist", decel_lag_dist_);
  pocket_check_dist_ = param_with_default(node_, ns + "pocket_check_dist", pocket_check_dist_);
  pocket_clearance_ = param_with_default(node_, ns + "pocket_clearance", pocket_clearance_);
  pocket_speed_ = param_with_default(node_, ns + "pocket_speed", pocket_speed_);
  stuck_time_ = param_with_default(node_, ns + "stuck_time", stuck_time_);
  escape_clear_dist_ = param_with_default(node_, ns + "escape_clear_dist", escape_clear_dist_);
  escape_speed_ = param_with_default(node_, ns + "escape_speed", escape_speed_);
  pose_step_arc_ = param_with_default(node_, ns + "pose_step_arc", pose_step_arc_);
  esdf_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/esdf_costmap", rclcpp::QoS(1).reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      esdf_ = *msg;
      has_esdf_ = true;
    });

  // 导航姿态：底盘跟随云台参考角（云台参考角由控制器设为路径朝向）。
  // 仿真器里底盘是 0.5 m 方形且默认以 3 rad/s 自旋，一旦贴墙（净空 < 外接圆
  // 0.354 m）就会永久卡死。让车体始终顺着走廊方向（横向占地 0.5 m），
  // 贴墙也只会轻微剐蹭、可自行恢复。
  lock_nav_pose_ = param_with_default(node_, ns + "lock_nav_pose", lock_nav_pose_);
  pose_max_step_rad_ = param_with_default(node_, ns + "pose_max_step_rad", pose_max_step_rad_);
  if (lock_nav_pose_) {
    gimbal_pub_ = node_->create_publisher<robot_msg::msg::GimbalControlMsg>("/gimbal/control", 1);
    chassis_pub_ = node_->create_publisher<robot_msg::msg::ChassisModeMsg>("/chassis/mode", 1);
    publishNavPose();
  }

  RCLCPP_INFO(node_->get_logger(),
    "[%s] configured: kp=%.2f ki=%.2f kd=%.2f lookahead=%.2f+%.2f*v max_vel=%.2f lock_nav_pose=%d",
    plugin_name_.c_str(), kp_, ki_, kd_, lookahead_base_, lookahead_gain_, max_vel_,
    lock_nav_pose_ ? 1 : 0);
  RCLCPP_INFO(node_->get_logger(),
    "[%s] pocket: check_dist=%.2f clearance=%.2f speed=%.2f lag=%.2f a_lat=%.2f",
    plugin_name_.c_str(), pocket_check_dist_, pocket_clearance_, pocket_speed_,
    decel_lag_dist_, corner_lat_accel_);
}

void PidController::publishNavPose()
{
  robot_msg::msg::GimbalControlMsg gimbal;
  gimbal.mode = 3;          // yaw 指定角度
  gimbal.big_yaw = static_cast<float>(last_pub_heading_rad_ * 180.0 / M_PI);
  robot_msg::msg::ChassisModeMsg chassis;
  chassis.mode = 1;         // 底盘跟随云台（云台定住后底盘不再自旋）
  chassis.is_stop = false;
  chassis.rotate_velocity = 0.0f;
  if (gimbal_pub_) gimbal_pub_->publish(gimbal);
  if (chassis_pub_) chassis_pub_->publish(chassis);
}

double PidController::distPointToSegment(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-12) {
    return std::hypot(px - ax, py - ay);
  }
  double t = ((px - ax) * dx + (py - ay) * dy) / len2;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (path.poses.size() < 2) {
    plan_ = path;
    arc_.clear();
    heading_.clear();
    iex_ = iey_ = 0.0;
    return;
  }

  processPlan();

  // 换新路径时重置积分，避免旧路径的积分残留在新路径上产生偏置
  iex_ = iey_ = 0.0;

  if (local_path_pub_) {
    local_path_pub_->publish(plan_);
  }
}

void PidController::processPlan()
{
  const auto & raw = global_plan_.poses;
  const std::size_t n = raw.size();

  // 1) 直线捷径：若中间点偏离首尾连线超过 shortcut_dev_ 才保留，
  //    把 A* 的锯齿折线压缩成干净的多段线（偏离上限保证不离开走廊中心）
  std::vector<Point> shorted;
  shorted.push_back({raw[0].pose.position.x, raw[0].pose.position.y});
  std::size_t anchor = 0;
  for (std::size_t j = 2; j < n; ++j) {
    double max_dev = 0.0;
    for (std::size_t k = anchor + 1; k < j; ++k) {
      max_dev = std::max(max_dev, distPointToSegment(
        raw[k].pose.position.x, raw[k].pose.position.y,
        raw[anchor].pose.position.x, raw[anchor].pose.position.y,
        raw[j].pose.position.x, raw[j].pose.position.y));
    }
    if (max_dev > shortcut_dev_) {
      shorted.push_back({raw[j - 1].pose.position.x, raw[j - 1].pose.position.y});
      anchor = j - 1;
    }
  }
  shorted.push_back({raw[n - 1].pose.position.x, raw[n - 1].pose.position.y});

  // 2) 等距重采样，保证后续平滑/曲率估计不受点距不均匀影响
  std::vector<Point> resampled;
  resampled.push_back(shorted.front());
  for (std::size_t i = 1; i < shorted.size(); ++i) {
    const double dx = shorted[i].x - shorted[i - 1].x;
    const double dy = shorted[i].y - shorted[i - 1].y;
    const double len = std::hypot(dx, dy);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / resample_step_)));
    for (int s = 1; s <= steps; ++s) {
      const double t = static_cast<double>(s) / steps;
      resampled.push_back({shorted[i - 1].x + t * dx, shorted[i - 1].y + t * dy});
    }
  }

  // 3) 滑动平均平滑（两端点不动），把直角弯磨圆，限制曲率
  std::vector<Point> smoothed = resampled;
  for (int pass = 0; pass < 2; ++pass) {
    std::vector<Point> tmp = smoothed;
    for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
      double sx = 0.0, sy = 0.0;
      int cnt = 0;
      const int lo = std::max<int>(0, static_cast<int>(i) - smooth_window_);
      const int hi = std::min<int>(static_cast<int>(smoothed.size()) - 1,
                                   static_cast<int>(i) + smooth_window_);
      for (int k = lo; k <= hi; ++k) {
        sx += smoothed[k].x;
        sy += smoothed[k].y;
        ++cnt;
      }
      tmp[i] = {sx / cnt, sy / cnt};
    }
    smoothed = tmp;
  }

  // 4) ESDF 净空约束（关键）：滑动平均在转角处会把顶点压向墙袋
  //    （如断墙口袋只剩 0.35 m），plant 方向滞后再叠加 ~0.15 m 偏移
  //    就会楔死。净空低于下限的点拉回未平滑位置，迭代两遍收敛，
  //    保证跟踪路径本身离墙足够远。
  if (has_esdf_) {
    for (int pass = 0; pass < 2; ++pass) {
      for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
        if (esdfClearance(smoothed[i].x, smoothed[i].y) < min_path_clearance_) {
          smoothed[i].x = 0.5 * (smoothed[i].x + resampled[i].x);
          smoothed[i].y = 0.5 * (smoothed[i].y + resampled[i].y);
        }
      }
    }
  }

  // 5) 组装 Path，计算累计弧长与各段方向角
  plan_ = global_plan_;
  plan_.poses.clear();
  plan_.poses.reserve(smoothed.size());
  for (const auto & p : smoothed) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = plan_.header;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    plan_.poses.push_back(ps);
  }

  arc_.resize(plan_.poses.size());
  heading_.resize(plan_.poses.size());
  arc_[0] = 0.0;
  for (std::size_t i = 1; i < arc_.size(); ++i) {
    const double dx = plan_.poses[i].pose.position.x - plan_.poses[i - 1].pose.position.x;
    const double dy = plan_.poses[i].pose.position.y - plan_.poses[i - 1].pose.position.y;
    arc_[i] = arc_[i - 1] + std::hypot(dx, dy);
    heading_[i - 1] = std::atan2(dy, dx);
  }
  heading_.back() = heading_[heading_.size() - 2];
}

std::size_t PidController::findNearestIndex(double x, double y) const
{
  std::size_t best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < plan_.poses.size(); ++i) {
    const double d = std::hypot(
      plan_.poses[i].pose.position.x - x, plan_.poses[i].pose.position.y - y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

PidController::Point PidController::findLookahead(
  std::size_t from, double lookahead, bool & at_end, std::size_t * seg_idx) const
{
  at_end = false;
  const std::size_t n = plan_.poses.size();
  if (n == 0) return {0.0, 0.0};

  const double target = arc_[from] + lookahead;
  for (std::size_t i = from + 1; i < n; ++i) {
    if (arc_[i] >= target) {
      const double seg_len = arc_[i] - arc_[i - 1];
      const double t = (seg_len > 1e-12) ? (target - arc_[i - 1]) / seg_len : 0.0;
      if (seg_idx) *seg_idx = i - 1;
      return {
        plan_.poses[i - 1].pose.position.x +
          t * (plan_.poses[i].pose.position.x - plan_.poses[i - 1].pose.position.x),
        plan_.poses[i - 1].pose.position.y +
          t * (plan_.poses[i].pose.position.y - plan_.poses[i - 1].pose.position.y)};
    }
  }
  at_end = true;
  if (seg_idx) *seg_idx = n - 2;
  return {plan_.poses.back().pose.position.x, plan_.poses.back().pose.position.y};
}

double PidController::esdfClearance(double x, double y) const
{
  if (!has_esdf_ || esdf_.info.resolution <= 0.0 || esdf_.data.empty()) {
    return 1e9;
  }
  const double res = esdf_.info.resolution;
  const int gx = static_cast<int>(std::floor((x - esdf_.info.origin.position.x) / res));
  const int gy = static_cast<int>(std::floor((y - esdf_.info.origin.position.y) / res));
  const int W = static_cast<int>(esdf_.info.width);
  const int H = static_cast<int>(esdf_.info.height);
  if (gx < 0 || gy < 0 || gx >= W || gy >= H) {
    return 1e9;
  }
  return static_cast<double>(esdf_.data[gy * W + gx]) / 100.0;
}

double PidController::speedProfile(std::size_t from, double v_goal) const
{
  // 沿路径每 0.1 m 采样一点：
  // 1) 用前后 ±0.15 m 的朝向变化估计该点局部曲率，得到允许的最大速度
  //    （向心加速度不超过 corner_lat_accel_，与仿真 plant 加速度上限一致）；
  // 2) 口袋检测：沿该点方向前方 pocket_check_dist_ 净空 < pocket_clearance_
  //    说明弯道外侧有墙袋（或对角窄道），该点限速 pocket_speed_ 蠕行——
  //    plant 方向滞后 ~0.55 s，低速下超调只有 ~0.22 m，任何口袋都安全；
  // 3) 从远到近按 decel_accel_ 收紧，并给约束点前移 decel_lag_dist_：
  //    plant 的速度响应同样有 ~0.5 s 滞后，减速指令晚生效，实际入弯速度
  //    远高于运动学剖面——约束前移让实际速度在到弯前就降到目标值。
  const double ds = 0.1;
  const double win = 0.15;
  const std::size_t n = arc_.size();
  const int samples = static_cast<int>(speed_horizon_ / ds) + 1;

  std::vector<double> v_at(samples, max_vel_);
  std::size_t idx = from + 1;
  for (int i = 0; i < samples; ++i) {
    const double s = i * ds;
    const double arc_target = arc_[from] + s;
    if (arc_target >= arc_.back()) {
      // 路径终点附近：限速交给终点减速逻辑
      break;
    }
    while (idx < n - 1 && arc_[idx] < arc_target) ++idx;
    // 局部曲率：窗口 [arc_target-win, arc_target+win] 内的朝向变化
    std::size_t lo = idx;
    while (lo > from + 1 && arc_target - arc_[lo] < win) --lo;
    std::size_t hi = idx;
    while (hi + 1 < n && arc_[hi + 1] - arc_target < win) ++hi;
    const double arc_span = std::max(arc_[hi] - arc_[lo], 0.05);
    const double turn = std::fabs(angleDiff(heading_[hi], heading_[lo]));
    const double kappa = turn / arc_span;
    v_at[i] = std::min(max_vel_, std::sqrt(corner_lat_accel_ / std::max(kappa, 1e-4)));

    // 口袋检测：仅当「前方 0.5 m 内确实有转角」时，沿当前行进方向
    // （plant 滞后时机器人过弯会继续冲的方向）探测墙距。
    // 探测点 0.5 m 处净空 < pocket_clearance_ 说明墙距 < ~0.65 m
    // （真墙袋），蠕行通过；普通弯道外侧墙距 0.7 m，不触发——
    // 普通弯道靠弯速 √(corner_lat_accel/κ) 与滞后补偿减速保证安全。
    if (has_esdf_) {
      std::size_t idx_fwd = idx;
      const double arc_fwd = std::min(arc_target + 0.5, arc_.back());
      while (idx_fwd + 1 < n && arc_[idx_fwd] < arc_fwd) ++idx_fwd;
      const double turn_ahead = std::fabs(angleDiff(heading_[idx_fwd], heading_[idx]));
      if (turn_ahead > 0.20) {
        const double h = heading_[idx];
        const double px = plan_.poses[idx].pose.position.x;
        const double py = plan_.poses[idx].pose.position.y;
        const double d_probe = esdfClearance(
          px + std::cos(h) * pocket_check_dist_,
          py + std::sin(h) * pocket_check_dist_);
        if (d_probe < pocket_clearance_) {
          v_at[i] = std::min(v_at[i], pocket_speed_);
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[pocket] crawl at path point (%.2f, %.2f) probe_clr=%.2f", px, py, d_probe);
        }
      }
    }
  }

  // 反向收紧（含 plant 速度响应滞后补偿）：对每个采样点 i，
  // 前方每个约束点 j 都要求其目标速度能在 (距离 - decel_lag_dist_) 内减到
  double v_limit = max_vel_;
  for (int i = samples - 1; i >= 0; --i) {
    for (int j = i + 1; j < samples; ++j) {
      const double d = (j - i) * ds;
      v_at[i] = std::min(v_at[i], std::sqrt(
        v_at[j] * v_at[j] + 2.0 * decel_accel_ * std::max(0.0, d - decel_lag_dist_)));
    }
  }
  v_limit = std::min(max_vel_, v_at.front());
  return v_limit;
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double speed_now = std::hypot(velocity.linear.x, velocity.linear.y);

  // 控制周期 dt（用节点时钟兜底）
  const rclcpp::Time now = node_->now();
  const bool first_cycle = !has_last_time_;
  double dt = 0.02;
  if (!first_cycle) {
    dt = (now - last_time_).seconds();
  }
  last_time_ = now;
  has_last_time_ = true;
  if (dt <= 0.0 || dt > 0.5) {
    dt = 0.02;
  }

  // 云台自旋补偿（云台自由自旋、未锁定姿态时生效）：
  // 云台扫描自旋时（scan_speed 1 rad/s），cmd_vel 在 base_link 系下被
  // 执行时刻的 yaw 旋转到世界系。若只用 TF 读到 yaw 旋转，几十 ms 的
  // 通信/执行时延会引入与速度成正比的横向漂移，越跑越偏甚至撞墙。
  // 姿态锁定（lock_nav_pose）时云台由控制器保持在固定参考角，不再自旋，
  // 此时无需预测——预测反而会在参考角步进时产生尖峰误差。
  double yaw_cmd = yaw;
  if (!lock_nav_pose_) {
    if (!first_cycle) {
      const double measured = angleDiff(yaw, last_yaw_) / dt;
      const double w = std::clamp(measured, -2.5, 2.5);
      yaw_rate_est_ = (yaw_rate_est_ == 0.0) ? w : 0.7 * yaw_rate_est_ + 0.3 * w;
    }
    yaw_cmd = yaw + yaw_rate_est_ * pred_latency_;
  }
  last_yaw_ = yaw;

  // 没有路径时：仍运行卡死脱困。规划失败（机器人楔进 lethal 区）时
  // BT 会反复重试规划，此时机器人必须能漂回自由区，规划才能恢复。
  // 注意：云台扫描自旋时，脱困方向必须用预测 yaw 旋转——若用原始 yaw，
  // 漂移方向会跟着云台 1 rad/s 打转，脱困净位移趋近于零。
  if (plan_.poses.size() < 2 || arc_.empty()) {
    if (has_esdf_) {
      const double c_self = esdfClearance(x, y);
      if (speed_now < 0.12 && c_self < escape_clear_dist_) {
        double best = c_self;
        double bx = 0.0, by = 0.0;
        for (int k = 0; k < 8; ++k) {
          const double a = k * M_PI / 4.0;
          const double ck = esdfClearance(x + std::cos(a) * 0.15, y + std::sin(a) * 0.15);
          if (ck > best) {
            best = ck;
            bx = std::cos(a);
            by = std::sin(a);
          }
        }
        const double cy = std::cos(yaw_cmd);
        const double sy = std::sin(yaw_cmd);
        cmd_vel.twist.linear.x = cy * bx * escape_speed_ + sy * by * escape_speed_;
        cmd_vel.twist.linear.y = -sy * bx * escape_speed_ + cy * by * escape_speed_;
      }
    }
    return cmd_vel;
  }


  // 1) 最近点与剩余弧长
  const std::size_t near_idx = findNearestIndex(x, y);
  const double remaining = arc_.back() - arc_[near_idx];

  // 已到终点附近：停稳
  if (remaining < stop_dist_) {
    iex_ = iey_ = 0.0;
    return cmd_vel;
  }

  // 2) 纯追踪前瞻点（前瞻距离随速度增大，高速时更稳、低速转弯时贴路径）
  const double lookahead = lookahead_base_ + lookahead_gain_ * speed_now;
  bool at_end = false;
  std::size_t la_seg = 0;
  const Point la = findLookahead(near_idx, lookahead, at_end, &la_seg);

  // 2.5) 物理停摆检测：odom 速度非零但位置长时间不动，说明仿真器 GUI
  // 循环在 CPU 高负载下被饿死（物理冻结）。恢复后 plant 会以停摆前的
  // 旧速度滑行，在弯道处会冲进墙袋楔死。检测到停摆立即零速刹车，
  // 恢复后的滑行距离只剩 plant 滤波器本身的制动距离。
  if (has_last_pos_ && speed_now > 0.15) {
    const double dp = std::hypot(x - last_pos_x_, y - last_pos_y_);
    if (dp < 0.01) {
      if (stall_since_.nanoseconds() == 0) {
        stall_since_ = now;
      } else if ((now - stall_since_).seconds() > 0.3) {
        stall_brake_ = true;
      }
    } else {
      stall_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      stall_brake_ = false;
    }
  } else if (stall_brake_ && speed_now <= 0.15) {
    // 刹车生效（plant 已停）后释放，恢复跟踪——
    // 否则位置不动无法满足释放条件，刹车永久卡死
    stall_brake_ = false;
    stall_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  if (has_last_pos_) {
    travel_dist_ += std::hypot(x - last_pos_x_, y - last_pos_y_);
  }
  last_pos_x_ = x;
  last_pos_y_ = y;
  has_last_pos_ = true;

  // 导航姿态：云台参考角以小步进平滑转向前瞻点处的路径朝向，底盘(mode=1)跟随，
  // 车体始终顺着走廊方向。云台 mode=3 是瞬时设置，若整段跳变（45°/90°），
  // TF 滞后（30 Hz，最长 ~50 ms）期间的 1~2 个控制周期里速度指令系会被
  // 错误旋转整个跳变角，机器人朝错误方向冲一小段——这正是偶发卡墙的根源。
  // 步进按「累计行驶里程」门控（每 0.1 m 或停住时步进一次，每次 ≤ pose_max_step_rad）：
  // 仿真卡顿变慢时步进也随之变慢，底盘 3 rad/s 的跟随永远跟得上，
  // 车体不会在弯道中落后成 45° 斜姿卡进墙袋。
  // 注意：不能用 arc_[near_idx] 门控——BT 每 100 ms 重规划一次，新路径的
  // 弧长从 0 重新计起，而上次步进的弧长不会随之清零，progress 恒为负，
  // 云台参考角从此冻结、车体不再旋转，过弯直冲墙楔死（"行走时自转停止"）。
  if (lock_nav_pose_ && gimbal_pub_ && heading_.size() > la_seg) {
    const double progress = travel_dist_ - last_step_dist_;
    if (progress >= pose_step_arc_ || speed_now < 0.15) {
      const double target = heading_[la_seg];
      const double delta = angleDiff(target, last_pub_heading_rad_);
      if (std::fabs(delta) > 0.02) {   // >1° 才发，避免稳态刷屏
        const double step = std::clamp(delta, -pose_max_step_rad_, pose_max_step_rad_);
        last_pub_heading_rad_ += step;
        publishNavPose();
      }
      last_step_dist_ = travel_dist_;
    }
    // 周期重发（0.5 s）：仿真器晚于控制器启动、或消息丢失时，
    // 底盘会在收到第一条姿态指令前一直以 3 rad/s 自旋（相位彩票），
    // 周期重发保证姿态指令尽快生效。
    if (last_pose_pub_time_.nanoseconds() == 0 ||
        (now - last_pose_pub_time_).seconds() > 0.5)
    {
      publishNavPose();
      last_pose_pub_time_ = now;
    }
  }

  const double ex = la.x - x;
  const double ey = la.y - y;

  // 3) 终点减速：带 plant 滞后补偿的运动学停车剖面。
  //    v = √(2·a·max(0, r − τ·v))：按当前速度把停车距离外推 τ·v，
  //    保证 plant 速度滞后不会把机器人带过终点线。
  const double r_eff = std::max(0.0, remaining - goal_lag_ * speed_now);
  const double v_goal = std::sqrt(2.0 * goal_decel_ * r_eff);

  // 4) 曲率限速剖面（含减速约束），保证直线段跑满 max_vel、
  //    弯道按 plant 加速度能力提前减速
  double v_limit = std::min(v_goal, speedProfile(near_idx, v_goal));

  // 物理停摆刹车：立即零速，防止恢复后旧速度滑行冲墙
  if (stall_brake_) {
    v_limit = 0.0;
  }

  // 5) 贴墙紧急限速：机器人/前瞻点附近净空过低时降速，
  //    防止偏离路径时楔进墙袋
  if (has_esdf_) {
    const double d1 = esdfClearance(x, y);
    const double d2 = esdfClearance(la.x, la.y);
    const double d = std::min(d1, d2);
    if (d < wall_brake_dist_) {
      const double v_wall = 0.5 + wall_speed_gain_ * std::max(0.0, d - 0.35);
      v_limit = std::min(v_limit, std::max(0.3, v_wall));
    }

    // 速度方向前瞻刹车：过弯时 plant 的速度方向滞后于指令方向，
    // 实际速度仍指向弯道外侧的墙袋。沿当前速度方向向前采样净空，
    // 若即将撞墙则提前急刹，避免楔死（直线行驶时不受影响）。
    const double vx_now = velocity.linear.x;
    const double vy_now = velocity.linear.y;
    const double v_now = std::hypot(vx_now, vy_now);
    if (v_now > 0.3) {
      const double dir_x = vx_now / v_now;
      const double dir_y = vy_now / v_now;
      const double check = std::max(0.25, 0.35 * v_now);   // 前瞻距离随速度增大
      const double d_ahead =
        std::min(esdfClearance(x + dir_x * check, y + dir_y * check),
                 esdfClearance(x + dir_x * check * 0.6, y + dir_y * check * 0.6));
      if (d_ahead < 0.40) {
        v_limit = std::min(v_limit, 0.5);
      }
    }

    // 前瞻点净空过低（原路径本身贴着窄口）时蠕行通过
    if (esdfClearance(la.x, la.y) < gap_crawl_dist_) {
      v_limit = std::min(v_limit, gap_crawl_speed_);
    }
  }

  // 5.5) 卡死检测与贴墙脱困：指令明显但机器人实际不动（楔墙/沿墙滑行）
  //       持续 stuck_time_ 秒判定卡死，放弃路径跟踪，朝 8 个采样方向中
  //       净空最大的方向以 escape_speed_ 漂离；净空恢复到
  //       escape_clear_dist_ 后退出脱困、恢复跟踪。
  //       按"卡死"触发而非按净空触发，避免在对角窄道（净空 0.35~0.45）
  //       正常行驶时误触发。
  double ux = 0.0, uy = 0.0;
  bool escaping = false;
  if (has_esdf_) {
    const double c_self = esdfClearance(x, y);
    // 卡死判定只看「实际不动 + 贴墙」，不依赖 v_limit——
    // 停摆刹车会把 v_limit 置 0，若条件含 v_limit 则脱困永远无法触发，
    // 两个保护机制互相压制，机器人卡死在墙边。
    if (speed_now < 0.12 && c_self < escape_clear_dist_) {
      if (stuck_since_.nanoseconds() == 0) {
        stuck_since_ = now;
      } else if ((now - stuck_since_).seconds() > stuck_time_) {
        escaping = true;
      }
    } else {
      stuck_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    if (escaping) {
      double best = c_self;
      double bx = 0.0, by = 0.0;
      for (int k = 0; k < 8; ++k) {
        const double a = k * M_PI / 4.0;
        const double ck = esdfClearance(x + std::cos(a) * 0.15, y + std::sin(a) * 0.15);
        if (ck > best) {
          best = ck;
          bx = std::cos(a);
          by = std::sin(a);
        }
      }
      ux = bx * escape_speed_;
      uy = by * escape_speed_;
      iex_ = iey_ = 0.0;
      if (c_self > escape_clear_dist_) {
        escaping = false;
      }
    }
  }

  // 6) PID 决定期望速度的方向（前瞻点误差作为跟踪误差，
  //    积分项消除转弯时的稳态横向偏差）
  //    首周期/路径跳变/接近终点时清积分，防止过冲
  if (first_cycle || std::fabs(ex - prev_ex_) > 0.5 || std::fabs(ey - prev_ey_) > 0.5 ||
      remaining < 0.4)
  {
    iex_ = iey_ = 0.0;
  }
  prev_ex_ = ex;
  prev_ey_ = ey;

  if (!escaping) {
    iex_ += ex * dt;
    iey_ += ey * dt;
    iex_ = std::clamp(iex_, -ki_max_, ki_max_);
    iey_ = std::clamp(iey_, -ki_max_, ki_max_);

    ux = kp_ * ex + ki_ * iex_;
    uy = kp_ * ey + ki_ * iey_;

    // 7) 速度大小显式取速度剖面（巡航/转弯/终点/贴墙），方向保持 PID 输出方向。
    //    这样直线段能尽快到达巡航速度，不被 kp*|e| 隐式压住。
    const double u_norm = std::hypot(ux, uy);
    if (u_norm > 1e-6) {
      ux *= v_limit / u_norm;
      uy *= v_limit / u_norm;
    }
  }

  // 8) map 系 -> base_link 系（用预测的执行时刻 yaw 旋转，补偿云台自旋时延）
  const double cos_yaw = std::cos(yaw_cmd);
  const double sin_yaw = std::sin(yaw_cmd);
  cmd_vel.twist.linear.x = cos_yaw * ux + sin_yaw * uy;
  cmd_vel.twist.linear.y = -sin_yaw * ux + cos_yaw * uy;

  if (debug_pub_) {
    geometry_msgs::msg::TwistStamped dbg;
    dbg.header.stamp = now;
    dbg.header.frame_id = "map";
    dbg.twist.linear.x = ux;
    dbg.twist.linear.y = uy;
    dbg.twist.angular.z = yaw_cmd;
    debug_pub_->publish(dbg);
  }

  RCLCPP_DEBUG(node_->get_logger(),
    "[%s] rem=%.2f v_lim=%.2f cmd=(%.2f, %.2f)",
    plugin_name_.c_str(), remaining, v_limit,
    cmd_vel.twist.linear.x, cmd_vel.twist.linear.y);

  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
