#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <tf2/utils.h>

#include <rclcpp/exceptions.hpp>
#include <algorithm>
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

// 带默认值的参数读取：yaml 里没写就用默认值
template<typename T>
T param_with_default(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  node->declare_parameter<T>(name, def);
  return def;
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
  stop_dist_        = param_with_default(node_, ns + "stop_dist", stop_dist_);
  shortcut_dev_     = param_with_default(node_, ns + "shortcut_dev", shortcut_dev_);
  smooth_window_    = param_with_default(node_, ns + "smooth_window", smooth_window_);
  resample_step_    = param_with_default(node_, ns + "resample_step", resample_step_);
  pred_latency_     = param_with_default(node_, ns + "pred_latency", pred_latency_);

  // 发布预处理后的路径，供 RViz 观察控制器实际跟踪的轨迹
  local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/local_path", 1);

  // 调试用：发布 map 系期望速度（linear）与所用 yaw（angular.z）
  debug_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/controller/intended_vel", 1);

  // 导航姿态：发布器与参数（底盘跟随路径朝向，消除自旋相位彩票）
  lock_nav_pose_ = param_with_default(node_, ns + "lock_nav_pose", lock_nav_pose_);
  pose_max_step_rad_ = param_with_default(node_, ns + "pose_max_step_rad", pose_max_step_rad_);
  if (lock_nav_pose_) {
    gimbal_pub_ = node_->create_publisher<robot_msg::msg::GimbalControlMsg>("/gimbal/control", 1);
    chassis_pub_ = node_->create_publisher<robot_msg::msg::ChassisModeMsg>("/chassis/mode", 1);
    // 先把云台锁在当前位置、底盘跟随过去并停住，避免开局自旋
    robot_msg::msg::GimbalControlMsg gimbal;
    gimbal.mode = 2;  // yaw 锁定（保持当前角，不瞬跳）
    gimbal.big_yaw = 0.0f;
    robot_msg::msg::ChassisModeMsg chassis;
    chassis.mode = 1;  // 底盘跟随云台
    chassis.is_stop = false;
    chassis.rotate_velocity = 0.0f;
    if (gimbal_pub_) gimbal_pub_->publish(gimbal);
    if (chassis_pub_) chassis_pub_->publish(chassis);
  }

  // ESDF 代价地图：净空栅格，用于贴墙紧急限速
  wall_brake_dist_ = param_with_default(node_, ns + "wall_brake_dist", wall_brake_dist_);
  wall_speed_gain_ = param_with_default(node_, ns + "wall_speed_gain", wall_speed_gain_);
  gap_crawl_dist_  = param_with_default(node_, ns + "gap_crawl_dist", gap_crawl_dist_);
  gap_crawl_speed_ = param_with_default(node_, ns + "gap_crawl_speed", gap_crawl_speed_);
  ct_gain_         = param_with_default(node_, ns + "ct_gain", ct_gain_);
  ct_max_          = param_with_default(node_, ns + "ct_max", ct_max_);
  esdf_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/esdf_costmap", rclcpp::QoS(1).reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      esdf_ = *msg;
      has_esdf_ = true;
    });

  RCLCPP_INFO(node_->get_logger(),
    "[%s] configured: kp=%.2f ki=%.2f kd=%.2f lookahead=%.2f+%.2f*v max_vel=%.2f",
    plugin_name_.c_str(), kp_, ki_, kd_, lookahead_base_, lookahead_gain_, max_vel_);
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

  // 4) 组装 Path，计算累计弧长与各段方向角
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
  std::size_t from, double lookahead, bool & at_end,
  std::size_t * seg_idx) const
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
  // 沿路径每 0.1 m 采样一点，用前后 ±0.15 m 的朝向变化估计该点局部曲率，
  // 得到该点允许的最大速度（向心加速度不超过 corner_lat_accel_，
  // 与仿真 plant 的加速度上限一致，plant 能实际跟踪）。
  // 再从远到近按 decel_accel_ 的减速能力收紧，保证弯前刹得住。
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
    // 采样点坐标（沿路径插值）
    double px = plan_.poses[idx].pose.position.x;
    double py = plan_.poses[idx].pose.position.y;
    const double seg_len = arc_[idx] - arc_[idx - 1];
    if (seg_len > 1e-9) {
      const double t = (arc_target - arc_[idx - 1]) / seg_len;
      px = plan_.poses[idx - 1].pose.position.x +
        t * (plan_.poses[idx].pose.position.x - plan_.poses[idx - 1].pose.position.x);
      py = plan_.poses[idx - 1].pose.position.y +
        t * (plan_.poses[idx].pose.position.y - plan_.poses[idx - 1].pose.position.y);
    }
    // 局部曲率：窗口 [arc_target-win, arc_target+win] 内的朝向变化
    std::size_t lo = idx;
    while (lo > from + 1 && arc_target - arc_[lo] < win) --lo;
    std::size_t hi = idx;
    while (hi + 1 < n && arc_[hi + 1] - arc_target < win) ++hi;
    const double arc_span = std::max(arc_[hi] - arc_[lo], 0.05);
    const double turn = std::fabs(angleDiff(heading_[hi], heading_[lo]));
    const double kappa = turn / arc_span;
    v_at[i] = std::min(max_vel_, std::sqrt(corner_lat_accel_ / std::max(kappa, 1e-4)));
    // 紧窄段限速：路径点距墙净空不足时提前蠕行，防止 plant 方向滞后
    // 使实际轨迹切入墙袋（正常走廊净空约 0.75，不受影响）
    if (has_esdf_) {
      const double gap = esdfClearance(px, py);
      if (gap < gap_crawl_dist_) {
        v_at[i] = std::min(v_at[i], gap_crawl_speed_);
      }
    }
  }

  // 反向收紧：从最远采样点向当前位置传播减速约束
  double v_limit = max_vel_;
  double v_next = v_at.back();
  for (int i = samples - 2; i >= 0; --i) {
    v_at[i] = std::min(v_at[i], std::sqrt(v_next * v_next + 2.0 * decel_accel_ * ds));
    v_next = v_at[i];
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

  // 没有路径时输出零速
  if (plan_.poses.size() < 2 || arc_.empty()) {
    return cmd_vel;
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);

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

  // 估计云台 yaw 角速度（低通滤波），用于预测执行时刻的 yaw：
  // 云台持续自旋（scan_speed 1 rad/s），cmd_vel 在 base_link 系下被
  // 执行时的 yaw 旋转到世界系。若只用 TF 读到 yaw 旋转，几十 ms 的
  // 通信/执行时延会引入与速度成正比的横向漂移，越跑越偏甚至撞墙。
  if (!first_cycle) {
    const double measured = angleDiff(yaw, last_yaw_) / dt;
    if (std::fabs(measured) > 3.0) {
      // 云台瞬跳（姿态指令步进）：瞬时跳变不能用角速度预测，清空估计
      yaw_rate_est_ = 0.0;
    } else {
      const double w = std::clamp(measured, -2.5, 2.5);
      yaw_rate_est_ = (yaw_rate_est_ == 0.0) ? w : 0.7 * yaw_rate_est_ + 0.3 * w;
    }
  }
  last_yaw_ = yaw;
  const double yaw_cmd = yaw + yaw_rate_est_ * pred_latency_;

  // 1) 最近点与剩余弧长
  const std::size_t near_idx = findNearestIndex(x, y);
  const double remaining = arc_.back() - arc_[near_idx];

  // 已到终点附近：停稳
  if (remaining < stop_dist_) {
    iex_ = iey_ = 0.0;
    return cmd_vel;
  }

  // 2) 纯追踪前瞻点（前瞻距离随速度增大，高速时更稳、低速转弯时贴路径）
  const double speed_now = std::hypot(velocity.linear.x, velocity.linear.y);
  const double lookahead = lookahead_base_ + lookahead_gain_ * speed_now;
  bool at_end = false;
  std::size_t la_seg = 0;
  const Point la = findLookahead(near_idx, lookahead, at_end, &la_seg);

  // 导航姿态：云台参考角以小步进（≤ pose_max_step_rad_）平滑转到
  // 前瞻点处的路径朝向，底盘 mode=1 跟随——车体始终顺着走廊方向，
  // 且无自旋、无大角度瞬跳（瞬跳会短暂破坏速度指令系）
  if (lock_nav_pose_ && gimbal_pub_ && heading_.size() > la_seg) {
    const double la_heading = heading_[la_seg];
    double dh = angleDiff(la_heading, last_pub_heading_rad_);
    dh = std::clamp(dh, -pose_max_step_rad_, pose_max_step_rad_);
    if (std::fabs(dh) > 0.01) {
      last_pub_heading_rad_ += dh;
      robot_msg::msg::GimbalControlMsg gimbal;
      gimbal.mode = 3;  // yaw 指定角度
      gimbal.big_yaw = static_cast<float>(last_pub_heading_rad_ * 180.0 / M_PI);
      robot_msg::msg::ChassisModeMsg chassis;
      chassis.mode = 1;  // 底盘跟随云台
      chassis.is_stop = false;
      chassis.rotate_velocity = 0.0f;
      gimbal_pub_->publish(gimbal);
      chassis_pub_->publish(chassis);
    }
  }

  const double ex = la.x - x;
  const double ey = la.y - y;

  // 3) 终点减速
  const double v_goal = goal_gain_ * remaining + goal_end_vel_;

  // 4) 曲率限速剖面（含减速约束），保证直线段跑满 max_vel、
  //    弯道按 plant 加速度能力提前减速
  double v_limit = std::min(v_goal, speedProfile(near_idx, v_goal));

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

    // 5b) 速度方向前瞻刹车：plant 的速度方向滞后于指令方向，
    // 实际速度可能仍指向弯道外侧的墙袋。沿当前实际速度方向、
    // 0.6*v 米前方采样净空，低于 0.55 m（底盘接触距离 0.354 m
    // 留足余量）就提前急刹到 0.4 m/s，给方向留出追上的时间。
    const double vx_now = velocity.linear.x;
    const double vy_now = velocity.linear.y;
    const double v_now = std::hypot(vx_now, vy_now);
    if (v_now > 0.3) {
      const double dir_x = vx_now / v_now;
      const double dir_y = vy_now / v_now;
      const double check = std::max(0.25, 0.35 * v_now);
      const double d_ahead = std::min(
        esdfClearance(x + dir_x * check, y + dir_y * check),
        esdfClearance(x + dir_x * check * 0.6, y + dir_y * check * 0.6));
      if (d_ahead < 0.40) {
        v_limit = std::min(v_limit, 0.5);
      }
    }
  }

  // 5c) 卡死检测与倒车脱困：指令速度明显大于零但实际速度接近零
  // 持续 0.8 s，说明方形底盘楔进墙袋（plant 方向滞后偶发超调）。
  // 沿路径反方向倒车 ~0.4 m 脱困后恢复正常跟踪。
  const double actual_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  if (recovering_) {
    recover_t_ += dt;
    recover_dist_ += actual_speed * dt;
    // 倒车 1.2 s 内几乎没动：换横向方向尝试脱困
    if (recover_t_ > 1.2 && recover_dist_ < 0.08) {
      recover_stage_ = 2;
      recover_t_ = 0.0;
      recover_dist_ = 0.0;
    }
    if (recover_t_ > 2.5 || recover_dist_ > 0.45) {
      recovering_ = false;
      recover_stage_ = 1;
      recover_t_ = recover_dist_ = 0.0;
      ict_ = 0.0;
      RCLCPP_WARN(node_->get_logger(), "[%s] 倒车脱困结束，恢复跟踪", plugin_name_.c_str());
    }
  } else if (v_limit > 0.25 && actual_speed < 0.12) {
    stuck_t_ += dt;
    if (stuck_t_ > 0.8) {
      recovering_ = true;
      stuck_t_ = 0.0;
      recover_t_ = recover_dist_ = 0.0;
      RCLCPP_WARN(node_->get_logger(),
        "[%s] 检测到卡死（指令 %.2f m/s 但实际 0），开始倒车脱困",
        plugin_name_.c_str(), v_limit);
    }
  } else {
    stuck_t_ = 0.0;
  }

  // 6) 方向 = 前瞻方向 + 横向积分修正。
  //    横向积分只累计「到路径的垂直偏移」（带符号），持续抵消
  //    plant 方向滞后在每段弯道入口处累积的横向偏移；
  //    不触碰前进方向分量，因此不会出现整向量积分的方向卷绕。
  const double path_tan_x = std::cos(heading_[near_idx]);
  const double path_tan_y = std::sin(heading_[near_idx]);
  // 垂直偏移（带符号）：路径法向 = (-tan_y, tan_x)，左正
  const double px_n = plan_.poses[near_idx].pose.position.x;
  const double py_n = plan_.poses[near_idx].pose.position.y;
  const double ct = (-path_tan_y) * (x - px_n) + path_tan_x * (y - py_n);
  if (first_cycle || std::fabs(ex - prev_ex_) > 0.5 || std::fabs(ey - prev_ey_) > 0.5 ||
      remaining < 0.4)
  {
    ict_ = 0.0;
  }
  prev_ex_ = ex;
  prev_ey_ = ey;
  ict_ += ct * dt;
  ict_ = std::clamp(ict_, -ct_max_, ct_max_);
  const double corr = ct_gain_ * ict_;
  const double nx = -path_tan_y;   // 法向（左）
  const double ny = path_tan_x;

  double ux = kp_ * ex + ki_ * iex_ - corr * nx;
  double uy = kp_ * ey + ki_ * iey_ - corr * ny;

  // 6b) 贴墙安全气泡（最后一道防线）：
  //   净空 < 0.48 m：完全放弃路径跟踪，方向改为纯 ESDF 梯度
  //   （远离最近墙，凹形口袋中同样指向口袋外），速度 0.2 m/s
  //   缓慢漂离，直到净空恢复——机器人永远碰不到墙。
  //   0.48~0.55 m：过渡带，梯度排斥叠加 + 限速。
  if (has_esdf_) {
    const double d_robot = esdfClearance(x, y);
    if (d_robot < 0.55) {
      const double dxp = esdfClearance(x + 0.1, y) - d_robot;
      const double dyp = esdfClearance(x, y + 0.1) - d_robot;
      const double gn = std::hypot(dxp, dyp);
      double gx = 0.0, gy = 0.0;
      if (gn > 1e-6) {
        gx = dxp / gn;
        gy = dyp / gn;
      }
      if (d_robot < 0.48) {
        ux = gx;
        uy = gy;
        v_limit = 0.2;
      } else {
        const double rep = rep_gain_ * (0.55 - d_robot);
        ux += gx * rep;
        uy += gy * rep;
        v_limit = std::min(v_limit, std::max(0.2, 3.0 * (d_robot - 0.48)));
      }
    }
  }

  // 7) 速度大小显式取速度剖面（巡航/转弯/终点/贴墙），方向保持 PID 输出方向。
  //    这样直线段能尽快到达巡航速度，不被 kp*|e| 隐式压住
  const double u_norm = std::hypot(ux, uy);
  if (u_norm > 1e-6) {
    ux *= v_limit / u_norm;
    uy *= v_limit / u_norm;
  }

  // 8) 倒车脱困时覆盖为沿路径反向低速后退（绕过限速逻辑）；
  //    倒车无效时改横向挪动（沿路径法向）
  if (recovering_) {
    const double tan_x = std::cos(heading_[near_idx]);
    const double tan_y = std::sin(heading_[near_idx]);
    if (recover_stage_ == 1) {
      ux = -tan_x * 0.4;
      uy = -tan_y * 0.4;
    } else {
      ux = -tan_y * 0.35;
      uy = tan_x * 0.35;
    }
  }

  // 9) map 系 -> base_link 系（用预测的执行时刻 yaw 旋转，补偿云台自旋时延）
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
