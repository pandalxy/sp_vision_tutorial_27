#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <robot_msg/msg/chassis_mode_msg.hpp>
#include <robot_msg/msg/gimbal_control_msg.hpp>

#include "sp_controller_server/controller_plugin.hpp"

namespace pid_controller {

// 纯追踪(Pure Pursuit) + 速度剖面 路径跟踪控制器
//
// 思路：
// 1. setPlan 对 A* 全局路径做预处理（直线捷径 + 重采样 + 滑动平均平滑），并计算弧长/朝向表；
// 2. 每个控制周期在路径上找「最近点」和「前瞻点」（纯追踪），
//    前瞻点与机器人位置的差即跟踪误差 e；
// 3. 方向由 e（可加 PID 项，实测积分项在弯道会引起指令方向卷绕，默认 ki=0）决定；
// 4. 速度大小显式取速度剖面：min(巡航 max_vel, 弯道曲率限速(含减速约束), 终点减速)，
//    另可选 ESDF 贴墙紧急限速（默认关闭）；
// 5. 用「预测的执行时刻云台 yaw」把速度旋转到 base_link 系下发，
//    补偿云台持续自旋（scan 1 rad/s）导致的执行时延方向漂移。
class PidController : public sp_controller_server::ControllerPlugin {
public:
  PidController() = default;
  ~PidController() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  struct Point {
    double x, y;
  };

  // 路径预处理：捷径 -> 等距重采样 -> 滑动平均平滑
  void processPlan();
  // 在路径上找离 (x,y) 最近的点下标（全路径线性扫描，路径只有几百个点）
  std::size_t findNearestIndex(double x, double y) const;
  // 从 from 点沿路径前进 lookahead 弧长，返回前瞻点（必要时插值）；
  // seg_idx 输出前瞻点所在路径段下标
  Point findLookahead(std::size_t from, double lookahead, bool & at_end,
                      std::size_t * seg_idx = nullptr) const;
  // 曲率限速剖面：沿路径采样局部曲率得到每点限速，
  // 再按减速能力从远到近收紧，返回当前位置允许的速度
  double speedProfile(std::size_t from, double v_goal) const;
  // 点到线段的距离
  static double distPointToSegment(
    double px, double py, double ax, double ay, double bx, double by);

  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr node_;
  std::string base_frame_id_;

  nav_msgs::msg::Path global_plan_;   // 收到的原始全局路径
  nav_msgs::msg::Path plan_;          // 预处理后的路径
  std::vector<double> arc_;           // 每个点距起点的累计弧长
  std::vector<double> heading_;       // 每段的方向角

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_sub_;

  // 导航姿态：把底盘朝向平滑领到路径方向（云台 mode=3 小步进 + 底盘 mode=1 跟随）。
  // 底盘默认 3 rad/s 自旋，碰墙瞬间朝向随机，45° 时方形占地 0.707 m
  // 会卡死窄通道（相位彩票）。让底盘始终顺着走廊方向可彻底消除。
  rclcpp::Publisher<robot_msg::msg::GimbalControlMsg>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<robot_msg::msg::ChassisModeMsg>::SharedPtr chassis_pub_;
  bool lock_nav_pose_{true};        // 是否在导航期间控制云台/底盘姿态
  double last_pub_heading_rad_{0.0};  // 最近一次发布的云台参考角

  // ESDF 代价地图（/esdf_costmap：data = 净空距离*100 的 int8 栅格）
  nav_msgs::msg::OccupancyGrid esdf_;
  bool has_esdf_{false};
  // 查询 (x,y) 处距墙净空 m；无数据返回大数
  double esdfClearance(double x, double y) const;

  // ---- 可调参数（均可在 nav_params.yaml 的 PidController 下覆盖）----
  double kp_{3.0};          // 比例增益 1/s（方向项，归一化后仅影响大小）
  double ki_{0.0};          // 积分增益；实测弯道中会引起方向卷绕，默认 0
  double kd_{0.0};          // 微分增益（预留，默认 0）
  double ki_max_{0.5};      // 积分抗饱和上限 m/s
  double lookahead_base_{0.25};  // 前瞻距离基值 m（小前瞻减小弯道切角）
  double lookahead_gain_{0.10};  // 前瞻距离随速度的增益 s
  double max_vel_{1.8};     // 最大线速度 m/s
  double corner_lat_accel_{1.5};  // 转弯允许的最大向心加速度 m/s^2（与仿真加速度上限一致）
  double speed_horizon_{3.0};     // 速度剖面前瞻弧长 m
  double decel_accel_{1.8};       // 剖面反向收紧用的减速能力 m/s^2（留裕量）
  double goal_gain_{1.0};   // 终点减速：v = gain*剩余距离 + end_vel
  double goal_end_vel_{0.05};     // 终点减速的末端速度 m/s
  double stop_dist_{0.03};  // 距终点小于该距离时输出零速 m
  double shortcut_dev_{0.15};     // 路径捷径允许的最大偏离 m
  int smooth_window_{7};    // 平滑窗口半径（点数）
  double resample_step_{0.05};    // 重采样步长 m
  double pred_latency_{0.06};     // 云台 yaw 预测时延 s（补偿自旋导致的执行时延）
  double wall_brake_dist_{0.55};  // 距墙净空低于该值开始限速 m
  double wall_speed_gain_{2.0};   // 贴墙限速：v = 0.5 + gain*(d - 0.35)
  double gap_crawl_dist_{0.60};   // 路径前方点距墙净空低于该值时蠕行 m
  double gap_crawl_speed_{0.35};  // 紧窄段蠕行速度 m/s
  double pose_max_step_rad_{0.14};  // 云台参考角每周期最大步进 rad（约 8°）
  double ct_gain_{1.5};   // 横向积分增益（抵消滞后横向偏移）
  double ct_max_{0.8};    // 横向积分饱和上限 m·s
  double rep_gain_{2.0};  // 贴墙排斥力场增益

  // ---- 运行状态 ----
  double iex_{0.0}, iey_{0.0};    // 积分项
  double ict_{0.0};              // 横向（cross-track）积分项
  double stuck_t_{0.0};           // 卡死累计时间 s
  bool recovering_{false};        // 倒车脱困中
  int recover_stage_{1};          // 1=沿路径倒车 2=横向挪动
  double recover_t_{0.0};         // 脱困已耗时 s
  double recover_dist_{0.0};      // 脱困已后退距离 m
  double prev_ex_{0.0}, prev_ey_{0.0};
  double last_yaw_{0.0};          // 上次云台 yaw
  double yaw_rate_est_{0.0};      // 云台 yaw 角速度估计 rad/s
  rclcpp::Time last_time_;
  bool has_last_time_{false};
};

}

#endif
