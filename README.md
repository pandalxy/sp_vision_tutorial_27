# 27算法组导航方向招新大作业

## 0\. 环境配置

```Plain Text
// 已装 Ubuntu 22.04 和 ROS2 Humble
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-pluginlib \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-srvs \
  ros-humble-rviz2 \
  ros-humble-behaviortree-cpp-v3 \
  qtbase5-dev \
  libopencv-dev \
  libyaml-cpp-dev
  
pip3 install --user pygame numpy Pillow
```

## 1\. 作业目标

在给定的迷宫地图上，完成 **一次点击导航** 全流程：

1. 机器人从 **固定起点** 出发；

2. 选手在 RViz 中对 **指定终点** 发布一次目标；

3. 上层行为树 `click_nav.xml` 触发导航；

4. 下层行为树 `default_nav_with_fallback.xml` 调用 A\* 规划全局路径；

5. **你实现的控制器插件** 跟踪规划轨迹，使机器人准确、平滑地到达终点。

## 2\. 任务场景

### 2\.1 地图

|项目|数值|
|---|---|
|文件|`maze_map.pgm` / `maze_map.yaml`|
|分辨率|`0.05 m/pixel`|
|尺寸|`15.00 m × 15.00 m`（300×300）|
|走廊宽度|约 `1.50 m`|
|墙厚|约 `0.15 m`<br>|

<p align="center">
  <img src="images/image_1.png" alt="仿真 GUI" width="420" />
  <img src="images/image_2.png" alt="代价地图与全局路径" width="420" />
</p>

### 2\.2 固定起终点

||**map 系坐标 \(x, y\)**|**说明**|
|---|---|---|
|**起点**|`(0.900, 0.900)`|迷宫左下角通路中心；仿真初始位置已设为此处|
|**终点**|`(14.100, 14.100)`|迷宫右上角通路中心；评测时只允许对该点 click 一次|

- **不允许拖拽改起点、不允许中途二次改目标；**

- **不允许手动发布 ****`cmd_vel`**** 绕过控制器。**

LIXINY

### 2\.3 仿真器

- **不允许修改仿真器内部代码及参数；**

- **仿真里机器人撞墙便会卡住。**

## 3\. 发放内容与你需要完成的部分

### 3\.1 发放给你的代码

|包|内容|
|---|---|
|`robot_msg`|消息 / Action|
|`sp_map_server`|ESDF / 全局代价地图|
|`sp_global_planner`|A\* 全局规划|
|`sp_nav_bt`|下层 BT：`default_nav_with_fallback.xml`|
|`sp_decision`|上层 BT：`click_nav.xml`|
|`sp_nav_bringup`|启动与地图（含 `maze_map`）|
|`sp_nav_sim`|拖拽仿真（里程计 \+ TF）|
|`sp_controller_server`|控制器框架（`ControllerPlugin` 接口、`controller_node`、pluginlib 加载）|

控制器侧（名称可自定，但须 pluginlib 可加载）：

- `include/sp_controller_server/controller_plugin.hpp`（接口，只读）

- `plugins/<你的控制器>.hpp/.cpp`（由你实现）

- `plugin_description.xml`、`CMakeLists.txt`（需正确注册插件）

- `launch/controller.launch.py`

**注意：代码中已给出 PidController（\.cpp 与 \.hpp）的大体框架，可直接在此基础上修改。**

### 3\.2 你必须完成的工作（代码中已用 TODO 标明）

1. **实现路径跟踪控制器插件**

实现 `ControllerPlugin` 三个接口：

- `configure(...)`：读参数、初始化

- `setPlan(path)`：接收 `/global_path`（经 controller server 转发）

- `computeVelocityCommands(pose, velocity)`：输出 `TwistStamped`（线速度）

2. **修改参数使工程能在迷宫上跑通**

发放包中部分路径 / 话题 / 坐标系 / 为空，直接运行`sim_nav_all_start.sh`不能正确评测迷宫任务，你需要自行排查并修改。

3. **可视化与代码管理**

在 RViz 中至少清晰展示：全局代价地图、全局路径、机器人位姿、目标点等；代码要有清晰的 commit 记录，知道该忽略哪些文件。

## 4\. 评分标准（100 分）

**评测方式：固定起点 → 一次 click 到固定终点 → 记录从目标下发到判定到达的过程。**

|**分项**|**分值**|**考察内容**|
|---|---|---|
|**准确到达**|25|最终位置与终点距离|
|**时间**|25|从收到目标到到达的用时|
|**跟踪精度**|20|对采样位姿计算到全局参考路径的误差|
|**报告**|15|清晰的 README 介绍你的项目|
|**可视化与代码管理**|15|RViz 地图、路径、机器人清晰；代码管理清晰|

## 5\. 加分项（**不超过总分 100**）

- 轨迹平滑（平滑 A\* 轨迹或直接使用其他算法）；

- 自研优于 PID 的控制器（例如使用 LQR 或 MPC）。

## 6\. 提交要求

- 线上提交到自己的 git 仓库对应分支，需包含完整可编译源码；

- 如虚拟机卡顿且自己无法解决，10\.6后可线下到地下室使用小电脑调试（需提前告知群里导航方向管理员）；

- DDL：10\.25。

---

# 实现报告

## 7\. 总体方案

一次点击导航的数据流如下（灰色为发放代码，**加粗为本次完成的修改**）：

```
RViz "2D Goal Pose" --/goal_pose--> sp_decision 上层行为树 click_nav.xml
   --action: navigate_to_pose--> sp_nav_bt 下层行为树 default_nav_with_fallback.xml
   --service: make_plan--> sp_global_planner (A* 全局规划, /global_costmap)
   --/global_path--> **sp_controller_server (本作业实现的控制器插件)**
   --/sentry/cmd_vel--> sp_nav_sim (仿真器)
```

本作业完成的内容：

|任务|实现|
|---|---|
|**控制器插件**|`sp_controller_server/plugins/pid_controller.hpp/.cpp`：纯追踪（Pure Pursuit）+ 速度剖面控制器，全流程实现 `configure / setPlan / computeVelocityCommands` 三个接口|
|**参数补全**|`sp_nav_bringup/config/nav_params.yaml`：全部话题/坐标系/插件参数按仿真器接口逐一核对填齐|
|**可视化**|RViz 已配置：全局代价地图 `/global_costmap`、全局路径 `/global_path`、实际跟踪路径 `/local_path`（控制器发布）、TF、起终点标记；`sim_nav_all_start.sh` 一键启动|
|**代码管理**|`.gitignore`（忽略 build/install/log 等）、分功能提交|

## 8\. 控制器设计

### 8.1 路径预处理（`setPlan`）

A\* 输出是 0.05 m 栅格上的锯齿折线，直接跟踪会很抖。收到路径后做三步处理：

1. **直线捷径**：若中间点偏离首尾连线不超过 `shortcut_dev=0.15 m` 就跳过，把锯齿压成干净的多段线（偏离上限保证不离开走廊中心）；
2. **等距重采样**：按 `resample_step=0.05 m` 重新布点，保证后续平滑不受点距不均影响；
3. **滑动平均平滑**：窗口半径 `smooth_window=7`、两遍，两端点不动——把直角弯磨圆、限制曲率，磨圆后的弯道半径约 0.45 m；
4. **ESDF 净空约束（关键）**：滑动平均在转角处会把顶点压向墙袋（如断墙口袋净空只剩 0.35 m），叠加 plant 方向滞后 ~0.15 m 的跟踪偏移就会楔死。净空低于 `min_path_clearance=0.55 m` 的点拉回未平滑位置（迭代两遍），保证跟踪路径本身离墙足够远。

处理后的路径发布到 `/local_path`，在 RViz 中用蓝色显示，可直接观察控制器实际跟踪的轨迹。

### 8.2 纯追踪 + 速度剖面（`computeVelocityCommands`）

每周期（50 Hz）：

1. 在路径上找**最近点**，计算剩余弧长；
2. 取**前瞻点**：沿路径前进 `L = lookahead_base + lookahead_gain·v` 弧长（`0.25+0.10v`，小前瞻减小弯道切角）；
3. **方向** = 前瞻点 − 机器人位置（纯追踪）。整向量 PID 积分实测会在弯道引起指令方向卷绕（正反馈），故 `ki=0`；
4. **速度剖面** `min(巡航 1.8 m/s, 弯道限速, 贴墙限速, 终点限速)`：
   - **弯道限速**：沿路径 3 m 内每 0.1 m 采样局部曲率 κ，允许速度 `v=√(a_lat/κ)`（`a_lat=1.5`，与仿真 plant 加速度上限 2 m/s² 匹配），再按减速能力 `decel_accel=1.8 m/s²` 反向收紧，保证直线跑满巡航、入弯刹得住；
   - **贴墙防护**（订阅 `/esdf_costmap`）：机器人/前瞻点净空 < `wall_brake_dist=0.5 m` 时降速；沿**实际**速度方向 0.35v 前方净空 < 0.40 m 时急刹到 0.5 m/s（plant 方向滞后时实际速度仍可能指向墙）；前瞻点净空 < `gap_crawl_dist=0.55 m` 时蠕行 `0.4 m/s` 通过窄口；
   - **终点减速**：`v = goal_gain·剩余弧长 + goal_end_vel`（`1.6r+0.03`），距终点 `stop_dist=0.03 m` 内输出零速。
5. 无路径时持续发布零速 cmd_vel（50 Hz）：仿真器连续 5 s 收不到 cmd_vel 会转入"拖拽目标"自主行走，机器人会直冲仿真器窗口点击点撞墙。

### 8.3 底盘姿态控制（消除自旋相位彩票）

仿真器底盘默认以 3 rad/s 自旋，第一次碰墙时底盘朝向随机：若恰在 ~45°，方形占地 0.707 m 会卡死窄通道——**同样的配置有时连过几轮、有时连卡几轮**（相位彩票）。对策：控制器在导航期间发布：

- `GimbalControl(mode=3)`：云台参考角**以小步进**（每周期 ≤`pose_max_step_rad=0.14 rad` ≈ 8°，与底盘 3 rad/s 跟随速度匹配）平滑转到前瞻点处的路径朝向。云台 mode=3 是瞬时设置，整段跳变（45°/90°）会在 TF 滞后（30 Hz，最长 ~50 ms）的 1~2 个控制周期里让速度指令系被错误旋转整个跳变角——这是偶发卡墙的根源之一；
- `ChassisMode(mode=1)`：底盘跟随云台——车体始终顺着走廊方向，横向占地恒 0.5 m。
- 姿态锁定期间云台不再自旋，yaw 预测补偿自动关闭（该补偿按"匀速自旋"设计，参考角步进时反而产生预测尖峰）。

### 8.4 卡墙问题与切角预算

plant 速度滤波器实测方向阶跃滞后约 0.55 s：过弯时实际轨迹相对路径前移 v·τ。弯道内外两侧的墙袋余量只有 0.3~0.35 m，因此从三方面压住偏移：路径净空约束（跟踪路径本身离墙 ≥0.55 m）、弯道曲率限速 + 减速约束（入弯速度 √(1.5/κ)）、贴墙急刹兜底。配合姿态锁定（车体横向占地恒 0.5 m），4/4 轮连续到达无卡墙。

### 8.5 调参记录（按时间顺序）

|阶段|配置|结果|
|---|---|---|
|v1：PID 隐式速度 `u=kp·e`，ki=0.1|51.2 s 到达|速度被 `kp·\|e\|` 隐式压住|
|v2：显式速度剖面 + ki=0.3|偶尔卡墙|积分项在弯道引起方向卷绕（实测指令方向 ~40°/s 自转）|
|v3：ki=0 + 窗口式弯道限速|44.2 s 到达|直线被弯道窗口拖慢|
|v4：采样式速度剖面（3 m 前瞻+减速约束）|40.8 s 到达（不稳定）|max_vel 2.0 时入弯太猛|
|v5：小前瞻+慢弯速|44~46 s，间歇卡墙|90° 弯误差仍有 ~0.2 m 地板|
|v6：v5+姿态控制+横向积分+紧窄蠕行|8/10 轮到达|偶发楔墙（墙袋余量耗尽）|
|v7：v6+贴墙安全气泡+卡死脱困|10/10 轮到达（57~74 s）|稳定但慢（气泡频繁干预）|
|v8：回退简化 + 平滑净空约束 + 云台参考角平滑步进|**4/4 轮连续到达（45.2~49.9 s）**|落点误差 0.132~0.158 m，见 §8.4 根因分析|

### 8.6 测试结果（无头全链路，含决策树/行为树/规划/控制器/仿真）

最终版本（v8）连续 4 轮全部到达：用时 45.2~49.9 s，判定到达时距目标均 <0.3 m，停稳后距目标 0.132~0.158 m。此前各版本在弯道墙袋处有 20%~40% 的楔墙概率（仿真器撞墙即永久卡死）。v8 定位并修复了两个卡墙根因：① 滑动平均平滑把转角顶点压进墙袋（净空只剩 0.35 m，plant 方向滞后再叠加 ~0.15 m 即楔死）——用 ESDF 净空约束拉回；② 云台 mode=3 参考角整段跳变在 TF 滞后期间造成整段跳变角的方向突刺——改为每周期 ≤8° 平滑步进。

### 8.7 运行注意事项

1. **终点只能在 RViz 里点**（深灰背景、彩色代价地图的窗口，用顶部 "2D Goal Pose" 工具）。白色迷宫窗口是仿真器，**左键点击会把机器人直接拖到点击处**（无避障，直线撞墙）。
2. **不要按仿真窗口左上角的 "Sentry Move" 按钮**：它变成 BLOCK 后仿真器会忽略一切 cmd_vel（机器人彻底冻结不动，这是仿真器的"血量禁用"状态）。误按了再点一下变回 ALLOW 即可恢复。
3. **误点已不会把机器人拖走**：`controller_server` 在未收到路径时也持续发布零速 cmd_vel（50 Hz），仿真器的 cmd_vel 超时（5 s）永不触发，拖拽目标会被每个控制周期清除。
4. 点击终点后，`sp_nav` 终端应出现 `New goal_pose` → `Received goal request` → `Planning took XX ms`，缺任意一步说明点击没生效（多数是点错了窗口），重新点一次即可。

## 9\. 运行方法

```bash
# 0. 环境（Ubuntu 22.04 + ROS2 Humble），缺依赖时：
sudo apt install -y python3-colcon-common-extensions python3-rosdep ros-humble-pluginlib \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-nav-msgs ros-humble-std-srvs \
  ros-humble-rviz2 ros-humble-behaviortree-cpp-v3 qtbase5-dev libopencv-dev libyaml-cpp-dev
pip3 install --user pygame numpy Pillow

# 1. 构建
cd sp_vision_tutorial_27
colcon build --symlink-install

# 2. 一键启动（自动开 3 个终端：TF + 导航栈 + 仿真器）
bash/sim_nav_all_start.sh

# 3. 在 RViz 中点击顶部 "2D Goal Pose" 工具，在 (14.1, 14.1) 处点击一次即可
```

参数全部位于 `src/sp_nav_bringup/config/nav_params.yaml`，控制器参数在 `controller_server / PidController` 节，改完重启节点即生效（`--symlink-install` 下无需重新编译）。
