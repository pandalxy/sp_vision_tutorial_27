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
3. **滑动平均平滑**：窗口半径 `smooth_window=7`、两遍，两端点不动——把直角弯磨圆、限制曲率，磨圆后的弯道半径约 0.45 m。

处理后的路径发布到 `/local_path`，在 RViz 中用蓝色显示，可直接观察控制器实际跟踪的轨迹。

### 8.2 纯追踪 + 速度剖面（`computeVelocityCommands`）

每周期（50 Hz）：

1. 在路径上找**最近点**，计算剩余弧长；
2. 取**前瞻点**：沿路径前进 `L = lookahead_base + lookahead_gain·v` 弧长（速度低时贴路径减小切角）；
3. **方向** = 前瞻点 − 机器人位置（纯追踪）；PID 的 I 项实测会在弯道引起指令方向卷绕（正反馈），故 `ki=0`，详见 §8.5；
4. **速度大小**显式取剖面 `min(巡航 1.8 m/s, 弯道限速, 终点限速)`：
   - **弯道限速**：沿路径 3 m 内每 0.1 m 采样局部曲率 κ，各点允许速度 `v=√(a_lat/κ)`（`a_lat=1.5`，与仿真 plant 的转向能力匹配），再从远到近按减速能力 `decel_accel=1.8 m/s²` 反向收紧——保证直线段跑满巡航速度、入弯前刹得住；
   - **终点减速**：`v = goal_gain·剩余弧长 + goal_end_vel`，距终点 `stop_dist=0.03 m` 内输出零速，保证准确停位不冲线。
5. **贴墙紧急限速**（可选，默认关闭）：订阅 `/esdf_costmap`，机器人或前瞻点净空 < `wall_brake_dist` 时强制降速，防止偏离路径时楔进墙袋。

### 8.3 云台自旋补偿（本作业最大的坑）

仿真器初始即进入云台扫描模式，**云台以 1 rad/s 持续自旋**，且 `cmd_vel` 在 base_link（云台）系下解释、由仿真器按*执行时刻*的 yaw 旋转到世界系。若只按 TF 读到的 yaw 旋转指令，几十毫秒的通信/执行时延会引入与速度成正比的方向漂移（实测漂移方向与自旋方向一致），速度越高偏得越狠，最终冲进墙袋卡死。

对策：控制器用相邻周期的 yaw 差分估计云台角速度（低通滤波），按 `pred_latency=0.06 s` **外推执行时刻的 yaw** 再旋转指令。隔离实验证明该补偿把方向误差从约 +5° 压到 +1.6°。

### 8.4 导航姿态：底盘像车一样沿走廊转向

仿真器里底盘是 **0.5 m 方形**且默认以 3 rad/s **自旋**（小陀螺模式）。方形车体自旋时外接圆 0.354 m，而迷宫部分走廊/对角通道净空只有 0.3~0.4 m——一旦贴墙，仿真器的碰撞逻辑会把车体**永久楔死**（实测一次卡墙恢复要 20 秒到几分钟，整个任务拖到 5 分钟以上）。

对策（导航姿态，控制器在导航期间自动执行）：

1. 发布 `GimbalControl(mode=3, big_yaw=路径朝向)`：把云台参考角设成**前瞻点处的路径朝向**；
2. 发布 `ChassisMode(mode=1)`：底盘以 3 rad/s 跟随云台——车体**始终顺着走廊方向**，横向占地恒为 0.5 m，任何 ≥0.6 m 的通道都能过；
3. 只在路径朝向变化超过 10° 时更新参考角（弯道前提前转向，直道保持稳定）；
4. 云台 yaw 不再自旋后，§8.3 的时延漂移也自然消除（补偿代码保留作为兜底）。

导航结束后上层行为树照常发布 `ChassisMode(0)+GimbalControl(0)` 恢复小陀螺演示。实测该机制把成功率从"间歇性卡墙"提升到 **7/7 轮连续通过**（此前不加姿态控制时同一批参数 8 轮中卡墙 2 轮）。

### 8.5 卡墙问题与切角预算

仿真器底盘是**边长 0.5 m 的方形且会自旋**（外接圆半径 0.354 m），迷宫走廊宽 1.5 m、路径居中时每侧净空仅约 0.35 m（墙袋处更窄）。过弯时纯追踪切角 + plant 速度滤波器滞后（实测方向阶跃响应滞后约 0.57 s）会吃掉全部净空预算，一旦碰墙仿真器会彻底卡死。

实测结论：

- 前瞻距离越大切角越大（`L²κ/2`），**小前瞻**（0.25+0.10v）把切角压到约 0.1 m；
- 弯速取 `√(1.5/κ)`（约 1.0~1.2 m/s）时 plant 能跟上弯道，过快会甩出弯外；
- 两者配合后连续多轮测试无一次卡墙。

### 8.6 调参记录（按时间顺序）

|阶段|配置|结果|
|---|---|---|
|v1：PID 隐式速度 `u=kp·e`，ki=0.1|51.2 s 到达|速度被 `kp·\|e\|` 隐式压住，直线只有 ~1.1 m/s|
|v2：显式速度剖面 + ki=0.3|一次 50.8 s 到达，两次卡墙|积分项在弯道引起方向卷绕（实测指令方向以 ~40°/s 自转）|
|v3：ki=0 + 窗口式弯道限速|44.2 s 到达|直线仍被 1.2 m 弯道窗口拖慢|
|v4：采样式速度剖面（3 m 前瞻 + 减速约束）|40.8 s 到达（不稳定，换 max_vel 2.0 后连续卡墙）|max_vel 2.0 时入弯太猛|
|v5：小前瞻 0.25+0.10v + 弯速 √(1.5/κ)|三连到达（44.0/45.9/42.7 s）|后实测 8 轮中 2 轮在对角走廊楔墙|
|v6：v5 + 云台/底盘锁 0°|4/4 到达，但 4 轮中 1 轮楔墙|固定 0° 时车体在对角走廊占地 0.707 m，窄通道过不去|
|v7：v5 + 底盘沿路径朝向（§8.4 导航姿态）+ 速度方向前瞻刹车|**7/7 连续到达（41.5~46.0 s）**|稳定，落点误差 0.13~0.16 m|

### 8.7 测试结果（无头全链路，含决策树/行为树/规划/控制器/仿真）

|轮次|到达用时|判定到达时距目标|停稳后距目标|
|---|---|---|---|
|第 1 轮|46.0 s|0.29 m|0.160 m|
|第 2 轮|42.0 s|0.29 m|0.155 m|
|第 3 轮|43.6 s|0.29 m|0.138 m|
|第 4 轮|43.4 s|0.29 m|0.150 m|
|第 5 轮|44.4 s|0.29 m|0.149 m|
|第 6 轮|44.1 s|0.29 m|0.132 m|
|第 7 轮|41.5 s|0.29 m|0.157 m|

（到达判定 = 行为树 `NavStateTrack` 0.3 m 阈值；用时从 `/goal_pose` 下发起算。迷宫真实路线长约 49 m、弯道 20+ 处，巡航 1.8 m/s + 弯道 1.0~1.2 m/s 下 41~46 s 已接近 plant 加速度 2 m/s² 限制下的实际极限。）

### 8.8 实测中发现的运行注意事项

1. **终点只能在 RViz 里点**（深灰背景、彩色代价地图的窗口，用顶部 "2D Goal Pose" 工具）。白色迷宫窗口是仿真器，**左键点击会把机器人直接拖到点击处**（无避障，直线撞墙）。
2. **不要按仿真窗口左上角的 "Sentry Move" 按钮**：它变成 BLOCK 后仿真器会忽略一切 cmd_vel（机器人彻底冻结不动，这是仿真器的"血量禁用"状态）。误按了再点一下变回 ALLOW 即可恢复。
3. **误点已不会把机器人拖走**：`controller_server` 在未收到路径时也持续发布零速 cmd_vel（50 Hz），仿真器的 cmd_vel 超时（5 s）永不触发，拖拽目标会被每个控制周期清除（仅产生约 1 cm 的轻微顿挫）。
4. 点击终点后，`sp_nav` 终端应出现 `New goal_pose` → `Received goal request` → `Planning took XX ms`，缺任意一步说明点击没生效（多数是点错了窗口）。

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
