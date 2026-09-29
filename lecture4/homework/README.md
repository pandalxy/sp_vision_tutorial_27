# nav_lecture4小作业：Qos_debugger
这道题的目标就是让你快速上手、理解什么是ros。抛开复杂的概念，ros本质上完成的任务就是便利的进程间通信。比如，我有两个进程，一个进程发布雷达数据，另一个进程接收。使用ros就可以方便的完成通讯。你可以搜索以下，发送信息有哪些类型，分别有何特点。特别注意，不同的信息传输方式有不同的质量要求。你不会允许送的外卖没到你手上，但是一个电话过来，也许漏接了也无所谓，可能只是个诈骗。ros2也是这样。重点关注这一点会对这道题有所帮助

> 环境要求：ROS2 Humble

## 包结构

```
src/
  nav_hw_interfaces/     # 接口包：只放 .msg，无业务代码
    msg/SensorData.msg   # 可以打开.msg文件查看接口详细内容
  qos_debugger/          # 业务节点包
    src/qos_debugger_pub.cpp   # 发布 /SensorData
    src/qos_debugger_sub.cpp   # 订阅 /SensorData
```

## 编译

```bash
cd lecture4/homework
colcon build 
source install/setup.bash
```


## 任务一：实现pub和sub的通信

```bash
ros2 -h     //有忘记的命令就输入-h去查询用法
```

**现象**：启动pub和sub节点后sub节点订阅不到任何消息
提示：如果两个节点不能通过话题通信，我们应该如何区查看话题的详细信息（有没有相关的命令）
任务一仅修复qos_debugger_pub.cpp的一处或几处代码即可完成


---

## 任务二：为什么收到的消息会丢包？/(ㄒoㄒ)/~~

第一问找到问题并修改代码后，记得重新
```colcon build```
```source install/setup.bash```
**现象**：sub会打印黄色的warning输出告诉你丢包的序列，每秒还会打印出丢包率

提示：
有没有什么命令可以查看节点的配置(ros2 param -h)
可以通过修复qos_debugger_sub.cpp中的一处或几处代码解决该问题（可能会有多种解决方法）

## 任务三：把收到的消息的帧率计算并打印出来（放在定时器回调函数中每秒打印一次即可）
补全qos_debugger_sub.cpp即可



在下面按顺序完成三个任务，要求把用到的命令放入代码块中并讲解命令，每一问最好加入自己的理解


## 任务一：实现 pub 和 sub 的通信

### 现象

两个终端分别启动发布者和订阅者：

```bash
ros2 run qos_debugger qos_debugger_pub   # 终端 A：发布者
ros2 run qos_debugger qos_debugger_sub   # 终端 B：订阅者
```


`ros2 run` 表示「运行某个包里的某个节点」，`ros2 run <包名> <可执行文件名>`。启动后 sub 终端一片安静，**一条消息都收不到**，而 pub 终端一直在正常发布。
收不到消息时，用话题相关的命令一层层查：

```bash
ros2 topic list                            # 1.查看当前有哪些话题在跑
ros2 topic info /sensor_data               # 2.查看该话题上连着几个发布者/订阅者
ros2 topic info /sensor_data --verbose     # 3.显示双方 QoS 配置和是否兼容
ros2 topic echo /sensor_data               # 4.直接打印话题上的数据
```


- `ros2 topic list`：列出所有话题，确认 `/sensor_data` 这个「频道」确实存在；
- `ros2 topic info`：能看到话题上 Pub 有 1 个、Sub 有 1 个，说明两个节点确实都连到了同一个话题上，但是不通信；
- `ros2 topic info /sensor_data --verbose`（加 `-v` 也可）：能看到 QoS 兼容性检查结果，会明确显示 `reliability` 不兼容（Incompatible）；
- `ros2 topic echo`：直接打印话题数据，如果通信正常，这里会不断刷出消息；此时刷不出任何东西，就是通信断了。

### 原因分析

QoS是发布者和订阅者之间必须「对得上」的传输约定，其中一项是 reliability（可靠性）：


规则：reliable 只能和 reliable 配对，best_effort 只能和 best_effort 配对。本题中 pub 的默认参数是 `best_effort`，sub 的默认参数是 `reliable`，协议对不上，DDS 中间件直接拒绝建立连接，所以 sub 一条消息也收不到。

### 修复方法

把发布者默认的可靠性改成 `reliable`，与订阅者一致：

```cpp
// src/qos_debugger/src/qos_debugger_pub.cpp
this->declare_parameter("reliability", "reliable");   // 原来这里是 "best_effort"
```

改完重新编译生效：

```bash
cd lecture4/homework
colcon build               # 编译工作空间
source install/setup.bash  # 让当前终端能找到编译好的可执行文件
```

### 验证

再次运行 `ros2 topic info /sensor_data -v`，reliability 显示 Compatible（兼容）；sub 终端开始不断打印 `收到 seq=...`，


## 任务二：为什么收到的消息会丢包？

### 现象

通信恢复后，sub 终端会打出黄色 WARN，告诉我们丢了哪些序列，比如：

```
[WARN] [sensor_subscriber]: 检测到丢包: 期望 seq=5, 实际 seq=8, 丢失 3 条
```

每秒还会打印一行累计统计（丢包率）。

```bash
ros2 param list
ros2 param get /sensor_subscriber callback_delay_ms
ros2 param describe /sensor_subscriber callback_delay_ms
```


- `ros2 param list`：列出 `/sensor_subscriber` 节点的全部参数；
- `ros2 param get`：查看某个参数的当前值。这里查出 `callback_delay_ms` 的值是 30；
- `ros2 param describe`：看参数的元信息。

### 原因分析

sub 的回调函数故意加暂停指令：

```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(callback_delay_ms_));
```


而 pub 以 100Hz（每 10ms 一条）的速度发消息。那么：

- 消息到达速度：100 条/秒（每 10ms 一条）
- 回调处理速度：每处理一条要停 30ms，最快约 33 条/秒
- 队列大小：depth = 10

处理速度远跟不上到达速度，消息在订阅端的队列里越积越多；队列（只保留最新 10 条）塞满后，新消息进不来、旧消息被挤掉，seq 就出现跳号，也就是丢包。

### 修复方法

修改 `qos_debugger_sub.cpp` 两处：

```cpp
this->declare_parameter("callback_delay_ms", 0);

const uint32_t lost = msg->seq - expected_seq_;
lost_count_ += lost;
```

### 验证

重新 `colcon build` + `source install/setup.bash` 后再跑，WARN 消失，每秒统计为：

```
累计: 收到 50 条, 丢失 0 条, 丢包率 0%
```


## 任务三：计算并打印接收帧率

### 思路

report() 是每秒触发一次的定时器回调，用它来统计。类里已经预留了两个成员变量：

```cpp
uint32_t last_received_count_{0};
std::chrono::steady_clock::time_point last_report_time_{...};
```

### 代码

在 `qos_debugger_sub.cpp` 中补上：

```cpp

auto now = std::chrono::steady_clock::now();
double elapsed_seconds =
    std::chrono::duration<double>(now - last_report_time_).count();
if (elapsed_seconds > 0.0)
{
  double fps = static_cast<double>(received_count_ - last_received_count_) / elapsed_seconds;
  RCLCPP_INFO(this->get_logger(), "接收帧率: %.1f Hz", fps);
}
last_received_count_ = received_count_;
last_report_time_ = now;
```

逐行理解：

- `std::chrono::steady_clock::now()`：取当前时刻（单调时钟，不会像系统时间那样跳变，适合算时间差）；
- `received_count_ - last_received_count_`：这一秒内新收到的消息条数；
- 两者相除得到帧率，注意先转成 `double`，否则两个整数相除会被截断成 0；
- 打印完后把当前计数和时间存回两个变量，**为下一秒的统计做准备**（这就是「滑动统计」——每次只算最新一秒，而不是从开机到现在的平均值）。

### 验证

重新编译运行，sub 每秒打印：

```
累计: 收到 400 条, 丢失 0 条, 丢包率 0.00%
接收帧率: 100.0 Hz
```

帧率稳定在 100Hz，与发布频率一致，三个任务全部完成。

### 总结（自己的理解）

这道题从头到尾围绕一个词：**QoS**。它像发布者和订阅者之间的「通信合同」：可靠性条款决定消息能不能送到（任务一），队列深度和消费速度决定消息会不会被挤掉（任务二），而帧率统计则是站在订阅者视角量化「通信质量到底怎么样」（任务三）。ROS2 把进程间通信的脏活累活全包了，我们要做的只是理解并选对合同条款，再用命令行工具和日志验证它真的生效。



