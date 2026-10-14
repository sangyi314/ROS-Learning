# ROS 2 第六周完整自学讲义：AMCL 定位、Nav2 自主导航与 C++ 任务控制

> 环境：Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic；C++17，Launch 使用 Python。
>
> 直接承接你上传的《ROS2-Week5-Complete-Guide(1).md》。复用第五周的房间、机器人、雷达、桥接和 `room_final.yaml + room_final.pgm`，本周让机器人在这张地图上自主到达目标。
>
> 前提是第五周的验收已经实际完成。拿到第五周文档不等于完成第五周。若地图尚未保存、TF 尚未稳定，先补齐这些前置任务。

<a id="contents"></a>
## 阅读导航

- [剩余学习时间与明确的结束点](#finish)
- [本周目标、时间表和工程约定](#overview)
- [第 1 天：导航架构、准备工程与速度链](#day1)
- [第 2 天：地图服务器、AMCL 与初始位姿](#day2)
- [第 3 天：代价地图、机器人尺寸与 Nav2 启动](#day3)
- [第 4 天：规划、跟踪与第一段自动导航](#day4)
- [第 5 天：C++ NavigateToPose 客户端与取消](#day5)
- [第 6 天：多目标任务与受控失败实验](#day6)
- [第 7 天：录包、测量、报告与综合验收](#day7)
- [附录：排错、速查、自测和官方资料](#appendix)

<a id="finish"></a>
## 剩余学习时间与明确的结束点

最初的基础路线约八周，按每天 2～3 小时、每周六天安排；仿真、导航联调可以增加 1～2 周缓冲。后面逐周讲义展开后，单周内容比最初的提纲更充实，第五周已经完成原本靠后安排的建图，因此现在按你实际拿到的讲义衔接，**不用再重复一整周 SLAM**。

本次把剩余基础阶段明确为下面三个阶段。第七周的细分内容是根据当前第五周成果做的调整，第八周继续以完成第三个项目为结束点。

| 阶段 | 学什么 | 结束时拿得出什么 | 预计有效学习时间 |
|---|---|---|---:|
| 第六周 | AMCL、Nav2、costmap、规划与控制、C++ 导航 Action | 能定位、到点、取消、执行简单多点任务的导航系统 | 26～32 小时 |
| 第七周 | 稳定导航、行为树阅读、故障定位、参数对比、任务逻辑完善 | 多轮实验数据、可解释的失败记录、改进后的任务程序 | 18～24 小时 |
| 第八周 | 整理第三个完整项目，从干净终端复现，README、视频、项目讲解 | 可复现仓库、演示、系统说明、测量结果 | 12～16 小时 |

**从第五周验收通过开始，还剩约 56～72 小时的基础阶段工作。** 这是学习与排错的估算，不是软件运行时间，也不是保证天数。

| 实际投入 | 大致还需多久 |
|---|---|
| 每天 3～4 小时，每周六天 | 约 3～4 个自然周 |
| 每天 2～3 小时，每周六天 | 通常约 4～6 个自然周 |
| 遇到虚拟机图形、地图质量或版本问题 | 在上述估算之外留出 1～2 周机动时间 |

“第六周”是一个学习单元，不要求你必须在七个日历日内硬做完。若从 2026 年 9 月 28 日开始、第五周已验收、每天能投入 3～4 小时，目标可以设在 **10 月中下旬完成基础项目阶段**；若还在补前几周内容，就从实际开始第六周的日期重新计算。

完成计划的标准：你能从工作空间和配置出发，解释并运行“速度指令—底盘—里程计—TF—雷达—地图—定位—导航—任务程序”，遇到问题能找出是哪一层，而不是只能跟着教程复制命令。

之后的 Eigen/EKF、`ros2_control`、路径跟踪算法属于后续专项路线，不计入这次八周基础阶段。你不必把机器人全部知识学完，才算完成当前计划。也不要求这三周额外购买实物机器人。

<a id="overview"></a>
## 本周目标、时间表和工程约定

第五周回答“环境长什么样”；本周回答“机器人在哪里、怎样到目标、任务失败或取消时怎么办”。

| 天 | 主要内容 | 当天成果 | 建议用时 |
|---|---|---|---:|
| 1 | 系统分工、工程复制、重新组织速度保护链 | 新工作空间、能停止的仿真输入链 | 4～5 小时 |
| 2 | 静态地图、AMCL、初始位姿、TF 责任 | 雷达与地图对齐，AMCL 提供 map→odom | 3～4 小时 |
| 3 | global/local costmap、尺寸、完整参数和启动 | Nav2 节点 active，代价地图正常 | 4～5 小时 |
| 4 | NavFn、RPP、到点和朝向、基础调参 | 首次自主到点和转弯导航 | 3～4 小时 |
| 5 | C++ Action 请求、反馈、结果、取消 | 可重复调用的 C++ 导航客户端 | 4～5 小时 |
| 6 | 串行多目标、不可达、取消、传感器故障 | 成功和失败都能解释的任务记录 | 4～5 小时 |
| 7 | 录包、结果测量、报告、独立复现 | 第六周验收报告与演示 | 4 小时 |

### 本周固定约定

| 项目 | 固定值或责任方 |
|---|---|
| 工作空间 | `~/ros2_week6_ws` |
| 复用包 | `gazebo_robot_lab`、`mapping_lab`，复制第五周源码 |
| 新包 | `navigation_lab` |
| ROS 域 / Gazebo 分区 | `42` / `ros2_week6_lab` |
| 世界与模型名 | 保留 `week5`、`week5_bot`，不更改已有 SDF |
| 坐标系 | `map`、`odom`、`base_link`、`laser_link` |
| 地图 / 激光 / 里程计 | `/map`、`/scan`、`/odom` |
| map→odom | 本周只由 AMCL 发布 |
| odom→base_link | Gazebo 差速插件，经桥接进入 ROS |
| 雷达固定外参 | `robot_state_publisher` |
| `/clock` | 只来自当前 Gazebo |
| 速度消息类型 | 全链路 `geometry_msgs/msg/Twist` |
| 输出目录 | `maps/`、`bags/`、`reports/` |

**运行纪律：**第五周建图、地图显示、离线回放都要先退出。本周不启动 SLAM Toolbox，不用旧 `mapping.launch.py`，也不同时启动另一套 Nav2。所有终端先加载本周环境。Gazebo 分区不同不能隔离同 ROS 域的同名话题。

### 本周接入方案

第五周的 `velocity_guard` 在全向最近距离小于 0.60 m 时会同时阻挡前进和转弯，可能使导航无法脱困。本周保留那份源码和旧配置，但新 Launch **不启动旧 guard**，改成明确的五段速度链：

| 顺序 | 谁发布 | 输出话题 | 谁接收 |
|---|---|---|---|
| 1 | controller_server 或 behavior_server | `/cmd_vel_nav` | velocity_smoother |
| 2 | velocity_smoother | `/cmd_vel_smoothed` | collision_monitor |
| 3 | collision_monitor | `/cmd_vel_checked` | 本周 C++ motion_gate |
| 4 | motion_gate | `/cmd_vel` | base_bridge |
| 5 | base_bridge | Gazebo `/week5/cmd_vel` | 差速插件 |

Collision Monitor 用机器人外形和运动预测检查激光；`motion_gate` 保留命令超时、扫描有效性、时钟异常、速度限幅和可锁定停车。`/cmd_vel` 在 ROS 侧只有 motion_gate 一个发布者。这里的保护是教学仿真逻辑，实物还需要硬件停止与驱动层超时。

### 验证范围

本文针对你提供的第五周接口编写；Nav2 配置与 Action 用法按 Jazzy 官方资料核对。文档中的 Python、YAML、XML、Shell 片段做静态检查，纯数学例题可本地执行。编写环境未安装 ROS 2/Gazebo，**没有声称 ROS C++ 节点已在仿真中编译运行，也没有预先测得导航成功率**。运行验收必须在你的 Ubuntu 环境完成。具体软件小版本记录在第七天报告中。

<a id="day1"></a>
## 第 1 天：导航架构、准备工程与速度链

### 1.1 先把各组件的工作分清

| 组件 | 输入 | 输出/职责 | 一个常见误解 |
|---|---|---|---|
| map_server | 保存好的 YAML/PGM | 发布静态 `/map` | 发布地图不等于定位 |
| AMCL | 地图、雷达、里程计 TF、初值 | 估计地图位姿，发布 map→odom | 不负责重新建图 |
| global_costmap | 地图、实时障碍、TF、尺寸 | 全局规划使用的代价场 | 不是原始 PGM 图片 |
| planner_server | 起点、目标、global costmap | 一串路径位姿 `/plan` | 不直接驱动轮子 |
| local_costmap | 附近地图/障碍、TF、尺寸 | 跟随机器人移动的局部环境 | 不是另一张 SLAM 地图 |
| controller_server | 路径、局部环境、机器人状态 | 速度指令 | 本周不是左右轮 PID |
| bt_navigator | NavigateToPose 目标 | 编排规划、跟踪、恢复、结果 | 不包含所有算法本身 |
| behavior_server | 行为请求 | 旋转、后退、等待 | 这些运动也必须经过保护链 |
| lifecycle_manager | 节点列表 | 按依赖顺序配置和激活 | 节点存在不代表 active |

你第二个 C++ 项目的轮速 PID 负责让轮子跟随目标转速。Nav2 控制器产生的是底盘的 `v、ω`，层级更高。本周仍使用 Gazebo 差速插件完成底盘运动，没有偷偷接入 `ros2_control`。

### 1.2 创建工作空间与复制已有成果

停止第五周终端，在新终端执行。以下检查失败会中止子 shell，不会关闭终端，也不会覆盖已经存在的包。

```bash
(
  set -e
  test -f ~/ros2_week5_ws/src/gazebo_robot_lab/package.xml
  test -f ~/ros2_week5_ws/src/gazebo_robot_lab/urdf/robot.urdf
  test -f ~/ros2_week5_ws/src/mapping_lab/worlds/mapping_room.sdf
  test -f ~/ros2_week5_ws/src/mapping_lab/config/bridge.yaml
  test -f ~/ros2_week5_ws/src/mapping_lab/config/laser_bridge.yaml
  test -f ~/ros2_week5_ws/maps/room_final.yaml
  test -f ~/ros2_week5_ws/maps/room_final.pgm
  mkdir -p ~/ros2_week6_ws/src
  for pkg in gazebo_robot_lab mapping_lab; do
    if [ ! -d "$HOME/ros2_week6_ws/src/$pkg" ]; then
      cp -a "$HOME/ros2_week5_ws/src/$pkg" ~/ros2_week6_ws/src/
    fi
  done
  mkdir -p ~/ros2_week6_ws/src/navigation_lab/{src,config,launch,rviz}
  mkdir -p ~/ros2_week6_ws/{maps,bags,reports,scripts}
  cp -n ~/ros2_week5_ws/maps/room_final.yaml ~/ros2_week6_ws/maps/
  cp -n ~/ros2_week5_ws/maps/room_final.pgm ~/ros2_week6_ws/maps/
)
```

若实际地图叫 `room_v1`，把上述源文件名替换成真实文件名，后续 Launch 的 `map:=` 也用真实 YAML。不要为了通过检查复制一张不相关的示例地图。检查 YAML 的 `image:` 指向同目录对应 PGM；若第五周用了绝对路径，将它改成正确的相对图片文件名，分辨率和 origin 保持原值。

保存 `~/ros2_week6_ws/setup_build.bash`：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=ros2_week6_lab
```

保存 `~/ros2_week6_ws/setup_run.bash`：

```bash
source ~/ros2_week6_ws/setup_build.bash
source ~/ros2_week6_ws/install/local_setup.bash
```

`.bashrc` 不要自动加载第四、第五周 overlay。编译终端仅 source `setup_build.bash`，运行终端 source `setup_run.bash`。

安装本周软件：

```bash
source ~/ros2_week6_ws/setup_build.bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-nav2-collision-monitor ros-jazzy-nav2-velocity-smoother \
  ros-jazzy-nav2-rviz-plugins python3-yaml
```

这里只新增 Nav2 相关依赖；Gazebo、`ros_gz_bridge`、SLAM Toolbox 与编译工具沿用前几周的安装。找不到 Jazzy 包时先检查 Ubuntu/软件源，不混装其他 ROS 发行版。

### 1.3 包说明与第一阶段 CMake

保存 `~/ros2_week6_ws/src/navigation_lab/package.xml`：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>navigation_lab</name>
  <version>0.1.0</version>
  <description>Week 6 localization and navigation laboratory</description>
  <maintainer email="learner@example.com">learner</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>nav2_msgs</depend>
  <exec_depend>mapping_lab</exec_depend>
  <exec_depend>gazebo_robot_lab</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_amcl</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>
  <exec_depend>nav2_controller</exec_depend>
  <exec_depend>nav2_planner</exec_depend>
  <exec_depend>nav2_navfn_planner</exec_depend>
  <exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>
  <exec_depend>nav2_behaviors</exec_depend>
  <exec_depend>nav2_bt_navigator</exec_depend>
  <exec_depend>nav2_velocity_smoother</exec_depend>
  <exec_depend>nav2_collision_monitor</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

保存 `~/ros2_week6_ws/src/navigation_lab/CMakeLists.txt`。第五天再添加客户端 target，今天不引用还不存在的源文件。

```cmake
cmake_minimum_required(VERSION 3.8)
project(navigation_lab)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
add_executable(motion_gate src/motion_gate.cpp)
ament_target_dependencies(motion_gate rclcpp geometry_msgs sensor_msgs std_msgs std_srvs)
target_compile_options(motion_gate PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS motion_gate DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
```

### 1.4 完整 C++ 文件：最终速度门与停车服务

这个节点只负责速度链最后的基本条件与停车，不替代路径规划或 Collision Monitor。默认处于 HOLD，必须由你完成输入检查后主动解除。

保存 `~/ros2_week6_ws/src/navigation_lab/src/motion_gate.cpp`：

```cpp
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using Steady = std::chrono::steady_clock;
using Twist = geometry_msgs::msg::Twist;

class MotionGate : public rclcpp::Node {
public:
  MotionGate() : Node("motion_gate") {
    output_ = create_publisher<Twist>("/cmd_vel", 1);
    status_ = create_publisher<std_msgs::msg::String>(
      "/motion_gate/status", rclcpp::QoS(1).transient_local());
    input_ = create_subscription<Twist>("/cmd_vel_checked", 1,
      [this](Twist::ConstSharedPtr msg) {
        const bool finite = std::isfinite(msg->linear.x) &&
          std::isfinite(msg->linear.y) && std::isfinite(msg->linear.z) &&
          std::isfinite(msg->angular.x) && std::isfinite(msg->angular.y) &&
          std::isfinite(msg->angular.z);
        const bool planar = std::abs(msg->linear.y) < 1e-6 &&
          std::abs(msg->linear.z) < 1e-6 &&
          std::abs(msg->angular.x) < 1e-6 &&
          std::abs(msg->angular.y) < 1e-6;
        command_valid_ = finite && planar && !hold_;
        command_ = *msg;
        received_ = Steady::now();
      });
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        scan_valid_ = msg->header.frame_id == "laser_link" &&
          std::isfinite(msg->range_min) && std::isfinite(msg->range_max) &&
          msg->range_min >= 0.0 && msg->range_max > msg->range_min &&
          std::any_of(msg->ranges.begin(), msg->ranges.end(),
            [msg](float r) {
              return std::isfinite(r) && r >= msg->range_min && r <= msg->range_max;
            });
        scan_stamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
        scan_received_ = Steady::now();
      });
    service_ = create_service<std_srvs::srv::SetBool>("/motion_gate/set_hold",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
        hold_ = req->data;
        command_valid_ = false;
        output_->publish(Twist{});
        resp->success = true;
        resp->message = hold_ ? "HOLD latched" : "Released; waiting for fresh command";
      });
    last_clock_change_ = Steady::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { tick(); });
  }

private:
  void tick() {
    const auto wall = Steady::now();
    const auto stamp = now().nanoseconds();
    if (stamp < previous_stamp_) {
      hold_ = true;
      command_valid_ = false;
    }
    if (stamp != previous_stamp_) last_clock_change_ = wall;
    previous_stamp_ = stamp;
    const auto age = [wall](Steady::time_point t) {
      return std::chrono::duration<double>(wall - t).count();
    };
    const double scan_age = static_cast<double>(stamp - scan_stamp_) / 1e9;
    std::string reason = "PASS";
    if (hold_) reason = "HOLD";
    else if (stamp <= 0 || age(last_clock_change_) > 1.0) reason = "CLOCK_STALE";
    else if (!scan_valid_ || age(scan_received_) > 1.0 ||
             scan_age < -0.05 || scan_age > 0.5) reason = "SCAN_INVALID_OR_STALE";
    else if (!command_valid_ || age(received_) > 0.5) reason = "COMMAND_INVALID_OR_STALE";

    Twist out;
    if (reason == "PASS") {
      const double linear = std::abs(command_.linear.x);
      const double angular = std::abs(command_.angular.z);
      double scale = 1.0;
      if (linear > 0.18) scale = std::min(scale, 0.18 / linear);
      if (angular > 0.40) scale = std::min(scale, 0.40 / angular);
      out.linear.x = command_.linear.x * scale;
      out.angular.z = command_.angular.z * scale;
      if (scale < 1.0) reason = "LIMITED";
    }
    output_->publish(out);
    if (reason != last_status_) {
      std_msgs::msg::String msg;
      msg.data = reason;
      status_->publish(msg);
      RCLCPP_INFO(get_logger(), "%s", reason.c_str());
      last_status_ = reason;
    }
  }
  bool hold_{true}, command_valid_{false}, scan_valid_{false};
  Twist command_;
  std::int64_t previous_stamp_{0}, scan_stamp_{0};
  Steady::time_point received_{}, scan_received_{}, last_clock_change_{};
  std::string last_status_;
  rclcpp::Publisher<Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<Twist>::SharedPtr input_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionGate>());
  rclcpp::shutdown();
  return 0;
}
```

读代码时只抓五件事：

1. 输入是已经经过碰撞检查的速度，输出才是真正送给底盘的 `/cmd_vel`。
2. `Steady` 测量现实经过了多久，仿真暂停也会继续计时；扫描年龄则用仿真时间检查。
3. 停车时每 50 ms 持续发布零，避免 Gazebo 保留最后一条非零命令。
4. 解除 HOLD 后要求新命令，避免立即重放解除前的缓存。
5. 同比例缩放 `v、ω` 尽量保留曲率；碰撞停车可以立即归零，不受舒适性加速度限制约束。

本节点使用单线程 `rclcpp::spin`，回调顺序执行，所以此处不需要 mutex。以后换成多线程执行器且允许回调并发时，要重新分析共享状态。

本例只用于第五周的封闭房间：全为无穷远的扫描被视为不可用，在空旷真实环境中需要重新设计该判断。时间倒退会锁定 HOLD；仿真重置后建议完整重启系统。

### 1.5 完整 Launch：仿真输入链

保存 `~/ros2_week6_ws/src/navigation_lab/launch/sim.launch.py`。这里直接引用第五周资源，但不 include 旧 sim Launch，避免它顺带启动旧 guard。

```python
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    mapping = Path(get_package_share_directory('mapping_lab'))
    base = Path(get_package_share_directory('gazebo_robot_lab'))
    gui = IfCondition(LaunchConfiguration('gui')).evaluate(context)
    headless = IfCondition(LaunchConfiguration('headless')).evaluate(context)
    if gui and headless:
        raise ValueError('headless:=true requires gui:=false')
    cmd = ['gz', 'sim', '-r', '-v', '3']
    if not gui:
        cmd.append('-s')
    if headless:
        cmd.append('--headless-rendering')
    cmd.append(str(mapping / 'worlds' / 'mapping_room.sdf'))
    description = (base / 'urdf' / 'robot.urdf').read_text(encoding='utf-8')
    stop = [EmitEvent(event=Shutdown(reason='Simulation input chain exited'))]
    return [
        ExecuteProcess(cmd=cmd, output='screen', on_exit=stop),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='base_bridge', output='screen',
             parameters=[{'config_file': str(mapping / 'config' / 'bridge.yaml')}],
             on_exit=stop),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='laser_bridge', output='screen',
             parameters=[{'config_file': str(mapping / 'config' / 'laser_bridge.yaml')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': True, 'publish_frequency': 50.0,
                          'robot_description': ParameterValue(description, value_type=str)}]),
        Node(package='navigation_lab', executable='motion_gate', name='motion_gate',
             output='screen', parameters=[{'use_sim_time': True}], on_exit=stop),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=start),
    ])
```

### 1.6 编译与当天验收

编译终端：

```bash
source ~/ros2_week6_ws/setup_build.bash
cd ~/ros2_week6_ws
colcon build --symlink-install --packages-up-to navigation_lab
```

新的运行终端 A：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 launch navigation_lab sim.launch.py
```

新终端 B：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 pkg prefix mapping_lab
ros2 pkg prefix navigation_lab
ros2 topic echo /clock --once
ros2 topic echo /scan --once --field header
ros2 topic echo /odom --once --field header
ros2 run tf2_ros tf2_echo odom base_link --ros-args -p use_sim_time:=true
```

最后一条持续运行，观察后 Ctrl+C。检查 package prefix 都指向第六周 install，而不是旧工作空间。继续：

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel --once
ros2 topic echo /motion_gate/status --once --qos-durability transient_local
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: true}'
```

应看到 `/cmd_vel` 唯一发布者是 `motion_gate`，速度全零，状态 HOLD。今天尚无 Nav2，机器人保持静止就是正确结果。

**当天练习：**规划器输出 Path，为什么不能直接交给第五周的差速插件？

答案：Path 是位置序列，插件需要速度；控制器需要根据机器人当前位姿、路径与局部障碍，把“想去哪”持续转换成“此刻怎么走”。

[回到导航](#contents)

<a id="day2"></a>
## 第 2 天：地图服务器、AMCL 与初始位姿

### 2.1 AMCL 在估计什么

AMCL 的全称是 Adaptive Monte Carlo Localization，中文通常叫自适应蒙特卡洛定位。它假设地图已经有了，用一批“机器人可能处于这里”的位姿假设，即粒子，估计当前位姿。

一次更新可以这样理解：里程计说机器人移动了，粒子按运动模型扩散；雷达说周围墙在这些位置，算法检查各粒子对应的预测是否与观察相符；匹配好的粒子获得较大权重，再重采样。自适应粒子数量不等于绝对不会定位失败。

你本周需要理解过程和排查输入，不需要从零编写粒子滤波器。

**例子：**左右对称的走廊里，两处位置可能得到相似扫描，仅凭一帧难区分。第五周 `box_a`、`box_b` 的不对称布局有助于消除这种歧义；良好初始位姿能缩小搜索范围。

### 2.2 三种位置不要混用

| 名称 | 含义 | 本周用途 |
|---|---|---|
| Gazebo world 中的模型坐标 | 仿真世界真实模型位置 | 理解场景和检查出生点 |
| odom 中的位姿 | 从起点累积的连续局部运动 | 局部控制、短期运动估计 |
| map 中的位姿 | 当前机器人相对保存地图的位置 | 导航目标、全局定位 |

它们在出生时可能近似相同，但不是同一概念。地图 YAML 的 `origin` 是图像栅格的摆放位置，**不是机器人初始位姿**。

本周 TF 仍是 `map → odom → base_link → laser_link`，但第一条边的发布者从 SLAM 换成 AMCL。地图服务器不发布这条边，也不要用静态零变换临时补上。

### 2.3 完整定位参数

保存 `~/ros2_week6_ws/src/navigation_lab/config/localization.yaml`：

```yaml
map_server:
  ros__parameters:
    use_sim_time: true
    topic_name: map
    frame_id: map
amcl:
  ros__parameters:
    use_sim_time: true
    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_link
    scan_topic: /scan
    map_topic: /map
    tf_broadcast: true
    robot_model_type: nav2_amcl::DifferentialMotionModel
    laser_model_type: likelihood_field
    laser_min_range: 0.05
    laser_max_range: 8.0
    laser_likelihood_max_dist: 2.0
    max_beams: 60
    min_particles: 500
    max_particles: 2000
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    z_hit: 0.8
    z_rand: 0.2
    sigma_hit: 0.2
    do_beamskip: false
    update_min_d: 0.05
    update_min_a: 0.10
    resample_interval: 1
    transform_tolerance: 1.0
    set_initial_pose: false
    always_reset_initial_pose: true
    save_pose_rate: -1.0
```

理解四处即可：`base_link` 与自己的 TF 一致；`scan_topic` 与实际激光一致；`set_initial_pose: false` 要求你给初始位姿；较小的 `update_min_d/a` 让慢速教学机器人更容易触发更新。`transform_tolerance` 不能补救错误时钟。

### 2.4 完整定位 Launch

保存 `~/ros2_week6_ws/src/navigation_lab/launch/localization.launch.py`：

```python
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('navigation_lab'))
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise FileNotFoundError(f'Map YAML not found: {map_path}')
    params = str(share / 'config' / 'localization.yaml')
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params, {'yaml_filename': str(map_path)}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[{
                 'use_sim_time': True, 'autostart': True,
                 'node_names': ['map_server', 'amcl'], 'bond_timeout': 10.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(
            Path.home() / 'ros2_week6_ws' / 'maps' / 'room_final.yaml')),
        OpaqueFunction(function=start),
    ])
```

编译一次，再在运行终端加载本周环境。三个终端分别执行，前一个程序不退出：

终端 A：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 launch navigation_lab sim.launch.py
```

终端 B：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 launch navigation_lab localization.launch.py
```

终端 C：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix --share nav2_bringup)/rviz/nav2_default_view.rviz" \
  --ros-args -p use_sim_time:=true
```

RViz 使用安装包的 Nav2 默认视图，后续按你的话题修改显示项并保存为自己的配置。今天 planner/controller 尚未启动，局部地图和路径显示暂时没有数据是正常的。

### 2.5 设置初始位姿的完整操作

1. 确认 Gazebo 正在运行，机器人还在出生点；motion_gate 保持 HOLD。
2. RViz 的 Fixed Frame 设为 `map`，Map 显示话题为 `/map`。地图若不出现，设 Reliable、Transient Local。
3. 添加或检查 LaserScan 显示：话题 `/scan`，Reliability 为 Best Effort。
4. 点击工具栏 **2D Pose Estimate**，在保存地图中机器人实际所在位置按下鼠标，拖拽到机器人朝向，再松开。
5. 第五周默认从原点朝 +x 开始建图，出生点在地图中通常接近 `(0,0,0)`；这是本场景的初值提示，不能推广成任意地图的固定初值。以第五周保存时的坐标关系和场景特征核对。
6. 等待粒子与扫描更新。雷达命中的外墙和箱体轮廓应与地图对应；不能只看“有红点”。

在新终端 D 检查：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /map --verbose
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true
```

`/amcl_pose` 不保证静止时持续高频发布。若 `--once` 等待新数据，在 RViz 重新设置一次正确初值；不要仅因频率低就判断 AMCL 死机。还可以运行 `ros2 topic info /amcl_pose --verbose`，按实际发布者 QoS 设置订阅。

初值设置工具的鼠标拖动长度主要帮助表达朝向，不是设置机器人线速度。**2D Pose Estimate 不会把 Gazebo 机器人瞬移。** 若方向错了，就重新设置，不要拖动 Gazebo 模型来迎合错误估计。

### 2.6 初始误差与协方差

平面位姿通常写成 `(x,y,θ)`。定位结果中的协方差描述估计的不确定性；不是“真实误差值”。协方差小而墙对不上，依然可能是错误收敛或配置错误。

例题：地图估计为 `(1,2,π/2)`，odom 位姿为 `(0.5,0,0)`。求 map→odom。

\[
T_{map,odom}=T_{map,base}\,T_{odom,base}^{-1}.
\]

逆位姿平移是 `(-0.5,0)`，经 `π/2` 旋转变成 `(0,-0.5)`，所以 map→odom 为 **`(1,1.5,π/2)`**。不能把三个数简单逐项相减。

**当天验收：**map_server、AMCL 均 active；只有一个地图发布者；设初值后 map→base_link 可查；扫描与地图合理对齐；你能说清 AMCL 和 SLAM 的区别。此时尚未允许自动运动。

[回到导航](#contents)

<a id="day3"></a>
## 第 3 天：代价地图、机器人尺寸与 Nav2 启动

### 3.1 占据地图和代价地图

第五周的占据栅格表达哪里是空闲、障碍和未知；Nav2 的代价地图把这些信息、实时扫描和机器人的尺寸综合成规划与控制的代价。

| 信息 | 用途 | 本周实例 |
|---|---|---|
| Static Layer | 保存地图提供的环境 | 外墙、箱体、隔板 |
| Obstacle Layer | 当前雷达发现/清除的障碍 | 扫描射线与命中点 |
| Inflation Layer | 给障碍周围增加代价 | 鼓励路径远离墙体 |
| footprint / robot_radius | 描述机器人占用的空间 | 避免只考虑中心点 |

Nav2 内部代价值常见为 `0` 空闲、`253` 内切膨胀障碍、`254` 致命障碍、`255` 未知；对外发布的 `OccupancyGrid` 会转换编码，不要把 `/costmap` 消息中的数字直接当作内部字节值。

**膨胀不是把整个半径内全变成绝对禁区。** 有的是不可通过代价，有的是软代价梯度。是否通过还取决于机器人尺寸、规划与碰撞检查。

### 3.2 为什么本周用保守圆形尺寸

你的第五周 SDF 中车体长宽 `0.24 × 0.20 m`，左右轮中心 `y=±0.15 m`、轮厚 `0.03 m`，因此总体横向至少到 `±0.165 m`。只把车体宽度 0.20 m 作为整车宽度，会漏掉轮子。

本周 costmap 使用 `robot_radius: 0.30`、`footprint_padding: 0.01`，是覆盖当前模型的保守包络，方便先在宽通道验证；这不是测得的机器人真实半径。Collision Monitor 的预测矩形为 `x=±0.18、y=±0.22 m`，位于这个圆形包络内。

以后要过窄通道，可以测量真实轮廓、使用多边形 footprint 并重新验证规划器碰撞模型。不要为了让路径生成，把机器人尺寸改得比真实模型还小。

### 3.3 两张 costmap 分别放在哪

| 项目 | global_costmap | local_costmap |
|---|---|---|
| 坐标系 | map | odom |
| 范围 | 已保存的整张地图 | 机器人附近 4 m × 4 m |
| 是否滚动 | 否 | 是 |
| 作用 | 找整体路线 | 执行路径时检查附近环境 |
| 本例插件 | Static + Obstacle + Inflation | Static + Obstacle + Inflation |

本例两张图都接入实时激光，所以临时障碍也可进入全局规划；局部图同时有 Static Layer，避免仅凭此刻扫描忽略已知墙壁。

### 3.4 完整 Nav2 参数文件

保存 `~/ros2_week6_ws/src/navigation_lab/config/nav2.yaml`。不要把下面各顶层节点缩进到 `controller_server` 内。两张 costmap 的双层同名结构是 ROS 节点命名空间需要，不是笔误。

```yaml
planner_server:
  ros__parameters:
    use_sim_time: true
    expected_planner_frequency: 1.0
    planner_plugins: [GridBased]
    GridBased:
      plugin: nav2_navfn_planner::NavfnPlanner
      tolerance: 0.10
      use_astar: false
      allow_unknown: false
      use_final_approach_orientation: false

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 10.0
    odom_topic: /odom
    enable_stamped_cmd_vel: false
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: [progress_checker]
    goal_checker_plugins: [goal_checker]
    controller_plugins: [FollowPath]
    progress_checker:
      plugin: nav2_controller::SimpleProgressChecker
      required_movement_radius: 0.10
      movement_time_allowance: 30.0
    goal_checker:
      plugin: nav2_controller::SimpleGoalChecker
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.15
      stateful: true
    FollowPath:
      plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
      desired_linear_vel: 0.15
      lookahead_dist: 0.45
      use_velocity_scaled_lookahead_dist: false
      rotate_to_heading_angular_vel: 0.35
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.60
      max_angular_accel: 0.8
      allow_reversing: false
      transform_tolerance: 0.20
      min_approach_linear_velocity: 0.04
      approach_velocity_scaling_dist: 0.60
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.5
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.8
      regulated_linear_scaling_min_speed: 0.11
      cost_scaling_dist: 0.50
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link
      update_frequency: 2.0
      publish_frequency: 1.0
      resolution: 0.05
      robot_radius: 0.30
      footprint_padding: 0.01
      transform_tolerance: 0.30
      track_unknown_space: true
      rolling_window: false
      always_send_full_costmap: true
      plugins: [static_layer, obstacle_layer, inflation_layer]
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_topic: /map
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          data_type: LaserScan
          marking: true
          clearing: true
          obstacle_min_range: 0.05
          obstacle_max_range: 7.5
          raytrace_min_range: 0.05
          raytrace_max_range: 8.0
          inf_is_valid: true
          max_obstacle_height: 2.0
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.65
        cost_scaling_factor: 3.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.05
      robot_radius: 0.30
      footprint_padding: 0.01
      transform_tolerance: 0.30
      track_unknown_space: true
      always_send_full_costmap: true
      plugins: [static_layer, obstacle_layer, inflation_layer]
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_topic: /map
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          data_type: LaserScan
          marking: true
          clearing: true
          obstacle_min_range: 0.05
          obstacle_max_range: 7.5
          raytrace_min_range: 0.05
          raytrace_max_range: 8.0
          inf_is_valid: true
          max_obstacle_height: 2.0
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.65
        cost_scaling_factor: 3.0

behavior_server:
  ros__parameters:
    use_sim_time: true
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    cycle_frequency: 10.0
    transform_tolerance: 0.20
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.35
    min_rotational_vel: 0.10
    rotational_acc_lim: 0.8
    enable_stamped_cmd_vel: false
    behavior_plugins: [spin, backup, wait]
    spin:
      plugin: nav2_behaviors::Spin
    backup:
      plugin: nav2_behaviors::BackUp
    wait:
      plugin: nav2_behaviors::Wait

velocity_smoother:
  ros__parameters:
    use_sim_time: true
    smoothing_frequency: 20.0
    scale_velocities: true
    feedback: OPEN_LOOP
    max_velocity: [0.18, 0.0, 0.40]
    min_velocity: [-0.15, 0.0, -0.40]
    max_accel: [0.30, 0.0, 0.8]
    max_decel: [-0.50, 0.0, -1.0]
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 0.5
    odom_topic: /odom
    odom_duration: 0.1
    enable_stamped_cmd_vel: false

collision_monitor:
  ros__parameters:
    use_sim_time: true
    base_frame_id: base_link
    odom_frame_id: odom
    cmd_vel_in_topic: /cmd_vel_smoothed
    cmd_vel_out_topic: /cmd_vel_checked
    state_topic: /collision_monitor_state
    transform_tolerance: 0.20
    source_timeout: 0.5
    base_shift_correction: true
    stop_pub_timeout: 2.0
    enable_stamped_cmd_vel: false
    polygons: [BodyStop, BodyApproach]
    BodyStop:
      type: polygon
      points: '[[0.15, 0.19], [0.15, -0.19], [-0.15, -0.19], [-0.15, 0.19]]'
      action_type: stop
      min_points: 1
      visualize: true
      polygon_pub_topic: /body_stop
      enabled: true
    BodyApproach:
      type: polygon
      points: '[[0.18, 0.22], [0.18, -0.22], [-0.18, -0.22], [-0.18, 0.22]]'
      action_type: approach
      time_before_collision: 1.5
      simulation_time_step: 0.05
      min_points: 1
      visualize: true
      polygon_pub_topic: /body_approach
      enabled: true
    observation_sources: [scan]
    scan:
      type: scan
      topic: /scan
      source_timeout: 0.5
      enabled: true

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    transform_tolerance: 0.20
    bt_loop_duration: 10
    default_server_timeout: 100
    wait_for_service_timeout: 2000
    navigators: [navigate_to_pose]
    navigate_to_pose:
      plugin: nav2_bt_navigator::NavigateToPoseNavigator
```

这是当前房间的起步配置，不是经过优化的通用参数。几个容易混淆的点：

- `allow_unknown: false` 明确要求在已知区域规划。地图没扫全，要补图，不能直接把未知当可通行。
- `marking` 用命中点添加障碍，`clearing` 用有效射线清除过时观测；前者不能代替后者。
- `min_velocity` 的负角速度必须允许，否则机器人无法向右转。RPP 不倒车不妨碍恢复行为在检查后短距离后退。
- `OPEN_LOOP` 只描述速度平滑器内部如何估计上一次速度，不表示整个导航是开环。RPP 仍然持续使用机器人位姿反馈。
- Collision Monitor 的 `approach` 检查当前指令预测的碰撞；较远侧墙不会仅因“全向最近距离小于 0.6 m”一律锁住旋转。BodyStop 仍阻挡紧贴机器人轮廓的危险情况。
- 不同 Jazzy 补丁包的参数可能有增减。若某参数不存在，先对照本机 `ros2 param list /节点名` 与插件错误日志，保留发行版记录；不要转抄 Rolling 参数。

### 3.5 完整 Nav2 Launch：显式组织速度链

保存 `~/ros2_week6_ws/src/navigation_lab/launch/nav.launch.py`：

```python
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    filename = Path(LaunchConfiguration('params').perform(context)).expanduser().resolve()
    if not filename.is_file():
        raise FileNotFoundError(filename)
    common = [str(filename), {'use_sim_time': True}]
    specs = [
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_controller', 'controller_server', 'controller_server',
         [('cmd_vel', '/cmd_vel_nav')]),
        ('nav2_behaviors', 'behavior_server', 'behavior_server',
         [('cmd_vel', '/cmd_vel_nav')]),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', '/cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel_smoothed')]),
        ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', []),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
    ]
    nodes = [Node(package=pkg, executable=exe, name=name, output='screen',
                  parameters=common, remappings=remaps)
             for pkg, exe, name, remaps in specs]
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', parameters=[{
            'use_sim_time': True, 'autostart': True, 'bond_timeout': 10.0,
            'node_names': [name for _, _, name, _ in specs]}]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('navigation_lab'))
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=str(share / 'config' / 'nav2.yaml')),
        OpaqueFunction(function=start),
    ])
```

本周自己列出要运行的服务器，便于你看清配置和速度去向。因此不要再额外启动官方 `bringup_launch.py`、`navigation_launch.py` 或另一份地图服务器。

Jazzy 的 BT Navigator 会加载内置 BT 插件；本例不写一长串 `plugin_lib_names`。未指定默认树文件时使用安装包自带的 NavigateToPose 树；本周先理解它编排规划、跟踪和恢复的职责，第七周再细读和修改行为树。

### 3.6 按顺序启动和验收

重新编译。保持第二天的 A/B/C 三个终端；先完成正确初始定位，再开终端 D：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 launch navigation_lab nav.launch.py
```

检查终端 E：

```bash
source ~/ros2_week6_ws/setup_run.bash
for node in planner_server controller_server behavior_server velocity_smoother collision_monitor bt_navigator; do
  ros2 lifecycle get "/$node"
done
ros2 action list -t
ros2 topic info /cmd_vel --verbose
ros2 topic info /cmd_vel_nav --verbose
ros2 topic type /cmd_vel_smoothed
ros2 topic type /cmd_vel_checked
ros2 param get /global_costmap/global_costmap global_frame
ros2 param get /local_costmap/local_costmap global_frame
```

六个管理节点应 active，Action 列表包含 `/navigate_to_pose`，速度话题应为 Twist。`/cmd_vel_nav` 可以有 controller 与 behavior 两个发布端，这是行为树协调下的两个生产者；最终 `/cmd_vel` 仍只能有 motion_gate 一个 ROS 发布者。

RViz 增加/检查以下显示项，名字随意，类型和话题准确即可：

| Display 类型 | Topic | 用途 |
|---|---|---|
| Map | `/map` | 原始静态地图 |
| Map | `/global_costmap/costmap` | 全局代价；Color Scheme 可选 costmap |
| Map | `/local_costmap/costmap` | 局部代价；调低 Alpha 避免遮住地图 |
| Path | `/plan` | 全局路径；未发目标时可为空 |
| Polygon | `/local_costmap/published_footprint` | 查看规划使用的机器人包络 |
| Polygon | `/body_stop`、`/body_approach` | 查看近身和预测检查区域 |
| LaserScan | `/scan` | 实时障碍；Best Effort |

Map 显示晚订阅收不到数据时，检查该发布者的 QoS，再匹配 Reliability/Durability。Collision Monitor 尚未处理速度输入时，碰撞区域显示可能暂时没有数据；第四天发送目标时再观察。把配置保存到 `~/ros2_week6_ws/src/navigation_lab/rviz/navigation.rviz`。以后可以直接 `-d` 这个文件启动 RViz。

**今天先保持 HOLD，不发目标。** 代价地图正确、节点 active、保护链无旁路，再进入第四天。

例题：0.05 m 分辨率的 4 m × 4 m 局部图有多少格？答案：`80 × 80 = 6400`。改成 0.025 m 后是 25600 格；提高分辨率会增加处理负担，不能当作万能修复。

[回到导航](#contents)

<a id="day4"></a>
## 第 4 天：规划、跟踪与第一段自动导航

### 4.1 两个问题、两种算法

NavFn 解决“经过哪些位置去目标”，本例 `use_astar: false` 使用 Dijkstra 式扩展。算法在可通行代价场中搜索，路径未必是欧氏距离最短的直线，因为墙壁、未知与软代价都会影响路线。

Regulated Pure Pursuit，简称 RPP，解决“现在给多大线速度和角速度来跟随路径”。它从路径上寻找前瞻点，根据几何关系转向，并在弯曲、接近障碍和到终点等情况下调节速度。

本周先用 NavFn + RPP，便于把你做过的差速运动学接进来。DWB、MPPI、Smac 等留待第七周对照认识，不要求这周把所有插件都配置一遍。

### 4.2 从前瞻点推导速度

在机器人坐标系中，前瞻点为 `(x_L,y_L)`，距离为 `L`。纯追踪的几何曲率可写成：

\[
\kappa=\frac{2y_L}{L^2},\qquad \omega=v\kappa.
\]

这里假定沿机器人前向构造经过前瞻点的圆弧；实际插件还处理旋转到路径方向、碰撞检查、终点朝向等情形。

例题：`L=0.5 m、y_L=0.1 m、v=0.15 m/s`，得到：

\[
\kappa=\frac{0.2}{0.25}=0.8\ \mathrm{m}^{-1},\qquad
\omega=0.12\ \mathrm{rad/s}.
\]

结合第二个 C++ 项目的轮半径 `r=0.05 m`、轮距 `b=0.30 m`：

\[
\dot\phi_L=\frac{v-\omega b/2}{r}=2.64\ \mathrm{rad/s},\quad
\dot\phi_R=\frac{v+\omega b/2}{r}=3.36\ \mathrm{rad/s}.
\]

右轮更快，机器人向左转。Nav2 发布 `v、ω`，Gazebo 插件再转成轮子运动。这就是导航与差速运动学的连接。

### 4.3 第一个目标：先选近处空地

从头启动时，依次完成：sim → localization → RViz 设置初值 → nav。motion_gate 默认 HOLD。

先确认：地图和扫描对齐；六个 Nav2 节点 active；两张 costmap 正常；没有键盘节点向速度链发送指令。然后解除停车：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: false}'
```

未发送导航目标时，状态可能是 `COMMAND_INVALID_OR_STALE`，这时输出零是正确的。HOLD 解除不等于机器人立即运动。

在 RViz 工具栏选择 **Nav2 Goal**。若工具栏没有，通过 Panels/工具配置添加 `nav2_rviz_plugins/GoalTool` 对应的导航目标工具；不要把“2D Pose Estimate”当作目标工具。使用 Nav2 默认视图通常已经包含它。

在机器人附近 1～2 m、已知空闲、距离墙和箱体至少约 0.7 m 的位置按下鼠标，拖动设置目标朝向后松开。此时观察：

1. 出现绕开障碍的 `/plan`。
2. `/cmd_vel_nav` 出现速度，后面的速度链继续传递。
3. 机器人移动，local costmap 随机器人平移。
4. 接近目标后减速，必要时旋转到目标方向。
5. Action 返回成功，机器人停下。

本场景可优先考虑地图上接近 `(-1,-1)` 的空地，但必须以你实际的 map 坐标和 costmap 为准。第五周的人工参考路线用的是 odom，不能直接把整张表当成本周 map 目标。

### 4.4 路径有了，机器人不动，怎么查

用不同终端分别观察以下话题，每条 echo 看完后 Ctrl+C，再执行下一条：

```bash
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel_smoothed --once
ros2 topic echo /cmd_vel_checked --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /motion_gate/status --once --qos-durability transient_local
ros2 topic echo /collision_monitor_state --once
```

| 观察 | 优先解释 |
|---|---|
| 没路径，planner 有报错 | 目标/起点是否在地图内，是否未知或不可通行 |
| 有路径，nav 速度没有 | controller、局部图、TF、Action 是否仍执行 |
| nav 有速度，smoothed 没有 | 平滑器生命周期、输入 remap、消息类型 |
| smoothed 有速度，checked 为零 | 检查碰撞状态、扫描时间与保护区域 |
| checked 有速度，最终为零 | motion_gate 是否 HOLD、扫描失效、时钟冻结 |
| 最终有速度，模型不动 | base_bridge、Gazebo 话题、仿真是否暂停 |

`PASS` 只代表最终门允许传递当前输入；输入本身可能已被 Collision Monitor 归零。应结合上下游速度和碰撞状态判断。

### 4.5 “到达”包含位置和朝向

本例目标容差是位置 `0.15 m`、角度 `0.15 rad`，后者约 `8.6°`。通常需要两者都满足，才符合这个 Goal Checker 的成功条件。

平面朝向转四元数：

\[
q_x=q_y=0,\quad q_z=\sin(\theta/2),\quad q_w=\cos(\theta/2).
\]

| 方向 | yaw | z | w |
|---|---:|---:|---:|
| +x | 0 | 0 | 1 |
| +y | π/2 | 0.7071068 | 0.7071068 |
| -x | π | 1 | 0 |
| -y | -π/2 | -0.7071068 | 0.7071068 |

`orientation: {w: 1.0}` 表示朝 +x，不表示“忽略方向”。也不能把 yaw 原值直接填进 `orientation.z`。

### 4.6 正常取消与锁定停车

正常结束未完成目标，用 RViz 的 Nav2 面板 **Cancel**。需要立即暂停运动时：

```bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: true}'
```

HOLD 会阻断最终输出，但 Nav2 的 Action 可能仍在执行。应继续 Cancel，确认任务进入终态，再处理问题。不要在未知旧任务仍活跃时直接解除 HOLD，否则它可能继续原任务。

完整退出顺序：取消任务 → 锁定 HOLD → 确认 `/cmd_vel` 为零 → 停止 nav → 停止 localization/RViz → 最后停止 sim。不要靠关闭 Action 客户端窗口来假定服务端停止。

### 4.7 两个小实验

**实验 A：终点朝向。** 在宽敞位置先发朝 +x 的目标，成功后在近似相同位置发朝 +y 的目标。观察机器人主要旋转而不是重新走很远。改变定位初值不是此实验的操作。

**实验 B：前瞻距离。** 保留原始 `nav2.yaml`，复制成 `nav2_lookahead_060.yaml`，只把 `lookahead_dist: 0.45` 改为 `0.60`。停止任务并重启 nav，使用：

```bash
ros2 launch navigation_lab nav.launch.py \
  params:="$HOME/ros2_week6_ws/src/navigation_lab/config/nav2_lookahead_060.yaml"
```

两次采用同一地图、相同起终点和相同初始朝向，分别记录拐弯轨迹与耗时。较长前瞻可能更平滑，也可能更容易切弯；结论以你的实验为准。不要只比较两张不同起点的截图。

如果速度门经常报 LIMITED，说明上游指令持续超过配置上限，要检查 RPP 轨迹曲率/速度和平滑器，不要只把最终限幅抬高。

**当天验收：**至少两次成功到点，其中一次有转弯；能解释位置和朝向容差；执行过一次 Cancel，并确认最后输出为零。

[回到导航](#contents)

<a id="day5"></a>
## 第 5 天：C++ NavigateToPose 客户端与取消

### 5.1 为什么导航适合 Action

导航可能持续几十秒甚至更久。调用者需要知道是否接受、还剩多远、有没有恢复、最终成功还是失败，并能中途取消。因此它适合 Action。

| 部分 | 本周使用的字段/状态 | 含义 |
|---|---|---|
| Goal | `pose`、`behavior_tree` | 目标位姿，可选特定行为树 |
| Feedback | `current_pose`、`distance_remaining`、`number_of_recoveries` | 运行中状态 |
| Result 终态 | SUCCEEDED / ABORTED / CANCELED | ROS Action 层的结束状态 |
| Result 内容 | `error_code` 等，以本机接口为准 | 导航算法层的错误说明 |

请求被接受不代表成功；发出取消请求不代表取消已完成；“预计剩余时间”是估计，不是承诺。先看本机定义：

```bash
ros2 interface show nav2_msgs/action/NavigateToPose
ros2 action info /navigate_to_pose
```

下面客户端只使用 Jazzy 稳定的目标、反馈和通用结果状态；具体错误结合 Nav2 日志与 `ros2 interface show` 查看，避免把不同补丁版新增的字段写成必需条件。

### 5.2 完整 C++ 客户端

保存 `~/ros2_week6_ws/src/navigation_lab/src/navigate_client.cpp`：

```cpp
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using Nav = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<Nav>;
using SteadyClock = std::chrono::steady_clock;

bool latch_hold(const rclcpp::Node::SharedPtr & node) {
  if (!rclcpp::ok()) return false;
  auto service = node->create_client<std_srvs::srv::SetBool>("/motion_gate/set_hold");
  if (!service->wait_for_service(2s)) return false;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future = service->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 2s) !=
      rclcpp::FutureReturnCode::SUCCESS) return false;
  return future.get()->success;
}

int navigate(const rclcpp::Node::SharedPtr & node) {
  const double x = node->declare_parameter<double>("x", -1.0);
  const double y = node->declare_parameter<double>("y", -1.0);
  const double yaw = node->declare_parameter<double>("yaw", 0.0);
  const double cancel_after = node->declare_parameter<double>("cancel_after_s", -1.0);
  const double deadline = node->declare_parameter<double>("deadline_s", 300.0);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
      !std::isfinite(cancel_after) || !std::isfinite(deadline) || deadline <= 0.0) {
    throw std::invalid_argument("Finite parameters and positive deadline_s required");
  }
  auto client = rclcpp_action::create_client<Nav>(node, "/navigate_to_pose");
  if (!client->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node->get_logger(), "Navigation action server unavailable");
    return 3;
  }
  if (node->now().nanoseconds() <= 0) {
    RCLCPP_ERROR(node->get_logger(), "No simulation clock yet");
    return 3;
  }

  Nav::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
  goal.behavior_tree = "";
  auto last_log = SteadyClock::time_point{};
  auto options = rclcpp_action::Client<Nav>::SendGoalOptions();
  options.feedback_callback = [node, &last_log](
      GoalHandle::SharedPtr, const std::shared_ptr<const Nav::Feedback> feedback) {
    const auto now = SteadyClock::now();
    if (now - last_log < 1s) return;
    last_log = now;
    RCLCPP_INFO(node->get_logger(), "remaining=%.2f m recoveries=%d pose=(%.2f, %.2f)",
      static_cast<double>(feedback->distance_remaining),
      static_cast<int>(feedback->number_of_recoveries),
      feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
  };

  auto sent = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, sent, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    const bool held = latch_hold(node);
    RCLCPP_ERROR(node->get_logger(),
      "Goal acknowledgement uncertain; HOLD=%s. Restart navigation before releasing.",
      held ? "confirmed" : "unconfirmed");
    return 6;
  }
  const auto handle = sent.get();
  if (!handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected");
    return 3;
  }
  RCLCPP_INFO(node->get_logger(), "Goal accepted: (%.2f, %.2f, %.2f rad)", x, y, yaw);
  auto result = client->async_get_result(handle);
  const auto started = SteadyClock::now();
  bool requested_cancel = false;
  bool deadline_reached = false;
  auto cancel_started = SteadyClock::time_point{};

  while (rclcpp::ok()) {
    const auto state = rclcpp::spin_until_future_complete(node, result, 100ms);
    if (state == rclcpp::FutureReturnCode::SUCCESS) {
      const auto wrapped = result.get();
      const double elapsed = std::chrono::duration<double>(SteadyClock::now() - started).count();
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "SUCCEEDED wall_elapsed=%.2f s", elapsed);
        return 0;
      }
      if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(node->get_logger(), "CANCELED wall_elapsed=%.2f s", elapsed);
        return deadline_reached ? 4 : 2;
      }
      RCLCPP_ERROR(node->get_logger(), "ABORTED_OR_UNKNOWN wall_elapsed=%.2f s", elapsed);
      return 1;
    }
    if (state == rclcpp::FutureReturnCode::INTERRUPTED) break;
    const auto current = SteadyClock::now();
    const double elapsed = std::chrono::duration<double>(current - started).count();
    const bool timed_cancel = cancel_after >= 0.0 && elapsed >= cancel_after;
    if (!requested_cancel && (timed_cancel || elapsed >= deadline)) {
      deadline_reached = elapsed >= deadline;
      requested_cancel = true;
      cancel_started = current;
      auto cancel = client->async_cancel_goal(handle);
      if (rclcpp::spin_until_future_complete(node, cancel, 5s) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        const bool held = latch_hold(node);
        RCLCPP_ERROR(node->get_logger(), "Cancel acknowledgement missing; HOLD=%s",
          held ? "confirmed" : "unconfirmed");
        return 6;
      }
      const auto reply = cancel.get();
      RCLCPP_INFO(node->get_logger(), "Cancel reply: code=%d goals_canceling=%zu; waiting for terminal result",
        static_cast<int>(reply->return_code), reply->goals_canceling.size());
    }
    if (requested_cancel && current - cancel_started > 10s) {
      const bool held = latch_hold(node);
      RCLCPP_ERROR(node->get_logger(), "No terminal result after cancel; HOLD=%s",
        held ? "confirmed" : "unconfirmed");
      return 6;
    }
  }
  RCLCPP_ERROR(node->get_logger(),
    "Client interrupted. This does NOT confirm server cancellation; use HOLD and RViz Cancel.");
  return 5;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("navigate_client");
  int code = 3;
  try {
    // Spin once so an available /clock can be processed before constructing the goal.
    const auto until = SteadyClock::now() + 5s;
    while (rclcpp::ok() && node->now().nanoseconds() <= 0 && SteadyClock::now() < until) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(20ms);
    }
    code = navigate(node);
  } catch (const std::exception & ex) {
    const bool held = latch_hold(node);
    RCLCPP_ERROR(node->get_logger(), "Exception: %s; HOLD=%s", ex.what(),
      held ? "confirmed" : "unconfirmed");
    code = 6;
  }
  rclcpp::shutdown();
  return code;
}
```

程序的主要顺序：创建客户端 → 等服务器 → 发目标 → 得到 goal handle → 反复处理反馈、等待结果 → 必要时取消自己的目标 → 等最终结果。`spin_until_future_complete` 在等待时仍处理回调，因此能同时看到反馈。

`cancel_after_s` 和 `deadline_s` 使用真实经过的秒数，虚拟机跑得慢时不会按仿真时间自动放大。演示目标没走完但现实已超过 300 秒，可在确认系统正常后把单次期限设为 600 秒。

### 5.3 更新构建并运行

在 `CMakeLists.txt` 的 `ament_package()` **前面**添加：

```cmake
add_executable(navigate_client src/navigate_client.cpp)
ament_target_dependencies(navigate_client rclcpp rclcpp_action nav2_msgs std_srvs)
target_compile_options(navigate_client PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS navigate_client DESTINATION lib/${PROJECT_NAME})
```

编译终端重新构建。运行终端先检查地图、定位、Nav2 和 HOLD，目标必须是你在 RViz 验证过的空地：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 run navigation_lab navigate_client --ros-args \
  -p use_sim_time:=true -p x:=-1.0 -p y:=-1.0 -p yaw:=0.0
```

参数写成小数，例如 `1.0`；本例声明的是 double，写成整数 `1` 在严格类型校验时可能被拒绝。

输出先 accepted，过程中有 remaining，最后必须检查终态。成功目标结束后再观察最终速度：

```bash
ros2 topic echo /cmd_vel --once
```

程序反馈的 `current_pose` 是定位估计，不是 Gazebo 真值；反馈中的 `distance_remaining` 也不是机器人直线离目标的精确误差。

### 5.4 亲手测试取消

选择一个确认可达且需要走 10 秒以上的远一点目标，示例坐标必须先在你的地图上核对：

```bash
ros2 run navigation_lab navigate_client --ros-args \
  -p use_sim_time:=true -p x:=-3.0 -p y:=-1.0 -p yaw:=0.0 \
  -p cancel_after_s:=5.0
```

如果目标尚未完成，约五个现实秒后发取消，随后应看到 CANCELED。若机器人先到达，SUCCEEDED 也正确：成功与取消可能发生竞争，不能只凭“我调用过取消”就把结果写成取消成功。

**不要用 Ctrl+C 替代这个实验。** Ctrl+C 可能只结束客户端；服务端任务还活着。遇到这种情况，从另一个终端锁定 HOLD，再在 RViz Cancel 或停止并重启 nav。取消确认缺失时客户端会尝试 HOLD；若日志是 unconfirmed，必须在另一个终端检查最终速度并处理。

| 退出码 | 本例含义 | 后续任务应如何处理 |
|---:|---|---|
| 0 | 服务端报告成功 | 允许继续下一个目标 |
| 1 | ABORTED 或未知终态 | 停止任务序列，查 Nav2 日志 |
| 2 | 已确认取消 | 按“用户取消”记录，不算成功 |
| 3 | 无服务器、无时钟或被拒绝 | 修复启动/输入条件 |
| 4 | 达到期限后已确认取消 | 记录超时，不算成功 |
| 5 | 客户端中断 | 不推断服务端已停 |
| 6 | 接受/取消状态不确定或异常 | 检查 HOLD，重启导航后再运行 |

**当天验收：**用自己的 C++ 程序成功到点一次、确认取消一次；能解释 goal handle、反馈与最终状态的区别；没有把退出客户端当成取消。

[回到导航](#contents)

<a id="day6"></a>
## 第 6 天：多目标任务与受控失败实验

### 6.1 本周多目标先做成什么样

任务逻辑是：A 成功 → B 成功 → C；任意目标失败或取消 → 停止序列并停车。每次只保留一个正在执行的目标。

本周使用一个简短 Shell 调度器串行调用你写的 C++ 客户端，降低一次引入的复杂度。它是“串行多个 NavigateToPose”，**不等同于** `NavigateThroughPoses` 或 Waypoint Follower。第七周再把任务队列、任务状态和报告进一步收进 C++。

为什么不同时发三个目标？因为第二个目标可能抢占第一个，最后只执行剩下那个，而不是自动按发送顺序排队。

### 6.2 建立自己的目标表

先在 RViz 上逐个验证候选点。下表适用于第五周原始房间、地图与出生点近似对齐的情况；如果你重定义了地图坐标或没有扫描到某处，用你自己的 map 坐标替换。

| 名称 | x / m | y / m | yaw / rad | 用途 |
|---|---:|---:|---:|---|
| A | -1.0 | -1.0 | 0.0 | 近处空地 |
| B | -3.0 | -1.0 | 1.5708 | 左下方宽通道 |
| C | -3.0 | 2.5 | 0.0 | 左上方通道，路径需要考虑 box_b |
| HOME | 0.0 | 0.0 | 0.0 | 返回最初附近 |

目标不必位于所需路径的每个拐角，规划器会选择路径。坐标表记录的是目标位姿，不是手动遥控轨迹。初期先只用 A/B，成功后再加入 C/HOME。

保存 `~/ros2_week6_ws/reports/goals.txt`：

```text
# name x_m y_m yaw_rad
A -1.0 -1.0 0.0
B -3.0 -1.0 1.5708
C -3.0 2.5 0.0
HOME 0.0 0.0 0.0
```

### 6.3 完整任务脚本

保存 `~/ros2_week6_ws/scripts/run_mission.bash`：

```bash
#!/usr/bin/env bash
set -euo pipefail
source "$HOME/ros2_week6_ws/setup_run.bash"
goals_file="${1:-$HOME/ros2_week6_ws/reports/goals.txt}"
test -f "$goals_file"
run_dir="$HOME/ros2_week6_ws/reports/mission_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
cp "$goals_file" "$run_dir/goals.txt"

hold_robot() {
  ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: true}' || true
}
trap hold_robot EXIT
trap 'exit 130' INT TERM

while read -r label gx gy gyaw extra; do
  if [[ -z "${label:-}" || "$label" == \#* ]]; then
    continue
  fi
  if [[ ! "$label" =~ ^[A-Za-z0-9_-]+$ || -z "${gyaw:-}" || -n "${extra:-}" ]]; then
    echo "Invalid goal line; expected: label x y yaw" >&2
    exit 3
  fi
  if ros2 run navigation_lab navigate_client --ros-args \
      -p use_sim_time:=true -p x:="$gx" -p y:="$gy" -p yaw:="$gyaw" \
      -p deadline_s:=300.0 2>&1 | tee "$run_dir/${label}.log"; then
    echo "$label SUCCEEDED" >> "$run_dir/summary.txt"
  else
    code=$?
    echo "$label FAILED exit=$code" >> "$run_dir/summary.txt"
    echo "Mission stopped. Check HOLD and cancel any uncertain active goal." >&2
    exit "$code"
  fi
done < "$goals_file"
echo "Mission completed; HOLD will be latched on exit. Logs: $run_dir"
```

`pipefail` 很重要：客户端失败不能因为 `tee` 保存日志成功而伪装成任务成功。脚本退出时锁定 HOLD；它不会偷偷解除 HOLD。每次任务开始前，你先检查定位、确认没有旧目标，再解除：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: false}'
bash ~/ros2_week6_ws/scripts/run_mission.bash
```

`trap` 尽力停车，但不是“服务一定调用成功”的保证。中断脚本后仍要检查 `/cmd_vel` 和 Action 状态；如果服务端旧目标状态不确定，停止并重启 nav，重新定位核对后再解锁。

### 6.4 失败实验 A：地图外目标

前提：机器人位于宽敞空地，正常导航已经成功过，当前没有未结束的旧目标。上一轮任务脚本会锁定 HOLD，先核对定位并解除，再发送明显超出当前房间地图的目标：

```bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: false}'
ros2 run navigation_lab navigate_client --ros-args \
  -p use_sim_time:=true -p x:=100.0 -p y:=100.0 -p yaw:=0.0 \
  -p deadline_s:=60.0
```

记录是否被拒绝、规划失败后 ABORTED，或在期限到达后取消。具体错误码随实现路径而异；默认行为树也可能执行清图、旋转、等待或后退恢复，所以这不是“机器人绝对不会动”的实验。

验收点是：**没有可达路径时不伪报成功，且任务程序不会继续下一点。** 若行为不符合预期，HOLD + Cancel，再排查。不要通过清除静态墙或允许所有未知区域，强行让此目标“可达”。

### 6.5 失败实验 B：雷达失联

这一项用参数重映射制造独立实验，不必杀进程或修改原有桥接。

1. 先 Cancel、HOLD，停止 nav，复制 `nav2.yaml` 为 `nav2_missing_scan.yaml`。
2. 只将 `collision_monitor → scan → topic` 改成 `/scan_missing_for_test`，其他扫描配置不改。
3. 用该参数文件重新启动 nav，确认所有节点状态，再解除 HOLD。
4. 在宽敞附近发送一个可达目标；观察 `/cmd_vel_nav`、`/cmd_vel_checked` 和最终 `/cmd_vel`。
5. Collision Monitor 没有需要的扫描数据，应阻断输出；任务可能进入恢复/失败，不算导航成功。
6. Cancel、HOLD，停止 nav，恢复原配置并重新启动，完成一次正常目标证明故障已恢复。

```bash
ros2 launch navigation_lab nav.launch.py \
  params:="$HOME/ros2_week6_ws/src/navigation_lab/config/nav2_missing_scan.yaml"
```

这个实验只验证 Collision Monitor 输入缺失的处理。motion_gate 仍从真正的 `/scan` 收到数据，不能把结果描述成“雷达驱动整体失联测试”。准确写明你断开的是哪一条数据关系。

### 6.6 失败实验 C：定位初值不准

保持 HOLD，取消所有目标。在 RViz 把初值平移约 0.5 m 或转错约 20°，观察扫描与地图的偏差。然后恢复正确初值，不在错误定位下继续导航。

记录：何种图形证据说明初值错了？AMCL 是否收敛？如果本轮没有通过运动提供足够观测，不能宣称已经验证“自动恢复定位”。本周只要求你识别并修正错初值。

### 6.7 恢复行为到底能解决什么

| 恢复方式 | 可能有帮助的情况 | 解决不了什么 |
|---|---|---|
| 清理 costmap | 过时的临时障碍残影 | 墙实际存在、地图本身错误 |
| 旋转 | 改变朝向、获得不同观测、尝试脱困 | 缺失 TF、断开的雷达 |
| 后退 | 前方局部受阻且后方确实可通行 | 后方无空间、保护链失效 |
| 等待 | 临时障碍离开 | 永久堵死的通路 |
| 重新规划 | 当前路线被新的障碍占据 | 完全不存在连通路径 |

恢复次数增加不是“系统更聪明”的证据，可能说明定位、代价或速度配置存在问题。首先检查输入，再谈参数优化。

**当天验收：**完成一轮串行多点；失败后后续点不执行；做过地图外目标和一个数据故障实验；记录故障注入、现象、原因和恢复方法。

[回到导航](#contents)

<a id="day7"></a>
## 第 7 天：录包、测量、报告与综合验收

### 7.1 本周 bag 和第五周 bag 的目的不同

第五周只录 SLAM 输入，是为了重新运行 SLAM 比较算法。本周录完整导航过程，是为了复盘定位、路径、速度与任务状态；因此会保存 `/map`、动态 TF 和结果话题。

本周回放用于诊断观察，**不宣称只放这份 bag 就能完整重新执行同一导航任务**。Action 的服务请求、环境响应和闭环控制不等同于普通话题回放。

保存 `~/ros2_week6_ws/src/navigation_lab/config/record_qos.yaml`：

```yaml
/map:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 1
/tf_static:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 100
/motion_gate/status:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 1
```

重新构建或直接使用下面给出的源码路径。完整导航已启动、定位完成、尚未发目标时，开录包终端：

```bash
source ~/ros2_week6_ws/setup_run.bash
ros2 bag record --use-sim-time --include-hidden-topics \
  --qos-profile-overrides-path "$HOME/ros2_week6_ws/src/navigation_lab/config/record_qos.yaml" \
  -o "$HOME/ros2_week6_ws/bags/nav_run_$(date +%Y%m%d_%H%M%S)" \
  /map /tf /tf_static /scan /odom /amcl_pose /plan \
  /global_costmap/costmap /local_costmap/costmap \
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_checked /cmd_vel \
  /motion_gate/status /collision_monitor_state \
  /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status
```

本例不录 `/clock`，回放时由 bag player 生成。先录几秒静止数据，再执行多点任务；任务结束、速度归零后再录几秒，最后 Ctrl+C 停止记录。关闭在线系统前，先执行第 7.4 节的版本与运行参数保存命令，再进入回放。

检查实际目录名并查看信息：

```bash
ls ~/ros2_week6_ws/bags
ros2 bag info ~/ros2_week6_ws/bags/你的实际录包目录
```

上面最后一行是路径占位，必须替换。检查 scan、odom、TF、map、速度与 Action 状态有消息；`/plan` 只有发出有效规划时才会出现，不能把“列出了话题名”当作“录到了数据”。

### 7.2 安全复盘：与在线系统完全分开

先取消任务、HOLD、停止全部在线 sim/localization/nav。重新打开 RViz，设置 `use_sim_time: true`。不要启动任何控制节点或 Gazebo 桥接。

指定实际 bag 目录，下面播放的集合特意不包含最终执行速度 `/cmd_vel`：

```bash
source ~/ros2_week6_ws/setup_run.bash
bag_dir="$HOME/ros2_week6_ws/bags/你的实际录包目录"
ros2 bag play "$bag_dir" --clock 50 --rate 0.5 --delay 3.0 \
  --qos-profile-overrides-path "$HOME/ros2_week6_ws/src/navigation_lab/config/record_qos.yaml" \
  --topics /map /tf /tf_static /scan /odom /amcl_pose /plan \
  /global_costmap/costmap /local_costmap/costmap \
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_checked \
  /motion_gate/status /collision_monitor_state \
  /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status
```

如果 RViz 没有 `/robot_description`，RobotModel 显示可能为空，但地图、扫描、路径与 TF 的复盘仍可进行。本实验已有录制 TF，不需要重新启动 robot_state_publisher 补发一套相同 TF。

这次 TF 的责任与第五周离线重建不同：由 bag 重放当时 AMCL 和底盘产生的 TF，不运行新的 AMCL 或 SLAM 争用同一条边。回放时间倒退后，重新启动 RViz/分析节点再看下一轮，减少旧缓存影响。

### 7.3 至少记录哪些指标

不要只写“导航成功了”。对三到五次正常导航分别记录：

| 指标 | 如何获得 | 如何解释 |
|---|---|---|
| 起点和目标 | 发送前 TF、goals.txt | 用 map 坐标，记录朝向 |
| 终态 | 客户端最终日志 | 成功、失败、取消分别计数 |
| 现实耗时 | 客户端 wall_elapsed | 包含虚拟机速度影响 |
| 恢复次数 | Feedback 中最大 number_of_recoveries | 不是越多越好 |
| 终点估计误差 | 停稳后的 map→base_link 与目标之差 | 定位坐标系中的一致性，不是真值精度 |
| 路径/环境截图 | RViz | 说明路线为何绕行 |
| 停车输出 | `/cmd_vel` 观察 | 取消和终态后是否归零 |

停止后的估计位姿为 `(x_f,y_f,θ_f)`，目标为 `(x_g,y_g,θ_g)`：

\[
e_p=\sqrt{(x_f-x_g)^2+(y_f-y_g)^2},
\]
\[
e_\theta=\left|\operatorname{atan2}\bigl(\sin(\theta_f-\theta_g),
\cos(\theta_f-\theta_g)\bigr)\right|.
\]

例题：目标 `(1,2,0)`，最终估计 `(1.06,1.92,0.08)`，则 `e_p=0.10 m`、`e_θ=0.08 rad≈4.58°`。仅看位置误差不足以评价目标朝向。

通过 `tf2_echo map base_link` 读取停稳位姿。不要把 `/odom` 的位置和 map 目标直接相减，也不要把 AMCL 自己估计的坐标误差写成定位的真实精度。独立真值评估将在后续项目完善时增加。

成功率统计只对你定义的正常可达任务计算；故意发送地图外目标的失败实验单独列出。三次成功只能说明这三次条件下通过，不能写成复杂环境中“成功率 100%”。

### 7.4 保存版本和配置

```bash
source ~/ros2_week6_ws/setup_run.bash
mkdir -p ~/ros2_week6_ws/reports
dpkg-query -W 'ros-jazzy-nav2-*' 'ros-jazzy-navigation2' 'ros-jazzy-ros-gz-*' \
  > ~/ros2_week6_ws/reports/package_versions.txt
ros2 param dump /amcl > ~/ros2_week6_ws/reports/amcl_runtime.yaml
ros2 param dump /controller_server > ~/ros2_week6_ws/reports/controller_runtime.yaml
ros2 param dump /collision_monitor > ~/ros2_week6_ws/reports/collision_runtime.yaml
sha256sum ~/ros2_week6_ws/maps/room_final.yaml ~/ros2_week6_ws/maps/room_final.pgm \
  > ~/ros2_week6_ws/reports/map_checksums.txt
```

参数 dump 需要在相应在线节点仍运行时执行；不要在线系统已关闭后一直等待。它保存运行时实际值，便于区分“编辑了源码 YAML”和“该值真的被节点加载”。

### 7.5 直接填写的报告模板

保存 `~/ros2_week6_ws/reports/week6_report.md`，把 `待填写` 换成你的实测结果：

```markdown
# 第六周定位与导航实验报告

## 实验条件
- 系统/ROS/Gazebo：待填写
- Nav2 软件版本：见 package_versions.txt
- 虚拟机 CPU、内存和仿真运行速度：待填写
- 地图文件和校验值：待填写
- 使用参数文件：待填写
- 地图与第五周原场景是否一致：待填写

## 系统关系
- map→odom 发布者：AMCL
- odom→base_link 发布者：Gazebo 差速插件经桥接
- 导航速度链：待自己写出五段话题
- 最终 /cmd_vel 唯一发布者：待填写
- 初始位姿设置方法与确认依据：待填写

## 正常到点实验
| 编号 | 起点 | 目标 | 终态 | 现实耗时/s | 恢复次数 | 估计位置误差/m | 估计朝向误差/rad |
|---|---|---|---|---:|---:|---:|---:|
| 1 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |
| 2 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |
| 3 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |

## 取消与失败实验
| 实验 | 故障注入/操作 | 观察证据 | 任务结果 | 最终速度 | 恢复方法 |
|---|---|---|---|---|---|
| 取消 | 待填 | 待填 | 待填 | 待填 | 待填 |
| 地图外目标 | 待填 | 待填 | 待填 | 待填 | 待填 |
| Collision Monitor 扫描缺失 | 待填 | 待填 | 待填 | 待填 | 待填 |

## 参数对比
- 只修改的参数：待填写
- 控制不变的条件：待填写
- 两轮结果：待填写
- 观察到的差异及限制：待填写

## 复盘与结论
- bag 目录：待填写
- 关键截图/日志：待填写
- 我能独立解释的故障：待填写
- 仍然不稳定的情况：待填写
- 本周尚未验证的内容：动态人群、实物安全、独立定位真值精度等
```

### 7.6 工程整理

本周应保留 `src/`、环境脚本、任务脚本、地图、目标表、配置、报告和一个代表性 bag。`build/ install/ log/` 可以重新生成；bag 较大，先单独备份，不要不加选择地塞进普通 Git 提交。

如果已有机器人总仓库，就把本周包和报告放进既定结构，不要在多个包内反复 `git init`。提交前查看 `git status`，避免复制的包内嵌 `.git` 导致源码没有真正加入父仓库。地图应成对备份；忽略文件不等于备份完成。

第八周再统一整理成第三个项目，本周先保留清楚的阶段记录即可。推荐最终项目方向名称：**ROS 2 Differential-Drive Robot Mapping and Autonomous Navigation**。

### 7.7 第六周最终验收

- [ ] 本周工作空间从新终端可以构建，复用包来自第六周 install。
- [ ] 地图来自第五周真实建图，YAML 与 PGM 一致。
- [ ] 本周 AMCL 独占 map→odom；没有 SLAM 或静态补零 TF。
- [ ] 能在 RViz 设置正确初始位姿，并解释为什么这样设置。
- [ ] global/local costmap 范围、frame、尺寸合理。
- [ ] Nav2 管理的六个节点 active，Action 服务可用。
- [ ] `/cmd_vel` 只有 motion_gate 发布，所有运动都经过保护链。
- [ ] 完成至少三次正常导航，其中包含转弯和目标朝向调整。
- [ ] C++ 客户端获得成功结果；取消实验获得明确终态。
- [ ] 多目标脚本只在上一目标成功时继续。
- [ ] 地图外目标、扫描输入缺失等实验有证据和恢复步骤。
- [ ] Nav2 未返回成功时，没有把“客户端退出”记成成功。
- [ ] 保存可复盘 bag、版本、参数、地图校验值与报告。
- [ ] 能不看文档解释从目标到轮子运动的主要数据关系。

完成这些就可以进入第七周。**第七周重点是让这套系统稳定、可测量、可解释，不是再重装一遍 ROS 或重写一遍第五周 SLAM。**

[回到导航](#contents)

<a id="appendix"></a>
## 附录 A：按症状排错

| 症状 | 优先检查 | 处理方向 |
|---|---|---|
| 找不到 navigation_lab | 编译与 source | 新终端加载第六周 install |
| 地图加载失败 | YAML image 路径、文件成对 | 保持 PGM/YAML 对应关系 |
| 有地图，没有 map→odom | AMCL active、scan、初值 | 正确设初始位姿，不补静态 TF |
| 雷达与地图整体错位 | 初值、地图来源、雷达外参 | 先静止对齐，再允许运动 |
| map→odom 跳来跳去 | 同边多发布者、错误收敛 | 退出旧 SLAM/AMCL，重查初值 |
| 导航启动卡在 configure/activate | 日志中最先失败的节点 | 检查插件、参数、TF；后续未 active 可能只是连带结果 |
| controller 报插件不存在 | 包安装、完整类名 | RPP 包已安装，类名使用本文 Jazzy 名称 |
| 参数 YAML 不生效 | 节点名与命名空间、启动路径 | param get/dump 证实，必要时重启 |
| 仿真暂停后不能走 | `/clock`、motion_gate 状态 | 恢复时钟；重置导致时间倒退则整套重启 |
| `/cmd_vel` 多个发布者 | 旧 guard、默认 bringup、teleop | 退出多余节点，恢复唯一最终发布者 |
| Twist 与 TwistStamped 不匹配 | 全链路消息类型 | Jazzy 本例明确关闭 stamped |
| 能左转不能右转 | 平滑器 theta 最小值 | 必须允许负角速度 |
| planner 说目标不可达 | 图内、已知、代价、尺寸 | 目标放空地，检查地图连通性 |
| 路径贴墙/绕路过大 | 包络、膨胀、地图质量 | 不减小到真实尺寸以下；一次改一项 |
| 有路径但不动 | 逐段速度链 | 找第一处变零/无消息的组件 |
| 频繁停住又恢复 | 扫描时序、负载、碰撞区域 | 先降低 GUI/计算负载，再看参数 |
| 接近终点一直转 | 朝向、容差、角速度、定位稳定性 | 检查真实目标四元数与 RPP 设置 |
| progress checker 失败 | 是否实际移动、是否只在旋转、被保护阻挡 | 先找不动原因，不直接无限延长期限 |
| 客户端 Ctrl+C 后机器人继续 | 服务端目标仍活跃 | HOLD + Cancel，必要时重启 nav |
| 多点只去了最后一点 | 同时发目标导致抢占 | 串行等待前一目标终态 |
| bag 看不到静态地图/TF | 录制内容、Transient Local QoS | 核对消息数量与录制时发布者状态 |
| 回放里机器人模型不显示 | 未回放 robot_description | 用 TF/扫描看轨迹；不额外制造重复 TF |
| 普通机器人运动正常，复位后异常 | 旧 TF/AMCL 状态、时间倒退 | 退出并重启整套实验，重新初始化 |

排错顺序：**进程与生命周期 → 时钟 → 传感器 → TF/定位 → 地图与代价 → 规划 → 控制与速度链 → 任务逻辑。**

如果仿真很慢，先关闭 Gazebo GUI（`gui:=false`）、降低 RViz 显示负担，确认 GPU 激光仍能正常工作。不要同时修改五个频率，更不要把传感器超时机制全部关闭。

## 附录 B：完成整周后的操作速查

以下每个长时间运行的命令使用独立终端；均先执行 `source ~/ros2_week6_ws/setup_run.bash`。

1. 编译：

```bash
source ~/ros2_week6_ws/setup_build.bash
cd ~/ros2_week6_ws
colcon build --symlink-install --packages-up-to navigation_lab
```

2. 仿真：

```bash
ros2 launch navigation_lab sim.launch.py
```

3. 定位：

```bash
ros2 launch navigation_lab localization.launch.py \
  map:="$HOME/ros2_week6_ws/maps/room_final.yaml"
```

4. RViz，设初始位姿：

```bash
ros2 run rviz2 rviz2 \
  -d "$HOME/ros2_week6_ws/src/navigation_lab/rviz/navigation.rviz" \
  --ros-args -p use_sim_time:=true
```

若尚未保存自己的 RViz 文件，使用第二天的官方默认视图命令。

5. 确认初值正确后启动 Nav2：

```bash
ros2 launch navigation_lab nav.launch.py
```

6. 检查 active、速度链和地图后解除 HOLD：

```bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: false}'
```

7. 发送已检查的目标：

```bash
ros2 run navigation_lab navigate_client --ros-args \
  -p use_sim_time:=true -p x:=-1.0 -p y:=-1.0 -p yaw:=0.0
```

8. 停车：RViz Cancel，随后：

```bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /cmd_vel --once
```

## 附录 C：自测与参考答案

| 问题 | 参考答案 |
|---|---|
| AMCL 和 SLAM 有什么区别？ | 本周 AMCL 在已有地图中估计位姿；SLAM 同时估计地图与轨迹。 |
| map_server 为什么不能负责定位？ | 它加载并发布地图，没有完成传感器与地图匹配的位姿估计。 |
| 谁发布本周 map→odom？ | AMCL；不能与 SLAM、静态补零同时争用。 |
| 初始位姿等于地图 origin 吗？ | 不等于；前者是机器人位置，后者是栅格的摆放位姿。 |
| RViz 2D Pose Estimate 会让仿真机器人瞬移吗？ | 不会，它给定位系统一个初始估计。 |
| 为什么两个 costmap 用不同 frame？ | map 适合全局定位，odom 连续性适合局部运动；具体选择是系统配置的一部分。 |
| inflation_radius 是真实机器人半径吗？ | 不是；它描述障碍代价扩散范围，尺寸由 footprint/robot_radius 表达。 |
| 规划器和控制器分别输出什么？ | 前者主要输出路径，后者根据状态持续输出速度。 |
| 这里 RPP 是否等于以前轮速 PID？ | 不是；RPP 做底盘路径跟踪，轮速 PID 是更低层执行控制。 |
| 为什么需要 Collision Monitor 之外的 motion_gate？ | 补上最终命令时效、扫描/时钟条件、停车锁与持续零输出。 |
| motion_gate 显示 PASS 为什么可能没动？ | 输入可能本身就是零，需看上游碰撞/控制状态。 |
| 关闭客户端等于取消 Action 吗？ | 不等于，要请求取消并确认终态。 |
| Action 被 accepted 等于成功吗？ | 不等于；只是服务端接受了请求。 |
| 连续快速发 A/B/C 等于排队吗？ | 不等于，可能发生抢占；要显式串行管理。 |
| 平滑器 OPEN_LOOP 等于整套导航开环吗？ | 不等于，它只描述平滑器内部的速度反馈模式。 |
| 为什么给目标设置 w=1 仍会转向？ | w=1、其余为零表示目标 yaw=0，需要满足目标朝向。 |
| cancel 返回后为什么还等 result？ | 取消响应和任务终态是不同步骤，还可能与已完成结果竞争。 |
| 回放 bag 为什么不能开新的 AMCL？ | 本次已有录制 map→odom，新 AMCL 会竞争同一 TF 边。 |
| 根据 AMCL 终点计算的误差是真值误差吗？ | 不是，是定位估计与目标的一致性，需要独立真值才能评价真实精度。 |
| 什么时候完成这套基础计划？ | 第八周交付可复现的第三个项目，并能解释、运行和排查主要链路。 |

## 附录 D：官方资料与版本依据

核对日期：2026-09-28。正文给出本周需要的步骤，链接用于追溯与深入阅读。下面使用官方 Jazzy 文档；网页或安装包后续有变化时，以你的 Jazzy 实际接口和版本记录为准。

| 官方资料 | 本周对应内容 |
|---|---|
| [Nav2 AMCL](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/) | frame、激光、差速模型、初始位姿 |
| [Nav2 Costmap 2D](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/) | 全局/局部图、机器人尺寸、插件与命名空间 |
| [NavFn Planner](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/) | Dijkstra/A*、未知区域、目标容差 |
| [Controller Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/) | FollowPath、进度检查、终点检查、Twist 类型 |
| [Regulated Pure Pursuit](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/) | 前瞻、减速、旋转、碰撞预测 |
| [Velocity Smoother](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/) | 速度、加减速度、超时、反馈模式 |
| [Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/) | 检查区域、approach、传感器超时、输入输出 |
| [Behavior Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/) | 旋转、后退、等待与碰撞输入 |
| [BT Navigator](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_bt_navigator/) | 导航 Action、默认树、Jazzy 内置插件加载 |
| [Jazzy NavigateToPose 实现](https://api.nav2.org/nav2-jazzy/html/navigate__to__pose_8cpp_source.html) | 默认行为树、反馈位姿与剩余距离、目标抢占 |
| [Jazzy Collision Monitor Polygon 实现](https://api.nav2.org/nav2-jazzy/html/polygon_8cpp_source.html) | 静态多边形 points 的读取与碰撞预测 |
| [NavigateToPose Jazzy 接口](https://docs.ros.org/en/jazzy/p/nav2_msgs/action/NavigateToPose.html) | Goal/Feedback/Result；网页不可访问时用本机 interface show |
| [ROS 2 Jazzy C++ Action 教程](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) | 客户端、goal handle、异步反馈与结果 |

本讲义的 `motion_gate`、客户端、任务脚本与具体参数组合是本课程的教学实现，不是 Nav2 官方提供的已验收机器人整机配置。

[回到导航](#contents)
