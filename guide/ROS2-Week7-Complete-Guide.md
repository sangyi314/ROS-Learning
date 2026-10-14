# ROS 2 第七周完整自学讲义：稳定导航、行为树与 C++ 多目标任务

> 环境：Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic；C++17。
>
> 承接《ROS2-Week6-Complete-Guide.md》：继续使用第五周房间和地图、第六周 AMCL、NavFn、RPP、Collision Monitor 与 motion_gate，不更换机器人，不增加硬件采购。
>
> 本周成果：看懂并修改一个简化导航行为树；编写能串行执行、取消、失败即停并记录结果的 C++ 任务程序；完成可复现的参数对照实验；形成第八周项目交付所需的证据。

<a id="contents"></a>
## 阅读导航

- [本周安排与剩余时间](#overview)
- [第 1 天：保存基线，建立第七周工程](#day1)
- [第 2 天：读懂行为树，写一个受限恢复策略](#day2)
- [第 3 天：任务数据、状态约束与纯 C++ 测试](#day3)
- [第 4 天：完整 C++ 多目标执行器](#day4)
- [第 5 天：取消、失败、超时与排错](#day5)
- [第 6 天：参数对照与结构化结果分析](#day6)
- [第 7 天：独立复现与项目阶段验收](#day7)
- [附录：速查、自测、官方资料](#appendix)

<a id="overview"></a>
## 本周安排与剩余时间

第六周解决“能不能导航”；第七周解决“为什么成功、为什么失败、同样条件能否重复完成”。你不用这周去实现 SLAM 优化器或重新学习整套 C++，重点是组织已有系统、写清任务逻辑、用数据验证。

| 天 | 核心任务 | 当天交付 | 用时 |
|---|---|---|---:|
| 1 | 复制已验收工程，冻结基线配置 | 独立工作空间、基线记录 | 2～3 小时 |
| 2 | BT 状态、黑板、顺序、重规划、恢复 | 一个简化行为树 XML | 2～3 小时 |
| 3 | 目标文件解析、串行状态约束、测试 | 纯 C++ 核心与测试程序 | 3～4 小时 |
| 4 | Action 串行调用、反馈、取消、报告 | C++ mission_runner | 4～5 小时 |
| 5 | 可达/不可达、取消、超时、故障证据 | 功能验收矩阵 | 2～3 小时 |
| 6 | 单变量对比、重复试验、统计 | 两组实验数据和结论 | 3～4 小时 |
| 7 | 从新终端复现，整理报告和演示 | 第七周验收材料 | 2 小时 |

本周约 **18～24 小时**。完成本周验收后，当前八周基础计划只剩第八周的 **12～16 小时**：整理第三个项目、README、演示视频、复现说明和项目讲解。若第六周还没实做完，这份文档可以先保存，等第六周验收通过再执行；生成讲义不等于学习进度已经完成。

### 本周保留的系统接口

| 项目 | 本周约定 |
|---|---|
| 工作空间 | `~/ros2_week7_ws` |
| 复用包 | `gazebo_robot_lab`、`mapping_lab`、`navigation_lab` |
| 新包 | `mission_lab` |
| ROS 域 / Gazebo 分区 | `42` / `ros2_week7_lab` |
| 仿真世界/机器人 | `week5` / `week5_bot`，保留 SDF 内部名字 |
| 地图 | `~/ros2_week7_ws/maps/room_final.yaml` 与配套 PGM |
| 定位与 TF | AMCL：map→odom；差速插件：odom→base_link；RSP：固定外参 |
| 导航 Action | `/navigate_to_pose` |
| 新任务取消服务 | `/mission/cancel`，类型 `std_srvs/srv/Trigger` |
| 最终停车服务 | `/motion_gate/set_hold`，类型 `std_srvs/srv/SetBool` |
| 新任务文件 | `src/mission_lab/config/mission.txt` |
| 实验目录 | `reports/`、`bags/`、`baseline/` |

速度链保持为 `/cmd_vel_nav → /cmd_vel_smoothed → /cmd_vel_checked → /cmd_vel`，最终 `/cmd_vel` 仍由 motion_gate 独占发布。任务程序只发送导航目标，不直接发布速度。导航取消与锁定停车是两件事，不能互相替代。

### 本周刻意限制的范围

- 多目标采用单个 C++ 进程串行调用 NavigateToPose；不是同时运行多个导航客户端。
- 内层 BT 可做一次有上限的恢复；外层任务程序失败后停止，不再加一套无限重试。
- 本周不实现自动断点续跑、不自动跳过失败点、不自动解除 HOLD。
- 参数对比只改变前瞻距离，其他算法先保持不变。DWB/MPPI/Smac 的系统比较不挤进必做部分。
- 不把 AMCL 自己估计的位置当作独立真值；实物安全和真实定位精度不是本周验证结果。

### 验证说明

讲义里的纯 C++ 核心与测试在编写环境中实际编译运行；Python、XML、YAML 与 Shell 片段做静态检查。ROS 节点与 Nav2 配置按 Jazzy 官方接口核对，但编写环境没有 ROS/Gazebo，**ROS C++ 构建、任务取消和仿真效果仍需在你的 Ubuntu 中按文档验收**。不预先编造成功率、运行时间或实验截图。

<a id="day1"></a>
## 第 1 天：保存基线，建立第七周工程

### 1.1 开始前的门槛

你应该已经能在第六周工程中：

1. 启动仿真、加载第五周地图、设置 AMCL 初值，扫描与地图对齐。
2. 用 RViz 或第六周 C++ 客户端至少完成一次含转弯的到点。
3. 正确取消目标，确认最终速度归零。
4. 解释为什么 `/cmd_vel` 只能由最后的 motion_gate 发布。

如果还卡在“没有 map→odom”或“机器人完全不动”，先按第六周排错表修好。没有能工作的基线，后续参数比较就无法归因。

### 1.2 复制源码和地图，不覆盖旧成果

停止第六周全部在线程序和 bag 回放，在新终端执行：

```bash
(
  set -e
  test -f ~/ros2_week6_ws/src/navigation_lab/src/motion_gate.cpp
  test -f ~/ros2_week6_ws/src/navigation_lab/src/navigate_client.cpp
  test -f ~/ros2_week6_ws/src/navigation_lab/config/nav2.yaml
  test -f ~/ros2_week6_ws/maps/room_final.yaml
  test -f ~/ros2_week6_ws/maps/room_final.pgm
  mkdir -p ~/ros2_week7_ws/src
  for pkg in gazebo_robot_lab mapping_lab navigation_lab; do
    test -f "$HOME/ros2_week6_ws/src/$pkg/package.xml"
    if [ ! -d "$HOME/ros2_week7_ws/src/$pkg" ]; then
      cp -a "$HOME/ros2_week6_ws/src/$pkg" ~/ros2_week7_ws/src/
    fi
  done
  mkdir -p ~/ros2_week7_ws/{maps,bags,reports,baseline}
  mkdir -p ~/ros2_week7_ws/src/mission_lab/{include/mission_lab,src,test,config,behavior_trees,scripts}
  cp -n ~/ros2_week6_ws/maps/room_final.yaml ~/ros2_week7_ws/maps/
  cp -n ~/ros2_week6_ws/maps/room_final.pgm ~/ros2_week7_ws/maps/
)
```

如果你实际使用 `room_v1`，保留真实文件名，并把下面涉及地图的路径一起替换。检查地图 YAML 的 `image:` 指向当前配套图片；不能只复制 YAML 而遗漏图片。

保存 `~/ros2_week7_ws/setup_build.bash`：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=ros2_week7_lab
```

保存 `~/ros2_week7_ws/setup_run.bash`：

```bash
source ~/ros2_week7_ws/setup_build.bash
source ~/ros2_week7_ws/install/local_setup.bash
```

编译终端只加载 build 脚本；运行终端加载 run 脚本。不要让 `.bashrc` 自动 source 第六周的 install。Gazebo 分区不同，不代表 ROS 同名话题已被隔离；旧程序必须退出。

### 1.3 新包与第一阶段构建文件

保存 `~/ros2_week7_ws/src/mission_lab/package.xml`：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>mission_lab</name>
  <version>0.1.0</version>
  <description>Week 7 repeatable navigation missions</description>
  <maintainer email="learner@example.com">learner</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>nav2_msgs</depend>
  <depend>std_srvs</depend>
  <exec_depend>navigation_lab</exec_depend>
  <exec_depend>python3</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

保存 `~/ros2_week7_ws/src/mission_lab/CMakeLists.txt`，第三天会整体替换为带测试的版本：

```cmake
cmake_minimum_required(VERSION 3.8)
project(mission_lab)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config behavior_trees scripts DESTINATION share/${PROJECT_NAME})
ament_package()
```

第六周软件已经安装，本周不用重装 Nav2。确认 `g++`、`cmake`、`colcon` 和 `python3-yaml` 仍可用。

```bash
source ~/ros2_week7_ws/setup_build.bash
cd ~/ros2_week7_ws
colcon build --symlink-install --packages-up-to mission_lab
```

### 1.4 冻结基线

只第一次建立时复制，保留原参数不在其上反复修改：

```bash
cp -n ~/ros2_week7_ws/src/navigation_lab/config/nav2.yaml ~/ros2_week7_ws/baseline/nav2.yaml
cp -n ~/ros2_week7_ws/src/navigation_lab/config/localization.yaml ~/ros2_week7_ws/baseline/localization.yaml
sha256sum ~/ros2_week7_ws/maps/room_final.yaml ~/ros2_week7_ws/maps/room_final.pgm \
  > ~/ros2_week7_ws/reports/map_checksums.txt
dpkg-query -W 'ros-jazzy-nav2-*' 'ros-jazzy-ros-gz-*' \
  > ~/ros2_week7_ws/reports/package_versions.txt
```

本周如果需要修改复制来的源包，只改第七周工作空间；第六周保留为已知基线。不要在实验中顺便升级发行版、换地图、改机器人尺寸。

### 1.5 本周统一启动顺序

每个终端先 `source ~/ros2_week7_ws/setup_run.bash`。下面四条长时间运行命令各用一个终端。

终端 A：

```bash
ros2 launch navigation_lab sim.launch.py
```

终端 B：必须显式传第七周地图，因为复用的 Launch 默认路径仍写着第六周：

```bash
ros2 launch navigation_lab localization.launch.py \
  map:="$HOME/ros2_week7_ws/maps/room_final.yaml"
```

终端 C：

```bash
ros2 run rviz2 rviz2 \
  -d "$HOME/ros2_week7_ws/src/navigation_lab/rviz/navigation.rviz" \
  --ros-args -p use_sim_time:=true
```

若你第六周未保存自己的 RViz 文件，改用安装包默认视图：

```bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix --share nav2_bringup)/rviz/nav2_default_view.rviz" \
  --ros-args -p use_sim_time:=true
```

在 RViz 设置正确初始位姿，扫描与地图对齐，再开终端 D：

```bash
ros2 launch navigation_lab nav.launch.py \
  params:="$HOME/ros2_week7_ws/baseline/nav2.yaml"
```

检查终端 E：

```bash
ros2 pkg prefix navigation_lab
ros2 pkg prefix mission_lab
ros2 topic info /cmd_vel --verbose
ros2 action info /navigate_to_pose
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 topic echo /motion_gate/status --once --qos-durability transient_local
```

确认包来自 week7，最终速度只有 motion_gate 发布。解除 HOLD 后，用第六周客户端去一个已验证目标，成功后重新 HOLD：

```bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: false}'
ros2 run navigation_lab navigate_client --ros-args \
  -p use_sim_time:=true -p x:=-1.0 -p y:=-1.0 -p yaw:=0.0
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: true}'
```

示例点须在你实际 map/costmap 上确认，不同地图坐标不能盲用。记录“原配置在 week7 工作空间仍能跑通”，这就是基线复现。

**当天验收：**独立工作空间构建成功；路径和地图没有暗中落回 week6；基线完成一段导航；保存版本和地图校验值。

[回到导航](#contents)

<a id="day2"></a>
## 第 2 天：读懂行为树，写一个受限恢复策略

### 2.1 行为树不是路径规划算法

行为树，Behavior Tree，决定“什么时候规划、什么时候跟踪、失败后做什么”。规划器决定路径，控制器决定速度；BT 负责调用和组合这些能力。

“节点”有两种含义要分开：ROS 节点是运行中的通信实体；BT 节点是树里的逻辑单元。一个 BT Navigator ROS 节点可以执行许多 BT 节点。

### 2.2 三个运行状态和一次 tick

| 状态 | 含义 | 例子 |
|---|---|---|
| RUNNING | 尚未结束，需要后续 tick | 正在跟踪路径 |
| SUCCESS | 本次逻辑成功完成 | 路径已计算出、已经到点 |
| FAILURE | 本次逻辑失败 | 无法规划出路径 |

还有 IDLE、SKIPPED 等框架状态，本周先抓住上述三种。tick 是父节点询问/推进子节点一次，不等于重新发一次完整导航目标。异步 Action 在 RUNNING 时可以持续反馈，BT 线程不必阻塞等待机器人走完。

| 控制结构 | 本周的理解 |
|---|---|
| Sequence | 依次执行；某步失败则整段失败；普通 Sequence 在运行中继续当前子节点 |
| Fallback | 当前选择失败时尝试下一个选择；成功或运行时不继续向后选 |
| PipelineSequence | 后面执行时可继续 tick 已经过的前序步骤，支持边跟踪边重规划 |
| RateController | 限制对子树的 tick 节奏，不是直接修改底盘控制频率 |
| RecoveryNode | 两个子节点：主任务、恢复；主任务失败后在上限内恢复并重试 |

`number_of_retries="1"` 的含义是允许一次恢复成功后的重试；如果主任务首次就成功，就不恢复。不是无条件执行两次，也不是“允许失败后永远循环”。

### 2.3 黑板与端口

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
<FollowPath path="{path}" controller_id="FollowPath"/>
```

`{goal}`、`{path}` 是黑板变量：前者来自导航请求，规划产生的路径写入后者，再交给跟踪节点。`GridBased`、`FollowPath` 是第六周 YAML 中定义的插件 ID；不是任意中文备注，也不是完整插件类名。

例题：把 `controller_id="FollowPath"` 改成 `controller_id="MyController"`，但 YAML 里没有这个 ID，会怎样？答案：服务请求找不到对应已加载控制器，不能靠改 XML 名字自动生成插件。

### 2.4 先阅读你机器实际安装的默认树

```bash
bt_share="$(ros2 pkg prefix --share nav2_bt_navigator)"
sed -n '1,220p' "$bt_share/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
```

按以下顺序找，不要求一次记住所有标签：根节点 → 主导航分支 → ComputePathToPose → FollowPath → 各自恢复 → 总体恢复。默认树可能包含插件选择、错误码判断、清理 costmap、旋转、等待和后退；本机补丁版 XML 是你的具体实验依据。

第六周 `bt_loop_duration: 10` 单位是毫秒，约为 100 Hz 的 BT 执行节拍；树中的重规划 RateController 即使是 1 Hz，也不等于底盘只每秒更新一次速度。第六周 controller 配置仍为 10 Hz，平滑器为 20 Hz。

### 2.5 写一个本周可解释的受限恢复树

保存 `~/ros2_week7_ws/src/mission_lab/behavior_trees/nav_clear_wait_once.xml`：

```xml
<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="1" name="OneRecoveryOnly">
      <PipelineSequence name="PlanAndTrack">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
                             error_code_id="{compute_path_error_code}"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker"
                    error_code_id="{follow_path_error_code}"/>
      </PipelineSequence>
      <Sequence name="ClearWaitRetry">
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        <Wait wait_duration="1.0"/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

这棵树保持周期重规划；主分支失败后最多清理两张 costmap、等待一次，再重试主分支。它没有自动旋转和后退，但所有正常导航运动仍经过第六周保护链。

简化树故意不包含默认树的完整错误分类、目标更新恢复打断等策略，**不声称比默认树更优，也不是可直接用于真实工业机器人的通用策略**。本周用它理解因果，参数对比时必须保持树不变。

清理 costmap 不会删除 PGM，也不会让墙实际消失。传感器或静态层会重新加入仍然存在的障碍；地图外目标仍不可达。

### 2.6 何时测试它

第四天的新客户端有 `bt_xml` 参数，可以把该文件的绝对路径随目标传给服务端，不必覆盖 Nav2 安装目录的 XML。第二天先完成静态结构检查与手动推演。

| 场景 | 应如何执行 |
|---|---|
| 第一次导航成功 | 直接成功，恢复分支不执行 |
| 首次失败、清理等待成功、第二次导航成功 | 执行一次恢复后成功 |
| 首次失败、恢复分支自己失败 | 整体失败，不继续无限等待 |
| 恢复完成，但第二次导航仍失败 | 整体失败 |

**当天验收：**能解释 `{path}` 在两步之间如何传递；能区分 BT、规划器、控制器；能手动推演上述四种状态路径。

[回到导航](#contents)

<a id="day3"></a>
## 第 3 天：任务数据、状态约束与纯 C++ 测试

### 3.1 为什么先把一部分逻辑从 ROS 中拿出来

“文件里多了一个字段”“失败后不该发下一个点”“同一个点不能重复开始”都不是 ROS 问题。把它们写成普通 C++，不用启动 Gazebo 就能测试，排错范围也更小。

今天只做两件事：读取任务文件；约束串行任务的推进。第四天再连接 ROS Action。不要为了这个小任务引入复杂框架。

保存 `~/ros2_week7_ws/src/mission_lab/config/mission.txt`：

```text
# name  x(m)  y(m)  yaw(rad), frame=map
A     -1.0  -1.0  0.0
B     -3.0  -1.0  1.5708
C     -3.0   2.5  0.0
HOME   0.0   0.0  0.0
```

这些仍是第六周的候选点。先在实际地图上检查：点必须在已知自由区，离障碍留出机器人半径和余量，且有通路。机器人真实起点和 map 原点不一定相同；`HOME` 是标签，不是自动识别的出生点。

为降低解析复杂度，本周文件不是 YAML：每行恰好四项，以空格分隔，`#` 后是注释。名字只接受字母、数字、下划线和连字符，且不可重复。数值必须有限；地图可达性由导航系统判断，解析器不冒充规划器。

### 3.2 完整核心头文件

保存 `~/ros2_week7_ws/src/mission_lab/include/mission_lab/mission_core.hpp`：

```cpp
#pragma once
#include <cmath>
#include <cstddef>
#include <istream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace mission_lab {
struct Waypoint {
  std::string name;
  double x{}, y{}, yaw{};
};

inline bool safe_name(const std::string & s) {
  if (s.empty() || s.size() > 64) return false;
  for (unsigned char c : s) {
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
  }
  return true;
}

inline std::vector<Waypoint> read_mission(std::istream & input) {
  std::vector<Waypoint> result;
  std::set<std::string> names;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    std::istringstream row(line);
    row >> std::ws;
    if (row.eof()) continue;
    Waypoint p;
    std::string extra;
    if (!(row >> p.name >> p.x >> p.y >> p.yaw) || (row >> extra) ||
        !safe_name(p.name) || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.yaw)) {
      throw std::runtime_error("invalid mission line " + std::to_string(line_number));
    }
    if (!names.insert(p.name).second) {
      throw std::runtime_error("duplicate waypoint: " + p.name);
    }
    result.push_back(p);
    if (result.size() > 1000) throw std::runtime_error("too many waypoints");
  }
  if (input.bad()) throw std::runtime_error("mission read failed");
  if (result.empty()) throw std::runtime_error("mission is empty");
  return result;
}

enum class State { Ready, Active, Completed, Failed };

class Cursor {
public:
  explicit Cursor(std::size_t count) : count_(count) {
    if (count == 0) throw std::invalid_argument("empty mission");
  }
  std::size_t begin() {
    if (state_ != State::Ready) throw std::logic_error("not ready to start");
    state_ = State::Active;
    return index_;
  }
  void finish(bool success) {
    if (state_ != State::Active) throw std::logic_error("no active waypoint");
    if (!success) { state_ = State::Failed; return; }
    ++index_;
    state_ = index_ == count_ ? State::Completed : State::Ready;
  }
  State state() const { return state_; }
  std::size_t index() const { return index_; }
private:
  std::size_t count_;
  std::size_t index_{0};
  State state_{State::Ready};
};
}  // namespace mission_lab
```

这里的关键不是类的数量，而是一个不变量：**只有前一个点得到成功终态，游标才能推进；失败没有通向下一个点的路径。** `Active` 时再次 `begin()` 会直接报错。

`std::isfinite()` 排除 NaN/无穷；重复名检查使报告能稳定关联目标；行号帮助找输入错误。目标名限制也让后面的 CSV 不需要处理逗号、引号和换行。

### 3.3 不依赖 ROS 的测试

保存 `~/ros2_week7_ws/src/mission_lab/test/test_mission.cpp`：

```cpp
#include "mission_lab/mission_core.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>

int main() {
  int checks = 0;
  auto check = [&](bool good) {
    if (!good) throw std::runtime_error("check failed: " + std::to_string(checks + 1));
    ++checks;
  };
  auto throws = [&](auto fn) {
    bool caught = false;
    try { fn(); } catch (const std::exception &) { caught = true; }
    check(caught);
  };
  auto parse = [](const std::string & s) {
    std::istringstream input(s);
    return mission_lab::read_mission(input);
  };
  try {
    const auto points = parse("# comment\n A 1 2 0 # note\n\nB -3 4 1.57\n");
    check(points.size() == 2);
    check(points[0].name == "A" && points[0].x == 1 && points[1].y == 4);
    check(mission_lab::safe_name("A-01_home"));
    check(!mission_lab::safe_name("A,B"));
    check(!mission_lab::safe_name(""));
    throws([&] { parse("# empty\n"); });
    throws([&] { parse("A 1 2\n"); });
    throws([&] { parse("A 1 2 0 extra\n"); });
    throws([&] { parse("A bad 2 0\n"); });
    throws([&] { parse("A nan 2 0\n"); });
    throws([&] { parse("A 1e999 2 0\n"); });
    throws([&] { parse("A,B 1 2 0\n"); });
    throws([&] { parse("A 1 2 0\nA 3 4 0\n"); });
    throws([&] { mission_lab::Cursor bad(0); });

    using mission_lab::State;
    mission_lab::Cursor good(2);
    check(good.state() == State::Ready);
    throws([&] { good.finish(true); });
    check(good.begin() == 0 && good.state() == State::Active);
    throws([&] { good.begin(); });
    good.finish(true);
    check(good.index() == 1 && good.state() == State::Ready);
    check(good.begin() == 1);
    good.finish(true);
    check(good.index() == 2 && good.state() == State::Completed);
    throws([&] { good.begin(); });

    mission_lab::Cursor failed(3);
    check(failed.begin() == 0);
    failed.finish(false);
    check(failed.index() == 0 && failed.state() == State::Failed);
    throws([&] { failed.begin(); });
    throws([&] { failed.finish(true); });
    std::cout << "PASS: " << checks << " checks\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
```

直接编译，先不启动 ROS 节点：

```bash
mkdir -p ~/ros2_week7_ws/core_test_build
g++ -std=c++17 -Wall -Wextra -Wpedantic \
  -I ~/ros2_week7_ws/src/mission_lab/include \
  ~/ros2_week7_ws/src/mission_lab/test/test_mission.cpp \
  -o ~/ros2_week7_ws/core_test_build/test_mission
~/ros2_week7_ws/core_test_build/test_mission
```

预期输出 `PASS: 26 checks`，退出码为 0。不要只看“没有报错”；尝试临时破坏一条输入，再解释测试为什么应失败，之后恢复文件。

### 3.4 接入 CMake 测试

把 `CMakeLists.txt` 整体替换为下面内容；第四天只需按说明增加依赖和可执行文件部分：

```cmake
cmake_minimum_required(VERSION 3.8)
project(mission_lab)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_mission test/test_mission.cpp)
  target_include_directories(test_mission PRIVATE include)
  target_compile_options(test_mission PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME mission_core COMMAND test_mission)
endif()

install(DIRECTORY config behavior_trees scripts DESTINATION share/${PROJECT_NAME})
ament_package()
```

```bash
source ~/ros2_week7_ws/setup_build.bash
cd ~/ros2_week7_ws
colcon build --symlink-install --packages-select mission_lab --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mission_lab --event-handlers console_direct+
colcon test-result --verbose
```

`colcon test` 负责运行测试，`test-result` 用于检查结果；编译成功不代表断言通过。若其他包残留测试结果，查看输出的包名，不把无关失败算到新包头上。

**当天验收：**能解释四种状态和非法转换；26 项检查通过；输入错误发生在发目标之前。

[回到导航](#contents)

<a id="day4"></a>
## 第 4 天：完整 C++ 多目标执行器

### 4.1 先把接口和边界说清楚

`mission_runner` 读取任务 → 发送一个目标 → 等待它的 Action 终态 → 写一行报告 → 决定是否发下一个目标。只有成功且没有停止请求才继续。

| 接口/参数 | 含义 |
|---|---|
| `mission_file` | 任务文本绝对路径，必填 |
| `report_file` | 新 CSV 文件绝对路径，必填；已有文件拒绝覆盖 |
| `variant` | 报告中的实验标签，例如 `look045`，默认 `baseline` |
| `bt_xml` | 自定义树绝对路径；空字符串使用服务端默认树 |
| `deadline_s` | 每个目标的墙上时钟上限，默认 300 秒，不是整个任务上限 |
| `/mission/cancel` | 请求结束当前任务；响应只表示已记录请求 |
| `/motion_gate/set_hold` | 结束或异常时请求 HOLD，不自动解锁 |

限制：只运行一个 mission_runner；不要同时在 RViz 点目标、运行旧客户端或旧脚本。导航服务器收到新目标可能发生抢占，不会替你维护本周的任务队列。

本例采用单线程“处理回调 + 检查 future”的组织方式，方便追踪。反馈保存到 `shared_ptr` 状态中，不捕获会提前销毁的局部引用。取消必须针对本客户端拿到的 goal handle，不能用“取消全部目标”省事。

### 4.2 完整节点源码

保存 `~/ros2_week7_ws/src/mission_lab/src/mission_runner.cpp`：

```cpp
#include "mission_lab/mission_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using Node = rclcpp::Node;
using Nav = nav2_msgs::action::NavigateToPose;
using Client = rclcpp_action::Client<Nav>;
using Handle = rclcpp_action::ClientGoalHandle<Nav>;
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;

template<class Predicate>
bool pump_until(const Node::SharedPtr & node, Predicate ready, double seconds) {
  const auto end = Clock::now() + std::chrono::duration<double>(seconds);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (ready()) return true;
    if (Clock::now() >= end) return false;
    std::this_thread::sleep_for(20ms);
  }
  return false;
}

template<class Future>
bool ready_within(const Node::SharedPtr & node, Future & future, double seconds) {
  return pump_until(node, [&] {
    return future.wait_for(0s) == std::future_status::ready;
  }, seconds);
}

bool hold(const Node::SharedPtr & node) {
  if (!rclcpp::ok()) return false;
  try {
    auto client = node->create_client<std_srvs::srv::SetBool>("/motion_gate/set_hold");
    if (!client->wait_for_service(2s)) {
      RCLCPP_ERROR(node->get_logger(), "HOLD service unavailable");
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = client->async_send_request(request);
    const bool confirmed = ready_within(node, future, 2.0) && future.get()->success;
    if (confirmed) RCLCPP_INFO(node->get_logger(), "HOLD confirmed");
    else RCLCPP_ERROR(node->get_logger(), "HOLD NOT confirmed; inspect simulator immediately");
    return confirmed;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "HOLD exception: %s", e.what());
    return false;
  }
}

struct FeedbackData {
  int count{0}, recoveries{0};
  double x{std::numeric_limits<double>::quiet_NaN()};
  double y{std::numeric_limits<double>::quiet_NaN()};
  Clock::time_point last_print{};
};

struct Record {
  std::string status{"UNCERTAIN"};
  std::string stop_reason;
  double wall_s{0.0};
  double sim_s{std::numeric_limits<double>::quiet_NaN()};
  FeedbackData feedback;
};

Record run_one(const Node::SharedPtr & node, const Client::SharedPtr & client,
               const mission_lab::Waypoint & point, const std::string & bt,
               double deadline, const std::shared_ptr<bool> & cancel_requested) {
  Record result;
  auto feedback = std::make_shared<FeedbackData>();
  const auto wall_start = Clock::now();
  const auto ros_start = node->now().nanoseconds();
  auto ros_previous = ros_start;
  bool clock_backwards = false;
  auto finish = [&]() {
    result.wall_s = std::chrono::duration<double>(Clock::now() - wall_start).count();
    const auto ros_end = node->now().nanoseconds();
    if (!clock_backwards && ros_end >= ros_start) result.sim_s = (ros_end - ros_start) * 1e-9;
    result.feedback = *feedback;
    if (*cancel_requested && result.stop_reason.empty()) result.stop_reason = "USER_CANCEL";
    return result;
  };

  Nav::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node->now();
  goal.pose.pose.position.x = point.x;
  goal.pose.pose.position.y = point.y;
  goal.pose.pose.orientation.z = std::sin(point.yaw * 0.5);
  goal.pose.pose.orientation.w = std::cos(point.yaw * 0.5);
  goal.behavior_tree = bt;
  Client::SendGoalOptions options;
  const auto logger = node->get_logger();
  options.feedback_callback = [feedback, logger](
      Handle::SharedPtr, const std::shared_ptr<const Nav::Feedback> msg) {
    ++feedback->count;
    feedback->recoveries = std::max(feedback->recoveries, static_cast<int>(msg->number_of_recoveries));
    feedback->x = msg->current_pose.pose.position.x;
    feedback->y = msg->current_pose.pose.position.y;
    if (Clock::now() - feedback->last_print >= 1s) {
      RCLCPP_INFO(logger, "remaining=%.2f m recoveries=%d", msg->distance_remaining, feedback->recoveries);
      feedback->last_print = Clock::now();
    }
  };
  RCLCPP_INFO(logger, "SEND %s (%.2f, %.2f)", point.name.c_str(), point.x, point.y);
  auto goal_future = client->async_send_goal(goal, options);
  if (!ready_within(node, goal_future, 10.0)) {
    result.stop_reason = "GOAL_ACK_TIMEOUT";
    hold(node);
    return finish();  // Goal may exist on server: never send the next one.
  }
  auto handle = goal_future.get();
  if (!handle) { result.status = "REJECTED"; return finish(); }
  auto result_future = client->async_get_result(handle);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    const auto ros_now = node->now().nanoseconds();
    clock_backwards = clock_backwards || ros_now < ros_previous;
    ros_previous = ros_now;
    if (clock_backwards) result.stop_reason = "CLOCK_BACKWARD";
    else if (*cancel_requested) result.stop_reason = "USER_CANCEL";
    else if (std::chrono::duration<double>(Clock::now() - wall_start).count() >= deadline)
      result.stop_reason = "DEADLINE";

    if (result_future.wait_for(0s) == std::future_status::ready) break;
    if (!result.stop_reason.empty()) {
      RCLCPP_WARN(logger, "Cancel requested: %s", result.stop_reason.c_str());
      auto cancel_future = client->async_cancel_goal(handle);
      hold(node);
      if (ready_within(node, cancel_future, 5.0)) {
        const auto reply = cancel_future.get();
        RCLCPP_INFO(logger, "Cancel reply: %zu goal(s) canceling", reply->goals_canceling.size());
      } else {
        RCLCPP_WARN(logger, "Cancel reply timeout; still wait for terminal result");
      }
      if (!ready_within(node, result_future, 10.0)) {
        RCLCPP_ERROR(logger, "Terminal result unknown; keep HOLD and restart Nav2 before reuse");
        return finish();
      }
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  if (!rclcpp::ok()) {
    result.stop_reason = "ROS_SHUTDOWN";
    return finish();  // Ctrl+C is NOT confirmation of action cancellation.
  }
  switch (result_future.get().code) {
    case rclcpp_action::ResultCode::SUCCEEDED: result.status = "SUCCEEDED"; break;
    case rclcpp_action::ResultCode::ABORTED: result.status = "ABORTED"; break;
    case rclcpp_action::ResultCode::CANCELED: result.status = "CANCELED"; break;
    default: result.status = "UNCERTAIN"; break;
  }
  return finish();
}

class Report {
public:
  explicit Report(const fs::path & path) {
    if (fs::exists(path)) throw std::runtime_error("report already exists: " + path.string());
    if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
    out_.exceptions(std::ios::badbit | std::ios::failbit);
    out_.open(path);
    out_ << "variant,index,label,status,stop_reason,wall_s,sim_s,recoveries,feedback_count,"
            "last_feedback_x,last_feedback_y,goal_x,goal_y,goal_yaw\n";
    out_ << std::fixed << std::setprecision(6);
    out_.flush();
  }
  void add(const std::string & variant, std::size_t index,
           const mission_lab::Waypoint & point, const Record & r) {
    out_ << variant << ',' << index << ',' << point.name << ',' << r.status << ','
         << r.stop_reason << ',' << r.wall_s << ',';
    number(r.sim_s);
    out_ << ',' << r.feedback.recoveries << ',' << r.feedback.count << ',';
    number(r.feedback.x); out_ << ','; number(r.feedback.y);
    out_ << ',' << point.x << ',' << point.y << ',' << point.yaw << '\n';
    out_.flush();
  }
private:
  void number(double value) { if (std::isfinite(value)) out_ << value; }
  std::ofstream out_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Node>("mission_runner");
  int exit_code = 2;
  try {
    const auto mission_path = node->declare_parameter<std::string>("mission_file", "");
    const auto report_path = node->declare_parameter<std::string>("report_file", "");
    const auto variant = node->declare_parameter<std::string>("variant", "baseline");
    const auto bt = node->declare_parameter<std::string>("bt_xml", "");
    const auto deadline = node->declare_parameter<double>("deadline_s", 300.0);
    if (mission_path.empty() || report_path.empty() || !fs::path(mission_path).is_absolute() ||
        !fs::path(report_path).is_absolute()) throw std::runtime_error("use absolute mission/report paths");
    if (!mission_lab::safe_name(variant)) throw std::runtime_error("invalid variant label");
    if (!std::isfinite(deadline) || deadline <= 0) throw std::runtime_error("deadline_s must be positive");
    if (!bt.empty() && (!fs::path(bt).is_absolute() || !fs::is_regular_file(bt)))
      throw std::runtime_error("bt_xml must be an existing absolute file path");
    if (!node->get_parameter("use_sim_time").as_bool()) throw std::runtime_error("set use_sim_time:=true");
    std::ifstream input(mission_path);
    if (!input) throw std::runtime_error("cannot open mission file");
    const auto points = mission_lab::read_mission(input);
    Report report(report_path);

    auto canceled = std::make_shared<bool>(false);
    auto cancel_service = node->create_service<std_srvs::srv::Trigger>("/mission/cancel",
      [canceled](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        *canceled = true;
        response->success = true;
        response->message = "Request recorded; wait for terminal result and HOLD confirmation.";
      });
    (void)cancel_service;
    auto client = rclcpp_action::create_client<Nav>(node, "/navigate_to_pose");
    if (!client->wait_for_action_server(10s)) throw std::runtime_error("navigation action unavailable");
    if (!pump_until(node, [&] { return node->now().nanoseconds() > 0; }, 5.0))
      throw std::runtime_error("no simulation clock");

    mission_lab::Cursor cursor(points.size());
    std::size_t written = 0;
    while (cursor.state() == mission_lab::State::Ready && rclcpp::ok()) {
      rclcpp::spin_some(node);
      if (*canceled) break;
      const auto index = cursor.begin();
      const auto r = run_one(node, client, points[index], bt, deadline, canceled);
      report.add(variant, index, points[index], r);
      ++written;
      const bool continue_ok = r.status == "SUCCEEDED" && r.stop_reason.empty();
      cursor.finish(continue_ok);
      RCLCPP_INFO(node->get_logger(), "%s: %s stop_reason=%s", points[index].name.c_str(),
                  r.status.c_str(), r.stop_reason.c_str());
      if (cursor.state() == mission_lab::State::Ready)
        pump_until(node, [&] { return *canceled; }, 1.0);
    }
    for (std::size_t i = written; i < points.size(); ++i) {
      Record skipped;
      skipped.status = "SKIPPED_AFTER_STOP";
      skipped.stop_reason = *canceled ? "USER_CANCEL" : "MISSION_STOPPED";
      report.add(variant, i, points[i], skipped);
    }
    const bool completed = cursor.state() == mission_lab::State::Completed;
    const bool held = hold(node);
    exit_code = completed && held ? 0 : 2;
    RCLCPP_INFO(node->get_logger(), "MISSION %s; hold=%s; report=%s",
      completed ? "COMPLETED" : "STOPPED", held ? "CONFIRMED" : "UNCONFIRMED", report_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "MISSION ERROR: %s", e.what());
    hold(node);
    RCLCPP_ERROR(node->get_logger(), "If a goal may have been sent without a terminal result, restart Nav2 before releasing HOLD.");
  }
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(), "ROS interrupted: cancellation and HOLD are not confirmed. Use another terminal to stop safely.");
  } else {
    rclcpp::shutdown();
  }
  return exit_code;
}
```

### 4.3 读代码时抓住这六点

1. **`SUCCEEDED` 来自终态，不来自距离显示。** `distance_remaining` 适合观察进度；它不是本客户端的到点判据。
2. **请求取消、服务端接受取消、目标进入终态是三个时刻。** `goals_canceling` 非空也不能替代最终结果。
3. **超时用 `steady_clock`。** Gazebo 暂停时 ROS 时间可能不走，墙上时钟仍可触发停止。CSV 另记 ROS 时间，用于区分仿真慢和算法慢。
4. **失败即停没有例外分支。** 只有成功且 `stop_reason` 为空才推进 Cursor；其他点明确记为 `SKIPPED_AFTER_STOP`。
5. **取消与成功可能竞争。** 如果停止请求到达时服务器刚成功，报告可出现 `SUCCEEDED + USER_CANCEL`，外层仍结束任务，不偷偷发下一个点。
6. **停车请求也必须确认。** `HOLD confirmed` 只确认软件锁定服务响应；最终 `/cmd_vel` 为零、机器人停稳仍要观察，不等同于真实硬件安全认证。

`number_of_recoveries` 是 Nav2 提供的反馈计数，不保证等于 XML 的 `number_of_retries`，也不等于每一种恢复动作各发生一次。报告保留收到的最大值，没有收到反馈时为 0；不要据此断言“完全没有任何恢复”。

CSV 的 `last_feedback_x/y` 只是最后一次反馈采样，不是保证与终态同时刻的最终位姿，也不是 Gazebo 真值。暂时没拿到的数据留空，不用 0 假装测量值。`wall_s` 从发送前计到终态/不确定退出，包含确认和取消等待；不包含点间的 1 秒等待及最后的 HOLD 调用，因此各行相加不是完整任务总时长。

工程限制也要记住：报告文件“存在即拒绝”面向本周单进程运行，不是支持多进程抢占的原子文件锁；异常或进程被杀可能留下不完整报告。后面的统计会校验计划点数，把缺行文件标成未完成，而不是假装全部成功。

### 4.4 补充构建配置

在第三天 `CMakeLists.txt` 的 `find_package(ament_cmake REQUIRED)` 后，加入：

```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
add_executable(mission_runner src/mission_runner.cpp)
target_include_directories(mission_runner PRIVATE include)
target_compile_options(mission_runner PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(mission_runner rclcpp rclcpp_action nav2_msgs std_srvs)
install(TARGETS mission_runner DESTINATION lib/${PROJECT_NAME})
```

构建并重新加载环境：

```bash
source ~/ros2_week7_ws/setup_build.bash
cd ~/ros2_week7_ws
colcon build --symlink-install --packages-select mission_lab
source ~/ros2_week7_ws/setup_run.bash
ros2 pkg executables mission_lab
```

应该看到 `mission_lab mission_runner`。如果编译失败，先处理第一条真正的编译错误，不要被后续连锁错误淹没；不要跳过本地构建就当成源码已经通过 ROS 验证。

### 4.5 首次运行：先默认树，再自定义树

按 1.5 启动系统并正确定位。确认没有别的活动目标；运行前解锁，运行后程序会重新 HOLD。第一次只需任务文件中的 A、B 两点也可以，但另存 `mission_short.txt`，别在参数实验中悄悄改路线。

```bash
source ~/ros2_week7_ws/setup_run.bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: false}'
ros2 run mission_lab mission_runner --ros-args \
  -p use_sim_time:=true \
  -p mission_file:="$HOME/ros2_week7_ws/src/mission_lab/config/mission.txt" \
  -p report_file:="$HOME/ros2_week7_ws/reports/default_tree_run01.csv" \
  -p variant:=default_tree -p deadline_s:=300.0
```

第二次先复位到同样的物理起点并重新定位，再解锁，使用自定义树：

```bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: false}'
ros2 run mission_lab mission_runner --ros-args \
  -p use_sim_time:=true \
  -p mission_file:="$HOME/ros2_week7_ws/src/mission_lab/config/mission.txt" \
  -p report_file:="$HOME/ros2_week7_ws/reports/custom_tree_run01.csv" \
  -p variant:=custom_tree -p deadline_s:=300.0 \
  -p bt_xml:="$HOME/ros2_week7_ws/src/mission_lab/behavior_trees/nav_clear_wait_once.xml"
```

`bt_xml` 是**导航服务器所在机器**能读取的路径；本周客户端和服务器都在同一台 Ubuntu。以后跨主机部署，不是客户端文件存在就能用。文件不存在会在本地预检失败；XML 结构或插件有误则需看 bt_navigator 日志。

已有报告不覆盖：新实验使用新的编号。任务程序不会负责重置世界，也不会因为你改变初始位姿就把机器人实体移回起点。最可靠的重复起点做法见第六天。

**当天验收：**正常任务完成；每点只在前点终态之后发送；报告每个计划点一行；结束时 HOLD 确认；能说清默认树和自定义树的区别。

[回到导航](#contents)

<a id="day5"></a>
## 第 5 天：取消、失败、超时与排错

### 5.1 不只测试“顺利到达”

每次实验使用独立报告名；每次重跑前确认旧目标已终止。下面测试是对第六周仿真软件链的验证，不要照搬到没有独立急停的实物。

| 测试 | 操作 | 预期任务层结果 | 必须观察的证据 |
|---|---|---|---|
| 正常路线 | 已验证的 A、B、C、HOME | 每点 `SUCCEEDED`，任务完成 | 报告、到点顺序、HOLD、最终速度 |
| 不可达目标 | A → 地图外 BAD → C | BAD 失败；C 不发送 | Nav2 原因日志，C 为 skipped |
| 主动取消 | 运动中调用取消服务 | 当前目标最终取消或竞态成功；任务停止 | goal 终态、停止原因、HOLD |
| 单点超时 | 对远目标设很短 deadline | 触发取消；后续不发送 | `DEADLINE` 与真实终态分开记录 |
| 输入错误 | 缺字段/重复名称 | 发目标前拒绝运行 | 解析错误，无 `SEND` 日志 |
| Action 不可用 | 不启动导航，仅保留仿真与 gate | 预检超时退出并请求 HOLD | 无 `SEND`，错误原因明确 |

“预期”是验收标准，不是已经测到的结果。把你实际观察到的状态写入记录；若与预期不同，保留日志排查，不手改 CSV 让它看起来通过。

### 5.2 不可达目标：看外层是否真的停止

另存 `config/mission_unreachable.txt`：

```text
A    -1.0  -1.0  0.0
BAD 100.0 100.0  0.0
C    -3.0   2.5  0.0
```

确认 `(100,100)` 的确在当前房间地图外。按第四天自定义树命令运行，把任务路径改成该文件，报告改成 `fault_unreachable01.csv`，`variant` 改成 `fault_unreachable`。

你应看到 A 完成后才发送 BAD；BAD 无法规划，简化树最多执行一次恢复分支；随后任务停止，C 的报告状态为 `SKIPPED_AFTER_STOP`。常见失败终态是 `ABORTED`；服务端在更早阶段拒绝或其他故障也可能得到不同终态，以日志和 Action 结果为准。

这个测试只能证明“不可达时有界停止”，不能证明恢复对临时障碍有效。要评估恢复效果，需要另外构造确实会消失的阻挡，并控制消失时刻，不算本周必做。

### 5.3 主动取消：服务回复不是停车证明

启动一段需要十几秒以上的可达任务，在另一个已加载 week7 环境的终端执行：

```bash
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
```

然后依次确认：

1. mission_runner 打印停止原因 `USER_CANCEL`。
2. 当前目标有最终结果；通常是 `CANCELED`，临近到点也可能竞争成 `SUCCEEDED`。
3. 后续点没有 `SEND`，报告中明确 skipped。
4. 出现 `HOLD confirmed`，并观察最终速度与实体运动：

```bash
ros2 topic echo /motion_gate/status --once --qos-durability transient_local
ros2 topic echo /cmd_vel
```

观察一段时间内线速度和角速度均为 0、机器人停稳，再结束 `echo`。不要只看一个瞬时零值；服务调用本身也不证明所有下游节点或仿真实体正常响应。

如果在单次目标尚未返回 goal handle 时取消，程序先等确认；拿到 handle 后对自己的目标取消。如果确认超时，报告标记 `UNCERTAIN/GOAL_ACK_TIMEOUT` 并 HOLD。此时客户端无法确认服务端到底有没有开始目标，**不要重新解锁试试看**，按 5.5 重启导航消除未知目标。

### 5.4 超时：使用墙上时钟停止

另存只含远处可达 B 点的 `config/mission_timeout.txt`：

```text
B -3.0 -1.0 1.5708
```

从原起点开始，用第四天命令，改为该任务和 `fault_timeout01.csv`，设置：

```text
-p variant:=fault_timeout -p deadline_s:=0.5
```

远目标不应在半秒内完成；等待停止流程结束。报告应保留 `stop_reason=DEADLINE` 和服务端真实终态，而不是把客户端自己的计时直接写成“服务器失败”。取消回复和终态确认另有等待时间，所以总退出耗时可以大于 `deadline_s`。

可选：在仿真界面暂停物理更新后观察 gate 对停滞时钟的处理。暂停并不等于 ROS 节点退出；本例仍按墙上时间触发取消，若服务端依赖仿真时间而无法及时结束，则记录 `UNCERTAIN`。保持 HOLD，重启在线系统后再使用，不强行继续。

### 5.5 三种状态不能混为一谈

| 状态 | 含义 | 接下来怎么做 |
|---|---|---|
| 终态已确认 + HOLD 已确认 | 当前目标结束且软件速度锁生效 | 检查实际停车，之后可准备下一次独立实验 |
| 终态未知 + HOLD 已确认 | 速度锁住，但服务器可能还认为目标在运行 | 保持 HOLD，停止并重启导航，不发新目标 |
| HOLD 未确认 | 服务无响应或 ROS 已关闭，不能声称已锁住 | 用仍工作的终端请求 HOLD 并观察；必要时停止本次仿真 |

安全停止未知任务的仿真流程：

1. 在另一个终端请求 `/motion_gate/set_hold` 为 `true`，检查状态和零速度。
2. 停止 mission_runner 和 `nav.launch.py`，确认六个导航相关服务节点都退出；不要只杀客户端。
3. 不确定组件状态时，停止本次整套仿真、定位、RViz 与导航，再按 1.5 全新启动。
4. 重新确认定位、生命周期和唯一速度发布者，最后才考虑解锁。

不要把 Ctrl+C 当作任务取消 API。Ctrl+C 可能先使客户端的 ROS context 关闭，程序无法再完成取消或 HOLD 服务调用；motion_gate 对上游命令超时有保护，但若服务端仍持续发有效速度，杀客户端不一定触发该保护。也不要关闭 gate 而让后级插件保留旧速度命令。

### 5.6 日志先按层找

| 现象 | 先看哪层 | 先检查什么 |
|---|---|---|
| 没有 `SEND` | 任务输入/预检 | 文件、参数类型、报告重名、服务器、`/clock` |
| 目标拒绝 | 导航入口/BT | bt_navigator 日志、XML 路径、插件 ID |
| 有路径但不动 | 控制/保护链 | HOLD、scan 新鲜度、四段 cmd_vel、Collision Monitor |
| 没有路径 | 定位/规划/地图 | map→odom、目标自由区、膨胀层、规划器 ID |
| 接近目标来回转 | 控制/目标检查 | 朝向、RPP 参数、姿态容差、角速度限制 |
| 请求取消后一直等 | Action/时间/进程 | 服务端是否存活、仿真是否暂停、终态是否可取 |
| 下一轮立刻不动 | 生命周期/任务锁 | 上一轮结束会 HOLD；先检查旧目标再手动解锁 |

每次记录：现象 → 一条可检验的猜想 → 一项检查 → 证据 → 结论。不要同时改五个参数；那样即使好了也无法解释是哪项起作用。

**当天验收：**至少完成正常、不可达、主动取消、超时四项；另确认一种输入错误在发送前被拒绝。未知状态不算通过，必须记录并恢复到已知状态。

[回到导航](#contents)

<a id="day6"></a>
## 第 6 天：参数对照与结构化结果分析

### 6.1 今天只回答一个问题

在相同地图、机器人、任务点、起点、速度、行为树和保护链下，RPP 的固定前瞻距离由 `0.45 m` 改为 `0.60 m`，会怎样影响任务完成和耗时？

前瞻距离决定跟踪时选取路径上多远处的目标点。较大的值可能让局部转向更平缓，也可能在某些弯道上增加偏离；不存在“越大越好”。本周必须实际测量，不能从概念直接推导哪组会获胜。

本周固定 `use_velocity_scaled_lookahead_dist: false`，确保比较的是这里指定的固定前瞻距离。RPP 在接近目标、曲率变化、障碍附近还可能调节速度，因而实际速度不一定一直等于 `desired_linear_vel`。

### 6.2 生成两个配置，确认只差一个值

保存 `~/ros2_week7_ws/src/mission_lab/scripts/make_variants.py`：

```python
#!/usr/bin/env python3
import argparse
import copy
from pathlib import Path
import yaml

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    base = yaml.safe_load(args.baseline.read_text(encoding="utf-8"))
    controller = base["controller_server"]["ros__parameters"]["FollowPath"]
    if controller["use_velocity_scaled_lookahead_dist"] is not False:
        raise SystemExit("Expected fixed lookahead: use_velocity_scaled_lookahead_dist must be false")
    if controller["plugin"] != "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController":
        raise SystemExit("Expected the Week 6 RPP plugin")
    if abs(float(controller["lookahead_dist"]) - 0.45) > 1e-9:
        raise SystemExit("Baseline is not 0.45 m; inspect your saved baseline first")
    outputs = [("look045", 0.45), ("look060", 0.60)]
    paths = [args.output_dir / f"nav2_{name}.yaml" for name, _ in outputs]
    if any(path.exists() for path in paths):
        raise SystemExit("Output exists; inspect it or choose a new directory (no overwrite)")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for (name, distance), path in zip(outputs, paths):
        variant = copy.deepcopy(base)
        variant["controller_server"]["ros__parameters"]["FollowPath"]["lookahead_dist"] = distance
        with path.open("x", encoding="utf-8") as stream:
            yaml.safe_dump(variant, stream, sort_keys=False, allow_unicode=True)
        print(f"{name}: lookahead_dist={distance:.2f} -> {path}")

if __name__ == "__main__":
    main()
```

执行并检查差异：

```bash
python3 ~/ros2_week7_ws/src/mission_lab/scripts/make_variants.py \
  ~/ros2_week7_ws/baseline/nav2.yaml \
  ~/ros2_week7_ws/src/mission_lab/config/variants
diff -u ~/ros2_week7_ws/src/mission_lab/config/variants/nav2_look045.yaml \
  ~/ros2_week7_ws/src/mission_lab/config/variants/nav2_look060.yaml
```

两份生成文件之间应仅有 `lookahead_dist` 值不同。`diff` 发现差异时退出码为 1 是正常行为。不要拿原始 YAML 的格式差异当成参数差异：重新序列化可能改变注释、引号和排版，但两份生成配置采用相同格式。

### 6.3 六次独立运行，比随手跑两次更有意义

每组至少三次完整尝试，包含失败尝试。建议顺序为：

| 顺序 | 配置/标签 | 报告名 |
|---:|---|---|
| 1 | look045 | `look045_run01.csv` |
| 2 | look060 | `look060_run01.csv` |
| 3 | look060 | `look060_run02.csv` |
| 4 | look045 | `look045_run02.csv` |
| 5 | look045 | `look045_run03.csv` |
| 6 | look060 | `look060_run03.csv` |

这个交错顺序能减少“先跑的一组刚好机器空闲”的偏差，但每组三次样本很少，只能做小规模工程对比，不能声称统计显著或普遍最优。

每次都采用以下流程：

1. 确认上一轮终态和 HOLD，再正常退出上一轮仿真、定位、导航、RViz 和客户端。
2. 按 1.5 全新启动，机器人从相同的 SDF 出生位姿开始。**不要只重设 RViz 的 2D Pose Estimate 来假装实体复位。**
3. 使用相同初始定位位置、朝向与等待标准，检查扫描对齐。手动初始化和 AMCL 随机性仍是误差来源；记录它们，不声称完全确定性复现。
4. 终端 D 使用当前组的参数文件；不要同时启动第二套 nav。
5. 核对实际运行值，再用固定任务和同一棵自定义树运行。
6. 保存结果和异常；某轮启动条件不一致，明确标记无效并说明原因，另补一轮，不能无理由删除失败结果。

以 look060 为例，终端 D：

```bash
ros2 launch navigation_lab nav.launch.py \
  params:="$HOME/ros2_week7_ws/src/mission_lab/config/variants/nav2_look060.yaml"
```

检查实际参数，而不只相信文件名：

```bash
ros2 param get /controller_server FollowPath.lookahead_dist
ros2 param get /controller_server FollowPath.use_velocity_scaled_lookahead_dist
ros2 param get /controller_server FollowPath.desired_linear_vel
ros2 param dump /controller_server > ~/ros2_week7_ws/reports/look060_run01_controller.yaml
```

预期依次为 `0.6`、`false` 和第六周的 `0.15`。保留同样的时间上限；若电脑仿真明显慢，先为所有正式试验统一选择一个足够的上限，再开始整组比较，不临时只给某组放宽。

运行：

```bash
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: false}'
ros2 run mission_lab mission_runner --ros-args \
  -p use_sim_time:=true \
  -p mission_file:="$HOME/ros2_week7_ws/src/mission_lab/config/mission.txt" \
  -p report_file:="$HOME/ros2_week7_ws/reports/look060_run01.csv" \
  -p variant:=look060 -p deadline_s:=300.0 \
  -p bt_xml:="$HOME/ros2_week7_ws/src/mission_lab/behavior_trees/nav_clear_wait_once.xml"
```

look045 组对应修改参数文件、标签和报告名；每次递增编号。`variant` 只是你填写的标签，不会替你切换 Nav2 参数，所以必须保存并检查实际参数。

### 6.4 统计脚本：别让漏写的失败消失

保存 `~/ros2_week7_ws/src/mission_lab/scripts/summarize_runs.py`：

```python
#!/usr/bin/env python3
import argparse
import csv
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

STATUSES = {"SUCCEEDED", "ABORTED", "CANCELED", "REJECTED", "UNCERTAIN", "SKIPPED_AFTER_STOP"}

def finite(value, field, optional=False):
    if value == "" and optional:
        return None
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"non-finite {field}")
    return number

def read_plan(path):
    plan = []
    for line in path.read_text(encoding="utf-8").splitlines():
        cells = line.split("#", 1)[0].split()
        if not cells:
            continue
        if len(cells) != 4:
            raise ValueError("invalid mission file")
        plan.append((cells[0], tuple(finite(v, "goal") for v in cells[1:])))
    if not plan or len({p[0] for p in plan}) != len(plan):
        raise ValueError("empty/duplicate mission")
    return plan

def read_report(path, plan):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty report: cannot infer variant; record this failed attempt separately")
    if len(rows) > len(plan):
        raise ValueError("more rows than planned points")
    variants = {r.get("variant") for r in rows}
    if len(variants) != 1 or not next(iter(variants)):
        raise ValueError("missing/mixed variant")
    for i, row in enumerate(rows):
        if None in row or any(v is None for v in row.values()):
            raise ValueError("malformed/truncated CSV row")
        if int(row["index"]) != i or row["label"] != plan[i][0]:
            raise ValueError("goal order differs from the planned route")
        actual = tuple(finite(row[k], k) for k in ("goal_x", "goal_y", "goal_yaw"))
        if any(abs(a - b) > 1e-5 for a, b in zip(actual, plan[i][1])):
            raise ValueError("goal coordinates differ from the planned route")
        if row["status"] not in STATUSES:
            raise ValueError("unknown status")
        row["wall"] = finite(row["wall_s"], "wall_s")
        row["sim"] = finite(row["sim_s"], "sim_s", optional=True)
        if row["wall"] < 0 or (row["sim"] is not None and row["sim"] < 0):
            raise ValueError("negative time")
        int(row["recoveries"])
        int(row["feedback_count"])
        row["stop_reason"]  # Require this column even for successful goals.
    return next(iter(variants)), rows

def median_text(values):
    return "--" if not values else f"{statistics.median(values):.3f}"

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument("reports", nargs="+", type=Path)
    args = parser.parse_args()
    if len({p.resolve() for p in args.reports}) != len(args.reports):
        parser.error("duplicate report paths")
    plan = read_plan(args.mission)
    groups = defaultdict(list)
    for path in args.reports:
        try:
            variant, rows = read_report(path, plan)
        except (ValueError, KeyError, TypeError, OSError) as error:
            raise SystemExit(f"STOP: {path}: {error}; fix/investigate, do not silently omit") from error
        groups[variant].append(rows)
        if len(rows) < len(plan):
            print(f"INCOMPLETE: {path.name}: {len(rows)}/{len(plan)} rows (counted as incomplete mission)")
    print("variant | completed/runs | success/attempted | success/planned | median_goal_wall_sum_s | median_goal_sim_sum_s")
    for variant, runs in sorted(groups.items()):
        full = [rows for rows in runs if len(rows) == len(plan) and all(
            r["status"] == "SUCCEEDED" and not r["stop_reason"] for r in rows)]
        rows = [r for run in runs for r in run]
        successes = sum(r["status"] == "SUCCEEDED" for r in rows)
        attempted = sum(r["status"] != "SKIPPED_AFTER_STOP" for r in rows)
        planned = len(runs) * len(plan)
        wall = [sum(r["wall"] for r in run) for run in full]
        sim = [sum(r["sim"] for r in run) for run in full if all(r["sim"] is not None for r in run)]
        print(f"{variant} | {len(full)}/{len(runs)} | {successes}/{attempted} | {successes}/{planned} | {median_text(wall)} | {median_text(sim)}")
        failures = Counter(r["status"] for r in rows if r["status"] != "SUCCEEDED")
        reasons = Counter(r["stop_reason"] for r in rows if r["stop_reason"])
        missing = planned - len(rows)
        print(f"  non_success={dict(failures)}, stop_reasons={dict(reasons)}, missing_rows={missing}")
    print("Times use only completed routes and sum goal durations; inter-goal waits/final HOLD are excluded.")
    print("CSV does not prove HOLD/physical stop; also inspect node logs and final velocity.")

if __name__ == "__main__":
    main()
```

仅汇总正式参数试验，不混入故障注入、默认树试跑和不同路线：

```bash
python3 ~/ros2_week7_ws/src/mission_lab/scripts/summarize_runs.py \
  --mission ~/ros2_week7_ws/src/mission_lab/config/mission.txt \
  ~/ros2_week7_ws/reports/look045_run01.csv \
  ~/ros2_week7_ws/reports/look045_run02.csv \
  ~/ros2_week7_ws/reports/look045_run03.csv \
  ~/ros2_week7_ws/reports/look060_run01.csv \
  ~/ros2_week7_ws/reports/look060_run02.csv \
  ~/ros2_week7_ws/reports/look060_run03.csv
```

为什么同时看三个比例？

- `completed/runs`：整条任务真正完成了几次，最贴近多点任务目标。
- `success/attempted`：已经发送的目标中成功多少；单看它会忽略失败后没执行的点。
- `success/planned`：原计划全部目标中完成多少，保留 skipped 和缺失点的影响。

程序因异常退出而只有部分完整行时，统计把它算未完成；空文件、字段损坏或路线不符时直接报错，不默默跳过。如果确实发生这类坏报告，把该尝试作为失败/无效启动另列在实验台账中，说明原因，再决定是否补测；不要只提交“还能正常解析的那些成功文件”。

耗时中位数仅来自完整成功的路线，存在成功样本筛选偏差，必须和完成比例一起看；没有完整成功路线就显示 `--`。CSV 中成功的 Action 也不等于最终 HOLD 确认成功，停车验收要结合终端日志与速度观察。

### 6.5 定位误差、终点误差与轨迹平滑度别混算

本周必做指标是任务完成比例、终态分类和耗时。若你额外测终点误差，先定义清楚测量对象：

- `map→base_link` 与目标的距离，是**估计位姿的到点偏差**；可计算 `hypot(x_est-x_goal, y_est-y_goal)`。
- AMCL 位姿与 Gazebo 独立真值之间的偏差，需要正确的坐标系对齐；没有完成对齐，不报告“定位真实精度”。
- CSV 最后反馈点可能早于停稳，不直接当成最终误差。想测某个点，使用单点任务、确认终态并停稳后再取该点的 TF，注明采样时刻。
- “看起来更顺”不是定量结论。没有固定采样频率、角速度变化定义和相同路径条件时，只能写成观察，不能伪造平滑度分数。

### 6.6 保存必要证据

正式六次实验至少保留报告、运行参数、地图校验、统一任务文件和 BT。若存储有限，选一次代表性成功和一次代表性失败录 bag；不要只保留成功视频。

沿用第六周已经验收的 rosbag 录制命令，把输出目录改为 week7 的新名字，保留其中的 QoS 设置。关键通道包括 map/TF、scan/odom、plan、四段速度、motion_gate 状态、导航 Action feedback/status。不要在同一目录覆盖旧 bag。

回放仍遵守第六周约束：先停止全部在线仿真/导航/控制，检查没有第二个时钟源；采用明确话题白名单，**不回放 `/cmd_vel` 给活着的机器人**。bag 用于观察当时的数据，不等同于重新执行闭环实验；回放时长也不替代原运行的墙上耗时。

### 6.7 写出一段有证据的结论

使用下面模板填写你的实测值，不预填虚构数字：

> 在相同地图、四个目标和同一行为树下，look045 完成 __/3 条任务，look060 完成 __/3 条任务。成功完整路线的各点墙上耗时之和中位数分别为 __ 和 __ 秒，仿真耗时分别为 __ 和 __ 秒。主要失败发生在 __，日志显示 __。因为 __，当前保留 __ 配置。样本量较小，且存在 __ 的控制误差，这个结论只适用于本次条件。

如果两组差异小、三次结果不一致，写“当前证据不足以证明有稳定收益”，完全合格。保留原配置并不是实验失败；能够知道证据支持到哪里，比强行选赢家更重要。

**当天验收：**两组配置只差一个参数；每组至少三次独立尝试；有效/无效/失败都有说明；统计能复算；结论不超出数据。

[回到导航](#contents)

<a id="day7"></a>
## 第 7 天：独立复现与项目阶段验收

### 7.1 合上过程笔记，从新终端复现一次

今天不增加新功能。按你自己写的启动清单，完成一次全新启动、定位、任务执行、结果查看和安全退出。遇到必须靠回忆补充的参数，把它写回清单。

必做顺序：环境 → 仿真 → 定位与初值 → 导航 → 参数/保护链检查 → 确认没有旧目标 → 解锁 → 任务 → 报告 → HOLD/零速度/停稳确认 → 退出。

不要在活动目标执行期间重置仿真时间。需要重新开始时，先结束或锁住任务，正常停止整套进程，再重新启动。保存下来的第六周基线不覆盖；第七周没有证明新参数更好时，可继续使用 look045。

### 7.2 最小交付清单

| 路径（相对 week7 工作空间） | 应有什么 |
|---|---|
| `src/mission_lab/include/mission_lab/mission_core.hpp` | 输入解析与任务状态约束 |
| `src/mission_lab/test/test_mission.cpp` | 26 项纯 C++ 检查 |
| `src/mission_lab/src/mission_runner.cpp` | 串行 Action、取消、超时、报告 |
| `src/mission_lab/behavior_trees/nav_clear_wait_once.xml` | 受限恢复行为树 |
| `src/mission_lab/config/` | 正常/故障任务和两份实验参数 |
| `src/mission_lab/scripts/` | 配置生成和结果统计脚本 |
| `baseline/` | 第六周已验证配置快照 |
| `reports/` | 版本、地图校验、参数快照、逐次报告和总结 |
| `bags/` | 代表性成功/失败证据，可选但推荐 |
| `README_week7.md` | 你的复现说明和结论 |

`build/`、`install/`、`log/` 是可重新生成的构建产物，不是核心源码交付；大型 bag 和视频也不必直接塞进源码仓库。第八周再整理正式项目结构，本周先保证文件对应关系清楚。

### 7.3 README 至少回答八件事

1. 这个项目解决什么问题？本周成果与第六周相比增加了什么？
2. 在什么系统、ROS/Gazebo 版本下验证？如何构建？
3. 完整启动顺序、必须设置的 map 路径和 use_sim_time 是什么？
4. 任务文件、BT、参数配置分别在哪？怎么选择？
5. 哪些条件满足才允许解锁？如何取消？未知终态怎么办？
6. 正常/不可达/取消/超时分别测到了什么？附哪些证据？
7. 参数对比的控制条件、原始结果、统计方法、结论和局限是什么？
8. 换一台机器或从全新终端复现还缺哪些东西？

建议附一张 RViz 截图、一段 1～2 分钟任务视频和一份真实 CSV。视频可以加速，但标明倍速；不要用剪掉失败片段的演示冒充一次完整连续运行。

### 7.4 最终验收表

- [ ] 第六周核心功能在独立 week7 工作空间可复现。
- [ ] `/cmd_vel` 的唯一发布者仍是 motion_gate，没有旁路速度源。
- [ ] 能解释 BT tick、规划频率、控制频率为什么不是一个数字。
- [ ] 能解释黑板、PipelineSequence 和有界 RecoveryNode。
- [ ] 纯 C++ 核心测试通过；ROS 节点在你的 Jazzy 中真实构建通过。
- [ ] 多目标串行执行，只有成功且无停止请求才进入下一点。
- [ ] 不可达点之后不再发送后续目标。
- [ ] 主动取消与超时都有明确终态/不确定状态记录。
- [ ] 所有任务结束后检查 HOLD、最终零速度与实体停稳。
- [ ] 两组至少三次独立参数试验，失败没有被悄悄排除。
- [ ] 汇总脚本能从原始报告复算，耗时口径和缺失数据说明清楚。
- [ ] 已形成不夸大精度、成功率和泛化能力的结论。

没通过的项目写成“未完成 + 当前证据 + 下一步检查”。如果还没有稳定多点运行，先把第三/四/五天补完，第六天可以顺延，不为赶七天而省略停止验收。

### 7.5 到底还要多久完成原计划

以实际验收为准，不以“已经拿到第几周文档”为准：

| 你现在实际完成到哪里 | 剩余必做内容 | 预计净学习时间 |
|---|---|---:|
| 第六周已通过，第七周尚未开始 | 本周 + 第八周 | **30～40 小时** |
| 第七周已通过 | 第八周项目整理与交付 | **12～16 小时** |
| 第六周仍有阻塞项 | 先补第六周，再执行上面内容 | 上述时间再加未完成部分 |

如果每周学习 6 天、每天 3～4 小时，从“第六周已通过”算起约再 **2～3 周**比较稳妥；遇到环境或导航故障另留 1 周缓冲。若每天只能 2 小时，30～40 小时约是 15～20 个学习日。不是到某个日历日期就必然完成，最终看验收。

第八周不再突然塞进一门新算法，主要做：整理第三个机器人项目；写环境和复现说明；补测试与已知限制；录完整演示；准备 5～8 分钟的项目讲解和常见追问。

完成八周基础计划的含义是：你已经有从仿真、定位导航到 C++ 任务逻辑和工程证据的一个完整项目基础。它不等于已经掌握所有机器人算法，也不保证某种岗位录用。EKF、ros2_control、实车部署和更深入的控制/规划算法另立下一阶段，不挤进本次完成时间。

[回到导航](#contents)

<a id="appendix"></a>
## 附录：速查、自测、官方资料

### A. 常用检查命令

以下均在已经加载 `~/ros2_week7_ws/setup_run.bash` 的终端执行：

```bash
ros2 node list
ros2 action info /navigate_to_pose
ros2 interface show nav2_msgs/action/NavigateToPose
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 param get /controller_server FollowPath.lookahead_dist
ros2 topic info /cmd_vel --verbose
ros2 topic echo /motion_gate/status --once --qos-durability transient_local
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
ros2 service call /motion_gate/set_hold std_srvs/srv/SetBool '{data: true}'
```

不要把整段“检查命令”无脑连跑当启动脚本：最后两条会请求取消和锁定，按实际需要执行。

### B. 最容易混淆的六组概念

| 不能混为一谈 | 本周判断方式 |
|---|---|
| 收到目标 / 完成目标 | goal handle 非空只表示接受，最后看 result code |
| 取消请求响应 / 取消终态 | 等待当前 handle 的最终结果 |
| 客户端退出 / 服务端停止任务 | 客户端退出可能留下活动目标 |
| HOLD / Action 终止 | 锁速度不自动取消服务器任务 |
| 最后反馈位姿 / 最终真值位姿 | 采样时刻、数据来源和坐标系都不同 |
| 一次到点 / 稳定任务系统 | 还需失败、取消、超时和重复实验 |

### C. 十个口头自测问题

1. 为什么只在 `SUCCEEDED` 后推进任务还不完全够，代码还检查 `stop_reason`？
2. `RecoveryNode number_of_retries="1"` 最多意味着多少次主分支尝试？如果恢复分支自己失败呢？
3. PipelineSequence 和普通 Sequence 在后续节点 RUNNING 时有什么差别？
4. 黑板中的 `{path}` 是怎么被生成和消费的？
5. 为什么取消回复中列出了目标，也必须继续等终态？
6. 为什么 deadline 使用 steady clock，而报告同时保留 ROS 时间？
7. 为什么不能在 RViz 重设初始位姿就声称机器人实体回到了同一起点？
8. 为什么必须同时报告整任务完成率、成功/已尝试和成功/计划？
9. 为什么清理 costmap 不会让地图中的墙永久消失？
10. 如果 goal 确认超时，你为什么不能直接发下一个点？

答案要点：

1. 取消/超时可能与成功竞争；外层停止意图也必须被尊重。
2. 初次 + 一次重试；恢复分支失败会结束，不保证一定重试。
3. PipelineSequence 会重新 tick 前面已经通过的分支，使规划可在跟踪期间更新；普通顺序节点语义不同。
4. ComputePathToPose 写入，FollowPath 读取同一黑板条目。
5. 接受取消不等于执行已结束，且可能与到点竞争。
6. 仿真暂停不能让墙上超时失效；ROS 时间有助于区分仿真速率差异。
7. 初始位姿消息改变定位估计，不搬动 Gazebo 实体。
8. 只看已尝试目标会漏掉停止后未执行的计划点。
9. 静态地图/仍存在的传感器障碍会重新进入图层。
10. 服务端可能已经接受，只是客户端没拿到确认；继续发会产生未知抢占/任务状态。

### D. 官方资料（按 Jazzy 阅读）

资料用于核对接口与行为，学习顺序仍按本讲义。官网目录可能重定向，打开后确认发行版标为 Jazzy；不要把 Rolling/Kilted 的参数直接混入本周。

- [Nav2：导航行为树详细讲解](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/detailed_behavior_tree_walkthrough/detailed_behavior_tree_walkthrough/)
- [Nav2：默认到点恢复树](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/trees/nav_to_pose_recovery/)
- [Nav2：PipelineSequence 节点](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/bt_plugins/controls/PipelineSequence/)
- [Nav2 Jazzy：PipelineSequence 实现](https://api.nav2.org/nav2-jazzy/html/pipeline__sequence_8cpp_source.html)
- [Nav2 Jazzy：RecoveryNode 实现](https://api.nav2.org/nav2-jazzy/html/recovery__node_8cpp_source.html)
- [Nav2：ComputePathToPose BT 节点](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/bt_plugins/actions/ComputePathToPose/)
- [Nav2：FollowPath BT 节点](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/bt_plugins/actions/FollowPath/)
- [Nav2：RPP 参数说明](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- [Nav2 Jazzy：NavigateToPose 导航器实现](https://api.nav2.org/nav2-jazzy/html/navigate__to__pose_8cpp_source.html)

阅读目标不是把所有源码看完，而是能用自己的话回答：**什么时候发目标、谁负责路径、谁负责速度、失败在哪里结束、证据存在哪里。**

[回到导航](#contents)
