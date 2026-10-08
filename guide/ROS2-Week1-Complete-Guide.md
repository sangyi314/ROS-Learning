# ROS 2 第一周完整自学手册

> 适用对象：已经学过 C++、Linux、Git、CMake，并完成多线程数据管线与差速机器人仿真项目的学习者。  
> 实践环境：Ubuntu 24.04 Desktop、Bash、ROS 2 Jazzy，所有节点运行在同一台 Ubuntu 虚拟机内。  
> 安排：6 天正式学习，每天约 2.5～3 小时；第 7 天可用于复习和补做。首次下载、安装及网络排障可能额外耗时。  
> 本周结果：能独立启动、观察、控制、配置、记录和回放一个 ROS 2 系统，具备下周编写 C++ 节点的基础。  
> 内容核对日期：2026-09-22。

这是一份可以按顺序学习的讲义，包含知识解释、完整命令、操作前提、预期现象、例题、练习、答案和排错方法。末尾的来源用于追溯与扩展，完成本周不要求打开它们。

本手册的命令、接口和关键行为已对照 Jazzy 官方教程及对应源码检查；编写环境没有安装 ROS 2，也没有桌面图形会话，因此这里的结果是依据接口与实现给出的**预期结果**，不是声称在你的虚拟机上实测得到的记录。终端中的时间戳、编号、节点数量和测得频率可能不同。

## 目录

- [使用方法与本周安排](#start)
- [第 1 天：环境与第一次通信](#day1)
- [第 2 天：节点、功能包与通信图](#day2)
- [第 3 天：话题、消息与运动控制](#day3)
- [第 4 天：服务、参数与配置文件](#day4)
- [第 5 天：Action、launch 与日志](#day5)
- [第 6 天：录制回放与综合实验](#day6)
- [第一周自测与参考答案](#exam)
- [常见问题排查](#troubleshooting)
- [命令速查与术语表](#cheatsheet)
- [验收标准与下周衔接](#finish)
- [核对来源](#sources)

<a id="start"></a>

## 使用方法与本周安排

### 你应该怎样学

每一小节都按“读解释 → 预测结果 → 输入命令 → 对照现象 → 回答问题”的顺序完成。已经会 Linux，不需要重复学一遍 Linux；遇到命令中的新符号，本手册会在第一次使用时解释。

不要把整份文档的所有命令一次性粘贴到终端。某些命令会一直运行，直到按下 `Ctrl+C`；还有些命令必须在不同终端同时运行。

| 天数 | 主线 | 当天必须做出的结果 | 建议用时 |
|---|---|---|---|
| 第 1 天 | 安装、环境、talker/listener、turtlesim | 两个节点通信，小海龟能被键盘控制 | 150～180 分钟，下载另计 |
| 第 2 天 | package、executable、node、graph、重映射 | 能找出谁向谁发消息，并控制指定海龟 | 150 分钟 |
| 第 3 天 | Topic、Message、Twist、Pose、发布频率 | 用命令完成直行、转向、圆周运动、停止 | 180 分钟 |
| 第 4 天 | Service、Parameter、YAML | 创建海龟、传送、改画笔、保存并加载配置 | 180 分钟 |
| 第 5 天 | Action、launch、日志 | 发送和取消任务，同时启动两个独立仿真 | 180 分钟 |
| 第 6 天 | rosbag2、综合实验、排障 | 录制圆周运动并回放，完成系统说明 | 180 分钟 |

本周暂不要求自己编写 `rclcpp` 节点、实现导航算法或配置 Gazebo。你会使用已经编译好的程序来理解 ROS 2 系统，下一周再写它们。

### 终端约定

| 名称 | 通常负责什么 |
|---|---|
| 终端 A | 运行仿真器，例如 `turtlesim_node` |
| 终端 B | 键盘控制、持续发布，或者录制数据 |
| 终端 C | 查看节点、调用服务、修改参数 |
| 终端 D | 按需增加，例如同时查看话题或发送指令 |

`终端 A` 是文档给窗口起的名字，不是需要输入的命令。每次换天学习，都可以先把上一天的实验进程在各自终端用 `Ctrl+C` 结束，再从该天的“准备”开始。

所有命令在 **Ubuntu 的终端**输入。`Ctrl+Alt+T` 打开终端。VSCode 集成终端也可以；键盘遥控时要让焦点停在运行遥控程序的终端。

代码块的阅读规则：

- `bash`：可以输入终端的命令，按本小节顺序执行。
- `text`：输出示意、接口结构或模板，不能直接当成命令执行。
- `yaml`：配置文件内容，需要保存到指定文件。
- `python`：launch 文件内容，保存为指定 `.launch.py` 文件。
- 行末的 `\` 表示命令接到下一行，后面不要加空格或注释；可以整块复制。
- `Ctrl+C` 表示同时按键，不需要输入这几个字符。

### 全周统一工作目录

第 1 天会创建 `~/ros2_week1`。`~` 代表你的 Ubuntu 用户主目录，例如 `/home/你的用户名`。

| 路径 | 作用 |
|---|---|
| `~/ros2_week1/setup_env.bash` | 各终端统一环境 |
| `~/ros2_week1/config` | 参数 YAML 文件 |
| `~/ros2_week1/launch` | 启动文件 |
| `~/ros2_week1/bags` | 录制数据 |
| `~/ros2_week1/notes` | 知识与实验记录 |

这个目录是学习材料目录，还不是需要用 `colcon` 编译的 ROS 2 工作空间。

<a id="day1"></a>

## 第 1 天：环境与第一次通信

### 1.1 先理解 ROS 2 在你项目中的位置

ROS 是 Robot Operating System 的缩写。ROS 2 通常运行在 Ubuntu 等操作系统之上，提供机器人软件常用的通信、接口、工具和开发框架。

你之前可以把运动学、编码器、里程计、PID 都放进同一个 C++ 程序，通过函数调用交换数据。系统变大后，传感器驱动、控制、定位、导航、可视化可能需要分别开发和运行。ROS 2 让这些模块按照约定的接口互相协作。

| 已有经验 | 在 ROS 2 中可以怎样延伸 |
|---|---|
| `SensorData` 结构体 | 用具有固定字段的消息传递数据 |
| 生产者与消费者线程 | 把数据产生与处理逻辑做成节点，通过话题连接 |
| 命令行选项 | 用节点参数调整配置 |
| CSV 日志 | 用 rosbag2 记录带类型和时间信息的话题数据 |
| 差速机器人中的速度与姿态 | 通过标准接口与其他机器人软件连接 |

这只是功能类比：**节点不是线程，Topic 也不等于你写的 `ThreadSafeQueue`。** 节点可以在不同进程、不同计算机中运行，消息的传递规则由 ROS 2 通信机制管理。

本周实验里的三个角色：

| 角色 | 做什么 |
|---|---|
| `turtlesim_node` | 模拟一个平面运动的小海龟并显示窗口 |
| `turtle_teleop_key` | 读取键盘输入，发送运动指令 |
| `ros2` 命令工具 | 启动程序、观察接口，也可以临时充当通信参与者 |

### 1.2 检查 Ubuntu 版本和网络

在 Ubuntu 终端运行：

```bash
cat /etc/os-release
uname -m
```

本手册的安装步骤要求 `VERSION_ID="24.04"`。普通 Windows 电脑的 VMware 虚拟机通常显示 `x86_64`。

如果你已经在 Ubuntu 24.04 中安装了 Jazzy，可以直接到 **1.6** 配置环境，再用 **1.7** 验证。

如果当前是 Ubuntu 22.04 或其他版本，不要把下面的 Jazzy 安装命令直接套进去。为了严格按本手册学习，可以保留原虚拟机，另建 Ubuntu 24.04 Desktop 虚拟机：从 [Ubuntu 24.04 官方下载页](https://releases.ubuntu.com/24.04/)选择 Desktop amd64 ISO，在 VMware 的“新建虚拟机”中选择它，按安装器完成安装后再继续。不要把 Windows 物理磁盘作为安装目标；只使用新建虚拟机自己的虚拟磁盘。

第一周可以先给虚拟机分配 2～4 个虚拟 CPU、4～8 GB 内存，具体不要超过宿主机能承受的资源。这里是学习环境建议，不是硬性配置要求。

检查能否解析和连接网站：

```bash
getent hosts packages.ros.org
```

能看到 IP 地址，表示该域名解析成功，但不等于所有下载地址都能连接。如果虚拟机整体没网，先看附录“虚拟机不能联网”。安装需要联网，安装完成后的本周本机通信练习通常不需要互联网。

### 1.3 准备系统语言与基础工具

先查看字符编码：

```bash
locale
```

只要正在使用 UTF-8，例如 `zh_CN.UTF-8`、`en_US.UTF-8` 或 `C.UTF-8`，就可以继续，不要求把中文桌面改成英文。

安装必要工具：

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl python3
sudo add-apt-repository -y universe
```

`sudo` 表示用管理员权限执行；输入密码时终端不显示星号是正常现象。`apt update` 更新软件包索引，不等于把所有软件升级。

**只有在没有 UTF-8 locale 时**，再运行：

```bash
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
locale
```

### 1.4 添加官方 ROS 2 软件源

Jazzy 官方安装流程使用 `ros2-apt-source` 配置软件源与签名密钥。[S1]、[S2]

下面这一整块在同一个终端执行。它只下载并安装用于配置 ROS 软件源的官方 `.deb` 包，不是安装全部 ROS 2。

```bash
(
  set -euo pipefail

  ros_week1_apt_version="$(
    curl -fsSL --retry 3 \
      https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])'
  )"

  if [ -z "$ros_week1_apt_version" ]; then
    echo "没有取得 ros2-apt-source 版本，请检查网络。"
    exit 1
  fi

  curl -fL --retry 3 \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_week1_apt_version}/ros2-apt-source_${ros_week1_apt_version}.noble_all.deb" \
    -o /tmp/ros2-week1-apt-source.deb

  sudo dpkg -i /tmp/ros2-week1-apt-source.deb
)
```

你只需要理解这三步：查询官方包版本 → 下载对应 Ubuntu 24.04 的包 → 安装软件源配置。

补充理解：`$(...)` 取出命令输出；`|` 把前一条命令的输出交给后一条；外面的圆括号创建子 shell，因此其中的严格错误设置不会影响你接着使用的交互终端。

**如果出现下载错误、JSON 解析错误或 `dpkg` 报文件无效，先解决本步骤，不要继续安装。** 常见原因是网络无法访问 GitHub/API，或者下载响应不是软件包。不能通过把 `noble` 改成不匹配的系统代号解决网络问题。

### 1.5 安装本周软件

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-turtlesim \
  ros-jazzy-rqt-graph \
  ros-jazzy-rqt-console \
  ros-jazzy-rosbag2-storage-mcap \
  ros-dev-tools
```

`apt upgrade` 会列出待升级软件并要求你确认；如果系统提示需要重启，完成安装后重启再继续。下载速度取决于网络，不要把等待安装当作没学会。

| 软件包 | 本周用途 |
|---|---|
| `ros-jazzy-desktop` | ROS 2、常见工具、演示程序及图形工具 |
| `ros-jazzy-turtlesim` | 小海龟仿真及接口 |
| `ros-jazzy-rqt-graph` | 查看节点和话题连接 |
| `ros-jazzy-rqt-console` | 查看日志 |
| `ros-jazzy-rosbag2-storage-mcap` | 以 MCAP 格式存储录制数据 |
| `ros-dev-tools` | 后续开发与编译工具 |

有些组件已被 desktop 间接安装，再列出不会重复安装一份。若遇到依赖冲突，查看附录，不要随便删除系统库。

### 1.6 统一所有终端的环境

运行一次，创建学习目录和环境脚本：

```bash
mkdir -p ~/ros2_week1/{config,launch,bags,notes}

cat > ~/ros2_week1/setup_env.bash <<'EOF'
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
EOF
```

解释：

- `source /opt/ros/jazzy/setup.bash`：让当前终端知道 ROS 2 的命令、包与库在哪里。
- `ROS_DOMAIN_ID=42`：所有本周节点使用同一个通信域；42 是本手册选择的编号，没有特殊含义。
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`：本周只在同一台 Ubuntu 系统里发现节点，减少和同网络其他人的实验互相影响。
- `unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS`：移除旧的发现设置与显式远端节点列表，避免与本周单机实验设置混用。
- `'EOF'`：告诉 shell 按原样把中间内容写进文件。

**以后每新开一个终端，先运行下面两行：**

```bash
source ~/ros2_week1/setup_env.bash
cd ~/ros2_week1
```

然后验证：

```bash
echo "$ROS_DISTRO"
echo "$ROS_DOMAIN_ID"
ros2 --help
```

前两条应该分别显示 `jazzy`、`42`，第三条显示命令帮助。这种做法不需要修改 `~/.bashrc`，也不会把其他 ROS 版本写进同一个自动启动配置。

环境变量属于当前进程及其后续启动的子进程。**在终端 A 中 source，不会自动配置已经打开的终端 B。** 改完通信环境后，已经运行的旧节点也需要重新启动才能统一。

### 1.7 例题 1：让两个节点通信

**目标：**先验证安装，再理解发布者与订阅者。

终端 A：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run demo_nodes_cpp talker
```

终端 B：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run demo_nodes_cpp listener
```

预期现象：A 持续输出正在发布字符串的日志，B 持续输出收到字符串的日志，计数会增加。具体文字和编号不是验收重点。

终端 C：

```bash
source ~/ros2_week1/setup_env.bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter --once
```

预期能看到 `/talker`、`/listener`、`/chatter`，最后一条收到一条消息后退出。输出示意：

```text
data: 'Hello World: 7'
---
```

发生了什么？`talker` 创建发布者，把一个 `std_msgs/msg/String` 消息送到 `/chatter`；`listener` 订阅该话题，在收到消息时处理它。`topic echo` 也创建了订阅者，所以它同样能看到数据。

**操作对照：**在 A 中按 `Ctrl+C`，观察 B 不再收到新数据；B 没有自动退出，因为它仍在等待。重新启动 A，等待通信发现完成后，B 应继续收到消息。

结束后，在 A、B 中分别按 `Ctrl+C`。如果 A 已停，不要重复输入退出指令。

### 1.8 例题 2：启动小海龟并用键盘控制

终端 A：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtlesim_node
```

预期出现一个小海龟窗口。窗口要保持运行。

终端 B：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtle_teleop_key
```

用鼠标点一下 **终端 B**，再按方向键：

| 按键 | 含义 |
|---|---|
| 上 | 沿海龟自己的朝向前进 |
| 下 | 后退 |
| 左 | 逆时针转动 |
| 右 | 顺时针转动 |

“上”表示前进，不保证往屏幕上方移动。初始朝向接近屏幕右方时，上键会让海龟向右走。不要在小海龟窗口中输入键盘控制。

短按后它可能继续动一小段再停，这是这个仿真程序的速度超时处理；不是每次按键都指定了一个终点。第 3 天会解释。

终端 C：

```bash
source ~/ros2_week1/setup_env.bash
ros2 node list
ros2 topic list
```

关注 `/turtlesim`、`/teleop_turtle`，以及 `/turtle1/cmd_vel`、`/turtle1/pose`。

### 1.9 当天练习与答案

**练习 1：**只启动海龟窗口，不启动键盘程序。海龟为什么不自己运动？  
**答案：**仿真程序在运行，但没有收到非零速度指令。程序运行与机器人运动是两件事。

**练习 2：**关闭键盘程序后，海龟窗口为什么还在？  
**答案：**它们是独立启动的程序，结束键盘控制程序不会自动结束仿真器。

**练习 3：**新终端出现 `ros2: command not found`，第一步做什么？  
**答案：**先 `source ~/ros2_week1/setup_env.bash`，再尝试 `ros2 --help`。不要立即重装。

**练习 4：**如果 A 使用通信域 42，B 使用 43，它们是否会正常发现彼此？  
**答案：**这两个不同域中的节点不会按本周的默认配置互相发现。所有实验终端要统一环境并重启旧进程。

**当天验收：**你能独立启动 talker/listener，能控制海龟，也能解释为什么要在每个新终端加载环境。

<a id="day2"></a>

## 第 2 天：节点、功能包与通信图

### 2.1 学习目标与准备

今天要把“输入命令可以运行”提升到“知道运行了什么”。建议先花 35 分钟读概念，再用 80 分钟做例题，最后 35 分钟复述和练习。

结束昨天仍在运行的实验进程，然后重新启动：

终端 A：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtlesim_node
```

终端 B：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtle_teleop_key
```

终端 C：

```bash
source ~/ros2_week1/setup_env.bash
cd ~/ros2_week1
```

### 2.2 六个名词，必须分开

| 名词 | 含义 | 本周例子 |
|---|---|---|
| 工作空间 Workspace | 组织自己开发的一组功能包及构建产物的目录 | 下周创建的 `ros2_ws` |
| 功能包 Package | 按功能组织代码、接口、配置和安装信息的单位 | `turtlesim` |
| 可执行文件 Executable | 可以启动的程序入口 | `turtlesim_node`、`turtle_teleop_key` |
| 进程 Process | 操作系统中运行起来的程序实例 | 由一次 `ros2 run` 启动的程序通常形成一个进程 |
| 节点 Node | ROS 2 通信图中的逻辑参与者 | `/turtlesim`、`/teleop_turtle` |
| 线程 Thread | 进程内部的执行单元 | 读取键盘或处理回调可能使用不同线程 |

**一个进程可以包含多个节点，一个节点也可以通过执行器使用多个线程。** 本周例子中常见“一个进程对应一个节点”，但这是例子的组织方式，不是 ROS 2 的强制规定。[S3]

对照你之前的项目：`EncoderSimulator` 这样的 C++ 类不会因为存在就自动成为节点。只有程序创建并运行了 ROS 2 节点，它才会出现在 ROS 通信图中。

### 2.3 逐项拆解 `ros2 run`

```bash
ros2 run turtlesim turtlesim_node
```

| 部分 | 解释 |
|---|---|
| `ros2` | ROS 2 命令行入口 |
| `run` | 查找并运行某个包中的可执行文件 |
| `turtlesim` | 功能包名称 |
| `turtlesim_node` | 包中已安装的可执行文件名称 |

`ros2 run` 不会现场编译你的 C++ 源文件。下周写程序时，要先构建并加载工作空间环境，再运行安装好的可执行文件。

终端 C 查看这个包提供什么程序：

```bash
ros2 pkg executables turtlesim
ros2 pkg prefix turtlesim
```

第一条输出会包含 `turtlesim turtlesim_node` 和 `turtlesim turtle_teleop_key`，也可能包含其他示例；第二条通常显示 `/opt/ros/jazzy`，表示当前找到的是已安装的系统包。

### 2.4 例题 3：检查节点各自负责什么

终端 C：

```bash
ros2 node list
ros2 node info /turtlesim
ros2 node info /teleop_turtle
```

`node list` 列出发现的节点。`node info` 查看某个节点的发布者、订阅者、服务和动作接口。

重点对照下面这张表：

| 节点 | 接收或发送的内容 | 意义 |
|---|---|---|
| `/teleop_turtle` | 发布 `/turtle1/cmd_vel` | 把按键变成速度指令 |
| `/turtlesim` | 订阅 `/turtle1/cmd_vel` | 根据指令更新模拟运动 |
| `/turtlesim` | 发布 `/turtle1/pose` | 对外报告当前位置和姿态 |
| `/turtlesim` | 提供 `/spawn` 等服务 | 接受创建海龟等请求 |

输出中还有 `/rosout`、`/parameter_events` 等系统接口。第一轮先认识它们的作用，不需要把完整列表背下来。

你应该能回答：“谁计算按键含义，谁更新位置？”前者是遥控程序，后者是仿真程序。两个程序之间传递的是消息。

### 2.5 例题 4：看懂节点通信图

在终端 D 运行：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run rqt_graph rqt_graph
```

图形工具打开后点刷新按钮。显示模式可选 `Nodes/Topics (all)`；若有 Hide 过滤选项，可以取消相关隐藏以观察更多接口。不同版本的界面标签可能略有不同。

本实验的核心通信关系是：

| 发布者 | 话题 | 订阅者 |
|---|---|---|
| `/teleop_turtle` | `/turtle1/cmd_vel` | `/turtlesim` |
| `/turtlesim` | `/turtle1/pose` | 之后运行的 `ros2 topic echo` |

如果 `/turtle1/pose` 暂时没有订阅者，一些图形过滤模式会把它隐藏。图中没画出来，不一定表示话题不存在，先用 `ros2 topic list` 核实。

运行 `ros2 topic echo`、`ros2 topic pub` 等命令时，可能出现带 `_ros2cli` 或类似前缀的临时节点。这些工具为了观察或发送消息，也会参与 ROS 通信。结束命令后，它们可能需要一小段时间才从发现结果中消失。

### 2.6 例题 5：修改节点名称，观察哪些名字会变

先在 A、B 中分别结束仿真器和键盘控制，只保留检查终端。然后在 A 中运行：

```bash
ros2 run turtlesim turtlesim_node --ros-args -r __node:=practice_sim
```

终端 C：

```bash
ros2 node list
ros2 node info /practice_sim
ros2 topic list
```

预期：节点名称变为 `/practice_sim`，但海龟的话题仍然包含 `/turtle1/cmd_vel` 和 `/turtle1/pose`。

解释：

- `--ros-args`：后面是交给 ROS 2 解析的参数。
- `-r`：remap，重映射规则。
- `__node:=practice_sim`：把节点名称改成 `practice_sim`。
- `:=`：分隔要替换的名称和新名称，是 ROS 参数语法，不是 C++ 赋值。

**节点名称、话题名称、海龟对象名称不是同一个名字。** 修改节点名，不会自动把所有普通话题也改成同名路径。

不要同时启动两个只改了节点名、却仍使用同一套话题的仿真器，再误以为它们已经隔离。它们可能同时接收同一速度指令。第 5 天用命名空间实现隔离。

结束 A 中的改名实验，恢复默认仿真：

```bash
ros2 run turtlesim turtlesim_node
```

### 2.7 话题不是变量，节点也不是函数

`/turtle1/cmd_vel` 是通信通道的名字，不是 C++ 变量的内存地址。发布者发布一条消息，订阅者在消息到达时收到它；这不等于两个程序共享同一个变量。

一般话题允许多个发布者和多个订阅者，但机器人速度控制应明确由哪个模块负责输出。两个发布者交替发不同速度，可能让机器人运动与预期不一致。

ROS 2 常用的 DDS 通信实现支持发现与数据传输，本周不用手写这层网络代码。节点能够通信通常需要：

1. 处于能互相发现的通信环境。
2. 使用对应的话题名称和消息类型。
3. 发布端与订阅端的 QoS 要兼容。

同名话题只是条件之一。更换名称可以改变连接目标，更换消息类型则改变数据结构。

### 2.8 工作空间与 CMake 的联系

下周工作空间中常见这些目录：

| 目录 | 内容 |
|---|---|
| `src/` | 你开发的功能包源代码 |
| `build/` | 构建过程中的中间文件 |
| `install/` | 安装后的程序、库、接口和环境脚本 |
| `log/` | 构建日志 |

`colcon` 负责组织多个包的构建；C++ 包里仍会使用 CMake，`ament_cmake` 提供 ROS 2 包的相关支持。你已有的 CMake 知识会继续使用。

今天只需要理解关系，正式创建、编译 C++ 包放在下周。本周的 `~/ros2_week1` 不需要为了“看起来完整”而提前创建空的 build/install/log 目录。

### 2.9 当天练习与答案

**练习 1：**`turtlesim`、`turtlesim_node`、`/turtlesim` 各是什么？  
**答案：**依次是功能包名、可执行文件名、默认运行时节点名。

**练习 2：**一个节点既能订阅速度又能发布姿态吗？  
**答案：**可以。节点可以同时创建多个发布者、订阅者、服务和动作接口。

**练习 3：**把节点改名为 `/practice_sim` 后，为什么 `ros2 node info /turtlesim` 不再找到它？  
**答案：**要查询新的运行时节点名；这个变化不代表 turtlesim 功能包消失。

**练习 4：**如果观察到 3 个节点，能否断言有 3 个线程？  
**答案：**不能。节点、进程、线程不是一一对应关系。

**练习 5：**不看正文，说出键盘控制到海龟运动经过的两个节点和话题。  
**答案：**`/teleop_turtle` 发布 `/turtle1/cmd_vel`，`/turtlesim` 订阅并更新模拟状态。

**当天验收：**你能使用 `node info`、`pkg executables` 和 `rqt_graph` 解释一个正在运行的系统。

<a id="day3"></a>

## 第 3 天：话题、消息与运动控制

### 3.1 学习目标与准备

今天重点是“数据长什么样”和“数据如何改变运动”。建议 45 分钟学知识，100 分钟做例题，35 分钟做练习。

结束旧的仿真、遥控和持续发布进程，重新开一个默认海龟窗口。今天先不运行键盘遥控，避免它和命令发布者同时发速度。

终端 A：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtlesim_node
```

终端 B、C 都先运行：

```bash
source ~/ros2_week1/setup_env.bash
cd ~/ros2_week1
```

### 3.2 Topic 和 Message 到底是什么

**Topic（话题）**是一个有名称、有类型的通信通道。**Message（消息）**是发布到通道上的一份具体数据。

例如 `/turtle1/cmd_vel` 是话题名；`geometry_msgs/msg/Twist` 是该话题使用的消息类型；“前进速度 1.0，转动速度 0.5”是某一条消息携带的数值。

这与 `std::vector<int>` 中“变量名、类型和当前内容”的区别类似，但话题的作用是让通信参与者交换消息，不是一个普通容器。

Topic 通常用于连续数据流，例如雷达扫描、机器人姿态、速度指令。它没有内置“每发一条消息，必须收到对方的业务处理结果”这样的请求响应规则。

### 3.3 例题 6：先查类型，再看字段

终端 C：

```bash
ros2 topic list -t
ros2 topic type /turtle1/cmd_vel
ros2 topic info /turtle1/cmd_vel
ros2 interface show geometry_msgs/msg/Twist
```

`-t` 让列表同时显示类型。`type` 只查询一个话题的类型。`info` 查看发布者和订阅者数量等信息。接口结构的核心部分是：

```text
Vector3 linear
  float64 x
  float64 y
  float64 z
Vector3 angular
  float64 x
  float64 y
  float64 z
```

`geometry_msgs/msg/Twist` 可以拆成：功能包 `geometry_msgs`，接口类别 `msg`，类型名 `Twist`。

对平面差速机器人，最常用的是：

| 字段 | 平面运动中的意义 | 真实机器人通常使用的单位 |
|---|---|---|
| `linear.x` | 车体向前方向的线速度 v | m/s |
| `angular.z` | 绕竖直轴转动的角速度 ω | rad/s |
| 其他分量 | 这里通常设为 0 | 按对应物理量定义 |

`Twist` 本身既没有位置字段，也没有时间戳或坐标系名称。具体表示哪个坐标系下的速度，要由接口约定说明；本周海龟实验使用车体前向速度和偏航角速度。

角度常用值：90° = π/2 ≈ 1.5708 rad，180° ≈ 3.1416 rad，完整一圈约 6.2832 rad。把 `angular.z` 写成 90 不表示“旋转 90°”，而是极大的角速度指令。

### 3.4 例题 7：观察姿态消息

终端 C：

```bash
ros2 topic type /turtle1/pose
ros2 interface show turtlesim/msg/Pose
ros2 topic echo /turtle1/pose --once
```

Pose 的字段如下：

```text
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

| 字段 | 解释 |
|---|---|
| `x`、`y` | 世界平面中的位置 |
| `theta` | 海龟的朝向角，单位 rad |
| `linear_velocity` | 该仿真报告的线速度大小 |
| `angular_velocity` | 该仿真报告的角速度 |

Jazzy 的 turtlesim 把 `linear_velocity` 按速度大小计算，因此后退时它不一定为负；不能只凭这个字段的正负判断是否在倒车。[S11]

本手册把小海龟的长度称为仿真长度单位，避免把屏幕上的比例和真实机器人尺寸混淆。运动学形式与你之前的差速机器人一致：

$$
\dot{x}=v\cos\theta,\qquad
\dot{y}=v\sin\theta,\qquad
\dot{\theta}=\omega.
$$

同样是 `linear.x=1.0`，如果 θ=0，主要改变 x；如果 θ=π/2，主要改变 y。这解释了为什么“前进”不是固定朝屏幕某个方向。

### 3.5 发布命令中的 YAML 语法

接下来要使用这样的消息内容：

```yaml
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5
```

终端中可以写成紧凑形式：`'{linear: {x: 1.0}, angular: {z: 0.5}}'`。本接口中省略的数值字段使用默认值 0。

注意三个细节：使用英文标点；冒号后保留空格；最外层用单引号或双引号包住整个消息，避免 shell 把它拆成多个参数。YAML 文件缩进使用空格，不用 Tab。

### 3.6 例题 8：前进与停止

终端 B 持续发布前进指令：

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.0}}'
```

预期海龟直行，B 持续输出发布信息。观察约 1～2 秒后，在 B 中按 `Ctrl+C`，接着输入停止指令：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

`--rate 10` 表示目标发布频率 10 Hz，即每秒约 10 条消息，不表示速度 10，也不表示发送 10 秒。速度由消息里的 `linear.x` 决定。

`--once` 表示发送一条后退出。对于 Jazzy 的这个命令，它通常会先等待匹配的订阅者；如果一直显示等待，先检查海龟是否运行、话题名是否写错。[S12]

**必须先结束持续发送非零速度的发布者，再发零速度。** 否则零速度可能马上被仍在运行的另一发布者覆盖。

不要用这套手工命令来验证“恰好走了 2.000 米”。新进程启动、发现连接、消息传输、人工反应与仿真更新都需要时间，它适合验证运动方向和通信，不适合精确控制时间。

### 3.7 为什么发一次指令，它还会继续动

速度描述的是运动状态，不是一次离散位移。仿真器收到 `v=1` 后，会在随后每个更新周期里继续用这个速度计算位置，直到收到新指令或触发自身的超时处理。

Jazzy 的 turtlesim 实现会在约 1 秒没收到新的速度指令后将速度清零。[S11] 这是**这个仿真程序实现的行为**。ROS 2 并不保证所有机器人都在 1 秒后自动停止。

区分以下动作：

| 操作 | 确切含义 |
|---|---|
| `Ctrl+C` 结束 `topic pub` | 结束这个发布进程，不再发送消息 |
| 发布零速度 | 给接收者发送明确的速度设定 |
| 速度超时保护 | 接收者发现长时间没更新后，按照自身逻辑处理 |

这正好对应你后面给差速机器人增加“指令超时停止”的需求。

### 3.8 例题 9：原地旋转

确认前进发布者已停，在 B 中执行：

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.8}}'
```

在 C 中观察：

```bash
ros2 topic echo /turtle1/pose
```

预期：x、y 基本不变，theta 持续变化。达到角度表示范围边界时，theta 可能从接近 π 跳到接近 −π；这只是同一朝向的角度表示换边，不是机器人瞬间反向。

在 B、C 各自按 `Ctrl+C`，然后 B 发零速度：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
```

对于这个消息类型，空字典 `{}` 使用全零默认值，所以这条也是停止指令。理解之后可以使用简写；刚开始时写完整字段更直观。

### 3.9 例题 10：圆周运动，并计算半径

如果海龟已靠近边界，在 A 中结束后重新启动默认仿真，让它回到中间；所有发布者保持停止。

终端 B：

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 1.0}}'
```

观察约一圈，然后 `Ctrl+C` 并发布零速度：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
```

圆周运动中 `v=ωR`，因此半径大小和周期是：

$$
R=\frac{|v|}{|\omega|}=1,\qquad
T=\frac{2\pi}{|\omega|}\approx 6.283\text{ s}.
$$

这里 ω 是整车偏航角速度，不是左轮或右轮的转速。角速度为正时逆时针，负时顺时针；在非零 v、ω 情况下，半径大小由它们的比值决定。

三种情况要能直接判断：

| v | ω | 运动 |
|---|---|---|
| 非零 | 0 | 直行 |
| 0 | 非零 | 原地转动 |
| 非零 | 非零 | 理想平面模型中的圆弧运动 |

ω=0 时不能直接计算 `v/ω`；此时应单独理解为直线运动。

### 3.10 例题 11：测量发布频率和连接情况

终端 B：

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
```

终端 C：

```bash
ros2 topic hz /turtle1/cmd_vel
```

观察几秒后，输出中的平均频率应接近 10 Hz，但不要求精确为 10.000。`topic hz` 测量的是这个观察者接收到消息的频率，会受到 QoS、调度、计算负载和丢包等影响。

在 C 中 `Ctrl+C`，然后查看详细连接信息：

```bash
ros2 topic info /turtle1/cmd_vel --verbose
```

输出会列出通信端点及 QoS。当前 B 是发布者，仿真器是订阅者；如果还有 echo、hz、录制器，订阅者数量就可能增加。

结束 B 的持续发布。即使消息全零，也不要留下不明来源的旧发布者干扰后续 Action 实验。

### 3.11 QoS 第一周只需要掌握到这里

QoS 是 Quality of Service，表示通信行为的配置。例如历史队列深度、可靠性、是否为晚加入的订阅者保存样本。

| 项目 | 先这样理解 |
|---|---|
| Reliable | 尝试保证可靠传输；不能因此推断业务一定成功或延迟一定很小 |
| Best effort | 尽力发送，允许丢失；某些高频传感器会使用 |
| Keep last / depth | 只保留最近若干个样本的历史策略 |
| Volatile | 通常不向晚加入的订阅者补发已经过去的数据 |
| Transient local | 配合相应配置，可向晚加入者提供发布者保留的样本 |

发布与订阅必须 QoS 兼容。例如 Best effort 发布者无法满足要求 Reliable 的订阅者。这能造成“名字、类型都对，但收不到”的现象。[S13]

今天不要求背所有兼容性矩阵；要记得用 `topic info --verbose` 收集证据，不能遇到收不到就盲目改随机参数。

### 3.12 当天练习与答案

**练习 1：**想让海龟以 0.5 的线速度、−0.5 rad/s 的角速度画圆，半径与方向是什么？

答案：半径 1，顺时针。先确认只有默认仿真运行，再用：

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: -0.5}}'
```

观察后结束发布并发零速度。

**练习 2：**同样的 `linear.x=1.0`，发布频率由 10 Hz 改为 20 Hz，会不会让理想运动速度翻倍？  
**答案：**不会。消息内容中的速度设定没有变。频率影响更新节奏、超时风险与通信负载。

**练习 3：**θ=π/2，v=0.5，ω=0，持续 2 秒，理想位移是什么？  
**答案：**Δx=0，Δy=1，朝向不变；实际手工 CLI 控制不会保证精确持续 2 秒。

**练习 4：**只让命令发布 5 条该怎么写？

```bash
ros2 topic pub --times 5 --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
```

`--times 5` 限制消息条数。第一条到第五条的目标间隔约为 `(5−1)/10=0.4 s`，另有发现、调度与退出等待时间；不能直接当成精确运动时长。

**练习 5：**为什么看到 `/turtle1/cmd_vel` 出现在话题列表里，还不能证明海龟在接收？  
**答案：**话题可能只有发布者或只有订阅者；还要检查端点、类型、QoS 和实际接收/运动现象。

**当天验收：**你能自行构造 Twist 消息，预测直行/转动/圆弧，观察 Pose，区分速度、角度、频率和发送次数。

<a id="day4"></a>

## 第 4 天：服务、参数与配置文件

### 4.1 学习目标与准备

今天学习两类不同需求：让节点“执行一次操作”，以及调整节点“使用什么配置”。建议用 40 分钟学概念，100 分钟实践，40 分钟整理。

关闭所有旧的仿真、键盘控制与持续发布进程。终端 A 启动默认仿真，B、C 用作命令终端。

终端 A：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtlesim_node
```

终端 B、C：

```bash
source ~/ros2_week1/setup_env.bash
cd ~/ros2_week1
```

### 4.2 Service：一次请求，一次响应

Service 是服务通信。Client（客户端）提交 Request（请求），Server（服务端）处理并返回 Response（响应）。常用于创建对象、重置状态、查询结果等相对短的操作。[S5]

“请求响应”描述通信结构，不保证操作一定成功、实时完成或没有副作用。客户端应该根据接口定义和响应判断结果。

和 Topic 比较：

| 需求 | 更适合的接口 | 原因 |
|---|---|---|
| 连续报告机器人位置 | Topic | 形成持续数据流，可能被多个模块消费 |
| 请求生成一只海龟并拿到名字 | Service | 一次操作有对应响应 |
| 请求机器人导航到目标，查看进度并可取消 | Action | 任务可能持续较久，需要反馈和任务管理 |
| 设置节点的背景颜色 | Parameter | 这是节点的运行配置 |

参数是节点的配置机制，其读写在底层也会使用 ROS 通信接口，不是一个完全独立于服务的网络系统。

### 4.3 例题 12：查找服务与接口

终端 B：

```bash
ros2 service list -t
ros2 service type /spawn
ros2 interface show turtlesim/srv/Spawn
```

Spawn 的接口核心结构：

```text
float32 x
float32 y
float32 theta
string name
---
string name
```

`---` 上面是请求，下面是响应。这两个 `name` 虽然同名，但属于不同方向的数据：请求里指定期望名字，响应里返回创建出的名字。

接口类型名 `turtlesim/srv/Spawn` 中，`srv` 表示服务。不要把服务名 `/spawn` 和服务类型混为一谈：前者是运行中的接口地址，后者规定请求和响应结构。

### 4.4 例题 13：创建第二只海龟

终端 B：

```bash
ros2 service call /spawn turtlesim/srv/Spawn \
  '{x: 2.0, y: 2.0, theta: 0.0, name: "turtle2"}'
```

预期：同一个窗口内出现第二只海龟，响应包含名字 `turtle2`。

然后查看：

```bash
ros2 node list
ros2 topic list
ros2 service list
```

**关键观察：**生成第二只海龟，不会因此生成第二个 `/turtlesim` 节点。它是同一个仿真节点管理的另一个对象。新增的是 `/turtle2/cmd_vel`、`/turtle2/pose`、`/turtle2/set_pen` 等接口。

让第二只海龟转动：

```bash
ros2 topic pub --rate 10 /turtle2/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.8}}'
```

观察后 `Ctrl+C`，再发零速度：

```bash
ros2 topic pub --once /turtle2/cmd_vel geometry_msgs/msg/Twist '{}'
```

只有第二只应转动，因为话题明确指向它。

若重复执行相同名字的 spawn，服务可能拒绝重复名称。看到失败先检查是否已经创建过，不要认为 ROS 整体失效。

### 4.5 例题 14：位置传送与清除轨迹

确保所有速度发布者已停止。终端 B：

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute \
  '{x: 5.5, y: 5.5, theta: 0.0}'
ros2 service call /clear std_srvs/srv/Empty '{}'
ros2 topic echo /turtle1/pose --once
```

预期：第一只海龟被直接设置到 `(5.5, 5.5)` 附近，朝向约 0；随后旧轨迹被清除。这里第二条会清除画面中的轨迹，不只是当前海龟的轨迹。

传送是仿真工具，不表示真实机器人可以瞬间移动，也不等于控制器已经计算出到达该点的路径。

检查 Empty 接口：

```bash
ros2 interface show std_srvs/srv/Empty
```

它没有业务请求字段，也没有业务响应字段。`{}` 就是空请求。请求与响应为空不代表“没做任何事”，例如 `/clear` 仍会清除轨迹。

### 4.6 例题 15：修改画笔、删除海龟、重置仿真

把第一只海龟的画笔改成红色，宽度 3，打开画笔：

```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
  '{r: 255, g: 40, b: 40, width: 3, "off": 0}'
```

RGB 每个分量取 0～255。`"off": 0` 表示不关闭，即开启画笔；`"off": 1` 表示关闭。这里特意给 `off` 这个键加双引号，避免 YAML 解析器把未加引号的 off 识别成布尔值。它控制之后的绘制，不是给已有全部轨迹重新着色。

给海龟持续发布圆周速度，观察新轨迹颜色，结束后发零速度：

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 1.0}}'
```

按 `Ctrl+C` 后：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
```

删除第二只海龟：

```bash
ros2 service call /kill turtlesim/srv/Kill '{name: "turtle2"}'
```

这个 `/kill` 是仿真器提供的服务，只删除指定模拟对象，不是操作系统的 `kill` 命令，也不会关闭整个 turtlesim 进程。

重置整个模拟场景：

```bash
ros2 service call /reset std_srvs/srv/Empty '{}'
```

它会重建初始海龟并清理场景。不要假定这个操作会把节点的所有配置参数都恢复出厂值；场景状态与参数配置是不同层次。

### 4.7 Parameter：节点的配置

参数一般属于某个节点，必须指定“哪个节点的哪个参数”。例如 `/turtlesim` 的 `background_r` 控制背景红色分量；其他节点即使有同名参数，也不等于共享同一个值。

参数可以是整数、浮点数、布尔值、字符串及一些对应数组。参数的类型和能否在运行时修改，取决于节点如何声明和处理它。

终端 B：

```bash
ros2 param list /turtlesim
ros2 param get /turtlesim background_r
ros2 param describe /turtlesim background_r
```

第一条列名称，第二条查当前值，第三条查描述与类型等信息。不是所有参数都有详细范围说明，用户仍应遵守该参数的用途。

### 4.8 例题 16：运行时修改背景颜色

```bash
ros2 param set /turtlesim background_r 40
ros2 param set /turtlesim background_g 50
ros2 param set /turtlesim background_b 70
ros2 param get /turtlesim background_r
```

预期各次设置返回成功，最后读取红色分量为 40，背景颜色相应改变。若画面未及时刷新，可以调用 `/clear`，但它也会清除旧轨迹。

为什么写 `40` 而不是 `40.0`？这里声明的是整数参数，浮点数与整数是不同类型。参数设置可能因类型不符、只读或校验失败而被拒绝；输出成功与否要实际看。

`ros2 param set` 改的是当前节点实例。结束节点再按默认方式重启，通常会回到程序默认配置；要持久复用，就保存参数文件并在启动时加载。[S6]

### 4.9 例题 17：保存快照与制作可复用配置

先把当前节点参数导出为快照：

```bash
ros2 param dump /turtlesim > ~/ros2_week1/config/turtlesim_snapshot.yaml
cat ~/ros2_week1/config/turtlesim_snapshot.yaml
```

`>` 把命令标准输出写到文件；同名文件已存在会被覆盖。这里是你本次实验使用的参数快照。

导出的完整快照可能包含一些只读参数，例如 QoS 覆盖项。为了避免把它们也拿来运行时修改，本例另建一个只包含需要管理的参数的文件：

```bash
cat > ~/ros2_week1/config/turtlesim.yaml <<'EOF'
/turtlesim:
  ros__parameters:
    background_r: 40
    background_g: 50
    background_b: 70
    use_sim_time: false
EOF
```

逐行解释：

| 内容 | 作用 |
|---|---|
| `/turtlesim:` | 这些参数要应用到这个完整节点名 |
| `ros__parameters:` | ROS 2 参数文件固定使用的键，ros 后面是两个下划线 |
| 三个 `background_*` | 明确保存本实验关心的颜色配置 |
| `use_sim_time: false` | 本周不使用 `/clock` 驱动 ROS 时间 |

仿真程序并不必然要求 `use_sim_time=true`。只有系统提供并正确使用 `/clock` 时，相关节点才应统一配置仿真时间；本周 turtlesim 实验保持 false。Gazebo 阶段再专门学习时钟。

现在先故意改掉红色，再加载文件恢复：

```bash
ros2 param set /turtlesim background_r 180
ros2 param load /turtlesim ~/ros2_week1/config/turtlesim.yaml
ros2 param get /turtlesim background_r
```

最后应读到 40。若加载完整 snapshot 出现部分只读参数不能设置，不意味着所有参数加载失败，要按每条结果检查；本例的精简配置专门避免这个问题。

### 4.10 例题 18：重启后自动使用参数文件

先在 A 中 `Ctrl+C` 结束仿真，再运行：

```bash
ros2 run turtlesim turtlesim_node --ros-args \
  --params-file ~/ros2_week1/config/turtlesim.yaml
```

终端 B 验证：

```bash
ros2 param get /turtlesim background_r
ros2 param get /turtlesim background_g
ros2 param get /turtlesim background_b
```

预期依次为 40、50、70。这才是一个可重复使用的启动配置。

如果你把节点重命名，或加了命名空间，完整节点名可能不再是 `/turtlesim`。参数文件顶部也要匹配实际节点名；不能只改启动命令的名字而忽略配置目标。

### 4.11 当天练习与答案

**练习 1：**创建名为 `practice_turtle` 的海龟，位置 `(3, 4)`，朝向 π/2。

```bash
ros2 service call /spawn turtlesim/srv/Spawn \
  '{x: 3.0, y: 4.0, theta: 1.5708, name: "practice_turtle"}'
```

验证：

```bash
ros2 topic echo /practice_turtle/pose --once
```

**练习 2：**把这只海龟的画笔关闭。

```bash
ros2 service call /practice_turtle/set_pen turtlesim/srv/SetPen \
  '{r: 255, g: 255, b: 255, width: 3, "off": 1}'
```

**练习 3：**为什么创建了三只海龟，`node list` 中却可能仍然只有一个仿真节点？  
**答案：**模拟对象与 ROS 节点不是同一概念，一个仿真节点可以管理多只海龟。

**练习 4：**服务接口里一条 `---` 分隔什么？  
**答案：**分隔请求与响应。接口定义的分隔符和 `topic echo` 输出里的消息分隔线用途不同。

**练习 5：**参数文件存在，但重启没恢复颜色，优先查什么？  
**答案：**启动时是否传了 `--params-file`，路径是否正确，YAML 格式是否正确，顶部节点名是否匹配实际节点名。

**练习 6：**要实时传递 100 Hz 的编码器数据，用参数还是话题？  
**答案：**用话题更合适；轮径、编码器每圈计数等配置才适合作为参数。

**当天验收：**你能独立查接口、构造服务请求、读取响应、操作多只海龟，并保存与恢复节点配置。

<a id="day5"></a>

## 第 5 天：Action、launch 与日志

### 5.1 学习目标与准备

今天让系统能执行一个有开始、有过程、有结束的任务，并认识如何统一启动和观察日志。建议 Action 70 分钟、launch 65 分钟、日志与练习 45 分钟。

结束旧的仿真、键盘程序、持续发布者。终端 A 启动一个默认海龟：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtlesim_node
```

终端 B、C 先加载环境，再继续：

```bash
source ~/ros2_week1/setup_env.bash
cd ~/ros2_week1
```

### 5.2 Action 为什么存在

假设要让机器人到达走廊尽头：任务可能持续 30 秒，你想知道它走到哪里、能否取消，以及最后是成功还是失败。Action 为这类任务提供目标、反馈、结果及取消机制。[S7]

| 部分 | 含义 | 本周“转到指定角度”的例子 |
|---|---|---|
| Goal | 希望完成什么 | 转到 θ=1.5708 rad |
| Feedback | 执行中的信息 | 离目标还差多少角度 |
| Result | 任务结束时的业务结果 | 该实现报告的角度差值 |
| Status | 任务生命周期状态 | 成功、取消、中止等 |

Goal 被接受，只能说明服务端同意尝试执行，不代表已经成功。反馈字段也不一定是百分比，要按每种 Action 的接口理解。

取消是请求，服务端可以根据自身实现决定是否接受；你应该观察最终状态，不能把“客户端已经不输出了”当作任务取消的证据。

### 5.3 例题 19：查询 Action 并发送目标

终端 B：

```bash
ros2 action list -t
ros2 action info /turtle1/rotate_absolute
ros2 interface show turtlesim/action/RotateAbsolute
```

接口核心结构：

```text
float32 theta
---
float32 delta
---
float32 remaining
```

`.action` 接口按顺序分成 **Goal、Result、Feedback** 三部分，所以有两条分隔线。不要按“执行时先反馈后结果”的时间顺序误读接口文件结构。

设置可重复的起点：

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute \
  '{x: 5.5, y: 5.5, theta: 0.0}'
```

再发送目标：

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute \
  '{theta: 1.5708}' --feedback
```

预期：先显示目标被接受，随后输出反馈，海龟原地转动，最后显示结果与 `SUCCEEDED`。终端恢复输入状态后检查：

```bash
ros2 topic echo /turtle1/pose --once
```

theta 应接近 1.5708，允许存在该示例的停止容差。`rotate_absolute` 是转到世界中的绝对朝向，不是每调用一次就再相对旋转同样的角度。

补充：Jazzy turtlesim 的 `delta` 使用“初始朝向减当前朝向”的归一化值，因此朝正方向旋转后，这个结果字段可能为负。[S11] 不要把 `delta` 直接当成最终朝向；用状态和 Pose 检查任务结果。

### 5.4 例题 20：明确地取消一个任务

本例使用键盘程序自己的 Action 客户端发送、取消同一个目标，避免混淆客户端归属。

先在 B 中重设朝向：

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute \
  '{x: 5.5, y: 5.5, theta: 0.0}'
```

终端 B 接着启动键盘程序：

```bash
ros2 run turtlesim turtle_teleop_key
```

把焦点放在 B：

1. 按小写 `d`，向接近 π 的绝对朝向发出旋转目标。
2. 观察它开始转动后，在大约半秒内按小写 `f`。
3. 观察海龟提前停止，并查看 A 中的取消日志。

如果你按 `f` 太迟，任务已经完成，就不会再表现出中途停止。可以先按 `g` 让它回到朝向 0，等完成后重做。

| 按键 | 键盘程序中的作用 |
|---|---|
| `g` | 发送绝对朝向 0 的目标 |
| `r` | 发送绝对朝向约 π/2 的目标 |
| `d` | 发送绝对朝向约 π 的目标 |
| `f` | 请求取消这个键盘客户端持有的旋转目标 |
| `q` | 退出键盘程序 |

不要用 `f` 去证明取消了另一个 `ros2 action send_goal` 进程发送的目标；键盘客户端默认管理的是自己的目标句柄。[S11]

**不要把终止客户端与取消远端任务画等号。** `Ctrl+C` 的直接作用是终止当前前台程序；是否另外发送取消、服务端如何处理，取决于客户端和服务端实现。

此外，turtlesim 在执行旋转 Action 时收到新的普通速度消息，会中止当前旋转任务。[S11] 因此实验前要关掉旧的持续速度发布者，即使它一直发的是零速度。

结束后按 `q` 退出 B 的键盘程序。

### 5.5 launch：统一描述如何启动系统

每次启动多个节点都手动敲命令，很容易忘记参数、名称和命名空间。launch 文件把这些启动配置组织起来，方便复现。

本周先学会读一个简单 launch 文件。ROS 2 launch 可以用多种格式编写，这里采用 Python，因为下一阶段经常会遇到。

你不需要先学完 Python。只需知道：`import` 导入工具；`def` 定义函数；方括号是列表；`key=value` 是给函数指定参数；Python 布尔值写作 `False`。

### 5.6 例题 21：编写并运行两个独立海龟窗口

先在 A 中关闭默认仿真，确认 B 的键盘程序也已退出。终端 C 创建 launch 文件：

```bash
cat > ~/ros2_week1/launch/two_turtles.launch.py <<'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='blue',
            name='sim',
            parameters=[{
                'background_r': 35,
                'background_g': 65,
                'background_b': 130,
                'use_sim_time': False,
            }],
            output='screen',
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='orange',
            name='sim',
            parameters=[{
                'background_r': 180,
                'background_g': 100,
                'background_b': 35,
                'use_sim_time': False,
            }],
            output='screen',
        ),
    ])
EOF
```

这是完整文件，不需要你补充省略部分。逐项理解：

| 字段 | 作用 |
|---|---|
| `generate_launch_description()` | launch 系统调用的入口函数 |
| `LaunchDescription([...])` | 返回要执行的启动动作列表 |
| `Node(...)` | 描述一个 ROS 节点程序怎样启动 |
| `package`、`executable` | 对应之前 `ros2 run` 的两个关键名称 |
| `namespace` | 给相对接口名增加一层前缀 |
| `name` | 指定节点名称 |
| `parameters` | 传入启动参数 |
| `output='screen'` | 将进程输出显示在启动终端 |

终端 A：

```bash
source ~/ros2_week1/setup_env.bash
ros2 launch ~/ros2_week1/launch/two_turtles.launch.py
```

本周直接通过文件路径运行 launch，不需要先做成自己的功能包。下一周再学习把 launch 文件安装进包并通过包名启动。

终端 C：

```bash
ros2 node list
ros2 topic list
ros2 param get /blue/sim background_b
```

预期看到两个不同颜色的窗口，节点名 `/blue/sim`、`/orange/sim`；蓝色节点的 `background_b` 为 130。

| 窗口 | 节点完整名称 | 海龟速度话题 |
|---|---|---|
| 蓝色 | `/blue/sim` | `/blue/turtle1/cmd_vel` |
| 橙色 | `/orange/sim` | `/orange/turtle1/cmd_vel` |

虽然两个窗口中的对象都叫 `turtle1`，完整接口路径不同，所以可以分别控制。

### 5.7 例题 22：只控制蓝色窗口

终端 B：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/blue
```

让焦点停在 B，按方向键。预期只有蓝色窗口中的海龟运动。

`__ns:=/blue` 为这个键盘节点设置命名空间，它在程序中使用的相对话题 `turtle1/cmd_vel` 会解析为 `/blue/turtle1/cmd_vel`。

终端 C：

```bash
ros2 node info /blue/teleop_turtle
ros2 topic info /blue/turtle1/cmd_vel
ros2 topic info /orange/turtle1/cmd_vel
```

对照连接数量，但注意 CLI 等其他工具也可能影响数量，不能死背一个固定数字。

按 `q` 结束 B 的键盘程序。接着学习显式话题重映射：

```bash
ros2 run turtlesim turtle_teleop_key --ros-args \
  -r __node:=orange_keyboard \
  -r turtle1/cmd_vel:=/orange/turtle1/cmd_vel
```

此时方向键应控制橙色窗口。这里仅把速度话题重映射到了橙色海龟，Action 名称没有随之修改，所以这个实验只使用方向键，不使用 `g/r/d/f` 等动作键。

**命名空间会影响相对接口名，显式话题重映射只影响指定规则对应的名称。** 不能把“速度能发过去”推断成所有服务和 Action 都已经连接正确。

### 5.8 日志不是普通的屏幕文字

ROS 日志带有严重级别、时间和记录者名称，方便在多节点系统中定位问题。常见级别从低到高：

| 级别 | 用途 |
|---|---|
| DEBUG | 调试细节，通常默认不显示 |
| INFO | 正常运行的重要信息 |
| WARN | 需要注意，但不一定导致程序退出 |
| ERROR | 操作失败或明显异常 |
| FATAL | 严重错误级别；仅打印该级别不等于语言层面自动结束进程 |

很多工具显示的输出只是普通标准输出，不一定都经过 ROS 日志系统。`rqt_console` 主要观察发布到 `/rosout` 的 ROS 日志。

### 5.9 例题 23：用日志定位“海龟不再往前走”

先退出 B 的键盘程序，在 A 中 `Ctrl+C` 结束整个 launch。两个由它启动的仿真器应一起关闭；等待它们退出后再继续。

终端 A 启动一个默认仿真：

```bash
ros2 run turtlesim turtlesim_node
```

终端 C 打开日志工具：

```bash
ros2 run rqt_console rqt_console
```

终端 B 持续发布较快的前进速度：

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 2.0}, angular: {z: 0.0}}'
```

海龟到达画面边界后位置不再前进，你应在仿真终端或日志工具里看到碰到边界的 WARN 信息。这个实验只在 turtlesim 内执行。

立刻在 B 中结束持续发布，再发零速度：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
```

这个现象不一定是消息没收到：它可能正在接收速度，但受仿真场景边界约束。检查问题时应同时观察通信、状态和日志。

要观察更低级别日志，可以在退出旧仿真后这样启动：

```bash
ros2 run turtlesim turtlesim_node --ros-args --log-level debug
```

设置 DEBUG 只会让已经存在的调试日志可见，不会凭空为程序增加每个变量的打印。

### 5.10 当天练习与答案

**练习 1：**海龟当前朝向 0.5 rad，发送绝对目标 1.0 rad，是否要求再转 1.0 rad？  
**答案：**不是。它的目标最终朝向是 1.0 rad；不考虑角度归一化和容差时，需要变化约 0.5 rad。

**练习 2：**Action 接口两条 `---` 按什么顺序划分？  
**答案：**Goal、Result、Feedback。

**练习 3：**蓝色命名空间中，节点 `sim` 的完整名称是什么？相对话题 `turtle1/pose` 的完整名称是什么？  
**答案：**`/blue/sim` 和 `/blue/turtle1/pose`。

**练习 4：**给两个节点不同的节点名，能否保证它们不接收同一个速度话题？  
**答案：**不能。还要看实际话题路径；通常用命名空间与重映射明确隔离。

**练习 5：**先看到 Goal accepted，后来看到 ABORTED，能否写“动作执行成功”？  
**答案：**不能。它曾被接受，最终却中止了，应检查日志和外部干扰。

**练习 6：**蓝色窗口参数文件的根节点还写 `/turtlesim:`，为什么可能不生效？  
**答案：**实际节点完整名称为 `/blue/sim`，该配置目标不匹配。应调整根节点名，或在 launch 中给对应节点传配置。

**当天验收：**能解释并操作 Action，能启动两个隔离仿真，能根据日志区分通信问题与执行问题。

<a id="day6"></a>

## 第 6 天：录制回放与综合实验

### 6.1 学习目标

今天把前五天内容串起来：启动系统、指定初始状态、配置参数、发送指令、观察消息、录制、复现实验并解释误差。建议原理 30 分钟，完整实验 100 分钟，整理与排错练习 50 分钟。

你之前做过 CSV 日志。CSV 常由你自己选择字段并写成表格；rosbag2 可以记录指定话题的消息及时间信息，之后重新发布这些消息，供程序回放分析。[S9]

| 记录内容 | 回放时可以做什么 |
|---|---|
| 速度指令 | 重新给仿真器输入相似的指令序列 |
| 姿态消息 | 给分析程序提供历史姿态数据 |
| 雷达、相机等传感器话题 | 后续离线测试感知或定位程序 |

**录制器不会自动保存整个世界状态、所有参数、所有节点和所有文件。** 本周明确录制两个话题；要复现实验，还需要保存环境、初始条件和配置。

### 6.2 两种“回放”必须区分

第一种是把历史速度指令重新发布，让运行中的仿真器重新计算运动。第二种是把历史姿态直接作为数据发布给观察者。

这两件事不能混为一谈。往 `/turtle1/pose` 发布一条历史位置，不会自动把 turtlesim 中的海龟移动过去；仿真器的运动输入是速度话题及相关操作接口。

同样，如果运行中的仿真器正在发布 `/turtle1/pose`，你又把历史 Pose 回放到同一个话题，观察者可能收到两个来源的数据。因此本实验“重新控制海龟”时只回放速度话题。

### 6.3 综合实验目标与终端分工

**任务：**把第一只海龟放到固定位置，画一圈附近的圆，录下速度和姿态，然后从同样起点回放速度指令。

| 终端 | 本实验任务 |
|---|---|
| A | 默认 turtlesim 仿真器 |
| B | rosbag2 录制器，之后用于播放 |
| C | 设置初始状态、发布指令、查询数据 |
| D | 可选，用于 `rqt_graph` 或观察姿态 |

开始前结束所有旧的 launch、键盘程序、仿真器、持续 `topic pub`、录制器和播放器。可以保留空终端，但不要留下后台控制来源。

### 6.4 例题 24：建立可重复的初始条件

终端 A：

```bash
source ~/ros2_week1/setup_env.bash
ros2 run turtlesim turtlesim_node
```

终端 C：

```bash
source ~/ros2_week1/setup_env.bash
cd ~/ros2_week1
ros2 param set /turtlesim background_r 40
ros2 param set /turtlesim background_g 50
ros2 param set /turtlesim background_b 70
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute \
  '{x: 5.5, y: 5.5, theta: 0.0}'
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
  '{r: 255, g: 100, b: 50, width: 3, "off": 0}'
ros2 service call /clear std_srvs/srv/Empty '{}'
ros2 topic echo /turtle1/pose --once
```

确认初始位置接近 `(5.5, 5.5)`、朝向接近 0。这样理论上 v=1、ω=1 的轨迹圆心在 `(5.5, 6.5)`，半径 1，不容易碰到边界。

为什么要清轨迹？为了区分本轮新画出的线和前一次实验残留。为什么要设置初始朝向？同一组车体速度指令，从不同朝向开始会产生不同世界轨迹。

### 6.5 例题 25：启动录制器

终端 B：

```bash
source ~/ros2_week1/setup_env.bash
cd ~/ros2_week1
ros2 bag record -s mcap -o bags/circle_run_01 \
  --topics /turtle1/cmd_vel /turtle1/pose
```

命令保持运行，等待数据。不要提前手工创建 `bags/circle_run_01` 这个输出目录，录制器会自己创建。

| 参数 | 解释 |
|---|---|
| `record` | 开始订阅并录制数据 |
| `-s mcap` | 指定 MCAP 存储格式 |
| `-o bags/circle_run_01` | 输出目录 |
| `--topics ...` | 明确指定要录制的话题 |

如果输出目录已存在，使用新的名字，例如 `circle_run_02`，同时把后续播放命令中的目录改成同一个名字。不要为了重做实验删掉自己还需要的数据。

### 6.6 例题 26：发布一组可重复的速度指令

确保 B 的录制器还在运行。在 C 中执行下面三条命令，可以整块复制：

```bash
ros2 topic pub --rate 10 --times 20 --wait-matching-subscriptions 2 \
  /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'

ros2 topic pub --rate 10 --times 63 --wait-matching-subscriptions 2 \
  /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 1.0}}'

ros2 topic pub --once --wait-matching-subscriptions 2 \
  /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
```

三条命令分别是：先保持停止一小段时间 → 发送圆周运动指令 → 明确停止。

`--wait-matching-subscriptions 2` 的目的是等待两个匹配订阅者：仿真器与录制器。它确认匹配端点数量，不是证明某条消息的业务处理一定已经完成。本实验不额外运行订阅速度话题的 echo，便于理解这两个来源。

如果卡在等待两个订阅者，优先检查 B 的录制是否仍在运行、它是否录制了正确话题，以及三处终端的环境是否一致。

为什么 63 条不是“精确走一圈”？10 Hz 下，第 1 条到第 63 条的目标时间差是 6.2 秒；末尾还涉及进程退出、下一发布者发现连接、消息调度等时间。这里预期画出一圈附近的圆，**不把 CLI 当成精密控制器**。

### 6.7 结束录制并查看内容

确保停止指令已发出，在 C 中保存一次终点观测：

```bash
ros2 topic echo /turtle1/pose --once > ~/ros2_week1/notes/circle_end_original.txt
```

然后在 B 中按 `Ctrl+C`，让录制器正常收尾并写入元数据。回到 C：

```bash
ros2 bag info ~/ros2_week1/bags/circle_run_01
ros2 bag info ~/ros2_week1/bags/circle_run_01 \
  > ~/ros2_week1/notes/circle_bag_info.txt
```

应该能看到：存储格式、持续时间、总消息数、话题列表，以及各话题消息类型与数量。确认 `/turtle1/cmd_vel` 和 `/turtle1/pose` 都有大于 0 的消息数。

Pose 消息条数通常比速度消息多，因为仿真器按自己的更新节奏报告状态，而本例速度按 10 Hz 发布。不要要求录制结果中的计数与手动输入的发布次数绝对一致；发现时机和实际收包会影响记录。

目录里通常包含 `metadata.yaml` 和一个或多个 `.mcap` 数据文件。复制实验记录时保留整个目录。

### 6.8 例题 27：只回放速度，让海龟重新运动

仿真器 A 保持运行。确认所有手动速度发布者已经退出，录制器 B 已停止。

终端 C 重设相同初始条件：

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute \
  '{x: 5.5, y: 5.5, theta: 0.0}'
ros2 service call /clear std_srvs/srv/Empty '{}'
ros2 topic echo /turtle1/pose --once
```

终端 B 播放：

```bash
ros2 bag play ~/ros2_week1/bags/circle_run_01 \
  --topics /turtle1/cmd_vel
```

预期先短暂停留，随后画出相似圆形，再停止。播放器完成后会退出。为明确实验结束，在 C 中再发一次零速度：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
ros2 topic echo /turtle1/pose --once > ~/ros2_week1/notes/circle_end_replay.txt
cat ~/ros2_week1/notes/circle_end_original.txt
cat ~/ros2_week1/notes/circle_end_replay.txt
```

比较两次终点。正常的观察重点是运动顺序和几何形状相似，不要求每个浮点数都相同。

存在差异的可能原因包括：消息交付与调度、初始状态不完全相同、仿真更新节奏、首次订阅发现时机、结束时的控制指令时序。你应该先核对这些条件，再判断是否有算法错误。

### 6.9 例题 28：离线查看历史姿态

结束 A 的仿真器，确保没有其他程序正在发布 `/turtle1/pose`。

终端 C 先启动观察者，并显式给出类型，避免在话题尚未出现时无法推断类型：

```bash
ros2 topic echo /turtle1/pose turtlesim/msg/Pose
```

终端 B 再启动播放：

```bash
ros2 bag play ~/ros2_week1/bags/circle_run_01 --topics /turtle1/pose
```

预期：没有海龟仿真窗口，但 C 能看到历史姿态序列。它显示的是录制时的数据，不是此刻有一只真实海龟正在运动。

如果本次播放器使用的 QoS 不满足观察者要求，可根据 `topic info --verbose` 检查；对于允许尽力接收的观察场景，也可以明确使用：

```bash
ros2 topic echo /turtle1/pose turtlesim/msg/Pose --qos-reliability best_effort
```

这条是排错时的替代观察命令，先结束原来的 echo 再运行。不要把更改 QoS 当成任何收不到数据问题的通用答案。

结束播放与观察后，本周核心实验完成。

### 6.10 你必须能够解释的完整关系

| 阶段 | 参与者 | 数据或配置 | 结果 |
|---|---|---|---|
| 启动 | ROS launch 或 `ros2 run` | 程序、名称、参数 | 节点开始运行 |
| 配置 | 参数客户端 | 背景颜色等 | 节点调整运行配置 |
| 设定起点 | 服务客户端与仿真器 | 目标 x、y、theta | 初始状态变得可重复 |
| 发指令 | CLI 发布者 | Twist 速度消息 | 仿真器更新运动 |
| 观测 | 仿真器与观察者 | Pose 状态消息 | 外部知道模拟状态 |
| 录制 | rosbag2 recorder | 收到的话题消息 | 写成可回放数据 |
| 回放控制 | rosbag2 player 与仿真器 | 历史速度消息 | 重新计算一次运动 |
| 离线观察 | rosbag2 player 与 echo | 历史姿态消息 | 查看曾发生过的状态 |

其中前进和旋转靠哪段算法计算，在你的下一阶段可以由自己的差速机器人 C++ 库负责；ROS 2 负责组织接口与通信，并不会自动替你写出运动学或 PID。

### 6.11 综合练习：故意制造一个能解释的小错误

重新启动默认仿真后，在 B 中执行：

```bash
ros2 topic pub --rate 10 /turtle9/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.8}, angular: {z: 0.0}}'
```

预期海龟不运动。不要立即改参数，先在 C 中收集证据：

```bash
ros2 topic info /turtle9/cmd_vel
ros2 topic info /turtle1/cmd_vel
ros2 node info /turtlesim
```

答案：发布者把数据送到了不存在对应海龟订阅者的 `/turtle9/cmd_vel`。持续发布模式默认不一定等待订阅者，所以即使终端在输出 publishing，也不能证明仿真器收到了。

结束错误发布者，再把话题改为 `/turtle1/cmd_vel` 验证。观察后结束持续发布并发零速度。

### 6.12 实验记录模板

在 `~/ros2_week1/notes/week1-report.md` 中写一份简短报告。可以用 VSCode 新建该文件，下面是可复制的内容模板：

```text
# ROS 2 第一周实验记录

## 环境
- Ubuntu 版本：
- ROS 2 版本：
- ROS_DOMAIN_ID：

## 我能解释的系统关系
- 仿真节点：
- 控制指令发布者：
- 速度话题及类型：
- 状态话题及类型：

## 圆周实验
- 初始 x、y、theta：
- 线速度 v：
- 角速度 omega：
- 理论半径：
- 理论一圈时间：
- 录制目录：
- 两个话题各自记录的消息数量：
- 原始与回放的终点差异：

## 我实际遇到并解决的问题
- 现象：
- 检查了哪些证据：
- 原因：
- 解决动作：

## 接入自己差速机器人项目时的对应关系
- 哪个模块接收速度：
- 哪个模块计算运动：
- 哪个模块报告里程计：
```

报告中的数据由你实测填写，不需要凭空凑指标。保存几张 rqt_graph、参数读取、bag info 和轨迹截图，会让下周回顾更方便。

**当天验收：**能从空终端独立完成一次录制和回放，解释两种回放的区别，并用端点信息定位一个话题名称错误。

<a id="exam"></a>

## 第一周自测与参考答案

### A. 先不看答案，完成这 20 题

建议 35～45 分钟。第 1～12 题口头回答或写一句话；第 13～18 题写命令；第 19～20 题计算并解释。

1. ROS 2 与 Ubuntu 分别负责什么？
2. Node、Process、Thread 是否一一对应？
3. `ros2 run turtlesim turtlesim_node` 中哪个是包、哪个是可执行文件？
4. `/turtle1/cmd_vel` 与 `geometry_msgs/msg/Twist` 分别是什么？
5. `topic echo` 为什么会影响话题订阅者数量？
6. `linear.x` 和 `angular.z` 分别控制什么？
7. Topic、Service、Action 分别适合怎样的需求？
8. 参数为什么要指定所属节点？
9. `Goal accepted` 与 `SUCCEEDED` 有什么区别？
10. 为什么关闭发布程序不等于机器人必然立即停止？
11. 为什么同名同类型的接口还可能无法通信？
12. 同时播放历史 Pose 与运行仿真器，可能出现什么问题？
13. 查看 `/turtlesim` 的发布者、订阅者和服务。
14. 读取 `/turtle1/pose` 的一条消息后退出。
15. 以 10 Hz 发送原地顺时针旋转 0.5 rad/s 的速度。
16. 把海龟设置到 `(4, 5)`，朝向 0。
17. 把背景绿色分量设为 80，并读取验证。
18. 向旋转 Action 发送绝对朝向 −π/2 的目标，同时显示反馈。
19. v=0.6、ω=0.3 时，圆周半径和周期是多少？
20. 20 Hz 发布 41 条消息，第 1 条到第 41 条的目标时间差是多少？能否据此断言机器人精确运动了这么久？

### B. 参考答案

1. Ubuntu 提供操作系统层面的进程、文件、设备、网络等能力；ROS 2 在其上提供机器人软件通信、接口、工具与开发框架。
2. 不一一对应。一个进程可以包含多个节点，线程负责执行逻辑，节点的回调也可以使用不同执行安排。
3. `turtlesim` 是包，`turtlesim_node` 是可执行文件；默认节点名称是 `/turtlesim`。
4. 前者是运行中的话题名称，后者是消息类型。
5. echo 要订阅话题才能收到数据，它也是通信参与者。
6. 在本周平面模型里，分别是车体前向线速度和偏航角速度，常用单位 m/s 与 rad/s；它们不是目标位置和目标角度。
7. Topic 适合数据流；Service 适合一次请求响应；Action 适合有反馈、结果、取消需求的任务。
8. 参数属于节点实例，同名参数在不同节点中可以有不同值。
9. accepted 表示目标被接受尝试执行，SUCCEEDED 表示任务已经成功结束。
10. 停止发布不会自动向接收者传递“速度归零”语义；是否停下取决于已有指令、接收端逻辑和超时机制。
11. 还可能有通信域、发现范围、网络、QoS 等问题；即使通信正常，业务处理也可能失败。
12. 相同话题出现历史与实时两类发布来源，数据交错，导致观察或后续计算混乱。

第 13～18 题的答案如下。需要实际执行时，先按照对应章节准备默认仿真；不要在持续旋转尚未停止时直接执行后面的动作题。

第 13 题：

```bash
ros2 node info /turtlesim
```

第 14 题：

```bash
ros2 topic echo /turtle1/pose --once
```

第 15 题：

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: -0.5}}'
```

观察后 `Ctrl+C`，再明确停下：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'
```

第 16 题：

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute \
  '{x: 4.0, y: 5.0, theta: 0.0}'
```

第 17 题：

```bash
ros2 param set /turtlesim background_g 80
ros2 param get /turtlesim background_g
```

第 18 题：

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute \
  '{theta: -1.5708}' --feedback
```

第 19 题：`R=|0.6/0.3|=2`，`T=2π/0.3≈20.944 s`。

第 20 题：`(41−1)/20=2 s`。这只是理想发布时间间隔；还涉及指令交付、控制端状态与更新周期等因素，不能当作实际运动时间的精确证明。

### C. 一道必须独立完成的综合题

**题目：**只参考本手册的速查表，从空终端开始，完成以下任务，并口头解释每一步。

1. 启动默认仿真器，确认节点名和姿态话题类型。
2. 创建第二只海龟，把它命名为 `review_turtle`。
3. 把第一只海龟的背景配置保存到 YAML 文件并重启恢复。
4. 对第一只海龟发送一个带反馈的旋转任务。
5. 录制第一只海龟的速度和姿态，完成一小段圆周运动。
6. 停止录制，从同样起点只回放速度。
7. 解释为什么回放时不要同时往姿态话题注入历史数据。

参考检查：步骤 3 重启整个仿真器会丢失通过 spawn 创建的运行时对象，这是正常现象。如果之后还要使用第二只海龟，需要重新 spawn；参数文件不会自动保存并恢复所有模拟对象。这是这道题特意检查的“配置与运行状态的区别”。

如果你能够完成这题，但常用命令还需要看速查表，仍然可以进入下一周。掌握系统关系比机械背下所有选项更重要。

<a id="troubleshooting"></a>

## 常见问题排查

### 先使用一个固定的检查顺序

遇到问题先描述现象，然后按下面顺序收集证据：

1. **环境：**ROS 版本、Domain ID、发现范围是否一致？
2. **进程与节点：**需要的程序是否确实还在运行？
3. **名字与类型：**完整接口名称和类型是否正确？
4. **连接与 QoS：**是否存在匹配通信端点？
5. **实际数据：**有没有消息，字段是否正确，频率是否合理？
6. **业务状态与日志：**是否碰墙、动作被中止、参数被拒绝？

下面这组只读命令适用于“默认海龟收不到速度”的检查：

```bash
echo "$ROS_DISTRO"
echo "$ROS_DOMAIN_ID"
echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"
ros2 node list
ros2 node info /turtlesim
ros2 topic list -t
ros2 topic info /turtle1/cmd_vel --verbose
```

发现具体原因后只改对应设置，再重新验证。不要同时重装 ROS、换中间件、改 Domain ID 和重写代码，否则很难知道究竟是哪一步解决了问题。

### 问题 1：`ros2: command not found`

先执行：

```bash
source ~/ros2_week1/setup_env.bash
ros2 --help
```

如果环境脚本不存在，回到 1.6 创建它。如果 `/opt/ros/jazzy/setup.bash` 不存在，说明 Jazzy 安装还没成功或版本不是 Jazzy；检查：

```bash
ls /opt/ros
dpkg -s ros-jazzy-desktop
```

不要用 `ros2 --version` 作为统一验证方式，`echo "$ROS_DISTRO"` 与 `ros2 --help` 更适合本手册。

### 问题 2：`Package 'turtlesim' not found` 或找不到可执行文件

```bash
source ~/ros2_week1/setup_env.bash
ros2 pkg executables turtlesim
```

确认拼写是 `turtlesim_node` 与 `turtle_teleop_key`。如果包没安装：

```bash
sudo apt update
sudo apt install ros-jazzy-turtlesim
```

如果安装的是其他发行版的包，回到环境选择，不要把 Humble、Jazzy 的 setup 文件连续 source 来碰运气。

### 问题 3：两个终端看不到彼此

在每个终端加载相同的 `setup_env.bash`，检查三个环境变量。结束旧的实验进程，然后从新环境重新启动。

如果实际程序能通信，但 CLI 节点列表疑似保留旧信息，可以在正确环境中重启发现缓存服务：

```bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

ROS 2 daemon 主要帮助命令行工具缓存发现信息，不是 ROS 1 的中心 master，也不是所有消息必须经过的数据转发站。重启它不等于重启你运行的全部节点。

### 问题 4：一直等待 subscriber 或 action server

等待不是一定卡死，而是当前所需通信对象尚未匹配。

- `topic pub --once` 等待：确认对应海龟存在、完整话题名正确。
- 等待两个订阅者：确认录制器也在运行且录制该话题。
- `action send_goal` 等待：用 `ros2 action list -t` 查实际 Action 名称。
- `service call` 等待：用 `ros2 service list -t` 查服务名称。

带 `/blue` 命名空间的系统，接口通常也要带 `/blue`。不要把“默认示例中的名字”当成任何实验都适用的固定名字。

### 问题 5：键盘没有反应

先点运行 `turtle_teleop_key` 的终端，再按方向键。不要把焦点放在海龟窗口、编辑器正文或另一个空终端。

如果键盘程序已经退出，重新启动。终端配置导致按键显示异常时，在退出键盘程序后可以运行：

```bash
stty sane
```

这用于恢复当前终端显示/输入设置，不会修复 ROS 通信本身。

### 问题 6：有 publishing 输出，海龟却不动

检查话题是否为当前海龟的 `cmd_vel`，是不是发送了全零消息，仿真器是否运行，海龟是否在边界，以及是否有其他发布者。

```bash
ros2 topic info /turtle1/cmd_vel --verbose
ros2 topic echo /turtle1/cmd_vel --once
ros2 topic echo /turtle1/pose --once
```

若发现旧发布者，回到对应终端 `Ctrl+C`；不要随意结束不认识的系统进程。

### 问题 7：YAML 报错、字段不存在、消息类型无效

先回到 `ros2 interface show` 查字段，检查英文冒号、冒号后的空格、引号与嵌套关系。

正确紧凑形式：

```text
'{linear: {x: 1.0}, angular: {z: 0.5}}'
```

`linear_x` 不是 Twist 的字段；应该使用 `linear` 下的 `x`。`angular.z` 是对嵌套成员的说明，不应直接当成本例 YAML 的一级键。

Jazzy 本手册使用 `turtlesim/msg/Pose`、`turtlesim/srv/Spawn`、`turtlesim/action/RotateAbsolute`。不要直接粘贴其他发行版中不同包名的接口。

### 问题 8：参数设置失败或文件不生效

检查参数类型、是否只读、节点名与文件路径：

```bash
ros2 param describe /turtlesim background_r
ros2 param get /turtlesim background_r
cat ~/ros2_week1/config/turtlesim.yaml
```

`ros__parameters` 是两个下划线；节点名要匹配；文件缩进使用空格。`ros2 param set` 的修改不会自动写回 YAML 文件。

### 问题 9：Action 立即中止或一直被打断

先检查是否还有键盘方向键输入或持续速度发布者。turtlesim 收到普通速度指令时，会中止正在执行的旋转目标。零速度消息同样是普通速度指令。

不要同时运行圆周 `topic pub` 和旋转 Action，再把两个控制源的冲突理解为 Action 功能坏了。

### 问题 10：rqt_graph 看不到想看的节点或话题

点击刷新，调整显示模式与 Hide 过滤项，用 `ros2 node list`、`ros2 topic list` 核对。没有订阅者的话题可能被图形过滤。

如果连命令行都看不到，回到环境与发现问题；只反复刷新图形界面不会解决 Domain ID 不一致。

### 问题 11：Qt 窗口打不开或提示无法连接显示器

确认是在 Ubuntu Desktop 的图形会话中打开终端，而不是只有命令行的服务器、无图形 SSH 会话或无显示设备的容器。

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

这两个值依桌面会话而不同。不要为了“让输出不为空”随便写一个 DISPLAY 值。先登录虚拟机图形桌面再启动 turtlesim；若你在 Conda 等环境里遇到 Qt 插件冲突，可退出该环境，在新的系统终端重新加载 ROS 环境后重试。

### 问题 12：录制目录已存在、MCAP 插件不存在

输出目录已存在时换一个新的实验编号，并同步修改后续播放路径。

如果明确报找不到 MCAP 存储插件：

```bash
sudo apt install ros-jazzy-rosbag2-storage-mcap
source ~/ros2_week1/setup_env.bash
```

不要把录制器创建的目录提前建好；只需要父目录 `~/ros2_week1/bags` 存在。

### 问题 13：回放与原轨迹不一致

依次确认：相同起点与朝向、没有其他速度发布者、只播放速度话题、没有碰墙、bag 中确实记录了速度与停止消息。

如果在观察当前状态时同时播放了历史姿态，先结束播放器，再按 6.8 只回放速度。几何形状相似但末端有小偏差，需要结合调度与时序分析；它不等于所有回放都必须逐浮点数一致。

### 问题 14：安装时 `Unable to locate package ros-jazzy-desktop`

先核对系统版本确实是 24.04，以及 1.4 是否成功安装软件源配置：

```bash
cat /etc/os-release
dpkg -s ros2-apt-source
sudo apt update
apt-cache policy ros-jazzy-desktop
```

如果 `apt update` 自己就在网络或签名处报错，先修复它。没有成功更新索引时反复执行 install 不会解决根因。

### 问题 15：依赖版本冲突或缺少 Ubuntu 更新仓库

Jazzy 官方文档提醒：某些 Ubuntu 24.04 安装的源只有基础 `noble`，可能导致开发工具依赖问题。[S1]

在通常的 24.04 源文件里检查：

```bash
grep '^Suites:' /etc/apt/sources.list.d/ubuntu.sources
```

Ubuntu 软件仓库条目的 Suites 通常应包含 `noble noble-updates noble-backports`，安全更新条目包含 `noble-security`。如果你使用的是传统 `.list` 格式，配置布局会不同，不要因为没有 `ubuntu.sources` 就另建一份重复源。

确认是缺少上述更新 suite 时，先备份再编辑对应现有文件：

```bash
sudo cp -n /etc/apt/sources.list.d/ubuntu.sources \
  /etc/apt/sources.list.d/ubuntu.sources.week1-backup
sudo nano /etc/apt/sources.list.d/ubuntu.sources
```

只修改实际 Ubuntu 仓库条目的 Suites，保留 URI、签名设置和独立安全更新条目。保存后运行 `sudo apt update`、`sudo apt upgrade`，再重试安装。不要把这些 Ubuntu suite 名写进 ROS 的仓库配置里。

如果报 `Signed-By` 冲突，通常要检查是否同时保留了旧 ROS 源配置和新的 ros2-apt-source 配置。先根据报错中的具体路径核对重复条目，再备份并处理旧条目，不要批量删除 `/etc/apt/sources.list.d`。

### 问题 16：VMware 的 Ubuntu 没网

本周系统运行在同一台 Ubuntu 内。先恢复虚拟机的基础联网能力，再继续下载：

1. 在 VMware 虚拟机设置的网络适配器中，确认“已连接”和“启动时连接”已勾选；普通入门实验可以使用 NAT。
2. 在 Ubuntu 设置的“网络”中确认有线网络已经连接。
3. 查看地址、默认路由与域名解析：

```bash
ip -brief address
ip route
getent hosts packages.ros.org
getent hosts github.com
```

如果没有非回环网络地址，检查虚拟网卡连接；没有 `default` 路由，检查网络是否成功获取网关；有地址和路由但域名解析失败，再检查 DNS。若系统使用 NetworkManager，可用：

```bash
nmcli device status
```

网站不能访问也可能是某个域名链路的问题，并不一定是 Ubuntu 完全没网。用浏览器或下面命令区分：

```bash
curl -I --connect-timeout 10 https://github.com
```

如果已装好 ROS，本周单机话题、服务、动作练习通常可以先继续；缺少的软件安装和下载仍要等网络恢复。

<a id="cheatsheet"></a>

## 命令速查与术语表

### 先记住命令家族，再记选项

下面是速查，不是要求把所有命令连续执行。持续运行的命令要在对应终端使用 `Ctrl+C` 结束。

| 目的 | 示例命令 |
|---|---|
| 新终端加载本周环境 | `source ~/ros2_week1/setup_env.bash` |
| 进入学习目录 | `cd ~/ros2_week1` |
| 查 ROS 发行版 | `echo "$ROS_DISTRO"` |
| 启动节点程序 | `ros2 run turtlesim turtlesim_node` |
| 列出包的程序 | `ros2 pkg executables turtlesim` |
| 查看节点列表 | `ros2 node list` |
| 查看节点接口 | `ros2 node info /turtlesim` |
| 列出话题及类型 | `ros2 topic list -t` |
| 查看话题类型 | `ros2 topic type /turtle1/pose` |
| 查看话题端点和 QoS | `ros2 topic info /turtle1/cmd_vel --verbose` |
| 显示一条消息 | `ros2 topic echo /turtle1/pose --once` |
| 持续观察消息 | `ros2 topic echo /turtle1/pose` |
| 测接收频率 | `ros2 topic hz /turtle1/pose` |
| 查看消息结构 | `ros2 interface show geometry_msgs/msg/Twist` |
| 明确发送停止指令 | `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{}'` |
| 查看服务 | `ros2 service list -t` |
| 查看服务类型 | `ros2 service type /spawn` |
| 清除轨迹 | `ros2 service call /clear std_srvs/srv/Empty '{}'` |
| 重置模拟场景 | `ros2 service call /reset std_srvs/srv/Empty '{}'` |
| 列参数 | `ros2 param list /turtlesim` |
| 读参数 | `ros2 param get /turtlesim background_r` |
| 改参数 | `ros2 param set /turtlesim background_r 40` |
| 导出参数 | `ros2 param dump /turtlesim` |
| 加载精简参数文件 | `ros2 param load /turtlesim ~/ros2_week1/config/turtlesim.yaml` |
| 查看 Action | `ros2 action list -t` |
| 查看 Action 端点 | `ros2 action info /turtle1/rotate_absolute` |
| 启动本周 launch | `ros2 launch ~/ros2_week1/launch/two_turtles.launch.py` |
| 查看图 | `ros2 run rqt_graph rqt_graph` |
| 查看日志 | `ros2 run rqt_console rqt_console` |
| 查看录制信息 | `ros2 bag info ~/ros2_week1/bags/circle_run_01` |
| 回放速度 | `ros2 bag play ~/ros2_week1/bags/circle_run_01 --topics /turtle1/cmd_vel` |

想查看本机已安装版本的选项，可以使用 `ros2 topic pub --help`、`ros2 bag record --help` 等。这是本地帮助，不需要联网；本手册涉及的实践命令已经给全，不要求你自己读完全部帮助页。

### 再补两个观察工具

在默认海龟运行时，只观察 theta 字段：

```bash
ros2 topic echo /turtle1/pose --field theta
```

测量观察者收到的消息数据量：

```bash
ros2 topic bw /turtle1/pose
```

先结束上一条持续观察命令，再运行下一条。`hz` 关注每秒接收多少条，`bw` 关注消息数据量随时间的变化；后者不等于包含所有协议开销后的精确物理网卡总带宽。

### Topic、Service、Action、Parameter 总结

| 机制 | 最核心的问题 | 例子 | 你要检查什么 |
|---|---|---|---|
| Topic | 有哪些数据正在流动？ | 速度、姿态、传感器 | 名称、类型、频率、QoS、字段 |
| Service | 一次操作请求处理得怎样？ | spawn、clear、reset | 请求与响应、服务端是否存在 |
| Action | 一个任务进行得怎样？ | 旋转、后续导航 | 目标是否接受、反馈、最终状态、取消结果 |
| Parameter | 这个节点使用什么配置？ | 颜色、轮径、采样频率 | 节点归属、类型、当前值、保存方式 |

### 本周核心术语

| 英文 | 中文 | 一句话解释 |
|---|---|---|
| Node | 节点 | ROS 2 系统中的逻辑通信参与者 |
| Package | 功能包 | 组织代码、接口和配置的单位 |
| Executable | 可执行文件 | 可以被启动的程序入口 |
| Workspace | 工作空间 | 管理一组开发包与构建结果的目录 |
| Publisher | 发布者 | 向话题发送消息的端点 |
| Subscriber | 订阅者 | 从话题接收消息的端点 |
| Topic | 话题 | 有名称和类型的数据通信通道 |
| Message | 消息 | 一份按接口结构组织的数据 |
| Interface | 接口 | 规定消息、请求响应或目标反馈结果的结构 |
| Client / Server | 客户端 / 服务端 | 提出请求或目标 / 处理请求或目标 |
| Request / Response | 请求 / 响应 | 一次服务交互的两部分 |
| Goal / Feedback / Result | 目标 / 反馈 / 结果 | Action 任务的业务数据 |
| Parameter | 参数 | 某个节点的配置项 |
| Namespace | 命名空间 | 帮助组织和隔离相对接口名的前缀 |
| Remapping | 重映射 | 启动时调整节点使用的接口名称等 |
| QoS | 服务质量配置 | 影响通信可靠性、历史记录等行为 |
| Launch | 启动描述 | 组织节点、名称、参数等启动步骤 |
| rosbag2 | 录制回放工具 | 保存并重新发布话题数据 |
| Pose | 位姿 | 位置与姿态；本例为 x、y、theta |
| Twist | 速度类消息 | 包含线速度与角速度分量 |
| Callback | 回调 | 事件到达时被安排执行的函数，下周写 C++ 时重点学习 |
| Executor | 执行器 | 管理回调如何被执行，下周继续学习 |

### 与已有 C++ 项目的对应关系

| 你的第二个项目中的内容 | 后面接入 ROS 2 时的定位 |
|---|---|
| 目标线速度、角速度 | 节点接收到的速度指令 |
| 差速逆运动学 | 把整车速度转成左右轮目标速度的内部算法 |
| 轮速 PID | 控制逻辑，不会因为使用 ROS 2 就自动生成 |
| 编码器仿真 | 数据来源，可以在节点内部调用或拆分 |
| 里程计积分 | 用于估计机器人运动的核心计算 |
| 参数解析 | 后续结合 ROS 参数与 YAML 配置 |
| CSV 输出 | 可以保留，同时增加话题与 rosbag2 记录 |

标准 ROS 机器人常用的 `nav_msgs/msg/Odometry` 与 turtlesim 的 `Pose` 不同。第一周借助简化接口理解概念，第三周再接入标准里程计与 TF，不要把 turtlesim 的接口直接当成最终机器人接口。

<a id="finish"></a>

## 验收标准与下周衔接

### 满足这些条件，就可以进入第二周

- [ ] 能从新终端加载环境，知道为什么其他终端也要加载。
- [ ] 能分清 package、executable、node、process、thread。
- [ ] 能找到一个节点的输入、输出及对应类型。
- [ ] 能构造 Twist，解释速度、朝向与发布频率的区别。
- [ ] 能用 Service 创建对象、修改状态，并解释请求响应结构。
- [ ] 能保存精简 YAML 参数文件，并在重启时加载。
- [ ] 能发送有反馈的 Action，知道取消与结束客户端的区别。
- [ ] 能用命名空间隔离两个仿真，解释各接口完整路径。
- [ ] 能用日志和端点信息定位一次简单问题。
- [ ] 能录制速度和姿态，并正确区分控制回放与历史数据观察。

只会复制命令、无法解释系统关系时，优先重做第 2、3、6 天。能解释关系但偶尔忘记命令拼写，可以继续用速查表进入下一周。

### 第一周最终保留的材料

| 材料 | 对应能力 |
|---|---|
| `setup_env.bash` | 环境一致性 |
| `config/turtlesim.yaml` | 可复用参数 |
| `launch/two_turtles.launch.py` | 多节点启动与命名空间 |
| `bags/circle_run_01` 或自己的实验编号 | 录制与回放 |
| `notes/week1-report.md` | 系统理解与实验记录 |
| 几张接口、轨迹或 rqt_graph 截图 | 运行证据 |

### 第二周从哪里开始

下一步创建真正的 `ros2_ws/src` 工作空间，用 `ament_cmake` 建立 C++ 包，编写自己的发布者、订阅者与定时器回调。

建议第一项编程练习复用你第一个项目的经验：一个 C++ 节点产生模拟传感器数据，另一个节点接收并处理。随后学习参数、服务与自定义消息；再把第二个项目的差速算法接进来。

到那时你已经知道程序运行起来之后应怎样用命令观察它，不会只盯着 C++ 源码而不知道系统实际在做什么。

<a id="sources"></a>

## 核对来源

以下为官方文档和项目源码入口，仅供追溯，不是必须额外阅读的课程任务。本手册的日程、例题组织、中文解释与综合练习为按你的学习进度编写；软件命令与行为以 Jazzy 对应版本为基准。

- **S1：**[Jazzy Ubuntu 安装文档](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)，并核对[文档仓库 Jazzy 分支](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)。
- **S2：**[官方软件源配置说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst)与 [ros-apt-source 项目](https://github.com/ros-infrastructure/ros-apt-source)。
- **S3：**[Understanding nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)。
- **S4：**[Understanding topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)。
- **S5：**[Understanding services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)。
- **S6：**[Understanding parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)。
- **S7：**[Understanding actions](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)。
- **S8：**[Launching nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)。
- **S9：**[Recording and playing back data](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)，以及 [rosbag2 Jazzy 命令实现](https://github.com/ros2/rosbag2/tree/jazzy/ros2bag/ros2bag/verb)。
- **S10：**[Using rqt_console](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)。
- **S11：**[turtlesim Jazzy 运动与 Action 实现](https://github.com/ros/ros_tutorials/blob/jazzy/turtlesim/src/turtle.cpp)、[场景实现](https://github.com/ros/ros_tutorials/blob/jazzy/turtlesim/src/turtle_frame.cpp)、[键盘客户端实现](https://github.com/ros/ros_tutorials/blob/jazzy/turtlesim/tutorials/teleop_turtle_key.cpp)。
- **S12：**[ros2 topic pub 的 Jazzy 实现](https://github.com/ros2/ros2cli/blob/jazzy/ros2topic/ros2topic/verb/pub.py)。
- **S13：**[ROS 2 QoS 概念](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)。
- **S14：**[配置 ROS 2 环境](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)与[改进动态发现范围](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)。

[S1]: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
[S2]: https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst
[S3]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html
[S4]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html
[S5]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html
[S6]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html
[S7]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html
[S8]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html
[S9]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html
[S10]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html
[S11]: https://github.com/ros/ros_tutorials/blob/jazzy/turtlesim/src/turtle.cpp
[S12]: https://github.com/ros2/ros2cli/blob/jazzy/ros2topic/ros2topic/verb/pub.py
[S13]: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
[S14]: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html
