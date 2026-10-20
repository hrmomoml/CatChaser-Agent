# 上位机和机器人主控部署路径

## 1. 本电脑：Ubuntu 22.04 / ROS 2 Humble

本机实测为 WSL2、Ubuntu 22.04、x86_64，ROS 2 Humble、rclpy、消息包、numpy、yaml、colcon 均已可用；不是需要重装 ROS 的空环境。上位机只运行动作、监视和 ROS 网络发送，不装板卡 ARM 内核，不占用 CAN，不需要构建 robot_py。

工作目录：`D:\CodexProjects\rpo_sim2real`，WSL 为 `/mnt/d/CodexProjects/rpo_sim2real`。

```bash
cd /mnt/d/CodexProjects/rpo_sim2real
source /opt/ros/humble/setup.bash
python3 scripts/doctor.py --role host
bash scripts/build_host.sh
source scripts/host_env.sh
```

另一台 Ubuntu 22.04 上缺依赖时，在 ROS 2 Humble 安装完成后补齐：

```bash
sudo apt install python3-colcon-common-extensions python3-numpy python3-yaml \
  ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-rmw-fastrtps-cpp
```

MuJoCo 导出使用项目现有的 `.venv` Python；ROS节点使用系统 Python 和 ROS 的构建环境，避免用一个不含 rclpy 的虚拟环境启动 ROS 节点。

## 2. 无真机时的完整消息演示

终端 A：

```bash
cd /mnt/d/CodexProjects/rpo_sim2real
bash scripts/run_mock.sh
```

终端 B：

```bash
cd /mnt/d/CodexProjects/rpo_sim2real
bash scripts/run_player.sh
```

终端 C：

```bash
cd /mnt/d/CodexProjects/rpo_sim2real
source scripts/host_env.sh
ros2 topic pub --once /rpo/command std_msgs/msg/String "{data: '7'}"
ros2 topic echo /rpo/status
```

输入 `8` 播放快扫，`7 8` 排队。`0` 或下列服务清队列并让模拟策略接管手臂：

```bash
ros2 service call /rpo/stop std_srvs/srv/Trigger '{}'
```

模拟目标使用 `/rpo_mock/joint_ref_states`，不会触及官方真机话题。播放器保持最后姿态直到停止，初始过渡和播放都有限速；动作实际耗时可能比原仿真长。

## 3. WSL2 与机器人联网

接到同一局域网，主控与上位机都使用 Humble、`ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=0`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；变量必须在启动节点前设置。

Windows 11 22H2 及以后可在已有 `%USERPROFILE%\.wslconfig` 的 `[wsl2]` 区段加入 `networkingMode=mirrored`，支持局域网直连和组播。合并已有设置，不覆盖其他配置；需要用户在 Windows 侧应用并重启 WSL。这里没有自动修改 Windows 网络或防火墙。

先在无电机动作的条件下测试：一端运行 `ros2 multicast receive`，另一端 `ros2 multicast send`，交换两端再测。随后在PC查看 `ros2 node list`、`ros2 topic list` 和 `/joint_states`。若组播或 DDS 失败，检查 Windows/Hyper-V 防火墙的受信局域网规则、路由和 DDS 配置；不要用关闭整个防火墙来绕过问题。仅 SSH 能连接不代表 DDS 可用。旧版 Windows/WSL NAT 可能需要 FastDDS Discovery Server 或改用原生 Ubuntu；本文没有验证这些替代网络方案。

Microsoft 原理：<https://learn.microsoft.com/en-us/windows/wsl/networking>

## 4. 机器人主控：SDK和硬件配置

以下在实机主控执行，不是在上位机的 WSL 执行。主控型号、BOM、接线、零位、IMU方向和机器人可靠支撑应先确认。源码读取或 mock 验证不能代替这些检查。

```bash
source /opt/ros/humble/setup.bash
git clone https://github.com/Roboparty/roboparty_deploy.git
cd roboparty_deploy
git checkout e4a1f064aa75b628f66eed9b10e891954bc9e0dc
git submodule update --init --recursive src/motors src/imu src/inference
sudo apt install ccache libfmt-dev libspdlog-dev libeigen3-dev screen zlib1g-dev \
  libboost-system-dev pybind11-dev python3-yaml python3-numpy ros-humble-joy
```

按官方说明配置适配该板卡的实时内核、运行用户 `rtprio 98` 和 memlock，重新登录/重启后验证；不能把 Orange Pi 的 ARM `.deb` 装到本电脑。核对四路 CAN 1 Mbps、udev物理绑定、IMU串口和电机零位。`robot.yaml` 的腰部 2.093 rad 软件偏移取决于标定方法，不能盲改。实际硬件配置与本项目默认配置不同，应先更新 profile 和重新验证映射。

把本项目复制到主控某个固定目录，例如 `~/rpo_sim2real`，应用小范围补丁：

```bash
bash ~/rpo_sim2real/scripts/apply_sdk_patch.sh ~/roboparty_deploy
cd ~/roboparty_deploy
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select roboparty_motors roboparty_imu roboparty_inference
source install/setup.bash
python3 ~/rpo_sim2real/scripts/doctor.py --role robot
```

完整 SDK 补丁目前没有在主控编译或连接硬件验证；上面是后续必须执行并通过的步骤。如果实际 SDK 版本不同，补丁脚本会拒绝应用，不会静默覆盖。

## 5. 首先连接真机手臂控制

主控上设置双方约定的域和网络变量，按官方流程标定后运行：

```bash
cd ~/roboparty_deploy
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash tools/start_robot.sh --robot rpo --policy interrupt
source install/setup.bash
```

官方脚本会启用项目 FastDDS 配置并后台运行推理和手柄。连接件、零位和急停确认完成、机器人可靠支撑后，按官方顺序初始化、复位并启动推理。初始化和推理会驱动真实电机：

```bash
ros2 service call /init_motors std_srvs/srv/Trigger '{}'
ros2 service call /reset_joints std_srvs/srv/Trigger '{}'
# 确认约 4 秒复位已经完成，机器人姿态正常，再启动。
ros2 service call /start_inference std_srvs/srv/Trigger '{}'
```

PC 先确认 `/joint_states` 连续更新、`/set_interrupt` 服务存在，然后在可靠支撑下进行低速小幅试验。现有导出的动作参考仍未经过真机验证，首次不应直接在自由站立状态执行完整 40 cm 动作。

```bash
cd /mnt/d/CodexProjects/rpo_sim2real
bash scripts/run_player.sh -p backend:=official_interrupt -p hardware_enabled:=true -p time_scale:=0.1
```

另一个PC终端 `source scripts/host_env.sh` 后，通过 `/rpo/command` 发送已审核的动作。播放器在收到有效反馈、发送首个目标后请求 `/set_interrupt`，让SDK显式进入上肢覆盖模式。

```bash
ros2 topic pub --once /rpo/command std_msgs/msg/String "{data: '7'}"
```

不启动本项目的 23 轴 SDK桥接器与官方推理节点同时控制 CAN。`/rpo/stop` 让策略接管手臂；按官方顺序停止推理、失能，才是整机停机流程。

## 6. 23轴上下肢接口：仅机械支撑调试

此路径用于关节方向、限位、低速目标验证，不具备双足平衡能力，浅蹲参考也还没有通过实机验证。先结束官方推理/CAN所有者。

无硬件演示：

```bash
ros2 run rpo_cat_control supported_sdk_bridge --ros-args -p profile:=config/rpo_23.yaml
ros2 service call /rpo_sdk/initialize std_srvs/srv/Trigger '{}'
```

主控上的 SDK 模式（须先 source 官方构建环境，再 source 本项目构建环境）：

```bash
ros2 run rpo_cat_control supported_sdk_bridge --ros-args \
  -p profile:=/absolute/path/rpo_sim2real/config/rpo_23.yaml \
  -p backend:=sdk -p hardware_enabled:=true -p supported_fixture:=true \
  -p sdk_config:=/absolute/path/roboparty_deploy/src/inference/robots/rpo/robot.yaml
```

此模式在调用 `/rpo_sdk/initialize` 后才打开SDK并使能；接收 `/rpo_sdk/joint_target` 的23维绝对关节目标。最大参考速度0.2 rad/s，0.5秒断流后保持最后发出的位置。异常和正常关闭会尝试失能，所以必须有机械支撑。Python定时器不保证硬实时。此节点的真实SDK后端尚未构建/硬件验证，只有mock后端已验证。

## 7. 发布 GitHub

本地代码按独立仓库组织；build/install/log、日志和生成的NPZ不进入Git。确定账号、仓库名、可见性并登录后再创建远端和 push。当前没有选定远端，也没有 GitHub CLI 登录环境，所以没有虚构发布链接。首次提交前设置自己的 Git 作者身份。
