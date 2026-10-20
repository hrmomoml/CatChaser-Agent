# RPO 23 自由度接口核对

本页对照用户粘贴的《部署与SDK》与锁定版本源码，不把旧文档的泛化描述当作当前接口契约。主仓库为 `Roboparty/roboparty_deploy`，推理、驱动、IMU 已分成 Git 子模块；版本见 `config/upstream.lock.json`。

## 型号和通信

| 项目 | 核实结果 |
|---|---|
| 本体 | RPO / ROBOTO ORIGIN，23 DOF：左腿 6、右腿 6、腰 1、左臂 5、右臂 5 |
| 已列主控 | Orange Pi 5 Plus 或 RDK X5；文档列 Ubuntu 22.04 |
| 电机驱动 | DM；配置枚举 0=DM4340P_48V，1=DM10010L_48V，默认配置共 14 个 0、9 个 1 |
| IMU | HIPNUC 驱动、串口 `/dev/ttyUSB0`、921600 baud；文档建议 500 Hz。具体 IMU 产品料号没有给出，需看实机 |
| CAN | 四路标准 CAN，文档给出 1 Mbps；`can0` 左腿 ID 1–6，`can1` 右腿和腰 ID 7–13，`can2` 左臂 ID 14–18，`can3` 右臂 ID 19–23 |
| 控制周期 | 粘贴文档称 200 Hz；锁定的 `interrupt.yaml` 为 dt=0.004，即 250 Hz 电机目标周期，decimation=5，即 50 Hz 策略推理。以实际部署版本配置及实测为准 |

电机型号来自默认 YAML 和枚举定义，不替代实机 BOM 核对。上位机不接 CAN；上述接线、udev、IMU 和实时内核属于机器人主控。

## SDK

- 部署框架：<https://github.com/Roboparty/roboparty_deploy>
- 推理和机器人级 SDK：<https://github.com/Roboparty/roboparty_inference>，构建生成 `robot_py`。
- 电机 SDK：<https://github.com/Roboparty/roboparty_motors>，构建生成 `motors_py`。
- IMU SDK：<https://github.com/Roboparty/roboparty_imu>，构建生成 `imu_py`。

不是从 PyPI 安装同名模块。需要递归获取官方源码，在主控构建并 source `install/setup.bash`。

`motors_py` 暴露 MIT、POS、SPD，分别对应位置/速度/刚度/阻尼/前馈力矩组合、位置控制、速度控制。`NONE` 不是实际控制模式。机器人统一接口在 MIT 控制链路中使用：

```python
robot = robot_py.RobotInterface(robot_yaml, control_dt=0.004)
robot.apply_action(p, v=[], kp=[], kd=[], tau=[])
```

`p` 是 23 个逻辑关节绝对角度 rad，不是可以任意解释的“位置或力矩动作”；`v` 为 rad/s、`tau` 为前馈力矩。空 v/tau 使用零值，空 kp/kd 使用官方 robot.yaml。SDK处理 `urdf2motor`、电机方向、软件零偏和并联脚踝解耦。PC不能重复施加这些变换。当前 MuJoCo 的 kp=1200、隐式阻尼、重力/惯性前馈不能原样抄入真机。

## 官方现有 ROS 2 接口

| 名称 | 类型 | 用途 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 策略速度命令，需要切换到 cmd_vel 控制模式；不用于指定腿关节或蹲下 |
| `/joint_ref_states` | `sensor_msgs/msg/JointState` | 仅 Interrupt 模式覆盖十个手臂关节 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 23 个逻辑关节反馈；锁定版本的 name 是 joint_1..joint_23 |
| `/imu` | `sensor_msgs/msg/Imu` | IMU反馈 |
| `/init_motors`、`/deinit_motors` | `std_srvs/srv/Trigger` | 使能、失能 |
| `/start_inference`、`/stop_inference` | `std_srvs/srv/Trigger` | 策略启动、暂停；暂停不等于失能 |
| `/reset_joints`、`/read_joints`、`/read_imu` | `std_srvs/srv/Trigger` | 姿态复位、状态读取 |

零位服务会修改硬件零点，不属于环境检查或正常动作指令。

### 十轴手臂消息

```yaml
position: [left_pitch, left_roll, left_yaw, left_elbow_pitch, left_elbow_yaw,
           right_pitch, right_roll, right_yaw, right_elbow_pitch, right_elbow_yaw]
```

全部为绝对 rad。锁定版本只读取 position，忽略 name、velocity、effort。发送 23 维消息不会控制腿；原回调甚至不会校验维数，可能错误地把前十项当作手臂。本项目补丁明确要求恰好十维、有限值且在配置限位内。

### 本项目扩展，不能当作官方已有接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/set_interrupt` | `std_srvs/srv/SetBool` | 补丁新增的幂等切换服务；原版通过手柄 LB 切换 |
| `/rpo/command` | `std_msgs/msg/String` | 本项目动作播放器的 3..8 数字命令，空格排队；0 停止 |
| `/rpo/stop` | `std_srvs/srv/Trigger` | 清队列，要求策略重新接管手臂；不失能 |
| `/rpo/status` | `std_msgs/msg/String` | JSON运行状态 |
| `/rpo_sdk/joint_target` | `sensor_msgs/msg/JointState` | 本项目实验性、机械支撑模式的 23 轴 SDK 桥接；不提供平衡 |

补丁也增加 0.5 秒报文超时后退出外部手臂模式、接收过外部报文后的手臂目标 0.5 rad/s 限速。回退是让原策略接管手臂，不是急停。完整补丁尚需在实际主控编译和验证，不能把模拟后端测试当作这项验证。

## 原文与当前版本差异

原文 `atom01_deploy`、`src/inference/config/robot.yaml`、`inference_interrupt.yaml` 等路径与当前子模块布局不同；当前使用 `src/inference/robots/rpo/robot.yaml`、`configs/interrupt.yaml`，启动参数为 `--robot rpo --policy interrupt`。原文列出的 lock/unlock 等成员也不能直接假定当前 Python绑定存在，应查对应版本的 pybind_module.cpp。

证据链接和固定版本见 [SOURCES.md](SOURCES.md)。
