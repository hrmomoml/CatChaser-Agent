# ROS 2 命令格式

先在每个上位机终端 source `scripts/host_env.sh`。以下 mock 命令不会连接电机。

## 本项目数字动作

```bash
ros2 topic pub --once /rpo/command std_msgs/msg/String "{data: '7'}"
ros2 topic pub --once /rpo/command std_msgs/msg/String "{data: '8'}"
ros2 topic pub --once /rpo/command std_msgs/msg/String "{data: '7 8'}"
ros2 topic pub --once /rpo/command std_msgs/msg/String "{data: '0'}"
```

3上下、4左右、5画圆、6前伸停顿收回、7六周期正弦、8左右快扫各停两秒。初始硬件路径会减速并限速；时间缩放也相应延长停顿。`1`、`2`、`1+8` 当前会返回明确的不支持错误。

## 十轴手臂底层消息

下面只展示形状，用模拟话题和官方默认手臂角度；直接命令不会先自动切换模式：

```bash
ros2 topic pub --rate 50 /rpo_mock/joint_ref_states sensor_msgs/msg/JointState \
  "{position: [0.18, 0.06, 0.0, 0.78, 0.0, 0.18, -0.06, 0.0, 0.78, 0.0]}"
```

消息持续发送时，在另一个终端：

```bash
ros2 service call /rpo_mock/set_interrupt std_srvs/srv/SetBool '{data: true}'
```

真机同类接口为 `/joint_ref_states` 和补丁提供的 `/set_interrupt`；必须先完成主控部署、标定、支撑和推理准备。原始话题不接受23轴控制语义。正常任务建议用有校验、反馈和限速的播放器，不直接手写底层消息。

## 二十三轴支撑调试的模拟后端

先运行 `supported_sdk_bridge` 的默认mock后端，并调用 `/rpo_sdk/initialize`。以下以左膝默认0.3rad增至0.35rad作消息示例；其余关节维持官方默认目标：

```bash
ros2 topic pub --rate 20 /rpo_sdk/joint_target sensor_msgs/msg/JointState \
  "{position: [0.0, 0.0, -0.1, 0.35, -0.2, 0.0, 0.0, 0.0, -0.1, 0.3, -0.2, 0.0, 0.0, 0.18, 0.06, 0.0, 0.78, 0.0, 0.18, -0.06, 0.0, 0.78, 0.0]}"
```

23维消息是**本项目桥接器新增的接口**。此节点接收全部逻辑关节并调用SDK，不是官方Interrupt订阅，也没有腿部平衡算法。用于实际SDK时必须更换 backend、显式开启hardware_enabled和supported_fixture，在主控独占CAN。当前只有mock后端经过验证。

## 状态与停止

```bash
ros2 topic echo /rpo/status
ros2 topic echo /rpo_mock/joint_states --qos-reliability best_effort
ros2 service call /rpo/stop std_srvs/srv/Trigger '{}'
ros2 service call /rpo_sdk/deinitialize std_srvs/srv/Trigger '{}'
```

本项目停止手臂动作会让策略接管手臂。主控整机的暂停推理、失能电机是官方服务，按DEPLOYMENT的顺序执行；二者不是同一含义。
