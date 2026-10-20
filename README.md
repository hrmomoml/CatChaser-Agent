# RPO 仿真到真机任务迁移

将现有23轴RPO MuJoCo逗猫动作导出为角度参考，通过Ubuntu 22.04 / ROS2 Humble上位机发送给机器人主控。默认完全使用mock，不打开CAN、不初始化电机。

**已实现并在电脑验证：动作3～8的导出、十轴手臂ROS播放器、模拟机器人、23轴支撑调试桥接的mock后端、接口校验与断流处理。** 真机SDK完整编译、跨机网络、标定和硬件运动尚未验证。浅蹲参考已调整到部署配置限位内并通过12种组合仿真；脚踝连接处几何和真机平衡仍待验证，不能直接自由站立重放动作1/2。

明确路径：

```text
MuJoCo动作 → NPZ参考 → PC ROS2播放器 → 主控官方Interrupt策略 → SDK → 电机
                                      腿/腰由策略控制，双臂接受外部参考
```

- [型号、SDK、控制模式和真实接口](docs/INTERFACES.md)
- [PC环境、主控配置和逐步部署命令](docs/DEPLOYMENT.md)
- [现有文件如何迁移、NPZ/ONNX区分、蹲起缺口](docs/MIGRATION.md)
- [可以复制使用的ROS2命令格式](docs/COMMANDS.md)
- [验证结果与未验证范围](docs/VALIDATION.md)
- [固定版本来源](docs/SOURCES.md)
- [可复用的rpo-sim2real skill](skills/rpo-sim2real/SKILL.md)

## 在本电脑开始

```bash
cd /mnt/d/CodexProjects/rpo_sim2real
bash scripts/build_host.sh
```

终端A运行 `bash scripts/run_mock.sh`，终端B运行 `bash scripts/run_player.sh`，终端C运行：

```bash
source scripts/host_env.sh
ros2 topic pub --once /rpo/command std_msgs/msg/String "{data: '7 8'}"
```

`0`停止，`3`上下、`4`左右、`5`画圆、`6`前伸停顿收回、`7`六周期正弦、`8`左右快扫停顿。默认减速并限制关节目标变化率，所以停顿和总时长也会随播放速度变化。

当前D盘已经生成动作NPZ。Git不包含这些生成文件；仓库包含现有动作生成器快照 `sim_source/`。克隆项目后，先获取固定版本的RPO模型，再导出：

```bash
bash scripts/fetch_model.sh
/home/gyl/cat_agent_ws/.venv/bin/python scripts/export_motion.py --library
```

导出环境需要mujoco、numpy、PyYAML；ROS节点使用系统Python。换机器时通过 `--source`、`--repo` 指定原动作程序和模型路径。

## 真机接口边界

官方 `/joint_ref_states` 是十轴手臂角度接口，不是23轴任意关节命令；本项目 `/rpo/command` 是新增上层接口。提供的SDK补丁增加明确模式切换、报文检查和超时回退，必须在主控应用并编译后再联调。补丁完整构建和硬件行为尚未验证。

`/rpo_sdk/joint_target` 则是本项目另一个实验性23轴支撑调试接口，调用 `robot_py.apply_action`，需要独占CAN并可靠支撑机器人，不提供双足平衡。详见部署文档。

## 代码和发布

ROS包在 `src/rpo_cat_control`；模型和SDK通过固定版本引用，不打包上游二进制。项目采用GPL-3.0-only，包含对GPL上游的小范围补丁；引用的模型遵循其自身许可证。参考轨迹和仿真源码的来源见导出元数据。

GitHub目标账号、仓库和可见性尚未确定。本地仓库已准备，当前没有上传，也不包含GitHub凭据。

在其他电脑生成参考时可单独创建导出环境：

```bash
python3 -m venv .venv-export
.venv-export/bin/pip install -r requirements-export.txt
bash scripts/fetch_model.sh
.venv-export/bin/python scripts/export_motion.py --library
```
