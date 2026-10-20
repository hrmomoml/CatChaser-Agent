# 来源及版本

用户提供的飞书文档：<https://roboparty.feishu.cn/wiki/IyKNwwGcCiyjEckV9QucEuHcn10>。浏览请求跳转登录页，随后用户粘贴正文，分析已使用该正文。没有声称直接读到了受登录保护的在线页面。

固定版本：

- [部署主仓库 e4a1f064](https://github.com/Roboparty/roboparty_deploy/tree/e4a1f064aa75b628f66eed9b10e891954bc9e0dc)
- [RPO硬件配置](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/robots/rpo/robot.yaml)
- [Interrupt配置](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/robots/rpo/configs/interrupt.yaml)
- [ROS订阅与发布定义](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/src/inference_node.hpp)
- [ROS回调实现](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/src/ros_interface.cpp)
- [策略到关节目标的覆盖实现](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/src/inference_node.cpp)
- [机器人SDK及并联解耦调用](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/src/robot_interface.cpp)
- [Python绑定的准确签名](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/src/pybind_module.cpp)
- [官方MotionLoader数据格式](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/src/utils/motion_loader.cpp)
- [motion_command与关节观测顺序](https://github.com/Roboparty/roboparty_inference/blob/798f254a1754daff0a964a0e2db58e1c5a5450f7/src/obs_manager.cpp)
- [电机型号枚举说明](https://github.com/Roboparty/roboparty_deploy/blob/e4a1f064aa75b628f66eed9b10e891954bc9e0dc/docs/configuration_CN.md)
- [WSL2镜像网络及组播](https://learn.microsoft.com/en-us/windows/wsl/networking)

RPO模型版本：`8e0f8f78903dc147a5c094c0fdc14b4a78f1b667`。现有动作控制器是本次会话生成、保存在相邻 rpo_description/cat_play 下的程序，具体SHA256写在导出JSON。没有把RPO 23轴配置套到RP1模型上。
