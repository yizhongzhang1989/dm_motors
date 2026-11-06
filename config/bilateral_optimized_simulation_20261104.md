# 低阻力双边力控仿真

`bilateral_optimized_simulation_20261104.yaml` 在改进后的双轴电机模型上选择 `differential_torque`。它保留 K=50、Dc=0.4975、本地阻尼 B=0.005、摩擦补偿比例0.6，把两轴之间的弹簧改为主机计算的反向力矩，原生 MIT 使用 Kp=0、Kd=1。

两轴共同运动时不再承受滞后的共同位置参考产生的位置回路阻力。反馈滤波、剩余摩擦和转动惯量仍然存在。参数、对照数据、失败方案和验证范围见[优化报告](../src/dm_motor_bilateral_control/docs/2026-11-04-differential-simulation.md)。

本配置使用虚拟 CAN、空硬件端口、`require_virtual_bus=true`，独立命名空间 `bilateral_optimized_simulation`。补偿文件只含 simulation 树，沿用辨识场景的原标定律。此配置未开启真机试验许可，节点和 Dashboard 两处均要求两轴为模拟电机；默认共享参考配置保持原样。独立真机试验入口的只读许可见[控制器说明](../src/dm_motor_bilateral_control/README.md)。

构建本次源代码后，从项目根目录运行（8184须未被其它实例占用）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=143 ros2 launch dm_motor_bringup bilateral_control.launch.py \
  simulation:=true \
  config_file:="$PWD/config/bilateral_optimized_simulation_20261104.yaml" \
  motor_dashboard_ports:='{motor_1: 8181, motor_2: 8182}'
```

双边面板为 <http://127.0.0.1:8184/>，两轴外力面板为8181/8182。打开页面不会自动使能；点击“开始”后可在电机面板施加外力，再撤去外力检查停稳。已有本轮隔离仿真实例时直接使用其面板，无需重复启动。

`max_torque=0` 表示没有额外应用层总力矩预算。弹簧力仍须通过 MIT 前馈字段编码，接近协议上限时两侧按同一比例缩小弹簧，保留原生阻尼；面板显示耦合需求受限。模型自身的约7.8Nm饱和平台也仍然生效。

模型参数与 `bilateral_identification_20261104.yaml` 一致，包括31.25µs积分步长、反馈滤波和延迟；在线虚拟总线没有另加串口传输延迟。离线回归使用显式的因果命令/反馈队列覆盖额外时延，不把在线和离线的力矩数值视作同一个实验。
