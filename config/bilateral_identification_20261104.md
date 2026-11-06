# 真机记录对应的双边仿真候选

`bilateral_identification_20261104.yaml` 提供本轮手动记录对应的显式场景：shared_reference K50、native100/1、摩擦补偿0.6、逐轴原标定惯量/摩擦、有效反馈/报告动态和约7.8Nm饱和平台。参数、数据及验证边界见[实验报告](../src/dm_motor_bilateral_control/docs/2026-11-04-manual-identification.md)。这不是已完全辨识的数字模型，也不代表双边拖动阻力已经消除。

配置使用虚拟总线、空硬件端口、独立命名空间 `manual_identification_replay`，并要求驱动连接虚拟总线；不会自动使能或施加外力。补偿文件 `bilateral_identification_parameters_20261104.yaml` 只含 simulation 树，保留原硬件补偿律及其来源，原硬件参数文件不受影响。

完成包含最新 `dm_motor_control` 的常规工作区构建后，在项目根目录运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=137 ros2 launch dm_motor_bringup bilateral_control.launch.py \
  simulation:=true \
  config_file:="$PWD/config/bilateral_identification_20261104.yaml"
```

候选双边面板默认 `http://127.0.0.1:8184`。需要两个虚拟电机的手动外力面板时，在上述命令后增加：

```bash
  motor_dashboard_ports:='{motor_1: 8181, motor_2: 8182}'
```

新参数 `constant_torque_speed` 需要本次更新后的仿真节点；不能用旧的已安装节点运行后便假定恒扭矩区间已生效。本次只进行了离线模型/代码验证，没有替换正在运行的真机程序。

候选保留原J/Fc，内部附加速度滤波取0、力矩响应2ms，对外速度报告滤波4.5ms/延迟1.5ms，力矩报告滤波12ms/延迟2ms。内部0是有效近似，不表示真实固件零延时。力矩上限A7.87/B7.84Nm、恒扭矩区间8rad/s，后者不是唯一辨识出的真实拐点。步长31.25µs已与半步长比较。

原始CAN回放保留真实指令时序；交互式虚拟总线仍没有单独注入真实串口/设备的传递延迟，因此本配置尚不能保证在线拖动手感与真机完全一致。初始位置/速度为0，并非任一记录的初态。热参数维持旧值，已发现的温升及MOS模型偏差见实验报告。

完整生成依据及静态检查保存在 `~/.ros/dm_motor_bilateral_control/stability-20261102/manual-identification/replay-profile/`。项目副本仅保留组合双边场景及所需节点默认值，把补偿文件路径改为相对路径，并将双边面板改为8184、默认关闭其它面板。
