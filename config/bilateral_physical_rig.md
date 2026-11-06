# 三电机双边力控物理台架配置

[bilateral_physical_rig.yaml](bilateral_physical_rig.yaml) 是当前 A/B/C 台架的显式真机运行配置，复用现有启动代码。它固定了 2026-11-05 所选的 B100 减阻参数：13.2 rad/s 往返、释放、统一冷启动及标定历史恢复已通过；**V16正式连续变速验收通过，总长302.04 s、运动301.24 s、17组、408次速度目标阶跃，全部高速固定窗口三轴实测均达到4π rad/s，结束后三轴停止确认**。软件与交付验证的作用域见[阶段报告](../docs/experiment-history.md#three-motor-bilateral-baseline-2026-11-05)。普通 [dm_motors.yaml](dm_motors.yaml) 的默认行为没有改变。

相同B100系数的离线补充也已通过：新增3个短场景加保留的释放覆盖固定4场，另完成一次302.04 s序列。原长序列门与补充实际速度诊断分开，释放仍只有0.1238个百分点余量；条件及原B80证据边界见[仿真验证范围](../docs/experiment-history.md#three-motor-bilateral-baseline-2026-11-05)。

本配置只创建真实 CAN 服务、三个驱动、双边控制器及五个面板。启动、打开网页和重连均不使能。它不运行轨迹，不自动清错、设零或保存设备 Flash，也不启动独立试验守卫。

## 接线与控制关系

| 轴 | 设备 / namespace | 作用 |
| --- | --- | --- |
| A | id1 / `/motor_1` | 双边被拖动轴，与 C 同轴直连 |
| B | id2 / `/motor_2` | 机械自由，通过双边控制跟随 A |
| C | id3 / `/motor_3` | 台架驱动轴，执行有限运动来拖动 A |

三台型号相同，profile 为 `j4310_v12`。实测关系是 `Δq1 ≈ −Δq3`，物理传动比为 1:1。三台驱动仍使用 `direction=+1`、软件零偏 0；本配置没有改变坐标，也没有把识别回归斜率当作齿轮比。A/B 双边方向为 +1。换接、重装或位置基准变化后，重新检查实际关系和轨迹起点。接法证据见[第三电机物理测试报告](../docs/experiment-history.md#third-motor-physical-test-2026-11-05)。

## 参数与边界

| 参数 | A/B | C |
| --- | ---: | ---: |
| 控制 / 状态频率 | 500 / 500 Hz | 500 / 500 Hz |
| 模式 | MIT | MIT |
| 启动 RAM 速度带宽 | 100 Hz | 40 Hz |
| 指令超时 | 0.25 s | 0.12 s |
| 设备通信 watchdog | 0.2 s | 0.1 s |
| 反馈超时 | 0.1 s | 0.08 s |
| 请求 / 报告速度门限 | 26 / 26 rad/s | 16 / 16 rad/s |
| 请求 / 估算 / 报告力矩门限 | 0 / 0 / 0：不加额外门限 | 4 / 4 / 4 Nm |
| MOS / 转子温度门限 | 50 / 50 °C | 48 / 48 °C |
| 主机位置范围 | ±350 rad | ±350 rad |
| 保护连续样本数 | 1 | 1 |

三轴启用主机保护。启动时在失能条件下读取速度带宽，仅在需要时修改RAM并独立读回，A/B为100 Hz、C为40 Hz，不写Flash。每轮台架实验还要独立读取C的实际值并记录；不同值不能与本分组混合比较。

主机位置范围与 MIT 位置命令编码范围不是同一项。当前 PMAX=12.5，C 的位置参考必须留在原生 ±12.5 rad 内；高速度长行程前需要检查完整轨迹包络。26/16 rad/s 和 4 Nm 是运行保护设置，不是已完成的测试速度或标定有效范围。每轮实验还应有更严格、明确的速度、行程、力矩、温度和持续时间边界。

双边采用 `differential_torque`，显式 `allow_differential_hardware=true`；K=50 Nm/rad、耦合阻尼 0.4975、本地阻尼 0.005 Nm·s/rad，500 Hz，对应原生 P0/D1。`max_torque=0` 不加应用层预算，仍受设备与协议能力约束。控制器每次 START 独立读回 A/B 的 V_BW=100。

原摩擦补偿开启，比例0.6，使用实测速度。B另加独立台架辨识得到的粘性增益0.02584737521699079 Nm/(rad/s)，速度输入上界13.2 rad/s；A的附加增益为0。两部分共享每轴0.4 Nm补偿上限、原滤波/换向复位/斜率限制。这是显式控制增益，不是把原标定的低速适用域扩大。原模型仍来自同目录 [system_parameters.yaml](system_parameters.yaml) 的真实硬件注册表，没有被替换。控制原理与移植步骤见[双边力控教程](../docs/bilateral-control-tutorial.md)。

B100表示B附加粘性系数取独立辨识值的100%，不是把原摩擦补偿比例改成100%。13.2 rad/s分组固定窗中，C平均驱动力矩为0.64685 Nm，比B80的0.7101 Nm低8.9%，相对V9原补偿的1.01859 Nm低36.5%。8.9%来自V14/V15同一控制器实例；V9至V15跨控制器版本，36.5%仅描述整套阶段结果，不能单独归因于B100增益。C仍机械连接，其摩擦和惯量参与拖动及释放；这些结果不等于独立A物理标定、惯量消除或任意负载下的最优补偿。

## 冷启动

以下命令仅用于原有 CAN、三台驱动和双边进程已结束之后的完整冷启动。当前实例仍在运行时，使用现有面板。首次部署先按[教程第8.1节](../docs/bilateral-control-tutorial.md#simulation)从当前完整源码构建工作区，记录该次构建实际选择的安装前缀。下面的`/absolute/path/to/this-build/install`是占位符，须替换为该前缀；仓库中已有`install/`不等于已构建或已验收当前版本。然后在仓库根目录执行：

```bash
dm_install_prefix="/absolute/path/to/this-build/install"
source /opt/ros/humble/setup.bash
source "$dm_install_prefix/setup.bash"
ros2 launch dm_motor_bringup bilateral_control.launch.py \
  config_file:="$PWD/config/bilateral_physical_rig.yaml" \
  simulation:=false
```

本机V16的已验收安装前缀为`~/.ros/dm_motor_bilateral_control/three-motor-optimization-20261105/final-current-install`，其中双边节点SHA256为`ea0dbba86a6062e8a370385198d4913fcfb72005bc6113ba2a89851b4ef6642c`；这是[本机冷启动记录](/home/robot/.ros/dm_motor_bilateral_control/three-motor-optimization-20261105/runtime/selected-b100-cold-start-01/DEPLOYMENT.json)中的产物身份，不是其他机器的固定安装路径或预期哈希。

该文件采用项目 `version: 2` 配置格式，传给 `config_file`，不能当作 ROS `--params-file`。串口使用当前台架的 `/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00`。迁移到其他适配器时编辑该配置中的真实端口；不要把当前标定数值直接套到另一台实物。

现有 `dm_motor_bringup/bilateral_control.launch.py` 沿用同一 profile 调用驱动组合与控制器附加入口。其通用总线组合支持任意配置电机列表，因此会创建三个驱动，无需新增或复制 launch 文件。默认普通双边入口仍只有 A/B；必须显式传入本文件才能得到本台架配置。

| 面板 | 本机地址 |
| --- | --- |
| CAN | <http://127.0.0.1:8080/> |
| A | <http://127.0.0.1:8081/> |
| B | <http://127.0.0.1:8082/> |
| C | <http://127.0.0.1:8088/> |
| 双边 | <http://127.0.0.1:8084/> |

默认监听 `0.0.0.0`；只需本机访问时可以追加 `dashboard_host:=127.0.0.1`。本配置不启动标定节点或标定面板。

启动完成后先确认三轴 `ready`、真实反馈新鲜、失能且无活动目标，双边处于停止；核对模式、驱动频率、限值、模型路径及实际速度带宽。使能后才能确认实际原生 P0/D1 和双边补偿状态。面板能打开或 YAML 解析成功都不能代替这些运行时检查。

## 已有驱动时只附加控制器

仅当现有三台驱动已按上述参数准备、双边控制器尚未运行且 8084 未占用时，使用：

```bash
ros2 launch dm_motor_bilateral_control bilateral_control.launch.py \
  config_file:="$PWD/config/bilateral_physical_rig.yaml" \
  config_package:=dm_motor_bringup \
  config_launch:=bilateral_control.launch.py
```

这条命令只创建控制器及其面板，不初始化或修改已有驱动。显式指定两个 profile 选择参数，使附加入口与完整启动使用同一配置；本文件没有单独的 attach profile。所有进程必须使用一致的 ROS 域和 namespace。

双边面板的停止按钮控制 A/B。C 属于台架驱动轴，完整自动试验必须由独立守卫在正常结束和异常时停止 **A/B/C 三轴**，并等待 STOP 请求之后的新鲜失能反馈。此 YAML 不会替代守卫、有限轨迹、记录器或试验报告。当前高速度入口是[独立双边台架工具](../tools/physical_rig/bilateral/README.md)，有固定短轮、释放、连续变速试跑和正式长轮，默认仅生成计划。C的轨迹增益使用已测试的P10/D0.6。旧单轴 `rig_compare.py` 已移出生产树；所有联轴模式复用当前有限轨迹和三轴停机入口。

## 仿真与验证状态

这是硬件专用配置，刻意没有 `simulation` variant。传入 `simulation:=true` 会在创建设备前拒绝配置。三电机耦合离线试验使用[独立 fixture](../src/dm_motor_bilateral_control/test/three_motor_rig/README.md)；普通 ROS 虚拟电机各自独立，不能据此认为 A/C 自动形成物理连接。

本文件已通过离线配置展开检查：直接调用当前生产 `ConfigManager` 和 launch 组合函数，将 `Node` 动作替换为字典，不执行 LaunchService、不创建 ROS 节点、不打开串口。核对了：

- 1 个真实 CAN、3 个驱动、1 个双边控制器、5 个互不冲突的面板。
- 三轴所有保存的实际驱动参数、控制器参数及真实 CAN 参数，与本轮基准快照一致；`config_file` 改为本文件，ROS 自动参数字段按默认处理。
- 三轴各自的超时与面板一致；共享模型仍指向原 `system_parameters.yaml`。
- 附加入口只创建相同控制器和面板；请求 simulation variant 在设备创建前被拒绝。

上述离线检查与后续真机记录分别保留。统一冷启动已核对当前安装产物、运行参数和标定历史恢复；单组试跑及V16正式长测均通过，正式评分为`passed=true`、`final_acceptance=true`。原始评分保存在现场证据目录的`analysis/V16_COLD_B100_FULL_01.json`，方法与限制见[三电机阶段报告](../docs/experiment-history.md#three-motor-bilateral-baseline-2026-11-05)。这些实测只覆盖本配置和当前台架；软件清单另按当前输入运行与依赖等价保留完成核对，不能把运动验收或历史条目计数当作全新软件运行。
