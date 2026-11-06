# 三轴双边力控有限拖动试验

`run_trial.py` 用 C/id3 驱动与其反向直连的 A/id1，B/id2 通过所选双向耦合或A→B单向模式跟随。每次运行一轮有限往返、C 中性释放或连续变速试验，保存观测，并在结束或异常时停止三轴。默认只输出计划；只有显式 `--execute` 才连接 ROS、设置本轮耦合模式与补偿开关并使能。

这是从 2026-11-05 现场 V4 试验程序整理的可复用入口。**原双向`bilateral`版本已完成独立构建、离线及fake检查，并在所选B100/C40台架配置下通过短轮、释放、统一冷启动后的试跑和正式302.04 s连续变速实测；软件与交付的证据作用域见[阶段报告](../../../docs/experiment-history.md#three-motor-bilateral-baseline-2026-11-05)。** 新增单向模式的验证范围见下文，不能沿用V16验收。工具不自动发现接法、不启动设备、不设零、不保存 Flash，也不执行物理移位。C/A 接法、方向和起点需事先核对；本目录不包含现场使用的独立停车/移位程序。

## 文件与分工

| 文件 | 作用 |
| --- | --- |
| `run_trial.py` | 前置核验、操作会话、守卫心跳、有限试验、三停与耦合/补偿参数恢复 |
| `native_stream/stream.cpp` | 独立 C++ 进程，500 Hz 向 C 发送 MIT 轨迹，P/D默认20/0.6；不使能 |
| `stop_guard.py` | 独立 STOP-only 守卫；不使能、不发送目标、不改参数 |
| `record_only.py` | 独立只读记录器；不创建硬件发布者或服务客户端 |
| `score_release.py` | 仅离线审核完整释放记录，以实际 C 中性 CAN 帧为观察起点 |
| `compare_coupling.py` | 离线比较单向/双向完整短轮；按独立轮次统计C驱动力矩 |
| `test_offline.py` | 不加载 ROS 的 CLI、输出保护及守卫边界检查 |

固定 namespace 为 `/motor_1`、`/motor_2`、`/motor_3`、`/bilateral_control`、`/can/main`。仅支持目前已确认的同型号、A/C 同轴反向 1:1 接法，不能通过只修改 CLI 来支持别的机构。`bilateral_friction`/`bilateral_no_friction`分别开启/关闭两轴补偿，均保持双向耦合；`unilateral_friction`保持两轴补偿、改为A→B单向跟随；`passive`仅使能C、A/B失能。

## 环境与独立构建

首次部署先按[教程第8.1节](../../../docs/bilateral-control-tutorial.md#simulation)从当前源码完整构建工作区，再选择该次构建的实际安装前缀。已有设备进程时，该前缀必须与正在运行的驱动和控制器对应；不能仅因仓库存在`install/`就将其当作已验收产物。所有辅助进程继承当前环境，必须使用与设备一致的 `ROS_DOMAIN_ID`、RMW 和接口版本。工具不读取某个固定 PID 的环境，也不覆盖你的 ROS 域。

下面的安装前缀是占位符，替换为上述工作区构建的真实输出路径。在仓库根目录构建发送器，发送器构建目录和试验输出放在仓库外：

```bash
dm_install_prefix="/absolute/path/to/this-build/install"
source /opt/ros/humble/setup.bash
source "$dm_install_prefix/setup.bash"
rig_build="$HOME/.ros/dm_motor_bilateral_control/rig-stream-build"
cmake -S tools/physical_rig/bilateral/native_stream -B "$rig_build" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$rig_build" --parallel 2
"$rig_build/rig_stream" --build-info
```

CMake 需要当前环境中的 `ament_cmake`、`rclcpp` 和已生成的 `dm_motor_interfaces`。`--build-info` 在 ROS 初始化之前返回源文件/CMake 哈希、编译器和构建类型。主程序要求显式 `--stream-binary`，执行前核对这些嵌入哈希与本目录源文件一致；旧实验二进制没有该接口，不能直接代用。还会记录可执行文件 SHA256，并在真正启动发送器前重新检查文件未变。

这些记录证明所选文件及构建输入身份，不代替对编译器、ROS 依赖和已运行电机驱动的版本核验。不要在试验过程中重新构建或替换正在使用的文件。

## 准备与计划

真实 CAN、三台驱动和差模双边控制器必须已经准备、全部停止。可使用[显式三轴物理配置](../../../config/bilateral_physical_rig.md)按其完整冷启动流程准备；已有实例时不要叠加启动。

本入口要求差模模式、控制器 500 Hz、每次 START 核验 A/B V_BW=100、真实新鲜反馈、相同原生映射，且 C 的主机保护和 watchdog 覆盖本轮边界。

`--driver-bandwidth` 默认100 Hz，必须为可编码的正有限 float32 数且小于500。使能前会在三轴停止时只读C寄存器32（V_BW）：先取得唯一 operation ID，再通过 `OPERATION_RESULT` 轮询，只有 `CONFIRMED`、地址32且解码值与要求一致才通过。排队回复和PENDING不算读回确认。当前驱动不在结果中回填operation_id，结果关联依赖TopicEndpoint对本次client/request的匹配；请求的operation ID及原始回复一并记录。该步骤不写寄存器，值不符时拒绝ENABLE。

当前[显式物理profile](../../../config/bilateral_physical_rig.md)将C启动RAM速度带宽设为40 Hz，A/B为100 Hz，并已通过统一冷启动读回核验；驱动通用默认仍是0（不修改RAM）。本工具的`--driver-bandwidth 40`只要求读回40，不负责把设备从100改成40。使用该profile时须显式传40；使用其他配置时仍需核对实际带宽。改变C带宽后要重新评价运动记录。

非`passive`轮次在START前、控制器停止且两轴新鲜失能时，如需改变配置，通过一次原子参数请求设置`coupling_mode`与`friction_compensation_enabled`，读回保存为`effective_configuration`。结束后三轴STOP确认，再原子恢复改变过的参数并核对完整配置读回；恢复失败时本轮不能成功。`configuration`保留轮前原值，不能用它代替本轮effective值。实际控制器 `config_file`、`system_parameters_file` 及其内容、哈希由运行参数读取，不能把仓库默认文件的哈希冒充当前加载文件。

快照同时记录 `extra_viscous_a/b` 和 `extra_viscous_velocity_limit`；工具不会设置这些候选增益。
评分器还核对 ACTIVE 记录中唯一的 `controller_source`，自动配对只接受同一个控制器进程。
即使刚度、阻尼相同，重启部署后的新算法也不会与旧实例混为同条件实验；跨重启比较需单独审阅实现版本。

下面只是 0.6 rad/s 的计划示例，不保证任意起点下都能执行。输出的父目录要存在，最终输出目录必须不存在：

```bash
rig_output_parent="$HOME/.ros/dm_motor_bilateral_control/physical-rig"
mkdir -p "$rig_output_parent"
python3 tools/physical_rig/bilateral/run_trial.py \
  --stream-binary "$rig_build/rig_stream" \
  --condition bilateral_no_friction --driver-bandwidth 100 \
  --driver-kp 20 --driver-kd 0.6 \
  --speed 0.6 --speed-guard 1.6 --torque-guard 1.2 --travel-guard 0.8 \
  --output "$rig_output_parent/trial-06-off-001"
```

计划模式不导入 ROS、不运行发送器、不创建输出目录。它验证数值和几何边界；在线起点、设备身份、反馈、保护和实际带宽由执行阶段进一步检查。正式执行时，在同一条已核对的命令后增加 `--execute`。切换为 `bilateral_friction` 可完成相同轨迹的有补偿对照；每轮使用新的输出目录，并先检查上一轮三轴停止确认和参数恢复结果。

`--driver-kp`默认20，范围为`0 < kp ≤30`；`--driver-kd`默认0.6，范围为`0 < kd ≤1`。Python入口和C++发送器都检查有限正数及上界。这两项仅改变台架驱动轴C的原生MIT增益，A/B双边刚度、阻尼、补偿及轨迹时程保持不变；例如C40 Hz、P10/D0.6是台架条件对照，不构成A/B减阻优化的结论。命令JSONL、发送器结果和主结果保存所选值，实际CAN量化结果仍由原始记录核对。

CLI 边界为正有限数：`speed < speed-guard ≤18` rad/s，力矩边界 ≤5 Nm，行程边界 ≤25 rad；C++ 轨迹速度最多 13.3 rad/s。实际驱动配置可以更严格，例如物理 profile 的 C 速度门限为16 rad/s、力矩门限4 Nm；工具不能通过提高 CLI 绕过驱动限制。温度48°C、反馈80 ms、机械配对20 ms、A/C配对偏差0.08 rad、A/B相对误差0.15 rad仍独立检查。

<a id="coupling-comparison"></a>
## 单向/双向低阻力对比

控制器`coupling_mode`默认`bilateral`；`unilateral_friction`选择`a_leads_b`，只支持差模且`max_torque=0`，仅fresh disabled状态可切换。
A取消弹簧和来自B的速度反作用，但原生D、自身采集时间速度滤波及两轴补偿保留；B算法不变。
公式、切换方法及限制见[控制器说明](../../../src/dm_motor_bilateral_control/README.md#unilateral-coupling)。这项对照保持C与A机械连接，不等于只有A使能。

固定比较计划为0.6、3、9、13.2 rad/s四档，每档按 **U → D → D → U** 运行4个独立`sweep`轮次，共16轮；U=`unilateral_friction`，D=`bilateral_friction`。
每轮分别完成START、轨迹、三轴STOP和参数恢复，并使用新目录。同一档的轨迹、保护、C带宽/P/D、A/B补偿参数及控制器实例保持一致。
以下是低速U轮的计划示例；D轮仅替换condition和输出目录。各档边界须先按现场条件核对，失败不自动重试或用残缺记录凑足轮数。

```bash
python3 tools/physical_rig/bilateral/run_trial.py \
  --stream-binary "$rig_build/rig_stream" --protocol sweep \
  --condition unilateral_friction --driver-bandwidth 40 \
  --driver-kp 10 --driver-kd 0.6 \
  --speed 0.6 --speed-guard 1.6 --torque-guard 1.2 --travel-guard 0.8 \
  --output "$rig_output_parent/cmp-060-01-u"
```

记录齐备后使用离线比较器；例如目录统一命名为`cmp-060-01-u`、`cmp-060-02-d`等：

```bash
python3 tools/physical_rig/bilateral/compare_coupling.py \
  --root "$rig_output_parent" --glob 'cmp-*' \
  --output "$rig_output_parent/coupling-comparison.json" \
  --markdown-output "$rig_output_parent/coupling-comparison.md"
```

比较器不运行电机，复用原短轮固定窗口和成功门，核对effective模式、ACTIVE消息、来源及输入身份；失败轮和不同控制器实例保持分组或排除。
先对每轮正反向等权，再对独立轮次等权；反馈帧、两方向和交叉配对不是额外独立实验。输出保留U/D轮数及是否各2轮，不能因生成报告就声称16轮全部完成。
C报告力矩是整个台架的驱动负担代理，报告同时给出实际速度、起点、温度、跟随误差和纹波，不直接等同人手阻力。

新增单向模式的健康13.2 rad/s与最坏批次轻触仿真通过；拆C最坏释放速度增长10.173%，未通过10%门，不能宣称释放全覆盖。
原V16验收及B100四场结果属于双向模式。2026-11-06的16轮真机U/D矩阵已全部完成，结果见[三轴对比报告](../../../docs/experiment-history.md#2026-11-06-bilateral-unilateral-comparison)；[仿真证据与范围](../../../src/dm_motor_bilateral_control/README.md#unilateral-coupling)单独保留，不能将位置伺服拖动的成功外推为自由释放通过。

## 轨迹、时序与停机

默认 `--protocol sweep`：0.3 s 静止基线 → 正向余弦加速/恒速/减速 → 0.25 s 停顿 → 反向返回 → 0.5 s 停顿。加减速时长 `r=max(0.25,speed/15)`，单向恒速0.6 s，单向行程 `speed*(r+0.6)`，总运动时长 `2*(2*r+0.6)+1.05`。预先固定的评分窗口由计划结果给出，不能把提前停止后的静止数据计作成功恒速段。

`--protocol release`（协议版本2）：0.3 s 基线 → 同样的余弦加速 `r=max(0.25,speed/15)` → 0.6 s 恒速 → C 持续发送 `q=v=kp=kd=torque=0`，A/B 保持双边控制 → 停止三轴。计划中性阶段5.05 s，其中50 ms是传递余量；评分仍要求从**第一帧实际 C 零 PD CAN 指令**起完整观察5 s。驱动段行程为 `speed*(r/2+0.6)`。例如3 rad/s在1.15 s释放、驱动2.175 rad，总长6.20 s；13.2 rad/s在1.78 s释放、驱动13.728 rad，总长6.83 s。释放协议拒绝 `passive` 条件。版本1的0.2 s平台记录仍按原窗口和门限审核；评分不把两个版本自动混配。延长平台是为加速瞬态留出稳定时间，并不保证每次都能达速，仍须实测通过。

释放时 C 继续使能、保持同一来源和守卫链，并与 A 机械连接；C 的摩擦、惯量及内部损耗仍参与运动。MIT 零扭矩量化约为 −0.002442 Nm，也不是理想零力矩。因此这项测试只评价当前台架的释放行为，不能替代拆除 C 后的自由释放验证，也不能独立辨识 A 的摩擦。中性位置始终编码为0，物理反馈可使用连续多圈位置越过原生 ±12.5 rad；有 P 的驱动段仍受下面的位置参考边界限制。

C 的整个原生位置参考必须落在 ±12.4 rad 内，为 PMAX=12.5 留余量。主机 ±350 rad 范围不扩展 MIT 命令范围。工具会拒绝越界轨迹，不会自动停车或写设备零点；应先用已验证的单独流程完成必要移位。实测速度是否达到目标由位置/反馈记录判定，不能仅凭目标速度或进程成功作结论。

通信发现包含请求端匹配与结果主题图发现检查（等待上限8 s）和2 s稳定等待，实际能力回包才确认通信。只有只读能力请求可在超时后最多尝试5次；ENABLE、超时或结果不明的START均不重试。START准备另有总计2 s预算，要求A/B状态与反馈四路速度连续100 ms低于原0.05 rad/s门。只有控制器明确回复`Hold both disabled motors stationary before starting`时，才能在确认拒绝之后的新鲜失能、无命令来源、控制器IDLE数据后再次请求，最多3次；全部请求及拒绝进入events记录。这不重跑已开始的运动，也不放宽原启动阈值，实现见[start_preparation.py](start_preparation.py)。

C++ 发送间隔超过10 ms会失败。守卫要求父进程心跳，200 ms无心跳、管道EOF、时限、故障或运动边界触发时停止三轴。控制、监护和全量 JSON 记录各在独立进程中，减少记录工作对轨迹时序的影响。

发送器使用持久 `SingleThreadedExecutor`；命令日志在 READY 前分配缓冲区，结束时检查写出结果。原生 CLI 保持七个位置参数兼容，可追加 `--protocol sweep|release|long_variable_pilot|long_variable`、`--log-buffer-bytes BYTES`，两者顺序不限；日志缓冲区默认4 MiB，允许2–64 MiB。短协议日志可装入最小缓冲区。下述长协议强制显式使用64 MiB缓冲；V16已完成一次独立的302.04 s连续真机验收，证据见本节末尾，不能把短轮相加当作连续运行。

主程序结束时先请求双边STOP和三轴DISABLE，再结束发送器，随后等待新的失能反馈；独立守卫还检查最后0.2 s位置范围小于0.003 rad。只有停止确认后才恢复本轮改变的耦合模式与补偿开关，并检查读回一致。`Ctrl+C` 进入相同清理流程。失败记录和输出目录保留，不自动清错、删除、复用或重试运动。

`success:true` 表示本轮执行、停止、记录与参数恢复成功；它不自动证明拖动阻力最小、标定准确、没有任何丢帧或已经达到4π rad/s。C的报告力矩同时包含C自身摩擦/惯量及联轴器效应，只能作为同条件驱动用力的比较量，不能直接当作A的独立摩擦真值。

## 输出与验证

默认短协议输出包括三轴 `feedback/state`、各轴 `/target` 话题、双边状态、CAN RX/TX、总线状态、发送器目标、监护事件、能力快照、守卫停止证据、运行参数恢复结果，以及每个辅助进程的 PID/启动身份。当前 A/B 双边目标实际发布到 `/setpoint`，记录器的 `/target` 订阅不能声称覆盖这些目标；应结合状态中的 active_target 和实际 CAN 指令审核。C `/target` 和原生发送器命令完整记录。原始接收时间戳和反馈计数保留；最终分析仍须检查缺号、间隔、观察队列丢弃和完整固定窗口。

`sources/` 保存本轮工具输入，`provenance.json` 保存源码及二进制哈希和嵌入构建信息；`config_file.yaml`、`system_parameters_file.yaml` 是实际控制器路径读取的输入快照。既有输出目录一律拒绝，记录文件和结果使用新建模式。

只运行离线检查：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s tools/physical_rig/bilateral -p test_offline.py -v
```

这些检查覆盖默认计划、释放几何与条件限制、不覆盖既有输出、非法运动/带宽/增益边界、带宽PENDING/错误地址/拒绝结果不能当确认、错误构建身份、守卫身份/持续时间/保护边界（包括 `python -O`）、记录器拒绝已有数据。子进程使用 `-S`，不加载安装的 ROS 包；测试不运行ROS状态机或真机。发送器的假节点测试与真实运动验收需分别记录。

释放评分使用保存的文件，无需 ROS：

```bash
python3 tools/physical_rig/bilateral/score_release.py \
  --root "$rig_output_parent" --output "$rig_output_parent/release-scores.json" \
  release-3-on-001 release-132-on-001
```

正式评分要求执行、三轴停止、守卫、记录和参数恢复全部成功；来源、配置/源码/二进制哈希一致；C 真实指令在释放后持续零 PD/扭矩；A/B 持续 ACTIVE/NORMAL；反馈计数、原始 RX 时间和完整观察窗口有效。失败轮不借助 STOP 后帧补齐。不同协议、C 带宽/增益、控制器实例、注册表或有效设置分别保留，不自动混配。

预释放120 ms位置 OLS 速度须在目标±5%内；目标至少4π rad/s时，三轴实测也必须各达到4π。释放后以100 ms位置 OLS包络检查增长（容差为预释放速度的10%与0.2 rad/s两者较大值）；最后0.5 s须同时满足位置 OLS≤0.02 rad/s、位置范围≤0.01 rad、上报速度绝对值p95≤0.05且最大值≤0.1 rad/s。还报告持续停稳的确认时刻、滑行距离、位置范围、速度和扭矩峰值；这些预先固定标准不会按单轮结果改动。


## 连续变速试跑与正式长测

`long_variable_pilot`固定一组六档，运动17.72 s、总长18.52 s；`long_variable`固定17组，运动301.24 s、总长302.04 s。没有任意groups参数。两个协议均只执行一次START/ENABLE和GO，中途无静止等待、停止重启、设零或Flash；每档的余弦正反腿直接相接，只在反转瞬间经过零速。只有外部0.3 s启动段和0.5 s结束段，这0.8 s不计入运动时间。试跑通过不构成正式长测验收。

速度顺序固定为0.6、1.5、3、6、9、13.2 rad/s，每档正反各一次。每个0.6 s恒速段在第0.05 s把速度目标减小`min(0.6,0.5×本档速度)`，第0.15 s恢复；低档阶跃0.3 rad/s，其他档0.6 rad/s。位置由速度解析积分连续产生，正反位移抵消；没有位置目标跳变，也不要求真实电机速度物理上瞬跳。正式长测有408个目标速度阶跃，最大行程19.476 rad，C起点须严格落在`(-12.4,-7.076)`内，另保留数值舍入余量。工具不会自动停车到该起点。

以下仅生成单组试跑计划；实际C带宽/增益、三轴位置和所选控制器必须与现场部署相符，输出目录不能已存在：

```bash
python3 tools/physical_rig/bilateral/run_trial.py   --protocol long_variable_pilot --stream-binary "$rig_build/rig_stream"   --condition bilateral_friction --driver-bandwidth 40   --driver-kp 10 --driver-kd 0.6   --speed 13.2 --speed-guard 15.9 --torque-guard 3.8 --travel-guard 20.1   --output "$rig_output_parent/variable-pilot-001"
```

先完成所选安装产物的短时/释放验证，再用新输出目录执行单组试跑；完整记录合格后才切换为`--protocol long_variable`执行正式长测。正式运动需要显式`--execute`，本工具不自动从试跑升级到长测。`--speed 13.2`表示固定协议的最高档，不能用较低数值缩短正式验收。两种协议均拒绝passive条件，不修改A/B刚度、阻尼或助力参数。

长协议保留500 Hz、10 ms发送间隔门槛及全部物理保护。17组协议仅把独立守护整轮期限设为316.04 s、记录器有限期限设为342.04 s；单组仍使用短期限。心跳、反馈新鲜度、速度、力矩、行程、温度和独立STOP条件不放宽。任意保护触发即本轮失败，不能自动重启后补时长。READY前预分配64 MiB命令日志缓冲，最多160000行；结束关闭并验证文件后才发送DONE。执行前要求至少4 GiB可用磁盘空间，整轮原始记录预计约1.4 GiB。

这两种协议额外记录A/B的`/setpoint`（匹配原best-effort发布），C的`/target`和CAN观察帧使用原publisher兼容的reliable订阅。CAN同时保存原始`header.stamp`和`received_steady`；前者用于总线计数边界，后者用于运动反馈对齐。BilateralState和Bus附带ROS时钟到steady时钟的观测区间与转换不确定度。MotorState评分时刻来自`feedback_receive_time + feedback_age`；记录回调延迟单独报告，不把回调时间当作源状态时刻。

只读评分命令：

```bash
python3 tools/physical_rig/bilateral/score_long.py   "$rig_output_parent/variable-pilot-001"   --output "$rig_output_parent/variable-pilot-001-score.json"
```

评分以流式扫描核对全程ACTIVE/NORMAL、三轴使能和同一来源、连续反馈计数、实际CAN帧与源命令、408次阶跃（试跑24次）、总线RX/TX计数和零observer丢弃。源时间边界相同的歧义、时钟回退、缺失/截断文件、源码/参数或执行评分器与采前快照不一致均拒绝通过。C原生Kd按协议物理范围0..5编码核验，应用层Kd≤1不改变CAN量程。

每次巡航的预定0.25..0.55 s窗口都用三轴实测连续位置斜率评分；每个13.2档窗口均须实际达到4π rad/s，并满足预设跟随误差，不能用命令速度或跨轮平均代替。step响应只统计窗口内原始速度/力矩，固定边界需被源数据双侧包围、相邻receive间隔≤20 ms；批次接收不会因回调延迟被误判成控制变化，不外推或合成速度样本。停止后的记录只证明fresh STOP，不能补齐运动窗口。

结果中的`pilot_passed`与`final_acceptance`互斥：单组永远不能令`final_acceptance`为true；正式17组只有全部运动、记录完整性、三轴停止与恢复检查通过才取得正式验收。

2026-11-05的V16正式长测已取得`passed=true`、`final_acceptance=true`：B100（A附加项0、B附加粘性系数0.02584737521699079、原摩擦比例0.6）、C40 Hz/P10/D0.6，统一冷启动与标定历史恢复后先通过单组试跑，再连续完成302.04 s，其中运动301.24 s、17组、408次速度目标阶跃。全部高速固定窗口三轴实测均达到4π rad/s，三轴fresh STOP及记录完整性通过；发送器最大间隔5.795 ms，原10 ms保护未改。

独立原始评分为现场证据目录的`analysis/V16_COLD_B100_FULL_01.json`，说明见[三电机阶段报告](../../../docs/experiment-history.md#three-motor-bilateral-baseline-2026-11-05)。离线数学、跨语言轨迹、合成反例、fake、短轮/释放与正式长测分别保留身份和结果。该台架通过不代表任意负载或所有软件回归通过；软件与交付的证据作用域见[阶段报告](../../../docs/experiment-history.md#three-motor-bilateral-baseline-2026-11-05)。

所选B100的离线补充为3个新短场景加保留的释放，合计覆盖固定4场，并单次完成302.04 s序列。该离线序列的原门不含逐窗实际达速，102项高速轴窗口的位置斜率另作只读诊断；这里的真机长测评分仍执行前述逐窗达速门。B80逐字节一致性证明与B100释放0.1238个百分点窄余量保持各自范围，见[仿真补充证据](../../../docs/experiment-history.md#three-motor-bilateral-baseline-2026-11-05)。
