# 电机物理仿真

集成在 `dm_motor_control` 中、独立进程运行的时间积分关节电机仿真。
统一入口默认真机，必须显式传入 `simulation:=true`。仿真使用 `can_bus backend:=virtual` 的 Topic 虚拟线缆，
并运行与真机相同的 `dm_motor_control/motor_node`，不另写一套电机控制 API。
**不会连接串口/SocketCAN，不自动使能，不自动施加外力。**

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# 两台物理仿真电机、虚拟总线和配置的面板
ros2 launch dm_motor_control bus.launch.py simulation:=true

# 或：完整主从仿真场景（不要与上一条同时运行）
ros2 launch dm_motor_bringup lead_follow.launch.py simulation:=true

# 或：独立单电机
ros2 launch dm_motor_control motor.launch.py simulation:=true motor_id:=1

# 虚拟电机已运行时，只附加含外力操作的面板
ros2 launch dm_motor_control dashboard.launch.py simulation:=true motor_id:=1
```

最后一条仅在该端口没有其它面板时使用，不要重复启动。
所有 launch 可用 `config_file:=/绝对路径/site.yaml` 选择配置，不传则读取默认配置。
省略端口参数会使用该文件中的端口；显式空值关闭对应面板，例如
`can_dashboard_ports:='{}' motor_dashboard_ports:='{}' dashboard_port:="''"`。
组场景默认使用 8080/8081/8082/8083 的显式映射，
支持任意不重复端口和 `namespace`。单电机默认总线为 `/motor_1/can`；
组场景为 `/can/main`。同时运行不同场景应使用隔离 ROS domain/命名空间和端口。
单电机入口可通过 `control_rate` 和 `can_fd` 覆盖驱动频率/虚拟 FD 模式；
其它驱动和物理参数使用所选 common 配置中的节点覆盖。

物理默认值位于 [config/dm_motors.yaml](../../config/dm_motors.yaml) 的
`packages.dm_motor_control.nodes.sim_motor_node`。各 launch 保持独立 profile，
`variants.simulation` 覆盖仿真所需的 `parameters`、`system` 和 `nodes`；
其中 `devices.<motor_name>` 可覆盖某个虚拟设备的物理参数。
例如主从仿真读取 `packages.dm_motor_bringup.launches["lead_follow.launch.py"].variants.simulation`。
变体中的 `nodes.dm_motor_control.motor_node` 可以调整复用驱动的频率/保护。
仿真模式只接受 backend 为 `virtual` 且硬件 `port` 为空的总线；显式冲突参数会报错，
不会回退到真机。该开关不是 MIT 等驱动 `mode`，也不是 `use_sim_time`，不支持运行中切换。
驱动还设置 `require_virtual_bus=true`，实际发现非虚拟后端时拒绝注册和设备命令。

### 仿真默认不启用软件运动限制

电机/系统仿真入口均显式关闭主机运动保护，撤销额外的位置、速度、力矩、
测量/估算阈值、温度停机阈值和软件行程边界，避免继承基础真机配置的可选软限制。
主从仿真默认 `motion_limits_enabled=false`：不按跟随误差、行程、速度、力矩或温度阈值停机，
也不对跟随目标做 `target_speed` 限速；误差仍正常计算和显示。
需要验收保护功能时，可在仿真变体（包括逐轴设置）或显式 `config` 覆盖文件中重新启用；
主从运动保护开关也可在待机状态显式开启。基础真机配置不受影响。

“不启用软件运动限制”不等于伪造成功：CAN 编码范围、真实物理模型的电磁饱和/关节限位、
设备故障、失联/超时、有限数值校验以及显式启停仍然保留。
例如 MIT 目标仍须能被原协议编码；不能保证任意无限大输入或断连状态都继续跟随。
修改配置后应重启仿真场景并刷新主从页面，避免旧页面的已编辑表单再次提交旧保护设置。

## 相同接口，不伪装成真实硬件

`/motor_1/state`、`feedback`、`command`、`setpoint`、`target`、`control`、
`control_result`、`task_events`、`joint_states`、`diagnostics` 与真机相同，
消息类型、参数查询、模式/零点/清错/任务/保活处理都来自同一生产驱动。
CLI、主从控制器和通用面板不需要更换控制逻辑。

`MotorState.simulated=true` 和 CAN `backend=virtual` 保留真实来源信息，不谎报为硬件。
这两个诊断字段是有意区别；不能据此降低目标有效期、故障或驱动保护检查。

## 物理模型与边界

模型按单调墙钟推进，不以 CAN 控制包或查询包数量推进。默认积分步长 0.5 ms，
物理计时器 1000 Hz；控制命令之间继续运动，失能后也会受外力、重力和摩擦作用。
0.5 ms 是子步上限，命令到达边界可能使用更短子步；没有把定时器迟到简单变成漏算物理时间。
超过两秒的仿真进程停顿会明确报错并锁存故障，不冒充已经正确积分了未知期间的运动。

```text
J * acceleration = motor_torque + external_torque + gravity_torque
                   + friction_torque + contact_torque
```

包含转动惯量、粘性/库仑/静摩擦、正弦重力负载、弹簧阻尼关节限位、电磁力矩响应时间常数、
转速相关力矩饱和、I²R 发热与散热。四种控制模式均通过力矩驱动模型，不瞬间改位置或速度。
零位改变编码器偏置，不改变物理角度；CAN 反馈使用原协议的紧凑量化和回绕。
TIMEOUT 由虚拟固件独立计时，丢失命令会锁存通信故障，即使主机控制节点已暂停。

默认物理参数是可重复测试的示例，不是 J4310 工厂标定。需要真机测量惯量、摩擦、
负载、带宽和热参数后才能主张具体工况下的近似精度。这里不模拟电源瞬态波形、
电磁开关纹波、齿隙/柔性多体机构或全部固件私有功能，不能认证真实硬件急停或安全性能。
纯软件测试能证明接口和定义的模型行为一致，不能证明任意实物有“完全相同”的动力学。

| 参数 | 单位 / 含义 |
| --- | --- |
| `inertia` | kg·m²，输出轴等效转动惯量（含负载），默认 0.01 |
| `viscous_friction` | Nm/(rad/s)，默认 0.02 |
| `coulomb_friction` / `static_friction` | Nm，默认 0.03 / 0.05 |
| `gravity_torque` / `gravity_offset` | 重力力矩幅值（例如 m·g·r）/ 安装相位，默认 0 / 0 |
| `max_torque` / `no_load_speed` | Nm / rad/s，执行器峰值力矩及空载速度，默认 10 / 50 |
| `torque_time_constant` | s，电磁力矩一阶响应，默认 0.002 |
| `lower_limit` / `upper_limit` | rad，机械角度限位，默认 ±1e6（多圈测试，不代表真机行程） |
| `contact_stiffness` / `contact_damping` | Nm/rad / Nm/(rad/s)，限位接触模型 |
| `heat_capacity` / `thermal_resistance` | J/°C / °C/W，绕组热容和散热热阻 |
| `winding_resistance` / `torque_constant` | Ω / Nm/A，热模型电阻和输出轴力矩常数 |
| `initial_position` / `initial_velocity` | rad / rad/s，场景初值；只能重启配置，不通过外力 API 瞬移 |
| `anchor_stiffness` | Nm/rad，持久弹簧锚点刚度，默认 0.5；与 ref 的零预载弹簧相同 |
| `mapping_position/velocity/torque` | CAN 量化映射，默认 12.5 rad / 50 rad/s / 10 Nm，不等于机械限位 |

位置/速度模式采用可观测的级联位置/速度控制近似，`KP_ASR/KI_ASR/KP_APR/KI_APR`
读写会作用于对应模型增益。其它公开 RW 参数可按协议保存/回读，
但私有观测器、电流环参数的精确固件算法未复刻，不能仅凭回读成功声称其所有物理影响已认证。
型号、固件或机械参数变更后应重新标定，不能为了让测试通过而提高生产驱动的保护上限。

## 外力接口（仅仿真）

每个电机新增：

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `simulation/state` | `dm_motor_interfaces/msg/SimulationState` | 物理角度、力矩分量、锚点/版本、温度、能量及积分计数 |
| `simulation/external_torque` | `dm_motor_interfaces/msg/ExternalTorque` | 有时效的原生输出轴外力矩，Nm；可选弹性手柄 |
| `simulation/control` | `dm_motor_interfaces/srv/SimulationControl` | 故障源、回包暂停、虚拟掉电重启、电压、锚点及撤销外力 |

外力正方向是原生轴角度增加方向；若驱动 `direction=-1`，逻辑反馈方向相反。
临时外力使用最后一个有效输入，`valid_for` 为 0–5 秒（不含 0），到期自动撤销；
必须提供当前 ROS 时间戳，同一发布者的时间戳及非零序号递增。
电机无需使能就能被外力推动，这与手动转动失能电机一致。

以下程序假设虚拟系统已启动，通过 Topic 持续施力 0.5 秒后撤销，不请求电机使能：

```python
import time
import rclpy
from rclpy.node import Node
from dm_motor_interfaces.msg import ExternalTorque, SimulationState

rclpy.init()
node = Node("external_force_example")
states = []
sub = node.create_subscription(
    SimulationState, "/motor_1/simulation/state", states.append, 1)
pub = node.create_publisher(ExternalTorque, "/motor_1/simulation/external_torque", 1)
try:
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if pub.get_subscription_count() and states and states[-1].virtual_bus_confirmed:
            break
    else:
        raise RuntimeError("Virtual motor is not ready")
    for sequence in range(1, 12):
        command = ExternalTorque(
            source="automation_example", sequence=sequence,
            torque=0.08 if sequence <= 10 else 0.0, valid_for=0.2)
        command.header.stamp = node.get_clock().now().to_msg()
        pub.publish(command)
        until = time.monotonic() + 0.05
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
finally:
    node.destroy_node()
    rclpy.shutdown()
```

程序进程被中断时，最后一条临时外力也会在其有效期到期后撤销，独立锚点不受影响。订阅普通 `MotorState` 和
`SimulationState` 记录响应，用于 AI 自动测试的断言，不需要操作浏览器。

## 统一面板的轴圆盘

目标滚轮与“电机轴 / 实测反馈”圆盘放大并排显示，目标位置输入框位于两个圆盘下方；
空间不足时圆盘纵向排列，输入框仍在它们下方。

- 蓝色轴标记显示实际关节反馈，不跟随鼠标伪造位置；真机仅显示，不能施力。
- 仿真可拖动圆周或按住左右方向键，没有累计圈数上限。鼠标接触点与拖动目标点分开显示，
  箭头表示当前外力方向。目标位置滚轮仍用于驱动命令，两者用途不同。
- 拖拽不改控制目标、不请求使能；用有时效的弹性手柄向模型施力，失能时也可转动。
- 松手、指针取消/失去捕获、Esc、停止按钮、窗口失焦或离开页面会释放当前拖拽。
  不操作圆盘的观察页面不会撤销程序施力。刷新/重连不会恢复旧手势。
- 手柄每 50 ms 刷新，有效期 300 ms；外力弹簧在物理子步计算：
  `tau = K * (hand_position - physical_position) - D * velocity`，不裁剪力矩。
  默认 K=1 Nm/rad、D=0.05 Nm/(rad/s)，可在
  `packages.dm_motor_control.nodes.dashboard.shaft_*` 或对应 launch 的节点覆盖中配置。
  手势方向与关节坐标一致，发送给物理模型前考虑驱动 `direction` 和设备零偏。

### 双击设置持久锚点

双击圆盘创建一个紫色锚点，或在圆盘聚焦后按回车。每台虚拟电机只有一个锚点；
要更换位置，先双击现有紫点取消，再创建新的锚点。
单击/双击不会发送临时手力或改变程序正在施加的外力，只有实际拖动或方向键操作才启动手力刷新。

锚点记录创建时的原生物理位置 `q0`，没有预加载、不瞬移、不自动使能：

```text
anchor_torque = anchor_stiffness * (q0 - physical_position)
external_torque = transient_torque + anchor_torque
```

拖开转子后松手，临时手力撤销，但锚点仍继续施加恢复力；锚点不是刚性锁轴，
电机可以在驱动力矩、摩擦、惯量等作用下偏离或振荡。
紫点固定，虚线连接随转子转动的接触点；显示坐标考虑驱动方向及软件/设备零位。
轴圆盘的红箭头表示临时手力，锚点力矩单独显示；两者均不设置力矩限幅。

锚点保存在独立虚拟设备进程中。松手、Esc、失能、停止主从或刷新/关闭网页只结束相应控制/手力，
不会删除锚点；虚拟固件 `POWER_CYCLE` 也不删除环境锚点。
双击紫点、调用 `CLEAR_ANCHOR` 或重启整个仿真设备进程才能移除。
数值积分异常会撤销所有外部负载并锁存故障，不持续积分无效状态。

程序可调用 `SimulationControl.SET_ANCHOR`（6），`value` 为原生坐标下的锚点图示方位，
物理平衡点由设备在调用时捕获；`CLEAR_ANCHOR`（7）仅移除锚点。
`CLEAR_EXTERNAL_TORQUE`（5）清除临时外力及锚点，而发布普通零外力消息只清除临时输入。
状态新增 `anchor_active/anchor_position/anchor_bearing/anchor_stiffness/anchor_torque/anchor_revision`
及 `transient_torque`，`external_torque` 表示二者合力矩。

HTTP `/api/shaft-anchor` 接收 `active`、原生 `bearing`、观察到的 `revision` 和 `simulation_time`。
创建和删除都在设备端校验版本，迟到的创建不能覆盖已经确认的删除。
ROS 调用可设置 `check_anchor_revision=true` 与 `anchor_revision` 使用相同的原子校验；
冲突或未知结果会明确报错，不自动重试。所有锚点操作仅允许确认的虚拟设备，真机仍只读。

直接外力和弹性手柄均无幅值上限。原有 `external_torque_limit`、`shaft_torque_limit`、
`shaft_drag_limit` 配置及外力消息的 `torque_limit` 字段已移除，旧配置/脚本应同步更新。
输入仍必须为有限数值；极端数值导致积分溢出时明确报错、撤销该外力并锁存仿真故障，
保留最后有效物理状态，不用限幅或伪造反馈掩盖失败。故障后仍需显式清错。
这不改变电机自身的电磁力矩饱和、协议范围、关节限位或主机保护。
力箭头只为便于显示限制画面长度，旁边数值和实际施力不截断。

没有独立外力页签、滑条或故障按钮；高级测试继续使用上述 ROS 接口。
HTTP 保留 `/api/simulation`、`/api/external-torque` 和 `/api/simulation-control`。
圆盘专用 `/api/shaft-drag` 接收原生物理 `position` 及最近观察到的 `simulation_time`；
过期反馈和乱序请求会拒绝，迟到响应也不会恢复已释放的手势。
所有施力 API 都要求面板启用仿真、虚拟设备确认和真实的 `simulated` 反馈，
真机模式不会创建施力发布者。故障注入后须先移除注入源，再通过普通电机接口显式清错/复位。

程序也可发布 `ExternalTorque(spring=True, position=..., stiffness=..., damping=...,
valid_for=...)`，此时 `torque` 必须为 0；省略 `spring` 则保持普通力矩输入。
这不是驱动位置目标，正常电机控制 API 不受影响。

`POWER_CYCLE` 只在虚拟设备失能时执行；保留机械位置但恢复已保存寄存器/零偏，
测试端应像真实掉电一样重启驱动重新初始化，不假定旧映射和绝对位置仍有效。
默认 Flash 镜像仅在当前进程内保存；设置每台独立的 `persistence_file` 后，
显式 SAVE 才原子写入镜像，重新启动设备时加载。不会自动保存或覆盖生产配置。

## 自动化

```bash
colcon test --packages-select common dm_motor_control dm_motor_bringup --return-code-on-test-failure
colcon test-result --test-result-base build/dm_motor_control --verbose
colcon test-result --test-result-base build/dm_motor_bringup --verbose

# 全工作空间回归
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
```

仿真测试使用隔离 ROS domain、命名空间和临时端口。物理单元测试与解析解比较，
并检查控制/查询频率不改变积分速度；集成测试使用生产驱动、真实 ROS Topic 和 HTTP，
而不是用固定成功应答替代控制流程。具体用例和覆盖结果见包内 `test/`。

### 可选的真实浏览器回归

[browser_regression.py](../dm_motor_bringup/test/browser_regression.py) 自动启动隔离的纯虚拟主从场景和临时面板，
用 Chromium 点击实际页面，并根据生产驱动/物理模型反馈断言结果，结束后停止自己创建的节点。
不访问已经运行的面板或真实 CAN。需要先加载 ROS 和工作区环境。
Playwright 仅是该测试的可选工具，不是运行包或默认 colcon 测试的依赖：

```bash
python3 -m venv --system-site-packages /tmp/dm-browser-venv
/tmp/dm-browser-venv/bin/python -m pip install playwright
/tmp/dm-browser-venv/bin/python -m playwright install chromium
/tmp/dm-browser-venv/bin/python src/dm_motor_bringup/test/browser_regression.py \
  --legacy-ui --output /tmp/dm-motor-browser-results
```

`--legacy-ui` 额外执行原有电机/CAN 静态浏览器回归；省略时只验收真实虚拟设备上的交互。
`--output` 保存 JSON 结果和面板截图。布局断言依赖字体度量，建议提供页面声明的 Inter
及中文字体；缺失时应检查系统回退字体，而不是放宽原有布局断言。
覆盖只监控不发命令、失能圆盘拖拽、反向拖拽/指针取消、真机只读、迟到请求和成功响应不恢复施力、
离开页面撤销本次施力、纯观察者不撤销程序外力、通用位置控制、主从外力跟随和桌面/移动布局。

| 覆盖 | 验证方式 |
| --- | --- |
| 物理积分 | 恒定外力的解析位置/速度、自由滑行、粘性衰减、静摩擦、重力方向、弹簧限位及发热 |
| 与报文频率解耦 | 相同力矩下 100 / 1000 Hz 查询轨迹比较；积分时间不跟随帧数倍增 |
| 通用控制 | 四种模式通过原有 Topic 使能/目标/停止，非独占输入可替换内部任务 |
| 有界任务 | MOVE/HOLD/RAMP/SINE/TRAJECTORY/VELOCITY/TORQUE 全部正常结束并确认失能 |
| 维护 | 49 项寄存器读取、可写参数回读、只读拒绝、软件/设备零点、SAVE 与虚拟掉电恢复 |
| 传感器/范围 | 紧凑量化、连续多圈正反跨界、物理位置与编码器零偏分离 |
| 保护 | 命令有效期、无效外力、故障锁存/清除、反馈丢失、主机进程死亡后的独立固件 watchdog |
| 组合控制 | 正/反主从跟随真实积分的外力扰动，保持开始偏置，不用伪造主反馈 |
| 工具 | 既有 CLI 可执行任务；通用 HTTP 页面、外力扩展和反馈状态可用 |
| 浏览器（可选） | 真实页面操作、双击持久锚点及取消竞态；通过物理位移验证控制和主从效果，不只检查 HTTP 成功 |
| 虚拟 CAN | 保留帧时间戳、经典/FD 帧、路由冲突/租约/观察原有用例在 virtual 后端重复运行 |
| 硬件隔离 | 拒绝非虚拟 backend/非空硬件端口；虚拟专用驱动在错误后端前不注册/发命令；拒绝重复总线/设备 |

测试是软件接口和模型验收，不是“真机效果完全相同”的证明。实际跟踪误差、温升、电源波动、
固件私有行为和硬件急停应在具备现场安全条件时独立验收。
