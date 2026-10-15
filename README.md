# dm_motors

ROS 2 Humble / C++17 达妙电机控制工作空间。采用独立 CAN 服务和独立设备节点：

```text
电机 1 dashboard / CLI ─ motor_1 节点 ─┐
电机 2 dashboard / CLI ─ motor_2 节点 ─┼─ can_bus 节点 ─ USB2CAN / SocketCAN
其他设备应用 ─────────── 其他设备节点 ─┘        │
                                         CAN dashboard
```

**默认仿真、默认失能，启动和打开网页均不自动使能或运动。**
`can_bus` 是物理设备的唯一所有者，不解释达妙协议；每台电机由独立进程控制。
以下操作方法对应当前 `src/` 布局，不需要参考旧单电机阶段的启动方式。

## 目录

- [1. 工作空间与构建](#1-工作空间与构建)
- [2. 快速启动：双电机仿真与三个面板](#2-快速启动双电机仿真与三个面板)
- [3. 连接真实双电机](#3-连接真实双电机)
- [4. Dashboard 操作](#4-dashboard-操作)
- [5. 命令行控制与参数访问](#5-命令行控制与参数访问)
- [6. 配置更多电机与总线](#6-配置更多电机与总线)
- [7. 单独启动各组件](#7-单独启动各组件)
- [8. ROS 接口与控制模式](#8-ros-接口与控制模式)
- [9. 故障、保护与恢复](#9-故障保护与恢复)
- [10. 测试、日志与常见问题](#10-测试日志与常见问题)

## 1. 工作空间与构建

建议环境：Ubuntu 22.04、ROS 2 Humble、colcon。所有命令均在工作空间根目录执行。

```text
dm_motors/
├── src/
│   ├── can_bus/              # 通用帧、硬件收发、路由与客户端管理
│   ├── can_bus_dashboard/    # 总线监测与独立原始帧调试
│   ├── dm_motor_interfaces/  # 电机消息、服务和 action
│   ├── dm_motor_control/     # 每电机控制节点、CLI、协议模拟器和组合 launch
│   └── dm_motor_dashboard/   # 单电机网页面板，每台可独立启动
├── build/                   # 自动生成，不提交 Git
├── install/                 # 自动生成，不提交 Git
├── log/                     # 自动生成，不提交 Git
├── DM_CAN_API.md
└── README.md
```

首次构建：

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/dm_motors

# 首次使用、确实缺少依赖时执行：
# rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

**每个新终端都要加载 ROS 和本工作空间：**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/dm_motors/install/setup.bash
```

修改 C++、接口或构建配置后需要重新 build；`--symlink-install` 不意味着 C++ 修改会自动生效。
包已经迁移到 `src/`，旧构建产物已清理并重建；包名和 ROS 接口名称没有改变。
不要在 `src/` 内运行工作空间构建，也不要在虚拟环境未包含 ROS 依赖时直接启动 Python 节点。

## 2. 快速启动：双电机仿真与三个面板

终端 1：

```bash
ros2 launch dm_motor_control bus.launch.py dashboards:=true
```

默认读取 [sim_bus.yaml](src/dm_motor_control/config/sim_bus.yaml)，启动：

| 组件 | 命名空间 / 地址 |
| --- | --- |
| 通用 CAN 节点 | `/can/main` |
| 独立达妙设备模拟器 | `/can/main`，仅仿真需要 |
| ID 1 电机节点 | `/motor_1` |
| ID 2 电机节点 | `/motor_2` |
| CAN dashboard | <http://127.0.0.1:8090/> |
| ID 1 dashboard | <http://127.0.0.1:8081/> |
| ID 2 dashboard | <http://127.0.0.1:8082/> |

默认仿真配置中 ID 2 的 `direction=-1.0`；真机配置中两台均为 `1.0`。
模拟电机和真实电机走同样的 CAN 服务与独立节点边界，但仿真不验证实际负载、电气保护或固件行为。

终端 2 查询状态：

```bash
ros2 run dm_motor_control motor_client.py --namespace /motor_1 state
ros2 run dm_motor_control motor_client.py --namespace /motor_2 state
```

若不需要网页：

```bash
ros2 launch dm_motor_control bus.launch.py
```

组合 launch 参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `config` | 安装包中的 `config/sim_bus.yaml` | 总线和电机 YAML 配置 |
| `namespace` | 空 | 部署前缀，例如 `robot` |
| `dashboards` | `false` | 同时启动总线及每台电机面板 |
| `dashboard_host` | `127.0.0.1` | 组合启动时所有面板的监听地址 |
| `can_dashboard_port` | `8090` | 第一条总线的面板端口，后续总线依次递增 |
| `motor_dashboard_base_port` | `8081` | 第一台电机的面板端口，后续电机依次递增 |

例如：

```bash
ros2 launch dm_motor_control bus.launch.py dashboards:=true namespace:=robot \
  can_dashboard_port:=9090 motor_dashboard_base_port:=9081
```

此时接口为 `/robot/can/main`、`/robot/motor_1`、`/robot/motor_2`；
CLI 的 `--namespace` 也要加上 `/robot`。所有 HTTP 端口必须唯一。

## 3. 连接真实双电机

### 3.1 启动前

确认电源与型号匹配、接线/终端正确、机构与轴周围安全，并具备立即断电或硬件急停措施。
电机应失能、模式与配置一致，CAN 速率事先匹配。**同一个适配器只能有一个所有者**，
先停止旧驱动、厂商调试工具或其他占用串口的进程。

查看实际设备路径：

```bash
ls -l /dev/serial/by-id/
```

当前现场配置为旧式 USB2CAN、基础 ID 1/2、两台 MST_ID 均为 0、MIT 模式。
配置见 [dual_serial.yaml](src/dm_motor_control/config/dual_serial.yaml)。
其中串口路径是本机设备绑定，换适配器后必须修改，不能直接照搬：

```text
/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00
```

如需调整配置，复制一份后编辑，而不是修改节点源码：

```bash
cp src/dm_motor_control/config/dual_serial.yaml ./motors.local.yaml
# 编辑 motors.local.yaml 中的设备路径、ID、模式和现场参数。
```

### 3.2 启动

使用现有现场配置：

```bash
ros2 launch dm_motor_control bus.launch.py dashboards:=true \
  config:="$(ros2 pkg prefix --share dm_motor_control)/config/dual_serial.yaml"
```

或使用自己的配置：

```bash
ros2 launch dm_motor_control bus.launch.py dashboards:=true \
  config:="$PWD/motors.local.yaml"
```

启动会读回模式、映射及失能状态，配置并独立回读非零硬件通信 watchdog。
当前默认 `watchdog_timeout=0.2` 秒，对应该协议 `TIMEOUT=4000`，只改 RAM，
**不保存 Flash、不自动改模式、归零、标定或使能**。模式不匹配时明确报错，不自动迁就。

等待两台 `ready=true`、`feedback_fresh=true`，检查 `status=0` 和故障字段后再操作。
`simulated` 必须为 `false`；打开网页并不意味着已经使能。

后端能力：

| 后端 | 配置 | 边界 |
| --- | --- | --- |
| `sim` | 无物理设备 | 需要设备模拟器；通用 CAN 服务本身不模拟电机 |
| `serial` | 稳定串口路径 | 旧 USB2CAN，921600/8N1；标准经典 CAN、通道 0，4 字节兼容填充到 8 字节 |
| `socketcan` | 如 `port: can0` | 接口须由系统预先配置；支持相应经典 CAN/FD 能力 |

`can_bitrate` 记录配置并用于相关校验，**不会自动设置适配器、系统接口或电机速率**。
CAN-FD 的仲裁域、数据域及固件兼容性需独立确认。厂商原生 `dmcan` USB2CANFD 后端尚未接入。

## 4. Dashboard 操作

### 4.1 每台电机面板

1. 打开对应页面，核对电机 ID、HARDWARE/SIMULATION、状态和反馈新鲜度。
2. 页面会尝试申请该电机控制权；已有其他客户端持有时只能只读。
3. 点击“使能（中性目标）”，等待 **ENABLED**。
4. 编辑当前模式字段；输入、滑条及“填入当前位置”只改草稿，点击“应用目标”才发送。
5. 点击 **STOP / 失能**，以新鲜状态反馈确认失能。

使能清空旧目标，不恢复此前草稿。MIT 中 `Kp=Kd=前馈力矩=0` 是中性输出，
**不是锁轴或抱闸**；重力负载可能运动。Kp/Kd 本身也产生力矩，前馈力矩范围不是总输出硬限矩。
“填入当前位置”不设置零点，也不会自动添加保持增益。

页面显示位置、速度、反馈力矩、双温度、状态码/描述、主机与设备故障、
请求/活动目标、控制权及周期统计。曲线采用独立 SSE 推流；时间是主机接收时间，不是电机内部采样时间。

两个面板各持有自己的租约，主动 STOP 一台不会改写另一台目标。
浏览器关闭、隐藏、断开或心跳丢失时，会停止该页面输出并请求失能；
返回后需要重新连接，**不会自动恢复运动**。需要同时可见时使用两个窗口。
不要同时让 CLI 和网页争用同一电机；使用 CLI 前先在该电机网页点击“断开”。

设备 clear、主机 reset、模式切换、零点和保存位于维护区。
除明确的故障恢复条件外，维护要求失能、新鲜反馈且无活动任务；
寄存器事务还要求同总线客户端全部 quiet。界面暂时禁用操作时先等待维护窗口释放。

### 4.2 CAN 面板与独立调试

面板展示后端/设备、精确 epoch、实际收发计数与速率、队列、客户端、维护和错误，
并提供按 CAN ID/方向过滤的有界帧记录：

- `rx`：实际接收观测。
- `written`：后端成功写入观测；**不是 CAN ACK 或设备执行成功**。
- `submitted`：该 dashboard 提交了调试请求，尚不代表总线接受或写出。

默认只监测，不注册调试客户端、不发送帧。手动调试须明确确认风险、申请确切路由，
填写一次发送的数据；活动调试还需提供该设备真实有效的不透明 STOP 帧。
发送不自动循环、不扫描设备、不重放历史流量，也不解释达妙协议。
不能抢占已有电机路由，不能绕过维护锁；原始帧可能改参或运动，不能把它当作随意试发工具。

释放调试会话后停止续租，由约 0.3 秒 broker 租约到期清队列并尝试 STOP，
不是立即物理停机。完整字段、HTTP API 和后端限制见
[CAN dashboard 说明](src/can_bus_dashboard/README.md)。

### 4.3 网络访问

**两个 dashboard 均无登录鉴权，HTTP 不加密。** 只部署在本机或可信隔离网络，
不要直接暴露公网；控制租约只负责仲裁，不是网络访问鉴权。
组合启动默认监听 `127.0.0.1`。需要其他机器访问时，指定本机实际地址：

```bash
ros2 launch dm_motor_control bus.launch.py dashboards:=true \
  dashboard_host:=192.168.1.10
```

将示例 IP 换成本机地址，并通过该地址访问。CAN dashboard 校验 Host；
`0.0.0.0` 通配监听不等于允许任意 Host，优先绑定明确地址。
防火墙、VPN、路由由现场管理，程序不自动调整网络安全策略。

## 5. 命令行控制与参数访问

以下例子以 `/motor_1` 为例；操作第二台时改为 `/motor_2`。
CLI 会在需要时申请并独立续租，结束后释放控制权并请求 STOP。
同一电机已有网页或其他控制源持有租约时会明确拒绝，不强制夺权。

### 5.1 只读查询

```bash
ros2 run dm_motor_control motor_client.py --namespace /motor_1 info
ros2 run dm_motor_control motor_client.py --namespace /motor_1 state
ros2 run dm_motor_control motor_client.py --namespace /motor_1 read \
  ESC_ID CTRL_MODE TIMEOUT PMAX VMAX TMAX VBus p_m xout
ros2 run dm_motor_control motor_client.py --namespace /motor_1 export > motor_1_parameters.json
```

`state` 可在运动时读取；`read/export` 是寄存器维护事务，要求全总线 quiet。
`export` 是逐项读取快照，不是同时刻原子快照，也不保存 Flash。
部分固件的扩展寄存器可能返回 0；本机 ID 2 的 `VBus/Tpcb/Tmtr` 曾返回 0，
不能据此认定实际电压/温度为零，也不能用它覆盖普通温度反馈。

### 5.2 任务示例：先用仿真

以下有运动输出，默认仿真配置可用于学习：

```bash
# MIT 小幅定位，绝对位置以当前坐标为准
ros2 run dm_motor_control motor_client.py --namespace /motor_1 run move \
  --position 0.03 --kp 40 --kd 1 --duration 6 --settling-time 2 \
  --max-velocity 0.1 --max-acceleration 0.2 --tolerance 0.008

# MIT 阻尼型速度任务：Kp=0，Kd>0，由客户端按任务类型填写
ros2 run dm_motor_control motor_client.py --namespace /motor_1 run velocity \
  --velocity 0.05 --kd 1 --duration 2 --max-velocity 0.1 \
  --max-acceleration 1 --ramp-time 0.2

# 限时纯前馈力矩，Kp=Kd=0
ros2 run dm_motor_control motor_client.py --namespace /motor_1 run torque \
  --torque 0.1 --duration 1 --ramp-time 0.2
```

可用任务：`move`、`velocity`、`torque`、`hold`、`sine`、`ramp`、`trajectory`。
所有任务有时限；正常结束和取消都请求失能，当前不提供结束后永久保持使能的选项。
位置任务须与当前模式兼容，MIT 位置任务需要正 Kp。速度/力矩任务需要正 `ramp_time`。
分段轨迹使用 `--times` / `--positions`，2..256 点，首时间为 0、首位置接近当前反馈；
各段采用静止端点五次插值，需满足总时限和速度/加速度约束。
SINE 定义为 `base + amplitude × (1−cos(2πft))/2`，从 base 静止开始。
任务参考限速不等于实际速度硬限制。

### 5.3 真机命令的额外确认

CLI 对真机运动要求 `--allow-hardware-motion`，对维护要求 `--allow-hardware-maintenance`，
均放在子命令之前。它们只是防误操作确认，不替代现场安全批准。

真机运动前先查 `state`，根据**当时当前位置**决定目标，不直接照搬上面的绝对位置。
例如，经现场确认后：

```bash
ros2 run dm_motor_control motor_client.py --namespace /motor_1 \
  --allow-hardware-motion run move --position <确认后的绝对目标位置> \
  --kp 40 --kd 1 --duration 6 --settling-time 2 \
  --max-velocity 0.1 --max-acceleration 0.2 --tolerance 0.008
```

`<确认后的绝对目标位置>` 是说明占位符，必须替换为实际数值后才能执行。
Kp=40、Kd=1 是本机小行程测试参数，不是所有机构、负载通用的推荐增益。

### 5.4 STOP 与维护

STOP 不需要持有租约，也不需要硬件确认 flag：

```bash
ros2 run dm_motor_control motor_client.py --namespace /motor_1 stop
ros2 run dm_motor_control motor_client.py --namespace /motor_2 stop
```

下面是维护命令语法，**不是每次启动必须执行的步骤**：

```bash
# 清除设备保护错误；成功后仍失能
ros2 run dm_motor_control motor_client.py --namespace /motor_1 --allow-hardware-maintenance clear

# 仅在存在独立主机锁定、反馈新鲜且设备已失能时使用
ros2 run dm_motor_control motor_client.py --namespace /motor_1 --allow-hardware-maintenance reset

# 切模式或改内部增益：仅在理解型号参数、已失能且全总线 quiet 时操作
ros2 run dm_motor_control motor_client.py --namespace /motor_1 --allow-hardware-maintenance mode 2
ros2 run dm_motor_control motor_client.py --namespace /motor_1 --allow-hardware-maintenance write KP_ASR 2.0

# 软件零点仅改变主机关节坐标；--device 才是设备归零
ros2 run dm_motor_control motor_client.py --namespace /motor_1 --allow-hardware-maintenance zero
ros2 run dm_motor_control motor_client.py --namespace /motor_1 --allow-hardware-maintenance zero --device

# 显式保存全部 RAM 参数，不只保存最近一项
ros2 run dm_motor_control motor_client.py --namespace /motor_1 --allow-hardware-maintenance save
```

改模式后再启动时，配置的 `mode` 也必须匹配；未保存的设备参数可能在断电后恢复。
Flash 保存有寿命限制，不要周期执行，ACK 也不等于已经验证掉电持久化。
不开放任意写 ID、速率、映射、只读或未公开寄存器；`info` 中
`writable` 是固件 RW 权限，`raw_write_allowed` 才表示普通写接口是否开放。

## 6. 配置更多电机与总线

从 [sim_bus.yaml](src/dm_motor_control/config/sim_bus.yaml) 或
[dual_serial.yaml](src/dm_motor_control/config/dual_serial.yaml) 复制配置。
根结构是 `/**: ros__parameters:`，保留 `bus_names`、`motor_names`、`buses`、`motors`。

```yaml
/**:
  ros__parameters:
    control_rate: 200.0
    state_rate: 200.0
    bus_names: [main]
    motor_names: [motor_1, motor_2, motor_3]
    buses:
      main:
        backend: sim
        port: ""
        can_fd: false
        can_bitrate: 1000000
    motors:
      motor_1: {bus: main, motor_id: 1, master_id: 0, mode: 1}
      motor_2: {bus: main, motor_id: 2, master_id: 0, mode: 1}
      motor_3: {bus: main, motor_id: 3, master_id: 0, mode: 1}
```

- 电机名称形成 ROS 命名空间；HTTP 电机端口按 `motor_names` 顺序递增。
- 同一总线基础 ID 必须唯一，当前达妙 profile 使用 1..15。不同总线可重复 ID。
- 同一物理设备/接口不能声明成多个所有者；增加电机不等于增加 USB 所有者。
- `master_id` 必须与设备真实反馈地址一致，不自动按电机 ID 推导。
- 当前 profile 为 `j4310_v11` / `j4310_v12`；不能将所有达妙型号自动当作兼容。
- 增加第二条总线时，在 `bus_names/buses` 声明，并通过每台 `bus` 绑定。
  只启动通用总线而无电机时，使用下一节的独立总线 launch。
- 注册和发送受总线预算、路由及队列约束；配置 1000 Hz 不代表多设备实际达到 1000 Hz。

常用每轴参数：

| 参数 | 默认 / 语义 |
| --- | --- |
| `mode` | 1；启动时要求与设备一致 |
| `direction` / `software_offset` | `1.0` / `0.0`；目标和反馈双向坐标变换 |
| `command_timeout` / `feedback_timeout` | `0.25` / `0.1` 秒 |
| `watchdog_timeout` | `0.2` 秒，写入后回读，不自动保存 |
| `position_limit/velocity_limit/torque_limit` | 0 表示跟随映射；命令范围，不是物理硬限幅 |
| `kp_limit/kd_limit` | 500 / 5，受协议范围限制 |
| `host_protection_enabled` | false；额外主机物理阈值仅显式开启后实施 |
| `measured_velocity_limit/measured_torque_limit` | 默认 0，未额外设置阈值 |
| `estimated_torque_limit` | 默认 0；可选合成 MIT 请求估算限制 |
| `mos_temperature_stop/rotor_temperature_stop` | 默认 0；可选主机温度阈值 |
| `position_min/position_max` | 可选非对称关节边界；无显式设置时按映射/偏置解析 |
| `fault_group` | 空；独立轴，不因另一台设备保护错误联动 |
| `legacy_control` | false；新部署使用 tokenized 接口 |

配置是启动配置；模式、零点、参数和任务通过专门接口修改。
不支持无实际生效的 `ros2 param set`，其余配置需失能后修改 YAML 并重启。
[dual_serial_test.yaml](src/dm_motor_control/config/dual_serial_test.yaml) 是历史显式保护测试配置，
不是默认真机操作配置。

## 7. 单独启动各组件

### 7.1 只启动 CAN 服务

```bash
# 无硬件的通用模拟线缆；不会自动生成设备反馈
ros2 launch can_bus can_bus.launch.py namespace:=can/main backend:=sim

# 或：独立打开真实适配器；不能与组合 launch 重复占用
ros2 launch can_bus can_bus.launch.py namespace:=can/main \
  backend:=serial port:=/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00
```

总线 launch 参数为 `namespace/backend/port/can_fd/can_bitrate`。
仅启动模拟 CAN 服务时还需要独立设备模拟器；入门优先使用组合仿真。

### 7.2 电机节点连接已有总线

在已运行 `/can/main` 的情况下：

```bash
ros2 launch dm_motor_control motor.launch.py motor_id:=1 master_id:=0 \
  start_can_bus:=false can_bus_namespace:=/can/main

ros2 launch dm_motor_control motor.launch.py motor_id:=2 master_id:=0 \
  start_can_bus:=false can_bus_namespace:=/can/main
```

**必须设置 `start_can_bus:=false`**，否则单电机 launch 默认还会启动自己的总线服务。
直接运行 `motor_node` 也只连接已有 CAN 服务，不创建适配器：

```bash
ros2 run dm_motor_control motor_node --ros-args \
  -r __ns:=/motor_2 -p motor_name:=motor_2 -p motor_id:=2 \
  -p master_id:=0 -p can_bus_namespace:=/can/main
```

需要完整单电机仿真时：

```bash
ros2 launch dm_motor_control motor.launch.py motor_id:=1 backend:=sim
```

其默认 CAN 命名空间是 `/motor_1/can`，**不是**组合启动的 `/can/main`。

### 7.3 单独启动面板

不需要重新启动已经运行的总线或电机：

```bash
ros2 launch can_bus_dashboard dashboard.launch.py \
  bus_namespace:=/can/main host:=127.0.0.1 port:=8090

ros2 launch dm_motor_dashboard dashboard.launch.py \
  motor_id:=1 namespace:=/motor_1 host:=127.0.0.1 port:=8081 command_timeout:=0.25

ros2 launch dm_motor_dashboard dashboard.launch.py \
  motor_id:=2 namespace:=/motor_2 host:=127.0.0.1 port:=8082 command_timeout:=0.25
```

电机面板的 `command_timeout` 必须与对应驱动一致。**单独的电机面板 launch**
历史默认是 `host=0.0.0.0、port=8080`，因此上例显式指定本机地址及不同端口；
不要与组合启动重复启动同端口面板。

## 8. ROS 接口与控制模式

以 `/motor_1` 为例，类型均位于 `dm_motor_interfaces`：

| 相对接口 | 类型 | 含义 |
| --- | --- | --- |
| `state` | `msg/MotorState` | 反馈、阶段、模式、故障、控制权、时序和范围 |
| `setpoint` | `msg/MotorSetpoint` | 带 token/序号/时间戳/有效期的持续目标 |
| `control_lease` | `srv/ControlLease` | acquire / renew / release，0.3..5 秒租约 |
| `operation` | `srv/MotorOperation` | 使能、失能、主机复位、设备清错、模式、零点、保存 |
| `registers` | `srv/RegisterAccess` | 每批 1..16 项原始 uint32/binary32 参数事务 |
| `get_operation` | `srv/GetOperation` | PENDING / CONFIRMED / REJECTED / UNKNOWN |
| `get_capabilities` | `srv/GetCapabilities` | profile、后端、模式、49 项寄存器元数据 |
| `run_task` | `action/RunTask` | 有界任务、反馈、取消和结果 |
| `joint_states` / `diagnostics` | 标准 ROS 消息 | 每台电机自己的关节状态和诊断 |

`MotorTarget` 按模式区分字段，不适用字段必须为零：

| 模式 | 字段 | 单位 / 说明 |
| --- | --- | --- |
| 1 MIT | position、velocity、kp、kd、torque | rad、rad/s、Nm/rad、Nm·s/rad、Nm |
| 2 位置速度 | position、velocity_limit | rad、rad/s；速度字段为非负上限幅值 |
| 3 速度 | velocity | rad/s，允许正负 |
| 4 力位混控 | position、velocity_limit、current_ratio | 电流比例 0..1，不是直接给定 Nm |

持续目标必须晚于本次使能，时间戳/序号递增；
`valid_for` 在 `(0, command_timeout]`，其剩余有效期到期也会 STOP。
续租不能代替新鲜运动目标，内部任务生成目标也不能代替客户端续租。
先等待使能确认再发布目标；命令 QoS 为 best-effort、volatile、keep-last=1。
状态 QoS 为 reliable、volatile、keep-last=1。无效输入会明确报错，不静默截断。

服务成功可能只表示已接受/发送。`operation_id` 非零时必须查异步终态，
ID 为 0 的使能/失能以状态反馈确认。参数批次有序但非原子，超时不证明写入未发生；
UNKNOWN 不盲目重试，不用旧缓存假装本次成功。
旧 `command/enable/reset` 接口仍存在，但默认关闭无 token 使能；
`legacy_control=true` 仅供迁移旧客户端，不能绕过已有明确租约。

电机数据路径为：目标 → 电机节点校验/编码 → `CanTransport` →
`can_bus/TxFrame` → 总线仲裁 → 后端写入；反馈反向解码发布。
通用总线的注册、过滤器、心跳、epoch、维护及帧接口详见
[can_bus 说明](src/can_bus/README.md)；其他设备应新建设备协议节点，不修改总线服务。

## 9. 故障、保护与恢复

### 9.1 保护职责

默认 `host_protection_enabled=false`，电机固件负责物理保护。
主机不重复实施额外测量力矩/速度/温度或合成力矩阈值，但保留：

- 协议合法性和显式命令范围检查；
- 独占控制权、目标有效期、通信失联及进程失效 STOP；
- 显式使能、固件通信 watchdog 和不恢复旧目标。

这些不替代硬件急停，也不代表断链时失能帧一定到达。
PMAX/VMAX/TMAX 是编码映射，不是机械行程、额定性能或认证硬限矩。
需要机构专用主机保护时才显式开启并配置相应阈值。

### 9.2 设备故障与主机故障

| 情况 | 处理 |
| --- | --- |
| `device_fault=true` | 查看原始 `status/status_text`，先排除设备原因，再执行 `clear` |
| 设备 clear 得到新鲜 status=0 | 恢复 DISABLED；无独立主机错误时不再要求 reset，不自动使能 |
| `host_fault=true` | 排查通信、无效目标、发布超时等；新鲜失能反馈后显式 `reset` |
| 两类错误同时存在 | clear 不抹去独立主机错误，仍需对应恢复 |
| 非空故障组锁定 / 总线会话失效 | 见下一节，不能仅靠单轴 reset 绕过 |

设备保护出现后丢弃旧目标、发送一次 STOP 完成 CAN 活动结束握手，随后查询/报告状态；
真正主机故障仍有 STOP 重试。未知状态码保留并报告，不当作正常状态。
不要用重启、clear 或取消 watchdog 掩盖未排除的实际问题。

### 9.3 总线失联、故障组与停机顺序

空 `fault_group` 下单轴主动 STOP、设备错误或独立节点退出不改写其他轴目标。
USB 断开、CAN 服务故障等共享资源异常仍可能影响全部设备。
非空同名故障组使用保守分布式锁停，当前清除需重启 CAN 服务及该总线全部客户端。
总线 epoch 变化后，旧客户端不会自动恢复会话、租约或运动。

正常关闭建议：

1. 分别请求两台 STOP，检查 `status=0`、反馈新鲜；正常阶段应为 DISABLED。
2. 在网页断开控制会话，或退出任务客户端。
3. 对组合 launch 按 `Ctrl+C`；分开启动时先停电机客户端，再停 CAN 服务。

不能将 `Ctrl+C` 当成硬件急停；节点退出时会尽力 STOP，但硬件/通信失效可能无法确认。
清理 `build/install/log`、移动源码或重新部署前，必须先停止依赖它们的运行进程。

## 10. 测试、日志与常见问题

### 软件验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select can_bus can_bus_dashboard dm_motor_control dm_motor_dashboard
colcon test-result --verbose
```

当前源码迁移后的完整回归记录为 **111 tests，0 errors，0 failures**。
自动测试采用独立 ROS domain 和模拟后端，不打开真机、不自动进行实物运动。
涵盖协议、参数事务、路由冲突、第三种 CAN-FD 设备、进程退出/重启、
独立控制、故障注入/清除、dashboard API 和推流。

### 运行观测

```bash
ros2 topic echo /motor_1/state
ros2 topic echo /motor_2/state
ros2 topic echo /can/main/status
ros2 bag record /motor_1/state /motor_2/state \
  /motor_1/joint_states /motor_2/joint_states /can/main/status
```

`feedback_rate` 是有效反馈统计；`period_p50_ms/period_p95_ms/period_p99_ms` 来自近期控制周期，
最大周期/遗漏计数为累计值。普通 Linux、USB 和 DDS 不是硬实时系统。
寄存器维护期间其他失能节点的反馈可能暂时过期；必须如实显示，不能伪造为新鲜状态。

### 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `Package not found` | 确认在根目录构建，当前终端已 source `install/setup.bash` |
| 串口被占用 | 停止其他适配器所有者；同一 USB 不要同时用组合启动和单独总线启动 |
| 串口 `Permission denied` | 检查当前设备权限，按现场策略授权；不要把所有串口设为全局可写 |
| `ready=false` | 看电机 error 和总线 status；核对设备、ID、MST_ID、模式和 CAN 速率 |
| 控制权被占用 | 断开对应网页/退出旧控制客户端；不要强制绕过租约 |
| 维护被拒绝 / 按钮禁用 | 让同总线设备失能，等待 `maintenance_available`；使能还需 `activity_available` |
| 页面切回后不再控制 | 隐藏时已释放并 STOP；重新连接、显式使能，不恢复旧目标 |
| 定位任务失败 | 查看实际误差/稳定时间、增益及负载，不把“发出了帧”当定位成功 |
| 节点称设备故障 | 查看状态码并排除原因；clear 仅清设备错误，不能代替处理实际故障 |

最近一次真机测试只验证当前 ID 1/2、旧 USB2CAN 的 MIT 小行程：
Kp=40、Kd=1 下单独及同时运动通过，单轴 STOP 时另一轴继续工作。
设备 clear API 在健康失能真机上得到确认，真实过流/过温未人为触发；
实际故障展示与清除路径通过模拟注入验证。其他模式、负载、型号及固件组合仍需现场认证。

协议资料另见 [DM_CAN_API.md](DM_CAN_API.md)。该文包含特定型号/版本的协议归纳及历史记录，
不替代当前启动配置，也不意味着本仓库附带其引用的所有厂商手册。
