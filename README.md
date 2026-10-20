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
- [11. 后续开发的实时性边界与验收](#11-后续开发的实时性边界与验收)
- [12. 2026-10-19 真机验收记录](#12-2026-10-19-真机验收记录)

## 1. 工作空间与构建

建议环境：Ubuntu 22.04、ROS 2 Humble、colcon。所有命令均在工作空间根目录执行。

```text
dm_motors/
├── src/
│   ├── can_bus/              # 通用帧、硬件收发、路由与客户端管理
│   ├── can_bus_dashboard/    # 总线监测与独立原始帧调试
│   ├── dm_motor_interfaces/  # 电机 Topic 消息、异步命令和任务事件
│   ├── dm_motor_bringup/     # 系统级启动入口，后续场景 launch 统一放在这里
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

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

控制与延迟验证使用 `RelWithDebInfo`（GCC 下为 `-O2 -g`），保留调试符号并启用优化。
空的 `CMAKE_BUILD_TYPE` 通常不启用优化；不能把未优化构建的测量当成部署性能。
已有构建目录也可用上面的命令切换；不要在真实控制进程运行时替换其二进制或共享库。

**每个新终端都要加载 ROS 和本工作空间：**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/dm_motors/install/setup.bash
```

修改 C++、接口或构建配置后需要重新 build；`--symlink-install` 不意味着 C++ 修改会自动生效。
已有工作空间只需补建新入口及依赖时，可执行 `colcon build --packages-up-to dm_motor_bringup --symlink-install`，
随后重新加载 `install/setup.bash`。
包已经迁移到 `src/`，旧构建产物已清理并重建；包名和 ROS 接口名称没有改变。
不要在 `src/` 内运行工作空间构建，也不要在虚拟环境未包含 ROS 依赖时直接启动 Python 节点。
终端环境与编辑器解释器相互独立；若 VS Code/Pylance 仍引用已删除的 `.venv/bin/python`，
在 `Python: Select Interpreter` 中选择 ROS 对应的系统 Python（本机为 `/usr/bin/python3`），
重载语言服务后重新检查导入诊断，不能把一次构建通过当作编辑器索引也已更新。

**工作区 DDS 默认配置：** 加载 `install/setup.bash` 后，Fast DDS 默认使用随包的
[UDPv4 配置](src/can_bus/config/fastdds_udp.xml)，避免进程异常退出时共享内存端口恢复
影响其他电机。它不修改 ROS Topic QoS、CAN 后端或系统网络设置；显式 DDS 配置文件和其他
`RMW_IMPLEMENTATION` 选择会被保留。所有驱动、客户端和记录器应从加载过工作区环境的终端启动。
已有进程不会热切换配置，应先确认失能并停止旧系统，再重新加载环境和启动。

## 2. 快速启动：双电机仿真与三个面板

终端 1：

```bash
ros2 launch dm_motor_bringup dual_motor.launch.py
```

**这是仿真入口，不会连接串口，也不会使能真实电机。真机请使用
`ros2 launch dm_motor_bringup dual_motor_hardware.launch.py`，不要仅凭网页能打开就认为已连接真机。**

默认读取 [sim_bus.yaml](src/dm_motor_control/config/sim_bus.yaml)，启动：

| 组件 | 命名空间 / 地址 |
| --- | --- |
| 通用 CAN 节点 | `/can/main` |
| 独立达妙设备模拟器 | `/can/main`，仅仿真需要 |
| ID 1 电机节点 | `/motor_1` |
| ID 2 电机节点 | `/motor_2` |
| CAN dashboard | <http://127.0.0.1:8080/> |
| ID 1 dashboard | <http://127.0.0.1:8081/> |
| ID 2 dashboard | <http://127.0.0.1:8082/> |

以上回环地址供运行服务的本机使用。三个面板默认监听 `0.0.0.0`（全部本机 IPv4 接口），
同一可信局域网的其他电脑应把 `127.0.0.1` 换成**服务主机的局域网 IP**，端口仍为
8080 / 8081 / 8082。`0.0.0.0` 是监听配置，不是其他电脑应打开的网址。
面板没有登录鉴权，不应暴露到公网；只需本机访问时加 `dashboard_host:=127.0.0.1`。

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

`dm_motor_bringup` 是系统级启动包，当前提供两个明确区分的入口：

- `dual_motor.launch.py`：默认仿真。
- `dual_motor_hardware.launch.py`：默认加载现场 `dual_serial.yaml`，连接真实适配器，不启动模拟器。

复用 `dm_motor_control/bus.launch.py`，默认启动一条 CAN 总线、两台独立电机及三个 Dashboard。
仿真时额外启动协议模拟器，真机时不启动模拟器；始终不自动使能或运动。
后续其他场景在该包的 `launch/` 下新增入口，不重复实现总线/电机节点的组合逻辑。

bringup launch 参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `config` | 安装包中的 `config/sim_bus.yaml` | 总线和电机 YAML 配置 |
| `namespace` | 空 | 部署前缀，例如 `robot` |
| `control_rate` | 空（使用 YAML，当前 200 Hz） | 覆盖所有电机控制频率，20–1000 Hz；低于配置状态频率时同步降低状态频率 |
| `dashboards` | `true` | 默认启动 CAN 及所有电机面板，可显式设为 `false` |
| `dashboard_host` | `0.0.0.0` | 所有面板的 IPv4 监听地址；设为 `127.0.0.1` 可限制为仅本机访问 |
| `can_dashboard_port` | `8080` | 第一条总线的面板端口，后续总线依次递增 |
| `motor_dashboard_base_port` | `8081` | 第一台电机的面板端口，后续电机依次递增 |

单总线双电机默认对应 8080/8081/8082。多条总线时 CAN 面板端口也会递增，
可能与电机端口段重叠；请显式覆盖端口段。重复或越界端口会在启动设备前报错，不自动改号。
本入口默认启用所有 Dashboard；需要无面板启动可显式设置 `dashboards:=false`，
也可继续使用底层 `dm_motor_control bus.launch.py`，其 `dashboards` 默认仍为 `false`。

**本机同一 ROS_DOMAIN_ID 和命名空间只运行一套 bringup。** 已启动时直接使用现有网页，
不要再次执行同一启动命令。bringup 在创建任何节点前取得进程级独占锁并检查面板端口，
重复启动会明确退出，不再创建第二个 CAN 状态发布者或电机服务。独占锁在启动进程退出后自动释放。
多套独立系统必须使用不同命名空间、端口及硬件资源；只换网页端口并不能隔离 ROS 节点。
如果此前重复启动已触发“CAN bus restarted; relaunch required”，需停止旧故障服务后只启动一次，
不能通过关闭总线重启保护或强行复位继续输出。

例如：

```bash
ros2 launch dm_motor_bringup dual_motor.launch.py namespace:=robot \
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

当前现场配置为旧式 USB2CAN、基础 ID 1/2、两台 MST_ID 均为 0、
**MIT 模式（mode=1）**。使能后需显式设置合适的 Kp/Kd 才有位置跟随，
Kd 不超过 1；中性使能不恢复旧增益或旧目标。当前 PMAX=12.5 时，位置滚轮限制在有效的
±12.5 rad 范围内。需要多圈绝对位置目标时，可在失能后直接选择位置速度模式，
并设置正的速度上限；切换模式不会自动使能。
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
ros2 launch dm_motor_bringup dual_motor_hardware.launch.py
```

或使用自己的配置：

```bash
ros2 launch dm_motor_bringup dual_motor_hardware.launch.py \
  config:="$PWD/motors.local.yaml"
```

启动会读回模式、映射及失能状态，配置并独立回读非零硬件通信 watchdog。
当前默认 `watchdog_timeout=0.2` 秒，对应该协议 `TIMEOUT=4000`，只改 RAM，
**不保存 Flash、不归零、标定或自动使能**。通常模式不匹配会明确报错。
现场 `dual_serial.yaml` 显式启用 `configure_mode_on_startup: true`：
仅在读回并确认新鲜失能反馈后，才将 CTRL_MODE 配置为 mode=1，独立回读并确认失能后继续启动。
只写 RAM，不保存 Flash；下次启动会重新验证。其他配置该选项默认 false，保留严格模式匹配。

启动还会在失能、近零速度时**读取一次 `0x50 p_m`**，与随后收到的紧凑反馈建立连续位置基准。
为避免同总线其他节点的启动参数事务压住反馈，读取基准前先等待总线维护结束，
并持续获得一个 `feedback_timeout` 长度的新鲜失能反馈窗口；基准读取后的维护锁释放后才宣告就绪。
电机必须保持静止，直到 `ready=true`；读取超时、非有限值或基准不一致都会阻止初始化，不回退为零。
初始化后不周期性读取位置寄存器，而是用每帧紧凑位置的软件跨界累计维持连续位置。
已验证的 J4310 profile 使用 `2 * PMAX * 65536 / 65535` 作为反馈回绕周期；
这是本机正反 300 圈测试验证过的展开规则，不保证未经验证型号/固件的行为。

状态字段：

| 字段 | 含义 |
| --- | --- |
| `position` | 连续关节位置：`direction * absolute_position + software_offset`，rad |
| `absolute_position` | 原生连续位置：启动 `p_m` 基准加软件累计，rad |
| `native_position` | 原始紧凑反馈解码位置，仍会在映射范围内回绕 |
| `absolute_position_valid` | 连续位置基准及累计是否有效；无效时 `ready=false`，禁止使能 |
| `position_wraps` | 自本次启动或显式设备置零后累计的有符号跨界次数，不是物理圈数 |

反馈接收使用适配器传来的单调接收时间；没有时间戳的仿真传输使用控制时钟。
若反馈间隔按 `VMAX` 估算已可能跨过半个回绕周期，则连续位置标记无效并锁定故障；
恢复反馈或普通故障复位不会猜测丢失圈数，必须重启节点重新读取基准。软件累计要求相邻反馈间
真实位移小于半个回绕周期，不能恢复断流期间任意多圈运动，也不保证断电保持。
显式设备置零确认后，连续位置及跨界计数重置；软件置零只改变关节坐标偏移，不改变原生累计值。

MIT 目标仍受映射范围限制，不能把紧凑反馈回绕当成目标位置也可以回绕。
位置速度/力位模式使用 `float32` 位置目标；当 `position_limit=0` 且未设置机械行程边界时，
不再额外用 PMAX 限制连续多圈目标。显式的非零 `position_limit` 仍可进一步收紧命令范围，
较大的多圈机械行程应通过 `position_min/position_max` 配置。发布的 `position_limit`
按当前模式给出有效命令范围，`pmax` 始终只是协议反馈映射，二者不能混淆。
位置模式中性使能目标使用真实连续位置；若超出显式限制则拒绝使能，
不会用回绕位置代替当前位置而导致意外回转。`joint_states`、任务误差及主机位置保护均使用连续位置。

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

两个电机实例使用同一套工作台，浏览器标题和页头明确显示电机 ID 与仿真 / 硬件环境。
位置、速度、力矩、双温度与驱动阶段放在**页面最上方约 84px 高的紧凑固定栏**，
总高度与原页头一致：电机标识与阶段、四项实时读数、连接入口与 STOP 位于同一横排，
不再分成整行标题和整行数据。读数字号增大，完整连接状态可在“连接”菜单中查看。
固定栏不随页面滚动或分区切换移出视野。完整环境说明与过长数值可悬停查看；
累计圈数、角度、紧凑位置和控制权明细集中在“完整监控”，不再占用固定栏高度。
连接管理菜单在状态面板下方展开；曲线按面板实际高度吸顶，不会遮挡状态。
窗口高度不足时曲线恢复普通滚动。窄屏只在中间读数区域横向滚动查看全部四项数据，
电机标识、连接入口和 STOP 始终留在两侧；读数区域也支持键盘聚焦。

| 分区 | 主要用途 |
| --- | --- |
| 控制与曲线（默认） | 桌面并排显示曲线和手动控制；窄屏优先显示控制。当前模式适用的参数优先排列，其余参数保留显示但禁用 |
| 运动任务 | 七类有界任务、准备目标快捷入口、执行进度与取消 |
| 完整监控 | 坐标、通信、保护、周期、请求 / 活动目标与原始状态，状态 JSON 导出 |
| 参数维护 | 故障清除、置零、保存、参数读取 / 写入与已确认参数导出 |
| 事件记录 | 最近 100 条操作 / 错误与数量提示，可清空 |

操作反馈在所有分区都可见，禁用控制旁显示原因。分区支持鼠标、Tab、左右方向键及 Home / End。
Dashboard 按**明确点击即提交**工作：置零、清故障、保存参数、写参数、应用零目标和启动任务
均不再弹二次确认；CAN 调试也没有额外授权复选框。设备回读仍显示独立的最终结果，
“已提交”不冒充“已执行”。页面不根据总线忙闲快照预先禁用使能、参数读取和失能配置请求，
实际可用性由接口 / 驱动判断，拒绝原因直接显示。
STOP / Esc 无需先连接或获取控制权；已知活动任务的“取消任务并失能”也可在断连后直接请求停止。
若 HTTP 不可达会明确显示“失能未确认”，不能将按钮可点击当作物理停止保证。
协议输入校验、驱动故障保护、独占仲裁、真实使能状态及失联处理仍然保留。
**切换页内分区不会失能、释放控制权或停止采集**；隐藏曲线时只暂停绘图。
切换 / 隐藏整个浏览器页面仍按下文的安全规则释放控制权，不自动恢复。

1. 打开对应页面，核对电机 ID、HW（硬件）/ SIM（仿真）、状态和反馈新鲜度。
2. 页面会尝试申请该电机控制权；已有其他客户端持有时只能只读。
3. 点击“使能（中性目标）”，等待 **ENABLED**。
4. 位置默认取新鲜的当前连续位置。“实时手动目标”默认开启：
   **使能且拥有控制权时，位置、速度、Kp、Kd、力矩及当前模式其他有效参数一经编辑立即提交**，
   无需点击应用；输入不完整、越界或无效时明确提示并保持最后接受的目标。
   未使能或关闭该开关时只编辑草稿；“填入当前位置”仍只改草稿，不自动发送。
5. 点击 **STOP / 失能**，以新鲜状态反馈确认失能。

使能清空旧目标，不恢复此前增益/力矩草稿，位置草稿重新以当前位置为基准。MIT 中 `Kp=Kd=前馈力矩=0` 是中性输出，
**不是锁轴或抱闸**；重力负载可能运动。Kp/Kd 本身也产生力矩，前馈力矩范围不是总输出硬限矩。
“填入当前位置”不设置零点，也不会自动添加保持增益。

**MIT 增益存在协议量化，输入的小数不等于设备使用的小数。**
Kp / Kd 均使用 12 位向下量化，编码分别为 `floor(Kp / 500 * 4095)`、
`floor(Kd / 5 * 4095)`。Kp 的第一个非零编码对应 `500/4095 ≈ 0.1221001221`；
`Kp=0.01` 和 `Kp=0.1221` 都编码为 **0**，不会产生位置刚度。
`Kp=0.13` 编码为 1、协议值约 0.122100；这是编码示例，不是安全增益推荐。
`Kd=0.2` 编码为 163、协议值约 0.199023。应用上限不会改变这两个协议尺度。

面板在增益输入旁显示草稿编码预测，并区分服务目标（量化前）与预计协议值；
正数被量化为 0 时提示，但不自动抬高增益、不重写目标、不自动使能。
这些预测不是设备增益回读，也不改变命令的提交 / 接受 / 应用语义。

MIT 下以速度或力矩输出运动多圈，并不扩大其位置目标字段的 `±PMAX` 编码范围。
连续位置、紧凑反馈以及设备零点不能混为一谈；目标 `0` 在量程内也不表示当前机构适合直接回零。
当累计位置远离零点时，不要在仍使能且目标为 0 的状态下直接增大 Kp；
应先在安全状态核对模式、坐标和允许行程。超出 MIT 位置字段范围的连续多圈目标，
应使用驱动实际支持的浮点位置模式及有界任务，而不是静默回绕或更改编码比例。

页面显示位置、速度、反馈力矩、双温度、状态码/描述、主机与设备故障、
请求/活动目标、控制权及周期统计。曲线采用独立逐帧 `feedback` 话题及 SSE 推流，
不再从限频的 `state` 状态快照取样；控制 500 Hz、状态发布 200 Hz 时，仍会传输全部收到的反馈。
逐帧反馈由独立发布线程发送，慢监控客户端的 DDS 反压不会直接阻塞电机控制循环。
时间是主机收到 CAN 帧的时间，不是电机内部采样时间。

两个面板各持有自己的租约，主动 STOP 一台不会改写另一台目标。
浏览器关闭、隐藏、断开或心跳丢失时，会停止该页面输出并请求失能；
返回后需要重新连接，**不会自动恢复运动**。需要同时可见时使用两个窗口。
不要同时让 CLI 和网页争用同一电机；使用 CLI 前先在该电机网页点击“断开”。

驱动模式选择和切换结果位于“电机控制”栏。选择新模式即直接提交，不弹确认窗口，也无需再次点提交。
切换期间暂停目标编辑和使能，经驱动回读确认后按实际模式启用相关控件；
切换失败恢复实际模式显示，不自动使能。
所有目标参数均保留显示，不适用于当前模式的数值框/滑条禁用；速度模式同时禁用位置滚轮。
“设置设备零位”和“设置软件零位”直接位于“电机控制”栏的使能 / STOP 按钮下方，
无需打开维护页。前者修改电机设备零点，后者仅修改主机关节坐标；两者都是将当前位置设为零位，
**不是发送位置 0 让电机运动回零**，也不同于“应用零目标”。
已连接并持有控制权、失能且反馈就绪时，单击直接提交，不弹确认窗口、不自动失能或使能。
提交进度、确认结果或拒绝原因在按钮下方显示；置零后重新读取驱动状态，不在界面上伪造位置归零。
设备 clear 和保存仍位于维护区，主机 reset 位于“电机控制”栏。
除明确的故障恢复条件外，维护要求失能、新鲜反馈且无活动任务；
寄存器事务还要求同总线客户端全部 quiet。提交被驱动以维护窗口占用为由拒绝时，
等待窗口释放后再操作；拒绝不自动停止另一台电机。

位置控件为**可连续转动的圆形滚轮**，没有滑块或滚动轨道，但会在当前模式的有效位置边界停止。
从当前位置开始，沿外圈顺时针拖动增加目标、逆时针拖动减少目标；刻度环随目标角度转动，
中心显示 rad 和累计圈数。滚轮转一整圈累计 `2π rad`，跨过圆周角度的 ±π 接缝不会跳变。
**松手保持目标及滚轮角度，不回中、不回零，也没有惯性续转。**
鼠标/触控笔/触屏拖动使用 pointer capture，离开控件后仍可结束手势；
经过中心无法确定方向时暂停角度累计，防止突然跳转。STOP、断连、失去有效位置或任务接管会结束手势，
重新使能不会接着执行旧指针动作。
也可使用鼠标滚轮连续调整，每像素为 0.001 rad，方向键用于更细的明确步进。
方向键每次调整 0.01 rad，Shift 为 0.1 rad；数值框仍可输入更精确的位置。
连接或使能后，未手动调整前会跟随当前位置作为草稿基准；开始调整后保持用户目标，不被反馈拉回。
未使能时滚动只改草稿。实时手动目标开启且已使能时，修改立即通过 HTTP 和可靠的
`target` Topic 提交完整的当前模式目标，不再以 20 Hz 定时重发、等待 50 ms 刷新或等待运动 RPC 往返。
请求在途时只保留最新修改，响应后立即提交最新值，不堆积过时目标；静止时不发送目标请求。
驱动在接下来的控制周期使用最新目标，独立按设定频率持续发送电机命令。
MIT 需用户设置合适的非零 Kp/Kd 才有位置控制效果；滚轮不会自动使能或设置增益。
任务运行、只读、反馈过期或连续位置无效时禁用滚动控制。
拖拽、鼠标滚轮和方向键共用限位：目标到达当前模式编码范围与显式行程限位的交集后，
数值与刻度环均停止继续向外运动；反向操作立即离开边界，不累计越界的拖动量，也不重复发送边界目标。
例如当前 MIT 映射为 ±12.5 rad 时，滚轮停在该范围内；位置速度/力位模式不额外套用 MIT 的 PMAX 边界。
手动数值框超范围时仍保留输入并明确拒绝发送，不静默截断或回绕，也不会自动切换控制模式。多圈位置控制须先在失能状态下显式选择
位置速度或力位模式；PMAX 不作为这两种模式的默认多圈位置边界。
移除人为 PMAX 边界不代表无限数值精度或已认证无限固件行程：硬件累计上限仍未知，
本机此前仅做过 300 圈验证。目标请求被拒绝时会显示原因，不自动重试被拒绝值；
下一次有效修改直接提交，不再锁定前端要求另点“应用”。在途请求之后较新的明确修改仍保留，
成功提交后更新操作反馈；STOP、模式变化及断连仍丢弃旧的待发目标。
STOP、断连及模式变化会清除待发送位置；重新使能不会重放旧拖动。新页面使用控制代次和目标序号，
后端拒绝乱序或属于 STOP 前代次的迟到目标，旧版不带排序头的 API 调用保持兼容。

#### 控制频率与实际测量

启动时可覆盖频率，例如：

```bash
ros2 launch dm_motor_bringup dual_motor_hardware.launch.py control_rate:=200
```

仿真入口、下层 `bus.launch.py` 和单电机 `motor.launch.py` 同样支持 `control_rate`。
Dashboard 的“控制与曲线 → 控制频率与驱动状态”可在失能或手动使能状态下单独调整该电机，修改后立即提交；
任务执行时禁止在面板中调整频率。范围为 20–1000 Hz，且 command/feedback/watchdog 超时必须
允许至少两个控制周期；不自动延长超时、不修改固件参数或保存 Flash。
频率更改只影响当前运行进程，重启后仍使用 launch/YAML。
状态发布频率不超过控制频率，也不超过显式配置的 `state_rate`；`state_rate=0` 跟随控制频率。

面板分别显示 `control_rate`（设定）、`actual_control_rate`（约一秒窗口内真实控制循环计数/耗时）、
当前周期，以及独立的 CAN 有效反馈频率。频率切换会重新积累窗口，期间实测值显示为 0。
这不是硬实时承诺：USB、DDS、负载和总线调度可能使实际收发低于设定，须核对反馈频率。
浏览器心跳和驱动租约继续独立续约；目标不变不再重发，但心跳/租约丢失仍失能。

#### 完整监控

- 顶部区分真实硬件/仿真、失能/使能/故障、连接新鲜度及连续位置有效性。
  同时显示连续位置的 rad、角度、累计圈数，以及原始紧凑反馈；累计圈数是相对当前设备零点的角度换算。
- “完整监控”显示坐标方向、软件偏移、回绕计数、映射及关节范围、控制者/租约、
  主机/设备故障、主机保护状态、反馈年龄/速率、控制周期 P50/P95/P99、超周期和拒绝命令数。
  可展开请求目标、活动目标与实测值的对照，以及全部原始状态字段。
- 曲线支持 5/15/60 秒窗口、连续关节/原生连续/紧凑位置源切换、活动目标虚线、
  自动局部量程、暂停绘图和清空缓存。暂停/清空曲线**不停止电机**，暂停时继续采集。
  图表固定按 50 Hz 重绘，期间到达的每个反馈点都进入缓存，绘制时按时间顺序遍历全部点，
  不只取刷新瞬间的样本，也不按屏幕像素合并。暂停绘图仍然继续采集。
  缓存最近最多 60 秒、65,536 帧，覆盖 1000 Hz 下完整一分钟；CSV 保留全部缓存点、
  推流序号、反馈计数、位置源、速度、力矩、温度、活动目标和主机 CAN 接收时间。
  SSE 按每个连接的读取进度补发全部积压点，不再只发送最新一帧。
  超出后端 65,536 帧有界缓存的慢连接会收到明确的缺失事件；ROS 反馈序号缺失也会记录并显示，
  不插值伪造数据。面板分别显示实际推流 Hz、实际绘图 FPS / 50 和缺失数。
- 状态 JSON、反馈 CSV、已确认参数 JSON 可分别导出。参数结果带确认时间，不冒充实时值。
  页面最近 100 条事件仅保存在本页面，刷新后清空，不是设备持久化日志。

#### 有界运动任务

页面提供 MOVE、HOLD、VELOCITY、TORQUE、SINE、RAMP、TRAJECTORY 七类任务，
通过 `control` Topic 的 `TASK_START` 命令及 `task_events` Topic 执行，
不再创建 ROS Action，也不在浏览器定时生成轨迹。先显式使能，
再点击“编辑任务目标 · 关闭实时提交”，进入控制区编辑**目标草稿**，返回任务分区点击“启动任务”直接提交；
该快捷入口只关闭实时提交并切换分区，不使能、不发送目标，也不停止当前输出。
也可手动关闭“实时手动目标”后准备目标；
否则已使能状态下修改该区域会立即改变手动目标。任务提交本身不先发送一次终点命令。
任务面板说明各类型允许的模式、增益及斜坡要求。总时长还受驱动 `max_task_duration` 限制。
稳定时间必须小于总时长；速度 / 力矩任务的斜坡时间不能超过总时长的一半，提交前会明确校验。
切换任务类型时，不适用的输入退出表单校验，之前在其他类型中填错的隐藏字段不会阻止提交。
轨迹输入每行 `时间(s), 位置(rad)`，支持 2～256 点，首点必须匹配当前连续关节位置。

任务期间 dashboard 不再发布手动目标流，所有手动目标输入禁用，显示驱动进度、位置误差与最终结果。
切换到其他分区后，顶部仍提供运行任务入口；驱动确认成功后进度显示 100%，不受最后一帧进度采样的时刻影响。
成功、失败、取消或超时后均不恢复旧目标；“取消任务并失能”通过 STOP 终止驱动任务。
浏览器心跳/驱动租约在任务期间仍然有效，断连、页面隐藏或租约到期仍请求失能。
接受/结果超时明确标为未知并请求失能，不自动重试任务。

#### 参数写入与设备回读

可单独或顺序读取全部已公布参数；只有驱动声明 `raw_write_allowed=true` 的参数可输入数值并写入。
后端按 `float32`/`uint32` 编码，要求控制权和新鲜失能状态，驱动再次校验权限及范围。
写入必须等独立回读确认；只改 RAM，不自动保存 Flash。只读、ID、映射、波特率、
watchdog 等受保护参数不能通过本面板绕过专用维护路径。

置零、保存、参数写入、零目标和任务均在首次明确点击时提交，不再等待额外确认窗口。
参数写入仍等待驱动独立回读；回读期间禁止重复写入，但心跳与 STOP 继续工作。
软件零点等返回 `operation_id=0` 的操作明确显示“已提交 / 无异步操作 ID”，
以新鲜状态确认结果，不再保留“尚无操作”，也不会伪造异步回读成功。
`Esc`、顶部 STOP 和控制区 STOP 不受普通操作忙状态或页面控制会话状态阻挡。
软件 STOP 仍不能替代硬件急停；提交后必须以新鲜反馈判断设备是否已失能。

### 4.2 CAN 面板与独立调试

面板展示后端/设备、精确 epoch、实际收发计数与速率、队列、客户端、维护和错误，
并提供“帧概览 / 实时帧记录 / 总线诊断 / 手动调试”四个分区。
与电机面板一样，顶部使用约 **84px 的固定单行栏**：左侧为总线身份、环境与连接状态，
中间横排 RX、驱动写出 TX、拒绝数和客户端数（保留收发速率），右侧保留释放 / 取消申请。
滚动和分区切换时计数继续更新；窄屏仅中间读数区域可横向滚动。
接口、队列、观察链路丢帧和状态年龄明细与客户端表、原始状态一起放在诊断分区；
异常提示仍显示在页面状态提示中，过长计数和命名空间可悬停查看完整内容。
按 CAN ID / 方向的筛选同时作用于聚合表和帧记录；无效 ID 在输入旁提示，不误报总线断连。

- “暂停列表”只冻结帧列表，总线状态、计数和独立调试心跳继续；不是停止设备。
- 帧 CSV 导出当前筛选下显示的最新最多 200 帧，暂停时导出冻结的列表。
- 总线诊断可导出状态 JSON，保留精确 epoch 字符串，不包含私有会话凭据。
- 错误帧、观察链路丢帧、未连接 / 过期状态均有明确提示。

帧方向：

- `rx`：实际接收观测。
- `written`：后端成功写入观测；**不是 CAN ACK 或设备执行成功**。
- `submitted`：该 dashboard 提交了调试请求，尚不代表总线接受或写出。

默认只监测，不注册调试客户端、不发送帧。手动调试直接点击“申请独占路由”申请确切路由，
填写一次发送的数据；活动调试还需提供该设备真实有效的不透明 STOP 帧。
没有额外授权勾选或确认弹窗。为兼容既有 HTTP 合同，网页在明确点击申请 / 发送时携带
`confirm:true`；不会自动申请、自动发送或跳过总线的独占路由规则。
总线 / 端点状态新鲜且能力已发现后才能申请；成功后锁定路由字段，修改路由必须先释放。
“非活动租约”使用 `active=false`，**仍可发送原始帧，并不是禁止发送的只读监视模式**。
发送不自动循环、不扫描设备、不重放历史流量，也不解释达妙协议。
不能抢占已有电机路由，不能绕过维护锁；原始帧可能改参或运动，不能把它当作随意试发工具。

释放调试会话后停止续租，由约 0.3 秒 broker 租约到期清队列并尝试 STOP，
尚未激活的注册最长等待 2 秒，不是立即物理停机。页头的释放按钮在其他操作忙时仍可用。
页头可直接释放会话，也可取消仍在进行的路由申请；取消后的迟到结果会释放，不会恢复会话。
隐藏 / 离开页面、心跳或连接失败都会停止本页面续约；恢复后直接重新申请路由，
无需再次勾选授权，且不会自动恢复调试。
浏览器心跳独立于帧查询和列表暂停，慢查询不会阻塞续约。
完整字段、HTTP API 和后端限制见
[CAN dashboard 说明](src/can_bus_dashboard/README.md)。

### 4.3 网络访问

**所有 Dashboard 均无登录鉴权，HTTP 不加密。** 只部署在本机或可信隔离网络，
不要直接暴露公网；控制租约只负责仲裁，不是网络访问鉴权。
组合启动（仿真和真机入口）、CAN 单独启动和电机单独启动均默认监听 `0.0.0.0`。
其他电脑无需安装 ROS，只需能通过网络访问服务主机的 TCP 8080、8081、8082。
在**连接电机的服务主机**启动真机服务：

```bash
ros2 launch dm_motor_bringup dual_motor_hardware.launch.py dashboard_host:=0.0.0.0
```

仅测试网页与仿真时改用 `dual_motor.launch.py`，不要把仿真页面误认为连接了真实电机。
启动后，在服务主机使用 `ip -br -4 addr` 查找与客户端网络相通的网卡 IP，
例如 `192.168.1.10`，再从其他电脑打开：

| 面板 | 其他电脑使用的地址（替换示例 IP） |
| --- | --- |
| CAN 总线 | `http://192.168.1.10:8080/` |
| 电机 1 | `http://192.168.1.10:8081/` |
| 电机 2 | `http://192.168.1.10:8082/` |

不要在其他电脑上输入 `127.0.0.1` / `localhost`，它们指向的是客户端自己。
也可以显式设置 `dashboard_host:=192.168.1.10` 只绑定选定网卡，
或用 `dashboard_host:=127.0.0.1` 恢复仅本机访问。
修改监听配置需要重新启动相应服务；刷新网页不会改变已有进程的监听地址。
当前启动入口会拒绝重复实例，应先确认电机失能并退出旧启动进程，再启动新配置。

CAN dashboard 允许配置的绑定主机名、本机系统主机名 / 短主机名、完整主机名（FQDN）、
短主机名的 `.local` 形式、回环别名，以及每个已接受连接的**实际本地目的地址**。
主机名比较不区分大小写，并支持 DNS 末尾的点；仍要求 HTTP 端口匹配。
只要客户端能把该主机名解析到服务主机，就可以用 `http://主机名:8080/` 访问。
这些名称来自服务器自身，不会通过解析客户端提交的任意域名来扩大白名单；
未知域名和跨来源控制请求仍被拒绝。Host 拒绝提示会包含收到的 Host，便于定位别名或端口错误。
两个电机面板的 API 与反馈推流均使用当前页面来源，无需修改前端代码。

若仍打不开：

- 在服务主机运行 `ss -ltnp '( sport = :8080 or sport = :8081 or sport = :8082 )'`。
  无监听表示服务尚未启动；仅 `127.0.0.1` 监听表示仍是旧配置或显式限制为本机。
- 从客户端直接打开上述 IP 地址。网络必须互通，防火墙须允许可信客户端访问相应 TCP 端口；
  Wi-Fi 客户端隔离、不同 VLAN、代理或 VPN 路由也可能阻断访问。
- 页面能打开但提示“只读”时，先在原控制页面断开并释放该电机控制权，再在远程页面申请；
  控制权互斥不是网络故障。

防火墙、VPN、路由由现场管理，程序不会自动修改这些系统配置，也不做公网端口转发。

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
若显式启用了 `configure_mode_on_startup`，下次启动会在确认失能后恢复配置指定的模式，
不是保留上次临时切换的模式。
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
- 组合启动在创建节点前校验整数 ID/模式：`motor_id=1..15`、`master_id=0..0x7FE`、
  `mode=1..4`。同一总线的反馈地址不能落在任何电机的四种控制 ID 上，也不能为参数端口 `0x7FF`。
  共用 `master_id=0` 仍受支持；不同总线分别校验。共享 ID/模式参数与每轴覆盖值使用同一次解析结果，
  模拟器与设备节点不会采用不同的默认值。单独启动节点时仍需自行保证整个总线的地址规划。
- 当前 profile 为 `j4310_v11` / `j4310_v12`；不能将所有达妙型号自动当作兼容。
- 增加第二条总线时，在 `bus_names/buses` 声明，并通过每台 `bus` 绑定。
  只启动通用总线而无电机时，使用下一节的独立总线 launch。
- 注册和发送受总线预算、路由及队列约束；配置 1000 Hz 不代表多设备实际达到 1000 Hz。

常用每轴参数：

| 参数 | 默认 / 语义 |
| --- | --- |
| `mode` | 通用和现场配置默认 1（MIT）；默认要求与设备一致，除非显式启用启动配置模式 |
| `configure_mode_on_startup` | 默认 false；显式 true 时，确认失能后在 RAM 配置所需模式并独立回读；现场配置已开启 |
| `direction` / `software_offset` | `1.0` / `0.0`；目标和反馈双向坐标变换 |
| `command_timeout` / `feedback_timeout` | `0.25` / `0.1` 秒 |
| `watchdog_timeout` | `0.2` 秒，写入后回读，不自动保存 |
| `position_limit/velocity_limit/torque_limit` | v/t 的 0 跟随映射；p 的 0 在 MIT 跟随映射，浮点位置模式默认允许连续多圈；非零限制仍生效，均不是物理硬限幅 |
| `kp_limit/kd_limit` | 500 / 1；Kd 应用层硬上限为 1，可进一步降低，不可配置为更高值 |
| `host_protection_enabled` | false；额外主机物理阈值仅显式开启后实施 |
| `measured_velocity_limit/measured_torque_limit` | 默认 0，未额外设置阈值 |
| `estimated_torque_limit` | 默认 0；可选合成 MIT 请求估算限制 |
| `mos_temperature_stop/rotor_temperature_stop` | 默认 0；可选主机温度阈值 |
| `position_min/position_max` | 可选非对称关节边界；无显式设置时按映射/偏置解析 |
| `fault_group` | 空；独立轴，不因另一台设备保护错误联动 |
| `legacy_control` | false；新部署使用 tokenized 接口 |

配置是启动配置；模式、零点、参数和任务通过专门接口修改。
本机实测 MIT 的 Kd 大于 1 容易振荡，因此发送命令及配置均拒绝超出 1 的 Kd。
协议 Kd 的 12 位编码尺度仍为 0～5，未改为 0～1；面板数值框和滑条跟随驱动发布的有效上限。
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
  bus_namespace:=/can/main host:=0.0.0.0 port:=8080

ros2 launch dm_motor_dashboard dashboard.launch.py \
  motor_id:=1 namespace:=/motor_1 host:=0.0.0.0 port:=8081 command_timeout:=0.25

ros2 launch dm_motor_dashboard dashboard.launch.py \
  motor_id:=2 namespace:=/motor_2 host:=0.0.0.0 port:=8082 command_timeout:=0.25
```

电机面板的 `command_timeout` 必须与对应驱动一致。**单独的电机面板 launch**
默认 `host=0.0.0.0`、`port=auto`，端口按 `8080 + motor_id` 分配（ID 1 为 8081，ID 2 为 8082）；
CAN 面板默认 8080，显式指定 `port` 仍可覆盖。上例同时将监听地址限制为本机；
不要与组合启动重复启动同端口面板。

## 8. ROS 接口与控制模式

以 `/motor_1` 为例，类型均位于 `dm_motor_interfaces`：

所有电机应用接口均为 Topic，旧的电机 Service、Action 以及运行时参数 Service 已移除。
这是不兼容接口迁移：旧 RPC/Action 客户端必须改用 Topic 或随包 SDK，不能继续调用旧接口。
Dashboard 和 CLI 已迁移；通用 CAN 层的注册/维护服务及仿真故障注入服务不属于电机应用接口，仍独立保留。

| 相对 Topic | 类型 | 含义 |
| --- | --- | --- |
| `state` | `msg/MotorState` | 反馈、阶段、模式、故障、控制权、时序和范围 |
| `feedback` | `msg/MotorFeedback` | 每个有效 CAN 反馈的独立采样，含接收时间、计数、连续位置和目标；不受 state_rate 限制 |
| `target` | `msg/MotorSetpoint` | reliable、volatile、keep-last=1；事件驱动目标，Dashboard 使用，不等待逐条结果 |
| `setpoint` | `msg/MotorSetpoint` | best-effort、volatile、keep-last=1；外部实时控制器连续发布 |
| `control` | `msg/MotorRequest` | reliable 异步控制/查询命令，含 client_id、request_id、时间戳与时效 |
| `control_result` | `msg/MotorReply` | 异步接受/拒绝结果，按 client_id/request_id 关联，不是 Service |
| `task_events` | `msg/MotorTaskEvent` | 按 client_id/task_id 关联任务进度、成功、失败及取消终态，不是 Action |
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
`MotorSetpoint.hold_until_lease_expiry=false` 为连续控制：剩余有效期到期即 STOP，
续租不能代替新鲜目标。设为 true 为事件驱动保持：`valid_for` 限制请求接收时效，
接受后保持到下一次目标、STOP、故障或控制租约失效。内部任务不能代替客户端续租。
STOP 之前发出、之后才到达的目标仍被明确拒绝，但不会把 STOPPING/DISABLED 再改成故障；
使能期间的无效连续目标仍锁停，不放宽时效或序号校验。
先等待使能确认再发布目标。实时 Topic 回调仅更新原子“最新目标”邮箱，控制循环每周期取一次，
完成鉴权、时效/序号/限位检查并编码下发；不会排队追赶已被新目标替代的旧命令。
target/setpoint 使用独立回调组和接收执行线程，不与低频配置、查询和状态计时器争用同一个执行器。
反馈、命令结果和任务事件异步发布，DDS 慢读者的发送反压不直接阻塞控制循环或命令处理线程。
这一合并策略只适用于控制目标，`feedback` 仍逐帧保留。
`state` 中 `accepted_target_sequence/applied_target_sequence/target_accept_latency_ms`，
以及逐帧 `feedback` 的应用序号和目标接收延迟用于观测软件控制链路。
延迟按发送端 ROS 时间戳至驱动控制周期接受目标计算，依赖两端时钟同步，
不是机械跟随延迟或硬实时保证。
逐帧 `feedback` 的目标数值与 `applied_target_sequence` 使用同一份已下发软件快照，
不会把本周期新目标配给上周期序号。任务输出使用序号 0，不沿用此前手动目标的序号。
这些字段关联的是处理反馈时最近交给 CAN 桥的目标，电机反馈协议本身不携带目标序号，
不能据此宣称该帧已经确认执行某个目标。

`header.stamp` 使用 ROS 时钟；`feedback.receive_time`、`state.feedback_receive_time`
使用本机单调时钟秒，CAN 帧的 `received_steady` 使用本机单调时钟纳秒。
反馈年龄包含桥接/本地接收排队延迟，不在控制周期处理时重新置零。
这些接收时刻也不是电机内部采样时间；跨主机不能直接比较各自的单调时钟。

控制命令 `kind` 包括 LEASE、OPERATION、REGISTERS、CAPABILITIES、OPERATION_RESULT、
CONTROL_RATE、TASK_START、TASK_CANCEL。完整字段见
[MotorRequest](src/dm_motor_interfaces/msg/MotorRequest.msg) 与
[MotorReply](src/dm_motor_interfaces/msg/MotorReply.msg)：

- LEASE 的 operation 为 ACQUIRE/RENEW/RELEASE，duration 为 0.3..5 秒。
- OPERATION 提供使能/失能、模式、零点、清错、保存等原有操作。
- REGISTERS 每批 1..16 项；OPERATION_RESULT 查询异步寄存器事务终态。
- CAPABILITIES 返回模式、寄存器目录和 `MotorConfiguration`，不再依赖 ROS 参数 Service。
- CONTROL_RATE 的 value 为 Hz，需要有效 token；运行期仅允许调整该频率。
- TASK_START 的 task 是 `MotorTask`，request_id 同时为 task_id；
  TASK_CANCEL 必须匹配原 client_id/task_id 和控制 token。
- 除无条件安全 STOP 外，命令必须携带有效 client_id、递增 request_id、
  当前 ROS 时间和 `(0,1]` 秒 valid_for；完全相同的最近请求返回缓存结果，
  同一 request_id 更改载荷、过期请求或旧序号均拒绝。
- 低频命令的结果和任务反馈是异步 Topic，不能将发布成功解释为设备已执行成功。

Python 应用可复用 [topic_client.py](src/dm_motor_control/dm_motor_control/topic_client.py)：
`motor_topics(node).endpoint("control_lease")` 等名称只是 SDK 命令路由，不是 ROS Service 名称；
`call_async()` 返回由 `control_result` Topic 完成的 Future。实时控制器直接发布 `MotorSetpoint`，
无需等待 Future。若需要可选目标接受报告，可设置 client_id/request_id；
普通实时流留空/0，不产生逐条目标应答开销。

外部控制器的典型流程为：订阅 state/feedback → 查询能力 → LEASE 获取控制权并定时续约 →
OPERATION 显式使能并等待新鲜 ENABLED/status=1 → 持续发布 setpoint →
OPERATION 失能、确认 status=0 后释放租约。不要用固定时间戳/固定序号的 `ros2 topic pub -r`
充当实时控制器；每个目标必须更新时间戳并递增 sequence。
现有 CLI 自动完成这些步骤，命令行用法保持不变。

可直接发布安全停止请求（仍需查看新鲜状态确认失能，不能替代硬件急停）：

```bash
ros2 topic pub --once /motor_1/control dm_motor_interfaces/msg/MotorRequest \
  "{kind: 2, operation: 2}"
```
状态 QoS 为 reliable、volatile、keep-last=1；逐帧反馈 QoS 为 reliable、volatile、keep-last=4096。
无效输入会明确报错，不静默截断。
`RegisterDescriptor.writable` 表示固件寄存器可写，`raw_write_allowed` 才表示当前通用写入口允许。
两者刻意不同：当前通用写入仅开放十进制地址 24–28、31–34，其余可写项使用专门操作
或尚未提供的调试流程。驱动能力声明、写请求校验和运行时执行共用同一权限策略。

命令结果成功可能只表示已接受/发送。`operation_id` 非零时必须查异步终态，
ID 为 0 的使能/失能以状态反馈确认。参数批次有序但非原子，超时不证明写入未发生；
UNKNOWN 不盲目重试，不用旧缓存假装本次成功。
CLI 初次连接会用能力查询确认双向消息链路，允许最多三次无副作用的只读握手；
使能、运动、参数写入等请求在结果未知时不会因此自动重试。
旧 `command` Topic 默认关闭无 token 控制；`legacy_control=true` 仅供迁移。
旧 enable/reset Service 不再存在，迁移桥仅支持 control 命令中的 LEGACY_ENABLE/LEGACY_RESET，
不能绕过已有明确租约。默认客户端不使用这些桥接命令。

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
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
```

测试数量以本次 `colcon test-result` 输出为准，不以历史计数代替重新运行。
自动测试采用独立 ROS domain 和模拟后端，不打开真机、不自动进行实物运动。
涵盖协议、参数事务、路由冲突、第三种 CAN-FD 设备、进程退出/重启、
独立控制、故障注入/清除、dashboard API 和推流，以及公共 RX 单次转发、
接收排队年龄/看门狗、反馈目标与序号一致性、FD 电机的精确 STOP 和组合配置预检。
CAN 注册还覆盖延迟发现、首心跳激活、未激活资源回收及进程退出后的显式重新启动；
预注册不授予活动权限，激活后仍按原配置租期失效，不延长运动期间的超时。
DDS 配置回归同时验证安装路径、保留用户配置、其他 RMW 不被修改，
以及真实 Fast DDS 参与者不再建立共享内存映射。

查看模拟链路延迟与 500/1000 Hz 实测控制频率（不验证真实 USB/CAN 或电机）：

```bash
python3 -m pytest -q -s src/can_bus/test/test_latency.py \
  src/dm_motor_control/test/test_node_integration.py -k latency
```

#### Dashboard 全功能回归

后端用例可以单独运行（先按构建章节加载 ROS 和工作区环境）：

```bash
colcon test --packages-select dm_motor_dashboard can_bus_dashboard --return-code-on-test-failure
colcon test-result --test-result-base build/dm_motor_dashboard
colcon test-result --test-result-base build/can_bus_dashboard
```

浏览器回归不计入 colcon。工作区根目录启动**纯静态**服务：

```bash
python3 -m http.server 18991 --bind 127.0.0.1 --directory src
```

用现代 Chromium 浏览器打开 <http://127.0.0.1:18991/dm_motor_dashboard/test/browser.html>。
[统一测试入口](src/dm_motor_dashboard/test/browser.html) 自动运行
[电机界面回归](src/dm_motor_dashboard/test/test_dashboard.js) 和
[CAN 界面回归](src/can_bus_dashboard/test/test_dashboard.js)，显示断言数和完整失败信息。
每次运行重新加载测试脚本、HTML、JS、CSS，不使用过期缓存。无需 Node 或第三方前端依赖；
所有控制 API / 推流均为模拟，不连接 ROS 或真实硬件，测试完成后销毁隔离 iframe。
请勿在真实控制服务页面注入或运行测试脚本。

| 范围 | 浏览器回归覆盖 |
| --- | --- |
| 电机控制 | 控制权、连接 / 断开、四种模式、数值 / 滑条 / 旋转滚轮、范围限制、实时 / 草稿提交、目标排序、频率校验、STOP / Esc、失联 / 隐藏后不恢复 |
| 电机任务与维护 | 七类任务、任务准备与取消、隐藏字段与时间约束、成功进度、故障清除 / 主机复位、设备 / 软件零点、保存、单击写入与回读、忙时 / 断连 STOP、拒绝后直接修正 |
| 监控与导出 | 三种位置源、5 / 15 / 60 秒窗口、目标虚线、暂停 / 清空、逐帧缓存、500 Hz 完整采样、状态 / 参数 JSON、反馈 CSV、事件记录 |
| CAN 观察与调试 | 计数 / 状态 / 精确 epoch、ID / 方向筛选、错误 / 空列表、暂停、CSV / JSON、直接申请、冲突 / 取消 / 迟到申请、锁定路由、单帧、FD / BRS / RTR、非活动租约、释放 / 心跳 / 隐藏 / 失联 |
| 布局与操作 | 320 / 390 / 550 / 768 / 1024 / 1366 / 1440px、各分区无页面横向溢出、滚动时状态固定并持续更新、动态高度与曲线避让、低高度紧凑布局、键盘分区切换、连接菜单不遮挡状态或 STOP |

仿真集成用例验证实际 ROS / HTTP / SSE 链路，浏览器回归验证真实 DOM 与交互。
二者都不认证真实负载、电气保护、固件停止效果或硬件急停；真机验收仍需遵循现场安全流程。

### 运行观测

```bash
ros2 topic echo /motor_1/state
ros2 topic echo /motor_2/state
ros2 topic echo /can/main/status
ros2 bag record /motor_1/state /motor_2/state \
  /motor_1/joint_states /motor_2/joint_states /can/main/status
```

`feedback_rate` 是有效反馈统计；`period_p50_ms/period_p95_ms/period_p99_ms` 来自近期控制周期，
`control_rate` 与 `actual_control_rate` 分别为设定和实测循环频率；频率更改会清空周期窗口和最大周期，
遗漏计数仍为累计值。普通 Linux、USB 和 DDS 不是硬实时系统。
寄存器维护期间，其他电机事先注册的只读 `0xCC` 查询继续执行，避免全量读参使其他轴的
连续位置失效。运动、未声明帧和其他参数事务仍受维护锁限制；若真实反馈过期，仍必须如实报告，
不能伪造新鲜状态。CAN 注册请求新增 `poll_frames`，升级后须一起重新构建和重启总线及所有 CAN 客户端。

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
| `Request timed out: TimeoutError(...)` | HTTP 连接未在 2 秒内提供完整请求行，服务器会记录并关闭该连接；不等同于 CAN 或电机故障。偶发可由浏览器空闲/预连接引起，频繁发生时检查请求和网络 |
| HTTP `BrokenPipeError` / `Browser disconnected before HTTP response completed` | 浏览器关闭、刷新或取消请求时可能出现；SSE 连接断开需结合页面连接状态检查，不代表电机节点崩溃 |
| 定位任务失败 | 查看实际误差/稳定时间、增益及负载，不把“发出了帧”当定位成功 |
| 节点称设备故障 | 查看状态码并排除原因；clear 仅清设备错误，不能代替处理实际故障 |

最近一次真机测试只验证当前 ID 1/2、旧 USB2CAN 的 MIT 小行程：
Kp=40、Kd=1 下单独及同时运动通过，单轴 STOP 时另一轴继续工作。
设备 clear API 在健康失能真机上得到确认，真实过流/过温未人为触发；
实际故障展示与清除路径通过模拟注入验证。其他模式、负载、型号及固件组合仍需现场认证。

协议资料另见 [DM_CAN_API.md](DM_CAN_API.md)。该文包含特定型号/版本的协议归纳及历史记录，
不替代当前启动配置，也不意味着本仓库附带其引用的所有厂商手册。

## 11. 后续开发的实时性边界与验收

当前分层可继续复用：通用总线唯一拥有设备，电机层负责协议/状态机/保护，
接口包提供契约，bringup 组合部署，Dashboard/CLI 只作为客户端。兼容用的旧 `Controller`
和串口包装器不是新的控制入口；新功能应进入现有 `Bus`/独立电机节点，避免形成第二套设备所有者。

2026-10-19 本机离线基线：Linux 6.8 generic、ROS 2 Humble、`RelWithDebInfo`、
工作区 UDPv4 默认配置。六包构建与完整回归通过（colcon 汇总 213 条记录），
浏览器 mock 回归通过 329 项检查；已安装默认配置下的单轴异常退出/重新启动回归连续 20 次通过。
以下为双轴协议模拟中对其中一轴的短时测量，
每档连续发送 1.2 秒目标，同时进行 50 Hz 能力查询；未使用真实 USB/CAN 或加载机械负载：

| 目标频率 | 实际循环频率 | 目标接受延迟 P50 | P99 | 观察到的不同应用序号 / 发布目标 |
| --- | --- | --- | --- | --- |
| 500 Hz | 500.0 Hz | 1.559 ms | 1.957 ms | 586 / 600 |
| 1000 Hz | 998.7 Hz | 0.307 ms | 1.110 ms | 991 / 1200 |

最新目标允许覆盖尚未处理的旧目标，上表不是每条目标必达或反馈丢包统计。
独立 CAN 模拟测试在 20 Hz RX 轮询配置下，稀疏 TX 延迟中位数为 0.528 ms、最大 2.969 ms；
500 帧/s 注入在 `wire_tx` 观察到 496/500 帧。测量受调度相位与负载影响，不能把一次 P99 当成最坏时限。

**这是软实时控制基础，不是硬实时或实机安全认证。** 提高 `control_rate` 不会自动提高
总线容量，也不代表各轴同步或机械响应更快。当前仍有：

- 普通 Linux 调度、动态分配及互斥锁；CAN 桥发送与私有 reliable DDS 接收仍可能受中间件反压影响。
- CAN 的状态/注册/维护与 I/O 共用执行器；运动期间避免动态注册和维护。
- 独立电机线程各自调度，没有多轴共同生效时间或原子同步目标。需要协调轨迹时，应增加明确的
  同步/时钟契约，不能把“相同 Hz”当成同步采样与同步输出。
- 网页只适合少量可信客户端；HTTP/SSE 并发和浏览器长窗口绘图尚无全系统负载上界认证。
  高速外环直接使用 `setpoint`，不要经 HTTP 闭环；浏览器绘图频率不等于控制或采样频率。
- 没有自动重连/故障组自动复位；这保留了显式恢复、重新确认和不自动恢复运动的边界。

以两轴、每周期每轴一条 8 字节命令和一条反馈、经典 CAN 1 Mbps、
旧 USB2CAN 921600 baud/8N1 为例，纯线速预算如下（不含 USB/系统调度、额外事务和重传）：

| 每轴频率 | 串口 TX 占用（30 B/命令） | 串口 RX 占用（16 B/反馈） | CAN 占用（约 111–135 bit/帧） |
| --- | --- | --- | --- |
| 200 Hz | 13.0% | 6.9% | 8.9–10.8% |
| 1000 Hz | 65.1% | 34.7% | 44.4–54.0% |

TX/RX 是分方向预算，不能相加后当作全双工串口容量。扩轴、改模式/帧长或后端时重新计算，
并给 STOP、维护、抖动与重传留余量；模拟吞吐不等于上述物理链路吞吐。

进入进一步实机开发前，应在具备硬件急停、限位及获准试验工况的条件下完成：

1. 核实实际型号、固件、模式、逐轴 PMAX/VMAX/TMAX 与 watchdog 回读，维持默认失能。
2. 在目标轴数/控制频率、Dashboard 和记录器负载下持续测量周期/目标延迟的 P50/P95/P99/最大值、
   实际频率、deadline misses、反馈计数缺口、总线拒绝/观察丢帧与 CPU 负载；短时单机模拟结果不替代此项。
3. 验证显式 STOP、目标/租约过期、客户端退出、总线退出和通信断开时的最终设备状态，
   独立确认固件 watchdog 的失能时间。通信失效时软件 STOP 不能保证送达。
4. 对强同步或更严格的最坏延迟要求，先定义可测量的时间预算，再评估专用周期执行线程、
   DDS 配置以及实时内核/调度、内存锁定；这些系统设置不会由普通 launch 自动修改。

## 12. 2026-10-19 真机验收记录

本次经用户确认现场运动安全后，在本机旧 USB2CAN、ID 1/2、MST_ID=0、
经典 CAN 链路上执行；实际反馈均为 `simulated=false`。使用独立 ROS 域、本机 UDP 通信、
临时保护配置和录制器，没有关闭固件 watchdog，也没有修改生产 YAML。
结论仅覆盖此次设备、固件和工况，不是所有硬件后端或物理保护的完整认证。

### 覆盖与结果

| 项目 | 真机结果 |
| --- | --- |
| 启动、状态、映射、模式和 watchdog | 两轴通过；PMAX=12.5、VMAX=50、TMAX=10，TIMEOUT=4000（200 ms） |
| 全量参数读取 | 每轴 49 项真实回应；修复维护阻断其他轴反馈后，双方连续位置保持有效 |
| MIT、位置速度、速度、力位混控 | 两轴小幅运动、模式独立回读和任务结束失能通过；速度模式实际正反方向正确 |
| MOVE/HOLD/RAMP/SINE/TRAJECTORY/VELOCITY/TORQUE | 执行/停止链路通过；位置类结合真实曲线验证，力矩仅做低前馈短脉冲，不等于力矩精度标定 |
| 控制与维护 API | 互斥控制权、错误 token 拒绝、同值 RAM 写入及回读、保护寄存器拒绝、软件零点及恢复、任务取消通过 |
| 目标/租约有效期 | 80 ms 连续目标约 90 ms 观察到失能；400 ms 租约约 395 ms 观察到失能；保持目标仍依赖活租约 |
| 独立轴与真实通信故障 | 暂停每轴控制节点约 220 ms，均出现设备状态 13 和主机锁停；另一轴继续反馈，完成设备清错及主机显式复位 |
| 多圈位置 | 两轴分别到约 +25.60/-24.91 rad 并返回；跨界计数正确，软件绝对位置与独立 `p_m` 差小于 0.4 mrad |
| 配对故障组 | 软件拒绝不适用字段后，两轴原始 CAN 失能反馈约 18.8 ms 内确认；单轴复位不能绕过组锁存，整栈重启恢复 |
| Dashboard | 真实 HTTP 获取控制权、使能、目标、STOP、释放、陈旧目标拒绝、SSE 连续性及 CAN 路由互斥通过 |

临时保护包含行程、速度、测量/估算力矩、温度和通信限值。小行程测试在约 200 Hz 下进行，
位置类任务录制窗口没有已解码反馈序号缺口；本次低速运动轨迹段位置误差 RMS 约 2.1–3.7 mrad，
最大约 6.4 mrad，不能当成任意负载下的跟踪精度保证。

双轴保持失能、Dashboard 和录制器在线时的短时通信测量：

| 设定频率 | 实际控制频率（两轴） | 实测反馈频率（两轴） | 控制周期 P99（两轴） |
| --- | --- | --- | --- |
| 200 Hz | 约 200 Hz | 约 200 Hz | 约 6.03–6.24 ms |
| 500 Hz | 约 500 Hz | 约 500–502 Hz | 约 2.56–2.67 ms |
| 1000 Hz | 约 999 Hz | 约 988–990 Hz | 约 1.52–1.85 ms |

20/50 Hz 同样通过。上述不是双轴带载 1000 Hz 运动认证，也不是硬实时上界。

### 实测发现并修复的软件问题

- 长参数批次原先会压住另一轴的状态查询，触发连续位置失效。现通过 CAN 注册的
  `poll_frames` 精确白名单保留其他设备只读反馈，参数与运动仍受维护锁限制；
  已补充路由边界、仿真复现和真机双轴全量读取回归。
- 新建 CLI 的首个能力查询可能在消息通道双向就绪前丢失回执。现仅对无副作用的初始能力查询
  进行有界握手重试，不重试结果未知的使能、运动或写入。最终六包构建及 219 条 colcon 测试记录通过。

### 异常、未测项与结束状态

**不能给出“全部功能均已正确认证”的结论。** 两轴硬件版本原始值同为 `0x56303033`，
软件版本原始值分别为 `0x39313036` / `0x37313036`；不据此猜测厂商版本编码规则。
2 号的 `VBus/Imax/Tpcb/Tmtr` 扩展诊断多次读回 0，而紧凑温度和运动反馈正常；
录包核对确认这些寄存器的原始 CAN 应答数据字本身就是 0，不是主机浮点转换造成的。
这些 0 值不能作为有效电压/电流/温度测量使用，固件兼容性与寄存器可用性仍需厂商或独立仪表确认，
没有用另一轴读数替代或掩盖异常。

未获额外持久化许可，因此未执行设备归零、Flash 保存或掉电保持验证；
没有改 ID/CAN 速率，也没有人为制造过流、过温或过压。本次适配器不提供原生 CAN-FD/SocketCAN
实机覆盖，相关软件测试不等于这些后端的硬件认证。

结束时两轴新鲜 `status=0`、无故障/任务/控制者，模式恢复 MIT、控制频率 200 Hz、
软件偏移 0、空故障组；位置约 0.376608/0.294633 rad，回到初始附近。
全部可写寄存器与正常初始化后的基准快照逐位一致；watchdog 保持正常启动配置的 200 ms。
测试栈与录制器已关闭。原始 CAN、控制和反馈的完整录制及 JSON 验收证据保留在本次会话目录，不提交 Git。
