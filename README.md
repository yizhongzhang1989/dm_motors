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

colcon build --symlink-install
source install/setup.bash
```

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
| `dashboard_host` | `127.0.0.1` | 组合启动时所有面板的监听地址 |
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

1. 打开对应页面，核对电机 ID、HARDWARE/SIMULATION、状态和反馈新鲜度。
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
设备 clear、零点和保存位于维护区，主机 reset 位于“电机控制”栏。
除明确的故障恢复条件外，维护要求失能、新鲜反馈且无活动任务；
寄存器事务还要求同总线客户端全部 quiet。界面暂时禁用操作时先等待维护窗口释放。

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
本机此前仅做过 300 圈验证。更新被驱动拒绝后暂停实时滚动，修正并显式应用目标后恢复。
STOP、断连及模式变化会清除待发送位置；重新使能不会重放旧拖动。新页面使用控制代次和目标序号，
后端拒绝乱序或属于 STOP 前代次的迟到目标，旧版不带排序头的 API 调用保持兼容。

#### 控制频率与实际测量

启动时可覆盖频率，例如：

```bash
ros2 launch dm_motor_bringup dual_motor_hardware.launch.py control_rate:=200
```

仿真入口、下层 `bus.launch.py` 和单电机 `motor.launch.py` 同样支持 `control_rate`。
Dashboard 的“电机控制频率”可在失能或手动使能状态下单独调整该电机，修改后立即提交；
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
准备任务终点前先关闭“实时手动目标”，再编辑手动控制区的**目标草稿**并确认提交；
否则已使能状态下修改该区域会立即改变手动目标。任务提交本身不先发送一次终点命令。
任务面板说明各类型允许的模式、增益及斜坡要求。总时长还受驱动 `max_task_duration` 限制。
轨迹输入每行 `时间(s), 位置(rad)`，支持 2～256 点，首点必须匹配当前连续关节位置。

任务期间 dashboard 不再发布手动目标流，显示驱动进度、位置误差与最终结果。
成功、失败、取消或超时后均不恢复旧目标；“取消任务并失能”通过 STOP 终止驱动任务。
浏览器心跳/驱动租约在任务期间仍然有效，断连、页面隐藏或租约到期仍请求失能。
接受/结果超时明确标为未知并请求失能，不自动重试任务。

#### 参数写入与维护确认

可单独或顺序读取全部已公布参数；只有驱动声明 `raw_write_allowed=true` 的参数可输入数值并写入。
后端按 `float32`/`uint32` 编码，要求控制权和新鲜失能状态，驱动再次校验权限及范围。
写入必须等独立回读确认；只改 RAM，不自动保存 Flash。只读、ID、映射、波特率、
watchdog 等受保护参数不能通过本面板绕过专用维护路径。

置零、保存、参数写入和任务提交使用**非阻塞确认窗口**，等待确认时心跳继续，
避免原生阻塞对话框导致租约到期。确认窗口内也有 STOP；`Esc`、顶部 STOP 和控制区 STOP
均请求失能，不受普通操作忙状态阻挡。软件 STOP 仍不能替代硬件急停。

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
ros2 launch dm_motor_bringup dual_motor.launch.py \
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
- 当前 profile 为 `j4310_v11` / `j4310_v12`；不能将所有达妙型号自动当作兼容。
- 增加第二条总线时，在 `bus_names/buses` 声明，并通过每台 `bus` 绑定。
  只启动通用总线而无电机时，使用下一节的独立总线 launch。
- 注册和发送受总线预算、路由及队列约束；配置 1000 Hz 不代表多设备实际达到 1000 Hz。

常用每轴参数：

| 参数 | 默认 / 语义 |
| --- | --- |
| `mode` | 通用和现场配置默认 1（MIT）；默认要求与设备一致，除非显式启用启动配置模式 |
| `configure_mode_on_startup` | 默认 false；显式 true 时，确认失能后在 RAM 配置所需模式并独立回读；现场多圈配置已开启 |
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
  bus_namespace:=/can/main host:=127.0.0.1 port:=8080

ros2 launch dm_motor_dashboard dashboard.launch.py \
  motor_id:=1 namespace:=/motor_1 host:=127.0.0.1 port:=8081 command_timeout:=0.25

ros2 launch dm_motor_dashboard dashboard.launch.py \
  motor_id:=2 namespace:=/motor_2 host:=127.0.0.1 port:=8082 command_timeout:=0.25
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
先等待使能确认再发布目标。实时 Topic 回调仅更新原子“最新目标”邮箱，控制循环每周期取一次，
完成鉴权、时效/序号/限位检查并编码下发；不会排队追赶已被新目标替代的旧命令。
target/setpoint 使用独立回调组和接收执行线程，不与低频配置、查询和状态计时器争用同一个执行器。
反馈、命令结果和任务事件异步发布，DDS 慢读者的发送反压不直接阻塞控制循环或命令处理线程。
这一合并策略只适用于控制目标，`feedback` 仍逐帧保留。
`state` 中 `accepted_target_sequence/applied_target_sequence/target_accept_latency_ms`，
以及逐帧 `feedback` 的应用序号和目标接收延迟用于观测软件控制链路。
延迟按发送端 ROS 时间戳至驱动控制周期接受目标计算，依赖两端时钟同步，
不是机械跟随延迟或硬实时保证。

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

命令结果成功可能只表示已接受/发送。`operation_id` 非零时必须查异步终态，
ID 为 0 的使能/失能以状态反馈确认。参数批次有序但非原子，超时不证明写入未发生；
UNKNOWN 不盲目重试，不用旧缓存假装本次成功。
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
colcon test --packages-select can_bus can_bus_dashboard dm_motor_control dm_motor_dashboard dm_motor_bringup
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
`control_rate` 与 `actual_control_rate` 分别为设定和实测循环频率；频率更改会清空周期窗口和最大周期，
遗漏计数仍为累计值。普通 Linux、USB 和 DDS 不是硬实时系统。
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
