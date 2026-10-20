# dm_motor_lead_follow

ROS 2 Humble / C++17 单向主从位置跟随。参考 `DM_Motors_ref/dev/lead_follow` 的相对位置语义，
复用本工作空间的独立电机节点、非独占保活和 CAN 服务，不直接访问硬件、不写 Flash。

```text
leader/state ── lead_follow_node ── follower/setpoint
                     ├─────────── leader/setpoint（MIT Kp=Kd=速度=力矩=0）
                     └─────────── 两台 control / control_result（租约、使能、停止）

从理想目标 = 从启动位置 + direction × (主反馈位置 − 主启动位置)
```

使用电机节点的逻辑 `position`（已经应用电机 direction / software_offset 和连续位置展开），
不是紧凑反馈的单圈原始值。每次显式开始重新记录偏置，不自动对齐两台角度。
主电机零增益、零前馈不代表完全无摩擦或零实测力矩；这不是双向力反馈或硬实时控制器。

## 构建与启动

在工作空间根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to dm_motor_bringup --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# 显式选择四个面板；不自动使能
ros2 launch dm_motor_bringup lead_follow.launch.py \
  can_dashboard_ports:=config motor_dashboard_ports:=config dashboard_port:=config
# 省略全部端口则只启动 CAN、电机和主从节点，不启动任何面板
```

如果已经启动电机系统，**只添加控制器，不要重复启动总线/电机**：

```bash
ros2 launch dm_motor_lead_follow lead_follow.launch.py dashboard_port:=config
# 部署前缀与电机系统相同；角色交换示例
ros2 launch dm_motor_lead_follow lead_follow.launch.py namespace:=robot \
  leader:=motor_2 follower:=motor_1 dashboard_port:=config
```

`leader` / `follower` 是相对于 `namespace` 的电机命名空间，也接受绝对路径。
运行默认值统一来自 [config/dm_motors.yaml](../../config/dm_motors.yaml) 中
`packages.dm_motor_lead_follow.nodes.lead_follow_node`，再应用所选 package/launch 的覆盖。
端口由当前入口的 `parameters.dashboard_port` 单独定义，不再有 `follow_config` 或独立主从参数文件。
`config_file:=/绝对路径/配置.yaml` 可覆盖整套配置；角色等显式 launch 参数具有更高优先级。
直接运行节点也可使用 `--ros-args -p config_file:=...`，或环境变量 `COMMON_CONFIG`。
旧 ROS 拓扑覆盖文件使用 `config:=...`，仅用于显式测试等场景，不和公共 `config_file` 混用。

真机应先依照工作空间主 README 完成供电、安装、物理停止和单机保护检查，再启动：

```bash
ros2 launch dm_motor_bringup lead_follow.launch.py dashboard_port:=config
# 完全无网页：省略 dashboard_port（以及其它面板端口参数）
```

该入口读取 `packages.dm_motor_bringup.launches["lead_follow.launch.py"]` 的 `system` 拓扑，
一次启动真实 CAN 服务、
两台电机和主从控制器；以上命令仅显式开启主从面板，不启动其它面板或协议模拟器。
连接前核对该配置中的串口路径、电机 ID 和通信参数；
自定义整套参数通过 `config_file:=/绝对路径/配置.yaml` 指定。
支持 `namespace`、`leader`、`follower` 和 dashboard 参数。
bringup 只保留无 `_hardware` 后缀的真机入口，不再提供单独的仿真 launch。
离线回归继续通过显式模拟配置运行，不代表默认启动会使用模拟器。
如果真机电机系统已运行，仍只启动上面的独立控制器入口，不要重复启动总线。

这些命令不使能。开始跟随前确保两台处于 MIT 模式、静止、有新鲜失能反馈、无故障、
其它面板连接和操作会话不会阻挡主从启动。与参考实现不同，本包**不自动切换模式或复位故障**；
应在失能时通过现有单机 CLI/dashboard 完成维护，然后再开始。设备 watchdog 的设置与回读
仍由电机驱动负责；默认配置为 200 ms。不要在控制过程中改动电机模式、坐标或控制频率。

## 开始、停止与状态

```bash
ros2 topic echo /lead_follow/state
ros2 service call /lead_follow/start std_srvs/srv/Trigger '{}'
ros2 service call /lead_follow/stop std_srvs/srv/Trigger '{}'
```

带 `namespace:=robot` 时，上述路径加 `/robot` 前缀。
服务的 `success=true` 只表示请求已接受，**不是使能/失能完成确认**。

- `phase=3 (FOLLOWING)`：两台已确认使能，开始位置跟随。
- `phase=4 (STOPPING)`：已停止目标流，正在请求两台失能、释放租约。
- `stop_confirmed=true`：两台新鲜反馈均确认失能，控制器已不持有租约。
- `phase=5 (FAULT)`：因错误停止且已确认失能；`message` 保留错误原因。
- 无法确认停止时持续处于 STOPPING，3 秒后明确报告 `STOP UNCONFIRMED` 并继续重试；
  不假装停止完成。必要时使用物理停止。

控制器不排斥其他电机控制者；开始中途失败仍会停止两台。
运行期间单机 Dashboard 可以持续观察或发送有效目标；其它 Topic 也可以发送目标。
最后接受的有效目标生效，主从下一个周期可能覆盖手动目标，必须由用户自行协调输入源。
任何来源显式停止一台后，主从控制器会停止另一台。
其他控制者使一台失能后，跟随控制器将停止另一台。
包内 [Lead Follow Dashboard](DASHBOARD.md) 使用 `dashboard_port:=config` 后默认位于
<http://127.0.0.1:8083/>，省略端口则不启动；支持统一开始/停止、主从交换、方向、增益与运动限制配置。
开启的 Dashboard 默认监听 `0.0.0.0`，其它机器可将上述地址替换为服务主机的局域网 IP，
不需要额外指定 host。网络和防火墙仍须允许访问，仅用于可信内网。
它不改变三个通用面板。通过该面板开始时，关闭/隐藏控制页面或面板进程失联会触发操作心跳停止。
通过上述 CLI Trigger 开始的跟随不要求网页心跳，关闭原有通用网页**不会**停止，必须显式调用 stop。
正常 SIGINT/SIGTERM 退出尽力请求失能并等待最多 3 秒。进程崩溃/SIGKILL 时，电机驱动的
短有效期目标、命令超时与租约到期仍负责失能；不能替代物理停止或独立硬件安全回路。

## 参数

默认值见唯一运行配置 [config/dm_motors.yaml](../../config/dm_motors.yaml)。

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `leader`, `follower` | `motor_1`, `motor_2` | 不同电机命名空间；重启控制器后更改 |
| `direction` | `1` | 逻辑坐标同向 `1`、反向 `-1` |
| `swap_roles` | `false` | 在配置的两台电机之间交换主从；仅新鲜失能时可修改 |
| `rate` | `100.0` | C++ 周期目标发布频率，20–500 Hz；重启后更改，不承诺实测频率 |
| `kp`, `kd` | `8.0`, `0.5` | 从电机 MIT 增益，受协议及电机配置增益上限约束 |
| `feedback_timeout` | `0.1` | 状态/反馈总年龄上限（秒），包含传输及本地缓存年龄 |
| `target_validity` | `0.1` | 非保持目标有效期（秒），必须不大于两台驱动 command_timeout |
| `operator_timeout` | `1.0` | 网页操作心跳超时（秒），0.3–5；只约束经 operator_control 开始的跟随 |
| `motion_limits_enabled` | `false` | 是否启用以下额外运动限制 |
| `target_speed` | `0.5` | 从目标斜率上限，rad/s |
| `max_error` | `0.5` | 理想目标与从反馈的最大误差，rad |
| `max_travel` | `2.0` | 任一电机相对启动位置最大行程，rad |
| `max_speed`, `max_torque` | `2.0`, `3.0` | 任一电机反馈速度/力矩绝对值上限 |
| `max_temperature` | `55.0` | 任一 MOS/转子温度停止阈值，摄氏度 |

`feedback_timeout` 和 `target_validity` 至少为两个控制器周期。
开始前会读取驱动配置，拒绝不兼容的目标有效期、无效 watchdog 或过低状态频率，不进入使能。
电机状态发布频率至少为 `2/feedback_timeout`（默认要求至少 20 Hz，驱动默认 200 Hz）；
低频状态或拥塞会触发失联停止。
若驱动使用更短 command_timeout，必须相应缩短 target_validity。

额外运动限制关闭时，不限制跟随斜率、误差、行程、反馈速度/力矩/温度；
**反馈新鲜度、故障、目标有效期、有限数值、关节范围、驱动自身保护始终有效**。
不会对越界目标静默截断，而是停止两台。建议初次真机验证启用额外运动限制并使用保守增益。
最大误差检查针对理想目标，不会因目标斜率限制而掩盖主从偏差。

两台新鲜失能且控制器不在启动/运行/停止过程中时，可修改 direction、增益和运动限制：

```bash
ros2 param set /lead_follow direction -1
ros2 param set /lead_follow motion_limits_enabled true
ros2 param set /lead_follow kp 4.0
```

范围非法、非有限参数或运行中修改会明确拒绝。角色、频率、超时修改需停止后重启控制器。
既有电机 `direction` 不会被修改；离线测试用的模拟配置中 motor_2 的 direction=-1，
所以逻辑同向不一定等于原始轴坐标同向。

## 验证

```bash
colcon test --packages-select dm_motor_lead_follow dm_motor_bringup --event-handlers console_direct+
colcon test-result --verbose
```

测试使用隔离 ROS domain/命名空间，包含映射/运动限制单元测试、可控电机 Topic 测试和
真实 CAN 服务 + 电机驱动 + 协议模拟器的进程集成测试。不访问串口，不证明真实机械负载安全。

### 2026-10-20 真机非运动检查

以下为取消独占控制权之前的历史记录；其中租约冲突拒绝行为已由非独占多源输入取代。

使用真机组合入口（现为 `lead_follow.launch.py`）、现场 USB2CAN、ID 1/2，实际反馈均为
`simulated=false`。因未取得本次现场运动安全确认，全程不使能、不发送运动目标。

- 两轴 MIT 模式、TIMEOUT=4000（200 ms）回读通过；PMAX=12.5、VMAX=50、TMAX=10。
- 10 秒采样中两轴持续 ready、失能、无故障、反馈新鲜；反馈计数实测约 198.4 Hz，
  反馈年龄最大约 19.4 ms。控制器保持 IDLE，目标/使能请求计数均为零。
- 在电机 2 已被测试客户端持有租约时，主从 START 明确拒绝；没有进入使能。
  释放租约后 STOP 返回两轴新鲜失能确认。
- 原始 CAN 参数回应确认 ID 1 的 VBus 约 49.21 V、ID 2 为 0；
  软件版本原始整数分别为 959524918、925970486。
  [协议文档第 16 节](../../DM_CAN_API.md#16-本仓库当前覆盖情况与待确认项)已有 ID 2
  扩展诊断回零的记录。该值不是网页解码引入，不能据此判断实际供电电压或故障根因。

这些结果只验证本次真机通信、待机与拒绝/停止路径，**不包含主从运动精度、运动中停止时间、
负载或物理保护验收**。运动验证仍需现场安全确认及实际供电检查；不能用历史运动记录替代本次验证。
