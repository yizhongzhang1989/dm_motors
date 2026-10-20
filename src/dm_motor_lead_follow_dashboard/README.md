# dm_motor_lead_follow_dashboard

独立的 Lead Follow Web 控制面板，默认 <http://127.0.0.1:8083/>。
原有 CAN Dashboard 和两个单电机 Dashboard 保持各自通用职责及端口，
单机面板的操作/监控分离适用于所有电机场景，不包含主从专用功能。
面板只调用主从控制器的配置/控制接口，**不直接获取电机租约或发布单机目标**。

## 启动

在工作空间构建并加载环境后：

```bash
# 完整仿真系统：原有三个通用面板 + 独立主从面板
ros2 launch dm_motor_bringup lead_follow.launch.py

# 完整真机系统：不启动模拟器，仍默认失能
ros2 launch dm_motor_bringup lead_follow_hardware.launch.py

# 主从控制器已经运行时，只添加这个面板，避免重复启动设备/控制器
ros2 launch dm_motor_lead_follow_dashboard dashboard.launch.py
```

如果启动主从控制器时就需要面板，`ros2 launch dm_motor_lead_follow lead_follow.launch.py`
默认也会启动本包。不要再重复运行第三条命令。

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `namespace` | 空 | 与主从控制器所在命名空间一致 |
| `controller_namespace` | `lead_follow` | 相对部署 namespace 或绝对控制器路径 |
| `follow_dashboard` | `true` | 是否启动专用面板 |
| `follow_dashboard_host` | `127.0.0.1` | 默认只允许本机访问；可信局域网可显式改为 `0.0.0.0` |
| `follow_dashboard_port` | `8083` | 与三个通用面板独立的端口 |

系统入口会在启动设备前检查专用端口占用及与通用面板计划端口的冲突。
`dashboards:=false` 只关闭原有通用面板，`follow_dashboard:=false` 只关闭主从面板；
两者同时设为 false 才是完全无网页启动。

### 启动提示端口占用

`Lead Follow dashboard 127.0.0.1:8083 unavailable: ... Address already in use`
表示已有进程占用专用面板端口，不是电机使能或串口通信失败。
若已有主从系统正在运行，直接打开其面板，不要重复执行完整启动命令。

需要重新启动时，先请求停止并确认两台失能，再到原启动终端按 Ctrl+C，
等待启动进程及其子节点退出后，重新执行启动命令。若实例由助手在后台启动，
先让助手退出该实例，把启动控制权交回终端。
**关闭网页只释放网页控制/请求停止，不会退出服务器或释放端口。**
不要只修改网页端口来绕过同一 CAN 设备的占用，也不要终止身份不明的进程。

刚启动时，收到电机反馈不代表驱动初始化已完成。需等待两台驱动 `ready=true`、
反馈新鲜且确认失能后，面板才允许编辑配置或开始跟随；初始化期间显示「初始化中」。

## 操作

1. 打开面板仅观察，不使能。
2. 页面只有右上角固定的「开始」「停止」两个操作按钮，无需先启用网页操作。
   点击开始时自动建立非独占网页心跳会话，不获取电机独占权；打开页面仅监控，不建立操作会话。
   单机面板可以同时连接、监控和操作；其它 ROS Topic 也可以直接控制，不需先释放谁的租约。
   多个主从浏览器共享当前网页操作会话，任一页面可停止/结束会话。
3. 两台确认失能时，直接编辑主/从角色、同向/反向、Kp/Kd 和可选运动限制。
   编辑只改变本页草稿，点击「开始」时一并原子应用，不需要单独应用配置。
4. 点击右上角「开始」直接应用当前配置并请求开始，仿真和真机均不弹二次确认窗口。
   点击前请自行确认现场安全。该显式操作在真机请求中携带 `confirm_hardware=true`；
   API 的确认字段、参数校验和初始化/失能条件校验保持兼容，不会因页面加载或状态轮询自动开始。
   等待顶部显示「正在跟随」，不能把 HTTP 接受请求当作使能完成。
5. 运行时角色、方向和参数禁止修改。点击右上角「停止」或按 Escape；
   非输入控件聚焦时也可按 Space。等待顶部显示已停止以及双轴失能确认。
6. 隐藏或关闭页面会停止**网页会话启动**的跟随；监控连接仍保留。
   不会自动接管或停止经 CLI 启动的跟随。
   显式点击「停止」始终会请求停止主从控制器，无需先建立网页会话。
   如果正在等待后台会话，停止或离开页面会取消尚未提交的开始请求，迟到的会话响应不会使能。

页面显示双轴位置/速度/力矩/温度、反馈频率和年龄、启动偏置、
理想映射目标/最近发送目标/跟随误差、位置曲线以及控制器事件。
编辑中的配置不会被轮询覆盖。网页采样约 5 Hz，不是电机控制频率，也不替代逐帧记录。
初次未运行时不把零初始化目标显示为有效跟随目标。
**不设电机命令互斥，最新有效目标生效。** 主从持续输出时，单机面板或 Topic 的目标可能在下个周期
被覆盖，由用户决定何时停止各发送源。页面显示最近输入源，只有观测含义。
控制器 `rate`、超时及设备命名空间仍需停止后通过 launch/YAML 重启修改；
面板中的角色选择对应 `swap_roles`，不改动电机自身 direction 或坐标。

## 心跳与控制边界

- 浏览器每 250 ms 发送心跳，服务端 750 ms 未收到则释放网页会话并请求 STOP。
- 网页开始使用 `LeadFollowControl` 的独立操作会话。服务端只有在浏览器会话有效时，
  才向 C++ 控制器续发操作心跳。
- C++ 控制器的 `operator_timeout` 默认 1 秒；面板进程崩溃或 ROS 连接失效时也会停止。
  不依赖浏览器关闭事件一定送达。停止请求仍需新鲜失能反馈确认。
- 原有 `/lead_follow/start` Trigger CLI 路径不要求网页心跳，保持原有行为。
- 此面板不提供单机归零、清错、模式或寄存器维护；应停止后使用对应的通用单机工具。
- 没有登录鉴权。网页会话只用于输出心跳，同源检查防跨站误请求，不是用户认证或控制权互斥；
  不要暴露公网。ROS 图中的其他客户端仍属于可信控制环境。
- 软件保护不是硬件安全回路，异常时应使用物理停止。

## 测试

```bash
colcon test --packages-select dm_motor_lead_follow_dashboard dm_motor_lead_follow dm_motor_bringup
colcon test-result --verbose
```

HTTP 测试使用模拟控制器；主从包另验证真实驱动/协议模拟器下的面板崩溃停机，
以及角色切换、错误会话、操作心跳丢失和原有 CLI 行为。所有自动化测试不连接真实串口。

### 无硬件浏览器操作回归

在工作空间根目录、已加载 ROS 和安装环境的终端运行：

```bash
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 \
  python3 src/dm_motor_lead_follow_dashboard/test/browser_fixture.py
```

打开 <http://127.0.0.1:18993/>。夹具中的反馈和控制服务全部是假的，没有 CAN 或电机驱动；
「真机」标签仅用于覆盖真机开始请求路径，不能用这些结果宣称真实运动通过。
可验证自动监控、直接编辑参数、点击右上角开始自动建立会话并提交且没有确认窗口、Escape 停止、
开始后禁止修改及离开页面停止。通过独立 ROS 域设置夹具参数，可以复现单机控制者占用：

```bash
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 ros2 param set \
  /follow_ui_fixture/lead_follow fixture_motor_owner /motor_1/dashboard
# 清除模拟占用：
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 ros2 param set \
  /follow_ui_fixture/lead_follow fixture_motor_owner '""'
```

兼容 `control_owner` 字段即使非空也不会再阻挡主从操作；应能正常建立网页会话和配置。
设置 `fixture_ready=false` 则仍禁止开始，初始化保护与控制权互斥无关。测试结束在夹具终端按 Ctrl+C。
