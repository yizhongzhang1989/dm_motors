# 2026-11-06：DDS 发布停顿与本机 UDP 对照

本记录说明三轴台架在订阅者建立、退出期间出现的驱动停顿，以及本机 UDP 对照的证据边界。已实测到 `tx.publish` 阻塞约 204 ms，并阻塞同一 transport 的另一个线程；Fast DDS 的 SHM 端口健康检查提供了与约 333 ms 停顿相符的机制。**目前没有内部调用栈证据证明 SHM 是每次停顿的唯一根因。** 单边／双边真机对比仍在进行，本记录不预先给出其性能结论。

## 实测到哪一层

环境为 ROS Humble、`rmw_fastrtps` 6.2.10、Fast DDS 2.6.12。原驱动的锁、调用顺序和保护保持不变；独立诊断副本只增加分段计时，慢事件由独立线程输出。该副本只在失能状态下使用，未用于运动验收。

[原始诊断日志](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/diagnostic-deployment/motor_1.log)记录：

| 同一次事件 | 实测时长 |
| --- | ---: |
| `tx.publish` | 203.964677 ms |
| 包含该调用的 `CanTransport::send` | 203.970234 ms |
| 另一线程的 `Impl::tick` 等待 transport mutex | 199.010457 ms |

`send` 持有 transport mutex，经过 `flush_outgoing` 调用发布；`tick` 等待同一锁。这一层传播链已有直接证据，不能再仅凭驱动 `max_period` 猜测是发现计数查询慢。计时没有继续拆分 DDS 内部的 writer mutex、历史缓存和 SHM 操作，因此尚不能确定该 204 ms 样本在中间件内部等待了什么。诊断缓冲可能丢弃事件，无慢事件输出也不等于严格无停顿。源码差异、二进制和依赖身份见[诊断产物清单](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/transport-latency-diagnostic/FINAL_IDENTITY.json)。

该样本发生在显式 `ASYNCHRONOUS` 下。此前异步发布的四轮只读压力检查通过，但随后仍在使能前出现 A／B 驱动故障；该轮 A、B 的最大周期分别约 202.255、329.783 ms。异步发布本身不足以消除问题，不能把这两个数字都写成 331 ms。

## 源码能解释什么

1. **`ROS_LOCALHOST_ONLY=1` 会在 XML 之后加入 SHM。** Humble 的 [participant 创建代码](https://github.com/ros2/rmw_fastrtps/blob/6.2.10/rmw_fastrtps_shared_cpp/src/participant.cpp#L159)先加载 XML，再追加回环 UDP 和 SHM transport。因此旧 `fastdds_udp.xml` 虽有 `useBuiltinTransports=false`，与该环境变量组合后仍不能称为 UDP-only。
2. **ASYNC 不消除调用线程上的所有锁等待。** [DataWriterImpl](https://github.com/eProsima/Fast-DDS/blob/v2.6.12/src/cpp/fastdds/publisher/DataWriterImpl.cpp#L787)仍在调用线程取得 writer mutex、创建并加入历史样本。本机安装头文件的 `HAVE_STRICT_REALTIME=0` 分支使用普通 mutex 锁；不能假定 `max_blocking_time` 为这次取锁提供了严格截止时间。[StatefulWriter](https://github.com/eProsima/Fast-DDS/blob/v2.6.12/src/cpp/rtps/writer/StatefulWriter.cpp#L1012)的发现及 ACK 路径也可能持同一 writer 锁创建发送资源或发送消息。这使中间件内部等待传播到异步 `publish` 成为可能，但不是对实测内部调用栈的还原。
3. **SHM 健康检查存在约 333 ms 的等待步长。** [默认描述符](https://github.com/eProsima/Fast-DDS/blob/v2.6.12/src/cpp/rtps/transport/shared_mem/SharedMemTransportDescriptor.cpp#L25)设置健康检查超时为 1000 ms；[端口实现](https://github.com/eProsima/Fast-DDS/blob/v2.6.12/src/cpp/rtps/transport/shared_mem/SharedMemGlobal.hpp#L1247)把等待步长设为整数 `1000 / 3`，即 333 ms。[healthy_check](https://github.com/eProsima/Fast-DDS/blob/v2.6.12/src/cpp/rtps/transport/shared_mem/SharedMemGlobal.hpp#L860)在端口尚未就绪时执行该等待；发送侧打开未知或失效端口可进入此路径。进程退出留下异常端口与该机制相关，但仅凭约 331–333 ms 的相近时长，不能认定所有历史事件都来自这里。

## UDP 对照结果与限制

生产对照恢复了**原生产驱动二进制、原参数和默认 SYNCHRONOUS 发布模式**，只为重新启动的驱动、控制器及观测进程显式选择本机 UDP transport。诊断二进制和 ASYNC 都没有作为所谓修复遗留在该对照中。部署记录见[production-loopback-01](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/production-loopback-01/DEPLOYMENT.json)。旧 CAN broker 和 dashboard 保留原环境，不能据此声称当时所有存量进程均无 SHM。

[正式只读压力结果](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/production-readonly-stress-02/RESULT.json)为 **6 轮、18 次查询通过**：每轮重新建立短时观测连接，跨轮另有连续记录；没有发送 ENABLE 或运动目标，三轴始终失能、反馈新鲜且无故障。三驱动报告的最大控制周期分别为 5.289、5.646、5.510 ms，全局最大约 **5.65 ms**。这是驱动周期统计，**不是生产 `publish` 的逐调用最大时长，也不是长期最坏时延保证**。

另有独立进程回环测试验证新 XML 的 UDP／127.0.0.1 限制、消息往返以及两个进程均无 Fast DDS SHM 映射，覆盖 SYNCHRONOUS 和 ASYNCHRONOUS。它没有访问 CAN，也不替代运动或压力验收，见[测试记录](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/loopback-profile-validation-sync-async/RESULT.json)。

失败证据保留在原处，没有用成功结果覆盖：

- [ASYNC 后再次故障的跨阶段记录](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/baseline-async-v2/continuous/)；[更早的连续记录](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/baseline-v2/continuous/)。
- [诊断 UDP 压力轮次](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/diagnostic-readonly-stress-01/RESULT.json)因只读能力查询超时结束，不能称为全部通过。
- [生产压力工具首轮](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/production-readonly-stress-01/RESULT.json)因 Python 订阅对象 API 不匹配失败；修正工具后的结果另存为上述 `stress-02`，不把工具异常算成电机停顿。

## 显式复用方式

新配置为 [fastdds_loopback_udp.xml](../src/can_bus/config/fastdds_loopback_udp.xml)，仅允许 UDPv4 的 `127.0.0.1` 接口。**默认 environment hook、launch 默认值和用户自定义 profile 均未自动改写。** 选择此方案需在启动所有相关新进程之前显式设置环境；已有进程不会自动继承。下面恢复与生产对照相同的默认发布模式：

```bash
# 先 source 已安装新配置的工作空间；只在原设备 owner 已停机退出后重启。
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset RMW_FASTRTPS_PUBLICATION_MODE RMW_FASTRTPS_USE_QOS_FROM_XML
unset FASTDDS_DEFAULT_PROFILES_FILE
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix can_bus)/share/can_bus/config/fastdds_loopback_udp.xml"
ros2 launch dm_motor_bringup bilateral_control.launch.py config_file:=/absolute/path/to/site.yaml
```

`ROS_LOCALHOST_ONLY=0` 在这里用于阻止 Humble 追加 SHM；本机隔离由 XML 的回环白名单实现，不用于跨机器 DDS 通信。当前 canonical XML 已保存到本轮 overlay，并新增到当前实际 can_bus 安装包中，见[产物身份](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/installed-loopback-profile/IDENTITY.json)及[当前包安装记录](/home/robot/.ros/dm_motor_bilateral_control/coupling-comparison-20261106/installed-loopback-profile/ACTIVE_PACKAGE_INSTALL.json)；未覆盖原有产物。生产对照进程使用的是此前诊断目录中等价的回环 XML，后续重启可统一采用上述包内路径。

本次没有提高调度权限、放宽反馈／控制保护、扩大 TTL／队列，或删除其他进程的 SHM 对象。随后已在这一明确配置下完成16轮U/D实测，见[真机对比报告](experiment-history.md#2026-11-06-bilateral-unilateral-comparison)；这些结果仍不足以宣布所有启动停顿已彻底解决。
