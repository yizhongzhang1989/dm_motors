# 独立 A / C 轴短时高速测试

`run_axis_sweep.py` 面向 **A/id1 与 C/id3 已经机械分离**的台架。每次实验只使能其中一轴，另外两轴保持失能，双边控制器保持 IDLE。每次完成或失败都会请求三轴失能和双边 STOP，并用新的反馈、连续 0.2 秒静止窗口以及独立守护进程确认停机。

本工具采集无摩擦前馈的短时速度扫描，用于评价和建立独立电机的滑动摩擦模型。它不执行静摩擦、温度稳定或完整机械惯量标定，也不修改驱动参数、零点、Flash 或系统参数注册表。

## 轨迹

目标由 `../bilateral/native_stream/stream.cpp` 编译得到的 C++ 程序以 500 Hz 发送，固定 MIT `Kp=10`、`Kd=0.6`、`torque_ff=0`。Python 进程负责观察、操作握手和停机，不生成高频目标。

每次扫描包含起点等待、余弦加速、0.6 秒恒速、余弦减速、换向等待、反向返回和停止等待。首段方向由 `direction` 指定；每轮始终包含正负两个方向，整轮属于同一个训练或验证角色。

| 目标速度 / rad/s | 单轮运动时间 / s | 单向最大位移 / rad |
| --- | --- | --- |
| 0.6 | 3.25 | 0.510 |
| 1.5 | 3.25 | 1.275 |
| 3.0 | 3.25 | 2.550 |
| 6.0 | 3.85 | 6.000 |
| 9.0 | 4.65 | 10.800 |
| 13.2 | 5.77 | 19.536 |

`q0` 来自运行前的新反馈。程序验证整条参考轨迹严格位于原生 MIT 位置范围 `(-12.4,12.4)` rad 内；主机多圈位置限制并不会扩大原生位置指令的编码范围。越界会在使能前失败，需要另行停机移位或选择合适的首段方向。

## 配置与执行

每个实验提供一个 JSON 文件。路径必须绝对，`output` 必须尚不存在。`env` 是完整进程环境；执行入口会在导入 ROS 前使用这个环境重新启动 Python，确保 ROS overlay 的 `PYTHONPATH` 与动态库搜索路径生效。

```json
{
  "schema": 1,
  "output": "/absolute/experiments/A-training-1-13.2",
  "axis": 1,
  "speed": 13.2,
  "direction": -1,
  "role": "training",
  "repeat": 1,
  "native_binary": "/absolute/build/rig_stream",
  "env": {"PATH": "/replace/with/the/complete/validated/environment"},
  "expected_bandwidth_hz": 100.0,
  "guards": {"speed": 15.9, "torque": 3.8, "travel": 20.1, "temperature": 48.0},
  "files": {
    "system_parameters_file": "/absolute/config/system_parameters.yaml",
    "config_file": "/absolute/config/bilateral_physical_rig.yaml"
  },
  "metadata": {"mechanical_setup": "A-C disconnected"}
}
```

`env` 示例只是占位，必须替换为部署所使用的完整环境。`expected_bandwidth_hz` 用于核验对应电机的 RAM `V_BW` 寄存器 32，只读查询不会更改带宽。每个电机的实际带宽单独保存，分析时应检查条件是否一致。

```bash
python3 tools/physical_rig/independent/run_axis_sweep.py --config /absolute/INPUT.json
python3 tools/physical_rig/independent/run_axis_sweep.py --config /absolute/INPUT.json --execute
```

第一条仅输出计划，不导入 ROS、不创建实验目录，也不要求 native 二进制已存在。第二条执行真机流程。程序先完成固定 8 秒发现、额外 2 秒稳定等待、能力检查及 V_BW 确认，然后才启动有限时长守护和目标流。

测试结束输出 `TRIAL_RESULT.json`，协议为 `independent_speed_sweep_v1`。其中 `success` 代表执行和停机成功；实际速度是否达到 `4π rad/s`、数据是否完整和模型是否通过独立验证，需要离线分析结论。

## 保存内容

- `INPUT.json`、`inputs/`、`provenance.json`、`sources/`：运行配置、原配置副本、二进制与源码指纹。
- `commands.jsonl`：C++ 实际发送的参考目标；`stream-result.json` 包含 GO 后单调时钟起点、最大目标间隔、轴号和方向。
- `feedback.jsonl`、`state.jsonl`、`targets.jsonl`、`bilateral.jsonl`、`bus.jsonl`：原有只读记录器的完整消息格式。
- `wire.jsonl`：CAN RX 和实际 `TX_observed`，保留原帧数据与接收时间。
- `events.jsonl`、`guard-result.json`、`recorder-result.json`、`TRIAL_RESULT.json`：握手、寄存器读取、独立保护与结果。

固定稳态评价窗口取每个恒速段中央 0.3 秒，时间基准为 `native_summary.started`。反馈时间使用驱动的 `receive_time`；MotorState 的观测源时间为 `feedback_receive_time + feedback_age`，不应把记录器回调时间当作电机采样时间。

短窗口未必覆盖完整机械一圈，应描述为这些位置、速度、温度和带宽条件下的短时滑动特性评价。训练、验证按完整实验轮次分离；不能把同一轮的正向与反向片段分别当成独立训练集和验证集。

## 离线检查

```bash
python3 -m pytest -q tools/physical_rig/independent/test_run_axis_sweep.py
```

这些测试不初始化 ROS，覆盖错误轴号、无效配置、原生位置边界、双向轨迹、实际带宽确认、被动轴意外使能或运动、陈旧反馈、外来目标源和子进程退出超时处理。C++ 数学测试只提取轨迹函数编译执行，不创建 ROS 节点。

## 串行矩阵与完整批次归档

`run_matrix.py` 使用显式 JSON 计划逐个调用上述有限时长 runner，并在每轮停止后调用标定系统的 `load_trial`，检查两个方向的固定窗口及 CAN 质量。它冻结 runner、native、分析源码和输入配置的哈希；文件中途改变、窗口质量失败、失能确认不完整或结果缺失都会终止后续执行。

```json
{
  "schema": 1,
  "output": "/absolute/new_matrix",
  "inputs": ["/absolute/pending_trial_INPUT.json"],
  "completed_trials": ["/absolute/previously_completed_trial"],
  "diagnostic_trials": ["/absolute/previously_failed_preparation"]
}
```

`completed_trials` 和 `diagnostic_trials` 可省略。`inputs: []` 配合非空 `completed_trials` 是纯离线批次审计，不启动硬件子进程；已完成轮次同样必须通过 `load_trial` 的逐窗口质量检查，不能只凭 `success` 字段。正式轮次按轴号、角色、重复编号和速度去重，诊断必须通过 `inspect_diagnostic_trial` 的原始记录核验。

```bash
python3 tools/physical_rig/independent/run_matrix.py --config /absolute/MATRIX_INPUT.json
python3 tools/physical_rig/independent/run_matrix.py --config /absolute/MATRIX_INPUT.json --execute
python3 -m pytest -q tools/physical_rig/independent/test_run_matrix.py
```

第一条只输出计划。第二条执行计划；若 `inputs` 为空则只做离线审计。每个待执行输入最多自动重试一次，且只允许诊断检查证明 **没有 ENABLE 请求、没有任何目标、三轴已停机**的准备失败。新重试使用新的输入文件和 `-matrix-retry01` 输出目录；原文件保持原字节。任何 ENABLE 后的失败、目标/反馈质量失败、未知结果、进程中断或超时都禁止自动重试。

矩阵目录中的 `PLAN.json`、归档输入、每次进程日志和 `MANIFEST.json` 保留全部尝试。成功完成的 manifest 在 `report_selection` 中按轴列出 `formal_trials` 和 `diagnostic_trials`，供离线报告使用；`matrix_complete=true` 表示该轴明确包含 8 轮训练和 3 轮验证。只应使用 `status=completed` 的矩阵作为完成批次；`status=halted` 保留故障及已经获得的证据，不能代表整批完成。
