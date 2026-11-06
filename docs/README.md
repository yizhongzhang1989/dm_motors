# 项目文档

- [双边力控实现与迁移教程](bilateral-control-tutorial.md)：控制律、抑振与减阻、坐标与协议、构建、部署及验收。
- [迁移验收记录模板](bilateral-control-deployment-record.md)：设备身份、参数、源程序和逐轮结果。
- [源码精简与真机验收](cleanup-validation.md)：删减范围、保留功能、新构建和实际硬件测试。
- [A/C重接后的减阻对照](2026-11-06-reconnected-friction-comparison.md)：32轮、四档速度，单边/双边效果及波动、跟随边界。
- [A/C独立高速标定结果](2026-11-06-independent-ac-calibration.md)：独立训练/验证、实速超过4π/s及模型范围。
- [独立轴高速标定操作](calibration-independent-speed-sweep.md)：采集、拟合、导入及最新实验恢复。
- [DDS发布停顿与本机UDP配置](2026-11-06-dds-publication-stall.md)：已验证的时序证据、显式配置与未定位限制。
- [历史实验索引](experiment-history.md)：阶段报告、弃用候选、原始来源与清理前备份。

接口分别见[控制器](../src/dm_motor_bilateral_control/README.md)、[标定](../src/dm_motor_calibration/README.md)、[物理工具](../tools/physical_rig/README.md)。
