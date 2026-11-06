# 历史实验与清理边界

仓库保留当前实现教程、可复用测试及最新结果；一次性参数搜索、已弃用候选、阶段日志和重复图表不再作为生产源码维护。清理前的全部306个改动文件、HEAD差异和文件哈希保存在本机备份：

`/home/robot/.ros/dm_motor_cleanup/20261106T172309`

其中 `before.tar.gz` 包含完整清理前文件，`tracked.patch` 对应 HEAD `ac7c118`。下列原文有单独可读副本；原始实验数据仍在各原报告指定的 `.ros` 目录，未删除。它们是历史证据，不是新构建或部署的依赖。

## 已保留的结论

- shared_reference配合实测V_BW100消除了旧轻触持续振荡；differential_torque进一步去掉共同位置滞后产生的阻力。当前公式与迁移步骤见[教程](bilateral-control-tutorial.md)。
- 旧B100三轴方案完成过302.04秒连续变速；其拆C、摩擦偏低的仿真释放余量仅0.1238个百分点，不能把有限通过写成广泛稳定性保证。
- 标定预测合格不等于控制应用合格。自动激励v3曾通过标定但双边应用出现纹波，未替换生产摩擦注册表。
- 最新A/C独立标定和重接32轮对照仍保留在仓库，见[文档索引](README.md)。

## 原文索引

<a id="calibration-quick-validation-2026-11-04"></a>
- [calibration-quick-validation-2026-11-04.md](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/calibration-quick-validation-2026-11-04.md)

<a id="calibration-automatic-excitation-2026-11-04"></a>
- [calibration-automatic-excitation-2026-11-04.md](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/calibration-automatic-excitation-2026-11-04.md)

<a id="calibration-breakaway-2026-11-05"></a>
- [calibration-breakaway-2026-11-05.md](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/calibration-breakaway-2026-11-05.md)

<a id="calibration-low-speed-and-inertia-2026-11-05"></a>
- [calibration-low-speed-and-inertia-2026-11-05.md](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/calibration-low-speed-and-inertia-2026-11-05.md)

<a id="third-motor-physical-test-2026-11-05"></a>
- [third-motor-physical-test-2026-11-05.md](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/third-motor-physical-test-2026-11-05.md)

<a id="three-motor-bilateral-baseline-2026-11-05"></a>
- [three-motor-bilateral-baseline-2026-11-05.md](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/three-motor-bilateral-baseline-2026-11-05.md)

<a id="2026-11-06-bilateral-unilateral-comparison"></a>
- [2026-11-06-bilateral-unilateral-comparison.md](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/2026-11-06-bilateral-unilateral-comparison.md)

<a id="2026-11-06-unilateral-high-speed-drag"></a>
- [2026-11-06-unilateral-high-speed-drag.md](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/2026-11-06-unilateral-high-speed-drag.md)

<a id="third-motor-rig-20261105"></a>
- [third-motor-rig-20261105.json](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/data/third-motor-rig-20261105.json)

<a id="three-motor-speed-baseline-20261105"></a>
- [three-motor-speed-baseline-20261105.json](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/data/three-motor-speed-baseline-20261105.json)

<a id="calibration-native-position-20261105"></a>
- [calibration-native-position-20261105.png](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/images/calibration-native-position-20261105.png)

- [third-motor-rig-20261105.png](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/assets/third-motor-rig-20261105.png)

- [three-motor-speed-baseline-20261105.png](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/assets/three-motor-speed-baseline-20261105.png)

<a id="2026-11-06-c-axis-coupling-comparison"></a>
- [2026-11-06-c-axis-coupling-comparison.png](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/assets/2026-11-06-c-axis-coupling-comparison.png)

- [2026-11-06-c-axis-coupling-comparison.svg](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/docs/assets/2026-11-06-c-axis-coupling-comparison.svg)

<a id="motor_3_runtime"></a>
- [motor_3_runtime.yaml](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/config/motor_3_runtime.yaml)

<a id="physical_test_rig"></a>
- [physical_test_rig.yaml](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/config/physical_test_rig.yaml)

<a id="reconnected_friction_comparison_20261106"></a>
- [reconnected_friction_comparison_20261106.yaml](/home/robot/.ros/dm_motor_cleanup/20261106T172309/archive/config/reconnected_friction_comparison_20261106.yaml)
