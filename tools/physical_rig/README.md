# 三轴物理测试工具

当前台架：A=id1、B=id2、C=id3。C与A名义反向1:1同轴连接，B机械自由。工具默认只生成计划，显式 `--execute` 才运行；运行前核对实际接法和设备状态。

- [联轴单双边测试](bilateral/README.md)：复用 `run_trial.py` 执行短时往返、自由释放或连续变速；`score_trials.py`、`compare_coupling.py`、`score_release.py`、`score_long.py`评价原始记录。
- [独立轴高速标定采集](independent/README.md)：仅在A/C机械分离时使用 `run_axis_sweep.py`；`run_matrix.py`串行执行有明确训练/验证角色的计划，标定系统负责拟合及报告恢复。

联轴单边为C拖A、B单向跟随且不向A反馈反作用；双边为A/B双向耦合。短轮支持显式 `--extra-viscous-a`，开始前原子设置并核验，结束后三轴STOP并恢复参数。

旧 `rig_compare.py` 已由上述入口替代。只适用于2026-11-06固定A系数的32轮编排及评分脚本归入[实验历史](../../docs/experiment-history.md)，原实验目录保存其精确源码；不再为单次参数结论维护专属执行器及重复测试。最新[减阻结果](../../docs/2026-11-06-reconnected-friction-comparison.md)仍可查阅。

通用工具保留轨迹/CLI边界、反馈来源、STOP、错误结果拒绝及原始CAN评分回归；不写零点、Flash或生产标定注册表。源码精简后的新构建与真机结果见[清理验收](../../docs/cleanup-validation.md)。
