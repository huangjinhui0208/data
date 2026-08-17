# 底层解析器单元测试

- 命令：`PYTHONPATH=realtime_defect_report/scripts/vendor/realtime_collision_analysis/src python3 -m pytest -q realtime_defect_report/scripts/vendor/realtime_collision_analysis/tests/test_realtime_collision_core.py`
- 结果：`16 passed in 0.50s`
- 状态：PASS

第一次直接调用pytest时因未设置vendored解析器的`PYTHONPATH`而在收集阶段失败；补充正确导入路径后全部测试通过。该首次失败是测试启动环境问题，不是解析逻辑失败。
