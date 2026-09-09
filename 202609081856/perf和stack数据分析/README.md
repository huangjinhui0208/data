Prediction 11帧证据提取。复用 generate_p4_perf_analysis.py 的解析、快照、调度窗口与状态分解函数。
每帧目录包含 callback 与所有该帧实际worker的sched事件（前后100ms）及CPU完整调用栈，后者只取callback执行窗。active_trace_nodes为空的样本只表示同TID同时间，不能归因到该帧工作。
CPU采样不等于精确CPU时间；blocked可能含等待工作线程完成，不能仅凭callback stack判定根因。调度waking用于等待分界是代理，精确唤醒分界需wakeup事件。
时钟见clock_calibration.json，映射仅before快照。
