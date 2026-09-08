-- summary_kpis
SELECT
  COUNT(*) AS trace_frames,
  25 AS node_count,
  SUM(fixed_deadline_miss OR COALESCE(dynamic_deadline_miss, 0)) AS deadline_misses,
  MIN(dynamic_deadline_slack_ms) AS min_dynamic_slack_ms,
  MAX(proc_ms) AS max_proc_ms,
  12.072170 AS p99_proc_ms,
  MAX(frame_complete_ms) AS max_complete_ms,
  30.295566 AS p99_complete_ms,
  SUM(proc_top_1pct OR frame_complete_top_1pct) AS high_anomaly_frames,
  (SELECT COUNT(*) FROM anomalous_frames WHERE severity = 'medium') AS localized_spike_frames
FROM frame_timings;

-- main_tails
SELECT
  printf('F%04d', frame_index) AS frame_label,
  frame_index,
  input_seq,
  output_seq,
  proc_ms,
  12.072170 AS p99_ms,
  notify_tasks_ms AS notify_ms,
  notify_tasks_ms / proc_ms AS notify_share,
  obstacle_count
FROM frame_timings
WHERE proc_top_1pct = 1
ORDER BY frame_index;

-- async_tails
SELECT
  printf('F%04d', frame_index) AS frame_label,
  frame_index,
  input_seq,
  output_seq,
  frame_complete_ms,
  30.295566 AS p99_ms,
  async_draw_ms,
  draw_lanes_ms AS lanes_ms,
  draw_lanes_ms / async_draw_ms AS lanes_share,
  obstacle_count
FROM frame_timings
WHERE frame_complete_top_1pct = 1
ORDER BY frame_index;

-- high_anomalies
SELECT
  '主回调' AS domain,
  printf('F%04d', frame_index) AS frame_label,
  frame_index,
  input_seq,
  output_seq,
  proc_ms AS total_ms,
  12.072170 AS p99_ms,
  'cyber_taskmanager_notify_tasks' AS driver_node,
  notify_tasks_ms AS driver_ms,
  notify_tasks_ms / proc_ms AS driver_share,
  obstacle_count
FROM frame_timings
WHERE proc_top_1pct = 1
UNION ALL
SELECT
  '异步完成' AS domain,
  printf('F%04d', frame_index) AS frame_label,
  frame_index,
  input_seq,
  output_seq,
  frame_complete_ms AS total_ms,
  30.295566 AS p99_ms,
  'semantic_draw_lanes' AS driver_node,
  draw_lanes_ms AS driver_ms,
  draw_lanes_ms / async_draw_ms AS driver_share,
  obstacle_count
FROM frame_timings
WHERE frame_complete_top_1pct = 1
ORDER BY total_ms DESC;

-- node_summary
SELECT
  node,
  median_ms,
  p95_ms,
  p99_ms,
  max_ms,
  max_frame,
  max_output_seq
FROM node_summary
ORDER BY max_ms DESC;
