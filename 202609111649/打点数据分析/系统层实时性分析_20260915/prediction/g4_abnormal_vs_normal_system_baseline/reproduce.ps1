$ErrorActionPreference = 'Stop'
$analysisRoot = 'D:\data\202609111649\打点数据分析\系统层实时性分析_20260915\prediction'
$systemRoot = 'D:\data\202609111649\perf和stack数据分析\系统层实时性分析_20260915\prediction'
$runRoot = 'D:\data\202609111649'
$builder = "$analysisRoot\build_g4_system_baseline.py"
$selection = "$analysisRoot\g4_abnormal_vs_normal_system_baseline\selected_frames.csv"
$output = "$systemRoot\g4_abnormal_vs_normal_system_baseline\perf_extract"

python $builder prepare
if ($LASTEXITCODE -ne 0) { throw 'G4 selection preparation failed' }

python 'D:\data\reusable_scripts\prediction\extract_prediction_perf.py' `
  --timings-dir "$analysisRoot\timings" `
  --frame-csv $selection `
  --perf-run-dir "$runRoot\prediction_run_011" `
  --calibration-json "$runRoot\perf和stack数据分析\clock_calibration.json" `
  --output-dir $output `
  --sched-context "$systemRoot\sched_supported_prediction_tids.txt" `
  --allow-before-only-tid-mapping
if ($LASTEXITCODE -ne 0) { throw 'G4 perf extraction failed' }

python $builder summarize
if ($LASTEXITCODE -ne 0) { throw 'G4 baseline summarization failed' }
