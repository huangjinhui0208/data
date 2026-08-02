#!/bin/bash
# ══════════════════════════════════════════════════════════════
#  Apollo 10.0 perf_trace 采集（e2e_trace_v3，手动停止版）
#
#  触发：/apollo/data/scb_data/.collect_active（内容为 SESSION_ID）
#  落盘：
#    data/scb_data/<SESSION_ID>/trace/{events,message_context,fusion_inputs,trace_anchor}
#    data/scb_data/<SESSION_ID>/log/         （Apollo glog + Carla 碰撞日志）
#  （不再创建 write/：SHM write 观测默认关闭且分析未使用）
#
#  用法：
#    bash tools/start_collect.sh              # SESSION_ID = YYYYMMDDHHMM
#    bash tools/start_collect.sh my_run_01    # 同上，并写入 run_label.txt
#
#  停止：Ctrl+C 或 rm /apollo/data/scb_data/.collect_active
#        停止时会自动记录开始/结束时间并拷贝对应时间段的日志
#        （停止后等待 glog 刷盘，裁剪结束时间默认 +2s，宁可多留也不要少）。
#  分析：python3 tools/trace_analyzer/main.py <SESSION_ID> --plots
# ══════════════════════════════════════════════════════════════

SCB_DATA_BASE="${SCB_DATA_BASE:-/apollo/data/scb_data}"
SCB_COLLECT_ACTIVE="${SCB_COLLECT_ACTIVE:-${SCB_DATA_BASE}/.collect_active}"

SESSION_ID=""
RUN_LABEL=""
TRACE_DIR=""
COLLECT_START_TS=""          # 14 位 YYYYMMDDHHMMSS
COLLECT_END_TS=""            # 14 位 YYYYMMDDHHMMSS
COLLECT_TIME_FILE=""         # 持久化记录开始/结束时间，防止异常退出丢失
LOG_END_PADDING_SEC="${LOG_END_PADDING_SEC:-2}"  # 日志裁剪结束时间向后延长秒数

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC}  $(date +%H:%M:%S)  $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC}  $(date +%H:%M:%S)  $1"; }
log_step() { echo -e "\n${CYAN}══ $1 ══${NC}"; }

# 统计某目录下 *.csv shard 数量与总行数（不含表头时仍用 wc -l 近似）
summarize_csv_dir() {
    local label="$1"
    local dir="$2"
    if [[ ! -d "$dir" ]]; then
        log_warn "${label}：目录不存在（模块可能未启动或无打点）"
        return
    fi
    local shards files lines
    shards=$(find "$dir" -maxdepth 1 -name '*.csv' 2>/dev/null | wc -l)
    shards=${shards// /}
    if [[ "$shards" -eq 0 ]]; then
        log_warn "${label}：0 个 CSV shard"
        return
    fi
    files=$(find "$dir" -maxdepth 1 -name '*.csv' -print0 | xargs -0 wc -l 2>/dev/null | tail -n 1)
    lines=$(echo "$files" | awk '{print $1}')
    log_info "${label}：${shards} 个 shard，合计约 ${lines} 行"
}

add_seconds_to_ts() {
    local ts="$1"
    local secs="$2"
    local epoch
    epoch=$(date -d "${ts:0:4}-${ts:4:2}-${ts:6:2} ${ts:8:2}:${ts:10:2}:${ts:12:2}" +%s)
    date -d "@$((epoch + secs))" +%Y%m%d%H%M%S
}

stop_collect() {
    rm -f "$SCB_COLLECT_ACTIVE"

    # 记录结束时间（精确到秒），用于日志拷贝与时间一致性核对
    COLLECT_END_TS="$(date +%Y%m%d%H%M%S)"
    COLLECT_END_LOG_TS="$COLLECT_END_TS"
    if [[ "$LOG_END_PADDING_SEC" =~ ^[0-9]+$ && "$LOG_END_PADDING_SEC" -gt 0 ]]; then
        COLLECT_END_LOG_TS="$(add_seconds_to_ts "$COLLECT_END_TS" "$LOG_END_PADDING_SEC")"
    fi
    if [[ -n "$COLLECT_TIME_FILE" ]]; then
        echo "end=${COLLECT_END_TS}" >> "$COLLECT_TIME_FILE"
        echo "end_log=${COLLECT_END_LOG_TS}" >> "$COLLECT_TIME_FILE"
    fi

    if [[ "$LOG_END_PADDING_SEC" =~ ^[0-9]+$ && "$LOG_END_PADDING_SEC" -gt 0 ]]; then
        log_info "等待 ${LOG_END_PADDING_SEC}s，让 Apollo glog 刷盘..."
        sleep "$LOG_END_PADDING_SEC"
    fi

    log_info "触发文件已删除，落盘停止"
    if [[ "$COLLECT_END_LOG_TS" != "$COLLECT_END_TS" ]]; then
        log_info "采集时间段：${COLLECT_START_TS:0:8}-${COLLECT_START_TS:8:6} → ${COLLECT_END_TS:0:8}-${COLLECT_END_TS:8:6}（日志裁剪至 ${COLLECT_END_LOG_TS:0:8}-${COLLECT_END_LOG_TS:8:6}）"
    else
        log_info "采集时间段：${COLLECT_START_TS:0:8}-${COLLECT_START_TS:8:6} → ${COLLECT_END_TS:0:8}-${COLLECT_END_TS:8:6}"
    fi
    log_info "会话目录：${SCB_DATA_BASE}/${SESSION_ID}/"
    log_info "Trace 目录：${TRACE_DIR}/"
    log_info "Log  目录：${SCB_DATA_BASE}/${SESSION_ID}/log/"
    echo ""

    if [[ ! -d "$TRACE_DIR" ]]; then
        log_warn "Trace 目录不存在，请确认各模块已编译打点且进程曾运行"
        print_next_analysis_hint
        return
    fi

    log_step "落盘摘要（e2e_trace_v3）"
    summarize_csv_dir "events/" "${TRACE_DIR}/events"
    summarize_csv_dir "message_context/" "${TRACE_DIR}/message_context"
    summarize_csv_dir "fusion_inputs/" "${TRACE_DIR}/fusion_inputs"
    summarize_csv_dir "trace_anchor/" "${TRACE_DIR}/trace_anchor"

    # 旧版根目录宽表仅提示，不再作为验收依据
    local legacy=0
    for f in "${TRACE_DIR}"/*.csv; do
        [[ -f "$f" ]] || continue
        legacy=1
        log_warn "发现 legacy 宽表 $(basename "$f")，分析请用 events/ 或 trace_analyzer"
        break
    done

    # 按采集时间段拷贝 Apollo 日志到 scb_data/<SESSION_ID>/log
    copy_logs_by_time_window

    print_next_analysis_hint
}

# 调用 copy_apollo_logs_by_time.sh 拷贝采集时间段内的日志
copy_logs_by_time_window() {
    if [[ -z "$COLLECT_START_TS" || -z "$COLLECT_END_TS" ]]; then
        log_warn "开始/结束时间为空，跳过日志拷贝"
        return
    fi
    
    # 复制 Apollo 日志
    local script
    script="$(dirname "${BASH_SOURCE[0]}")/copy_apollo_logs_by_time.sh"
    if [[ ! -f "$script" ]]; then
        log_warn "未找到日志拷贝脚本：$script，跳过"
        return
    fi
    log_step "拷贝采集时间段内的 Apollo 日志"
    # 输出目录传 scb_data 根目录，脚本内部会加上 <SESSION_ID>/log
    # copy 脚本内部会对 END 再延长 LOG_END_PADDING_SEC；此处传原始 end 即可
    if ! LOG_END_PADDING_SEC="$LOG_END_PADDING_SEC" bash "$script" "$COLLECT_START_TS" "$COLLECT_END_TS" \
            "${APOLLO_LOG_SRC_DIR:-/apollo/data/log}" \
            "${SCB_DATA_BASE}/${SESSION_ID}"; then
        log_warn "日志拷贝过程中出现错误（详见上方输出）"
    fi
    
    # 复制 Carla 碰撞日志（完全不影响主流程）
    local carla_script
    carla_script="$(dirname "${BASH_SOURCE[0]}")/copy_carla_log.sh"
    if [[ -f "$carla_script" ]]; then
        log_step "Carla 碰撞日志收集"
        # 使用 set +e 确保即使这个脚本有问题也不会终止主程序
        (
            set +e
            bash "$carla_script" --session "$SESSION_ID" \
                --start "$COLLECT_START_TS" --end "$COLLECT_END_LOG_TS"
            # 忽略退出码
            true
        )
    else
        log_warn "未找到 Carla 日志拷贝脚本：$carla_script，跳过碰撞日志收集"
    fi
}

print_next_analysis_hint() {
    echo ""
    log_step "采集完成"
    echo -e "${CYAN}  SESSION_ID :${NC} ${SESSION_ID}"
    if [[ -n "$RUN_LABEL" ]]; then
        echo -e "${CYAN}  Run Label  :${NC} ${RUN_LABEL}"
    fi
    echo -e "${CYAN}  Trace 目录 :${NC} ${TRACE_DIR}"
    echo ""
    echo -e "${GREEN}  # trace 分析${NC}"
    echo "  python3 tools/trace_analyzer/main.py ${SESSION_ID} --plots"
    echo ""
}

cleanup() {
    echo ""
    log_warn "检测到手动停止，结束采集"
    if [[ -n "$SESSION_ID" ]]; then
        stop_collect
    else
        rm -f "$SCB_COLLECT_ACTIVE"
        log_warn "采集尚未初始化，已清理触发文件（如有）"
    fi
    exit 0
}

# ════════════════════════════════════════════════════════════
#  开始采集
# ════════════════════════════════════════════════════════════
log_step "开始 perf_trace 采集"

if [[ -f "$SCB_COLLECT_ACTIVE" ]]; then
    log_warn "已存在 .collect_active，将覆盖：$(cat "$SCB_COLLECT_ACTIVE" 2>/dev/null || true)"
fi

mkdir -p "$SCB_DATA_BASE"
SESSION_ID="$(date +%Y%m%d%H%M)"
if [[ -n "${1:-}" ]]; then
    RUN_LABEL="$1"
fi

TRACE_DIR="${SCB_DATA_BASE}/${SESSION_ID}/trace"
mkdir -p "${TRACE_DIR}"/{events,message_context,fusion_inputs,trace_anchor}

if [[ -n "$RUN_LABEL" ]]; then
    echo "$RUN_LABEL" > "${SCB_DATA_BASE}/${SESSION_ID}/run_label.txt"
fi

# 记录开始时间（精确到秒），作为日志拷贝的时间段起点
COLLECT_START_TS="$(date +%Y%m%d%H%M%S)"
COLLECT_TIME_FILE="${SCB_DATA_BASE}/${SESSION_ID}/collect_time.txt"
echo "start=${COLLECT_START_TS}" > "$COLLECT_TIME_FILE"

echo "$SESSION_ID" > "$SCB_COLLECT_ACTIVE"

trap cleanup SIGINT SIGTERM

log_info "SESSION_ID：${SESSION_ID}"
log_info "采集开始时间：${COLLECT_START_TS:0:8}-${COLLECT_START_TS:8:6}"
if [[ -n "$RUN_LABEL" ]]; then
    log_info "Run Label：${RUN_LABEL}"
fi
log_info "触发文件：${SCB_COLLECT_ACTIVE}"
log_info "Trace 目录：${TRACE_DIR}/"
log_info "Schema：events/ + message_context/ + fusion_inputs/ + trace_anchor/"
log_warn "不检查 channel、不自动停止；可先启模块再开采集"
log_warn "按 Ctrl+C 停止，或另开终端：rm ${SCB_COLLECT_ACTIVE}"
echo ""

SECONDS_ELAPSED=0
while true; do
    if [[ ! -f "$SCB_COLLECT_ACTIVE" ]]; then
        log_warn "检测到触发文件已删除，结束采集"
        break
    fi
    echo -ne "\r  采集中... 已运行 ${SECONDS_ELAPSED}s   按 Ctrl+C 停止   "
    sleep 1
    SECONDS_ELAPSED=$((SECONDS_ELAPSED + 1))
done
stop_collect
