#!/usr/bin/env bash
set -euo pipefail

# ══════════════════════════════════════════════════════════════
#  从 Carla 主机（10.0.0.10）拷贝本次运行的 SCB 控制时延文件，
#  本次发生碰撞时附带拷贝三个碰撞日志文件到本机（10.0.0.20）
#
#  两种使用模式：
#  1. 单独运行：
#       bash tools/copy_carla_log.sh [时间戳]
#       - 如果不传时间戳，拷贝最近仍在写入的一份 SCB 文件
#       - 如果传了 14 位时间戳（YYYYMMDDHHMMSS），按文件名精确匹配
#       - 目标目录：data/carla_log/
#
#  2. 配合 start_collect.sh 使用：
#       bash tools/copy_carla_log.sh --session <SESSION_ID> --start <START_TS> --end <END_TS>
#       - 始终只拷贝本次运行的一份 SCB 文件
#       - 采集窗口内发生碰撞时，附带拷贝三个碰撞日志文件
#       - 目标目录：data/scb_data/<SESSION_ID>/log/
#       - 这个会在 copy_apollo_logs_by_time.sh 之后调用
# ══════════════════════════════════════════════════════════════

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC}  $(date +%H:%M:%S)  $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC}  $(date +%H:%M:%S)  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $(date +%H:%M:%S)  $1"; }
log_step() { echo -e "\n${CYAN}══ $1 ══${NC}"; }

CARLA_HOST="scb@10.0.0.10"
CARLA_LOG_DIR="~/apollo10/apollo/data/log"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCB_DATA_BASE="${SCB_DATA_BASE:-${REPO_ROOT}/data/scb_data}"

# ══════════════════════════════════════════════════════════════
#  使用说明
# ══════════════════════════════════════════════════════════════
usage() {
  echo "用法："
  echo
  echo "  1. 单独运行（拷贝最新的或指定启动时间戳的 SCB 时延文件）："
  echo "     bash tools/copy_carla_log.sh                      # 拷贝最新的"
  echo "     bash tools/copy_carla_log.sh 20260626180257       # 拷贝指定时间戳的"
  echo "     目标目录：data/carla_log/"
  echo
  echo "  2. 配合 start_collect.sh 使用（SCB 必选，碰撞日志可选）："
  echo "     bash tools/copy_carla_log.sh --session <SESSION_ID> --start <START_TS> --end <END_TS>"
  echo "     目标目录：data/scb_data/<SESSION_ID>/log/"
  echo
  echo "时间格式：14 位 YYYYMMDDHHMMSS"
  echo
  exit 1
}

# ══════════════════════════════════════════════════════════════
#  检查与 Carla 主机的连接是否正常
# ══════════════════════════════════════════════════════════════
check_carla_connection() {
  if ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 -o BatchMode=yes "${CARLA_HOST}" "cd ${CARLA_LOG_DIR} && true" 2>/dev/null; then
    return 0
  else
    return 1
  fi
}

# ══════════════════════════════════════════════════════════════
#  获取 SCB 文件。每行格式：最后修改时间（epoch） 文件名
# ══════════════════════════════════════════════════════════════
get_scb_files_by_mtime() {
  # 先检查连接
  if ! check_carla_connection; then
    log_warn "无法连接到 Carla 主机 (${CARLA_HOST})"
    echo ""
    return
  fi
  ssh -o StrictHostKeyChecking=no "${CARLA_HOST}" "
    cd ${CARLA_LOG_DIR} &&
    for f in scb_control_delay_*.csv; do
      if [[ -f \"\$f\" ]]; then
        stat -c '%Y %n' \"\$f\"
      fi
    done
  " | sort -nr
}

timestamp_to_epoch() {
  local ts="$1"
  date -d "${ts:0:4}-${ts:4:2}-${ts:6:2} ${ts:8:2}:${ts:10:2}:${ts:12:2}" +%s
}

find_scb_for_run() {
  local start_ts="$1"
  local end_ts="$2"
  local file_start_ts
  local best_start_ts=""
  local best_name=""
  local mtime
  local name

  # SCB 默认只记录首次有效制动。第一次触发后文件可能不再更新，因此不能
  # 要求 mtime 位于本次采集窗口。按文件名中的 Bridge 启动时间选择结束时间
  # 之前最新的一份，整个 Bridge 生命周期内可重复拷贝同一个文件。
  while read -r mtime name; do
    if [[ "$name" =~ ^scb_control_delay_([0-9]{14})(_.*)?\.csv$ ]]; then
      file_start_ts="${BASH_REMATCH[1]}"
    else
      continue
    fi

    if [[ "$file_start_ts" -le "$end_ts" &&
          ( -z "$best_start_ts" || "$file_start_ts" > "$best_start_ts" ) ]]; then
      best_start_ts="$file_start_ts"
      best_name="$name"
    fi
  done < <(get_scb_files_by_mtime)

  echo "$best_name"
}

find_scb_by_start_timestamp() {
  local target_ts="$1"
  local mtime
  local name

  while read -r mtime name; do
    if [[ "$name" == "scb_control_delay_${target_ts}.csv" ||
          "$name" == "scb_control_delay_${target_ts}_"*.csv ]]; then
      echo "$name"
      return 0
    fi
  done < <(get_scb_files_by_mtime)

  echo ""
}

get_collision_timestamps() {
  ssh -o StrictHostKeyChecking=no "${CARLA_HOST}" \
    "cd ${CARLA_LOG_DIR} && (ls -1 carla_collision_events_*.jsonl 2>/dev/null || true)" | \
    sed -n 's/^carla_collision_events_\([0-9]\{14\}\)\.jsonl$/\1/p' | \
    sort
}

find_collision_in_window() {
  local start_ts="$1"
  local end_ts="$2"
  local found_ts=""
  local ts

  while read -r ts; do
    if [[ -n "$ts" && "$ts" -ge "$start_ts" && "$ts" -le "$end_ts" ]]; then
      found_ts="$ts"
    fi
  done < <(get_collision_timestamps)

  echo "$found_ts"
}

copy_run_files() {
  local scb_file="$1"
  local collision_ts="$2"
  local dst_dir="$3"

  mkdir -p "$dst_dir"

  if ! ssh -o StrictHostKeyChecking=no "${CARLA_HOST}" "cd ${CARLA_LOG_DIR} && test -f '${scb_file}'"; then
    log_error "Carla 主机上未找到 SCB 时延文件：${scb_file}"
    return 1
  fi

  local sources=("${CARLA_HOST}:${CARLA_LOG_DIR}/${scb_file}")
  local copied_count=1

  if [[ -n "$collision_ts" ]]; then
    local jsonl_file="carla_collision_events_${collision_ts}.jsonl"
    local csv_file="carla_collision_events_${collision_ts}.csv"
    local actor_history_file="carla_collision_actor_history_${collision_ts}.csv"

    if ssh -o StrictHostKeyChecking=no "${CARLA_HOST}" "
      cd ${CARLA_LOG_DIR} &&
      test -f '${jsonl_file}' &&
      test -f '${csv_file}' &&
      test -f '${actor_history_file}'
    "; then
      sources+=(
        "${CARLA_HOST}:${CARLA_LOG_DIR}/${jsonl_file}"
        "${CARLA_HOST}:${CARLA_LOG_DIR}/${csv_file}"
        "${CARLA_HOST}:${CARLA_LOG_DIR}/${actor_history_file}"
      )
      copied_count=4
      log_info "检测到碰撞 ${collision_ts}，同步拷贝三个碰撞日志文件"
    else
      log_warn "碰撞 ${collision_ts} 的日志组不完整，本次仅拷贝 SCB 时延文件"
    fi
  else
    log_info "本次采集未检测到碰撞，仅拷贝 SCB 时延文件"
  fi

  log_step "从 ${CARLA_HOST}:${CARLA_LOG_DIR}/ 拷贝"
  scp -o StrictHostKeyChecking=no "${sources[@]}" "${dst_dir}/"

  log_info "成功拷贝 ${copied_count} 个 Carla/SCB 文件到：${dst_dir}/"
}

# ══════════════════════════════════════════════════════════════
#  主程序
# ══════════════════════════════════════════════════════════════

# 模式 1：配合 start_collect.sh 使用（带 --session --start --end 参数）
if [[ "$#" -eq 6 && "$1" == "--session" && "$3" == "--start" && "$5" == "--end" ]]; then
  SESSION_ID="$2"
  START_TS="$4"
  END_TS="$6"

  log_step "SCB 控制时延文件收集"
  log_info "会话：${SESSION_ID}"
  log_info "采集时间窗口：${START_TS} → ${END_TS}"

  # 先检查连接是否正常
  if ! check_carla_connection; then
    log_warn "无法连接到 Carla 主机 (${CARLA_HOST})，跳过 SCB 时延文件采集"
    log_warn "（这不影响 Apollo 日志采集）"
    exit 0
  fi

  DST_DIR="${SCB_DATA_BASE}/${SESSION_ID}/log"

  # 选择采集结束前启动时间最新的一份 SCB 文件。即使同一 Bridge 中首次
  # 制动后文件不再更新，后续采集仍会把这一份文件拷入各自会话目录。
  SCB_FILE="$(find_scb_for_run "$START_TS" "$END_TS")"

  if [[ -z "$SCB_FILE" ]]; then
    log_warn "未找到采集结束时间之前生成的 scb_control_delay_*.csv"
    exit 0
  fi

  log_info "本次运行的 SCB 时延文件：${SCB_FILE}"
  COLLISION_TS="$(find_collision_in_window "$START_TS" "$END_TS")"
  if [[ -n "$COLLISION_TS" ]]; then
    log_info "本次采集匹配到碰撞时间戳：${COLLISION_TS}"
  fi

  if copy_run_files "$SCB_FILE" "$COLLISION_TS" "$DST_DIR"; then
    log_info "Carla/SCB 文件已成功采集"
  fi
  exit 0
fi

# 模式 2：单独运行
if [[ "$#" -eq 0 ]]; then
  log_step "模式：单独运行（拷贝最新的 SCB 时延文件）"

  if ! check_carla_connection; then
    log_error "无法连接到 Carla 主机 (${CARLA_HOST})"
    exit 1
  fi

  LATEST_SCB_ENTRY="$(get_scb_files_by_mtime | head -1)"

  if [[ -z "$LATEST_SCB_ENTRY" ]]; then
    log_warn "在 Carla 主机上未找到任何 scb_control_delay_*.csv"
    exit 0
  fi

  read -r LATEST_SCB_MTIME LATEST_SCB_FILE <<< "$LATEST_SCB_ENTRY"
  log_info "最新 SCB 时延文件：${LATEST_SCB_FILE}"
  DST_DIR="${REPO_ROOT}/data/carla_log"
  copy_run_files "$LATEST_SCB_FILE" "" "$DST_DIR"
  exit 0
elif [[ "$#" -eq 1 && "$1" =~ ^[0-9]{14}$ ]]; then
  log_step "模式：单独运行（按指定时间戳匹配 SCB 时延文件）"

  if ! check_carla_connection; then
    log_error "无法连接到 Carla 主机 (${CARLA_HOST})"
    exit 1
  fi

  TARGET_TS="$1"
  DST_DIR="${REPO_ROOT}/data/carla_log"

  SCB_FILE="$(find_scb_by_start_timestamp "$TARGET_TS")"
  if [[ -z "$SCB_FILE" ]]; then
    log_error "未找到启动时间戳为 ${TARGET_TS} 的 scb_control_delay_*.csv"
    exit 1
  fi

  copy_run_files "$SCB_FILE" "" "$DST_DIR"
  exit 0
fi

usage
