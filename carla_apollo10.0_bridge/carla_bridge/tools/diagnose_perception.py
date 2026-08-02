#!/usr/bin/env python3
"""
感知诊断工具 —— 全量记录 + 多维度分析

记录所有帧的所有障碍物信息到日志文件, 用于判断:
  1. 检测丢失: obstacle 的 frame_id 是否不连续 (track被杀重建)
  2. 分类跳变: type 是否在 VEHICLE/BICYCLE/UNKNOWN 间波动
  3. 框变形:   box 尺寸是否帧间抖动
  4. 位置跳变: 位置是否帧间突变

用法（在 Apollo 容器内）:
  python3 modules/carla_apollo10.0_bridge/carla_bridge/tools/diagnose_perception.py

日志输出在同目录:  diagnose_perception_YYYYMMDD_HHMMSS.log
运行中按 Ctrl+C 停止, 日志末尾自动追加诊断结论。
"""

import sys
import signal
import math
import os
import time
from datetime import datetime
from collections import defaultdict

# 容器内 activate.sh 已配置好 PYTHONPATH, 直接 import
try:
    import cyber_py3.cyber as cyber
    from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles
except ImportError as e:
    print(f"[ERROR] 导入失败: {e}")
    print("请确认已在 Apollo 容器内, 并执行过: source /apollo_workspace/scripts/activate.sh")
    sys.exit(1)

# ============================================================
# 可调参数
# ============================================================
BOX_DEFORM_THRESHOLD = 0.2        # box 尺寸变化 >20% 记为变形事件
POS_JUMP_THRESHOLD = 3.0          # 位置跳变 >3m 记为异常
EGO_RADIUS = 3.0                  # 距离原点 <3m 视为自车, 过滤掉
MAX_OBSERVE_SECONDS = 600         # 最长运行时间(10分钟)
# ============================================================

TYPE_NAMES = {
    0: "UNKNOWN", 1: "UNKNOWN_MOVABLE", 2: "UNKNOWN_UNMOVABLE",
    3: "PEDESTRIAN", 4: "BICYCLE", 5: "VEHICLE",
}

SUB_TYPE_NAMES = {
    0: "ST_UNKNOWN", 1: "ST_UNKNOWN_MOVABLE", 2: "ST_UNKNOWN_UNMOVABLE",
    3: "ST_CAR", 4: "ST_VAN", 5: "ST_TRUCK", 6: "ST_BUS",
    7: "ST_CYCLIST", 8: "ST_MOTORCYCLIST", 9: "ST_TRICYCLIST",
    10: "ST_PEDESTRIAN", 11: "ST_TRAFFICCONE", 12: "ST_SMALLMOT",
    13: "ST_BIGMOT", 14: "ST_NONMOT",
}

LOG_DIR = os.path.dirname(os.path.abspath(__file__))
TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = os.path.join(LOG_DIR, f"diagnose_perception_{TIMESTAMP}.log")


def velocity_magnitude(vel):
    """从 Point3D 提取速度大小"""
    try:
        return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
    except Exception:
        return -1


class Diagnoser:
    def __init__(self):
        self.frame = 0
        self.tracks = {}       # {oid: [record_dict]}
        self.ego_pose = None   # 最近一次自车位置 (用于过滤自车)
        self.running = True
        self.log_fp = None
        self.start_time = time.time()
        signal.signal(signal.SIGINT, self._stop)

    def _stop(self, *args):
        self.running = False

    def log(self, msg, also_print=True):
        if self.log_fp:
            self.log_fp.write(msg + "\n")
            self.log_fp.flush()
        if also_print:
            print(msg)

    def _is_ego(self, x, y):
        """粗略判断是否可能是自车 (位置在原点附近)"""
        if self.ego_pose is None:
            return False
        return math.hypot(x - self.ego_pose[0], y - self.ego_pose[1]) < EGO_RADIUS

    def on_obstacles(self, data):
        if not self.running:
            return
        self.frame += 1
        ts = data.header.timestamp_sec if data.header.HasField("timestamp_sec") else 0
        obs_count = len(data.perception_obstacle)

        # 更新自车位置: 取第一个最近的物体作为自车估计位置
        # (或者从 localization 获取, 但这里简化处理)
        ego_updated = False

        self.log(f"[FRAME {self.frame:>4}] ts={ts:.3f} count={obs_count}", also_print=False)

        for obj in data.perception_obstacle:
            oid = obj.id
            x, y, z = obj.position.x, obj.position.y, obj.position.z

            # 如果是自车, 更新自车位置
            if oid == 0 or obj.source == 0:  # source=HOST_VEHICLE
                if not ego_updated:
                    self.ego_pose = (x, y)
                    ego_updated = True

            # ---- 过滤自车 ----
            if self._is_ego(x, y):
                self.log(f"  [FILTERED] ID={oid:<4} pos=({x:.2f},{y:.2f}) ← 过滤(可能是自车)", also_print=False)
                continue

            # ---- 提取字段 ----
            otype = TYPE_NAMES.get(obj.type, f"TYPE_{obj.type}")
            sub_type = SUB_TYPE_NAMES.get(obj.sub_type, f"SUB_{obj.sub_type}") if obj.HasField("sub_type") else "-"

            length = obj.length if obj.HasField("length") else 0
            width = obj.width if obj.HasField("width") else 0
            height = obj.height if obj.HasField("height") else 0
            theta = obj.theta
            vel_mag = velocity_magnitude(obj.velocity)
            confidence = obj.confidence if obj.HasField("confidence") else -1
            tracking_time = obj.tracking_time if obj.HasField("tracking_time") else -1

            record = {
                "frame": self.frame,
                "type": otype,
                "sub_type": sub_type,
                "x": round(x, 2),
                "y": round(y, 2),
                "z": round(z, 2),
                "length": round(length, 2),
                "width": round(width, 2),
                "height": round(height, 2),
                "theta": round(theta, 4),
                "vel": round(vel_mag, 2),
                "conf": round(confidence, 4) if confidence >= 0 else -1,
                "tracking_time": round(tracking_time, 3) if tracking_time >= 0 else -1,
            }

            # ---- track 历史 ----
            if oid not in self.tracks:
                self.tracks[oid] = []
                self.tracks[oid].append(record)
                event_str = "[NEW]"
            else:
                prev = self.tracks[oid][-1]
                self.tracks[oid].append(record)
                event_str = self._compare(prev, record)

            # ---- 写日志 ----
            log_line = (
                f"  ID={oid:<4} | {otype:<10} | sub={sub_type:<12} "
                f"pos=({x:>7.2f},{y:>7.2f},{z:>5.2f}) "
                f"box=({length:>5.2f},{width:>5.2f},{height:>5.2f}) "
                f"theta={theta:>6.2f} vel={vel_mag:>5.2f} "
                f"conf={confidence if confidence>=0 else -1:>5.2f} "
                f"track_t={tracking_time:.2f}"
            )
            if event_str:
                log_line += f" | {event_str}"
            self.log(log_line, also_print=(event_str != "[NEW]" and event_str != ""))

    def _compare(self, prev, cur):
        events = []

        # 帧间隔 (检测丢失)
        gap = cur["frame"] - prev["frame"] - 1
        if gap > 0:
            events.append(f"GAP={gap}")

        # 类别跳变
        if prev["type"] != cur["type"]:
            events.append(f"TYPE:{prev['type']}→{cur['type']}")

        # sub_type 跳变
        if prev["sub_type"] != cur["sub_type"] and "TYPE:" in str(events):
            events.append(f"SUB:{prev['sub_type']}→{cur['sub_type']}")

        # 位置跳变
        dx = cur["x"] - prev["x"]
        dy = cur["y"] - prev["y"]
        dist = math.hypot(dx, dy)
        if dist > POS_JUMP_THRESHOLD:
            events.append(f"POS_JUMP:{dist:.2f}m")

        # box 尺寸变形
        lr = abs(cur["length"] - prev["length"]) / max(prev["length"], 0.01) if prev["length"] > 0 else 0
        wr = abs(cur["width"] - prev["width"]) / max(prev["width"], 0.01) if prev["width"] > 0 else 0
        hr = abs(cur["height"] - prev["height"]) / max(prev["height"], 0.01) if prev["height"] > 0 else 0
        if lr > BOX_DEFORM_THRESHOLD or wr > BOX_DEFORM_THRESHOLD or hr > BOX_DEFORM_THRESHOLD:
            events.append(
                f"BOX_DEFORM L{prev['length']:.1f}→{cur['length']:.1f} "
                f"W{prev['width']:.1f}→{cur['width']:.1f} "
                f"H{prev['height']:.1f}→{cur['height']:.1f}"
            )

        # tracking_time 重置 (track 被杀重建)
        if cur["tracking_time"] >= 0 and prev["tracking_time"] >= 0:
            if cur["tracking_time"] < prev["tracking_time"]:
                events.append("TRACK_RESET")
            elif gap == 0 and cur["tracking_time"] == prev["tracking_time"]:
                events.append("TRACK_STALL(tracking_time未增长)")

        return " | ".join(events)

    def generate_report(self):
        lines = []
        lines.append("")
        lines.append("=" * 90)
        lines.append(f"诊 断 汇 总  ({TIMESTAMP})")
        lines.append("=" * 90)

        if not self.tracks:
            lines.append("⚠ 未收到任何障碍物数据")
        else:
            elapsed = time.time() - self.start_time
            total_frames = self.frame
            lines.append(f"运行时间:       {elapsed:.0f}秒 ({elapsed/60:.1f}分钟)")
            lines.append(f"总帧数:         {total_frames}")
            lines.append(f"平均帧率:       {total_frames/max(elapsed,1):.2f} Hz")
            lines.append(f"障碍物ID总数:   {len(self.tracks)}")
            lines.append(f"阈值说明:")
            lines.append(f"  BOX_DEFORM阈值: 尺寸变化 >{BOX_DEFORM_THRESHOLD*100:.0f}%")
            lines.append(f"  POS_JUMP阈值:   位置跳变 >{POS_JUMP_THRESHOLD:.0f}m")
            lines.append(f"  EGO过滤半径:    距原点 <{EGO_RADIUS:.0f}m 过滤")
            lines.append("")

            # ---- 多维度统计 ----
            stats = {
                "type_jump": {"cnt": 0, "ids": []},
                "track_loss": {"cnt": 0, "ids": []},
                "box_deform": {"cnt": 0, "ids": []},
                "pos_jump": {"cnt": 0, "ids": []},
                "track_reset": {"cnt": 0, "ids": []},
                "stable": 0,
            }

            # box 抖动量化: 记录每个 ID 的变异系数
            box_cv = {}  # {oid: {"length_cv": float, "width_cv": float}}

            for oid, hist in self.tracks.items():
                if len(hist) < 2:
                    stats["stable"] += 1
                    continue

                flags = set()
                lengths = [r["length"] for r in hist if r["length"] > 0]
                widths = [r["width"] for r in hist if r["width"] > 0]

                # 变异系数 (CV = std/mean, 衡量抖动幅度)
                if len(lengths) > 2:
                    mean_l = sum(lengths) / len(lengths)
                    std_l = math.sqrt(sum((v-mean_l)**2 for v in lengths) / len(lengths))
                    cv_l = std_l / max(mean_l, 0.01)
                else:
                    cv_l = 0
                if len(widths) > 2:
                    mean_w = sum(widths) / len(widths)
                    std_w = math.sqrt(sum((v-mean_w)**2 for v in widths) / len(widths))
                    cv_w = std_w / max(mean_w, 0.01)
                else:
                    cv_w = 0
                box_cv[oid] = {"length_cv": round(cv_l, 4), "width_cv": round(cv_w, 4)}

                for i in range(1, len(hist)):
                    prev, cur = hist[i-1], hist[i]
                    if prev["type"] != cur["type"]:
                        flags.add("type_jump")
                    gap = cur["frame"] - prev["frame"] - 1
                    if gap > 0:
                        flags.add("track_loss")
                    lr = abs(cur["length"]-prev["length"])/max(prev["length"], 0.01) if prev["length"]>0 else 0
                    wr = abs(cur["width"]-prev["width"])/max(prev["width"], 0.01) if prev["width"]>0 else 0
                    if lr > BOX_DEFORM_THRESHOLD or wr > BOX_DEFORM_THRESHOLD:
                        flags.add("box_deform")
                    if math.hypot(cur["x"]-prev["x"], cur["y"]-prev["y"]) > POS_JUMP_THRESHOLD:
                        flags.add("pos_jump")
                    if cur["tracking_time"] >= 0 and prev["tracking_time"] >= 0:
                        if cur["tracking_time"] < prev["tracking_time"]:
                            flags.add("track_reset")

                if "type_jump" in flags:
                    stats["type_jump"]["cnt"] += 1
                    stats["type_jump"]["ids"].append(oid)
                if "track_loss" in flags:
                    stats["track_loss"]["cnt"] += 1
                    stats["track_loss"]["ids"].append(oid)
                if "box_deform" in flags:
                    stats["box_deform"]["cnt"] += 1
                    stats["box_deform"]["ids"].append(oid)
                if "pos_jump" in flags:
                    stats["pos_jump"]["cnt"] += 1
                    stats["pos_jump"]["ids"].append(oid)
                if "track_reset" in flags:
                    stats["track_reset"]["cnt"] += 1
                    stats["track_reset"]["ids"].append(oid)
                if not flags:
                    stats["stable"] += 1

            # ---- 汇总输出 ----
            t = len(self.tracks)
            lines.append(f"  {'统计项':<20} {'数量':>6} {'占比':>6}  {'涉及ID'}")
            lines.append(f"  {'─'*60}")
            lines.append(f"  {'稳定跟踪':<20} {stats['stable']:>6} {stats['stable']*100//max(t,1):>5}%")
            lines.append(f"  {'分类跳变(type)':<20} {stats['type_jump']['cnt']:>6} {stats['type_jump']['cnt']*100//max(t,1):>5}%  {stats['type_jump']['ids']}")
            lines.append(f"  {'检测丢失(track被杀)':<20} {stats['track_loss']['cnt']:>6} {stats['track_loss']['cnt']*100//max(t,1):>5}%  {stats['track_loss']['ids']}")
            lines.append(f"  {'框变形(box抖动)':<20} {stats['box_deform']['cnt']:>6} {stats['box_deform']['cnt']*100//max(t,1):>5}%  {stats['box_deform']['ids']}")
            lines.append(f"  {'位置跳变':<20} {stats['pos_jump']['cnt']:>6} {stats['pos_jump']['cnt']*100//max(t,1):>5}%  {stats['pos_jump']['ids']}")
            lines.append(f"  {'tracking_time重置':<20} {stats['track_reset']['cnt']:>6} {stats['track_reset']['cnt']*100//max(t,1):>5}%  {stats['track_reset']['ids']}")

            lines.append("")
            lines.append("─" * 60)
            lines.append("框抖动量化 (变异系数CV = std/mean, 越大表示帧间抖动越剧烈):")
            lines.append(f"  {'ID':<6} {'Length_CV':<12} {'Width_CV':<12} {'L均值':<10} {'W均值':<10} {'帧数':<6}")
            lines.append(f"  {'─'*56}")
            # 按 length_cv 排序, 展示最抖的
            sorted_by_cv = sorted(box_cv.items(), key=lambda kv: kv[1]["length_cv"] + kv[1]["width_cv"], reverse=True)
            for oid, cv in sorted_by_cv[:10]:
                hist = self.tracks[oid]
                mean_l = sum(r["length"] for r in hist if r["length"]>0) / max(sum(1 for r in hist if r["length"]>0), 1)
                mean_w = sum(r["width"] for r in hist if r["width"]>0) / max(sum(1 for r in hist if r["width"]>0), 1)
                lines.append(f"  ID{oid:<4} {cv['length_cv']:<12.4f} {cv['width_cv']:<12.4f} {mean_l:<10.2f} {mean_w:<10.2f} {len(hist):<6}")

            # ---- 主要问题判定 ----
            lines.append("")
            lines.append("─" * 60)
            scores = {
                "检测丢失(track被杀)": stats["track_loss"]["cnt"],
                "分类跳变(VEHICLE/BICYCLE)": stats["type_jump"]["cnt"],
                "框回归不稳定(box+pos抖动)": stats["box_deform"]["cnt"] + stats["pos_jump"]["cnt"],
            }
            primary = max(scores, key=scores.get)
            primary_cnt = scores[primary]

            if primary_cnt == 0:
                lines.append("结论: ✅ 感知基本稳定, 未发现明显异常")
            else:
                lines.append(f"结论: 🔴 主要问题是「{primary}」(影响{primary_cnt}/{t}个障碍物)")
                if "检测丢失" in primary:
                    lines.append("  原因推测: 点云稀疏 → 检测漏帧 → track 被杀重建")
                    lines.append("  建议:")
                    lines.append("    1) objects.json 增加 points_per_second (当前80000)")
                    lines.append("    2) mlf_engine.pb.txt 加大 reserved_invisible_time (当前0.3)")
                    lines.append("    3) center_point_param.pb.txt 降低 min_points_threshold (3→2)")
                elif "分类跳变" in primary:
                    lines.append("  原因推测: CenterPoint 分类头在决策边界附近波动")
                    lines.append("  建议:")
                    lines.append("    1) 检查 type_classifiers.property 混淆矩阵")
                    lines.append("    2) 加大 temporal_window 增加时间平滑")
                    lines.append("    3) 检查 sub_type 是否有类似的跳变")
                elif "框回归" in primary:
                    lines.append("  原因推测: 点云帧间特征不稳定, box 回归头输出抖动")
                    lines.append("  建议:")
                    lines.append("    1) objects.json 增加 points_per_second 提高密度")
                    lines.append("    2) 确认 enable_fuse_frames 是否已打开")
                    lines.append("    3) 检查 LiDAR 的 yaw 安装角与标定文件是否一致")

            # ---- 每个 ID 的完整序列 ----
            lines.append("")
            lines.append("=" * 90)
            lines.append("完整跟踪序列 (按ID排序)")
            lines.append("=" * 90)
            for oid in sorted(self.tracks.keys()):
                hist = self.tracks[oid]
                types_seen = []
                for r in hist:
                    if not types_seen or types_seen[-1] != r["type"]:
                        types_seen.append(r["type"])
                lines.append(f"\nID {oid}: {len(hist)}帧, 类别变化: {'→'.join(types_seen)}")
                for r in hist:
                    lines.append(
                        f"  帧{r['frame']:>4} | {r['type']:<10} sub={r['sub_type']:<12}"
                        f" ({r['x']:>7.2f},{r['y']:>7.2f},{r['z']:>5.2f})"
                        f" box=({r['length']:>5.2f},{r['width']:>5.2f},{r['height']:>5.2f})"
                        f" θ={r['theta']:>5.2f} v={r['vel']:.2f} c={r['conf']:.2f} tt={r['tracking_time']:.2f}"
                    )

        for line in lines:
            self.log(line)

    def run(self):
        self.log_fp = open(LOG_FILE, "w", encoding="utf-8")
        self.log(f"感知诊断日志 - {TIMESTAMP}")
        self.log(f"话题: /apollo/perception/obstacles")
        self.log(f"阈值: BOX_DEFORM>{BOX_DEFORM_THRESHOLD*100:.0f}% | POS_JUMP>{POS_JUMP_THRESHOLD:.0f}m | EGO_RADIUS<{EGO_RADIUS:.0f}m")
        self.log("")

        cyber.init("perception_diagnose")
        node = cyber.Node("perception_diagnose")
        node.create_reader("/apollo/perception/obstacles", PerceptionObstacles, self.on_obstacles)

        print(f"🚀 诊断启动")
        print(f"   日志: {LOG_FILE}")
        print(f"   按 Ctrl+C 停止并输出报告")
        print(f"")

        deadline = time.time() + MAX_OBSERVE_SECONDS
        while self.running and time.time() < deadline and not cyber.is_shutdown():
            cyber.spin(node, duration=0.1)

        cyber.shutdown()
        self.generate_report()
        if self.log_fp:
            self.log_fp.close()
        print(f"\n✅ 诊断完成, 日志: {LOG_FILE}")
        print(f"   请将此文件发回分析")


if __name__ == "__main__":
    d = Diagnoser()
    d.run()
