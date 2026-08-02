#!/usr/bin/env python3
"""
P3 单机全量数据采集脚本

设计：在 P3 上一次性采集全部所需的三层数据，无需帧偏移对齐。

  1. CARLA Ground Truth（P3 本地 CARLA，只读连接）
  2. 原始点云（P3 本地 CyberRT，Bridge 已发布的补偿后点云）
  3. Apollo 感知障碍物 + 定位位姿（Orin 端 CyberRT，跨机器订阅）

数据流:
  Bridge tick CARLA → Bridge 发点云到本地 CyberRT
  → Orin 收到点云 → CenterPoint 推理 → 发感知结果回 P3
  → 本脚本订阅所有数据，同一帧写入

用法（P3 上，需要 CyberRT + CARLA Python API 环境）:
  cd /apollo_workspace
  python3 modules/carla_apollo10.0_bridge/carla_bridge/tools/collect_p3_all.py

日志输出:  collect_p3_all_YYYYMMDD_HHMMSS.log
"""

import sys
import signal
import math
import time
import os
import threading
import statistics
from datetime import datetime
from collections import defaultdict

# ============================================================
# 路径配置（P3 上的 Bridge 环境）
# ============================================================
neo_paths = [
    "/opt/apollo/neo/lib/cyber/python/internal",
    "/opt/apollo/neo/lib/cyber/python/cyber/python",
    "/opt/apollo/neo/lib/cyber/python",
    "/opt/apollo/neo/python/",              # ← modules.common_msgs 所在路径
]
for p in neo_paths:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    import cyber_py3.cyber as cyber
    from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles
    from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointCloud
    from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
except ImportError as e:
    print(f"[ERROR] 导入 cyber 失败: {e}")
    print("请确认已在有 CyberRT 环境（如 Bridge 运行环境）下运行")
    sys.exit(1)

try:
    import carla
    HAS_CARLA = True
except ImportError:
    HAS_CARLA = False
    print("[WARN] CARLA Python API 不可用，将无法采集 Ground Truth")

# ============================================================
# 常量
# ============================================================
LIDAR_TOPIC = "/apollo/sensor/velodyne64/compensator/PointCloud2"
OBSTACLE_TOPIC = "/apollo/perception/obstacles"
LOCALIZATION_TOPIC = "/apollo/localization/pose"

# 外参文件路径（从脚本位置推算代码仓库根目录）
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
EXTRINSICS_PATH = os.path.join(
    REPO_ROOT,
    "modules/drivers/lidar/velodyne/params/velodyne64_novatel_extrinsics.yaml",
)
# 如果上面路径不对，试一下 Orin 容器内的绝对路径作为 fallback
if not os.path.exists(EXTRINSICS_PATH):
    EXTRINSICS_PATH = "/apollo_workspace/modules/drivers/lidar/velodyne/params/velodyne64_novatel_extrinsics.yaml"

LOG_DIR = SCRIPT_DIR
TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = os.path.join(LOG_DIR, f"collect_p3_all_{TIMESTAMP}.log")

# Apollo 障碍物类型名称（参考 modules/common_msgs/basic_msgs/geometry.proto）
TYPE_NAMES = {
    0: "UNKNOWN",
    1: "UNKNOWN_MOVABLE",
    2: "UNKNOWN_UNMOVABLE",
    3: "PEDESTRIAN",
    4: "BICYCLE",
    5: "VEHICLE",
}

# ============================================================
# 工具函数
# ============================================================
def quaternion_to_matrix(q):
    """四元数 → 3×3 旋转矩阵（右手系，Hamilton 约定）"""
    qx, qy, qz, qw = q
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]


def quat_to_yaw(q):
    """从四元数提取偏航角（弧度）"""
    qx, qy, qz, qw = q
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def read_extrinsics(path):
    """读外参 YAML，返回 (trans, quat)"""
    trans = [0.0, 0.0, 2.4]  # 默认 LiDAR 安装高度 2.4m
    quat = [0.0, 0.0, 0.0, 1.0]  # 默认无旋转
    section = None
    try:
        with open(path) as f:
            for line in f:
                s = line.strip()
                if "translation:" in s:
                    section = "trans"
                    continue
                if "rotation:" in s:
                    section = "rot"
                    continue
                if "child_frame" in s:
                    break
                if section == "trans":
                    if s.startswith("x:"):
                        trans[0] = float(s.split(":")[1])
                    elif s.startswith("y:"):
                        trans[1] = float(s.split(":")[1])
                    elif s.startswith("z:"):
                        trans[2] = float(s.split(":")[1])
                elif section == "rot":
                    if s.startswith("x:"):
                        quat[0] = float(s.split(":")[1])
                    elif s.startswith("y:"):
                        quat[1] = float(s.split(":")[1])
                    elif s.startswith("z:"):
                        quat[2] = float(s.split(":")[1])
                    elif s.startswith("w:"):
                        quat[3] = float(s.split(":")[1])
    except Exception as e:
        print(f"[WARN] 外参读取失败: {e}")
    return trans, quat


def point_in_box(px, py, pz, cx, cy, cz, half_l, half_w, half_h, cos_yaw, sin_yaw):
    """
    判断点 (px,py,pz) 是否在 3D 有向 bbox 内。
    bbox 中心 (cx,cy,cz)，半长 half_l，半宽 half_w，半高 half_h。
    朝向由 (cos_yaw, sin_yaw) 决定（yaw 是 Apollo 坐标系下的弧度）。

    旋转公式推导（从世界坐标到 bbox 局部坐标）：
      将点平移到 bbox 中心，再旋转 -yaw：
        lx =  dx * cos(yaw) + dy * sin(yaw)
        ly = -dx * sin(yaw) + dy * cos(yaw)
    """
    dx = px - cx
    dy = py - cy
    dz = pz - cz
    lx = dx * cos_yaw + dy * sin_yaw
    ly = -dx * sin_yaw + dy * cos_yaw
    return abs(lx) <= half_l and abs(ly) <= half_w and abs(dz) <= half_h


def carla_to_apollo(x, y, z, yaw_deg):
    """CARLA 坐标（左手系）→ Apollo 坐标（右手系 ENU）

    变换规则：
      x_apollo = x_carla
      y_apollo = -y_carla      （y 取反，左手→右手）
      z_apollo = z_carla
      yaw_apollo = -radians(yaw_carla)  （方向取反并转弧度）
    """
    return (x, -y, z, -math.radians(yaw_deg))


# ============================================================
# 读取外参
# ============================================================
extr_t, extr_q = read_extrinsics(EXTRINSICS_PATH)
extr_R = quaternion_to_matrix(extr_q)
print(f"[INFO] 外参 trans=({extr_t[0]:.3f}, {extr_t[1]:.3f}, {extr_t[2]:.3f})")
print(f"[INFO] 外参 quat=({extr_q[0]:.4f}, {extr_q[1]:.4f}, {extr_q[2]:.4f}, {extr_q[3]:.4f})")
# 注：trans.z=2.4 表示 LiDAR 在车顶 2.4m 处。如果点云世界坐标 Z 偏低，需要检查此值。
# 注：quat 如果是 (0,0,0,1) 恒等旋转，说明 LiDAR 坐标系与车辆坐标系对齐。


# ============================================================
# 主采集类
# ============================================================
class DataCollector:
    def __init__(self):
        self.running = True
        self.frame_no = 0  # 总帧计数器（由感知回调递增）
        self.gt_frame_no = 0  # CARLA GT 帧计数器
        self.log_fp = None
        self.start_time = time.time()

        # ---- CARLA 连接（P3 本地 127.0.0.1） ----
        self.carla_host = "127.0.0.1"
        self.carla_port = 2000

        # ---- 线程锁 ----
        self.lock = threading.Lock()

        # ---- CyberRT 异步数据缓存 ----
        self.latest_pc_ts = 0.0
        self.latest_lidar_pts = []  # 最新点云（LiDAR 坐标系，已 y 取反）
        self.latest_world_pts = []  # 最新点云（世界坐标系，由回调中变换）

        self.latest_ego_x = 0.0
        self.latest_ego_y = 0.0
        self.latest_ego_z = 0.0
        self.latest_ego_yaw = 0.0
        self.latest_ego_quat = (0.0, 0.0, 0.0, 1.0)
        self.latest_loc_ts = 0.0      # 定位时间戳（由 on_localization 更新）
        self.ego_pose_ready = False

        # ---- 帧间隔跟踪（用于检测点云帧率异常） ----
        self.prev_pc_ts = 0.0         # 上一帧点云时间戳
        self.pc_frame_dt = 0.0        # 当前帧与上一帧的间隔（秒）

        # ---- CARLA GT（由 CARLA 线程更新） ----
        self.carla_ego_gt = None
        self.carla_npcs = {}
        self.carla_connected = False

        # ---- 逐 NPC 累计统计 ----
        self.npc_stats = defaultdict(lambda: {
            "frames": 0,
            "detected": 0,
            "missed": 0,
            "missed_with_pts": 0,
            "missed_no_pts": 0,
            "type_ok": 0,
            "type_bad": 0,
            "confs": [],
            "pts_on_npc_list": [],
            "tracking_times": [],
        })

        signal.signal(signal.SIGINT, lambda *a: setattr(self, "running", False))

    # ----------------------------------------------------------------
    # 日志写入
    # ----------------------------------------------------------------
    def log(self, msg):
        """写日志，立即 flush 确保不丢数据"""
        if self.log_fp:
            self.log_fp.write(msg + "\n")
            self.log_fp.flush()

    # ----------------------------------------------------------------
    # CyberRT 回调
    # ----------------------------------------------------------------
    def on_pointcloud(self, data):
        """点云回调（来自 P3 本地 Bridge，已发布到 CyberRT）"""
        if not self.running:
            return
        ts = data.header.timestamp_sec if data.header.HasField("timestamp_sec") else 0
        pts = [(p.x, p.y, p.z, p.intensity) for p in data.point]
        with self.lock:
            # 帧间隔计算：用于检测点云帧率是否异常（应稳定在 100ms 左右）
            if self.prev_pc_ts > 0:
                self.pc_frame_dt = ts - self.prev_pc_ts
            self.prev_pc_ts = ts
            self.latest_pc_ts = ts
            self.latest_lidar_pts = pts
            world_pts = []
            if self.ego_pose_ready and len(pts) > 0:
                ego_R = quaternion_to_matrix(self.latest_ego_quat)
                for p in pts:
                    # LiDAR → Novatel（外参变换）
                    nx = (
                        extr_R[0][0] * p[0]
                        + extr_R[0][1] * p[1]
                        + extr_R[0][2] * p[2]
                        + extr_t[0]
                    )
                    ny = (
                        extr_R[1][0] * p[0]
                        + extr_R[1][1] * p[1]
                        + extr_R[1][2] * p[2]
                        + extr_t[1]
                    )
                    nz = (
                        extr_R[2][0] * p[0]
                        + extr_R[2][1] * p[1]
                        + extr_R[2][2] * p[2]
                        + extr_t[2]
                    )
                    # Novatel → World（ego 位姿）
                    wx = (
                        ego_R[0][0] * nx
                        + ego_R[0][1] * ny
                        + ego_R[0][2] * nz
                        + self.latest_ego_x
                    )
                    wy = (
                        ego_R[1][0] * nx
                        + ego_R[1][1] * ny
                        + ego_R[1][2] * nz
                        + self.latest_ego_y
                    )
                    wz = (
                        ego_R[2][0] * nx
                        + ego_R[2][1] * ny
                        + ego_R[2][2] * nz
                        + self.latest_ego_z
                    )
                    world_pts.append((wx, wy, wz, p[3]))
            self.latest_world_pts = world_pts

    def on_localization(self, data):
        """
        定位位姿回调（来自 Orin 端 localization 模块，跨机器订阅）。
        # 注：ego 位姿用于将点云从车辆坐标系变换到世界坐标系。
        # 如果 ego 的 z 值偏低，会导致所有点云在世界上偏低。
        """
        if not self.running:
            return
        with self.lock:
            p = data.pose
            self.latest_ego_x = p.position.x
            self.latest_ego_y = p.position.y
            self.latest_ego_z = p.position.z
            q = p.orientation
            self.latest_ego_quat = (q.qx, q.qy, q.qz, q.qw)
            self.latest_ego_yaw = quat_to_yaw(self.latest_ego_quat)
            self.latest_loc_ts = (
                data.header.timestamp_sec
                if data.header.HasField("timestamp_sec")
                else 0
            )
            self.ego_pose_ready = True

    # ----------------------------------------------------------------
    # CARLA 线程（只读连接，不 tick）
    # ----------------------------------------------------------------
    def carla_thread_func(self):
        """
        CARLA 只读线程。
        连接到 P3 本地的 CARLA 服务器，在每帧 tick 后获取 Ground Truth。
        注意：不调用 world.tick() —— Bridge 负责驱动仿真。
        """
        if not HAS_CARLA:
            return
        try:
            client = carla.Client(self.carla_host, self.carla_port)
            client.set_timeout(10.0)
            world = client.get_world()
            self.carla_connected = True
            print(f"[CARLA] ✅ 已连接: {self.carla_host}:{self.carla_port}")
        except Exception as e:
            print(f"[CARLA] ❌ 连接失败: {e}，降级运行（无 Ground Truth）")
            return

        while self.running:
            try:
                # 使用 wait_for_tick 同步 Bridge 的 tick（不主动 tick）
                # 注：这里不能用 world.tick() —— Bridge 是唯一的 tick 驱动者
                world.wait_for_tick(5.0)
                self.gt_frame_no += 1

                actors = world.get_actors()
                vehicles = actors.filter("*vehicle*")
                walkers = actors.filter("*walker*")

                with self.lock:
                    self.carla_npcs = {}
                    ego_actor = None

                    # ---- 找 ego（role_name 为 hero 或 ego_vehicle） ----
                    for v in vehicles:
                        role = v.attributes.get("role_name", "")
                        if role in ("hero", "ego_vehicle"):
                            ego_actor = v
                            break

                    # ---- 写入 ego GT ----
                    if ego_actor:
                        et = ego_actor.get_transform()
                        # CARLA → Apollo：y 取反，yaw 取反转弧度
                        self.carla_ego_gt = {
                            "id": ego_actor.id,
                            "x": et.location.x,
                            "y": -et.location.y,
                            "z": et.location.z,
                            "roll": et.rotation.roll,
                            "pitch": -et.rotation.pitch,
                            "yaw": -math.radians(et.rotation.yaw),
                            "vel": math.sqrt(
                                ego_actor.get_velocity().x ** 2
                                + ego_actor.get_velocity().y ** 2
                            ),
                        }

                        # ---- 写入每个车辆 NPC ----
                    for v in vehicles:
                        if ego_actor and v.id == ego_actor.id:
                            continue
                        t = v.get_transform()
                        bbox = v.bounding_box
                        ext = bbox.extent
                        vel = v.get_velocity()

                        # ★ 重要：bbox 几何中心 = transform * bbox.location
                        #   不是 transform.location（那是 rear axle 在地面的位置）
                        bbox_c = t.transform(bbox.location)
                        ax, ay, az, ayaw = carla_to_apollo(
                            bbox_c.x, bbox_c.y, bbox_c.z, t.rotation.yaw
                        )

                        if self.carla_ego_gt:
                            eg = self.carla_ego_gt
                            dx, dy = ax - eg["x"], ay - eg["y"]
                            dist = math.hypot(dx, dy)
                            eyaw = eg["yaw"]
                            rel_fwd = dx * math.cos(eyaw) + dy * math.sin(eyaw)
                            rel_right = -dx * math.sin(eyaw) + dy * math.cos(eyaw)
                        else:
                            dist = rel_fwd = rel_right = 0.0

                        self.carla_npcs[v.id] = {
                            "type": "VEHICLE",
                            "x_apollo": ax,
                            "y_apollo": ay,
                            "z_apollo": az,
                            "yaw_apollo": ayaw,
                            "t_z": t.location.z,  # 注：actor位置z，用于对比bbox_c_z
                            "half_l": ext.x,
                            "half_w": ext.y,
                            "half_h": ext.z,
                            "length": ext.x * 2,
                            "width": ext.y * 2,
                            "height": ext.z * 2,
                            "vel": math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2),
                            "dist": round(dist, 2),
                            "rel_fwd": round(rel_fwd, 2),
                            "rel_right": round(rel_right, 2),
                        }

                    # ---- 写入行人 NPC ----
                    for w in walkers:
                        t = w.get_transform()
                        bbox = w.bounding_box
                        ext = bbox.extent
                        vel = w.get_velocity()
                        bbox_c = t.transform(bbox.location)
                        ax, ay, az, ayaw = carla_to_apollo(
                            bbox_c.x, bbox_c.y, bbox_c.z, t.rotation.yaw
                        )

                        if self.carla_ego_gt:
                            eg = self.carla_ego_gt
                            dx, dy = ax - eg["x"], ay - eg["y"]
                            dist = math.hypot(dx, dy)
                            eyaw = eg["yaw"]
                            rel_fwd = dx * math.cos(eyaw) + dy * math.sin(eyaw)
                            rel_right = -dx * math.sin(eyaw) + dy * math.cos(eyaw)
                        else:
                            dist = rel_fwd = rel_right = 0.0

                        self.carla_npcs[w.id] = {
                            "type": "PEDESTRIAN",
                            "x_apollo": ax,
                            "y_apollo": ay,
                            "z_apollo": az,
                            "yaw_apollo": ayaw,
                            "t_z": t.location.z,
                            "half_l": ext.x,
                            "half_w": ext.y,
                            "half_h": ext.z,
                            "length": ext.x * 2,
                            "width": ext.y * 2,
                            "height": ext.z * 2,
                            "vel": math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2),
                            "dist": round(dist, 2),
                            "rel_fwd": round(rel_fwd, 2),
                            "rel_right": round(rel_right, 2),
                        }

            except Exception as e:
                if self.running:
                    print(f"[CARLA] 异常: {e}")
                time.sleep(0.5)

    # ----------------------------------------------------------------
    # 感知回调 —— 核心处理 + 写日志
    # ----------------------------------------------------------------
    def on_obstacles(self, data):
        """
        感知障碍物回调（来自 Orin 端 CenterPoint，跨机器订阅）。
        这是整个管线的"最后一环"——当数据到达时，点云和 GT 应该已就绪。
        在此完成：统计、匹配、漏检分析，并写入一帧完整数据。
        """
        if not self.running:
            return
        self.frame_no += 1
        ts = data.header.timestamp_sec if data.header.HasField("timestamp_sec") else 0

        # 加锁快照所有最新数据
        with self.lock:
            lidar_pts = list(self.latest_lidar_pts)
            world_pts = list(self.latest_world_pts)
            pc_ts = self.latest_pc_ts
            ego_x, ego_y, ego_z = self.latest_ego_x, self.latest_ego_y, self.latest_ego_z
            ego_yaw = self.latest_ego_yaw
            ego_ok = self.ego_pose_ready
            carla_npcs = dict(self.carla_npcs)
            carla_ego = dict(self.carla_ego_gt) if self.carla_ego_gt else None
            carla_connected = self.carla_connected

        n_total_pts = len(lidar_pts)

        # ================================================================
        # [FRAME] 帧头
        # ================================================================
        pc_dt_ms = round(self.pc_frame_dt * 1000, 1) if self.pc_frame_dt > 0 else 0
        self.log(f"[FRAME] frame={self.frame_no} ts={ts:.3f} "
                 f"gt_frame={self.gt_frame_no} "
                 f"ego_pose={ego_ok} n_pc_pts={n_total_pts} "
                 f"n_gt_npc={len(carla_npcs)} "
                 f"n_obs={len(data.perception_obstacle)} "
                 f"pc_dt_ms={pc_dt_ms}")
        self.log(f"# 帧号={self.frame_no} | 仿真时间={ts:.3f}s | "
                 f"GT帧={self.gt_frame_no} | "
                 f"点云={n_total_pts}点 | 帧间隔={pc_dt_ms}ms | "
                 f"CARLA NPC={len(carla_npcs)}个 | "
                 f"感知障碍物={len(data.perception_obstacle)}个")
        self.log(f"# 注：感知回调触发写入，点云和GT可能滞后0~2帧（取最新值）")
        self.log(f"# 注：点云帧间隔应稳定在 100ms（10Hz），如出现 300ms+ 说明有点云丢帧")
        self.log("")

        # ================================================================
        # [GT_EGO] CARLA Ground Truth Ego
        # ================================================================
        if carla_ego:
            self.log(f"[GT_EGO] id={carla_ego['id']} "
                     f"x={carla_ego['x']:.3f} y={carla_ego['y']:.3f} "
                     f"z={carla_ego['z']:.3f} yaw={carla_ego['yaw']:.3f} "
                     f"vel={carla_ego['vel']:.2f}")
            self.log(f"# 自车位姿（Apollo 坐标系）| "
                     f"位置=({carla_ego['x']:.2f}, {carla_ego['y']:.2f}, {carla_ego['z']:.2f})m | "
                     f"朝向={carla_ego['yaw']:.2f}rad | 速度={carla_ego['vel']:.2f}m/s")
            self.log(f"# 注：坐标经 CARLA→Apollo 转换（y取反，yaw取反转弧度）")
            self.log("")

        # ================================================================
        # [GT_NPC] 每个 CARLA NPC 的 Ground Truth
        # ================================================================
        for nid, npc in carla_npcs.items():
            self.log(f"[GT_NPC] id={nid} type={npc['type']} "
                     f"cx={npc['x_apollo']:.3f} cy={npc['y_apollo']:.3f} "
                     f"cz={npc['z_apollo']:.3f} "
                     f"t_z={npc.get('t_z',0):.3f} "
                     f"half_l={npc['half_l']:.3f} half_w={npc['half_w']:.3f} "
                     f"half_h={npc['half_h']:.3f} "
                     f"yaw={npc['yaw_apollo']:.3f} vel={npc['vel']:.2f} "
                     f"dist={npc['dist']:.2f} "
                     f"rel_fwd={npc['rel_fwd']:.2f} rel_right={npc['rel_right']:.2f}")
            self.log(f"# GT_NPC: id={nid} | 类型={npc['type']} | "
                     f"bbox中心=({npc['x_apollo']:.2f}, {npc['y_apollo']:.2f}, {npc['z_apollo']:.2f}) | "
                     f"尺寸={npc['length']:.2f}×{npc['width']:.2f}×{npc['height']:.2f}m | "
                     f"距自车={npc['dist']:.1f}m | "
                     f"相对方位 前/右=({npc['rel_fwd']:+.1f}, {npc['rel_right']:+.1f})m")
            self.log(f"# 注：bbox中心是几何中心（t.transform(bbox.location)），不是rear axle在地面的位置")
            self.log(f"# 注：dist<30m 且 rel_fwd>0（前方）的目标是本次关注重点")
        self.log("")

        # ================================================================
        # [PC] 原始点云统计（LiDAR 坐标系）
        # ================================================================
        if lidar_pts:
            xs = [p[0] for p in lidar_pts]
            ys = [p[1] for p in lidar_pts]
            zs = [p[2] for p in lidar_pts]
            intensities = [p[3] for p in lidar_pts]
            ground_pct = sum(1 for p in lidar_pts if p[2] < -1.8) / len(lidar_pts) * 100
            self.log(f"[PC] total={n_total_pts} "
                     f"x=[{min(xs):.2f},{max(xs):.2f}] "
                     f"y=[{min(ys):.2f},{max(ys):.2f}] "
                     f"z=[{min(zs):.2f},{max(zs):.2f}] "
                     f"int_mean={statistics.mean(intensities):.1f} "
                     f"int_med={statistics.median(intensities):.1f} "
                     f"ground_pct={ground_pct:.1f}")
            self.log(f"# 原始点云（LiDAR坐标系）| 总点数={n_total_pts} | "
                     f"x范围=[{min(xs):.2f}, {max(xs):.2f}]m | "
                     f"y范围=[{min(ys):.2f}, {max(ys):.2f}]m | "
                     f"z范围=[{min(zs):.2f}, {max(zs):.2f}]m | "
                     f"强度均值={statistics.mean(intensities):.1f} | "
                     f"地面点比例={ground_pct:.1f}%")
            self.log(f"# 注：64线激光雷达 ~13000点/帧 为正常，~6500点可能是 rotation_frequency 偏高（20Hz）")
            self.log(f"# 注：地面点比例 >70% 说明车体上点少，检测会困难")
            self.log("")

            # [PCW] 世界坐标点云 Z 分布
            if world_pts:
                wzs = [p[2] for p in world_pts]
                wz_mean = statistics.mean(wzs)
                near_pts = [
                    p[2]
                    for p in world_pts
                    if abs(p[0] - ego_x) < 2 and abs(p[1] - ego_y) < 2
                ]
                wz_ground = (
                    statistics.median(near_pts) if len(near_pts) > 5 else wz_mean
                )
                self.log(f"[PCW] total={len(world_pts)} "
                         f"z=[{min(wzs):.2f},{max(wzs):.2f}] "
                         f"z_mean={wz_mean:.2f} z_near_ego={wz_ground:.2f}")
                self.log(f"# 世界坐标点云 | 点数={len(world_pts)} | "
                         f"z范围=[{min(wzs):.2f}, {max(wzs):.2f}]m | "
                         f"z均值={wz_mean:.2f}m | 自车附近z中位数={wz_ground:.2f}m")
                self.log(f"# 注：z均值≈0为正常（地面在z=0附近）。如果z均值<-0.2，说明外参trans.z设低了或ego位姿z偏低")
                self.log(f"# 注：自车附近z中位数代表地面高度。如果远小于0，说明点云整体偏低")
                self.log("")

        # ================================================================
        # [PC_ON_NPC] 原始点云打在 CARLA 真实 bbox 内的点数
        # ================================================================
        pc_on_npc = {}
        if not carla_npcs:
            self.log(f"[PC_ON_NPC] 无CARLA GT数据 carla_connected={carla_connected}")
            self.log(f"# 注：CARLA连接状态={carla_connected}，无NPC，无法统计bbox内点数")
            self.log(f"# 注：如果 carla_connected=False 说明 CARLA 线程还没连上或已断开")
        elif world_pts:
            for nid, npc in carla_npcs.items():
                cos_y = math.cos(npc["yaw_apollo"])
                sin_y = math.sin(npc["yaw_apollo"])
                hl, hw, hh = npc["half_l"], npc["half_w"], npc["half_h"]
                cx, cy, cz = npc["x_apollo"], npc["y_apollo"], npc["z_apollo"]
                pts_in = 0
                pts_near = 0
                intensities_in = []
                for pw in world_pts:
                    if point_in_box(pw[0], pw[1], pw[2], cx, cy, cz, hl, hw, hh, cos_y, sin_y):
                        pts_in += 1
                        intensities_in.append(pw[3])
                    elif point_in_box(pw[0], pw[1], pw[2], cx, cy, cz,
                                      hl + 1.0, hw + 1.0, hh + 0.5, cos_y, sin_y):
                        pts_near += 1

                pc_on_npc[nid] = {
                    "pts": pts_in,
                    "pts_near": pts_near,
                    "int_mean": round(statistics.mean(intensities_in), 1) if intensities_in else 0,
                }
                self.log(f"[PC_ON_NPC] npc_id={nid} type={npc['type']} "
                         f"pts={pts_in} pts_near={pts_near} "
                         f"int_mean={pc_on_npc[nid]['int_mean']} "
                         f"dist={npc['dist']:.2f} rel_fwd={npc['rel_fwd']:.2f}")
                self.log(f"# NPC_{nid} bbox内点云 | 框内点数={pts_in} | "
                         f"框外1m内={pts_near} | "
                         f"强度均值={pc_on_npc[nid]['int_mean']} | "
                         f"距自车={npc['dist']:.1f}m")
                if pts_in >= 20:
                    self.log(f"# 评估：✅ 充足（≥20点），模型应该有足够特征")
                elif pts_in >= 5:
                    self.log(f"# 评估：🟡 一般（5~19点），勉强可用但检测可能不稳定")
                elif pts_in >= 1:
                    self.log(f"# 评估：🔴 稀少（1~4点），模型很难稳定检出")
                else:
                    self.log(f"# 评估：❌ 无点云（0点），请检查坐标变换或外参")
            self.log("")
        else:
            self.log(f"[PC_ON_NPC] 无世界坐标点云（可能ego_pose未就绪）")
            self.log(f"# 注：世界坐标点云为空（可能是定位位姿还没收到，或点云还没来）")
            self.log("")

        # ================================================================
        # [OBS] Apollo 感知输出的每个障碍物
        # ================================================================
        obstacles = []
        detected_npc_ids = set()

        for obj in data.perception_obstacle:
            oid = obj.id
            otype = TYPE_NAMES.get(obj.type, f"T{obj.type}")
            cx, cy, cz = obj.position.x, obj.position.y, obj.position.z
            L = obj.length if obj.HasField("length") else 0
            W = obj.width if obj.HasField("width") else 0
            H = obj.height if obj.HasField("height") else 0
            theta = obj.theta  # Apollo 坐标系下的朝向（弧度）
            conf = obj.confidence if obj.HasField("confidence") else -1
            tt = obj.tracking_time if obj.HasField("tracking_time") else -1
            vel_mag = math.sqrt(
                obj.velocity.x ** 2 + obj.velocity.y ** 2 + obj.velocity.z ** 2
            )
            dist_ego = math.hypot(cx - ego_x, cy - ego_y) if ego_ok else 0

            # ── OBS bbox 内点云统计（世界坐标系） ──
            pts_in = 0
            pts_near = 0
            intensities_in = []
            if world_pts and L > 0.1 and W > 0.1:
                cos_t = math.cos(theta)   # theta 是 Apollo 坐标系下的朝向
                sin_t = math.sin(theta)
                half_h = max(H / 2.0, 0.5)
                for pw in world_pts:
                    if point_in_box(pw[0], pw[1], pw[2],
                                    cx, cy, cz, L/2, W/2, half_h, cos_t, sin_t):
                        pts_in += 1
                        intensities_in.append(pw[3])
                    elif point_in_box(pw[0], pw[1], pw[2],
                                      cx, cy, cz, L/2+1.0, W/2+1.0, half_h+0.5, cos_t, sin_t):
                        pts_near += 1
            int_mean = statistics.mean(intensities_in) if intensities_in else 0

            # ── OBS ↔ GT NPC 匹配（Apollo 坐标系下） ──
            matched_npc = None
            match_dist = 999.0
            for nid, npc in carla_npcs.items():
                d = math.hypot(cx - npc["x_apollo"], cy - npc["y_apollo"])
                if d < match_dist and d < 2.5:
                    match_dist = d
                    matched_npc = nid
            if matched_npc is not None:
                detected_npc_ids.add(matched_npc)

            obs_entry = {
                "id": oid,
                "type": otype,
                "x": round(cx, 2),
                "y": round(cy, 2),
                "z": round(cz, 2),
                "L": round(L, 2),
                "W": round(W, 2),
                "H": round(H, 2),
                "theta": round(theta, 3),
                "conf": round(conf, 3),
                "tt": round(tt, 2),
                "vel": round(vel_mag, 2),
                "dist_ego": round(dist_ego, 2),
                "pts_in": pts_in,
                "pts_near": pts_near,
                "int_mean": round(int_mean, 1),
                "matched_npc": matched_npc,
                "match_dist": round(match_dist, 2) if matched_npc else None,
            }
            obstacles.append(obs_entry)

            # ── 写 OBS 日志行 + 中文注释 ──
            match_str = (
                f"gt={matched_npc} d={match_dist:.2f}"
                if matched_npc
                else "gt=None(假阳性)"
            )
            sub_type = obj.sub_type if obj.HasField("sub_type") else -1
            self.log(f"[OBS] id={oid} type={otype}({obj.type}) sub={sub_type} "
                     f"pos=({cx:.2f},{cy:.2f},{cz:.2f}) "
                     f"box=({L:.2f},{W:.2f},{H:.2f}) theta={theta:.3f} "
                     f"conf={conf:.3f} tt={tt:.2f} vel={vel_mag:.2f} "
                     f"dist={dist_ego:.2f} pts_in={pts_in} pts_near={pts_near} "
                     f"int_mean={int_mean:.1f} {match_str}")
            self.log(f"# OBS: id={oid} | 类型={otype} | 置信度={conf:.3f} | "
                     f"位置=({cx:.2f}, {cy:.2f}, {cz:.2f})m | "
                     f"尺寸={L:.2f}×{W:.2f}×{H:.2f}m | 朝向={theta:.3f}rad | "
                     f"跟踪时长={tt:.2f}s | 距自车={dist_ego:.1f}m | "
                     f"OBS框内点数={pts_in} | {match_str}")
            if matched_npc:
                self.log(f"# 注：匹配到 GT_NPC_{matched_npc}（距离={match_dist:.2f}m < 2.5m阈值）")
            else:
                self.log(f"# 注：未匹配到GT，可能是假阳性或超出GT范围的远距离目标")
            if conf < 0.5:
                self.log(f"# 注：⚠️ 置信度 {conf:.3f} < 0.5，可能被后处理滤掉")
            if tt < 1.0:
                self.log(f"# 注：⚠️ 跟踪时长 {tt:.2f}s < 1s，刚出现的不稳定目标")
            self.log("")

        # ================================================================
        # 逐 NPC 统计更新
        # ================================================================
        for nid in detected_npc_ids:
            npc = carla_npcs.get(nid)
            if not npc:
                continue
            ns = self.npc_stats[nid]
            ns["frames"] += 1
            ns["detected"] += 1
            # 取最高置信度
            best_conf = max(
                (o["conf"] for o in obstacles if o["matched_npc"] == nid),
                default=0,
            )
            if best_conf > 0:
                ns["confs"].append(best_conf)
            # 取最长 tracking_time
            best_tt = max(
                (o["tt"] for o in obstacles if o["matched_npc"] == nid),
                default=0,
            )
            if best_tt > 0:
                ns["tracking_times"].append(best_tt)
            # 类型分类正确性
            gt_type = npc["type"]
            any_type_ok = any(
                o["matched_npc"] == nid and o["type"] == gt_type for o in obstacles
            )
            if any_type_ok:
                ns["type_ok"] += 1
            else:
                ns["type_bad"] += 1
            # 检测到的 NPC 也要记录 pts_on_gt，否则汇总统计只反映漏检帧（偏保守）
            if nid in pc_on_npc:
                ns["pts_on_npc_list"].append(pc_on_npc[nid]["pts"])

        # ================================================================
        # [MISS] 漏检 NPC 分析
        # ================================================================
        missed_list = []
        for nid, npc in carla_npcs.items():
            if nid in detected_npc_ids:
                continue
            raw = pc_on_npc.get(nid, {})
            has_pts = raw.get("pts", 0) > 0
            pts_val = raw.get("pts", 0)

            missed_entry = {
                "npc_id": nid,
                "type": npc["type"],
                "dist": npc["dist"],
                "rel_fwd": npc["rel_fwd"],
                "rel_right": npc["rel_right"],
                "pts_on_gt": pts_val,
                "pts_near_gt": raw.get("pts_near", 0),
                "has_raw_points": has_pts,
            }
            missed_list.append(missed_entry)

            # 统计更新
            ns = self.npc_stats[nid]
            ns["frames"] += 1
            ns["missed"] += 1
            if has_pts:
                ns["missed_with_pts"] += 1
            else:
                ns["missed_no_pts"] += 1
            if raw:
                ns["pts_on_npc_list"].append(pts_val)

            # 漏检根因分类
            if pts_val == 0 and raw.get("pts_near", 0) == 0:
                reason = "pts_in_box=0(完全无点云)"
                reason_detail = "物理层问题：点没打到车上，请检查外参或LiDAR安装"
            elif pts_val == 0 and raw.get("pts_near", 0) > 0:
                reason = "pts_in_box=0(附近有点)"
                reason_detail = "物理层问题：点很近但没进bbox，可能是bbox偏小或坐标偏差"
            elif pts_val < 5:
                reason = f"pts_in_box={pts_val}(点云不足)"
                reason_detail = "物理层问题：只有极少数点，模型pillar特征不足"
            else:
                reason = f"pts_in_box={pts_val}(点云充足)"
                reason_detail = "模型层问题：点云充足但未检出，需要检查模型或参数"

            self.log(f"[MISS] npc_id={nid} type={npc['type']} "
                     f"dist={npc['dist']:.2f} "
                     f"rel_fwd={npc['rel_fwd']:.2f} rel_right={npc['rel_right']:.2f} "
                     f"pts_on_gt={pts_val} pts_near_gt={raw.get('pts_near',0)} "
                     f"reason={reason}")
            self.log(f"# 漏检: GT_NPC_{nid} | 类型={npc['type']} | "
                     f"距自车={npc['dist']:.1f}m | "
                     f"方位 前/右=({npc['rel_fwd']:+.1f}, {npc['rel_right']:+.1f})m | "
                     f"GT框内点数={pts_val} | 根因={reason}")
            self.log(f"# 诊断：{reason_detail}")

            # 针对近处漏检给出更强提示
            if npc["dist"] < 30 and pts_val < 5:
                self.log(f"# ⚠️ 重点关注：距自车仅{npc['dist']:.1f}m的目标漏检，点云覆盖={pts_val}点")
            self.log("")

        # ================================================================
        # [MATCH] 帧匹配摘要
        # ================================================================
        n_npc = len(carla_npcs)
        n_det = len(detected_npc_ids)
        n_miss = len(missed_list)
        missed_with = sum(1 for m in missed_list if m["has_raw_points"])
        missed_without = sum(1 for m in missed_list if not m["has_raw_points"])
        n_fp = sum(1 for o in obstacles if o["matched_npc"] is None)

        self.log(f"[MATCH] frame={self.frame_no} "
                 f"total_gt={n_npc} detected={n_det} missed={n_miss} "
                 f"missed_with_pts={missed_with} missed_no_pts={missed_without} "
                 f"false_positive={n_fp}")
        self.log(f"# 帧匹配汇总 | GT总数={n_npc} | "
                 f"检测到={n_det} | 漏检={n_miss} | 误报={n_fp}")
        self.log(f"# 漏检中：有点云={missed_with}个 | 无点云={missed_without}个")
        self.log(f"# 注：有点云但漏检 → 模型或参数问题；无点云漏检 → 物理或坐标变换问题")

        # 距离分桶统计
        near_total = sum(1 for n in carla_npcs.values() if n["dist"] < 15)
        mid_total  = sum(1 for n in carla_npcs.values() if 15 <= n["dist"] < 30)
        far_total  = sum(1 for n in carla_npcs.values() if n["dist"] >= 30)
        near_miss  = sum(1 for m in missed_list if m["dist"] < 15)
        mid_miss   = sum(1 for m in missed_list if 15 <= m["dist"] < 30)
        far_miss   = sum(1 for f in missed_list if f["dist"] >= 30)
        near_det   = near_total - near_miss
        mid_det    = mid_total - mid_miss
        far_det    = far_total - far_miss
        self.log(f"# 距离分桶 | "
                 f"近(0-15m): {near_det}/{near_total} | "
                 f"中(15-30m): {mid_det}/{mid_total} | "
                 f"远(>30m): {far_det}/{far_total}")
        self.log("")

        # 实时终端显示摘要
        print(f"[帧{self.frame_no:>4}] GT={n_npc} OBS={len(obstacles)} "
              f"OK={n_det} MISS={n_miss}(有{missed_with}/无{missed_without}) "
              f"FP={n_fp} PC={n_total_pts}")

        # 近处漏检警告
        for m in missed_list:
            if m["dist"] < 30 and m["has_raw_points"] and m["pts_on_gt"] >= 3:
                print(f"  ⚠ NPC{m['npc_id']}({m['type']}) d={m['dist']:.1f}m "
                      f"有{m['pts_on_gt']}点但未检出 "
                      f"方位=({m['rel_fwd']:+.1f},{m['rel_right']:+.1f})m")
            elif m["dist"] < 15 and m["pts_on_gt"] == 0:
                print(f"  ❌ NPC{m['npc_id']}({m['type']}) d={m['dist']:.1f}m "
                      f"0点 → 检查外参和坐标变换")

    # ----------------------------------------------------------------
    # 汇总报告
    # ----------------------------------------------------------------
    def generate_report(self):
        """采集结束后输出汇总分析"""
        self.log("")
        self.log("=" * 80)
        self.log("[SUMMARY] 汇总分析")
        self.log("=" * 80)
        self.log("")

        elapsed = time.time() - self.start_time
        self.log(f"[SUMMARY] 运行时间={elapsed:.0f}s 总帧数={self.frame_no}")
        self.log(f"# 采集耗时={elapsed:.0f}s | 总帧数={self.frame_no} | "
                 f"平均{self.frame_no / max(elapsed,1):.1f}帧/秒")

        # 全局检测/漏检统计
        total_det = sum(s["detected"] for s in self.npc_stats.values())
        total_miss = sum(s["missed"] for s in self.npc_stats.values())
        total_frames = sum(s["frames"] for s in self.npc_stats.values())
        det_rate = total_det * 100 // max(total_frames, 1)
        self.log(f"[SUMMARY] 总NPC-帧记录={total_frames} "
                 f"检测成功={total_det}({det_rate}%) "
                 f"漏检={total_miss}({100 - det_rate}%)")
        self.log(f"# 整体检测率={det_rate}% | "
                 f"成功={total_det}次 | 漏检={total_miss}次")
        self.log(f"# 注：检测率 >90% 为良好，<80% 需要排查")

        # 点云密度
        all_pts = []
        for r in self.npc_stats.values():
            all_pts.extend(r["pts_on_npc_list"])
        if all_pts:
            avg_pts = statistics.mean(all_pts)
            med_pts = statistics.median(all_pts)
            zero_pct = sum(1 for v in all_pts if v == 0) * 100 // len(all_pts)
            self.log(f"[SUMMARY] 原始点云打在GT-bbox内: "
                     f"avg={avg_pts:.2f} med={med_pts:.0f} zero_pct={zero_pct}% "
                     f"samples={len(all_pts)}")
            self.log(f"# GT框内点云统计 | 均值={avg_pts:.2f}点 | "
                     f"中位数={med_pts:.0f}点 | 零点点率={zero_pct}% | "
                     f"样本数={len(all_pts)}")
            if avg_pts >= 20:
                self.log(f"# 评估：✅ 点云充足（均值>{avg_pts:.0f}点），问题大概率在模型或后处理")
            elif avg_pts >= 10:
                self.log(f"# 评估：🟡 点云一般（均值{avg_pts:.1f}点），可能部分帧因点数不足漏检")
            else:
                self.log(f"# 评估：🔴 点云不足（均值{avg_pts:.1f}点），物理层是主要瓶颈")

        # 分类精度
        type_ok = sum(s["type_ok"] for s in self.npc_stats.values())
        type_bad = sum(s["type_bad"] for s in self.npc_stats.values())
        if type_ok + type_bad > 0:
            type_acc = type_ok * 100 // max(type_ok + type_bad, 1)
            self.log(f"[SUMMARY] 分类精度: 正确={type_ok}({type_acc}%) "
                     f"错误={type_bad}({100 - type_acc}%)")
            self.log(f"# 类型识别 | 正确={type_ok}次({type_acc}%) | "
                     f"错误={type_bad}次({100 - type_acc}%)")
            if type_acc < 80:
                self.log(f"# 注：⚠️ 类型混淆严重（如 VEHICLE↔BICYCLE），模型需要重新训练或调参")

        # 置信度分布
        all_conf = []
        for s in self.npc_stats.values():
            all_conf.extend(s["confs"])
        if all_conf:
            low_conf = sum(1 for c in all_conf if c < 0.5)
            self.log(f"[SUMMARY] 置信度: avg={statistics.mean(all_conf):.3f} "
                     f"med={statistics.median(all_conf):.3f} "
                     f"<0.5占比={low_conf * 100 // len(all_conf)}%")
            self.log(f"# 置信度 | 均值={statistics.mean(all_conf):.3f} | "
                     f"中位数={statistics.median(all_conf):.3f} | "
                     f"<0.5的占{low_conf * 100 // len(all_conf)}%")
            self.log(f"# 注：大部分目标置信度应在0.7以上，如果普遍偏低说明模型对仿真数据不适应")

        # tracking_time 分布
        all_tt = []
        for s in self.npc_stats.values():
            all_tt.extend(s["tracking_times"])
        if all_tt:
            tt_short = sum(1 for t in all_tt if t < 1.0)
            self.log(f"[SUMMARY] tracking_time: "
                     f"avg={statistics.mean(all_tt):.2f}s "
                     f"med={statistics.median(all_tt):.2f}s "
                     f"<1s占比={tt_short * 100 // len(all_tt)}%")
            self.log(f"# 跟踪时长 | 均值={statistics.mean(all_tt):.2f}s | "
                     f"中位数={statistics.median(all_tt):.2f}s | "
                     f"<1s占{tt_short * 100 // len(all_tt)}%")
            self.log(f"# 注：大部分目标应跟踪>3s，如果大量<1s说明框不稳定（闪烁）")

        # 有点云但漏检 — 模型问题
        total_missed_with = sum(s["missed_with_pts"] for s in self.npc_stats.values())
        total_missed = sum(s["missed"] for s in self.npc_stats.values())

        self.log("")
        self.log("[ROOT_CAUSE] 根因判定:")
        self.log("# ═══════════════════════════════════════════")
        self.log("# 综合以上数据，判断问题出在哪个层面：")
        self.log("# ═══════════════════════════════════════════")

        if all_pts:
            avg_pts = statistics.mean(all_pts)
            if total_missed > 0 and total_missed_with > total_missed * 0.3:
                self.log(
                    f"[ROOT_CAUSE] 🔴 模型/后处理问题：漏检中{total_missed_with}/{total_missed}"
                    f"（{total_missed_with*100//max(total_missed,1)}%）发生在有点云的情况下"
                )
                self.log(
                    f"# 点云已打到目标但模型未检出 → 检查CenterPoint模型的训练数据是否包含仿真场景"
                )
            if avg_pts < 10:
                self.log(
                    f"[ROOT_CAUSE] 🔴 物理层/坐标变换问题："
                    f"平均每目标仅{avg_pts:.1f}个原始点"
                )
                self.log(
                    f"# 15m内的目标应有20+点，当前仅{avg_pts:.1f}点 → 检查外参trans.z和ego pose的z值"
                )
            elif avg_pts < 30:
                self.log(
                    f"[ROOT_CAUSE] 🟡 点云偏低：平均每目标{avg_pts:.1f}个原始点"
                )
                self.log(
                    f"# 接近但不充足 → 检查LiDAR rotation_frequency或points_per_second配置"
                )
            else:
                self.log(
                    f"[ROOT_CAUSE] 🟢 点云充足：平均每目标{avg_pts:.1f}个原始点"
                )
                self.log(
                    f"# 问题大概率在模型侧 → 检查CenterPoint参数、置信度阈值、NMS配置"
                )

        if type_bad > type_ok * 0.2:
            self.log(
                f"[ROOT_CAUSE] 🔴 类型混淆严重：错误{type_bad}次 vs 正确{type_ok}次"
            )
            self.log(
                f"# VEHICLE↔BICYCLE频繁跳变 → 模型对仿真数据的类型判别能力不足"
            )

        self.log("")
        self.log("=" * 80)
        self.log("[SUMMARY_END]")
        self.log("")

    # ----------------------------------------------------------------
    # 主运行函数
    # ----------------------------------------------------------------
    def run(self):
        # 打开日志文件
        self.log_fp = open(LOG_FILE, "w", encoding="utf-8")
        self.log("=" * 80)
        self.log(f"[CONFIG] P3单机全量采集 - {TIMESTAMP}")
        self.log(f"[CONFIG] CARLA: {self.carla_host}:{self.carla_port}")
        self.log(f"[CONFIG] 外参: t=({extr_t[0]:.3f},{extr_t[1]:.3f},{extr_t[2]:.3f}) "
                 f"q=({extr_q[0]:.4f},{extr_q[1]:.4f},{extr_q[2]:.4f},{extr_q[3]:.4f})")
        self.log(f"[CONFIG] 外参文件: {EXTRINSICS_PATH}")
        self.log(f"[CONFIG] 话题: PC={LIDAR_TOPIC}（P3本地Bridge发布）")
        self.log(f"[CONFIG] 话题: OBS={OBSTACLE_TOPIC}（跨机器→Orin）")
        self.log(f"[CONFIG] 话题: LOC={LOCALIZATION_TOPIC}（跨机器→Orin）")
        self.log(f"[CONFIG] CARLA可用: {HAS_CARLA}")
        self.log("=" * 80)
        self.log("")
        self.log("# ═══════════════════════════════════════════")
        self.log("# 采集脚本说明")
        self.log("#")
        self.log("# 数据来源：")
        self.log("#   [GT]    CARLA Ground Truth（P3本地只读连接）")
        self.log("#   [PC]    原始点云（P3本地Bridge发布的补偿后点云）")
        self.log("#   [OBS]   Apollo感知结果（跨机器订阅Orin的CenterPoint输出）")
        self.log("#   [LOC]   Apollo定位位姿（跨机器订阅Orin的localization模块）")
        self.log("#")
        self.log("# 标签约定：")
        self.log("#   以 [TAG] 开头的是数据行，可使用 grep '^\\[TAG\\]' 过滤")
        self.log("#   以 # 开头的是中文注释行，解释字段含义和正常指标范围")
        self.log("#")
        self.log("# 数据流：Bridge tick CARLA → 发点云 → Orin推理 → 感知结果回传")
        self.log("# 注意：感知结果比GT/点云滞后约1~2帧，已取最新值")
        self.log("# ═══════════════════════════════════════════")
        self.log("")

        # 启动 CARLA 线程
        if HAS_CARLA:
            carla_thread = threading.Thread(target=self.carla_thread_func, daemon=True)
            carla_thread.start()
        else:
            self.log("[CONFIG] CARLA Python API不可用，无Ground Truth数据")
            print("[INFO] CARLA Python API不可用，无Ground Truth数据")

        time.sleep(2)
        self.log(f"[CONFIG] CARLA已连接: {self.carla_connected}")

        # 启动 Cyber
        cyber.init("collect_p3_all")
        node = cyber.Node("collect_p3_all")
        node.create_reader(LIDAR_TOPIC, PointCloud, self.on_pointcloud)
        node.create_reader(OBSTACLE_TOPIC, PerceptionObstacles, self.on_obstacles)
        node.create_reader(LOCALIZATION_TOPIC, LocalizationEstimate, self.on_localization)

        print(f"\n🚀 采集启动，日志: {LOG_FILE}")
        print(f"   按 Ctrl+C 停止\n")

        deadline = time.time() + 600
        while self.running and time.time() < deadline and not cyber.is_shutdown():
            time.sleep(0.1)

        self.running = False
        cyber.shutdown()
        time.sleep(1)

        # 汇总报告
        self.generate_report()
        self.log_fp.close()
        print(f"\n✅ 完成，日志: {LOG_FILE}")
        print(f"   包含: 配置头 + 全部帧数据(GT/点云/感知/匹配/漏检分析) + 汇总")


if __name__ == "__main__":
    dc = DataCollector()
    dc.run()
