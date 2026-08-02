#!/usr/bin/env python3
"""P3端：仅采集CARLA Ground Truth，输出JSON行日志，供Orin端合并"""
import carla, math, time, json
from datetime import datetime

HOST = "127.0.0.1"
PORT = 2000

client = carla.Client(HOST, PORT)
client.set_timeout(10)
world = client.get_world()
settings = world.get_settings()
sync = settings.synchronous_mode

log_file = f"carla_gt_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
fp = open(log_file, "w")
print(f"采集GT → {log_file}")

frame = 0
while True:
    try:
        if sync:
            world.wait_for_tick(5.0)
        else:
            time.sleep(0.1)
        frame += 1

        actors = world.get_actors()
        vehicles = actors.filter("*vehicle*")
        walkers = actors.filter("*walker*")

        # 用CARLA仿真时间（与Orin端Cyber消息时间戳同源）
        snap = world.get_snapshot()
        sim_ts = snap.timestamp.elapsed_seconds if snap and sync else time.time()

        # 找ego
        ego_id = None
        for v in vehicles:
            if v.attributes.get("role_name", "") in ("hero", "ego_vehicle"):
                ego_id = v.id
                break

        # 输出ego
        if ego_id:
            ego_v = [v for v in vehicles if v.id == ego_id][0]
            et = ego_v.get_transform()
            fp.write(json.dumps({
                "tag": "GT_EGO", "frame": frame, "ts": sim_ts,
                "id": ego_id,
                "x": et.location.x, "y": et.location.y, "z": et.location.z,
                "yaw": et.rotation.yaw,
            }) + "\n")

        # 输出每个NPC
        for v in vehicles:
            if v.id == ego_id: continue
            t = v.get_transform()
            bbox = v.bounding_box
            bbox_center = t.transform(bbox.location)
            ext = bbox.extent
            vel = v.get_velocity()
            fp.write(json.dumps({
                "tag": "GT_NPC", "frame": frame, "ts": sim_ts,
                "id": v.id, "type": "VEHICLE",
                "x": t.location.x, "y": t.location.y,
                "bbox_cx": bbox_center.x, "bbox_cy": bbox_center.y, "bbox_cz": bbox_center.z,
                "yaw": t.rotation.yaw,
                "half_l": ext.x, "half_w": ext.y, "half_h": ext.z,
                "length": ext.x*2, "width": ext.y*2, "height": ext.z*2,
                "vel": math.sqrt(vel.x**2+vel.y**2+vel.z**2),
            }) + "\n")

        for w in walkers:
            t = w.get_transform()
            bbox = w.bounding_box
            bbox_center = t.transform(bbox.location)
            ext = bbox.extent
            vel = w.get_velocity()
            fp.write(json.dumps({
                "tag": "GT_NPC", "frame": frame, "ts": sim_ts,
                "id": w.id, "type": "PEDESTRIAN",
                "x": t.location.x, "y": t.location.y,
                "bbox_cx": bbox_center.x, "bbox_cy": bbox_center.y, "bbox_cz": bbox_center.z,
                "yaw": t.rotation.yaw,
                "half_l": ext.x, "half_w": ext.y, "half_h": ext.z,
                "length": ext.x*2, "width": ext.y*2, "height": ext.z*2,
                "vel": math.sqrt(vel.x**2+vel.y**2+vel.z**2),
            }) + "\n")

        fp.flush()
        if frame % 50 == 0:
            print(f"  Frame {frame}")

    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"  Error: {e}")

fp.close()
print(f"完成, {frame}帧 → {log_file}")
