#!/usr/bin/env python3

"""Count raw Apollo lidar points inside CARLA actor bounding boxes.

This read-only diagnostic answers one narrow question: does the raw lidar topic
contain points on CARLA walkers before Apollo perception runs any detector or
filter?
"""

import argparse
import csv
import glob
import math
import os
import sys
import threading
import time

import numpy as np


def load_carla_module():
    platform_tag = "win-amd64" if os.name == "nt" else "linux-x86_64"
    version_tag = "%d.%d" % (sys.version_info.major, sys.version_info.minor)
    bridge_api_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "carla_api")
    compatible_patterns = [
        os.path.join(
            bridge_api_dir,
            "carla-0.9.15-py%s-%s.egg" % (version_tag, platform_tag)),
        os.path.join(
            bridge_api_dir,
            "carla-*py%s-%s.egg" % (version_tag, platform_tag)),
    ]
    for pattern in compatible_patterns:
        matches = sorted(glob.glob(pattern), reverse=True)
        if matches:
            sys.path.insert(0, matches[0])
            break
    try:
        import carla
    except ImportError as exc:
        raise RuntimeError(
            "No CARLA Python API compatible with Python {} was found. "
            "The bundled CARLA egg must match the current interpreter.".format(
                version_tag)) from exc
    return carla


def import_cyber():
    neo_paths = [
        "/opt/apollo/neo/lib/cyber/python/internal",
        "/opt/apollo/neo/lib/cyber/python/cyber/python",
        "/opt/apollo/neo/lib/cyber/python",
    ]
    for path in neo_paths:
        if path not in sys.path:
            sys.path.insert(0, path)
    try:
        from cyber_py3 import cyber
    except ImportError:
        from cyber.python.cyber_py3 import cyber
    return cyber


def parse_role_names(value):
    return {item.strip() for item in value.split(",") if item.strip()}


def find_ego(world, role_names):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") in role_names:
            return actor
    return None


def infer_ego_from_lidar(lidar):
    parent = getattr(lidar, "parent", None)
    if parent is not None and parent.type_id.startswith("vehicle."):
        return parent
    return None


def find_lidar(world, role_name):
    for actor in world.get_actors().filter("sensor.lidar.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def describe_actors(world):
    lines = []
    all_actors = list(world.get_actors())
    lines.append("  total actors: {}".format(len(all_actors)))
    if all_actors:
        lines.append("  first actors:")
        for actor in all_actors[:20]:
            parent_id = actor.parent.id if getattr(actor, "parent", None) else None
            lines.append(
                "    id={} type={} role_name={} parent_id={}".format(
                    actor.id,
                    actor.type_id,
                    actor.attributes.get("role_name", ""),
                    parent_id))
    for pattern in ("vehicle.*", "sensor.lidar.*"):
        actors = list(world.get_actors().filter(pattern))
        if not actors:
            lines.append("  {}: none".format(pattern))
            continue
        lines.append("  {}:".format(pattern))
        for actor in actors:
            parent_id = actor.parent.id if getattr(actor, "parent", None) else None
            lines.append(
                "    id={} type={} role_name={} parent_id={}".format(
                    actor.id,
                    actor.type_id,
                    actor.attributes.get("role_name", ""),
                    parent_id))
    return "\n".join(lines)


def parse_port_range(value):
    if "," in value:
        return [int(item.strip()) for item in value.split(",") if item.strip()]
    if "-" in value:
        start, end = value.split("-", 1)
        return list(range(int(start), int(end) + 1))
    return [int(value)]


def probe_ports(carla_module, host, ports):
    for port in ports:
        try:
            client = carla_module.Client(host, port)
            client.set_timeout(2.0)
            world = client.get_world()
            actors = list(world.get_actors())
            vehicles = list(world.get_actors().filter("vehicle.*"))
            lidars = list(world.get_actors().filter("sensor.lidar.*"))
            print(
                "port={} map={} total_actors={} vehicles={} lidars={}".format(
                    port,
                    world.get_map().name,
                    len(actors),
                    len(vehicles),
                    len(lidars)))
            for actor in vehicles[:5] + lidars[:5]:
                parent_id = actor.parent.id if getattr(actor, "parent", None) else None
                print(
                    "  id={} type={} role_name={} parent_id={}".format(
                        actor.id,
                        actor.type_id,
                        actor.attributes.get("role_name", ""),
                        parent_id))
        except RuntimeError as exc:
            print("port={} ERROR: {}".format(port, exc))


def watch_actors(world, duration, sample_period):
    deadline = time.time() + duration
    while time.time() < deadline:
        print("Available CARLA actors:\n{}".format(describe_actors(world)))
        wait_for_world_tick(world, sample_period)


def wait_for_world_tick(world, timeout=1.0):
    try:
        world.wait_for_tick(timeout)
    except RuntimeError:
        time.sleep(min(timeout, 0.2))


def wait_for_lidar(world, role_name, timeout):
    deadline = time.time() + timeout
    while True:
        lidar = find_lidar(world, role_name)
        if lidar is not None:
            return lidar
        if time.time() >= deadline:
            return None
        wait_for_world_tick(world)


def wait_for_ego(world, role_names, lidar, timeout):
    deadline = time.time() + timeout
    while True:
        ego = find_ego(world, role_names)
        if ego is not None:
            return ego
        ego = infer_ego_from_lidar(lidar)
        if ego is not None:
            return ego
        if time.time() >= deadline:
            return None
        wait_for_world_tick(world)


def transform_to_matrix(transform):
    return np.asarray(transform.get_matrix(), dtype=np.float64)


def inverse_transform_to_matrix(transform):
    return np.asarray(transform.get_inverse_matrix(), dtype=np.float64)


def apply_transform(points, matrix):
    if points.size == 0:
        return points.copy()
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    return points.dot(rotation.T) + translation


def actor_distance_xy(actor, ego):
    actor_loc = actor.get_location()
    ego_loc = ego.get_location()
    return math.hypot(actor_loc.x - ego_loc.x, actor_loc.y - ego_loc.y)


def bbox_point_stats(points_lidar_carla, lidar_transform, actor, margin):
    """Return point count and nearest distance for points inside actor bbox."""
    if points_lidar_carla.size == 0:
        return 0, float("nan")

    lidar_to_world = transform_to_matrix(lidar_transform)
    world_to_actor = inverse_transform_to_matrix(actor.get_transform())
    lidar_to_actor = world_to_actor.dot(lidar_to_world)
    points_actor = apply_transform(points_lidar_carla, lidar_to_actor)

    bbox = actor.bounding_box
    bbox_transform = actor.get_transform().__class__(bbox.location, bbox.rotation)
    actor_to_bbox = inverse_transform_to_matrix(bbox_transform)
    points_bbox = apply_transform(points_actor, actor_to_bbox)
    extent = np.array(
        [bbox.extent.x + margin, bbox.extent.y + margin,
         bbox.extent.z + margin],
        dtype=np.float64)

    delta = np.abs(points_bbox)
    inside_mask = np.all(delta <= extent, axis=1)
    count = int(np.count_nonzero(inside_mask))
    if count:
        nearest = float(np.min(np.linalg.norm(points_bbox[inside_mask], axis=1)))
    else:
        nearest = float(np.min(np.linalg.norm(points_bbox, axis=1)))
    return count, nearest


class LidarFrameCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.points_lidar_carla = np.empty((0, 3), dtype=np.float32)
        self.measurement_time = 0.0
        self.header_time = 0.0
        self.sequence_num = 0
        self.frame_id = ""
        self.receive_time = 0.0
        self.total_points = 0

    def update(self, msg):
        points = np.empty((len(msg.point), 3), dtype=np.float32)
        for index, point in enumerate(msg.point):
            # Bridge publishes Apollo lidar local points as [x, -carla_y, z].
            # Convert them back to CARLA lidar local frame before using CARLA
            # actor transforms.
            points[index, 0] = point.x
            points[index, 1] = -point.y
            points[index, 2] = point.z

        with self.lock:
            self.points_lidar_carla = points
            self.measurement_time = msg.measurement_time
            self.header_time = msg.header.timestamp_sec
            self.sequence_num = msg.header.sequence_num
            self.frame_id = msg.frame_id or msg.header.frame_id
            self.receive_time = time.time()
            self.total_points = len(msg.point)

    def snapshot(self):
        with self.lock:
            return {
                "points_lidar_carla": self.points_lidar_carla.copy(),
                "measurement_time": self.measurement_time,
                "header_time": self.header_time,
                "sequence_num": self.sequence_num,
                "frame_id": self.frame_id,
                "receive_time": self.receive_time,
                "total_points": self.total_points,
            }


def open_csv(path):
    if not path:
        return None, None
    csv_file = open(path, "w", newline="")
    writer = csv.DictWriter(
        csv_file,
        fieldnames=[
            "wall_time",
            "measurement_time",
            "header_time",
            "sequence_num",
            "frame_id",
            "total_points",
            "actor_id",
            "actor_type",
            "role_name",
            "distance_xy",
            "bbox_length",
            "bbox_width",
            "bbox_height",
            "bbox_margin",
            "points_in_bbox",
            "nearest_point_to_bbox_center",
            "hit",
        ])
    writer.writeheader()
    return csv_file, writer


def format_actor_line(row):
    return (
        "  actor id={actor_id} type={actor_type} role={role_name} "
        "dist={distance_xy:.2f}m bbox=({bbox_length:.2f},"
        "{bbox_width:.2f},{bbox_height:.2f}) points={points_in_bbox} "
        "nearest={nearest_point_to_bbox_center:.2f} hit={hit}"
    ).format(**row)


def build_row(snapshot, actor, distance, count, nearest, margin):
    bbox = actor.bounding_box
    return {
        "wall_time": time.time(),
        "measurement_time": snapshot["measurement_time"],
        "header_time": snapshot["header_time"],
        "sequence_num": snapshot["sequence_num"],
        "frame_id": snapshot["frame_id"],
        "total_points": snapshot["total_points"],
        "actor_id": actor.id,
        "actor_type": actor.type_id,
        "role_name": actor.attributes.get("role_name", ""),
        "distance_xy": distance,
        "bbox_length": bbox.extent.x * 2.0,
        "bbox_width": bbox.extent.y * 2.0,
        "bbox_height": bbox.extent.z * 2.0,
        "bbox_margin": margin,
        "points_in_bbox": count,
        "nearest_point_to_bbox_center": nearest,
        "hit": bool(count > 0),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--duration", default=20.0, type=float)
    parser.add_argument(
        "--lidar-topic",
        default="/apollo/sensor/velodyne64/compensator/PointCloud2")
    parser.add_argument("--lidar-role-name", default="velodyne64")
    parser.add_argument("--ego-role-names", default="ego_vehicle,hero")
    parser.add_argument("--walker-filter", default="walker.pedestrian.*")
    parser.add_argument(
        "--actor-filter",
        default=None,
        help="Override actor filter for sanity checks, e.g. vehicle.*")
    parser.add_argument("--max-distance", default=40.0, type=float)
    parser.add_argument("--bbox-margin", default=0.15, type=float)
    parser.add_argument("--sample-period", default=0.5, type=float)
    parser.add_argument("--actor-wait-timeout", default=30.0, type=float)
    parser.add_argument(
        "--list-actors",
        action="store_true",
        help="Print current CARLA actors and exit without reading Cyber topics")
    parser.add_argument(
        "--watch-actors",
        action="store_true",
        help="Print CARLA actors repeatedly for --duration seconds and exit")
    parser.add_argument(
        "--probe-ports",
        default=None,
        help="Probe CARLA RPC ports and exit, e.g. 2000-2010 or 2000,2002")
    parser.add_argument("--output-csv", default="/tmp/lidar_walker_hits.csv")
    args = parser.parse_args()

    carla = load_carla_module()
    print("CARLA Python API: {}".format(getattr(carla, "__file__", "<python-path>")))

    if args.probe_ports:
        probe_ports(carla, args.host, parse_port_range(args.probe_ports))
        return 0

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    print("CARLA map: {}".format(world.get_map().name))
    if args.list_actors:
        print("Available CARLA actors:\n{}".format(describe_actors(world)))
        return 0
    if args.watch_actors:
        watch_actors(world, args.duration, args.sample_period)
        return 0

    from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointCloud

    cyber = import_cyber()

    print("waiting up to {:.1f}s for lidar role_name '{}'".format(
        args.actor_wait_timeout,
        args.lidar_role_name))
    lidar = wait_for_lidar(world, args.lidar_role_name, args.actor_wait_timeout)
    if lidar is None:
        print("ERROR: no lidar actor found with role_name '{}'".format(
            args.lidar_role_name))
        print("Available CARLA actors:\n{}".format(describe_actors(world)))
        return 1

    role_names = parse_role_names(args.ego_role_names)
    ego = wait_for_ego(world, role_names, lidar, args.actor_wait_timeout)
    if ego is None:
        print("ERROR: no ego actor found for roles {}".format(args.ego_role_names))
        print("Available CARLA actors:\n{}".format(describe_actors(world)))
        return 1
    if ego.attributes.get("role_name") not in role_names:
        print(
            "WARN: no ego actor found for roles {}; using lidar parent "
            "vehicle id={} role_name={} instead".format(
                args.ego_role_names,
                ego.id,
                ego.attributes.get("role_name", "")))

    cache = LidarFrameCache()

    cyber.init()
    node = cyber.Node("carla_lidar_walker_hit_diagnostics")
    node.create_reader(args.lidar_topic, PointCloud, cache.update)

    csv_file, csv_writer = open_csv(args.output_csv)
    actor_filter = args.actor_filter or args.walker_filter
    deadline = time.time() + args.duration
    last_measurement_time = None

    print("ego id={} role={} lidar id={} role={} actor_filter={}".format(
        ego.id,
        ego.attributes.get("role_name", ""),
        lidar.id,
        lidar.attributes.get("role_name", ""),
        actor_filter))
    print("listening on {}".format(args.lidar_topic))

    try:
        while time.time() < deadline and not cyber.is_shutdown():
            time.sleep(args.sample_period)
            snapshot = cache.snapshot()
            if snapshot["receive_time"] <= 0.0:
                print("WARN: no lidar message received yet")
                continue

            if last_measurement_time == snapshot["measurement_time"]:
                print("WARN: lidar measurement_time did not advance: {:.6f}".format(
                    last_measurement_time))
            last_measurement_time = snapshot["measurement_time"]

            actors = []
            for actor in world.get_actors().filter(actor_filter):
                distance = actor_distance_xy(actor, ego)
                if distance <= args.max_distance:
                    actors.append((distance, actor))
            actors.sort(key=lambda item: item[0])

            print(
                "frame seq={} time={:.6f} total_points={} near_actors={}".format(
                    snapshot["sequence_num"],
                    snapshot["measurement_time"],
                    snapshot["total_points"],
                    len(actors)))

            if not actors:
                continue

            lidar_transform = lidar.get_transform()
            for distance, actor in actors:
                count, nearest = bbox_point_stats(
                    snapshot["points_lidar_carla"],
                    lidar_transform,
                    actor,
                    args.bbox_margin)
                row = build_row(
                    snapshot,
                    actor,
                    distance,
                    count,
                    nearest,
                    args.bbox_margin)
                print(format_actor_line(row))
                if csv_writer is not None:
                    csv_writer.writerow(row)
                    csv_file.flush()
    finally:
        if csv_file is not None:
            csv_file.close()
        cyber.shutdown()

    if args.output_csv:
        print("CSV written to {}".format(args.output_csv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
