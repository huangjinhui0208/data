#!/usr/bin/env python3

"""Read-only diagnostics for Apollo-CARLA transform consistency."""

import argparse
import glob
import math
import os
import sys
import threading
import time


def load_carla_module():
    platform_tag = 'win-amd64' if os.name == 'nt' else 'linux-x86_64'
    version_tag = '%d.%d' % (sys.version_info.major, sys.version_info.minor)
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
    # Previous broad fallback kept for rollback reference. Do not enable by
    # default: loading a py3.7 CARLA egg from Python 3.10 can segfault.
    # incompatible_patterns = [
    #     os.path.join(
    #         bridge_api_dir,
    #         "carla-0.9.15-*-{}.egg".format(platform_tag)),
    # ]
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
            "The bundled CARLA egg must match the current interpreter, e.g. "
            "carla-0.9.15-py{}-linux-x86_64.egg. Do not load a py3.7 egg "
            "from Python 3.10; it can segfault.".format(
                version_tag,
                version_tag)) from exc
    return carla


def parse_role_names(value):
    return {item.strip() for item in value.split(",") if item.strip()}


def cyber_y_from_carla(location):
    return -location.y


def obstacle_distance_to_carla_actor(obstacle, actor):
    location = actor.get_location()
    dx = obstacle.position.x - location.x
    dy = obstacle.position.y - cyber_y_from_carla(location)
    return math.hypot(dx, dy)


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.qw * q.qz + q.qx * q.qy)
    cosy_cosp = 1.0 - 2.0 * (q.qy * q.qy + q.qz * q.qz)
    return math.atan2(siny_cosp, cosy_cosp)


def find_ego(world, role_names):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") in role_names:
            return actor
    return None


def find_npcs(world, role_prefix, ego_id):
    actors = []
    for actor in world.get_actors().filter("vehicle.*"):
        role_name = actor.attributes.get("role_name", "")
        if actor.id != ego_id and role_name.startswith(role_prefix):
            actors.append(actor)
    return actors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--ego-role-names", default="ego_vehicle,hero")
    parser.add_argument("--npc-role-prefix", default="npc")
    parser.add_argument("--duration", default=5.0, type=float)
    parser.add_argument("--perception-topic", default="/apollo/perception/obstacles")
    parser.add_argument("--localization-topic", default="/apollo/localization/pose")
    args = parser.parse_args()

    from cyber.python.cyber_py3 import cyber
    from modules.common_msgs.localization_msgs.localization_pb2 import (
        LocalizationEstimate,
    )
    from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import (
        PerceptionObstacles,
    )

    carla = load_carla_module()
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    latest_localization = {"msg": None}
    latest_perception = {"msg": None}
    lock = threading.Lock()

    def on_localization(msg):
        with lock:
            latest_localization["msg"] = msg

    def on_perception(msg):
        with lock:
            latest_perception["msg"] = msg

    cyber.init()
    node = cyber.Node("carla_transform_diagnostics")
    node.create_reader(args.localization_topic, LocalizationEstimate, on_localization)
    node.create_reader(args.perception_topic, PerceptionObstacles, on_perception)

    deadline = time.time() + args.duration
    while time.time() < deadline and not cyber.is_shutdown():
        time.sleep(0.1)

    ego = find_ego(world, parse_role_names(args.ego_role_names))
    if ego is None:
        print("ERROR: no ego actor found")
        return
    npcs = find_npcs(world, args.npc_role_prefix, ego.id)
    if not npcs:
        print("ERROR: no NPC actors found with prefix '{}'".format(
            args.npc_role_prefix))
        return

    ego_transform = ego.get_transform()
    ego_location = ego_transform.location
    print("CARLA map: {}".format(world.get_map().name))
    print(
        "ego carla id={} role={} loc=({:.3f},{:.3f},{:.3f}) yaw={:.3f}".format(
            ego.id,
            ego.attributes.get("role_name", ""),
            ego_location.x,
            ego_location.y,
            ego_location.z,
            ego_transform.rotation.yaw))
    print(
        "ego expected cyber position=({:.3f},{:.3f},{:.3f}) heading=-yaw={:.6f} "
        "quat_yaw=-(yaw+90)={:.6f}".format(
            ego_location.x,
            cyber_y_from_carla(ego_location),
            ego_location.z,
            -math.radians(ego_transform.rotation.yaw),
            -math.radians(ego_transform.rotation.yaw + 90.0)))

    with lock:
        localization = latest_localization["msg"]
        perception = latest_perception["msg"]

    if localization is None:
        print("WARN: no localization message received on {}".format(
            args.localization_topic))
    else:
        pose = localization.pose
        print(
            "localization position=({:.3f},{:.3f},{:.3f}) heading={:.6f} "
            "orientation_yaw={:.6f}".format(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.heading,
                quaternion_to_yaw(pose.orientation)))

    if perception is None:
        print("WARN: no perception message received on {}".format(
            args.perception_topic))
        return

    for actor in npcs[:5]:
        transform = actor.get_transform()
        location = transform.location
        nearest = None
        nearest_distance = None
        for obstacle in perception.perception_obstacle:
            distance = obstacle_distance_to_carla_actor(obstacle, actor)
            if nearest is None or distance < nearest_distance:
                nearest = obstacle
                nearest_distance = distance
        print(
            "npc carla id={} role={} loc=({:.3f},{:.3f},{:.3f}) yaw={:.3f} "
            "expected cyber=({:.3f},{:.3f},{:.3f}) theta=-yaw={:.6f}".format(
                actor.id,
                actor.attributes.get("role_name", ""),
                location.x,
                location.y,
                location.z,
                transform.rotation.yaw,
                location.x,
                cyber_y_from_carla(location),
                location.z,
                -math.radians(transform.rotation.yaw)))
        if nearest is None:
            print("  no perception obstacle available")
        else:
            print(
                "  nearest perception id={} distance_to_expected={:.3f} "
                "position=({:.3f},{:.3f},{:.3f}) theta={:.6f}".format(
                    nearest.id,
                    nearest_distance,
                    nearest.position.x,
                    nearest.position.y,
                    nearest.position.z,
                    nearest.theta))


if __name__ == "__main__":
    main()
