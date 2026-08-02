#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

import glob
import math
import os
import sys
import time
import argparse
import logging
from numpy import random

carla = None
CARLA_MODULE_PATH = None


def find_carla_egg(script_dir):
    platform_tag = 'win-amd64' if os.name == 'nt' else 'linux-x86_64'
    version_tag = '%d.%d' % (sys.version_info.major, sys.version_info.minor)
    candidate_api_dirs = [
        # New location: modules/carla_apollo10.0_bridge/carla_bridge/
        os.path.join(script_dir, "carla_api"),
        # Old location: modules/
        os.path.join(
            script_dir,
            "carla_apollo10.0_bridge",
            "carla_bridge",
            "carla_api"),
    ]
    candidate_dist_dirs = [
        # Old location fallback: /apollo_workspace/carla/dist
        os.path.abspath(os.path.join(script_dir, "..", "carla", "dist")),
        # New location fallback: /apollo_workspace/carla/dist
        os.path.abspath(os.path.join(script_dir, "..", "..", "..", "carla", "dist")),
    ]
    compatible_patterns = []
    for api_dir in candidate_api_dirs:
        compatible_patterns.append(os.path.join(
            api_dir,
            "carla-0.9.15-py%s-%s.egg" % (version_tag, platform_tag)))
    for dist_dir in candidate_dist_dirs:
        compatible_patterns.append(os.path.join(
            dist_dir,
            "carla-0.9.15-py%s-%s.egg" % (version_tag, platform_tag)))
    for api_dir in candidate_api_dirs:
        compatible_patterns.append(os.path.join(
            api_dir,
            "carla-*py%s-%s.egg" % (version_tag, platform_tag)))
    for dist_dir in candidate_dist_dirs:
        compatible_patterns.append(os.path.join(
            dist_dir,
            "carla-*py%s-%s.egg" % (version_tag, platform_tag)))
    # Previous broad fallback kept for rollback reference. Do not enable by
    # default: loading a py3.7 CARLA egg from Python 3.10 can segfault.
    # incompatible_patterns = [
    #     os.path.join(
    #         bridge_api_dir,
    #         "carla-0.9.15-*-{}.egg".format(platform_tag)),
    #     os.path.join(
    #         carla_dist_dir,
    #         "carla-0.9.15-*-{}.egg".format(platform_tag)),
    # ]
    for pattern in compatible_patterns:
        matches = sorted(glob.glob(pattern), reverse=True)
        if matches:
            return matches[0]
    return None


def load_carla_module():
    global carla, CARLA_MODULE_PATH
    if carla is not None:
        return CARLA_MODULE_PATH

    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Previous broad lookup kept for rollback reference:
    # egg_patterns = [
    #     os.path.join(
    #         script_dir,
    #         "carla_apollo10.0_bridge",
    #         "carla_bridge",
    #         "carla_api",
    #         "carla-*%d.%d-%s.egg" % (
    #             sys.version_info.major,
    #             sys.version_info.minor,
    #             'win-amd64' if os.name == 'nt' else 'linux-x86_64')),
    #     os.path.join(
    #         script_dir,
    #         "carla_apollo10.0_bridge",
    #         "carla_bridge",
    #         "carla_api",
    #         "carla-*-linux-x86_64.egg"),
    #     os.path.join(
    #         script_dir,
    #         "..",
    #         "carla",
    #         "dist",
    #         "carla-*%d.%d-%s.egg" % (
    #             sys.version_info.major,
    #             sys.version_info.minor,
    #             'win-amd64' if os.name == 'nt' else 'linux-x86_64')),
    # ]
    try:
        import carla as carla_module
        carla = carla_module
        CARLA_MODULE_PATH = getattr(carla, "__file__", "<python-path>")
        return CARLA_MODULE_PATH
    except ImportError:
        pass

    egg_path = find_carla_egg(script_dir)
    if egg_path:
        sys.path.insert(0, egg_path)
        CARLA_MODULE_PATH = egg_path
    else:
        raise RuntimeError(
            "No CARLA Python API compatible with Python {} was found. "
            "The bundled CARLA egg must match the current interpreter, e.g. "
            "carla-0.9.15-py{}-linux-x86_64.egg. Do not load a py3.7 egg "
            "from Python 3.10; it can segfault.".format(
                "%d.%d" % (sys.version_info.major, sys.version_info.minor),
                "%d.%d" % (sys.version_info.major, sys.version_info.minor)))

    import carla as carla_module
    carla = carla_module
    if CARLA_MODULE_PATH is None:
        CARLA_MODULE_PATH = getattr(carla, "__file__", "<python-path>")
    return CARLA_MODULE_PATH


def normalize_town_name(name):
    if not name:
        return ""
    town = str(name).split("/")[-1].lower()
    if town.startswith("carla_"):
        town = town[len("carla_"):]
    return town


def parse_role_names(value):
    return {item.strip() for item in value.split(",") if item.strip()}


def actor_location(actor):
    try:
        return actor.get_location()
    except RuntimeError:
        return None


def is_dynamic_actor(actor):
    return actor.type_id.startswith("vehicle.") or actor.type_id.startswith("walker.")


def find_ego_actors(world, ego_role_names):
    ego_actors = []
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") in ego_role_names:
            ego_actors.append(actor)
    return ego_actors


def describe_vehicle_roles(world):
    descriptions = []
    for actor in world.get_actors().filter("vehicle.*"):
        descriptions.append("{}:{}:{}".format(
            actor.id,
            actor.type_id,
            actor.attributes.get("role_name", "")))
    return descriptions


def wait_for_ego_actors(world, ego_role_names, timeout):
    deadline = time.time() + timeout
    while True:
        ego_actors = find_ego_actors(world, ego_role_names)
        if ego_actors:
            return ego_actors
        if time.time() >= deadline:
            return []
        world.wait_for_tick()


def is_safe_location(location, ego_actors, dynamic_actors, reserved_locations,
                     ego_keepout_radius, ego_front_keepout,
                     ego_front_lateral_width, spawn_min_distance):
    for ego_actor in ego_actors:
        try:
            ego_transform = ego_actor.get_transform()
            ego_location = ego_transform.location
        except RuntimeError:
            ego_location = None
        if ego_location and location.distance(ego_location) < ego_keepout_radius:
            return False, "ego_keepout"
        if ego_location and ego_front_keepout > 0.0:
            forward = ego_transform.get_forward_vector()
            rel_x = location.x - ego_location.x
            rel_y = location.y - ego_location.y
            longitudinal = rel_x * forward.x + rel_y * forward.y
            lateral = abs(rel_x * forward.y - rel_y * forward.x)
            if (0.0 < longitudinal < ego_front_keepout and
                    lateral < ego_front_lateral_width):
                return False, "ego_front_keepout"

    for actor in dynamic_actors:
        actor_loc = actor_location(actor)
        if actor_loc and location.distance(actor_loc) < spawn_min_distance:
            return False, "existing_actor"

    for reserved_location in reserved_locations:
        if location.distance(reserved_location) < spawn_min_distance:
            return False, "reserved_spawn"

    return True, "ok"


def filter_vehicle_spawn_points(carla_map, spawn_points, ego_actors, dynamic_actors,
                                ego_keepout_radius, ego_front_keepout,
                                ego_front_lateral_width, spawn_min_distance):
    filtered = []
    reserved_locations = []
    stats = {
        "input": len(spawn_points),
        "non_driving_lane": 0,
        "ego_keepout": 0,
        "ego_front_keepout": 0,
        "existing_actor": 0,
        "reserved_spawn": 0,
    }

    for transform in spawn_points:
        waypoint = carla_map.get_waypoint(
            transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving)
        if waypoint is None:
            stats["non_driving_lane"] += 1
            continue

        safe, reason = is_safe_location(
            transform.location,
            ego_actors,
            dynamic_actors,
            reserved_locations,
            ego_keepout_radius,
            ego_front_keepout,
            ego_front_lateral_width,
            spawn_min_distance)
        if not safe:
            stats[reason] += 1
            continue

        filtered.append(transform)
        reserved_locations.append(transform.location)

    return filtered, stats


def collect_walker_spawn_points(world, number_of_walkers, ego_actors, dynamic_actors,
                                ego_keepout_radius, ego_front_keepout,
                                ego_front_lateral_width, spawn_min_distance):
    spawn_points = []
    reserved_locations = []
    stats = {
        "attempts": 0,
        "no_nav_location": 0,
        "ego_keepout": 0,
        "ego_front_keepout": 0,
        "existing_actor": 0,
        "reserved_spawn": 0,
    }
    max_attempts = max(number_of_walkers * 20, number_of_walkers)

    while len(spawn_points) < number_of_walkers and stats["attempts"] < max_attempts:
        stats["attempts"] += 1
        loc = world.get_random_location_from_navigation()
        if loc is None:
            stats["no_nav_location"] += 1
            continue

        safe, reason = is_safe_location(
            loc,
            ego_actors,
            dynamic_actors,
            reserved_locations,
            ego_keepout_radius,
            ego_front_keepout,
            ego_front_lateral_width,
            spawn_min_distance)
        if not safe:
            stats[reason] += 1
            continue

        spawn_point = carla.Transform()
        spawn_point.location = loc
        spawn_points.append(spawn_point)
        reserved_locations.append(loc)

    return spawn_points, stats


def get_safe_walker_target(world, ego_actors, dynamic_actors, ego_keepout_radius,
                           ego_front_keepout, ego_front_lateral_width,
                           spawn_min_distance, attempts=20):
    for _ in range(attempts):
        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        safe, _ = is_safe_location(
            loc,
            ego_actors,
            dynamic_actors,
            [],
            ego_keepout_radius,
            ego_front_keepout,
            ego_front_lateral_width,
            spawn_min_distance)
        if safe:
            return loc
    return None


def apply_safe_traffic_manager_settings(traffic_manager, actor, args):
    try:
        actor.set_simulate_physics(True)
        actor.apply_control(carla.VehicleControl(
            hand_brake=False, manual_gear_shift=False))
        actor.set_autopilot(True, traffic_manager.get_port())
        traffic_manager.distance_to_leading_vehicle(actor, args.tm_global_distance)
        traffic_manager.ignore_vehicles_percentage(actor, 0.0)
        traffic_manager.ignore_walkers_percentage(actor, 0.0)
        if hasattr(traffic_manager, "ignore_lights_percentage"):
            traffic_manager.ignore_lights_percentage(actor, 0.0)
        if hasattr(traffic_manager, "ignore_signs_percentage"):
            traffic_manager.ignore_signs_percentage(actor, 0.0)
        traffic_manager.vehicle_percentage_speed_difference(
            actor, args.tm_speed_difference)
        if args.tm_desired_speed > 0.0 and hasattr(traffic_manager, "set_desired_speed"):
            traffic_manager.set_desired_speed(actor, args.tm_desired_speed)
        if args.disable_auto_lane_change:
            traffic_manager.auto_lane_change(actor, False)
    except RuntimeError as exc:
        logging.error(
            'failed to configure Traffic Manager for actor id=%s: %s',
            actor.id,
            exc)
        return False
    return True


def get_waypoint_description(carla_map, transform):
    waypoint = carla_map.get_waypoint(
        transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving)
    if waypoint is None:
        return "no driving waypoint"
    return "road_id={} lane_id={} lane_type={} s={:.2f}".format(
        waypoint.road_id,
        waypoint.lane_id,
        waypoint.lane_type,
        waypoint.s)


def log_vehicle_tm_diagnostics(carla_map, traffic_manager, actor, tm_configured):
    try:
        transform = actor.get_transform()
        velocity = actor.get_velocity()
    except RuntimeError as exc:
        logging.warning('cannot read NPC actor id=%s for diagnostics: %s',
                        actor.id, exc)
        return
    speed = (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
    logging.info(
        'NPC TM diagnostic id=%s type=%s role_name=%s tm_port=%s tm_configured=%s '
        'location=(%.2f, %.2f, %.2f) yaw=%.2f speed=%.2f waypoint=(%s)',
        actor.id,
        actor.type_id,
        actor.attributes.get("role_name", ""),
        traffic_manager.get_port(),
        tm_configured,
        transform.location.x,
        transform.location.y,
        transform.location.z,
        transform.rotation.yaw,
        speed,
        get_waypoint_description(carla_map, transform))


def log_transform_diagnostics(carla_map, ego_actors, npc_actors):
    if not ego_actors:
        logging.warning('transform diagnostics skipped: no ego actor')
        return
    if not npc_actors:
        logging.warning('transform diagnostics skipped: no NPC vehicle actor')
        return

    ego_transform = ego_actors[0].get_transform()
    ego_location = ego_transform.location
    ego_heading = -math.radians(ego_transform.rotation.yaw)
    ego_quat_yaw = -math.radians(ego_transform.rotation.yaw + 90.0)
    logging.info(
        'transform diagnostic ego id=%s role_name=%s carla=(%.2f, %.2f, %.2f yaw=%.2f) '
        'cyber_position=(%.2f, %.2f, %.2f) heading=-yaw=%.6f quat_yaw=-(yaw+90)=%.6f',
        ego_actors[0].id,
        ego_actors[0].attributes.get("role_name", ""),
        ego_location.x,
        ego_location.y,
        ego_location.z,
        ego_transform.rotation.yaw,
        ego_location.x,
        -ego_location.y,
        ego_location.z,
        ego_heading,
        ego_quat_yaw)

    for actor in npc_actors:
        transform = actor.get_transform()
        location = transform.location
        rel_x = location.x - ego_location.x
        rel_y = -(location.y - ego_location.y)
        front_heading = rel_x * math.cos(ego_heading) + rel_y * math.sin(ego_heading)
        left_heading = -rel_x * math.sin(ego_heading) + rel_y * math.cos(ego_heading)
        front_quat = rel_x * math.cos(ego_quat_yaw) + rel_y * math.sin(ego_quat_yaw)
        left_quat = -rel_x * math.sin(ego_quat_yaw) + rel_y * math.cos(ego_quat_yaw)
        logging.info(
            'transform diagnostic npc id=%s role_name=%s carla=(%.2f, %.2f, %.2f yaw=%.2f) '
            'cyber_position=(%.2f, %.2f, %.2f) theta=-yaw=%.6f waypoint=(%s) '
            'relative_by_heading(front=%.2f,left=%.2f) relative_by_quat_yaw(front=%.2f,left=%.2f)',
            actor.id,
            actor.attributes.get("role_name", ""),
            location.x,
            location.y,
            location.z,
            transform.rotation.yaw,
            location.x,
            -location.y,
            location.z,
            -math.radians(transform.rotation.yaw),
            get_waypoint_description(carla_map, transform),
            front_heading,
            left_heading,
            front_quat,
            left_quat)


def report_vehicle_motion(world, actors, ticks, label_ticks=None):
    if not actors or ticks <= 0:
        return 0
    for _ in range(ticks):
        world.wait_for_tick()
    if label_ticks is None:
        label_ticks = ticks
    speeds = []
    controls = []
    for actor in actors:
        velocity = actor.get_velocity()
        speed = (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
        speeds.append(speed)
        control = actor.get_control()
        controls.append(
            "id={} throttle={:.2f} brake={:.2f} steer={:.2f} hand_brake={} reverse={}".format(
                actor.id,
                control.throttle,
                control.brake,
                control.steer,
                control.hand_brake,
                control.reverse))
    moving_count = sum(1 for speed in speeds if speed > 0.2)
    logging.info(
        'NPC vehicle motion check after %d ticks: %d/%d moving, speeds=%s m/s',
        label_ticks,
        moving_count,
        len(speeds),
        ['{:.2f}'.format(speed) for speed in speeds])
    logging.info('NPC vehicle controls after %d ticks: %s',
                 label_ticks, controls)
    if moving_count == 0:
        logging.warning(
            'all NPC vehicles are still stationary after %d ticks; Traffic Manager likely did not take control, '
            'or every NPC is blocked by lane topology, traffic light, ego keepout, or a stopped leading actor',
            label_ticks)
    return moving_count


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--expected-town',
        default=None,
        help='Expected CARLA town name. The script exits if the current world does not match.')
    argparser.add_argument(
        '--ego-role-names',
        default='ego_vehicle,hero',
        help='Comma-separated role_name values treated as ego vehicles (default: ego_vehicle,hero)')
    argparser.add_argument(
        '--ego-wait-timeout',
        default=30.0,
        type=float,
        help='Seconds to wait for the bridge to spawn an ego vehicle before failing (default: 30.0)')
    argparser.add_argument(
        '--allow-no-ego',
        action='store_true',
        default=False,
        help='Allow traffic generation when no ego vehicle actor is found')
    argparser.add_argument(
        '--ego-keepout-radius',
        default=25.0,
        type=float,
        help='Do not spawn traffic within this distance from ego vehicles in meters (default: 25.0)')
    argparser.add_argument(
        '--ego-front-keepout',
        default=60.0,
        type=float,
        help='Do not spawn traffic in front of ego within this longitudinal distance in meters (default: 60.0)')
    argparser.add_argument(
        '--ego-front-lateral-width',
        default=7.0,
        type=float,
        help='Half-width of the ego forward keepout corridor in meters (default: 7.0)')
    argparser.add_argument(
        '--spawn-min-distance',
        default=8.0,
        type=float,
        help='Minimum distance from existing/reserved dynamic actors in meters (default: 8.0)')
    argparser.add_argument(
        '--npc-role-prefix',
        default='npc',
        help='Prefix used for generated NPC role_name attributes (default: npc)')
    argparser.add_argument(
        '--tm-global-distance',
        default=6.0,
        type=float,
        help='Traffic Manager following distance in meters (default: 6.0)')
    argparser.add_argument(
        '--tm-speed-difference',
        default=0.0,
        type=float,
        help='Traffic Manager speed difference percentage for NPCs (default: 0.0)')
    argparser.add_argument(
        '--tm-desired-speed',
        default=30.0,
        type=float,
        help='Desired Traffic Manager vehicle speed in km/h; set <= 0 to disable (default: 30.0)')
    argparser.add_argument(
        '--tm-sync-mode',
        choices=['off', 'on', 'auto'],
        default='off',
        help='Traffic Manager synchronous mode policy. Use off when bridge is the tick master (default: off).')
    argparser.add_argument(
        '--motion-check-ticks',
        default=20,
        type=int,
        help='Ticks to wait before logging generated NPC vehicle speeds (default: 20)')
    argparser.add_argument(
        '--motion-check-final-ticks',
        default=50,
        type=int,
        help='Additional total tick target for a second NPC vehicle motion check; set <= 0 to disable (default: 50)')
    argparser.add_argument(
        '--diagnose-transforms',
        action='store_true',
        default=False,
        help='Log CARLA-to-Cyber position/yaw diagnostics for ego and generated NPC vehicles')
    argparser.add_argument(
        '--disable-auto-lane-change',
        action='store_true',
        default=False,
        help='Disable Traffic Manager automatic lane changes for generated NPCs')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Deprecated no-op. The script follows the current CARLA world mode.')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Deprecated no-op. Generated NPCs are never assigned hero role_name.')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Deprecated no-op. This external script never changes world rendering settings.')

    args = argparser.parse_args()
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    try:
        carla_module_path = load_carla_module()
    except RuntimeError as exc:
        logging.error(exc)
        return
    logging.info('using CARLA Python API: %s', carla_module_path)

    vehicles_list = []
    walkers_list = []
    all_id = []
    all_actors = []
    uncontrolled_walker_ids = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()
        carla_map = world.get_map()
        current_town = normalize_town_name(carla_map.name)
        logging.info('connected to CARLA map: %s', carla_map.name)
        if args.expected_town:
            expected_town = normalize_town_name(args.expected_town)
            if current_town != expected_town:
                raise RuntimeError(
                    'current CARLA map "{}" does not match expected town "{}"'.format(
                        carla_map.name, args.expected_town))

        ego_role_names = parse_role_names(args.ego_role_names)
        ego_actors = wait_for_ego_actors(
            world, ego_role_names, args.ego_wait_timeout)
        if ego_actors:
            for actor in ego_actors:
                loc = actor.get_location()
                logging.info(
                    'found ego actor id=%s role_name=%s location=(%.2f, %.2f, %.2f)',
                    actor.id,
                    actor.attributes.get("role_name"),
                    loc.x,
                    loc.y,
                    loc.z)
        elif not args.allow_no_ego:
            raise RuntimeError(
                'no ego vehicle found with role_name in {} after {:.1f}s. '
                'Current vehicle actors are {}. Start the bridge first, check objects.json spawn, '
                'or pass --allow-no-ego.'.format(
                    sorted(ego_role_names),
                    args.ego_wait_timeout,
                    describe_vehicle_roles(world)))
        else:
            logging.warning('no ego vehicle found; continuing because --allow-no-ego was set')

        if args.hero:
            logging.warning('--hero is deprecated and ignored; NPC role_name values use --npc-role-prefix')
        if args.no_rendering:
            logging.warning('--no-rendering is ignored; external traffic script does not change world settings')
        if args.asynch:
            logging.warning('--asynch is deprecated and ignored; following the current CARLA world mode')

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(args.tm_global_distance)
        logging.info(
            'Traffic Manager configured on port=%s global_distance=%.2f speed_difference=%.2f desired_speed=%.2f',
            traffic_manager.get_port(),
            args.tm_global_distance,
            args.tm_speed_difference,
            args.tm_desired_speed)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        # Previous auto policy kept for rollback reference. In the bridge
        # topology this external client is not the tick master, so forcing TM
        # synchronous mode here can leave NPC vehicles without continuous TM
        # control.
        # if settings.synchronous_mode:
        #     traffic_manager.set_synchronous_mode(True)
        #     logging.info(
        #         'world is already synchronous; bridge remains the sync master (fixed_delta_seconds=%s)',
        #         settings.fixed_delta_seconds)
        # else:
        #     traffic_manager.set_synchronous_mode(False)
        #     logging.warning(
        #         'world is asynchronous; continuing without changing world settings')
        if args.tm_sync_mode == 'on' or (
                args.tm_sync_mode == 'auto' and settings.synchronous_mode):
            traffic_manager.set_synchronous_mode(True)
            logging.info(
                'Traffic Manager synchronous mode enabled by --tm-sync-mode=%s; '
                'world_sync=%s fixed_delta_seconds=%s',
                args.tm_sync_mode,
                settings.synchronous_mode,
                settings.fixed_delta_seconds)
        else:
            traffic_manager.set_synchronous_mode(False)
            logging.info(
                'Traffic Manager synchronous mode disabled by --tm-sync-mode=%s; '
                'bridge remains the world tick master (world_sync=%s fixed_delta_seconds=%s)',
                args.tm_sync_mode,
                settings.synchronous_mode,
                settings.fixed_delta_seconds)

        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        if not blueprints:
            raise ValueError("Couldn't find any vehicles with the specified filters")
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
        if not blueprintsWalkers:
            raise ValueError("Couldn't find any walkers with the specified filters")

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        existing_dynamic_actors = [
            actor for actor in world.get_actors()
            if is_dynamic_actor(actor) and actor.id not in {ego.id for ego in ego_actors}
        ]

        spawn_points = carla_map.get_spawn_points()
        spawn_points, spawn_filter_stats = filter_vehicle_spawn_points(
            carla_map,
            spawn_points,
            ego_actors,
            existing_dynamic_actors,
            args.ego_keepout_radius,
            args.ego_front_keepout,
            args.ego_front_lateral_width,
            args.spawn_min_distance)
        number_of_spawn_points = len(spawn_points)
        logging.info('vehicle spawn candidates after filtering: %d (%s)',
                     number_of_spawn_points, spawn_filter_stats)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        skipped_vehicle_collisions = 0
        for n, transform in enumerate(spawn_points):
            if len(vehicles_list) >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if blueprint.has_attribute('role_name'):
                blueprint.set_attribute(
                    'role_name', '{}_vehicle_{}'.format(
                        args.npc_role_prefix, len(vehicles_list)))

            response = client.apply_batch_sync([
                SpawnActor(blueprint, transform).then(
                    SetAutopilot(FutureActor, True, traffic_manager.get_port()))
            ], False)[0]
            if response.error:
                skipped_vehicle_collisions += 1
                continue
            vehicles_list.append(response.actor_id)

        generated_vehicle_actors = world.get_actors(vehicles_list)
        if vehicles_list:
            world.wait_for_tick()
        tm_config_results = {}
        for actor in generated_vehicle_actors:
            tm_config_results[actor.id] = apply_safe_traffic_manager_settings(
                traffic_manager, actor, args)
            log_vehicle_tm_diagnostics(
                carla_map,
                traffic_manager,
                actor,
                tm_config_results[actor.id])
        if args.diagnose_transforms:
            log_transform_diagnostics(carla_map, ego_actors, generated_vehicle_actors)
        moving_count = report_vehicle_motion(
            world, generated_vehicle_actors, args.motion_check_ticks)
        if (args.motion_check_final_ticks > 0 and
                args.motion_check_final_ticks != args.motion_check_ticks):
            extra_ticks = args.motion_check_final_ticks - args.motion_check_ticks
            if extra_ticks > 0:
                final_moving_count = report_vehicle_motion(
                    world,
                    generated_vehicle_actors,
                    extra_ticks,
                    label_ticks=args.motion_check_final_ticks)
            else:
                final_moving_count = moving_count
            if final_moving_count == 0 and generated_vehicle_actors:
                logging.error(
                    'NPC vehicle motion failure: 0/%d vehicles moved by %.2f m/s threshold. '
                    'Verify CARLA API version, Traffic Manager port=%s, TM sync mode, and whether bridge is ticking.',
                    len(generated_vehicle_actors),
                    0.2,
                    traffic_manager.get_port())
        if skipped_vehicle_collisions:
            logging.info(
                'skipped %d vehicle spawn candidates due to CARLA collision checks',
                skipped_vehicle_collisions)

        # Set automatic vehicle lights update if specified
        if args.car_lights_on:
            for actor in generated_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.5      # how many pedestrians will run
        percentagePedestriansCrossing = 1.0     # how many pedestrians will walk through the road
        if args.seedw:
            world.set_pedestrians_seed(args.seedw)
            random.seed(args.seedw)
        # 1. take all the random locations to spawn
        dynamic_actors_for_walkers = existing_dynamic_actors + list(generated_vehicle_actors)
        spawn_points, walker_spawn_stats = collect_walker_spawn_points(
            world,
            max(args.number_of_walkers * 3, args.number_of_walkers),
            ego_actors,
            dynamic_actors_for_walkers,
            args.ego_keepout_radius,
            args.ego_front_keepout,
            args.ego_front_lateral_width,
            args.spawn_min_distance)
        logging.info('walker spawn candidates after filtering: %d (%s)',
                     len(spawn_points), walker_spawn_stats)
        # 2. we spawn the walker object
        walker_speed = []
        skipped_walker_collisions = 0
        for n, spawn_point in enumerate(spawn_points):
            if len(walkers_list) >= args.number_of_walkers:
                break
            walker_bp = random.choice(blueprintsWalkers)
            if walker_bp.has_attribute('role_name'):
                walker_bp.set_attribute(
                    'role_name', '{}_walker_{}'.format(
                        args.npc_role_prefix, len(walkers_list)))
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    speed = walker_bp.get_attribute('speed').recommended_values[1]
                else:
                    # running
                    speed = walker_bp.get_attribute('speed').recommended_values[2]
            else:
                print("Walker has no speed")
                speed = 0.0

            actor = world.try_spawn_actor(walker_bp, spawn_point)
            if actor is None:
                skipped_walker_collisions += 1
                continue
            walkers_list.append({"id": actor.id})
            walker_speed.append(speed)
        if skipped_walker_collisions:
            logging.info(
                'skipped %d walker spawn candidates due to CARLA collision checks',
                skipped_walker_collisions)
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, False)
        controlled_walkers = []
        controlled_walker_speed = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
                uncontrolled_walker_ids.append(walkers_list[i]["id"])
            else:
                walkers_list[i]["con"] = results[i].actor_id
                controlled_walkers.append(walkers_list[i])
                controlled_walker_speed.append(walker_speed[i])
        walkers_list = controlled_walkers
        walker_speed = controlled_walker_speed
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        world.wait_for_tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            target = get_safe_walker_target(
                world,
                ego_actors,
                dynamic_actors_for_walkers,
                args.ego_keepout_radius,
                args.ego_front_keepout,
                args.ego_front_lateral_width,
                args.spawn_min_distance)
            if target is None:
                logging.warning('no safe target found for walker actor id=%s; leaving controller stopped',
                                all_id[i + 1])
                continue
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(target)
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        while True:
            world.wait_for_tick()

    finally:
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            try:
                all_actors[i].stop()
            except RuntimeError:
                pass

        print('\ndestroying %d walkers' % (len(walkers_list) + len(uncontrolled_walker_ids)))
        client.apply_batch([
            carla.command.DestroyActor(x) for x in all_id + uncontrolled_walker_ids
        ])

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
