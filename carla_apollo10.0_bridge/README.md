# Based on Apollo11.0


To make it compatible with Apollo 11.0, I have modified the original repository: https://github.com/guardstrikelab/carla_apollo_bridge.

Additionally, the environment configuration has also been changed.

You can configure it by following the steps below:

Prerequisite: A running Apollo Docker container must already be deployed

clone the repository in the path apollo/modules outside the container


1.Environment Variable Configuration

Enter the Apollo container and modify /home/$USER/.bashrc.


Append the following to the end of the file:

```
export PYTHONPATH=$PYTHONPATH:/apollo/modules/carla_apollo_bridge/carla_bridge/carla_api/carla-0.9.15-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:/apollo/bazel-bin
export PYTHONPATH=$PYTHONPATH:/apollo/modules/carla_apollo_bridge
export PYTHONPATH=$PYTHONPATH:/apollo/cyber/python
```


2.Install the dependencies required for carla_bridge

cd carla_apollo_bridge/carla_bridge


cp -r map/. /apollo/modules/map/data


pip3 install -r requirements.txt


3.Download CARLA_0.9.15.tar.gz
https://github.com/carla-simulator/carla/releases


Running Instructions:

1.Inside the container, compile data modules

Modify channel name where use the "/apollo/canbus/chassis" to "/apollo/canbus/carla/chassis"
```
part 1
modules/control/control_component/control_component.cc
chassis_reader_config.channel_name = "/apollo/canbus/carla/chassis";

part 2
modules/external_command/command_processor/action_command_processor/conf/config.pb.txt
command_status_name: "/apollo/canbus/carla/chassis"

part 3
modules/planning/planning_component/conf/planning_config.pb.txt
chassis_topic: "/apollo/canbus/carla/chassis"
```

Start the three Apollo modules: planning, control and dreamview_plus.

2.Outside the container, launch CarlaUE4 with the specified port:

./CarlaUE4.sh -carla-port=2000

3.Inside the container, start the carla_bridge:

python carla_apollo_bridge/carla_bridge/main.py


![carla_apollo11 0_bridge](https://github.com/user-attachments/assets/6f399fc8-55ad-431b-8683-935081b52ea6)

## Low-overhead collision history

In synchronous mode the bridge reuses its existing CARLA `WorldSnapshot` to
keep a fixed-size, in-memory pre-collision history. With the default 0.1 s
step, 10 s window, and 8 actor slots, the binary ring buffer is 28,000 bytes.
It creates no extra CARLA RPCs and publishes no trajectory data to Apollo.

Only the first collision is persisted by default. The output directory gets
one collision event JSONL/CSV pair and one
`carla_collision_actor_history_*.csv` containing the ego and collision actor
history. File serialization starts after the collision on a one-shot worker
thread. The behavior is configured by the `collision_history_*` and
`collision_first_event_only` keys in `carla_bridge/config/settings.yaml`.

## SCB control-delay evidence

When `control_delay_injection.enabled=true`, the Bridge main entry creates
`scb_control_delay_*.csv` immediately after loading `settings.yaml`, before it
connects to CARLA. It writes `BRIDGE_CONFIG_LOADED`; the ego instance later
appends `INITIALIZED` to the same file. The rows contain the main/config/source
paths, PID and working directory. If neither the configured directory nor the
fallback directory is writable, Bridge fails fast instead of running an
experiment without evidence. Reaching
`activation_speed_mps` latches the instance as armed; the first later command
whose brake percentage reaches `brake_threshold_percentage` triggers the
configured fixed delay.

Apollo `/apollo/control` runs near 100 Hz in the captured Apollo 10 traces.
Keep `log_all_delayed_commands=false` for experiments.  The v2 CSV separates
command receipt, delay release, CARLA API call start, and API call end.  None
of these API timestamps alone proves physical braking; the offline analyzer
detects sustained deceleration from localization speed samples.


