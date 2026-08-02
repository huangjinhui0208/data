#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""cyber node wraper"""
#import sys
#sys.path.append("/opt/apollo/neo/lib/cyber/python")
#from cyber.python.cyber_py3 import cyber, cyber_time, cyber_timer, parameter

import sys
import os
from carla_bridge.utils.time_sync_manager import TimeSyncManager

# 1. 强行把 Neo 所有的 Python 路径塞进 Python 的内存搜索列表中
neo_paths = [
    "/opt/apollo/neo/lib/cyber/python/internal",
    "/opt/apollo/neo/lib/cyber/python/cyber/python",
    "/opt/apollo/neo/lib/cyber/python"
]

for p in neo_paths:
    if p not in sys.path:
        sys.path.insert(0, p) # 插入到最前面，拥有最高优先级

# 2. 现在尝试导入。如果路径里没有 cyber_py3 文件夹，
# 我们直接从 cyber.python.cyber_py3 导入
try:
    from cyber_py3 import cyber, cyber_time, cyber_timer, parameter
except ImportError:
    try:
        from cyber.python.cyber_py3 import cyber, cyber_time, cyber_timer, parameter
    except ImportError as e:
        print(f"仍然无法找到 Cyber 库。当前的 sys.path 是: {sys.path}")
        raise e



class CyberNode(object):
    """Cyber Node"""

    def __init__(self, name, **kwargs):
        self.kwargs = kwargs
        self.cyber = cyber
        self.cyber.init(name)
        self.node = self.cyber.Node(name)

        self.cyber_time = cyber_time
        self.cyber_timer = cyber_timer
        self.cyber_parameter = parameter
        self.time_sync_manager = TimeSyncManager.get_instance()

    def get_param(self, name, alternative_value=None):
        return self.cyber_parameter.Parameter(name, alternative_value)

    def configure_time_sync(self, enable_frame_lock=True):
        self.time_sync_manager.configure(enable_frame_lock=enable_frame_lock)

    def on_world_tick(self, frame_id, carla_elapsed_seconds=None):
        return self.time_sync_manager.on_world_tick(frame_id, carla_elapsed_seconds)

    def get_time(self, frame_id=None):
        return self.time_sync_manager.get_stamp(frame_id=frame_id)

    def get_clock_ns(self, frame_id=None):
        return self.time_sync_manager.get_clock_ns(frame_id=frame_id)

    def new_writer(self, name, data_type, qos_depth=1):
        return self.node.create_writer(name, data_type, qos_depth)

    def new_reader(self, name, data_type, callback, args=None):
        return self.node.create_reader(name, data_type, callback, args)

    def new_rate(self, frequency):
        return self.cyber_time.Rate(frequency)

    def new_timer(self, timer_period_sec, callback, oneshot=0):
        return self.cyber_timer.Timer(timer_period_sec * 1000, callback, oneshot)  # ms

    def new_service(
        self,
        name: object,
        req_type: object,
        res_type: object,
        callback: object,
        args: object = None,
    ) -> object:
        return self.node.create_service(name, req_type, res_type, callback, args)

    def new_client(self, name, req_type, res_type):
        return self.node.create_client(name, req_type, res_type)

    def call_service(self, client, req):
        response = client.send_request(req)
        return response

    def spin(self):
        self.node.spin()

    def destroy(self):
        # del self.node
        # self.shutdown()
        pass

    def ok(self):
        return not self.cyber.is_shutdown()

    def shutdown(self):
        self.cyber.shutdown()

    def wait_for_shutdown(self):
        self.cyber.waitforshutdown()

    def get_timestamp(self, sec=0, nsec=0, from_sec=False):
        total = 0
        if from_sec:
            total = self.cyber_time.Time(float(sec)).to_nsec()
        else:
            total = self.cyber_time.Time(sec * 1000000000 + nsec)
        secs = total / 1000000000
        nsecs = total - secs * 1000000000
        return {"secs": secs, "nsecs": nsecs}
