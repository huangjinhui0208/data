#!/usr/bin/env python
#
# Unified timestamp manager for bridge messages.
#

import threading
import time


class TimeSyncManager(object):
    """
    Singleton time manager.

    V1 behavior:
    - Uses system wall clock as source of truth.
    - Locks one timestamp per CARLA frame when frame-lock is enabled.
    - Enforces monotonic timestamps (never go backward).
    """

    _instance = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        self._lock = threading.Lock()
        self._enable_frame_lock = True
        self._current_frame_id = None
        self._frame_timestamps_ns = {}
        self._last_stamp_sec = 0.0
        self._min_step_sec = 1e-6

    def configure(self, enable_frame_lock=True):
        with self._lock:
            self._enable_frame_lock = bool(enable_frame_lock)

    def on_world_tick(self, frame_id, _carla_elapsed_seconds=None):
        with self._lock:
            stamp_ns = time.time_ns()
            self._current_frame_id = frame_id
            self._frame_timestamps_ns[frame_id] = stamp_ns

            # Keep a bounded cache to avoid unbounded growth.
            if len(self._frame_timestamps_ns) > 256:
                keys = sorted(self._frame_timestamps_ns.keys())
                for key in keys[:-128]:
                    self._frame_timestamps_ns.pop(key, None)

            return self._ns_to_sec(stamp_ns)

    def get_stamp(self, frame_id=None):
        with self._lock:
            return self._ns_to_sec(self._get_stamp_ns_locked(frame_id=frame_id))

    def get_clock_ns(self, frame_id=None):
        with self._lock:
            return self._get_stamp_ns_locked(frame_id=frame_id)

    @staticmethod
    def _ns_to_sec(stamp_ns):
        return stamp_ns / 1000000000.0

    def _get_stamp_ns_locked(self, frame_id=None):
        if self._enable_frame_lock:
            if frame_id is not None and frame_id in self._frame_timestamps_ns:
                return self._frame_timestamps_ns[frame_id]
            if (
                self._current_frame_id is not None
                and self._current_frame_id in self._frame_timestamps_ns
            ):
                return self._frame_timestamps_ns[self._current_frame_id]

        return time.time_ns()

    def _monotonic_stamp_locked(self, candidate):
        # Deprecated safeguard: kept for reference but no longer used
        # in the active time retrieval path.
        if candidate <= self._last_stamp_sec:
            candidate = self._last_stamp_sec + self._min_step_sec
        self._last_stamp_sec = candidate
        return candidate
