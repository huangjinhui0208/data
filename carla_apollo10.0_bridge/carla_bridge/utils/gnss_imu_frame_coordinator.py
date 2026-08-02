#!/usr/bin/env python
#
# Coordinates IMU/GNSS publication so Apollo observes IMU no later than the
# matching GNSS frame.
#

import threading


class GnssFrameBundle(object):
    """Container for one GNSS frame's converted Apollo messages."""

    def __init__(self, best_pose, odometry, heading, status, publish_cb):
        self.best_pose = best_pose
        self.odometry = odometry
        self.heading = heading
        self.status = status
        self._publish_cb = publish_cb

    def publish(self):
        self._publish_cb(self)


class GnssImuFrameCoordinator(object):
    """Caches IMU/GNSS messages by CARLA frame and releases matched pairs."""

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
        self._pending_frames = {}
        self._max_pending_frames = 128
        self._imu_publish_cb = None
        self._gnss_publish_cb = None

    def register_imu_publisher(self, publish_cb):
        with self._lock:
            self._imu_publish_cb = publish_cb

    def register_gnss_publisher(self, publish_cb):
        with self._lock:
            self._gnss_publish_cb = publish_cb

    def submit_imu(self, frame_id, imu_msg):
        publish_pair = None
        with self._lock:
            entry = self._pending_frames.setdefault(frame_id, {})
            entry["imu"] = imu_msg
            publish_pair = self._pop_ready_pair_locked(frame_id)
            self._cleanup_locked(frame_id)
            imu_publish_cb = self._imu_publish_cb
            gnss_publish_cb = self._gnss_publish_cb
        self._publish_pair(publish_pair, imu_publish_cb, gnss_publish_cb)

    def submit_gnss(self, frame_id, gnss_bundle):
        publish_pair = None
        with self._lock:
            entry = self._pending_frames.setdefault(frame_id, {})
            entry["gnss"] = gnss_bundle
            publish_pair = self._pop_ready_pair_locked(frame_id)
            self._cleanup_locked(frame_id)
            imu_publish_cb = self._imu_publish_cb
            gnss_publish_cb = self._gnss_publish_cb
        self._publish_pair(publish_pair, imu_publish_cb, gnss_publish_cb)

    def _pop_ready_pair_locked(self, frame_id):
        entry = self._pending_frames.get(frame_id)
        if not entry or "imu" not in entry or "gnss" not in entry:
            return None

        imu_msg = entry["imu"]
        gnss_bundle = entry["gnss"]
        self._pending_frames.pop(frame_id, None)
        return imu_msg, gnss_bundle

    def _cleanup_locked(self, current_frame_id):
        stale_before = current_frame_id - self._max_pending_frames
        stale_frame_ids = [
            frame_id
            for frame_id in self._pending_frames
            if frame_id < stale_before
        ]
        for frame_id in stale_frame_ids:
            self._pending_frames.pop(frame_id, None)

    @staticmethod
    def _publish_pair(publish_pair, imu_publish_cb, gnss_publish_cb):
        if publish_pair is None or imu_publish_cb is None or gnss_publish_cb is None:
            return

        imu_msg, gnss_bundle = publish_pair
        imu_publish_cb(imu_msg)
        gnss_publish_cb(gnss_bundle)
