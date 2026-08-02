#!/usr/bin/env python

#
# Copyright (c) 2018, Willow Garage, Inc.
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

import numpy

from carla_bridge.sensor.sensor import Sensor
from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointXYZIT, PointCloud


def _carla_intensity_to_apollo(raw_intensity):
    if not numpy.isfinite(raw_intensity) or raw_intensity <= 0:
        return 0

    intensity = float(raw_intensity)
    if intensity <= 1.0:
        intensity *= 255.0
    return int(max(0, min(255, round(intensity))))


class Lidar(Sensor):

    """
    Actor implementation details for lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Lidar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

        self.lidar_writer = node.new_writer(self.get_topic_prefix() + "/compensator/PointCloud2",
                                            PointCloud,
                                            qos_depth=5)
        lidar_num_count=0
        self.listen()

    def destroy(self):
        super(Lidar, self).destroy()

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/sensor/" + self.name

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        # Convert CARLA lidar's left-handed sensor frame to Apollo/Dreamview's
        # local vehicle convention: +X forward, +Y left, +Z up.
        lidar_data[:, 1] *= -1
        #
        # Previous 90-degree compensation kept for reference. It made the point
        # cloud roughly centered with the old mounting, but rotates the cloud
        # perpendicular to the vehicle once the lidar is mounted at yaw=0:
        # raw_x = lidar_data[:, 0].copy()
        # raw_y = lidar_data[:, 1].copy()
        # lidar_data[:, 0] = -raw_y
        # lidar_data[:, 1] = -raw_x

        # lidar_data = lidar_data[:, :3]
        # lidar_data[:, 0:2] *= -1
        # we also need to permute x and y
        # lidar_data = lidar_data[..., [1, 0, 2]]

        point_cloud_msg = PointCloud()
        #stamp = self.get_sensor_stamp(carla_lidar_measurement)
        stamp_ns = self.node.get_clock_ns(frame_id=carla_lidar_measurement.frame)
        stamp = stamp_ns / 1000000000.0
        point_cloud_msg.header.CopyFrom(
            self.get_sensor_msg_header(
                sensor_timestamp_field="lidar_timestamp",
                timestamp=stamp,
                time_frame_id=carla_lidar_measurement.frame,
                sensor_timestamp_ns=stamp_ns,
            )
        )

        lidar_id = self.carla_actor.attributes['role_name']

        point_cloud_msg.header.frame_id = lidar_id
        point_cloud_msg.frame_id = lidar_id

        point_cloud_msg.measurement_time = stamp

        for lidar_point in lidar_data:
            cyber_point = PointXYZIT()
            cyber_point.x = lidar_point[0]
            cyber_point.y = lidar_point[1]
            cyber_point.z = lidar_point[2]
            intensity = _carla_intensity_to_apollo(lidar_point[3])
            if intensity:
                cyber_point.intensity = intensity
            point_cloud_msg.point.append(cyber_point)

        self.lidar_writer.write(point_cloud_msg)


class SemanticLidar(Sensor):

    """
    Actor implementation details for semantic lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(SemanticLidar, self).__init__(uid=uid,
                                            name=name,
                                            parent=parent,
                                            relative_spawn_pose=relative_spawn_pose,
                                            node=node,
                                            carla_actor=carla_actor,
                                            synchronous_mode=synchronous_mode)

        self.semantic_lidar_writer = node.new_writer(
            "/apollo/sensor/lidar32/semantic/PointCloud2",
            PointCloud,
            qos_depth=10)
        self.listen()

    def destroy(self):
        super(SemanticLidar, self).destroy()

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform a received semantic lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla semantic lidar measurement object
        :type carla_lidar_measurement: carla.SemanticLidarMeasurement
        """
        lidar_data = numpy.fromstring(bytes(carla_lidar_measurement.raw_data),
                                      dtype=numpy.dtype([
                                          ('x', numpy.float32),
                                          ('y', numpy.float32),
                                          ('z', numpy.float32),
                                          ('CosAngle', numpy.float32),
                                          ('ObjIdx', numpy.uint32),
                                          ('ObjTag', numpy.uint32)
                                      ]))

        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1

        point_cloud_msg = PointCloud()
        stamp_ns = self.node.get_clock_ns(frame_id=carla_lidar_measurement.frame)
        stamp = stamp_ns / 1000000000.0
        point_cloud_msg.header.CopyFrom(
            self.get_sensor_msg_header(
                sensor_timestamp_field="lidar_timestamp",
                timestamp=stamp,
                time_frame_id=carla_lidar_measurement.frame,
                sensor_timestamp_ns=stamp_ns,
            )
        )

        lidar_id = self.carla_actor.attributes['role_name']

        point_cloud_msg.header.frame_id = lidar_id
        point_cloud_msg.frame_id = lidar_id

        point_cloud_msg.measurement_time = stamp

        for lidar_point in lidar_data:
            cyber_point = PointXYZIT()
            cyber_point.x = lidar_point[0]
            cyber_point.y = lidar_point[1]
            cyber_point.z = lidar_point[2]
            point_cloud_msg.point.append(cyber_point)

        self.semantic_lidar_writer.write(point_cloud_msg)
        
