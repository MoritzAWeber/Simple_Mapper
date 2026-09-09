#!/usr/bin/env python3

import math

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class LaserSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__('laser_simulator')

        self.get_logger().info(
            'Laser Simulator Node wurde gestartet.'
        )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odometry = False

        self.room_min_x = -4.0
        self.room_max_x = 4.0
        self.room_min_y = -4.0
        self.room_max_y = 4.0

        self.timer_period = 0.2

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10,
        )

        self.ray_marker_publisher = self.create_publisher(
            Marker,
            '/scan_rays',
            10,
        )

        self.timer = self.create_timer(
            self.timer_period,
            self.sensor_callback,
        )

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self.robot_x = message.pose.pose.position.x
        self.robot_y = message.pose.pose.position.y

        orientation = message.pose.pose.orientation

        self.robot_yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

        self.has_odometry = True

    def sensor_callback(self) -> None:
        if not self.has_odometry:
            return

        scan = self.publish_scan()

        self.publish_ray_marker(scan)

    def publish_scan(self) -> LaserScan:
        scan = LaserScan()

        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        measurement_count = 360

        scan.angle_min = -math.pi
        scan.angle_increment = (
            2.0 * math.pi / measurement_count
        )
        scan.angle_max = (
            scan.angle_min
            + (measurement_count - 1)
            * scan.angle_increment
        )

        scan.range_min = 0.1
        scan.range_max = 8.0

        scan.scan_time = self.timer_period
        scan.time_increment = (
            scan.scan_time / measurement_count
        )

        ranges: list[float] = []

        for index in range(measurement_count):
            local_angle = (
                scan.angle_min
                + index * scan.angle_increment
            )

            world_angle = (
                self.robot_yaw + local_angle
            )

            direction_x = math.cos(world_angle)
            direction_y = math.sin(world_angle)

            distance = self.cast_ray_to_walls(
                origin_x=self.robot_x,
                origin_y=self.robot_y,
                direction_x=direction_x,
                direction_y=direction_y,
                maximum_distance=scan.range_max,
            )

            ranges.append(distance)

        scan.ranges = ranges
        scan.intensities = []
        self.scan_publisher.publish(scan)

        return scan

    def publish_ray_marker(
        self,
        scan: LaserScan,
    ) -> None:
        marker = Marker()

        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = 'laser_frame'

        marker.ns = 'lidar_rays'
        marker.id = 0

        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.01

        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.7

        marker.pose.orientation.w = 1.0

        for index, distance in enumerate(scan.ranges):
            if index % 10 != 0:
                continue

            if not math.isfinite(distance):
                continue

            angle = (
                scan.angle_min
                + index * scan.angle_increment
            )

            start = Point()
            start.x = 0.0
            start.y = 0.0
            start.z = 0.0

            end = Point()
            end.x = distance * math.cos(angle)
            end.y = distance * math.sin(angle)
            end.z = 0.0

            marker.points.append(start)
            marker.points.append(end)

        self.ray_marker_publisher.publish(marker)

    def cast_ray_to_walls(
        self,
        origin_x: float,
        origin_y: float,
        direction_x: float,
        direction_y: float,
        maximum_distance: float,
    ) -> float:
        distances: list[float] = []
        epsilon = 1e-9

        if abs(direction_x) > epsilon:
            for wall_x in (
                self.room_min_x,
                self.room_max_x,
            ):
                distance = (
                    wall_x - origin_x
                ) / direction_x

                intersection_y = (
                    origin_y
                    + distance * direction_y
                )

                if (
                    distance > 0.0
                    and self.room_min_y
                    <= intersection_y
                    <= self.room_max_y
                ):
                    distances.append(distance)

        if abs(direction_y) > epsilon:
            for wall_y in (
                self.room_min_y,
                self.room_max_y,
            ):
                distance = (
                    wall_y - origin_y
                ) / direction_y

                intersection_x = (
                    origin_x
                    + distance * direction_x
                )

                if (
                    distance > 0.0
                    and self.room_min_x
                    <= intersection_x
                    <= self.room_max_x
                ):
                    distances.append(distance)

        valid_distances = [
            distance
            for distance in distances
            if distance <= maximum_distance
        ]

        if not valid_distances:
            return maximum_distance

        return min(valid_distances)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = LaserSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
