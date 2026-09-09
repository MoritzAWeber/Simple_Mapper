import math

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimpleMapperNode(Node):
    def __init__(self) -> None:
        super().__init__('simple_mapper')

        self.get_logger().info('Simple Mapper Node wurde gestartet.')

        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False

        self.grid_resolution = 0.05
        self.map_size = 12.0
        self.origin_x = -6.0
        self.origin_y = -6.0
        self.width = int(self.map_size / self.grid_resolution)
        self.height = int(self.map_size / self.grid_resolution)

        self.occupancy_grid = -1 * np.ones(
            (self.height, self.width), dtype=int)

        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        self.robot_x = position.x
        self.robot_y = position.y

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

        self.has_odom = True

        self.get_logger().info(
            'Robot at position '
            f'x={message.pose.pose.position.x:.2f}, '
            f'y={message.pose.pose.position.y:.2f}, '
        )

    def scan_callback(self, message: LaserScan) -> None:
        if not self.has_odom:
            return

        ranges = message.ranges
        angles = (
            message.angle_min
            + np.arange(len(ranges)) * message.angle_increment
        )
        points = []

        for angle, distance in zip(angles, ranges):

            if not math.isfinite(distance):
                continue

            if distance < message.range_min:
                continue

            if distance > message.range_max:
                continue

            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            points.append((x, y))

        for point in points:
            point_x, point_y = point
            x_world = (
                self.robot_x
                + math.cos(self.robot_yaw) * point_x
                - math.sin(self.robot_yaw) * point_y
            )
            y_world = (
                self.robot_y
                + math.sin(self.robot_yaw) * point_x
                + math.cos(self.robot_yaw) * point_y
            )
            x_grid, y_grid = self.world_to_grid(x_world, y_world)
            if self.is_in_bounds(x_grid, y_grid):
                self.occupancy_grid[y_grid, x_grid] = 100

        self.publish_map()

        # if len(points) > 0:
        #     occupied_cells = np.sum(self.occupancy_grid == 100)

        #     self.get_logger().info(
        #         f"Occupied cells: {occupied_cells}"
        #     )

    def world_to_grid(self, x_world: float, y_world: float):
        x_grid = int((x_world - self.origin_x) / self.grid_resolution)
        y_grid = int((y_world - self.origin_y) / self.grid_resolution)
        return x_grid, y_grid

    def is_in_bounds(self, x_grid: int, y_grid: int):
        x_in_bounds = x_grid >= 0 and x_grid < self.width
        y_in_bounds = y_grid >= 0 and y_grid < self.height
        return x_in_bounds and y_in_bounds

    def publish_map(self):
        map_message = OccupancyGrid()
        map_message.header.stamp = self.get_clock().now().to_msg()
        map_message.header.frame_id = 'odom'
        map_message.info.resolution = self.grid_resolution
        map_message.info.width = self.width
        map_message.info.height = self.height
        map_message.info.origin.position.x = self.origin_x
        map_message.info.origin.position.y = self.origin_y
        map_message.info.origin.position.z = 0.0
        map_message.info.origin.orientation.x = 0.0
        map_message.info.origin.orientation.y = 0.0
        map_message.info.origin.orientation.z = 0.0
        map_message.info.origin.orientation.w = 1.0
        map_message.data = self.occupancy_grid.flatten().tolist()
        self.map_publisher.publish(map_message)


def main(args=None):
    rclpy.init(args=args)

    node = SimpleMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
