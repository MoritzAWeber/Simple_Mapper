import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster


class RobotMotionNode(Node):
    def __init__(self):
        super().__init__('robot_motion_node')

        self.get_logger().info('Robot Motion Node wurde gestartet.')

        self.timer_period = 0.2
        self.simulation_time = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_laser_transform()

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(self.timer_period, self.motion_callback)

    def motion_callback(self) -> None:
        self.simulation_time += self.timer_period

        radius = 1.5
        angular_speed = 0.12

        angle = angular_speed * self.simulation_time

        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        yaw = angle + math.pi / 2.0

        stamp = self.get_clock().now().to_msg()

        self.publish_odometry(
            x=x,
            y=y,
            yaw=yaw,
            stamp=stamp,
        )

        self.publish_transform(
            x=x,
            y=y,
            yaw=yaw,
            stamp=stamp,
        )

    def publish_odometry(
        self,
        x: float,
        y: float,
        yaw: float,
        stamp,
    ) -> None:
        message = Odometry()

        message.header.stamp = stamp
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'

        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = 0.0

        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.odom_publisher.publish(message)

    def publish_transform(
        self,
        x: float,
        y: float,
        yaw: float,
        stamp,
    ) -> None:
        transform = TransformStamped()

        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)

        self.tf_broadcaster.sendTransform(transform)

    def publish_laser_transform(self) -> None:
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_frame'

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = RobotMotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
