import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtlePoseSubscriber(Node):
    def __init__(self) -> None:
        super().__init__('turtle_pose_subscriber')

        self._subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self._pose_callback,
            10,
        )
        self._last_log_time_ns: int | None = None
        self._log_period_ns = 1_000_000_000

        self.get_logger().info('Turtle pose subscriber started')

    def _pose_callback(self, pose: Pose) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._last_log_time_ns is not None
            and now_ns - self._last_log_time_ns < self._log_period_ns
        ):
            return

        self._last_log_time_ns = now_ns
        self.get_logger().info(
            'Pose: x=%.2f y=%.2f theta=%.2f linear=%.2f angular=%.2f'
            % (
                pose.x,
                pose.y,
                pose.theta,
                pose.linear_velocity,
                pose.angular_velocity,
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TurtlePoseSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
