from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleTwistPublisher(Node):
    FORWARD = 'forward'
    TURN = 'turn'

    def __init__(self) -> None:
        super().__init__('turtle_twist_publisher')

        self._publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._timer = self.create_timer(0.1, self._publish_command)

        self._state = self.FORWARD
        self._state_started_at = monotonic()
        self._forward_duration = 2.0
        self._turn_duration = 1.0
        self._linear_speed = 2.0
        self._angular_speed = 1.57

        self.get_logger().info('Turtle Twist publisher started: drawing a square')

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self._publisher.publish(Twist())
        return super().destroy_node()

    def _publish_command(self) -> None:
        elapsed = monotonic() - self._state_started_at

        if self._state == self.FORWARD and elapsed >= self._forward_duration:
            self._switch_state(self.TURN)
        elif self._state == self.TURN and elapsed >= self._turn_duration:
            self._switch_state(self.FORWARD)

        command = Twist()
        if self._state == self.FORWARD:
            command.linear.x = self._linear_speed
        else:
            command.angular.z = self._angular_speed

        self._publisher.publish(command)

    def _switch_state(self, state: str) -> None:
        self._state = state
        self._state_started_at = monotonic()
        self.get_logger().info('Motion state: %s' % state)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TurtleTwistPublisher()

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
