import rclpy
from rclpy.node import Node

from task1_msgs.msg import CustomMessage


class CustomPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("custom_publisher")
        self._publisher = self.create_publisher(CustomMessage, "/task1/custom_topic", 10)
        self._timer = self.create_timer(1.0, self._publish_message)
        self._message_id = 1
        self.get_logger().info("Custom publisher started")

    def _publish_message(self) -> None:
        message = CustomMessage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "task1_frame"
        message.id = self._message_id
        message.publisher_name = "task1_publisher"
        message.payload = f"Hello from publisher message #{self._message_id}"
        message.temperature_c = 20.0 + (self._message_id % 5)
        message.is_critical = self._message_id % 7 == 0

        self._publisher.publish(message)
        self.get_logger().info(
            "Publishing: id=%d publisher=%s payload='%s' temp=%.1f critical=%s"
            % (
                message.id,
                message.publisher_name,
                message.payload,
                message.temperature_c,
                message.is_critical,
            )
        )
        self._message_id += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CustomPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
