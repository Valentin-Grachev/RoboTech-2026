import rclpy
from rclpy.node import Node

from task1_msgs.msg import CustomMessage


class CustomSubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("custom_subscriber")
        self._subscription = self.create_subscription(
            CustomMessage,
            "/task1/custom_topic",
            self._message_callback,
            10,
        )
        self.get_logger().info("Custom subscriber started")

    def _message_callback(self, message: CustomMessage) -> None:
        self.get_logger().info(
            "Received: id=%d publisher=%s payload='%s' temp=%.1f critical=%s frame=%s"
            % (
                message.id,
                message.publisher_name,
                message.payload,
                message.temperature_c,
                message.is_critical,
                message.header.frame_id,
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CustomSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
