import sys

import rclpy
from rclpy.node import Node

from task2_interfaces.srv import Task2Command


class CustomServiceClient(Node):
    def __init__(self) -> None:
        super().__init__("custom_service_client")
        self._client = self.create_client(Task2Command, "/task2/command_service")
        self.get_logger().info("Waiting for service /task2/command_service...")
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service is not available yet, waiting...")
        self.get_logger().info("Service is available")

    def send_request(self, command: str, value: int) -> Task2Command.Response:
        request = Task2Command.Request()
        request.command = command
        request.value = value

        self.get_logger().info(
            "Sending request: command=%s value=%d" % (request.command, request.value)
        )
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CustomServiceClient()

    command = "double"
    value = 7
    if args is None:
        args = sys.argv[1:]
    if len(args) >= 1:
        command = args[0]
    if len(args) >= 2:
        value = int(args[1])

    try:
        response = node.send_request(command, value)
        if response is None:
            node.get_logger().error("Service call failed: empty response")
        else:
            node.get_logger().info(
                "Response: success=%s result=%d message='%s'"
                % (response.success, response.result, response.message)
            )
    except Exception as exc:
        node.get_logger().error(f"Service call failed: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
