import rclpy
from rclpy.node import Node

from task2_interfaces.srv import Task2Command


class CustomServiceServer(Node):
    def __init__(self) -> None:
        super().__init__("custom_service_server")
        self._service = self.create_service(
            Task2Command,
            "/task2/command_service",
            self._handle_request,
        )
        self.get_logger().info("Service server is ready: /task2/command_service")

    def _handle_request(
        self, request: Task2Command.Request, response: Task2Command.Response
    ) -> Task2Command.Response:
        command = request.command.strip().lower()
        value = request.value

        if command == "double":
            response.result = value * 2
            response.success = True
            response.message = f"Command double executed for value={value}"
        elif command == "square":
            response.result = value * value
            response.success = True
            response.message = f"Command square executed for value={value}"
        else:
            response.result = 0
            response.success = False
            response.message = (
                f"Unknown command '{request.command}'. Use 'double' or 'square'."
            )

        self.get_logger().info(
            "Request: command=%s value=%d -> Response: success=%s result=%d message='%s'"
            % (
                request.command,
                request.value,
                response.success,
                response.result,
                response.message,
            )
        )
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CustomServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
