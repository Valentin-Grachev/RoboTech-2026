import rclpy
from rclpy.node import Node
from robotech_msgs.srv import TurtleSceneCommand


class TurtleSceneClient(Node):
    def __init__(self) -> None:
        super().__init__('turtle_scene_client')
        self._client = self.create_client(TurtleSceneCommand, '/turtle/scene_command')
        self._spawn_timer = self.create_timer(5.0, self._send_spawn_command)
        self._color_timer = self.create_timer(2.5, self._send_color_command)
        self._next_turtle_number = 2
        self._color_index = 0

        self._colors = [
            (255, 0, 0),
            (0, 255, 0),
        ]

        self.get_logger().info(
            'Turtle scene client started: auto-spawn every 5s and color cycling.'
        )

    def _send_spawn_command(self) -> None:
        request = TurtleSceneCommand.Request()
        request.command = 'spawn'
        request.turtle_name = 'turtle%d' % self._next_turtle_number
        request.x = 2.0 + (self._next_turtle_number % 4)
        request.y = 2.0 + ((self._next_turtle_number + 1) % 4)
        request.theta = 0.8
        self._next_turtle_number += 1
        self._send_request(request)

    def _send_color_command(self) -> None:
        r, g, b = self._colors[self._color_index % len(self._colors)]
        self._color_index += 1

        request = TurtleSceneCommand.Request()
        request.command = 'set_color'
        request.turtle_name = 'turtle1'
        request.r = r
        request.g = g
        request.b = b
        request.pen_width = 5
        self._send_request(request)

    def _send_request(self, request: TurtleSceneCommand.Request) -> None:
        if not self._client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('Waiting for /turtle/scene_command service...')
            return

        self.get_logger().info(
            'Sending request: command=%s turtle=%s'
            % (request.command, request.turtle_name or '<default>')
        )
        future = self._client.call_async(request)
        future.add_done_callback(self._handle_response)

    def _handle_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime ROS transport error
            self.get_logger().error('Service call failed: %r' % exc)
            return

        self.get_logger().info(
            'Received response: success=%s message=%s turtle=%s'
            % (response.success, response.message, response.turtle_name or '<none>')
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TurtleSceneClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
