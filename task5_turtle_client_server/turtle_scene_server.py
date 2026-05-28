from threading import Event
from typing import Any

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robotech_msgs.srv import TurtleSceneCommand
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, Spawn


class TurtleSceneServer(Node):
    def __init__(self) -> None:
        super().__init__('turtle_scene_server')

        self._callback_group = ReentrantCallbackGroup()
        self._scene_service = self.create_service(
            TurtleSceneCommand,
            '/turtle/scene_command',
            self._handle_scene_command,
            callback_group=self._callback_group,
        )
        self._spawn_client = self.create_client(
            Spawn,
            '/spawn',
            callback_group=self._callback_group,
        )
        self._clear_client = self.create_client(
            Empty,
            '/clear',
            callback_group=self._callback_group,
        )
        self._set_pen_clients: dict[str, Any] = {}

        self.get_logger().info('Turtle scene service server started')

    def _handle_scene_command(
        self,
        request: TurtleSceneCommand.Request,
        response: TurtleSceneCommand.Response,
    ) -> TurtleSceneCommand.Response:
        command = request.command.strip().lower()
        turtle_name = request.turtle_name.strip()

        self.get_logger().info(
            'Request: command=%s turtle=%s' % (command, turtle_name or '<default>')
        )

        try:
            if command == 'spawn':
                response = self._spawn_turtle(request, response)
            elif command == 'set_color':
                response = self._set_pen_color(request, response)
            elif command == 'clear':
                response = self._clear_field(response)
            else:
                response.success = False
                response.message = 'unknown command: %s' % command
                response.turtle_name = turtle_name
        except Exception as exc:  # pragma: no cover - runtime ROS transport error
            response.success = False
            response.message = 'command failed: %s' % exc
            response.turtle_name = turtle_name

        self.get_logger().info(
            'Response: success=%s message=%s turtle=%s'
            % (response.success, response.message, response.turtle_name)
        )
        return response

    def _spawn_turtle(
        self,
        request: TurtleSceneCommand.Request,
        response: TurtleSceneCommand.Response,
    ) -> TurtleSceneCommand.Response:
        self._wait_for_service(self._spawn_client, '/spawn')

        spawn_request = Spawn.Request()
        spawn_request.x = request.x if request.x > 0.0 else 2.0
        spawn_request.y = request.y if request.y > 0.0 else 2.0
        spawn_request.theta = request.theta
        spawn_request.name = request.turtle_name.strip() or 'turtle2'

        spawn_response = self._call_service(self._spawn_client, spawn_request)

        response.success = True
        response.message = 'spawned turtle %s' % spawn_response.name
        response.turtle_name = spawn_response.name
        return response

    def _set_pen_color(
        self,
        request: TurtleSceneCommand.Request,
        response: TurtleSceneCommand.Response,
    ) -> TurtleSceneCommand.Response:
        turtle_name = request.turtle_name.strip() or 'turtle1'
        service_name = '/%s/set_pen' % turtle_name
        set_pen_client = self._set_pen_clients.get(service_name)

        if set_pen_client is None:
            set_pen_client = self.create_client(
                SetPen,
                service_name,
                callback_group=self._callback_group,
            )
            self._set_pen_clients[service_name] = set_pen_client

        self._wait_for_service(set_pen_client, service_name)

        set_pen_request = SetPen.Request()
        if request.r == 0 and request.g == 0 and request.b == 0:
            set_pen_request.r = 255
            set_pen_request.g = 0
            set_pen_request.b = 0
        else:
            set_pen_request.r = request.r
            set_pen_request.g = request.g
            set_pen_request.b = request.b
        set_pen_request.width = request.pen_width if request.pen_width > 0 else 3
        set_pen_request.off = 0

        self._call_service(set_pen_client, set_pen_request)

        response.success = True
        response.message = (
            'set %s pen color to rgb(%d,%d,%d)'
            % (turtle_name, set_pen_request.r, set_pen_request.g, set_pen_request.b)
        )
        response.turtle_name = turtle_name
        return response

    def _clear_field(
        self,
        response: TurtleSceneCommand.Response,
    ) -> TurtleSceneCommand.Response:
        self._wait_for_service(self._clear_client, '/clear')
        self._call_service(self._clear_client, Empty.Request())

        response.success = True
        response.message = 'cleared TurtleSim field'
        response.turtle_name = ''
        return response

    def _wait_for_service(self, client: Any, service_name: str) -> None:
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('service %s is not available' % service_name)

    def _call_service(self, client: Any, request: Any) -> Any:
        future = client.call_async(request)
        done_event = Event()
        future.add_done_callback(lambda _: done_event.set())

        if not done_event.wait(timeout=5.0):
            raise RuntimeError('service call timed out')

        exception = future.exception()
        if exception is not None:
            raise exception

        return future.result()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TurtleSceneServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
