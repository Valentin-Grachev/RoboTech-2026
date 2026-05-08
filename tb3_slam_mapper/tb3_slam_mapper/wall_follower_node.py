from math import isfinite, sqrt
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WallFollowerNode(Node):
    FIND_WALL = 'find_wall'
    TURN = 'turn'
    FOLLOW = 'follow'

    def __init__(self) -> None:
        super().__init__('wall_follower')

        self._state = self.FIND_WALL
        self._latest_scan: LaserScan | None = None
        self._path_message = Path()
        self._last_path_x: float | None = None
        self._last_path_y: float | None = None
        self._waiting_for_scan_logged = False

        self._forward_speed = 0.15
        self._turn_speed = 0.7
        self._front_obstacle_distance = 0.55
        self._front_clear_distance = 0.75
        self._path_distance_threshold = 0.05

        self._cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self._path_publisher = self.create_publisher(Path, '/wall_follower/path', 10)

        self._scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
        )
        self._control_timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('Wall follower started in state: find_wall')

    def destroy_node(self) -> bool:
        self._cmd_vel_publisher.publish(self._build_command_message())
        return super().destroy_node()

    def _scan_callback(self, scan_message: LaserScan) -> None:
        self._latest_scan = scan_message
        self._waiting_for_scan_logged = False

    def _odom_callback(self, odom_message: Odometry) -> None:
        pose = odom_message.pose.pose
        position = pose.position

        if not self._should_append_pose(position.x, position.y):
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = odom_message.header
        pose_stamped.pose = pose

        frame_id = odom_message.header.frame_id or 'odom'
        self._path_message.header.stamp = odom_message.header.stamp
        self._path_message.header.frame_id = frame_id
        self._path_message.poses.append(pose_stamped)
        self._path_publisher.publish(self._path_message)

        self._last_path_x = position.x
        self._last_path_y = position.y

    def _should_append_pose(self, position_x: float, position_y: float) -> bool:
        if self._last_path_x is None or self._last_path_y is None:
            return True

        delta_x = position_x - self._last_path_x
        delta_y = position_y - self._last_path_y
        return sqrt(delta_x * delta_x + delta_y * delta_y) >= self._path_distance_threshold

    def _control_loop(self) -> None:
        if self._latest_scan is None:
            if not self._waiting_for_scan_logged:
                self.get_logger().warning('Waiting for /scan messages...')
                self._waiting_for_scan_logged = True
            return

        front_distance = self._get_front_distance(self._latest_scan)

        if self._state == self.FIND_WALL:
            if front_distance < self._front_obstacle_distance:
                self._set_state(self.TURN)
                self._publish_turn_command()
                return

            self._set_state(self.FOLLOW)
            self._publish_forward_command()
            return

        if self._state == self.TURN:
            if front_distance > self._front_clear_distance:
                self._set_state(self.FOLLOW)
                self._publish_forward_command()
                return

            self._publish_turn_command()
            return

        if front_distance < self._front_obstacle_distance:
            self._set_state(self.TURN)
            self._publish_turn_command()
            return

        self._publish_forward_command()

    def _get_front_distance(self, scan_message: LaserScan) -> float:
        if not scan_message.ranges:
            return float('inf')

        front_ranges = self._extract_front_ranges(scan_message.ranges)
        valid_ranges = [distance for distance in front_ranges if isfinite(distance)]

        if not valid_ranges:
            return float('inf')

        return min(valid_ranges)

    def _extract_front_ranges(self, ranges: List[float]) -> List[float]:
        if len(ranges) < 6:
            return list(ranges)

        window_size = max(1, len(ranges) // 12)
        return list(ranges[:window_size]) + list(ranges[-window_size:])

    def _set_state(self, new_state: str) -> None:
        if self._state == new_state:
            return

        previous_state = self._state
        self._state = new_state
        self.get_logger().info(f'FSM transition: {previous_state} -> {new_state}')

    def _publish_forward_command(self) -> None:
        command = self._build_command_message()
        command.twist.linear.x = self._forward_speed
        self._cmd_vel_publisher.publish(command)

    def _publish_turn_command(self) -> None:
        command = self._build_command_message()
        command.twist.angular.z = self._turn_speed
        self._cmd_vel_publisher.publish(command)

    def _build_command_message(self) -> TwistStamped:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        return command


def main(args=None) -> None:
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()

    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
