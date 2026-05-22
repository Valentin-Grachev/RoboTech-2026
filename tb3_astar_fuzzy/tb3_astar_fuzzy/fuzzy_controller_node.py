import math
from dataclasses import dataclass
from typing import Iterable

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class FuzzyRule:
    weight: float
    linear_speed: float
    angular_speed: float


class FuzzyControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('fuzzy_controller')

        self._latest_scan: LaserScan | None = None
        self._waiting_for_scan_logged = False

        self._max_distance = 3.5
        self._max_linear_speed = 0.14
        self._min_linear_speed = 0.02
        self._turn_speed = 0.55
        self._last_linear_speed = 0.0
        self._last_angular_speed = 0.0
        self._smoothing_factor = 0.35

        self._cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self._scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._control_timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('Fuzzy controller started')

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self._cmd_vel_publisher.publish(self._build_command_message())
        return super().destroy_node()

    def _scan_callback(self, scan_message: LaserScan) -> None:
        self._latest_scan = scan_message
        self._waiting_for_scan_logged = False

    def _control_loop(self) -> None:
        if self._latest_scan is None:
            if not self._waiting_for_scan_logged:
                self.get_logger().warning('Waiting for /scan messages...')
                self._waiting_for_scan_logged = True
            return

        front = self._front_min_distance(self._latest_scan)
        left = self._sector_min_distance(self._latest_scan, 35.0, 95.0)
        right = self._sector_min_distance(self._latest_scan, -95.0, -35.0)

        command = self._defuzzify(front, left, right)
        self._cmd_vel_publisher.publish(command)

    def _front_min_distance(self, scan_message: LaserScan) -> float:
        if not scan_message.ranges:
            return self._max_distance

        window_size = max(1, len(scan_message.ranges) // 12)
        front_ranges = list(scan_message.ranges[:window_size]) + list(scan_message.ranges[-window_size:])
        return self._valid_min_distance(front_ranges, scan_message)

    def _sector_min_distance(
        self,
        scan_message: LaserScan,
        start_degrees: float,
        end_degrees: float,
    ) -> float:
        distances: list[float] = []
        start_angle = math.radians(start_degrees)
        end_angle = math.radians(end_degrees)

        for index, distance in enumerate(scan_message.ranges):
            angle = scan_message.angle_min + index * scan_message.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if start_angle <= angle <= end_angle:
                distances.append(distance)

        return self._valid_min_distance(distances, scan_message)

    def _valid_min_distance(self, distances: Iterable[float], scan_message: LaserScan) -> float:
        valid_distances = [
            min(distance, self._max_distance, scan_message.range_max)
            for distance in distances
            if math.isfinite(distance) and distance >= scan_message.range_min
        ]

        if not valid_distances:
            return self._max_distance

        return min(valid_distances)

    def _defuzzify(self, front: float, left: float, right: float) -> TwistStamped:
        front_near = self._near(front)
        front_medium = self._medium(front)
        front_far = self._far(front)
        left_near = self._near(left)
        right_near = self._near(right)

        turn_to_free_side = self._turn_speed if left > right else -self._turn_speed
        side_balance = max(-1.0, min(1.0, (left - right) / self._max_distance))

        rules = [
            FuzzyRule(front_near, self._min_linear_speed, turn_to_free_side),
            FuzzyRule(front_medium, 0.08, side_balance * self._turn_speed),
            FuzzyRule(front_far, self._max_linear_speed, side_balance * self._turn_speed * 0.55),
            FuzzyRule(left_near * (1.0 - front_near), 0.08, -self._turn_speed * 0.45),
            FuzzyRule(right_near * (1.0 - front_near), 0.08, self._turn_speed * 0.45),
        ]

        total_weight = sum(rule.weight for rule in rules)
        command = self._build_command_message()

        if total_weight <= 1e-6:
            command.twist.linear.x = self._smooth_linear_speed(0.08)
            command.twist.angular.z = self._smooth_angular_speed(0.0)
            return command

        linear_speed = self._weighted_average(
            (rule.linear_speed for rule in rules),
            (rule.weight for rule in rules),
            total_weight,
        )
        angular_speed = self._weighted_average(
            (rule.angular_speed for rule in rules),
            (rule.weight for rule in rules),
            total_weight,
        )
        command.twist.linear.x = self._smooth_linear_speed(linear_speed)
        command.twist.angular.z = self._smooth_angular_speed(angular_speed)
        return command

    def _build_command_message(self) -> TwistStamped:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        return command

    def _near(self, distance: float) -> float:
        return self._descending(distance, 0.30, 0.75)

    def _medium(self, distance: float) -> float:
        return self._triangle(distance, 0.45, 1.00, 1.70)

    def _far(self, distance: float) -> float:
        return self._ascending(distance, 1.20, 2.20)

    def _descending(self, value: float, full_membership: float, zero_membership: float) -> float:
        if value <= full_membership:
            return 1.0
        if value >= zero_membership:
            return 0.0
        return (zero_membership - value) / (zero_membership - full_membership)

    def _ascending(self, value: float, zero_membership: float, full_membership: float) -> float:
        if value <= zero_membership:
            return 0.0
        if value >= full_membership:
            return 1.0
        return (value - zero_membership) / (full_membership - zero_membership)

    def _triangle(self, value: float, left: float, peak: float, right: float) -> float:
        if value <= left or value >= right:
            return 0.0
        if value == peak:
            return 1.0
        if value < peak:
            return (value - left) / (peak - left)
        return (right - value) / (right - peak)

    def _weighted_average(
        self,
        values: Iterable[float],
        weights: Iterable[float],
        total_weight: float,
    ) -> float:
        return sum(value * weight for value, weight in zip(values, weights)) / total_weight

    def _smooth_linear_speed(self, target_speed: float) -> float:
        clamped_speed = max(0.0, min(self._max_linear_speed, target_speed))
        self._last_linear_speed = self._smooth(self._last_linear_speed, clamped_speed)
        return self._last_linear_speed

    def _smooth_angular_speed(self, target_speed: float) -> float:
        clamped_speed = max(-self._turn_speed, min(self._turn_speed, target_speed))
        self._last_angular_speed = self._smooth(self._last_angular_speed, clamped_speed)
        return self._last_angular_speed

    def _smooth(self, previous_value: float, target_value: float) -> float:
        return previous_value + self._smoothing_factor * (target_value - previous_value)


def main(args=None) -> None:
    rclpy.init(args=args)
    fuzzy_controller_node = FuzzyControllerNode()

    try:
        rclpy.spin(fuzzy_controller_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        fuzzy_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
