import math
from typing import Iterator

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SlamMapperNode(Node):
    def __init__(self) -> None:
        super().__init__('slam_mapper')

        self._map_frame_id = 'map'
        self._resolution = 0.05
        self._width = 400
        self._height = 400
        self._origin_x = -(self._width * self._resolution) / 2.0
        self._origin_y = -(self._height * self._resolution) / 2.0

        self._log_odds_free = -0.35
        self._log_odds_occupied = 0.85
        self._log_odds_min = -4.0
        self._log_odds_max = 4.0
        self._unknown_band = 0.05

        self._max_range = 3.5
        self._beam_stride = 2

        self._log_odds_grid = [0.0] * (self._width * self._height)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._pose_ready = False
        self._scan_ready = False

        self._map_publisher = self.create_publisher(OccupancyGrid, '/slam_map', 10)
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
        self._map_timer = self.create_timer(0.25, self._publish_map)

        self.get_logger().info('SLAM mapper started: occupancy grid + log-odds + Bresenham')

    def _odom_callback(self, odom_message: Odometry) -> None:
        pose = odom_message.pose.pose
        orientation = pose.orientation

        self._robot_x = pose.position.x
        self._robot_y = pose.position.y
        self._robot_yaw = self._quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self._pose_ready = True

    def _scan_callback(self, scan_message: LaserScan) -> None:
        if not self._pose_ready:
            return

        robot_cell = self._world_to_grid(self._robot_x, self._robot_y)
        if robot_cell is None:
            return

        robot_cell_x, robot_cell_y = robot_cell

        for beam_index in range(0, len(scan_message.ranges), self._beam_stride):
            beam_range = scan_message.ranges[beam_index]
            if not math.isfinite(beam_range):
                continue

            is_hit = beam_range < scan_message.range_max
            clipped_range = min(beam_range, self._max_range, scan_message.range_max)

            if clipped_range < scan_message.range_min:
                continue

            beam_angle = (
                scan_message.angle_min
                + beam_index * scan_message.angle_increment
                + self._robot_yaw
            )
            hit_x = self._robot_x + clipped_range * math.cos(beam_angle)
            hit_y = self._robot_y + clipped_range * math.sin(beam_angle)

            hit_cell = self._world_to_grid(hit_x, hit_y)
            if hit_cell is None:
                continue

            hit_cell_x, hit_cell_y = hit_cell
            ray_cells = list(
                self._bresenham_line(robot_cell_x, robot_cell_y, hit_cell_x, hit_cell_y)
            )
            if not ray_cells:
                continue

            # Every traversed cell except last one is observed as free space.
            for free_cell_x, free_cell_y in ray_cells[:-1]:
                self._update_log_odds(free_cell_x, free_cell_y, self._log_odds_free)

            if is_hit:
                occupied_cell_x, occupied_cell_y = ray_cells[-1]
                self._update_log_odds(
                    occupied_cell_x,
                    occupied_cell_y,
                    self._log_odds_occupied,
                )

        self._scan_ready = True

    def _publish_map(self) -> None:
        if not self._scan_ready:
            return

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = self._map_frame_id
        occupancy_grid.info.resolution = self._resolution
        occupancy_grid.info.width = self._width
        occupancy_grid.info.height = self._height
        occupancy_grid.info.origin.position.x = self._origin_x
        occupancy_grid.info.origin.position.y = self._origin_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        occupancy_grid.data = [self._log_odds_to_occupancy(value) for value in self._log_odds_grid]
        self._map_publisher.publish(occupancy_grid)

    def _world_to_grid(self, world_x: float, world_y: float) -> tuple[int, int] | None:
        grid_x = int((world_x - self._origin_x) / self._resolution)
        grid_y = int((world_y - self._origin_y) / self._resolution)

        if grid_x < 0 or grid_x >= self._width:
            return None
        if grid_y < 0 or grid_y >= self._height:
            return None

        return grid_x, grid_y

    def _grid_to_index(self, grid_x: int, grid_y: int) -> int:
        return grid_y * self._width + grid_x

    def _update_log_odds(self, grid_x: int, grid_y: int, log_odds_delta: float) -> None:
        map_index = self._grid_to_index(grid_x, grid_y)
        updated_value = self._log_odds_grid[map_index] + log_odds_delta
        self._log_odds_grid[map_index] = max(self._log_odds_min, min(self._log_odds_max, updated_value))

    def _log_odds_to_occupancy(self, log_odds_value: float) -> int:
        if abs(log_odds_value) < self._unknown_band:
            return -1

        probability = 1.0 - (1.0 / (1.0 + math.exp(log_odds_value)))
        return max(0, min(100, int(round(probability * 100.0))))

    def _quaternion_to_yaw(self, quat_x: float, quat_y: float, quat_z: float, quat_w: float) -> float:
        sin_yaw = 2.0 * (quat_w * quat_z + quat_x * quat_y)
        cos_yaw = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z)
        return math.atan2(sin_yaw, cos_yaw)

    def _bresenham_line(
        self,
        start_x: int,
        start_y: int,
        end_x: int,
        end_y: int,
    ) -> Iterator[tuple[int, int]]:
        delta_x = abs(end_x - start_x)
        delta_y = abs(end_y - start_y)
        step_x = 1 if start_x < end_x else -1
        step_y = 1 if start_y < end_y else -1

        current_x = start_x
        current_y = start_y
        error = delta_x - delta_y

        while True:
            yield current_x, current_y
            if current_x == end_x and current_y == end_y:
                break

            doubled_error = error * 2
            if doubled_error > -delta_y:
                error -= delta_y
                current_x += step_x
            if doubled_error < delta_x:
                error += delta_x
                current_y += step_y


def main(args=None) -> None:
    rclpy.init(args=args)
    slam_mapper_node = SlamMapperNode()

    try:
        rclpy.spin(slam_mapper_node)
    except KeyboardInterrupt:
        pass

    slam_mapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
