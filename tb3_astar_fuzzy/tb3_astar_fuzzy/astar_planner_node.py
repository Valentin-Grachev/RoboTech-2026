import heapq
import math
from itertools import count

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_msgs.srv import GetPlan
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

GridCell = tuple[int, int]


class AStarPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('astar_planner')

        self._latest_map: OccupancyGrid | None = None
        self._latest_pose: PoseStamped | None = None

        self._occupied_threshold = 50
        self._robot_radius = 0.18

        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            '/slam_map',
            self._map_callback,
            10,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
        )
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_callback,
            10,
        )
        self._path_publisher = self.create_publisher(Path, '/global_plan', 10)
        self._plan_service = self.create_service(GetPlan, '/plan_path', self._handle_plan_path)

        self.get_logger().info('A* planner started: service /plan_path, topic /global_plan')

    def _map_callback(self, map_message: OccupancyGrid) -> None:
        self._latest_map = map_message

    def _odom_callback(self, odom_message: Odometry) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_message.header
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = odom_message.pose.pose
        self._latest_pose = pose_stamped

    def _goal_callback(self, goal_message: PoseStamped) -> None:
        if self._latest_pose is None:
            self.get_logger().warning('Cannot plan from /goal_pose: waiting for /odom')
            return

        request = GetPlan.Request()
        request.start = self._latest_pose
        request.goal = goal_message
        request.tolerance = 0.0
        response = self._handle_plan_path(request, GetPlan.Response())

        if response.plan.poses:
            self.get_logger().info(f'Published plan with {len(response.plan.poses)} poses from /goal_pose')

    def _handle_plan_path(self, request: GetPlan.Request, response: GetPlan.Response) -> GetPlan.Response:
        if self._latest_map is None:
            self.get_logger().warning('Cannot plan path: waiting for /slam_map')
            return response

        map_message = self._latest_map
        start_cell = self._world_to_grid(request.start.pose.position.x, request.start.pose.position.y, map_message)
        goal_cell = self._world_to_grid(request.goal.pose.position.x, request.goal.pose.position.y, map_message)

        if start_cell is None or goal_cell is None:
            self.get_logger().warning('Cannot plan path: start or goal is outside the map')
            return response

        blocked_grid = self._build_blocked_grid(map_message)
        start_cell = self._nearest_free_cell(start_cell, blocked_grid, map_message.info.width, map_message.info.height)
        goal_cell = self._nearest_free_cell(goal_cell, blocked_grid, map_message.info.width, map_message.info.height)

        if start_cell is None or goal_cell is None:
            self.get_logger().warning('Cannot plan path: start or goal has no nearby free cell')
            return response

        cell_path = self._astar(start_cell, goal_cell, blocked_grid, map_message.info.width, map_message.info.height)
        if not cell_path:
            self.get_logger().warning('A* did not find a path')
            return response

        response.plan = self._build_path_message(cell_path, map_message)
        self._path_publisher.publish(response.plan)
        return response

    def _build_blocked_grid(self, map_message: OccupancyGrid) -> list[bool]:
        width = map_message.info.width
        height = map_message.info.height
        resolution = map_message.info.resolution
        inflation_cells = max(1, math.ceil(self._robot_radius / resolution))
        blocked = [False] * (width * height)
        occupied_cells: list[GridCell] = []

        for y in range(height):
            for x in range(width):
                value = map_message.data[self._grid_to_index(x, y, width)]
                if value < 0:
                    blocked[self._grid_to_index(x, y, width)] = True
                elif value >= self._occupied_threshold:
                    blocked[self._grid_to_index(x, y, width)] = True
                    occupied_cells.append((x, y))

        for occupied_x, occupied_y in occupied_cells:
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if math.hypot(dx, dy) > inflation_cells:
                        continue

                    inflated_x = occupied_x + dx
                    inflated_y = occupied_y + dy
                    if 0 <= inflated_x < width and 0 <= inflated_y < height:
                        blocked[self._grid_to_index(inflated_x, inflated_y, width)] = True

        return blocked

    def _astar(
        self,
        start: GridCell,
        goal: GridCell,
        blocked: list[bool],
        width: int,
        height: int,
    ) -> list[GridCell]:
        open_heap: list[tuple[float, int, GridCell]] = []
        tie_breaker = count()
        heapq.heappush(open_heap, (0.0, next(tie_breaker), start))

        came_from: dict[GridCell, GridCell] = {}
        cost_so_far = {start: 0.0}

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor, step_cost in self._neighbors(current, blocked, width, height):
                new_cost = cost_so_far[current] + step_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._octile_distance(neighbor, goal)
                    heapq.heappush(open_heap, (priority, next(tie_breaker), neighbor))
                    came_from[neighbor] = current

        return []

    def _neighbors(
        self,
        cell: GridCell,
        blocked: list[bool],
        width: int,
        height: int,
    ) -> list[tuple[GridCell, float]]:
        neighbors: list[tuple[GridCell, float]] = []
        cell_x, cell_y = cell

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue

                next_x = cell_x + dx
                next_y = cell_y + dy
                if not (0 <= next_x < width and 0 <= next_y < height):
                    continue
                if blocked[self._grid_to_index(next_x, next_y, width)]:
                    continue

                if dx != 0 and dy != 0:
                    first_side = blocked[self._grid_to_index(cell_x + dx, cell_y, width)]
                    second_side = blocked[self._grid_to_index(cell_x, cell_y + dy, width)]
                    if first_side or second_side:
                        continue
                    neighbors.append(((next_x, next_y), math.sqrt(2.0)))
                else:
                    neighbors.append(((next_x, next_y), 1.0))

        return neighbors

    def _nearest_free_cell(
        self,
        cell: GridCell,
        blocked: list[bool],
        width: int,
        height: int,
    ) -> GridCell | None:
        cell_x, cell_y = cell
        if not blocked[self._grid_to_index(cell_x, cell_y, width)]:
            return cell

        for radius in range(1, 11):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue

                    candidate_x = cell_x + dx
                    candidate_y = cell_y + dy
                    if not (0 <= candidate_x < width and 0 <= candidate_y < height):
                        continue
                    if not blocked[self._grid_to_index(candidate_x, candidate_y, width)]:
                        return candidate_x, candidate_y

        return None

    def _reconstruct_path(self, came_from: dict[GridCell, GridCell], current: GridCell) -> list[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _build_path_message(self, cell_path: list[GridCell], map_message: OccupancyGrid) -> Path:
        path_message = Path()
        path_message.header.stamp = self.get_clock().now().to_msg()
        path_message.header.frame_id = map_message.header.frame_id or 'map'

        for cell_x, cell_y in cell_path:
            pose = PoseStamped()
            pose.header = path_message.header
            pose.pose.position.x, pose.pose.position.y = self._grid_to_world(cell_x, cell_y, map_message)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_message.poses.append(pose)

        return path_message

    def _world_to_grid(
        self,
        world_x: float,
        world_y: float,
        map_message: OccupancyGrid,
    ) -> GridCell | None:
        origin = map_message.info.origin.position
        resolution = map_message.info.resolution
        grid_x = int((world_x - origin.x) / resolution)
        grid_y = int((world_y - origin.y) / resolution)

        if grid_x < 0 or grid_x >= map_message.info.width:
            return None
        if grid_y < 0 or grid_y >= map_message.info.height:
            return None

        return grid_x, grid_y

    def _grid_to_world(
        self,
        grid_x: int,
        grid_y: int,
        map_message: OccupancyGrid,
    ) -> tuple[float, float]:
        origin = map_message.info.origin.position
        resolution = map_message.info.resolution
        world_x = origin.x + (grid_x + 0.5) * resolution
        world_y = origin.y + (grid_y + 0.5) * resolution
        return world_x, world_y

    def _grid_to_index(self, grid_x: int, grid_y: int, width: int) -> int:
        return grid_y * width + grid_x

    def _octile_distance(self, first: GridCell, second: GridCell) -> float:
        delta_x = abs(first[0] - second[0])
        delta_y = abs(first[1] - second[1])
        return max(delta_x, delta_y) + (math.sqrt(2.0) - 1.0) * min(delta_x, delta_y)


def main(args=None) -> None:
    rclpy.init(args=args)
    astar_planner_node = AStarPlannerNode()

    try:
        rclpy.spin(astar_planner_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        astar_planner_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
