import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('planner_period_sec', 5.0)
        self.declare_parameter('min_frontier_cluster', 10)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('goal_min_distance', 2.0)
        self.declare_parameter('goal_max_distance', 25.0)

        self.declare_parameter('goal_offset_from_frontier', 0.8)
        self.declare_parameter('min_goal_clearance', 0.6)
        self.declare_parameter('blacklist_radius', 1.0)
        self.declare_parameter('exploration_stop_ratio', 0.97)
        self.declare_parameter('min_clusters_to_continue', 2)

        self.map_topic = self.get_parameter('map_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.planner_period_sec = float(self.get_parameter('planner_period_sec').value)
        self.min_frontier_cluster = int(self.get_parameter('min_frontier_cluster').value)
        self.occ_threshold = int(self.get_parameter('occ_threshold').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.goal_min_distance = float(self.get_parameter('goal_min_distance').value)
        self.goal_max_distance = float(self.get_parameter('goal_max_distance').value)

        self.goal_offset_from_frontier = float(self.get_parameter('goal_offset_from_frontier').value)
        self.min_goal_clearance = float(self.get_parameter('min_goal_clearance').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.exploration_stop_ratio = float(self.get_parameter('exploration_stop_ratio').value)
        self.min_clusters_to_continue = int(self.get_parameter('min_clusters_to_continue').value)

        self.map_msg = None
        self.busy = False
        self.blacklist = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.explore_request_pub = self.create_publisher(Bool, '/explore/request', 10)
        self.publish_explore_request(True)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(self.planner_period_sec, self.plan_cycle)

        self.goal_handle = None
        self.goal_sent_time = None
        self.current_goal_xy = None

        self.get_logger().info('FrontierExplorer avviato.')

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def plan_cycle(self):
        if self.map_msg is None:
            self.get_logger().debug('Mappa non ancora ricevuta.')
            return

        if self.busy:
            self.publish_explore_request(True)
            if self.goal_sent_time is not None:
                elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout_sec:
                    self.get_logger().warn('Timeout goal: annullo e ripianifico.')
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                    self.reset_goal_state()
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn('Impossibile leggere la posa robot da TF.')
            return

        self.publish_explore_request(True)

        frontiers = self.extract_frontiers(self.map_msg)
        clusters = self.cluster_frontiers(frontiers)

        if not clusters:
            self.get_logger().info('Nessuna frontier trovata.')
            self.publish_explore_request(False)
            return

        explored = self.explored_ratio(self.map_msg)
        if explored >= self.exploration_stop_ratio and len(clusters) <= self.min_clusters_to_continue:
            self.get_logger().info(
                f'Esplorazione considerata completata: explored_ratio={explored:.3f}, clusters={len(clusters)}'
            )
            self.publish_explore_request(False)
            return

        goal = self.choose_goal(clusters, robot_pose, explored)
        if goal is None:
            self.get_logger().info('Nessun goal valido trovato.')
            return

        self.send_goal(goal[0], goal[1])

    def reset_goal_state(self):
        self.busy = False
        self.goal_handle = None
        self.goal_sent_time = None
        self.current_goal_xy = None

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return (
                tf.transform.translation.x,
                tf.transform.translation.y
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def explored_ratio(self, map_msg: OccupancyGrid):
        data = list(map_msg.data)
        if not data:
            return 0.0
        known = sum(1 for v in data if v != -1)
        return known / len(data)

    def extract_frontiers(self, map_msg: OccupancyGrid):
        width = map_msg.info.width
        height = map_msg.info.height
        data = list(map_msg.data)

        frontiers = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x

                if data[idx] != -1:
                    continue

                neighbors = [
                    data[(y - 1) * width + x],
                    data[(y + 1) * width + x],
                    data[y * width + (x - 1)],
                    data[y * width + (x + 1)],
                ]

                if any(n == 0 for n in neighbors):
                    frontiers.append((x, y))

        return frontiers

    def cluster_frontiers(self, frontier_cells):
        frontier_set = set(frontier_cells)
        visited = set()
        clusters = []

        for cell in frontier_cells:
            if cell in visited:
                continue

            queue = deque([cell])
            cluster = []
            visited.add(cell)

            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))

                for nx in range(cx - 1, cx + 2):
                    for ny in range(cy - 1, cy + 2):
                        neighbor = (nx, ny)
                        if neighbor in frontier_set and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)

            dynamic_min_cluster = self.dynamic_min_frontier_cluster()
            if len(cluster) >= dynamic_min_cluster:
                clusters.append(cluster)

        return clusters

    def dynamic_min_frontier_cluster(self):
        if self.map_msg is None:
            return self.min_frontier_cluster

        explored = self.explored_ratio(self.map_msg)
        dynamic_value = max(4, int(round(self.min_frontier_cluster - 6.0 * explored)))
        return dynamic_value

    def dynamic_goal_min_distance(self):
        if self.map_msg is None:
            return self.goal_min_distance

        explored = self.explored_ratio(self.map_msg)
        dynamic_value = max(0.8, self.goal_min_distance - 1.2 * explored)
        return dynamic_value

    def choose_goal(self, clusters, robot_pose, explored):
        best_goal = None
        best_score = -float('inf')

        # all'inizio favorisce aperture ampie e un po' più lontane
        # poi diventa gradualmente più locale
        distance_weight = 1.2 - 0.8 * explored
        size_weight = 1.5 - 0.5 * explored
        clearance_weight = 1.0

        dynamic_goal_min_distance = self.dynamic_goal_min_distance()

        for cluster in clusters:
            gx = sum(c[0] for c in cluster) / len(cluster)
            gy = sum(c[1] for c in cluster) / len(cluster)

            frontier_wx, frontier_wy = self.grid_to_world(self.map_msg, gx, gy)
            goal_wx, goal_wy = self.offset_goal_from_frontier(
                frontier_wx,
                frontier_wy,
                robot_pose,
                offset=self.goal_offset_from_frontier
            )

            if self.is_blacklisted(goal_wx, goal_wy):
                continue

            dist = math.hypot(goal_wx - robot_pose[0], goal_wy - robot_pose[1])

            if dist < dynamic_goal_min_distance:
                continue

            if dist > self.goal_max_distance:
                continue

            clearance = self.goal_clearance(goal_wx, goal_wy)
            if clearance < self.min_goal_clearance:
                continue

            cluster_size = len(cluster)

            score = (
                size_weight * cluster_size
                + distance_weight * dist
                + clearance_weight * clearance * 10.0
            )

            if score > best_score:
                best_score = score
                best_goal = (goal_wx, goal_wy)

        if best_goal is not None:
            self.get_logger().info(
                f'Scelto goal x={best_goal[0]:.2f}, y={best_goal[1]:.2f}, '
                f'explored={explored:.3f}, min_cluster_dyn={self.dynamic_min_frontier_cluster()}, '
                f'goal_min_dist_dyn={dynamic_goal_min_distance:.2f}, score={best_score:.2f}'
            )

        return best_goal

    def offset_goal_from_frontier(self, goal_x, goal_y, robot_pose, offset=0.8):
        dx = goal_x - robot_pose[0]
        dy = goal_y - robot_pose[1]
        norm = math.hypot(dx, dy)

        if norm < 1e-6:
            return goal_x, goal_y

        ux = dx / norm
        uy = dy / norm

        safe_x = goal_x - ux * offset
        safe_y = goal_y - uy * offset

        return safe_x, safe_y

    def goal_clearance(self, wx, wy, radius_cells=6):
        if self.map_msg is None:
            return 0.0

        mx, my = self.world_to_grid(self.map_msg, wx, wy)
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        data = list(self.map_msg.data)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return 0.0

        center_idx = my * width + mx
        center_value = data[center_idx]

        # scarta goal su sconosciuto o occupato
        if center_value == -1 or center_value > self.occ_threshold:
            return 0.0

        min_dist = float('inf')

        y_min = max(0, my - radius_cells)
        y_max = min(height, my + radius_cells + 1)
        x_min = max(0, mx - radius_cells)
        x_max = min(width, mx + radius_cells + 1)

        for y in range(y_min, y_max):
            for x in range(x_min, x_max):
                idx = y * width + x
                value = data[idx]

                if value > self.occ_threshold:
                    d = math.hypot(x - mx, y - my)
                    min_dist = min(min_dist, d)

        if min_dist == float('inf'):
            return 999.0

        return min_dist * self.map_msg.info.resolution

    def is_blacklisted(self, x, y):
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    def world_to_grid(self, map_msg: OccupancyGrid, wx, wy):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        gx = int((wx - origin_x) / resolution)
        gy = int((wy - origin_y) / resolution)
        return gx, gy

    def grid_to_world(self, map_msg: OccupancyGrid, gx, gy):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        wx = origin_x + (gx + 0.5) * resolution
        wy = origin_y + (gy + 0.5) * resolution
        return wx, wy

    def send_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Action server navigate_to_pose non disponibile.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Invio goal frontier: x={x:.2f}, y={y:.2f}')

        self.busy = True
        self.goal_sent_time = self.get_clock().now()
        self.current_goal_xy = (x, y)

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rifiutato da Nav2.')
            self.reset_goal_state()
            return

        self.get_logger().info('Goal accettato.')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal raggiunto.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal abortito: aggiungo in blacklist.')
            if self.current_goal_xy is not None:
                self.blacklist.append(self.current_goal_xy)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal cancellato.')
        else:
            self.get_logger().info(f'Goal terminato con status={status}')

        self.reset_goal_state()

    def feedback_callback(self, feedback_msg):
        pass

    def publish_explore_request(self, value: bool):
        msg = Bool()
        msg.data = value
        self.explore_request_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()