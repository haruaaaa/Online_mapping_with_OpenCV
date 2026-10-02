#!/usr/bin/env python3
"""
Enhanced Frontier Exploration Node with OpenCV, Morphological Analysis & Nav2.
Designed for StarLine Hackathon 2026 (HSL26).

Features:
1. Exact Frontier Extraction via Morphological Dilation & Bitwise AND.
2. Connected Components Clustering: filters out small speckles and groups corridor openings.
3. Safe Goal Projection via cv2.distanceTransform with adaptive fallback for narrow maze passages.
4. Fast 2D Grid BFS for true geodesic path distance through free corridors.
5. Official Nav2 Action Client (NavigateToPose) with topic fallback (/goal_pose).
6. Rich RViz2 Visualization: /exploration/frontiers and /exploration/active_goal markers.
7. Support for manual /marker goals and temporary blacklisting of unreachable targets.
"""

from collections import deque
import math
import cv2
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('explore_cv')

        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('min_cluster_size', 5)          # Min frontier pixels (~20cm)
        self.declare_parameter('min_obstacle_clearance', 0.25)    # Ideal safe clearance in meters
        self.declare_parameter('min_fallback_clearance', 0.18)    # Fallback clearance for narrow corridors
        self.declare_parameter('goal_tolerance', 0.30)          # Proximity threshold in meters
        self.declare_parameter('goal_timeout_sec', 40.0)        # Timeout for single goal
        self.declare_parameter('stuck_timeout_sec', 12.0)       # Timeout if not moving
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('marker_topic', '/marker')

        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.min_clearance = float(self.get_parameter('min_obstacle_clearance').value)
        self.min_fallback_clearance = float(self.get_parameter('min_fallback_clearance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.stuck_timeout_sec = float(self.get_parameter('stuck_timeout_sec').value)
        self.map_topic = self.get_parameter('map_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value

        # Robot Pose State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_robot_pose = False

        self.last_robot_x = 0.0
        self.last_robot_y = 0.0
        self.last_robot_yaw = 0.0
        self.last_moved_time = None
        self._last_wait_log = 0.0

        # Map State
        self.map_data = None
        self.map_info = None
        self.new_map_received = False

        # Goal Management
        self.current_goal = None
        self.goal_active = False
        self.goal_start_time = None
        self.goal_handle = None
        self.goal_seq_id = 0

        # Blacklist (x, y, timestamp)
        self.blacklisted_goals = []

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.create_subscription(Pose, self.marker_topic, self.marker_callback, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 10)
        self.active_goal_pub = self.create_publisher(Marker, '/exploration/active_goal', 10)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timers
        self.create_timer(0.1, self.update_robot_pose)
        self.create_timer(1.0, self.exploration_loop)

        self.get_logger().info(
            f" [FrontierExplorer CV] Ready! clearance={self.min_clearance}m (fallback={self.min_fallback_clearance}m), "
            f"base={self.base_frame}, map_topic={self.map_topic}"
        )

    # --------------------------------------------------------------------------
    # Pose Tracking
    # --------------------------------------------------------------------------
    def update_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(seconds=0)
            )
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.has_robot_pose = True

            # Movement tracking for stuck detection
            now = self.get_clock().now().nanoseconds / 1e9
            dist_moved = math.hypot(self.robot_x - self.last_robot_x, self.robot_y - self.last_robot_y)
            yaw_diff = abs(math.atan2(math.sin(self.robot_yaw - self.last_robot_yaw), math.cos(self.robot_yaw - self.last_robot_yaw)))

            if dist_moved > 0.04 or yaw_diff > 0.10:
                self.last_robot_x = self.robot_x
                self.last_robot_y = self.robot_y
                self.last_robot_yaw = self.robot_yaw
                self.last_moved_time = now
            elif self.last_moved_time is None:
                self.last_moved_time = now

        except Exception:
            pass

    # --------------------------------------------------------------------------
    # Map Handling
    # --------------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.new_map_received = True

    # --------------------------------------------------------------------------
    # Main Exploration Loop
    # --------------------------------------------------------------------------
    def exploration_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if not self.has_robot_pose or self.map_data is None or self.map_info is None:
            if now - self._last_wait_log > 3.0:
                self._last_wait_log = now
                missing = []
                if not self.has_robot_pose:
                    missing.append(f"TF ({self.map_frame} -> {self.base_frame})")
                if self.map_data is None:
                    missing.append(f"map topic ({self.map_topic})")
                self.get_logger().info(f" [FrontierExplorer] Waiting for: {', '.join(missing)}...")
            return

        # 1. Health check for currently active goal
        if self.goal_active:
            if self.current_goal is not None:
                gx, gy = self.current_goal
                dist_to_goal = math.hypot(self.robot_x - gx, self.robot_y - gy)

                # Goal reached by proximity
                if dist_to_goal < self.goal_tolerance:
                    self.get_logger().info(f" [FrontierExplorer] Goal reached! (dist={dist_to_goal:.2f}m)")
                    self.on_goal_succeeded(cancel_active=True)
                    return

                # Check elapsed time
                elapsed = now - self.goal_start_time
                if elapsed > self.goal_timeout_sec:
                    self.get_logger().warn(f" [FrontierExplorer] Goal timeout ({elapsed:.1f}s)! Aborting.")
                    self.abort_current_goal(reason="timeout")
                    return

                # Check if robot is stuck
                if self.last_moved_time is not None:
                    time_not_moved = now - self.last_moved_time
                    if elapsed > 10.0 and time_not_moved > self.stuck_timeout_sec:
                        self.get_logger().warn(f" [FrontierExplorer] Robot stuck for {time_not_moved:.1f}s! Aborting goal.")
                        self.abort_current_goal(reason="stuck")
                        return

            return  # Still navigating towards active goal

        # 2. No active goal: select and dispatch next best frontier
        self.select_and_dispatch_next_frontier()

    # --------------------------------------------------------------------------
    # Frontier Extraction & Safe Goal Selection
    # --------------------------------------------------------------------------
    def select_and_dispatch_next_frontier(self):
        h, w = self.map_data.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        # Robot cell coordinates
        rx = int((self.robot_x - ox) / res)
        ry = int((self.robot_y - oy) / res)
        if rx < 0 or rx >= w or ry < 0 or ry >= h:
            return

        # Binary masks
        free_mask = (self.map_data == 0).astype(np.uint8) * 255
        unknown_mask = (self.map_data == -1).astype(np.uint8) * 255
        non_obstacle = (self.map_data <= 50).astype(np.uint8) * 255

        # Distance transform: true metric clearance from obstacles/walls
        dist_from_walls = cv2.distanceTransform(non_obstacle, cv2.DIST_L2, 5) * res

        # Safe free space mask (clearance >= min_clearance)
        safe_free_mask = (dist_from_walls >= self.min_clearance) & (free_mask > 0)

        # Extract Frontiers: free cells adjacent to unknown cells
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilated_unknown = cv2.dilate(unknown_mask, kernel, iterations=1)
        frontier_mask = cv2.bitwise_and(free_mask, dilated_unknown)

        # Filter out frontier pixels that hug walls (< 0.15m)
        frontier_mask[dist_from_walls < 0.15] = 0

        # Group frontier pixels into connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_mask, connectivity=8)

        candidate_goals = []
        frontier_markers = []

        # 2D Grid BFS for passable geodesic distance through corridors
        reachable_dist = self.compute_grid_bfs_distance(free_mask, rx, ry)

        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area < self.min_cluster_size:
                continue

            cx, cy = centroids[label]
            cluster_pts = np.argwhere(labels == label)  # (y, x) array

            # Project to safest cell near the cluster
            safe_goal_cell = self.find_safe_goal_cell(cluster_pts, dist_from_walls, safe_free_mask, free_mask)
            if safe_goal_cell is None:
                continue

            gy, gx = safe_goal_cell

            # Geodesic reachability check
            bfs_dist_cells = reachable_dist[gy, gx]
            if np.isinf(bfs_dist_cells) or bfs_dist_cells <= 0:
                # Fallback to euclidean distance with penalty if BFS had tiny disconnect
                euc_dist = math.hypot((gx - rx) * res, (gy - ry) * res)
                path_distance = euc_dist * 1.5
            else:
                path_distance = bfs_dist_cells * res

            # Convert to metric map coordinates
            mx = gx * res + ox
            my = gy * res + oy

            # Filter out blacklisted or overly close goals
            if self.is_blacklisted(mx, my):
                continue
            if math.hypot(mx - self.robot_x, my - self.robot_y) < 0.35:
                continue

            # Cost: prefer closer path and larger frontier cluster
            cost = path_distance - 0.04 * min(area, 50)

            candidate_goals.append({
                'x': mx,
                'y': my,
                'cost': cost,
                'area': area,
                'path_dist': path_distance
            })

            frontier_markers.append((mx, my, area))

        # Publish visualization markers in RViz
        self.publish_frontier_markers(frontier_markers)

        if not candidate_goals:
            if len(self.blacklisted_goals) > 0:
                self.get_logger().info(" [FrontierExplorer] No open frontiers. Decaying blacklist...")
                self.decay_blacklist()
            else:
                self.get_logger().info(" [FrontierExplorer] No frontiers detected in map! Territory fully explored.")
            return

        # Sort candidate goals by lowest cost
        candidate_goals.sort(key=lambda g: g['cost'])
        best_candidate = candidate_goals[0]

        self.get_logger().info(
            f" [FrontierExplorer] Selected Best Frontier: ({best_candidate['x']:.2f}, {best_candidate['y']:.2f}) | "
            f"Path={best_candidate['path_dist']:.2f}m, Area={best_candidate['area']}px, Total Candidates={len(candidate_goals)}"
        )

        self.dispatch_goal(best_candidate['x'], best_candidate['y'])

    # --------------------------------------------------------------------------
    # Safe Goal Projection
    # --------------------------------------------------------------------------
    def find_safe_goal_cell(self, cluster_pts, dist_from_walls, safe_free_mask, free_mask):
        """
        Finds a safe cell near the frontier cluster with >= min_clearance from walls.
        Enforces a 4-cell safety margin from map boundaries so Nav2 planner never fails.
        If no point meets min_clearance, falls back to maximum clearance >= min_fallback_clearance.
        """
        h, w = free_mask.shape
        margin = 4  # 4 cells = 0.16m from map edge

        # 1. Search among cluster points with ideal clearance
        best_pt = None
        best_clearance = -1.0

        for pt in cluster_pts:
            py, px = pt[0], pt[1]
            if margin <= px < w - margin and margin <= py < h - margin:
                if safe_free_mask[py, px]:
                    c = dist_from_walls[py, px]
                    if c > best_clearance:
                        best_clearance = c
                        best_pt = (py, px)

        if best_pt is not None:
            return best_pt

        # 2. Search neighbor free cells inward into corridor
        cy, cx = int(np.mean(cluster_pts[:, 0])), int(np.mean(cluster_pts[:, 1]))
        search_radius = 14  # ~0.56m at 0.04m resolution

        min_y = max(margin, cy - search_radius)
        max_y = min(h - margin, cy + search_radius + 1)
        min_x = max(margin, cx - search_radius)
        max_x = min(w - margin, cx + search_radius + 1)

        best_dist = float('inf')
        safe_pt = None

        for y in range(min_y, max_y):
            for x in range(min_x, max_x):
                if safe_free_mask[y, x]:
                    d = (x - cx) ** 2 + (y - cy) ** 2
                    if d < best_dist:
                        best_dist = d
                        safe_pt = (y, x)

        if safe_pt is not None:
            return safe_pt

        # 3. Fallback for narrow passages / obstacles:
        # Find cell with highest clearance >= min_fallback_clearance (down to robot radius ~0.18m)
        best_fb_pt = None
        best_fb_clearance = self.min_fallback_clearance

        for y in range(min_y, max_y):
            for x in range(min_x, max_x):
                if free_mask[y, x] and (self.map_data[y, x] == 0):
                    c = dist_from_walls[y, x]
                    if c > best_fb_clearance:
                        best_fb_clearance = c
                        best_fb_pt = (y, x)

        return best_fb_pt

    # --------------------------------------------------------------------------
    # 2D Grid BFS for Path Distance
    # --------------------------------------------------------------------------
    def compute_grid_bfs_distance(self, free_mask, start_x, start_y):
        h, w = free_mask.shape
        dist_grid = np.full((h, w), np.inf, dtype=np.float32)

        if start_x < 0 or start_x >= w or start_y < 0 or start_y >= h:
            return dist_grid

        # Morphological closing (3x3) to bridge 1-pixel lidar noise gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        passable = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)
        if self.map_data is not None:
            passable[self.map_data > 50] = 0

        q = deque()
        if passable[start_y, start_x]:
            dist_grid[start_y, start_x] = 0.0
            q.append((start_x, start_y))
        else:
            # Seed from nearest passable cell
            for r in range(1, 10):
                found = False
                for dy_off in range(-r, r + 1):
                    for dx_off in range(-r, r + 1):
                        nx, ny = start_x + dx_off, start_y + dy_off
                        if 0 <= nx < w and 0 <= ny < h and passable[ny, nx]:
                            dist_grid[ny, nx] = math.hypot(dx_off, dy_off)
                            q.append((nx, ny))
                            found = True
                if found:
                    break

        if not q:
            return dist_grid

        # 8-connectivity steps
        dx = [1, -1, 0, 0, 1, -1, 1, -1]
        dy = [0, 0, 1, -1, 1, 1, -1, -1]
        cost = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]

        while q:
            cx, cy = q.popleft()
            cd = dist_grid[cy, cx]

            for i in range(8):
                nx, ny = cx + dx[i], cy + dy[i]
                if 0 <= nx < w and 0 <= ny < h and passable[ny, nx]:
                    nd = cd + cost[i]
                    if nd < dist_grid[ny, nx]:
                        dist_grid[ny, nx] = nd
                        q.append((nx, ny))

        return dist_grid

    # --------------------------------------------------------------------------
    # Goal Dispatching (Nav2 Action + PoseStamped Topic)
    # --------------------------------------------------------------------------
    def dispatch_goal(self, x, y):
        # Clamp to ensure coordinates are inside map boundaries
        if self.map_info is not None:
            margin_dist = 4 * self.map_info.resolution
            x_min = self.map_info.origin.position.x + margin_dist
            x_max = self.map_info.origin.position.x + (self.map_info.width * self.map_info.resolution) - margin_dist
            y_min = self.map_info.origin.position.y + margin_dist
            y_max = self.map_info.origin.position.y + (self.map_info.height * self.map_info.resolution) - margin_dist
            x = min(max(x, x_min), x_max)
            y = min(max(y, y_min), y_max)

        self.goal_seq_id += 1
        seq_id = self.goal_seq_id

        self.current_goal = (x, y)
        self.goal_active = True
        self.goal_start_time = self.get_clock().now().nanoseconds / 1e9

        # Orient goal towards direction from robot
        target_yaw = math.atan2(y - self.robot_y, x - self.robot_x)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(target_yaw / 2.0)

        # 1. Publish active goal marker in RViz
        self.publish_active_goal_marker(x, y, target_yaw)

        # 2. Dispatch via Nav2 Action Client if available, else /goal_pose topic
        if self.nav_client.wait_for_server(timeout_sec=0.2):
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal_msg

            send_goal_future = self.nav_client.send_goal_async(
                nav_goal,
                feedback_callback=self.nav_feedback_callback
            )
            send_goal_future.add_done_callback(
                lambda fut, sid=seq_id, coords=(x, y): self.goal_response_callback(fut, sid, coords)
            )
        else:
            self.goal_pub.publish(goal_msg)
            self.get_logger().info(f" [FrontierExplorer] Dispatched goal ({x:.2f}, {y:.2f}) via {self.goal_topic}")

    def goal_response_callback(self, future, seq_id, coords):
        if seq_id != self.goal_seq_id:
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(" [FrontierExplorer] Nav2 rejected goal!")
            self.abort_current_goal(reason="nav2_rejected")
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, gh=goal_handle, sid=seq_id, coords=coords: self.goal_result_callback(fut, gh, sid, coords)
        )

    def nav_feedback_callback(self, feedback_msg):
        pass

    def goal_result_callback(self, future, goal_handle, seq_id, coords):
        if seq_id != self.goal_seq_id or self.goal_handle != goal_handle:
            return

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(" [FrontierExplorer] Nav2 reported SUCCEEDED!")
            self.on_goal_succeeded(cancel_active=False)
        elif status in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
            if self.goal_active and self.current_goal == coords:
                self.get_logger().warn(f" [FrontierExplorer] Nav2 reported status={status}. Aborting goal.")
                self.abort_current_goal(reason="nav2_aborted")

    def on_goal_succeeded(self, cancel_active=False):
        self.goal_seq_id += 1
        if cancel_active and self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.current_goal = None
        self.goal_active = False
        self.goal_start_time = None
        self.goal_handle = None

    def abort_current_goal(self, reason="unknown"):
        self.goal_seq_id += 1
        if self.current_goal is not None:
            gx, gy = self.current_goal
            now = self.get_clock().now().nanoseconds / 1e9
            self.blacklisted_goals.append((gx, gy, now))
            self.get_logger().warn(f" [FrontierExplorer] Blacklisted goal ({gx:.2f}, {gy:.2f}), reason={reason}")

        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass

        self.current_goal = None
        self.goal_active = False
        self.goal_start_time = None
        self.goal_handle = None

    # --------------------------------------------------------------------------
    # Blacklist Management
    # --------------------------------------------------------------------------
    def is_blacklisted(self, x, y, radius=0.35, max_age_sec=60.0):
        now = self.get_clock().now().nanoseconds / 1e9
        self.blacklisted_goals = [b for b in self.blacklisted_goals if (now - b[2]) < max_age_sec]
        for bx, by, _ in self.blacklisted_goals:
            if math.hypot(x - bx, y - by) < radius:
                return True
        return False

    def decay_blacklist(self):
        if len(self.blacklisted_goals) > 2:
            self.blacklisted_goals = self.blacklisted_goals[len(self.blacklisted_goals) // 2:]
        else:
            self.blacklisted_goals.clear()

    # --------------------------------------------------------------------------
    # Manual Marker Support (/marker)
    # --------------------------------------------------------------------------
    def marker_callback(self, msg: Pose):
        self.get_logger().info(f" [FrontierExplorer] Manual /marker target at ({msg.position.x:.2f}, {msg.position.y:.2f})")
        self.dispatch_goal(msg.position.x, msg.position.y)

    # --------------------------------------------------------------------------
    # Visualization Markers
    # --------------------------------------------------------------------------
    def publish_frontier_markers(self, frontiers):
        marker_array = MarkerArray()

        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        now = self.get_clock().now().to_msg()

        for idx, (fx, fy, area) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = "frontiers"
            m.id = idx + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(fx)
            m.pose.position.y = float(fy)
            m.pose.position.z = 0.1
            scale = min(max(0.1, math.sqrt(area) * 0.04), 0.35)
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            m.color.r = 0.0
            m.color.g = 0.85
            m.color.b = 0.2
            m.color.a = 0.8
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

    def publish_active_goal_marker(self, x, y, yaw):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "active_goal"
        m.id = 9999
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.15
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x = 0.35
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.8
        m.color.a = 1.0
        self.active_goal_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()