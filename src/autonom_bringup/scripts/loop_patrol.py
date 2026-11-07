#!/usr/bin/env python3
"""Drive continuous laps around the explored area; push the map frontier out until a loop fits."""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from autonom_bringup.loop_planner import (
    GridInfo, LoopSettings, StartLine, StartLineMonitor, drop_behind, loop_is_clockwise, plan_exploration_goal,
    plan_loop, rear_gate_cells, start_ahead)


class LoopPatrol(Node):

    def __init__(self):
        super().__init__('loop_patrol')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_base_frame', 'base_footprint').value
        self.min_known_area = self.declare_parameter('min_known_area', 4.0).value
        self.lap_pause = self.declare_parameter('lap_pause', 2.0).value
        self.retry_period = self.declare_parameter('retry_period', 3.0).value
        self.preempt_poses_remaining = self.declare_parameter('preempt_poses_remaining', 1).value
        # Rear gate: a keepout wall across the track behind the robot, so Nav2 can never plan a path that
        # turns back (no reverse travel, docs/06 section 4). Needs the global costmap KeepoutFilter.
        self.rear_gate_enabled = self.declare_parameter('rear_gate_enabled', True).value
        self.rear_gate_step = self.declare_parameter('rear_gate_update_distance', 0.3).value
        self.behind_drop_radius = self.declare_parameter('behind_drop_radius', 2.0).value
        self._gate_anchor = None
        self._gate_direction = None
        self._gate_pub = self.create_publisher(
            OccupancyGrid, self.declare_parameter('rear_gate_mask_topic', '/rear_gate_mask').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.wait_for_race_start = self.declare_parameter('wait_for_race_start', True).value
        # 'auto': laps run the way the robot faced at the start; it must never cross the start backwards.
        self.direction = self.declare_parameter('direction', 'auto').value
        if self.direction not in ('auto', 'clockwise', 'counterclockwise'):
            raise ValueError(f"direction must be 'auto', 'clockwise' or 'counterclockwise', got '{self.direction}'")
        self.start_line_half_width = self.declare_parameter('start_line_half_width', 1.0).value
        self.start_line = None
        self._start_monitor = None
        self.laps_completed = 0
        self._last_crossing = None
        self._last_loop = []
        self.reverse_crossings = 0
        self.settings = LoopSettings(
            clearance=self.declare_parameter('clearance', 0.35).value,
            min_spacing=self.declare_parameter('min_spacing', 0.5).value,
            free_threshold=self.declare_parameter('free_threshold', 25).value,
            waypoint_spacing=self.declare_parameter('waypoint_spacing', 1.0).value,
            min_loop_length=self.declare_parameter('min_loop_length', 2.0).value,
            cut_offset=self.declare_parameter('rear_gate_offset', 0.4).value,
            cut_half_width=self.declare_parameter('cut_half_width', 1.5).value,
            max_exploration_distance=self.declare_parameter('max_exploration_distance', 2.5).value,
        )
        self.rear_gate_offset = self.settings.cut_offset
        map_topic = self.declare_parameter('map_topic', '/map').value
        race_state_topic = self.declare_parameter('race_state_topic', '/race/state').value

        self._map = None
        map_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self._race_state = 'UNKNOWN'
        self.create_subscription(String, race_state_topic, self._on_race_state, map_qos)
        self._tf_buffer = Buffer()
        # Private node and thread: spin_once on this node also serves /clock (1 kHz in sim), which left the TF buffer
        # seconds behind, so start-line crossings and planning used stale poses.
        self._tf_listener = TransformListener(self._tf_buffer, None, spin_thread=True)

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_race_state(self, msg: String):
        if msg.data != self._race_state:
            self.get_logger().info(f'Race state: {msg.data}')
        self._race_state = msg.data

    @property
    def racing(self) -> bool:
        """Nav2 goals are allowed only after the start gate's 5 s delay (AC-3.2)."""
        return not self.wait_for_race_start or self._race_state == 'RACING'

    def spin_for(self, seconds: float):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def robot_pose(self):
        """(x, y, yaw) in the global frame, or None before TF is available."""
        try:
            transform = self._tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
        return transform.transform.translation.x, transform.transform.translation.y, yaw

    def plan_next_task(self):
        """Return (kind, waypoints) with kind 'loop' or 'explore'; empty waypoints after waiting `retry_period`."""
        self.spin_for(0.1)
        pose = self.robot_pose()
        if self._map is None or pose is None:
            self.get_logger().info('Waiting for /map and robot pose...', throttle_duration_sec=10.0)
            self.spin_for(self.retry_period)
            return 'wait', []

        robot_xy, heading = pose[:2], pose[2]
        if self.start_line is None:
            self.start_line = StartLine(*pose)
            self._start_monitor = StartLineMonitor(self.start_line, half_width=self.start_line_half_width)
            self._gate_anchor, self._gate_direction = robot_xy, pose[2]
            self.publish_rear_gate()
            self.get_logger().info(
                f'Start line at ({pose[0]:.2f}, {pose[1]:.2f}), race heading {pose[2]:.2f} rad')
        info = self._map.info
        grid = np.asarray(self._map.data, dtype=np.int16).reshape(info.height, info.width)
        grid_info = GridInfo(info.resolution, info.origin.position.x, info.origin.position.y)
        known_area = np.count_nonzero(grid >= 0) * info.resolution ** 2

        if known_area >= self.min_known_area:
            waypoints = self._plan_race_loop(grid, grid_info, robot_xy, heading)
            if waypoints:
                return 'loop', waypoints

        travel = self._gate_direction if self._gate_direction is not None else heading
        goal = plan_exploration_goal(grid, grid_info, robot_xy, self.settings, travel)
        if goal is not None:
            return 'explore', [goal]

        self.get_logger().warn('No reachable loop or frontier in the explored space yet', throttle_duration_sec=10.0)
        self.spin_for(self.retry_period)
        return 'wait', []

    def wait_until_active(self, node_names, request_timeout=2.0):
        """Block until every lifecycle node reports ACTIVE.

        Replaces BasicNavigator.waitUntilNav2Active(), which waits on get_state without a timeout: a request
        that reaches a node while it is still starting loses its response, and the patrol then hangs forever.
        """
        for name in node_names:
            client = self.create_client(GetState, f'{name}/get_state')
            try:
                while rclpy.ok():
                    if not client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info(f'Waiting for {name}...', throttle_duration_sec=10.0)
                        continue
                    future = client.call_async(GetState.Request())
                    deadline = time.monotonic() + request_timeout
                    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                        rclpy.spin_once(self, timeout_sec=0.05)
                    if future.done() and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                        self.get_logger().info(f'{name} is active')
                        break
                    if not future.done():
                        future.cancel()
                    self.spin_for(1.0)
            finally:
                self.destroy_client(client)

    def _plan_race_loop(self, grid, grid_info, robot_xy, heading):
        """Lap along the track in the direction of travel, starting at the first waypoint ahead of the robot."""
        direction = self._gate_direction if self._gate_direction is not None else heading
        waypoints = plan_loop(grid, grid_info, robot_xy, direction, self.settings)
        if not waypoints:
            if self._last_loop:
                # An obstacle on the track (another robot) breaks the closed ring in the map for a while;
                # keep lapping on the last valid loop and let Nav2 route around the obstacle.
                self.get_logger().info('Track ring not closed right now; reusing the last loop',
                                       throttle_duration_sec=10.0)
                return self._ahead_only(self._last_loop, robot_xy, heading)
            return []
        if self.direction != 'auto' and loop_is_clockwise(waypoints) != (self.direction == 'clockwise'):
            # Laps follow the travel direction set at the start; a forced direction that disagrees would need a
            # U-turn, which is never allowed.
            self.get_logger().error(
                f"direction '{self.direction}' runs against the robot's start heading: place the robot facing "
                'the race direction. Not driving this loop.', throttle_duration_sec=10.0)
            return []
        self._last_loop = waypoints
        return self._ahead_only(waypoints, robot_xy, heading)

    def _ahead_only(self, waypoints, robot_xy, heading):
        direction = self._gate_direction if self._gate_direction is not None else heading
        # Treat the lap as a cycle that starts at the waypoint nearest the robot (needed when a stored lap is
        # reused from another position), then skip anything just behind the robot.
        nearest = min(range(len(waypoints)), key=lambda k: math.dist(waypoints[k][:2], robot_xy))
        cycle = waypoints[nearest:] + waypoints[:nearest]
        return start_ahead(drop_behind(cycle, robot_xy, direction, self.behind_drop_radius), robot_xy, direction)

    def update_rear_gate(self, pose):
        """Advance the gate with the robot's forward travel; never flip it when the robot moves backwards."""
        if not self.rear_gate_enabled or self._gate_anchor is None:
            return
        dx, dy = pose[0] - self._gate_anchor[0], pose[1] - self._gate_anchor[1]
        if math.hypot(dx, dy) < self.rear_gate_step:
            return
        direction = math.atan2(dy, dx)
        if math.cos(direction - self._gate_direction) <= 0.0:
            return  # backing up or pushed back: keep the gate where it is
        self._gate_anchor, self._gate_direction = pose[:2], direction
        self.publish_rear_gate()

    def publish_rear_gate(self):
        if not self.rear_gate_enabled or self._map is None or self._gate_anchor is None:
            return
        info = self._map.info
        grid = np.asarray(self._map.data, dtype=np.int16).reshape(info.height, info.width)
        mask = np.zeros_like(grid, dtype=np.int8)
        cells = rear_gate_cells(
            grid, GridInfo(info.resolution, info.origin.position.x, info.origin.position.y),
            self._gate_anchor, self._gate_direction, offset=self.rear_gate_offset,
            max_half_width=self.settings.cut_half_width)
        for row, col in cells:
            mask[row, col] = 100
        msg = OccupancyGrid()
        msg.header.frame_id = self._map.header.frame_id or self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = info
        msg.data = mask.ravel().tolist()
        self._gate_pub.publish(msg)

    def watch_start_line(self):
        """Count forward crossings as laps; report reverse crossings (team rule, docs/06 section 4)."""
        pose = self.robot_pose()
        if pose is None or self._start_monitor is None:
            return
        self.update_rear_gate(pose)
        crossing = self._start_monitor.update(pose[:2])
        if crossing > 0:
            self.laps_completed += 1
            now = self.get_clock().now()
            lap_time = '' if self._last_crossing is None else f' in {(now - self._last_crossing).nanoseconds * 1e-9:.1f} s'
            self._last_crossing = now
            self.get_logger().info(f'Start line crossed forward: {self.laps_completed} lap(s) completed{lap_time}')
        elif crossing < 0:
            self.reverse_crossings += 1
            self.get_logger().error(
                f'REVERSE crossing of the start line ({self.reverse_crossings}): against the rules')


def to_pose(frame: str, x: float, y: float, yaw: float) -> PoseStamped:
    """Goal pose with a zero stamp ("latest TF"), valid under both wall and simulated time."""
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    return pose


def main():
    # Keep the context alive on Ctrl-C so the active Nav2 goal can still be cancelled.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    patrol = LoopPatrol()
    navigator = BasicNavigator('loop_patrol_navigator')
    navigator.set_parameters([patrol.get_parameter('use_sim_time')])
    try:
        patrol.wait_until_active(['slam_toolbox', 'bt_navigator'])
        lap = 0
        while rclpy.ok():
            if not patrol.racing:
                patrol.get_logger().info('Waiting for the race start (/race/state)...', throttle_duration_sec=10.0)
                patrol.spin_for(0.1)
                continue
            kind, waypoints = patrol.plan_next_task()
            if not waypoints:
                continue
            if kind == 'loop':
                lap += 1
                label = f'Lap {lap}'
            else:
                x, y, _ = waypoints[0]
                label = f'Exploration step to ({x:.2f}, {y:.2f})'
            patrol.get_logger().info(f'{label}: {len(waypoints)} waypoint(s)')
            poses = [to_pose(patrol.global_frame, *w) for w in waypoints]
            if kind == 'loop':
                # One continuous path through the whole lap; sending the next lap preempts this goal.
                navigator.goThroughPoses(poses)
            else:
                navigator.goToPose(poses[0])
            preempted = False
            while not navigator.isTaskComplete():
                rclpy.spin_once(patrol, timeout_sec=0.05)
                patrol.watch_start_line()
                if not patrol.racing:
                    patrol.get_logger().warn(f'Race stopped during {label}; cancelling the goal')
                    navigator.cancelTask()
                    break
                feedback = navigator.getFeedback()
                if (kind == 'loop' and feedback is not None
                        and 0 < feedback.number_of_poses_remaining <= patrol.preempt_poses_remaining):
                    # Plan and send the next lap while still driving, so the robot never brakes for a lap end.
                    preempted = True
                    break
            if not patrol.racing or preempted:
                continue

            result = navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                patrol.get_logger().warn(f'{label} ended with {result.name}')
            if kind == 'loop':
                patrol.spin_for(patrol.lap_pause)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        navigator.cancelTask()
        navigator.destroy_node()
        patrol.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
