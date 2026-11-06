#!/usr/bin/env python3
"""Drive continuous laps around the explored area; push the map frontier out until a loop fits."""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from autonom_bringup.loop_planner import GridInfo, LoopSettings, plan_exploration_goal, plan_loop


class LoopPatrol(Node):

    def __init__(self):
        super().__init__('loop_patrol')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_base_frame', 'base_footprint').value
        self.min_known_area = self.declare_parameter('min_known_area', 4.0).value
        self.lap_pause = self.declare_parameter('lap_pause', 2.0).value
        self.retry_period = self.declare_parameter('retry_period', 3.0).value
        self.settings = LoopSettings(
            waypoint_count=self.declare_parameter('waypoint_count', 8).value,
            clearance=self.declare_parameter('clearance', 0.35).value,
            max_radius=self.declare_parameter('max_radius', 0.0).value,
            min_spacing=self.declare_parameter('min_spacing', 0.5).value,
            free_threshold=self.declare_parameter('free_threshold', 25).value,
        )
        map_topic = self.declare_parameter('map_topic', '/map').value

        self._map = None
        map_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def spin_for(self, seconds: float):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def robot_xy(self):
        try:
            transform = self._tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        return transform.transform.translation.x, transform.transform.translation.y

    def plan_next_task(self):
        """Return (kind, waypoints) with kind 'loop' or 'explore'; empty waypoints after waiting `retry_period`."""
        self.spin_for(0.1)
        robot_xy = self.robot_xy()
        if self._map is None or robot_xy is None:
            self.get_logger().info('Waiting for /map and robot pose...', throttle_duration_sec=10.0)
            self.spin_for(self.retry_period)
            return 'wait', []

        info = self._map.info
        grid = np.asarray(self._map.data, dtype=np.int16).reshape(info.height, info.width)
        grid_info = GridInfo(info.resolution, info.origin.position.x, info.origin.position.y)
        known_area = np.count_nonzero(grid >= 0) * info.resolution ** 2

        if known_area >= self.min_known_area:
            waypoints = plan_loop(grid, grid_info, robot_xy, self.settings)
            if waypoints:
                return 'loop', waypoints

        goal = plan_exploration_goal(grid, grid_info, robot_xy, self.settings)
        if goal is not None:
            return 'explore', [goal]

        self.get_logger().warn('No reachable loop or frontier in the explored space yet', throttle_duration_sec=10.0)
        self.spin_for(self.retry_period)
        return 'wait', []

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
        navigator.waitUntilNav2Active(localizer='slam_toolbox')
        lap = 0
        while rclpy.ok():
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
            navigator.followWaypoints([to_pose(patrol.global_frame, *w) for w in waypoints])
            while not navigator.isTaskComplete():
                rclpy.spin_once(patrol, timeout_sec=0.05)

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
