#!/usr/bin/env python3
# Runs the routes on /waypoints (missions, ORO's "Visit waypoints") through Nav2, in place of
# waypoint_follower.py: launch/demo.launch.py starts this one with slam:=true.
# A route is split into legs: consecutive waypoints -> one navigate_through_poses (planned around
# obstacles on the map); waypoints at the same spot as the one before (a mission's spin) -> one spin
# by the sum of their heading changes. A new route first cancels any running Nav2 goal. Nav2 ends a navigate_through_poses as soon as the robot is
# at its LAST pose, so a leg is cut before a waypoint that comes back near the leg's start or one of
# its waypoints (out and back, a closed loop): otherwise it would "succeed" without moving. It is
# also cut where the route turns back sharply: a path that overlaps itself makes the robot dither. An empty route cancels every Nav2 goal (Stop mission,
# Emergency stop), including goals clicked in RViz or ORO, which go to Nav2 directly (/goal_pose).

import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateThroughPoses, Spin
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_action_status_default
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

SAME_SPOT = 1e-3            # m; a waypoint this close to the previous one is a turn in place
SPIN_SECONDS_PER_TURN = 10  # time allowance per full turn
CANCELLABLE = ('navigate_to_pose', 'navigate_through_poses', 'spin')
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
IDLE_WAIT = 5.0             # s; how long a new route waits for cancelled Nav2 goals to wind down
LOOP_RADIUS = 0.3           # m; > Nav2's xy_goal_tolerance (config/nav2.yaml)
TURN_BACK = math.radians(100)   # a sharper turn at a waypoint ends the leg there


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def legs_of(path, start=None):
    """[('drive', [PoseStamped]) | ('spin', radians)] for a route. start: robot (x, y), if known."""
    legs = []
    prev = None
    leg_start = start   # where the current drive leg starts
    for pose in path.poses:
        pose.header.frame_id = pose.header.frame_id or path.header.frame_id
        p = pose.pose
        if prev is not None and math.hypot(p.position.x - prev.position.x,
                                           p.position.y - prev.position.y) < SAME_SPOT:
            turn = wrap(yaw_of(p.orientation) - yaw_of(prev.orientation))
            if legs and legs[-1][0] == 'spin':
                legs[-1] = ('spin', legs[-1][1] + turn)
            else:
                legs.append(('spin', turn))
        elif legs and legs[-1][0] == 'drive' and not must_cut(p, leg_start, legs[-1][1]):
            legs[-1][1].append(pose)
        else:
            if legs and legs[-1][0] == 'drive':
                last = legs[-1][1][-1].pose.position
                leg_start = (last.x, last.y)
            elif prev is not None:
                leg_start = (prev.position.x, prev.position.y)
            legs.append(('drive', [pose]))
        prev = p
    return legs


def must_cut(p, leg_start, leg):
    """True if waypoint p has to start a new leg: it comes back near the leg's start or one of its
    waypoints, or the route turns back at the leg's last waypoint (the path would overlap itself, and
    the controller, which follows the nearest path point, swings between the two directions)."""
    spots = [leg_start] if leg_start else []
    spots += [(q.pose.position.x, q.pose.position.y) for q in leg]
    if any(math.hypot(p.position.x - x, p.position.y - y) < LOOP_RADIUS for x, y in spots):
        return True
    if len(spots) < 2:
        return False
    (ax, ay), (bx, by) = spots[-2], spots[-1]
    turn = wrap(math.atan2(p.position.y - by, p.position.x - bx) - math.atan2(by - ay, bx - ax))
    return abs(turn) > TURN_BACK


class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')
        self.through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.spin = ActionClient(self, Spin, 'spin')
        self.cancel_all = {name: self.create_client(CancelGoal, f'{name}/_action/cancel_goal')
                           for name in CANCELLABLE}
        # Nav2 goals running now, per action (from its status topic)
        self.active = {name: False for name in CANCELLABLE}
        for name in CANCELLABLE:
            self.create_subscription(
                GoalStatusArray, f'{name}/_action/status',
                lambda msg, name=name: self.active.__setitem__(
                    name, any(s.status in ACTIVE for s in msg.status_list)),
                qos_profile_action_status_default)
        self.waiting = None      # timer waiting for Nav2 to go idle before a route starts
        self.legs = []
        self.leg_handle = None   # goal handle of the leg running now
        self.route = 0       # bumped on every new route: results of an older one are ignored
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Path, '/waypoints', self.on_path, 10)

    def on_path(self, msg):
        self.route += 1
        self.legs = []
        if self.leg_handle is not None:
            self.leg_handle.cancel_goal_async()
            self.leg_handle = None
        if not msg.poses:
            self.cancel_nav2_goals()
            self.get_logger().info('Route cleared: Nav2 goals cancelled')
            return
        self.legs = legs_of(msg, self.robot_xy(msg.header.frame_id or 'map'))
        self.get_logger().info(f'New route: {len(msg.poses)} waypoints in {len(self.legs)} legs')
        # Nav2 runs one navigator at a time: a goal still running (RViz, ORO) would abort the route
        if not any(self.active.values()):
            self.next_leg(self.route)
            return
        self.cancel_nav2_goals()
        route, deadline = self.route, self.get_clock().now() + Duration(seconds=IDLE_WAIT)

        def check():
            if route != self.route:
                self.stop_waiting()
            elif not any(self.active.values()) or self.get_clock().now() > deadline:
                self.stop_waiting()
                self.next_leg(route)
        self.stop_waiting()
        self.waiting = self.create_timer(0.1, check)

    def stop_waiting(self):
        if self.waiting is not None:
            self.waiting.cancel()
            self.destroy_timer(self.waiting)
            self.waiting = None

    def cancel_nav2_goals(self):
        """Cancels every Nav2 goal, also the ones this node did not send (RViz, ORO's nav goal)."""
        for client in self.cancel_all.values():
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())   # zero goal id and stamp: cancel all

    def robot_xy(self, frame):
        try:
            t = self.tf.lookup_transform(frame, 'base_footprint', Time()).transform.translation
            return (t.x, t.y)
        except TransformException as e:
            self.get_logger().warn(f'No robot pose ({e}): a route back to the start may end early')
            return None

    def next_leg(self, route):
        if route != self.route:
            return
        if not self.legs:
            self.get_logger().info('Route done')
            return
        kind, arg = self.legs.pop(0)
        if kind == 'drive':
            client, goal = self.through, NavigateThroughPoses.Goal(poses=arg)
            what = f'driving through {len(arg)} waypoints'
        else:
            turns = abs(arg) / (2 * math.pi)
            client = self.spin
            goal = Spin.Goal(target_yaw=float(arg), time_allowance=Duration(
                seconds=max(10.0, SPIN_SECONDS_PER_TURN * turns)).to_msg())
            what = f'spinning {math.degrees(arg):.0f} deg'
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Nav2 action server {client._action_name} not up: route dropped')
            self.legs = []
            return
        self.get_logger().info(f'Leg: {what}, {len(self.legs)} legs after it')
        client.send_goal_async(goal).add_done_callback(lambda f: self.on_accepted(f, route))

    def on_accepted(self, future, route):
        handle = future.result()
        if route != self.route:
            if handle.accepted:
                handle.cancel_goal_async()   # a newer route took over while this one was sent
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the leg: route dropped')
            self.legs = []
            return
        self.leg_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, route))

    def on_result(self, future, route):
        if route != self.route:
            return
        self.leg_handle = None
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Leg ended with status {status}: route dropped')
            self.legs = []
            return
        self.next_leg(route)


def main():
    rclpy.init()
    rclpy.spin(Nav2Bridge())


if __name__ == '__main__':
    main()
