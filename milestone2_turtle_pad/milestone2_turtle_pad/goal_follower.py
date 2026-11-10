"""
goal_follower node
==================
Drives turtle1 to a goal point (x, y) with a proportional (P) controller.

Where do goals come from?
  Any node can publish a geometry_msgs/Point on the topic /goal_point:
    - mouse_goal  (click in the window, or follow the cursor)
    - goal_input  (type "x y" in a terminal)
    - or by hand: ros2 topic pub --once /goal_point geometry_msgs/msg/Point "{x: 2.0, y: 8.0}"

The P controller, 20 times per second:
  1. Measure two errors from /turtle1/pose:
       distance error = how far the goal is
       heading error  = angle between where the turtle faces and where the goal is
  2. Turning speed  = K_angular * heading error   (big error -> turn fast)
     Forward speed  = K_linear  * distance error  (far -> fast, close -> slow)
     The forward speed is multiplied by cos(heading error): full speed when we
     face the goal, zero when the goal is to our side or behind us. So the
     turtle first turns towards the goal, then drives.
  3. When distance < tolerance: stop and log "Goal reached".

Safety: goals are clamped into the window (with a margin), so the turtle
never drives into the walls.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, Twist
from turtlesim.msg import Pose

# Turtlesim's window goes from 0 to 11.088 in both x and y.
WORLD_SIZE = 11.088


def normalize_angle(angle):
    """Wrap any angle into [-pi, pi] so the turtle turns the short way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    """Limit value to the range [low, high]."""
    return max(low, min(value, high))


class GoalFollower(Node):

    def __init__(self):
        super().__init__('goal_follower')

        # ---------- Parameters ----------
        self.declare_parameter('k_linear', 1.5)      # P gain for forward speed
        self.declare_parameter('k_angular', 6.0)     # P gain for turning speed
        self.declare_parameter('max_linear', 2.0)    # speed limit (units/s)
        self.declare_parameter('max_angular', 4.0)   # turn-rate limit (rad/s)
        self.declare_parameter('tolerance', 0.1)     # "close enough" distance
        self.declare_parameter('wall_margin', 0.5)   # keep goals this far from walls

        # ---------- Topics ----------
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Point, '/goal_point', self.goal_callback, 10)

        self.pose = None   # latest turtle pose
        self.goal = None   # current goal (x, y), or None when there is nothing to do

        # Run the controller 20 times per second.
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Goal follower ready. Waiting for goals on /goal_point')

    # ------------------------------------------------------------------
    def pose_callback(self, msg):
        self.pose = msg

    def goal_callback(self, msg):
        """A new goal arrived: keep it inside the walls, then store it."""
        margin = self.get_parameter('wall_margin').value
        x = clamp(msg.x, margin, WORLD_SIZE - margin)
        y = clamp(msg.y, margin, WORLD_SIZE - margin)
        if (x, y) != (msg.x, msg.y):
            self.get_logger().warn(
                f'Goal ({msg.x:.2f}, {msg.y:.2f}) is too close to a wall, '
                f'using ({x:.2f}, {y:.2f}) instead',
                throttle_duration_sec=1.0)   # at most once per second
        self.goal = (x, y)
        # When following the cursor, goals arrive many times per second:
        # only print this message at most once per second.
        self.get_logger().info(f'New goal: ({x:.2f}, {y:.2f})',
                               throttle_duration_sec=1.0)

    # ------------------------------------------------------------------
    def control_loop(self):
        """The P controller. Runs every 0.05 s."""
        if self.pose is None or self.goal is None:
            return   # nothing to do yet

        # 1) Errors
        dx = self.goal[0] - self.pose.x
        dy = self.goal[1] - self.pose.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_goal - self.pose.theta)

        # 2) Reached?
        if distance < self.get_parameter('tolerance').value:
            self.cmd_pub.publish(Twist())   # all zeros = stop
            self.get_logger().info(
                f'Goal reached! Turtle at ({self.pose.x:.2f}, {self.pose.y:.2f})')
            self.goal = None
            return

        # 3) P control: speed = gain * error, then limit to a safe maximum.
        k_lin = self.get_parameter('k_linear').value
        k_ang = self.get_parameter('k_angular').value
        max_lin = self.get_parameter('max_linear').value
        max_ang = self.get_parameter('max_angular').value

        cmd = Twist()
        cmd.angular.z = clamp(k_ang * heading_error, -max_ang, max_ang)
        forward = clamp(k_lin * distance, 0.0, max_lin)
        # cos(error) is 1 when facing the goal, 0 when it is 90 deg to the side.
        # max(..., 0) means we never drive backwards: we turn round instead.
        cmd.linear.x = forward * max(math.cos(heading_error), 0.0)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalFollower()
    try:
        rclpy.spin(node)   # one thread only: callbacks and timer take turns
    except (KeyboardInterrupt, ExternalShutdownException):
        pass               # Ctrl+C: exit quietly
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
