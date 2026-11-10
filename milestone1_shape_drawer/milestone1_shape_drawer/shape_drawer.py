"""
shape_drawer node
=================
Makes turtle1 draw a geometric shape automatically.

How it works (in short):
  1. Each shape is turned into a list of corner points (vertices).
  2. The turtle lifts its pen, teleports to the first corner, and clears the screen.
  3. It puts the pen down and drives to each corner in turn:
       - first it TURNS until it faces the next corner,
       - then it DRIVES forward, slowing down as it gets close.
     Both use a simple proportional (P) controller: speed = gain * error.
  4. A circle is different: the turtle drives forward while turning at a
     constant rate (angular speed = linear speed / radius) until it has
     turned a full 360 degrees.

How to choose the shape:
  - Parameter `shape` (square, triangle, pentagon, hexagon, star, circle).
    Changing it with `ros2 param set /shape_drawer shape star` draws the new shape.
  - Service `/draw_shape` (std_srvs/Trigger) redraws the current shape.

Topics / services used:
  /turtle1/cmd_vel   (publish)   -> tells the turtle how fast to move/turn
  /turtle1/pose      (subscribe) -> where the turtle is right now
  /clear, /turtle1/teleport_absolute, /turtle1/set_pen  (service clients)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute
from std_srvs.srv import Empty, Trigger

# The turtlesim window is 11 x 11 units; its centre is (5.544, 5.544).
CENTER_X = 5.544
CENTER_Y = 5.544

SUPPORTED_SHAPES = ['square', 'triangle', 'pentagon', 'hexagon', 'star', 'circle']


def normalize_angle(angle):
    """Wrap any angle into the range [-pi, pi] so the turtle turns the short way."""
    return math.atan2(math.sin(angle), math.cos(angle))


def polygon_vertices(sides, radius):
    """Corners of a regular polygon centred in the window.

    The corners lie on a circle of the given radius. We rotate the polygon
    so that its bottom edge is horizontal (looks nicer).
    """
    start_angle = -math.pi / 2 - math.pi / sides
    points = []
    for i in range(sides):
        a = start_angle + 2 * math.pi * i / sides
        points.append((CENTER_X + radius * math.cos(a),
                       CENTER_Y + radius * math.sin(a)))
    return points


def shape_vertices(shape, size):
    """Return the list of points the turtle must visit to draw `shape`.

    The last point equals the first one, so the outline is closed.
    """
    if shape == 'triangle':
        pts = polygon_vertices(3, size)
    elif shape == 'square':
        pts = polygon_vertices(4, size)
    elif shape == 'pentagon':
        pts = polygon_vertices(5, size)
    elif shape == 'hexagon':
        pts = polygon_vertices(6, size)
    elif shape == 'star':
        # A 5-pointed star: take the 5 corners of a pentagon (pointing up)
        # and visit every SECOND corner: 0 -> 2 -> 4 -> 1 -> 3.
        corners = []
        for i in range(5):
            a = math.pi / 2 + 2 * math.pi * i / 5
            corners.append((CENTER_X + size * math.cos(a),
                            CENTER_Y + size * math.sin(a)))
        pts = [corners[i] for i in (0, 2, 4, 1, 3)]
    else:
        raise ValueError(f'unknown shape "{shape}"')
    return pts + [pts[0]]


class ShapeDrawer(Node):

    def __init__(self):
        super().__init__('shape_drawer')

        # ---------- Parameters (can be set from launch file or command line) ----------
        self.declare_parameter('shape', 'square')    # which shape to draw
        self.declare_parameter('size', 3.0)          # radius of the shape (window is 11 wide)
        self.declare_parameter('linear_gain', 3.0)   # P gain for forward speed
        self.declare_parameter('angular_gain', 6.0)  # P gain for turning speed

        # ---------- Publisher and subscriber ----------
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose = None   # latest pose; None until the first message arrives
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # ---------- Clients for turtlesim's services ----------
        self.clear_client = self.create_client(Empty, '/clear')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # ---------- Our own service: redraw the current shape ----------
        self.create_service(Trigger, '/draw_shape', self.draw_shape_callback)

        # React when someone changes the `shape` parameter at run time.
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # The shape waiting to be drawn. The main loop (run()) picks it up.
        self.requested_shape = self.get_parameter('shape').value
        self.busy = False   # True while a shape is being drawn

    # ------------------------------------------------------------------
    # Callbacks (these run in the ROS spin thread)
    # ------------------------------------------------------------------
    def pose_callback(self, msg):
        self.pose = msg

    def draw_shape_callback(self, request, response):
        """Service /draw_shape: queue the current shape for drawing."""
        if self.busy:
            response.success = False
            response.message = 'Busy drawing, try again when it finishes.'
            return response
        self.requested_shape = self.get_parameter('shape').value
        response.success = True
        response.message = f'Drawing {self.requested_shape}'
        return response

    def on_parameter_change(self, params):
        """Check new parameter values; a new `shape` is queued for drawing."""
        for p in params:
            if p.name == 'shape':
                if p.value not in SUPPORTED_SHAPES:
                    return SetParametersResult(
                        successful=False,
                        reason=f'shape must be one of {SUPPORTED_SHAPES}')
                if self.busy:
                    return SetParametersResult(successful=False,
                                               reason='Busy drawing, try again later.')
                self.requested_shape = p.value
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def call_service(self, client, request):
        """Call a service and wait for the answer (works because spin runs in another thread)."""
        client.wait_for_service()
        # timeout: if ROS shuts down (Ctrl+C) we must not wait for an answer forever
        return client.call(request, timeout_sec=5.0)

    def set_pen(self, on):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width = 255, 255, 255, 3   # white pen, 3 px wide
        req.off = 0 if on else 1
        self.call_service(self.pen_client, req)

    def stop(self):
        self.cmd_pub.publish(Twist())   # an all-zero Twist means "stand still"

    def errors_to(self, x, y):
        """Return (distance, heading_error) from the turtle to point (x, y)."""
        dx = x - self.pose.x
        dy = y - self.pose.y
        distance = math.hypot(dx, dy)
        heading_error = normalize_angle(math.atan2(dy, dx) - self.pose.theta)
        return distance, heading_error

    def go_to(self, x, y, tolerance=0.03):
        """Drive the turtle in a straight line to (x, y) using P control.

        Phase 1 - TURN on the spot until we face the point.
        Phase 2 - DRIVE forward (with small steering corrections) until we are
                  close enough, or until the point is behind us (we passed it).
        """
        k_lin = self.get_parameter('linear_gain').value
        k_ang = self.get_parameter('angular_gain').value
        period = 1.0 / 50   # control loop runs 50 times per second

        # ---- Phase 1: turn ----
        while rclpy.ok():
            distance, heading_error = self.errors_to(x, y)
            if distance < tolerance or abs(heading_error) < 0.01:   # ~0.6 degrees
                break
            cmd = Twist()
            cmd.angular.z = k_ang * heading_error   # P control: turn faster when error is big
            self.cmd_pub.publish(cmd)
            time.sleep(period)
        self.stop()

        # ---- Phase 2: drive ----
        while rclpy.ok():
            distance, heading_error = self.errors_to(x, y)
            if distance < tolerance:
                break                  # close enough
            if abs(heading_error) > math.pi / 2:
                break                  # point is behind us: we just passed it, stop here
            cmd = Twist()
            # P control on distance: fast when far, slow when near (between 0.3 and 2.0).
            cmd.linear.x = min(max(k_lin * distance, 0.3), 2.0)
            # Small steering correction, but not in the last 10 cm (avoids wobbling).
            if distance > 0.1:
                cmd.angular.z = k_ang * heading_error
            self.cmd_pub.publish(cmd)
            time.sleep(period)
        self.stop()

    # ------------------------------------------------------------------
    # Drawing a whole shape
    # ------------------------------------------------------------------
    def drive_circle(self, radius, speed=1.5):
        """Drive forward while turning at a constant rate: this traces a circle.

        For a circle of radius r:  angular speed = linear speed / r.
        We add up how much the turtle has turned and stop after 2*pi (360 deg).
        Near the end we slow down (P control on the remaining angle) so we
        stop exactly where we started.
        """
        period = 1.0 / 50
        turned = 0.0
        last_theta = self.pose.theta
        while rclpy.ok():
            remaining = 2 * math.pi - turned
            if remaining <= 0.01:
                break
            scale = min(1.0, max(remaining / 0.5, 0.1))   # 1.0 normally, smaller at the end
            cmd = Twist()
            cmd.linear.x = speed * scale
            cmd.angular.z = (speed / radius) * scale
            self.cmd_pub.publish(cmd)
            time.sleep(period)
            # How much did we turn since the last loop? (normalize handles the
            # jump from +pi to -pi in turtlesim's angle)
            theta = self.pose.theta
            turned += normalize_angle(theta - last_theta)
            last_theta = theta
        self.stop()

    def teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        self.call_service(self.teleport_client, req)

    def draw(self, shape):
        size = self.get_parameter('size').value
        self.get_logger().info(f'Drawing a {shape}')

        # 1) Work out where to start and which way to face.
        if shape == 'circle':
            # Start at the bottom of the circle, facing right (counter-clockwise).
            start_x, start_y, start_theta = CENTER_X, CENTER_Y - size, 0.0
        else:
            points = shape_vertices(shape, size)
            start_x, start_y = points[0]
            nx, ny = points[1]
            start_theta = math.atan2(ny - start_y, nx - start_x)   # face the 2nd corner

        # 2) Pen up, jump to the start, wipe the old drawing.
        self.set_pen(False)
        self.teleport(start_x, start_y, start_theta)
        self.call_service(self.clear_client, Empty.Request())
        time.sleep(0.2)   # give the pose topic time to report the new position

        # 3) Pen down and draw.
        self.set_pen(True)
        if shape == 'circle':
            self.drive_circle(size)
        else:
            for (x, y) in points[1:]:
                self.go_to(x, y)
        self.get_logger().info(f'Finished the {shape}')

    def run(self):
        """Main loop: wait for a shape request, then draw it."""
        self.get_logger().info('Waiting for the first /turtle1/pose message...')
        while rclpy.ok() and self.pose is None:
            time.sleep(0.1)
        self.get_logger().info(f'Ready. Shapes: {", ".join(SUPPORTED_SHAPES)}')

        while rclpy.ok():
            if self.requested_shape is not None:
                shape, self.requested_shape = self.requested_shape, None
                self.busy = True
                try:
                    self.draw(shape)
                except ValueError as e:
                    self.get_logger().error(str(e))
                self.busy = False
            time.sleep(0.1)


def main(args=None):
    # We handle Ctrl+C ourselves (below), so tell rclpy NOT to install its own
    # Ctrl+C handler. rclpy's handler shuts ROS down from another thread at a
    # random moment, which can crash a program that uses two threads like ours.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ShapeDrawer()

    # The executor processes callbacks (pose messages, service requests).
    # It runs in a background thread so the main thread can run simple
    # step-by-step drawing code and still receive pose updates.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass   # Ctrl+C: Python raises KeyboardInterrupt here, in the main thread
    finally:
        # Shut down in a fixed order, while ROS is still running:
        node.stop()            # 1) stop the turtle
        executor.shutdown()    # 2) stop the executor -> the spin thread ends
        spin_thread.join()     # 3) wait until it has really ended
        node.destroy_node()    # 4) now nothing uses the node: safe to destroy
        rclpy.shutdown()       # 5) finally shut down ROS


if __name__ == '__main__':
    main()
