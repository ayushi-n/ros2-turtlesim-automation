"""
goal_pad node
=============
An interactive "pad" that sits next to the turtlesim window. It is the same
size (500 x 500 px) and the same blue, and it maps 1:1 onto turtlesim's
11 x 11 world.

Two modes (press SPACE or the button to switch):
  CLICK  mode: click anywhere -> a marker appears and the turtle drives there.
               The marker turns green when the turtle has arrived.
  FOLLOW mode: the turtle continuously chases your mouse cursor.

The white dot (with a short line showing its heading) is the turtle's live
position from /turtle1/pose, so you can check it matches turtlesim.

This node only SENDS goals (geometry_msgs/Point on /goal_point).
The goal_follower node does the driving with its P controller.

Coordinates: tkinter pixels start at the TOP-left and y goes DOWN.
Turtlesim starts at the BOTTOM-left and y goes UP. So:
    world_x = pixel_x / SCALE
    world_y = (PAD_PX - pixel_y) / SCALE
"""

import math
import signal
import subprocess
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point
from turtlesim.msg import Pose

WORLD_SIZE = 11.088            # turtlesim world size, same in x and y
PAD_PX = 500                   # turtlesim's drawing area is 500 x 500 pixels
SCALE = PAD_PX / WORLD_SIZE    # pixels per world unit (~45.1)
TURTLESIM_BLUE = '#4556ff'     # turtlesim's background colour (r=69, g=86, b=255)
FOLLOW_PERIOD = 0.1            # follow mode: send at most 10 goals per second
REACHED_DISTANCE = 0.15        # marker turns green when the turtle is this close


def pixel_to_world(px, py):
    """tkinter pixel -> turtlesim coordinates."""
    return px / SCALE, (PAD_PX - py) / SCALE


def world_to_pixel(x, y):
    """turtlesim coordinates -> tkinter pixel."""
    return x * SCALE, PAD_PX - y * SCALE


def find_turtlesim_window():
    """Return (x, y, width, height) of the turtlesim window, or None.

    Uses the `xwininfo` tool. This only works when turtlesim runs through X11
    (the launch file sets QT_QPA_PLATFORM=xcb for that reason).
    """
    try:
        out = subprocess.run(['xwininfo', '-name', 'TurtleSim'],
                             capture_output=True, text=True, timeout=2).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    values = {}
    for line in out.splitlines():
        if ':' in line:
            key, _, val = line.partition(':')
            values[key.strip()] = val.strip()
    try:
        return (int(values['Absolute upper-left X']), int(values['Absolute upper-left Y']),
                int(values['Width']), int(values['Height']))
    except (KeyError, ValueError):
        return None


class GoalPad(Node):

    def __init__(self):
        super().__init__('goal_pad')
        self.declare_parameter('place_next_to_turtlesim', True)

        self.goal_pub = self.create_publisher(Point, '/goal_point', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.pose = None          # live turtle pose
        self.goal = None          # last goal we sent (x, y)
        self.mode = 'click'       # 'click' or 'follow'
        self.last_follow_time = 0.0
        self.stop_requested = False   # set by Ctrl+C, see main()
        self.build_window()

    # ------------------------------------------------------------------
    # Window
    # ------------------------------------------------------------------
    def build_window(self):
        self.root = tk.Tk()
        self.root.title('Turtle Pad')
        self.root.resizable(False, False)

        self.canvas = tk.Canvas(self.root, width=PAD_PX, height=PAD_PX,
                                bg=TURTLESIM_BLUE, highlightthickness=0, cursor='crosshair')
        self.canvas.pack()

        # Faint grid every 1 unit, numbers every 2 units, to help read positions.
        for i in range(1, 12):
            p = i * SCALE
            self.canvas.create_line(p, 0, p, PAD_PX, fill='#5967ff')
            self.canvas.create_line(0, PAD_PX - p, PAD_PX, PAD_PX - p, fill='#5967ff')
            if i % 2 == 0:
                self.canvas.create_text(p, PAD_PX - 8, text=str(i), fill='#c8ceff')
                self.canvas.create_text(9, PAD_PX - p, text=str(i), fill='#c8ceff')

        # Mode text (top-left) and switch button (top-right), drawn ON the pad
        # so the window stays exactly 500 x 500 like turtlesim.
        self.mode_text = self.canvas.create_text(10, 12, anchor='nw', fill='white',
                                                 font=('Helvetica', 12, 'bold'))
        button = tk.Button(self.canvas, text='Switch mode (Space)', command=self.toggle_mode,
                           takefocus=0)   # so Space never 'presses' it as well
        self.canvas.create_window(PAD_PX - 8, 8, anchor='ne', window=button)
        self.cursor_text = self.canvas.create_text(PAD_PX - 10, PAD_PX - 22, anchor='e',
                                                   fill='#c8ceff', text='')
        self.show_mode()

        # Mouse and keyboard events.
        self.canvas.bind('<Button-1>', self.on_click)
        self.canvas.bind('<Motion>', self.on_motion)
        self.canvas.bind('<Leave>', lambda e: self.canvas.itemconfig(self.cursor_text, text=''))
        self.root.bind('<space>', lambda e: self.toggle_mode())

        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)   # window close button

    def show_mode(self):
        if self.mode == 'click':
            text = 'Mode: CLICK  -  click to send the turtle'
        else:
            text = 'Mode: FOLLOW  -  turtle chases your cursor'
        self.canvas.itemconfig(self.mode_text, text=text)

    def place_window(self, tries_left=40):
        """Put the pad just to the right of turtlesim, at the same height.

        Turtlesim may take a few seconds to open, so we retry every 0.5 s.
        """
        geometry = find_turtlesim_window()
        if geometry is None:
            if tries_left > 0:
                self.root.after(500, self.place_window, tries_left - 1)
            else:
                self.get_logger().warn('Turtlesim window not found; '
                                       'the pad opens wherever the desktop puts it.')
                self.root.deiconify()
            return
        # xwininfo gives the outer frame (title bar included). Asking for our
        # frame at the same height and just to the right lines the two up.
        x, y, width, _ = geometry
        self.root.geometry(f'+{x + width + 10}+{y}')
        self.root.deiconify()

    # ------------------------------------------------------------------
    # Mouse / keyboard handlers
    # ------------------------------------------------------------------
    def toggle_mode(self):
        self.mode = 'follow' if self.mode == 'click' else 'click'
        self.show_mode()
        self.get_logger().info(f'Switched to {self.mode.upper()} mode')

    def on_click(self, event):
        if self.mode == 'click':
            self.send_goal(*pixel_to_world(event.x, event.y))

    def on_motion(self, event):
        x, y = pixel_to_world(event.x, event.y)
        self.canvas.itemconfig(self.cursor_text, text=f'({x:.1f}, {y:.1f})')
        if self.mode != 'follow':
            return
        # Don't flood the topic: at most one goal every FOLLOW_PERIOD seconds.
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_follow_time >= FOLLOW_PERIOD:
            self.last_follow_time = now
            self.send_goal(x, y)

    def send_goal(self, x, y):
        msg = Point()
        msg.x, msg.y = float(x), float(y)
        self.goal_pub.publish(msg)
        self.goal = (x, y)

    # ------------------------------------------------------------------
    # ROS + drawing
    # ------------------------------------------------------------------
    def pose_callback(self, msg):
        self.pose = msg

    def redraw(self):
        """Redraw the goal marker and the turtle dot (everything tagged 'dynamic')."""
        self.canvas.delete('dynamic')

        if self.goal is not None:
            gx, gy = world_to_pixel(*self.goal)
            reached = (self.pose is not None and
                       math.hypot(self.pose.x - self.goal[0],
                                  self.pose.y - self.goal[1]) < REACHED_DISTANCE)
            color = '#3ddc84' if reached else '#ff4040'   # green when reached, else red
            r = 8 if self.mode == 'click' else 5          # smaller marker in follow mode
            self.canvas.create_oval(gx - r, gy - r, gx + r, gy + r,
                                    outline=color, width=3, tags='dynamic')
            self.canvas.create_line(gx - r - 4, gy, gx + r + 4, gy, fill=color, width=2, tags='dynamic')
            self.canvas.create_line(gx, gy - r - 4, gx, gy + r + 4, fill=color, width=2, tags='dynamic')

        if self.pose is not None:
            tx, ty = world_to_pixel(self.pose.x, self.pose.y)
            hx = tx + 16 * math.cos(self.pose.theta)
            hy = ty - 16 * math.sin(self.pose.theta)   # minus: pixel y goes down
            self.canvas.create_line(tx, ty, hx, hy, fill='white', width=3, tags='dynamic')
            self.canvas.create_oval(tx - 6, ty - 6, tx + 6, ty + 6,
                                    fill='white', outline='black', tags='dynamic')

    def ros_tick(self):
        """Called by tkinter every 50 ms: handle ROS messages, then redraw.

        Doing ROS and the GUI in ONE thread keeps the program simple and safe.
        """
        if self.stop_requested:   # Ctrl+C was pressed: close the window
            self.root.quit()
            return
        rclpy.spin_once(self, timeout_sec=0)
        self.redraw()
        self.root.after(50, self.ros_tick)

    def run(self):
        if self.get_parameter('place_next_to_turtlesim').value:
            self.root.withdraw()       # stay hidden until we know where to go
            self.root.after(100, self.place_window)
        self.root.after(50, self.ros_tick)
        self.root.mainloop()           # runs until the window is closed


def main(args=None):
    # Handle Ctrl+C ourselves. If Ctrl+C raised an error inside a tkinter
    # callback, tkinter would just print it and keep running. Instead, Ctrl+C
    # (SIGINT) or a stop request from `ros2 launch` (SIGTERM) only sets a flag,
    # and ros_tick() closes the window cleanly the next time it runs.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GoalPad()

    def request_stop(signum, frame):
        node.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        node.run()   # returns when the window closes
    finally:
        node.root.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
