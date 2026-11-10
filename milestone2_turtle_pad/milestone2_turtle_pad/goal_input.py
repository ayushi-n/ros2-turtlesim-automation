"""
goal_input node
===============
Backup way to set a goal: type it in the terminal.

    Goal x y > 2 8        -> sends (2.0, 8.0) on /goal_point
    Goal x y > q          -> quit

Goals outside the walls are corrected by goal_follower.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class GoalInput(Node):

    def __init__(self):
        super().__init__('goal_input')
        self.goal_pub = self.create_publisher(Point, '/goal_point', 10)

    def send_goal(self, x, y):
        msg = Point()
        msg.x, msg.y = x, y
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalInput()
    print('Type a goal as "x y" (window is 0 to 11), or q to quit.')
    try:
        while rclpy.ok():
            text = input('Goal x y > ').strip()
            if text.lower() in ('q', 'quit', 'exit'):
                break
            # Accept "2 8", "2,8" or "2, 8".
            parts = text.replace(',', ' ').split()
            try:
                x, y = float(parts[0]), float(parts[1])
            except (ValueError, IndexError):
                print('  Please type two numbers, e.g.  2 8')
                continue
            node.send_goal(x, y)
            print(f'  Sent goal ({x:.2f}, {y:.2f})')
    except (KeyboardInterrupt, EOFError):
        pass   # Ctrl+C or Ctrl+D: quit
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
