# milestone1_shape_drawer

The turtlesim turtle automatically draws a geometric shape.

```bash
ros2 launch milestone1_shape_drawer shapes.launch.py shape:=star
```

Shapes: `square`, `triangle`, `pentagon`, `hexagon`, `star`, `circle`. Optional `size:=<radius>` (default 3.0).

While it runs:

```bash
ros2 param set /shape_drawer shape hexagon            # draw a different shape
ros2 service call /draw_shape std_srvs/srv/Trigger    # redraw the current one
```

**How it works**
1. The shape becomes a list of corner points. A star visits every second corner of a pentagon.
2. The pen goes up, the turtle teleports to the first corner, and the screen is cleared.
3. The pen goes down, and for each corner the turtle **turns** to face it, then **drives**
   there using P control on `/turtle1/pose`.
4. A circle is drawn by driving forward while turning at a constant rate
   (angular speed = linear speed / radius) until the turtle has turned 360°.

Node: `shape_drawer`
- Publishes `/turtle1/cmd_vel`
- Subscribes to `/turtle1/pose`
- Calls `/clear`, `/turtle1/teleport_absolute` and `/turtle1/set_pen`
- Provides the `/draw_shape` service
