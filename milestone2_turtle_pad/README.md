# milestone2_turtle_pad

A "Turtle Pad" that opens next to turtlesim. Click a point, or let the turtle follow your mouse.

```bash
ros2 launch milestone2_turtle_pad turtle_pad.launch.py
ros2 launch milestone2_turtle_pad turtle_pad.launch.py typing:=true   # also open a "type x y" terminal
```

| Mode | What happens |
|---|---|
| **Click** (default) | Click on the pad → a marker appears → the turtle drives there (the marker turns green on arrival) |
| **Follow** | The turtle continuously chases the mouse cursor |

Press **Space** or the **Switch mode** button to change mode. The white dot is the turtle's live position.

**Nodes**

| Node | Role |
|---|---|
| `goal_pad` | tkinter pad that publishes goals (`geometry_msgs/Point`) on `/goal_point` |
| `goal_follower` | P controller: reads `/turtle1/pose`, publishes `/turtle1/cmd_vel`, stops within 0.1 and logs "Goal reached". Goals are kept 0.5 away from the walls. |
| `goal_input` | Terminal backup: type `x y` to send a goal |

**P controller in one line:** turning speed = K × heading error, and
forward speed = K × distance × cos(heading error). So the turtle turns towards the goal,
drives forward, and slows down as it gets close.
