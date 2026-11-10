# ROS 2 Turtlesim Automation

Two small ROS 2 (Jazzy) projects that automate the classic **turtlesim** robot
using nodes, topics, services, parameters and a proportional (P) controller.

| Milestone | Package | What it does |
|---|---|---|
| 1 | [`milestone1_shape_drawer`](milestone1_shape_drawer) | The turtle draws a square, triangle, pentagon, hexagon, star or circle by itself |
| 2 | [`milestone2_turtle_pad`](milestone2_turtle_pad) | A clickable "Turtle Pad" next to turtlesim: click a point, or let the turtle chase your mouse |

Each folder is a **separate ROS 2 package** with its own launch file. You can
build and run either one without the other.

## Features

**Milestone 1 – Shape drawer**
- Six shapes: `square`, `triangle`, `pentagon`, `hexagon`, `star`, `circle`
- Choose the shape with a launch argument or a ROS parameter; switch it while running
- `/draw_shape` service (`std_srvs/Trigger`) to redraw
- Uses `/turtle1/cmd_vel`, `/turtle1/pose` and the turtlesim services `/clear`,
  `/turtle1/teleport_absolute` and `/turtle1/set_pen`
- Closed-loop driving: turn to face each corner, then drive with P control

**Milestone 2 – Turtle Pad (goal follower)**
- A 500 × 500 pad with turtlesim's blue that opens right next to the turtlesim window
- **Click mode**: click a point → a marker appears → the turtle drives there (marker turns green on arrival)
- **Follow mode**: the turtle continuously follows the mouse cursor
- Switch modes with **Space** or the on-screen button; the current mode is shown on the pad
- Live white dot shows the turtle's real position and heading
- P controller: turn towards the goal, drive forward, slow down when close, stop within 0.1
- Goals are kept 0.5 away from the walls, and "Goal reached" is logged
- Backup: type goals as `x y` in a terminal

## Repository layout

```
ros2-turtlesim-automation/
├── milestone1_shape_drawer/        # ROS 2 package
│   ├── launch/shapes.launch.py
│   └── milestone1_shape_drawer/shape_drawer.py
└── milestone2_turtle_pad/          # ROS 2 package
    ├── launch/turtle_pad.launch.py
    └── milestone2_turtle_pad/
        ├── goal_follower.py        # P controller: drives to /goal_point
        ├── goal_pad.py             # the clickable pad (tkinter)
        └── goal_input.py           # type "x y" goals in a terminal
```

## Requirements

- Ubuntu 24.04 with **ROS 2 Jazzy** (desktop install includes turtlesim)
- Python tkinter and `xwininfo` (used to place the pad beside turtlesim):

```bash
sudo apt install python3-tk x11-utils ros-jazzy-turtlesim
```

## Install and build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ayushi-n/ros2-turtlesim-automation.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select milestone1_shape_drawer milestone2_turtle_pad
source install/setup.bash
```

Run `source ~/ros2_ws/install/setup.bash` in every new terminal (or add it to `~/.bashrc`).

## Run Milestone 1 – Shape drawer

```bash
ros2 launch milestone1_shape_drawer shapes.launch.py shape:=star
```

Options: `shape:=square|triangle|pentagon|hexagon|star|circle` and `size:=<radius>` (default 3.0).
Change the shape while it runs, from a second terminal:

```bash
ros2 param set /shape_drawer shape circle
ros2 service call /draw_shape std_srvs/srv/Trigger     # redraw the current shape
```

## Run Milestone 2 – Turtle Pad

```bash
ros2 launch milestone2_turtle_pad turtle_pad.launch.py
```

- Click on the pad to send the turtle there.
- Press **Space** (or the button) to switch to follow mode and move the mouse.
- Add `typing:=true` to also open a terminal where you can type goals like `2 8`.
- Or send a goal by hand: `ros2 topic pub --once /goal_point geometry_msgs/msg/Point "{x: 2.0, y: 8.0}"`

Stop either demo with **Ctrl+C** in the launch terminal.

## How the P controller works

20 times per second the follower measures two errors from `/turtle1/pose`:
the **distance** to the goal and the **heading error** (how far it must turn).

- turning speed = `K_angular × heading error` → turns fast when far off, gently when almost aligned
- forward speed = `K_linear × distance × cos(heading error)` → fast when far, slow when close,
  and zero until it roughly faces the goal
- both are capped at a maximum, and it stops when the distance is below 0.1

## Notes

- On Ubuntu's Wayland desktop the launch files run turtlesim with `QT_QPA_PLATFORM=xcb`.
  This lets the pad find the turtlesim window, and avoids a turtlesim crash on Ctrl+C.
- turtlesim advances a fixed step per update, so on a slow virtual machine everything
  moves slower than real time. That is expected.

## License

Apache-2.0
