# go_to_room — MCG5138 Group 1

Sends the TurtleBot3 Waffle to any named room in the Gazebo House world.

---

## What is in this package

| File | What it does |
|---|---|
| `go_to_room/go_to_room_node.py` | **Server** — runs Nav2, listens for room requests |
| `go_to_room/go_to_room_client.py` | **Client** — sends a room name, waits for the result |
| `config/map.pgm` + `config/map.yaml` | The map — already bundled, no copy needed |
| `config/rooms.yaml` | Room names, boundaries, nav goals, doorway waypoints |
| `config/nav2_params.yaml` | Nav2 tuning (costmap, planner, AMCL) |
| `launch/navigation.launch.py` | Starts Nav2 + go_to_room_node in one command |

---

## Step 0 — One-time setup (only do this once ever)

Open a terminal and run these commands **in order**:

```bash
# 1. Go to your ROS 2 workspace source folder
cd ~/ros2_ws/src

# 2. Copy the package in (replace the path with wherever you extracted the zip)
cp -r /path/to/go_to_room .

# 3. Go back to the workspace root
cd ~/ros2_ws

# 4. Build ONLY the go_to_room package (faster than building everything)
colcon build --packages-select go_to_room

# 5. Source the workspace  ← YOU MUST DO THIS IN EVERY NEW TERMINAL
source ~/ros2_ws/install/setup.bash
```

> **If colcon says "package not found"** — check that the folder is called
> exactly `go_to_room` (no version numbers, no spaces).  Run `ls ~/ros2_ws/src/`
> to confirm you see `go_to_room` listed.

---

## Running the demo — 3 terminals

### Terminal 1 — Gazebo simulation

```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Wait until Gazebo has fully loaded (you see the house and the robot).

---

### Terminal 2 — Nav2 + go_to_room server

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go_to_room navigation.launch.py
```

**That is all.**  The map inside the package is used automatically.
You do NOT need to copy the map file or set any path.

Wait until you see these lines printed (takes ~20 seconds):

```
[go_to_room_node]: Nav2 is active — go_to_room_node ready.
[go_to_room_node]: Action server ready on /go_to_room_action
[go_to_room_node]: Topic listener ready on /go_to_room
```

> **If Terminal 2 says "package 'go_to_room' not found"** — you forgot to source.
> Run `source ~/ros2_ws/install/setup.bash` first, then try again.

> **If it says "map file does not exist"** — the package was not built after you
> added map.pgm.  Run `colcon build --packages-select go_to_room` again, then
> `source ~/ros2_ws/install/setup.bash`, then relaunch.

---

### Terminal 3 — Send the robot to a room

**Option A — Action client (recommended, gives you a result)**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run go_to_room go_to_room_client kitchen
```

The client prints:
```
[go_to_room_client]: Goal sent: navigate to 'kitchen'
[go_to_room_client]: Goal accepted — waiting for result …
[go_to_room_client]: Feedback received: Navigating to: kitchen
[go_to_room_client]: Result received: SUCCEEDED — arrived at 'kitchen'
```

**Option B — Topic (simple, fire-and-forget)**

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /go_to_room std_msgs/msg/String '{data: kitchen}' --once
```

Result comes back on `/go_to_room_result`:

```bash
ros2 topic echo /go_to_room_result
```

---

## Available rooms

| Room name | Description |
|---|---|
| `pantry` | Robot home — narrow centre strip |
| `kitchen` | NE large block |
| `living_room` | N-centre-W large block |
| `library` | NW block |
| `hallway` | Thin east-west corridor |
| `bedroom` | SW strip, below Library |
| `dining_room` | SE tall strip, below Kitchen |

Room names are **lower-case with underscores**.
`living room` and `Living_Room` also work — the node normalises them.

---

## How the action server works (Assignment 1 pattern)

```
CLIENT                          SERVER (go_to_room_node)
------                          ------------------------
send_goal("kitchen")   ──────►  "Goal received: kitchen"
                                accepts goal
                       ◄──────  GoalResponse.ACCEPT

                       ◄──────  Feedback: "Navigating to: kitchen"
                                  (published while Nav2 is running)

                       ◄──────  Result: SUCCEEDED  (or ABORTED)
```

The action type used is `nav2_msgs/action/NavigateToPose`.
The room name is passed in `goal.pose.header.frame_id` (a string field),
so no custom `.action` file is needed.

---

## Routing strategy

```
Pantry ──► hw_E ──► doorway ──► Kitchen
Pantry ──► hw_W ──► doorway ──► Library ──► doorway ──► Bedroom
Pantry ──► hw_W ──────────────► Living room
Pantry ──────────────────────► Hallway
```

`hw_W = (-0.03, -10.13)` and `hw_E = (2.32, -10.13)` are transit waypoints
in the thin hallway corridor.  The robot always passes through the correct one
before heading to the room.

---

## Coordinates reference

All coordinates are in the ROS `map` frame (metres).
Map: `map.pgm`  origin `[-7.43, -5.23]`  resolution `0.05 m/px`

| Room | Nav goal (x, y) | Doorway wp (x, y) |
|---|---|---|
| pantry ★ home | 1.72, −5.88 | 2.22, −5.88 |
| kitchen | 4.82, −10.13 | 2.32, −5.88 |
| living_room | −2.63, −10.13 | — |
| library | −6.28, −7.23 | −6.28, −9.48 |
| hallway | 1.07, −10.13 | — |
| bedroom | −6.28, −9.98 | −6.28, −9.53 |
| dining_room | 6.12, −11.78 | 6.12, −10.78 |

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `package 'go_to_room' not found` | Run `source ~/ros2_ws/install/setup.bash` in that terminal |
| `could not find package go_to_room` during colcon build | Check folder name is exactly `go_to_room`, no typos |
| `map file does not exist` | Rebuild: `colcon build --packages-select go_to_room` then source again |
| Robot does not move | Wait for "Nav2 is active" message before sending goals |
| `Action server not found after 15 s` | Terminal 2 (server) is not running — start it first |
| Robot spins in place endlessly | AMCL lost localisation — restart Terminal 2 |
| `Unknown room 'X'` | Check spelling — use lower-case with underscores |
