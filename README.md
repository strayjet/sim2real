# Sim2Real Hackathon — The Postman Problem

## Overview

This workspace contains the participant ROS 2 solution for all three tiers of Round 1 of the Sim2Real Robotics Hackathon. The robot must autonomously discover and visit the waypoint targets, stop at each target for the required dwell interval, account for targets that appear during a run, and return to the home zone.

The implementation is organized around three tier-specific nodes:

- **Tier 1 — Coordinate-based navigation:** consumes the permitted `/waypoints` and `/dynamic_waypoints` topics.
- **Tier 2 — Camera-based waypoint perception:** detects green target disks from `/overhead_camera/image_raw`; it does not subscribe to the waypoint coordinate topics.
- **Tier 3 — Camera-based perception and obstacle avoidance:** adds visual detection of obstacle walls and collision-aware grid path planning.

## Workspace contents

```text
.
├── README.md
└── src/
    ├── postman_pkg/          # Provided simulation platform and evaluator
    └── postman_solution/     # Participant solution
        ├── launch/
        │   └── solution.launch.py
        └── postman_solution/
            ├── tier1_node.py
            ├── tier2_node.py
            ├── tier3_node.py
            ├── nav_utils.py
            ├── vision_utils.py
            └── obstacle_utils.py
```

## Requirements

The official simulation environment specified by the rulebook is:

- Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic

The solution uses ROS 2 Python (`rclpy`), standard ROS messages, TF2, NumPy and OpenCV. Ensure the relevant ROS 2 packages and Python dependencies are installed in the evaluation environment.

## Build

Extract this archive, open a terminal at the extracted workspace root (the directory containing `src/` and `README.md`), and run:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Build artifacts are intentionally not included in this submission. The build commands generate `build/`, `install/`, and `log/` locally.

## Run

Use separate terminals for the simulator and the solution. In each terminal, source ROS 2 and the workspace overlay:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Terminal 1 — Start Gazebo simulation

Select the tier and seed as needed for local testing:

```bash
ros2 launch postman_pkg sim.launch.py tier:=1 seed:=42
```

Change `tier:=1` to `tier:=2` or `tier:=3` to use the corresponding environment. Use the launch arguments supported by the supplied simulation package if your local setup differs.

### Terminal 2 — Start the participant solution

For Tier 1:

```bash
ros2 launch postman_solution solution.launch.py tier:=1
```

For Tier 2:

```bash
ros2 launch postman_solution solution.launch.py tier:=2
```

For Tier 3:

```bash
ros2 launch postman_solution solution.launch.py tier:=3
```

### Optional — Run the supplied evaluator

From the workspace root, run the evaluator for the tier being tested:

```bash
python3 src/postman_pkg/scripts/evaluator.py --tier 1
```

Change `--tier 1` to the tier being evaluated. Follow the event's instructions for the official evaluation command and demonstration video.

## Implementation notes

### Shared navigation and state estimation

`postman_solution/tier1_node.py` provides the shared base implementation. It reads wheel joint information from `/joint_states` and yaw-rate information from `/imu`, estimates the robot pose, publishes `nav_msgs/Odometry` on `/odom`, broadcasts the `odom` to `base_link` transform, and publishes velocity commands on `/cmd_vel`.

`nav_utils.py` contains reusable navigation utilities, route ordering and a mission state machine for target visits, dwell handling, dynamic target insertion, and returning home. The controller limits commanded linear and angular velocities to the platform's operating limits.

### Tier 1

The Tier 1 node obtains target coordinates from `/waypoints` and `/dynamic_waypoints`. It plans the order of remaining targets, navigates to them, dwells, and then returns to the home zone. Already completed targets are not re-ordered when new targets are added.

### Tier 2

The Tier 2 node inherits the base navigation and state-estimation logic but disables subscriptions to waypoint coordinate topics. `vision_utils.py` processes the overhead camera image, detects green waypoint disks, maps image coordinates into the arena frame, and includes visual robot-pose measurements used to correct odometry. Camera calibration data is consumed from `/overhead_camera/camera_info` when available.

### Tier 3

The Tier 3 node builds on Tier 2 perception. `obstacle_utils.py` detects magenta obstacle-wall tops from the overhead image, converts their image-space corners into arena coordinates, inflates the detected obstacles to account for the robot footprint, and uses an A* grid planner with line-of-sight path simplification. The Tier 3 mission uses the resulting paths to navigate around detected obstacles while visiting targets and returning home.

## Competition constraints addressed

The implementation is designed around the Round 1 rules:

- Tier 1 uses the waypoint topics; Tiers 2 and 3 use visual target detection.
- The participant solution does not intentionally subscribe to `/evaluator` or Gazebo ground-truth pose topics.
- Navigation commands are published through `/cmd_vel`.
- Target visits include a stationary dwell before proceeding.
- Dynamic targets are incorporated into the remaining route.
- Tier 3 derives obstacle locations from camera imagery and plans around the detected obstacles.

The official rules define the waypoint capture radius, minimum continuous dwell, run time limit, and evaluation procedure. Final performance must be verified using the organizer's evaluation harness and unseen seeds.

## Submission contents

This archive contains the `src/` directory and this `README.md` only. Generated build products (`build/`, `install/`, `log/`), test artifacts, nested candidate archives, and Python bytecode caches are excluded.
