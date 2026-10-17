# Sim2Real Hackathon — The Postman Problem

**Round 1: Virtual Postman | ROS 2 Jazzy + Gazebo Harmonic**

This repository contains our participant-side solution for the Sim2Real Robotics Hackathon. The current implementation covers **Tier 1 (coordinate-based navigation)** and **Tier 2 (camera-based waypoint perception and navigation)**. **Tier 3 is not yet implemented**.

The goal is to autonomously visit the targets in the arena, stop at each target for the required dwell duration, handle targets that appear during the run, and return to the home zone.

## Current implementation status

| Tier / feature | Status | Implementation |
|---|---|---|
| Tier 1: encoder + IMU odometry | Implemented | `tier1_node.py` |
| Tier 1: `/waypoints` navigation | Implemented | `tier1_node.py`, `nav_utils.py` |
| Tier 1: dynamic waypoint insertion | Implemented | `tier1_node.py`, `nav_utils.py` |
| Tier 1: waypoint dwell and return home | Implemented | `nav_utils.py` |
| Tier 2: green-disk visual detection | Implemented | `tier2_node.py`, `vision_utils.py` |
| Tier 2: camera pixel-to-arena coordinate conversion | Implemented | `vision_utils.py` |
| Tier 2: camera-assisted pose correction | Implemented | `tier2_node.py` |
| Tier 2: detect newly spawned targets | Implemented | `tier2_node.py` |
| Tier 3: obstacle detection and collision-free planning | **Not implemented** | `tier3_node.py` remains unfinished |

“Implemented” describes the current code structure; performance should still be verified with the official evaluator, including unseen seeds.

## 1. Environment and dependencies

The provided simulation is intended for:

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3
- OpenCV and NumPy
- ROS–Gazebo integration packages

Install the packages used by the simulation and perception code if they are not already installed:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros \
  python3-opencv \
  python3-numpy
```

The solution's image conversion helper works directly with `sensor_msgs/Image`; `cv_bridge` is included as part of the suggested ROS setup but is not used for that conversion in the current implementation.

## 2. Build the workspace

Run build commands from the **workspace root**—the directory containing `src/`—not from inside a package.

```bash
cd ~/postman_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For every new terminal, source both the ROS installation and the workspace overlay:

```bash
cd ~/postman_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Replace `~/postman_ws` with the actual location of the extracted workspace.

## 3. Run the simulation and solution

Use separate terminals so the simulator, evaluator, and participant node can run independently.

### Terminal 1 — Launch Gazebo

For Tier 1:

```bash
ros2 launch postman_pkg sim.launch.py tier:=1 seed:=42
```

For Tier 2:

```bash
ros2 launch postman_pkg sim.launch.py tier:=2 seed:=42
```

Change `seed` to test different procedural layouts. Use the tier argument supported by the provided simulation launch file.

### Terminal 2 — Launch our solution

Tier 1:

```bash
ros2 launch postman_solution solution.launch.py tier:=1
```

Tier 2:

```bash
ros2 launch postman_solution solution.launch.py tier:=2
```

The launch file selects `tier1_node`, `tier2_node`, or `tier3_node` according to the `tier` argument.

### Terminal 3 — Run the evaluator

For Tier 1:

```bash
python3 src/postman_pkg/scripts/evaluator.py --tier 1
```

For Tier 2:

```bash
python3 src/postman_pkg/scripts/evaluator.py --tier 2
```

Run the evaluator using a terminal in the workspace root, with the ROS and workspace environments sourced. Check the evaluator's output for collected waypoints, dwell compliance, home return, collisions where applicable, distance, and elapsed simulation time.

## 4. How the solution works

### Tier 1 — Coordinate-based navigation

File: `src/postman_solution/postman_solution/tier1_node.py`

The Tier 1 node receives waypoint coordinates from ROS topics and performs state estimation and navigation.

**State estimation**
- Reads wheel joint data from `/joint_states`.
- Reads yaw rate from `/imu`.
- Converts wheel angular velocities into linear and angular velocity estimates using the wheel radius and wheel separation.
- Blends encoder-derived angular velocity with IMU yaw rate when IMU data is available.
- Integrates the estimated motion to maintain an `(x, y, heading)` pose.
- Publishes `nav_msgs/Odometry` on `/odom` and broadcasts the `odom` → `base_link` transform using TF2.

**Waypoint handling**
- Subscribes to `/waypoints` for the initial targets.
- Subscribes to `/dynamic_waypoints` for targets added during the run.
- Passes newly received points to the shared `Mission` controller.
- Uses the same navigation and dwell logic for static and dynamic targets.

### Tier 2 — Vision-guided navigation

Files:
- `src/postman_solution/postman_solution/tier2_node.py`
- `src/postman_solution/postman_solution/vision_utils.py`

Tier 2 inherits the odometry and navigation behaviour from Tier 1. It does **not** depend on `/waypoints` or `/dynamic_waypoints`; instead, it detects targets from the overhead camera.

**Visual waypoint detection**
- Subscribes to `/overhead_camera/image_raw` and `/overhead_camera/camera_info`.
- Converts incoming images to RGB and uses HSV colour thresholding to identify green target disks.
- Filters connected components by expected disk size, roundness, and fill ratio.
- Converts image pixel locations into arena-frame `(x, y)` coordinates using the camera intrinsics when available and the configured camera model otherwise.
- Requires detections to recur across multiple frames before registering them, reducing one-frame false detections.
- Deduplicates detections against previously confirmed targets.
- Continues scanning during the run so newly spawned green disks can be added to the remaining mission queue.

**Camera-assisted pose correction**
- Maintains a short history of odometry estimates to compare with the camera frame timestamp.
- Attempts to identify the robot body in the overhead image and estimate its position and, when the geometry is sufficiently clear, its heading.
- Applies a limited correction to the odometry estimate when the visual measurement is plausible; large position disagreements are rejected.
- Logs camera-fusion successes and skips to help diagnose perception/localisation issues.

The camera is therefore used both to locate target disks and to provide a visual reference for correcting accumulated odometry drift.

### Shared navigation — `nav_utils.py`

The shared navigation module contains the `Mission` state machine and route-ordering helpers.

- Removes duplicate waypoint coordinates.
- Computes the shortest route through a small set of remaining points by checking permutations (up to eight points); for larger sets it uses nearest-neighbour ordering followed by 2-opt improvement.
- Replans only the **unvisited** queue when new targets arrive, leaving already visited targets completed.
- Drives toward the next target with bounded linear and angular velocity commands.
- Rotates in place when the heading error is large, and slows down when the robot is not aligned with its target.
- Stops at a target and dwells before continuing.
- Returns to the home zone after the queue is empty and completes a final dwell.

The controller's configured dwell time is **2.6 seconds**, providing margin above the competition's minimum two-second dwell requirement. The configured target arrival radius is tighter than the evaluator's capture radius; actual scoring must be confirmed with the evaluator.

### Tier 2 perception helpers — `vision_utils.py`

The helper module includes:
- Camera intrinsics and pixel/world coordinate conversion.
- RGB image conversion without requiring `cv_bridge` for the conversion itself.
- Green waypoint-disk detection.
- Red home-marker detection for checking the pixel-to-world mapping.
- Robot-body detection for camera-assisted localisation.

## 5. Important topics

| Topic | Used for |
|---|---|
| `/joint_states` | Wheel encoder/joint information for odometry |
| `/imu` | Gyroscope yaw rate for heading estimation |
| `/cmd_vel` | Velocity commands sent to the robot |
| `/odom` | Estimated robot odometry published by our solution |
| `/tf` | Dynamic `odom` → `base_link` transform |
| `/overhead_camera/image_raw` | Tier 2 camera image input |
| `/overhead_camera/camera_info` | Tier 2 camera intrinsics |
| `/waypoints` | Tier 1 static waypoint coordinates |
| `/dynamic_waypoints` | Tier 1 dynamic waypoint coordinates |

The coordinate topics are used by Tier 1. Tier 2 obtains target positions visually.

## 6. Competition constraints addressed

The implementation is designed around the following Round 1 requirements:

- Forward-only linear movement; reverse commands are clamped to zero.
- Linear and angular velocity limits are enforced in the shared controller.
- Waypoints are added at runtime rather than assuming that all targets are available at startup.
- The robot must dwell at each target and return to the home zone.
- Tier 2 obtains target coordinates from the overhead camera rather than subscribing to the suppressed waypoint topics.
- The solution code is contained in `postman_solution`; the provided `postman_pkg` is the organizer's simulation/evaluation package and should not be modified.

Tier 3 additionally requires detecting obstacles and planning collision-free paths around them. That functionality is **not complete in this submission**.

## 7. Project layout

```text
postman_ws/
├── README.md
└── src/
    ├── postman_pkg/                   # Organizer-provided simulation package
    │   ├── launch/sim.launch.py
    │   ├── postman_pkg/
    │   ├── scripts/evaluator.py
    │   ├── urdf/
    │   └── worlds/
    └── postman_solution/              # Participant implementation
        ├── launch/solution.launch.py
        └── postman_solution/
            ├── tier1_node.py           # Odometry + coordinate-based navigation
            ├── tier2_node.py           # Camera perception + visual pose correction
            ├── tier3_node.py           # Tier 3 work in progress
            ├── nav_utils.py            # Route ordering + mission state machine
            └── vision_utils.py         # Camera geometry + visual detectors
```

## 8. Known limitations and next steps

- Tier 3 obstacle perception and collision-free path planning remain to be implemented and tested.
- Encoder/IMU dead reckoning accumulates error; camera-assisted correction in Tier 2 depends on successful robot-body detection and suitable camera frames.
- Visual detection uses colour and shape thresholds, so lighting, image encoding, partial occlusion, and camera calibration can affect detection quality.
- The exact route may be recomputed as new points are discovered; performance should be tested on multiple seeds.
- A successful build does not by itself guarantee a successful evaluation. Test each tier using the supplied evaluator and inspect the logs.

## Acknowledgements

The repository uses the ROS 2 and Gazebo packages supplied for the Sim2Real Hackathon. The organizer-provided `postman_pkg` contains the simulation environment, sensors, waypoint spawning, and evaluation harness. ROS 2, Gazebo, OpenCV, and NumPy are used as dependencies. Any additional third-party code or libraries should be acknowledged before final submission, in accordance with the hackathon rules.
