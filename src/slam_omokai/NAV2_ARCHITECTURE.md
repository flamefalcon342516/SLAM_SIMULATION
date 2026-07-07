# Nav2 & Full System Architecture — SLAM_OMOKAI

A top-to-bottom explanation of how this project works: where Nav2 comes from, how
the simulation, SLAM and navigation layers connect, what every Nav2 server does
internally, and how a single goal click turns into wheel rotation.

Companion to `README.md` (quick usage + troubleshooting). This document is the
deep dive.

---

## Table of contents

1. [Where Nav2 actually lives](#1-where-nav2-actually-lives)
2. [The four layers of the system](#2-the-four-layers-of-the-system)
3. [The TF tree — the backbone of everything](#3-the-tf-tree--the-backbone-of-everything)
4. [Layer 1: Simulation (Gazebo Harmonic)](#4-layer-1-simulation-gazebo-harmonic)
5. [Layer 2: The ros_gz bridge](#5-layer-2-the-ros_gz-bridge)
6. [Layer 3: SLAM (slam_toolbox)](#6-layer-3-slam-slam_toolbox)
7. [Layer 4: Nav2 — node-by-node](#7-layer-4-nav2--node-by-node)
   - [Lifecycle nodes & the lifecycle manager](#71-lifecycle-nodes--the-lifecycle-manager)
   - [bt_navigator — the orchestrator](#72-bt_navigator--the-orchestrator)
   - [planner_server — the global path](#73-planner_server--the-global-path)
   - [smoother_server — polishing the path](#74-smoother_server--polishing-the-path)
   - [controller_server — following the path](#75-controller_server--following-the-path)
   - [Costmaps — the world model](#76-costmaps--the-world-model)
   - [behavior_server — recoveries](#77-behavior_server--recoveries)
   - [velocity_smoother — the last gate](#78-velocity_smoother--the-last-gate)
   - [waypoint_follower](#79-waypoint_follower)
8. [Life of a goal — end-to-end trace](#8-life-of-a-goal--end-to-end-trace)
9. [Startup sequence and timing](#9-startup-sequence-and-timing)
10. [Topic & action reference](#10-topic--action-reference)
11. [Inspecting the running system](#11-inspecting-the-running-system)

---

## 1. Where Nav2 actually lives

**Nothing in this repo is Nav2 source code.** Nav2 is installed as binary Debian
packages with the rest of ROS 2 Humble:

```
ros-humble-navigation2   1.1.18   →  /opt/ros/humble/share/nav2_*  (all ~30 packages)
ros-humble-nav2-bringup  1.1.18   →  /opt/ros/humble/share/nav2_bringup
```

This repo contributes only **configuration and glue**:

| Repo file | Role |
|---|---|
| `launch/nav2.launch.py` | Includes the stock `nav2_bringup/launch/navigation_launch.py`, injecting our params file |
| `config/nav2_params.yaml` | Every Nav2 parameter: which plugins load and how they're tuned |
| `package.xml` | Declares `<exec_depend>nav2_bringup</exec_depend>` — a runtime dependency, not vendored code |

At launch, `get_package_share_directory("nav2_bringup")` resolves through the
sourced environment (`/opt/ros/humble/setup.bash`) to the apt install. If Nav2
were ever built from source in an overlay workspace and that workspace were
sourced, the same line would transparently resolve to the overlay instead —
that's standard ROS 2 workspace overlaying. Here, no overlay exists, so the
binary install is used.

The stock `navigation_launch.py` we include starts exactly these nodes:

```
controller_server, smoother_server, planner_server, behavior_server,
bt_navigator, waypoint_follower, velocity_smoother,
lifecycle_manager_navigation
```

Note what it does **not** start: `map_server` and `amcl`. Those live in a
different bringup file (`localization_launch.py`) and are unnecessary here
because slam_toolbox provides both the map and localization (§6).

---

## 2. The four layers of the system

```
┌─────────────────────────────────────────────────────────────────────────┐
│ LAYER 1 · SIMULATION            gz sim 8 (Harmonic)                     │
│   physics, gpu_lidar sensor, DiffDrive actuator, OdometryPublisher      │
└──────────────────────────────┬──────────────────────────────────────────┘
                               │ gz-transport topics
┌──────────────────────────────▼──────────────────────────────────────────┐
│ LAYER 2 · BRIDGE                ros_gz_bridge parameter_bridge          │
│   /clock  /scan  /odom  /tf  /joint_states   ←  gz → ROS                │
│   /cmd_vel                                    →  ROS → gz               │
└──────────────┬───────────────────────────────────────────┬──────────────┘
               │ /scan, /odom, TF                          │ /cmd_vel
┌──────────────▼──────────────────────────┐                │
│ LAYER 3 · SLAM   slam_toolbox           │                │
│   in : /scan + odom→base_footprint TF   │                │
│   out: /map (OccupancyGrid)             │                │
│        map→odom TF (drift correction)   │                │
└──────────────┬──────────────────────────┘                │
               │ /map, full TF tree                        │
┌──────────────▼───────────────────────────────────────────┴──────────────┐
│ LAYER 4 · NAV2                                                          │
│                                                                         │
│   /goal_pose ─► bt_navigator ─┬─► planner_server ──► global_costmap     │
│                (behavior tree)│         │  path                         │
│                               │         ▼                               │
│                               │   smoother_server                       │
│                               │         │  smoothed path                │
│                               │         ▼                               │
│                               ├─► controller_server ─► local_costmap    │
│                               │         │  /cmd_vel_nav (20 Hz)         │
│                               └─► behavior_server (recoveries)          │
│                                         │                               │
│                                         ▼                               │
│                               velocity_smoother ──► /cmd_vel ──► gz     │
│                                                                         │
│   lifecycle_manager_navigation: boots all of the above, in order        │
└─────────────────────────────────────────────────────────────────────────┘
```

Each layer only needs the ones below it to be up, which is exactly why
`bringup.launch.py` staggers them with timers (§9).


## 3. The TF tree — the backbone of everything

Every component communicates positions through TF. The full tree at runtime:

```
map ──► odom ──► base_footprint ──► base_link ──► left_wheel_link
 ▲       ▲                            ├──────────► right_wheel_link
 │       │                            ├──────────► front_caster_link
 │       │                            ├──────────► rear_caster_link
 │       │                            └──────────► lidar_link
 │       │
 │       └── published by gz OdometryPublisher (via bridge, 30 Hz)
 └────────── published by slam_toolbox (transform_publish_period 0.02 s)
```

Who publishes what, and why it's split this way:

| Transform | Publisher | Meaning |
|---|---|---|
| `map → odom` | slam_toolbox | The **drift correction**. Odometry drifts over time; SLAM compares lidar scans against its map and publishes the offset that makes odometry line up with reality. Jumps discontinuously when loop closure fires. |
| `odom → base_footprint` | gz OdometryPublisher (bridged on `/tf`) | The robot's **smooth, continuous** motion estimate. Never jumps — that's its contract; controllers depend on it being differentiable. In this sim it's ground truth; on a real robot it would come from wheel encoders + IMU fusion. |
| `base_footprint → …` | robot_state_publisher | The **rigid body** structure from the URDF (`omokai_bot.xacro`). `base_footprint` is the ground-level frame; `base_link` sits `0.16 m` above it at chassis center; `lidar_link` is on top of the chassis. |

This split is the standard ROS convention (REP-105): work needing **smoothness**
(local control) happens in `odom`; work needing **global consistency** (planning
on the map) happens in `map`. You can see the convention reflected directly in
`nav2_params.yaml`: the global costmap's `global_frame` is `map`, the local
costmap's and behavior server's is `odom`, and everyone's `robot_base_frame` is
`base_footprint`.

---

## 4. Layer 1: Simulation (Gazebo Harmonic)

`sim.launch.py` starts `gz sim 8` with `worlds/omokai_world.sdf` (walls plus
movable boxes/cylinders) and spawns the robot at world (−5, −5).

The robot (`urdf/omokai_bot.xacro`) is a true differential drive: a 0.60 × 0.40 m
chassis, **two driven wheels** on the center axle (0.45 m apart, 0.10 m radius)
and **two near-frictionless caster spheres** fore/aft. (The original 4-wheel
skid-steer design physically could not pivot in this sim — see README §8.)

Three gz-sim plugins (`urdf/ign_plugins.xacro`) make it come alive:

- **DiffDrive** — the *actuator*. Subscribes to `/cmd_vel` (Twist), converts
  linear+angular velocity into left/right wheel speeds using the wheel
  separation and radius, and drives the joints. Also computes encoder odometry,
  published on `/wheel_odom` for debugging only.
- **OdometryPublisher** — the *odometry source actually used*. Publishes the
  model's ground-truth motion as `/odom` and the `odom → base_footprint` TF at
  30 Hz. A sim-only shortcut: perfect odometry means SLAM quality depends only
  on scan matching, not on wheel slip.
- **JointStatePublisher** — publishes wheel joint angles so
  robot_state_publisher can pose the wheel meshes correctly.

The **gpu_lidar** sensor on `lidar_link`: 360 samples over a full 360°,
0.25–12 m range, 10 Hz, Gaussian noise σ = 5 mm. This single sensor feeds
*both* SLAM and *both* Nav2 costmaps — it is the robot's only perception.

One critical environment detail: `sim.launch.py` pins `GZ_IP=127.0.0.1`.
Without it, gz-transport can bind to a `docker0` interface and the sim↔bridge
channels silently die mid-run — `/clock`, `/scan`, `/odom` freeze while every
process still looks alive.

---

## 5. Layer 2: The ros_gz bridge

Gazebo and ROS 2 have separate transport systems; the `parameter_bridge` node
translates between them, message by message:

| Topic | Direction | Type | Consumer |
|---|---|---|---|
| `/clock` | gz → ROS | Clock | Every node (`use_sim_time: true` everywhere — ROS time follows sim time, so pausing the sim pauses the whole stack) |
| `/scan` | gz → ROS | LaserScan | slam_toolbox + both costmap obstacle layers |
| `/odom` | gz → ROS | Odometry | Nav2 (bt_navigator progress, velocity_smoother feedback) |
| `/tf` | gz → ROS | TFMessage | The `odom → base_footprint` transform |
| `/joint_states` | gz → ROS | JointState | robot_state_publisher (remapped from the long gz model topic) |
| `/cmd_vel` | ROS ⇄ gz (bidirectional) | Twist | The DiffDrive actuator — this is where Nav2's output leaves ROS |

---

## 6. Layer 3: SLAM (slam_toolbox)

`slam.launch.py` runs slam_toolbox's **online async** node with
`config/slam_params.yaml`. "Online" = live, while the robot drives; "async" =
process the newest scan and skip backlog rather than falling behind.

What it does, continuously:

1. Take a `/scan`, look up where odometry says the robot is.
2. **Scan-match** against the map built so far (correlative search, then Ceres
   solver refinement) to correct the odometric pose estimate.
3. Insert the scan into the pose graph; every `map_update_interval` (2 s)
   re-render the `/map` occupancy grid (5 cm resolution).
4. Publish `map → odom` as the difference between the matched pose and the
   odometric pose.
5. Watch for **loop closures**: when the robot revisits somewhere
   (`loop_search_maximum_distance` 4 m), re-optimize the whole graph and warp
   the map back into global consistency.

Scans are only processed after 0.2 m or 0.2 rad of travel
(`minimum_travel_*`) — a stationary robot adds no redundant data.

**Why this replaces two Nav2 components:** the classic Nav2 localization stack
is `map_server` (serves a static, pre-built map) + `amcl` (particle-filter
localization within it). slam_toolbox does both jobs at once *without needing a
map to exist beforehand* — it publishes `/map` and `map → odom` itself. That is
why `nav2.launch.py` uses `navigation_launch.py` (navigation servers only)
rather than the full `bringup_launch.py`, and why Nav2 here can navigate an
environment it has never seen: the global costmap's static layer simply mirrors
whatever SLAM has mapped *so far*, and the planner is allowed to route through
unknown space (`allow_unknown: true`).

---

## 7. Layer 4: Nav2 — node-by-node

Nav2 is not one program. It is a **federation of specialized servers**, each a
separate process exposing ROS actions/services, each loading its actual
algorithms as **plugins** chosen in `nav2_params.yaml`. The architecture is
plugins all the way down: swap a YAML string and you have a different planner,
controller, or recovery without recompiling anything.

### 7.1 Lifecycle nodes & the lifecycle manager

Every Nav2 server is a **managed lifecycle node** with explicit states:

```
unconfigured ──configure──► inactive ──activate──► active
     ▲                          │                     │
     └──────────cleanup─────────┘◄────deactivate──────┘
```

- `configure`: load parameters, create plugins, allocate memory, set up
  publishers/subscribers — but do nothing yet.
- `activate`: start processing. Only now are actions accepted.

`lifecycle_manager_navigation` (started with `autostart: true` in our launch)
walks every server through `configure` then `activate` **in dependency order**,
and afterwards heartbeat-checks them, so the stack comes up deterministically
instead of racing.

**The practical consequence** (worth internalizing for this project): action
goals sent to a server that isn't `active` yet are **silently dropped** — no
error, no feedback. This is why `bringup.launch.py` prints the
`NAV2 IS ACTIVE` banner only after polling
`ros2 lifecycle get /bt_navigator` returns `active` (~25 s after launch).

### 7.2 bt_navigator — the orchestrator

`bt_navigator` owns the top-level actions `NavigateToPose` and
`NavigateThroughPoses`. It contains **no navigation logic itself**. Instead, it
loads a **behavior tree** — an XML file of composable nodes — and *ticks* it
every `bt_loop_duration` (10 ms). Each BT node returns `SUCCESS`, `FAILURE`, or
`RUNNING`; control-flow nodes combine those results into the overall behavior.

The BT nodes themselves are plugins: the 50-odd libraries in
`plugin_lib_names` in `nav2_params.yaml` are shared objects loaded at
configure-time. Four categories:

| Category | Returns | Examples here |
|---|---|---|
| **Action** | RUNNING until its work (usually an action-server call) ends | `ComputePathToPose`, `FollowPath`, `Spin`, `BackUp`, `Wait`, `ClearEntireCostmap` |
| **Condition** | instant SUCCESS/FAILURE check | `GoalUpdated`, `IsStuck`, `IsPathValid`, `GoalReached`, `TransformAvailable` |
| **Control** | ticks children per a policy | `PipelineSequence`, `RecoveryNode`, `RoundRobin`, `ReactiveFallback` |
| **Decorator** | wraps exactly one child | `RateController`, `DistanceController`, `SpeedController`, `GoalUpdater` |

This project uses the stock default tree,
`navigate_to_pose_w_replanning_and_recovery.xml` (from
`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/`):

```
RecoveryNode "NavigateRecovery"  (number_of_retries: 6)
├─ PipelineSequence "NavigateWithReplanning"
│  ├─ RateController (1 Hz)
│  │  └─ RecoveryNode
│  │     ├─ ComputePathToPose (planner_id="GridBased")
│  │     └─ ClearEntireCostmap (global)          ← if planning fails
│  └─ RecoveryNode
│     ├─ FollowPath
│     └─ ClearEntireCostmap (local)              ← if control fails
└─ ReactiveFallback "RecoveryFallback"           ← if the whole pipeline fails
   ├─ GoalUpdated                                (new goal? abort recovery)
   └─ RoundRobin
      ├─ ClearEntireCostmap (both)
      ├─ Spin 90°
      ├─ Wait 5 s
      └─ BackUp 0.3 m
```

The magic ingredient is **`PipelineSequence`**: unlike a normal Sequence, it
keeps re-ticking *earlier* children while a later one is `RUNNING`. So while
`FollowPath` runs continuously, `ComputePathToPose` is re-ticked too — throttled
to 1 Hz by the `RateController` — giving **continuous replanning during
driving**. A fresh path every second means newly-mapped obstacles (this is live
SLAM; the map changes constantly) are routed around immediately.

`RecoveryNode` pairs a main child with a repair child: main fails → tick repair
→ retry main. `RoundRobin` hands out a *different* recovery on each successive
failure, escalating from cheap (clear costmaps) to disruptive (back up).

Data flows between BT nodes via the **blackboard** — a shared key/value store.
The goal pose is written to it when the action arrives; `ComputePathToPose`
writes `{path}`; `FollowPath` reads `{path}`.

To use a custom tree, set `default_nav_to_pose_bt_xml` in the `bt_navigator`
params, or send a per-goal tree in the `behavior_tree` field of the
`NavigateToPose` goal.

### 7.3 planner_server — the global path

Hosts the `ComputePathToPose` action. Given start (current pose in `map`) and
goal, it asks the plugin named by the BT's `planner_id` for a path through the
**global costmap**. Two plugins are loaded (`nav2_params.yaml`):

- **`GridBased` → SmacPlanner2D** *(the default the BT uses)* — cost-aware A*
  on the 2D grid. Configured with:
  - `allow_unknown: true` — may plan through unmapped cells. **Essential** in
    this project: the map is being built as the robot drives, and most goals
    initially lie in grey unknown space.
  - `tolerance: 0.5` — if the exact goal cell is infeasible, accept the
    nearest feasible pose within 0.5 m.
  - `cost_travel_multiplier: 2.0` — traveling through higher-cost (near-
    obstacle) cells is penalized, pushing paths toward corridor centers.
- **`NavFn` → NavfnPlanner** — the classic ROS 1 Dijkstra potential-field
  planner, kept loaded as a fallback (a BT could select it via
  `planner_id: NavFn`). It was demoted from default because it intermittently
  fails with *"Failed to create a plan from potential"* on partially-unknown
  SLAM maps — exactly the maps this project always has.

Other planners available in the binary install, for reference: 
**SmacPlannerHybrid** (Hybrid-A* over x, y, θ — kinematically-feasible curves
for car-like robots), **SmacPlannerLattice** (state-lattice A* for arbitrary
motion models), **ThetaStar** (any-angle A*, straighter paths in open space).
A diff-drive robot that spins in place doesn't need them.

### 7.4 smoother_server — polishing the path

Grid-based A* paths hug cell boundaries and have staircase corners. The BT's
`SmoothPath` action sends the raw path here; `nav2_smoother::SimpleSmoother`
iteratively relaxes waypoints toward their neighbors (up to `max_its: 1000`),
checking against the costmap so smoothing never pulls the path into an
obstacle. Smoother input paths make DWB's path-tracking critics behave better.

### 7.5 controller_server — following the path

Hosts the `FollowPath` action. At `controller_frequency` (**20 Hz**) it
computes the next velocity command from the current pose, the path, and the
**local costmap**. Three plugin sockets:

**The controller: `dwb_core::DWBLocalPlanner`** (Dynamic Window Approach). Each
cycle:

1. **Sample** the velocity space reachable within acceleration limits:
   `vx_samples: 20` × `vtheta_samples: 20` = up to 400 candidate `(vx, vθ)`
   pairs within `max_vel_x: 0.5 m/s`, `max_vel_theta: 1.0 rad/s`.
2. **Simulate** each candidate forward `sim_time: 1.7 s`, producing 400 short
   trajectory arcs.
3. **Score** every arc with the configured **critics**, sum the weighted
   scores, and command the arc with the lowest cost:

| Critic | Scale | What it punishes/rewards |
|---|---|---|
| `BaseObstacle` | 0.02 | Trajectories passing through high-cost costmap cells (collision safety) |
| `PathAlign` | 32.0 | Heading away from the path direction |
| `PathDist` | 32.0 | Ending far from the path |
| `GoalAlign` | 24.0 | Not facing toward the goal |
| `GoalDist` | 24.0 | Ending far from the goal |
| `RotateToGoal` | 32.0 | Once at the goal position: not rotating to the goal heading (with a slowing profile) |
| `Oscillation` | — | Flip-flopping between forward/backward or left/right decisions |

The tuning tells a story: path critics (32) outweigh goal critics (24) —
follow the planned path rather than beeline; `BaseObstacle` is tiny (0.02)
because the *inflation layer* already shapes costs smoothly, so the critic only
needs to veto genuinely dangerous arcs.

**The progress checker: `SimpleProgressChecker`** — the robot must move
≥ 0.3 m every 20 s, else `FollowPath` aborts → the BT declares failure → the
recovery branch takes over. This is the "am I stuck?" detector.

**The goal checker: `SimpleGoalChecker`** — declares success within
`xy_goal_tolerance: 0.25 m` and `yaw_goal_tolerance: 0.25 rad`. `stateful:
true` means once the position tolerance is met it only checks yaw — the robot
won't chase millimeters if it drifts slightly while rotating in place.

Output goes to `/cmd_vel_nav` — *not* directly to the robot.

### 7.6 Costmaps — the world model

Both planners and controllers reason over **costmaps**: 2D grids where each
cell holds 0 (free) … 253 (inscribed) … 254 (lethal obstacle) … 255 (unknown).
Costmaps are built from stacked, pluggable **layers**, each painting onto the
grid in order. Two instances run (inside planner_server and controller_server
respectively):

| | `global_costmap` | `local_costmap` |
|---|---|---|
| Frame | `map` (SLAM-corrected) | `odom` (smooth) |
| Extent | the whole known map | 5 × 5 m rolling window centered on the robot |
| Update rate | 1 Hz | 5 Hz |
| Served to | planner (strategic) | controller + behaviors (tactical) |
| Layers | static + obstacle + inflation | obstacle + inflation |

- **StaticLayer** (global only) — subscribes to `/map` from slam_toolbox
  (`map_subscribe_transient_local: true` latches the last map). Because SLAM
  republishes every 2 s, the "static" layer is actually *live* — walls appear
  in the global costmap as they are discovered. `track_unknown_space: true`
  keeps unmapped cells marked unknown instead of assuming them free, which
  pairs with the planner's `allow_unknown` to make exploration deliberate.
- **ObstacleLayer** (both) — marks lidar returns from `/scan` as lethal
  (`obstacle_max_range: 10 m`) and **raytrace-clears** the cells along each
  beam (`raytrace_max_range: 12 m`): seeing *through* space proves it empty.
  This is what erases a moved box from the costmap — but only when the lidar
  gets line-of-sight past its old position again.
- **InflationLayer** (both) — spreads an exponentially-decaying cost
  (`cost_scaling_factor: 3.0`) out to `inflation_radius: 0.55 m` around every
  lethal cell. Combined with `robot_radius: 0.35`, this converts "the robot is
  a circle" into "the robot is a point, but obstacles are fatter" — which is
  what lets planners treat the robot as a single cell. The 0.2 m of gradient
  beyond the lethal-for-the-robot zone is the "keep away" slope DWB and Smac
  surf on.

Layers available but unused here: VoxelLayer (3D from depth cameras),
RangeLayer (sonar/IR), DenoiseLayer (speckle removal).

### 7.7 behavior_server — recoveries

Hosts one action server per loaded behavior plugin: **Spin** (rotate in place
by a requested angle), **BackUp** (reverse a distance), **DriveOnHeading**
(forward a distance), **Wait** (timed pause). The BT's recovery branch calls
these when navigation stalls.

They are not blind motions: each behavior forward-simulates the commanded
motion (`simulate_ahead_time: 2.0 s`) against the **local costmap** and aborts
with *"Collision Ahead"* rather than executing into an obstacle. They run in
the `odom` frame (motion doesn't need global consistency) and are capped at
`max_rotational_vel: 1.0 rad/s`.

### 7.8 velocity_smoother — the last gate

Sits between the controller and the robot: subscribes `/cmd_vel_nav`, publishes
`/cmd_vel` at 20 Hz. It clamps velocity to `[0.5, 0, 1.0]` (x, y, θ) and —
more importantly — acceleration to `[1.5, 0, 2.0]`, so a controller decision
can never demand an instantaneous velocity jump the physical robot (or the
DiffDrive plugin) can't perform. `feedback: OPEN_LOOP` means it integrates its
own last command instead of trusting `/odom` for current velocity;
`velocity_timeout: 1.0` stops the robot if the controller goes silent for a
second — a deadman switch.

Note the three rotation caps that must agree (a real tuning gotcha in this
project): DWB `max_vel_theta`, behavior_server `max_rotational_vel`, and
velocity_smoother `max_velocity[2]` — all 1.0 rad/s. The tightest one wins.

### 7.9 waypoint_follower

Hosts `FollowWaypoints`: takes a list of poses, calls `NavigateToPose` for each
in turn, and runs a task executor plugin at each arrival (`WaitAtWaypoint`,
200 ms pause here). `stop_on_failure: false` — an unreachable waypoint is
skipped, not fatal. Unused by the RViz Nav2 Goal tool but available for
multi-stop patrol scripts.

---

## 8. Life of a goal — end-to-end trace

What happens between clicking **Nav2 Goal** in RViz and the robot arriving:

```
 1. RViz publishes PoseStamped on /goal_pose
 2. bt_navigator (its own small BT subscriber) wraps it into a
    NavigateToPose action goal → writes goal onto the BT blackboard
 3. BT tick → PipelineSequence → RateController fires (1 Hz slot free)
 4. ComputePathToPose BT node → action call to planner_server
 5. planner_server: current pose from TF (map→base_footprint),
    SmacPlanner2D searches the global costmap → nav_msgs/Path
 6. Path smoothed by smoother_server → written to blackboard {path}
 7. FollowPath BT node → action call to controller_server (stays RUNNING)
 8. controller_server @ 20 Hz:
      pose in odom frame → DWB samples 400 arcs → critics score
      → best (vx, vθ) → /cmd_vel_nav
 9. velocity_smoother: accel-clamp → /cmd_vel
10. ros_gz bridge: /cmd_vel → gz-transport
11. DiffDrive plugin: v, ω → left/right wheel speeds → physics steps
12. gpu_lidar sees the new surroundings → /scan → slam_toolbox updates
    /map and map→odom  →  costmaps update  →  (loop to step 3: replanning
    at 1 Hz uses the newest map; step 8 keeps following at 20 Hz)
13. SimpleGoalChecker: within 0.25 m & 0.25 rad → FollowPath returns
    SUCCESS → BT returns SUCCESS → NavigateToPose action succeeds
    → "Goal succeeded" in the terminal
```

If step 5 or 8 fails, the enclosing `RecoveryNode` clears the corresponding
costmap and retries; if the whole pipeline fails, the `RoundRobin` recoveries
(clear → spin → wait → backup) run and the pipeline restarts — up to 6 rounds
before the action finally aborts.

---

## 9. Startup sequence and timing

`bringup.launch.py` staggers the layers because each depends on the last:

| t (s) | Event |
|---|---|
| 0 | gz sim starts; robot spawns; bridge up; robot_state_publisher up; RViz opens. `/clock` starts ticking → sim time exists |
| 5 | slam_toolbox starts (needs `/scan`, `/clock`, and the `odom → base_footprint` TF to already exist). First `/map` and `map → odom` appear |
| 8 | Nav2 servers start; lifecycle_manager begins configure → activate walk. Costmaps need the *full* TF chain `map → odom → base_footprint`, hence after SLAM |
| 9 | Banner watcher starts polling `ros2 lifecycle get /bt_navigator` |
| ~25 | Everything `active` → **`NAV2 IS ACTIVE`** banner. Goals accepted from here on |

Goals sent before the banner are silently dropped (see §7.1). Every accepted
goal logs `Begin navigating from (…) to (…)` in the launch terminal — the
definitive "Nav2 heard me" signal.

---

## 10. Topic & action reference

**Topics (the data plane):**

| Topic | Type | From → To |
|---|---|---|
| `/scan` | LaserScan | gz lidar → slam_toolbox, both costmaps |
| `/map` | OccupancyGrid | slam_toolbox → global static layer, RViz |
| `/odom` | Odometry | gz OdometryPublisher → Nav2 |
| `/wheel_odom` | Odometry | gz DiffDrive → (debug only, nothing consumes it) |
| `/tf`, `/tf_static` | TFMessage | slam_toolbox + gz + robot_state_publisher → everyone |
| `/goal_pose` | PoseStamped | RViz Nav2 Goal tool → bt_navigator |
| `/cmd_vel_nav` | Twist | controller_server → velocity_smoother |
| `/cmd_vel` | Twist | velocity_smoother → gz DiffDrive |
| `/plan` | Path | planner_server → RViz (global path display) |
| `/local_plan` | Path | DWB → RViz (chosen local trajectory) |
| `/global_costmap/costmap`, `/local_costmap/costmap` | OccupancyGrid | costmaps → RViz |

**Actions (the control plane):**

| Action | Server | Called by |
|---|---|---|
| `/navigate_to_pose` | bt_navigator | you / RViz / waypoint_follower |
| `/navigate_through_poses` | bt_navigator | you |
| `/compute_path_to_pose` | planner_server | BT `ComputePathToPose` node |
| `/smooth_path` | smoother_server | BT `SmoothPath` node |
| `/follow_path` | controller_server | BT `FollowPath` node |
| `/spin`, `/backup`, `/drive_on_heading`, `/wait` | behavior_server | BT recovery nodes |
| `/follow_waypoints` | waypoint_follower | you |

---

## 11. Inspecting the running system

```bash
# Lifecycle state of any server (the readiness check)
ros2 lifecycle get /bt_navigator

# All Nav2 nodes present?
ros2 node list | grep -E "navigator|server|smoother|costmap|lifecycle"

# Watch the TF tree (generates frames.pdf)
ros2 run tf2_tools view_frames

# Is the map→odom correction alive?
ros2 run tf2_ros tf2_echo map odom

# Send a goal with live feedback (distance remaining, recovery count)
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"

# Watch the controller's output vs. what reaches the robot
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel --once

# Force a replan-from-scratch (clears learned obstacles)
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /local_costmap/clear_entirely_local_costmap  nav2_msgs/srv/ClearEntireCostmap "{}"

# Which params did a server actually load?
ros2 param dump /controller_server

# Where is the Nav2 code this launch resolves to?
ros2 pkg prefix nav2_bringup      # → /opt/ros/humble
```
