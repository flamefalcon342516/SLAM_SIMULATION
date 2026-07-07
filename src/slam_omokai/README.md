# SLAM_OMOKAI — SLAM + Nav2 Simulation Robot

A differential-drive robot (`omokai_bot`) in Gazebo Harmonic that maps its world live
with **slam_toolbox** and navigates it autonomously with **Nav2** (ROS 2 Humble).
No pre-built map is needed: Nav2 plans through the map SLAM is building in real time.

```
ros2 launch slam_omokai bringup.launch.py              # sim + SLAM + Nav2 + RViz
ros2 launch slam_omokai bringup.launch.py rviz:=false
ros2 launch slam_omokai bringup.launch.py headless:=true   # no Gazebo GUI
```

Or run the whole stack in Docker — no ROS install needed on the host
(see `docker/README.md` for details):

```bash
./docker/run.sh build && ./docker/run.sh gui     # or: headless / shell
```

Wait for the **`NAV2 IS ACTIVE`** banner in the terminal (~25 s), then set goals in
RViz with the **Nav2 Goal** tool, or from the CLI:

```bash
# fire-and-forget (same topic the RViz button uses)
ros2 topic pub -w 1 --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"

# with live feedback
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

> The map frame is aligned with the Gazebo world (odometry is world-absolute), so the
> robot starts around map (−5, −5), not (0, 0). Use RViz **Publish Point** +
> `ros2 topic echo /clicked_point --once` to find coordinates of any spot.

## 1. How the whole stack fits together

```
                  gz sim (physics + gpu_lidar)
                       │  ros_gz bridge (/clock /scan /odom /tf /cmd_vel)
                       ▼
   slam_toolbox ──► /map + map→odom TF        OdometryPublisher ──► /odom + odom→base_footprint TF
                       │                                              (ground truth, sim-only shortcut)
                       ▼
  ┌───────────────────────────── Nav2 ─────────────────────────────┐
  │  bt_navigator  ── runs a Behavior Tree that orchestrates:      │
  │     ├── planner_server    (global path through the map)        │
  │     ├── controller_server (follows the path, 20 Hz cmd_vel)    │
  │     ├── behavior_server   (recoveries: spin/backup/wait)       │
  │     ├── smoother_server   (smooths the global path)            │
  │     └── costmaps          (global + local obstacle grids)      │
  │  velocity_smoother ── final accel/vel clamp before /cmd_vel    │
  │  lifecycle_manager ── boots all of the above in order          │
  └─────────────────────────────────────────────────────────────────┘
```

Every Nav2 node is a **lifecycle node**: it goes `unconfigured → inactive → active`.
Goals sent before `bt_navigator` is *active* are silently dropped — that is what the
launch banner is for. Check states with `ros2 lifecycle get /bt_navigator`.

A goal travels: `/goal_pose` topic (or `/navigate_to_pose` action) → bt_navigator →
behavior tree ticks → `ComputePathToPose` (planner) → `FollowPath` (controller) →
`/cmd_vel_nav` → velocity_smoother → `/cmd_vel` → gz DiffDrive plugin → wheels.

---

## 2. Behavior Trees (bt_navigator)

Nav2 does **not** hard-code navigation logic. `bt_navigator` loads a **behavior tree
(BT)** — an XML file of composable nodes — and *ticks* it at `bt_loop_duration` (10 ms
here). The default tree (used by this project) is
`navigate_to_pose_w_replanning_and_recovery.xml` from `nav2_bt_navigator`.

### What the default tree does

```
RecoveryNode "NavigateRecovery" (retries navigation up to 6x)
├── PipelineSequence "NavigateWithReplanning"
│   ├── RateController (1 Hz)                 ← throttles replanning
│   │   └── RecoveryNode "ComputePathToPose"
│   │       ├── ComputePathToPose  (calls planner_server, planner_id="GridBased")
│   │       └── ClearEntireCostmap (global)   ← on planning failure
│   └── RecoveryNode "FollowPath"
│       ├── FollowPath           (calls controller_server)
│       └── ClearEntireCostmap (local)        ← on control failure
└── ReactiveFallback "RecoveryFallback"       ← runs when the pipeline fails
    ├── GoalUpdated                           (abort recoveries if a new goal arrives)
    └── RoundRobin "RecoveryActions"
        ├── ClearEntireCostmap (local + global)
        ├── Spin (90°)
        ├── Wait (5 s)
        └── BackUp (0.3 m)
```

In plain words: **replan once a second while driving; if planning or control fails,
clear costmaps; if navigation as a whole fails, cycle through recoveries (clear →
spin → wait → back up) and try again — up to 6 rounds.**

### BT node categories

| Type | Purpose | Examples (all loaded via `plugin_lib_names` in `nav2_params.yaml`) |
|---|---|---|
| **Action** | Do something (usually call an action server) | `ComputePathToPose`, `FollowPath`, `Spin`, `Wait`, `BackUp`, `ClearCostmapService` |
| **Condition** | Check something, return SUCCESS/FAILURE | `GoalUpdated`, `IsStuck`, `IsPathValid`, `GoalReached`, `IsBatteryLow`, `TransformAvailable` |
| **Control** | Decide which children tick | `PipelineSequence`, `RecoveryNode`, `RoundRobin`, `ReactiveFallback` |
| **Decorator** | Modify one child's ticking | `RateController` (limit frequency), `DistanceController`, `SpeedController`, `GoalUpdater` |

Key control nodes worth knowing:

- **`PipelineSequence`** — like a Sequence, but re-ticks *earlier* children while later
  ones run. This is what makes path-following and replanning run *concurrently*.
- **`RecoveryNode`** — two children: main + recovery. If main fails, tick recovery,
  then retry main (up to `number_of_retries`).
- **`RoundRobin`** — each time it is ticked after a failure it tries the *next* child,
  so successive failures get *different* recoveries.
- **`RateController`** — gates its child to N Hz (here: replanning at 1 Hz).

### Using a custom tree

Point `bt_navigator` at your own XML:

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: /path/to/my_tree.xml
```

You can also pass a per-goal tree in the `NavigateToPose` action's `behavior_tree`
field. Custom C++ BT nodes are plugins added to `plugin_lib_names`.

---

## 3. Planners (planner_server) — the global path

The planner produces a geometric path from the robot to the goal through the **global
costmap**. Which plugin runs is chosen by `planner_id` in the BT (`GridBased` by
default). Multiple planners can be loaded side by side — this project loads two:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased", "NavFn"]
    GridBased:                                  # ← BT default
      plugin: "nav2_smac_planner/SmacPlanner2D"
    NavFn:
      plugin: "nav2_navfn_planner/NavfnPlanner"
```

### The planner options in Nav2

| Plugin | Algorithm | Robot types | Brief |
|---|---|---|---|
| **NavFn** (`nav2_navfn_planner`) | Dijkstra (or A*) on a potential field | Circular, differential/omni | The classic ROS 1 planner. Fast, simple, well-tested. Ignores robot orientation (paths can end in awkward headings). Can fail with *"Failed to create a plan from potential"* on partially-unknown SLAM maps — the reason this project defaults to Smac 2D instead. |
| **SmacPlanner2D** (`nav2_smac_planner`) | A* on a 2D grid, cost-aware | Circular, differential/omni | Modern replacement for NavFn. Supports goal `tolerance`, `allow_unknown` (plan through unmapped space — essential while SLAMing), cost penalties to stay away from obstacles. **Used here.** |
| **SmacPlannerHybrid** (`nav2_smac_planner`) | Hybrid-A* (SE2: x, y, θ) | Car-like / Ackermann, legged | Plans *kinematically feasible* curves respecting a minimum turning radius (Dubins/Reeds-Shepp motion primitives). Use when the robot cannot turn in place. |
| **SmacPlannerLattice** (`nav2_smac_planner`) | State-lattice A* | Any — arbitrary motion models | Like Hybrid but searches over a pre-generated *lattice* of motion primitives for exotic kinematics (diff, omni, Ackermann, custom). Most flexible, needs a lattice file. |
| **ThetaStar** (`nav2_theta_star_planner`) | Theta\*: any-angle A* | Circular, differential/omni | Produces straighter, any-angle paths (not grid-aligned zig-zags) by line-of-sight shortcuts during the search. Nice in open spaces; less cost-aware near clutter. |

Rules of thumb: differential drive that can spin in place → **Smac 2D** (or NavFn);
car-like → **Smac Hybrid**; unusual kinematics → **Smac Lattice**; want straight
diagonal paths in open areas → **Theta\***.

After planning, the **smoother_server** (`nav2_smoother::SimpleSmoother` here) rounds
off the grid path's corners before it is handed to the controller.

---

## 4. Controllers (controller_server) — following the path

The controller turns the global path into actual `cmd_vel` at `controller_frequency`
(20 Hz). Options in Nav2:

| Plugin | Approach | Brief |
|---|---|---|
| **DWB** (`dwb_core::DWBLocalPlanner`) | Dynamic-Window sampling + critics | Samples many candidate (vx, vθ) trajectories, simulates each `sim_time` seconds ahead, scores them with **critics**, picks the best. Extremely configurable. **Used here** — critics: `RotateToGoal`, `Oscillation`, `BaseObstacle`, `GoalAlign`, `PathAlign`, `PathDist`, `GoalDist`. |
| **Regulated Pure Pursuit** (`nav2_regulated_pure_pursuit_controller`) | Geometric path chasing | Chases a lookahead point on the path, slowing for curvature/obstacles. Smooth, predictable, few knobs; great for industrial/AMR-style driving. No dynamic obstacle avoidance of its own. |
| **MPPI** (`nav2_mppi_controller`) | Sampling-based Model-Predictive control | Thousands of GPU/CPU-sampled rollouts each cycle, optimizing a cost function. Excellent dynamic-obstacle behavior; heavier CPU. (Successor to TEB-style local planning in newer Nav2 releases.) |
| **Rotation Shim** (`nav2_rotation_shim_controller`) | Wrapper | First rotates the robot roughly toward the path heading, then delegates to a real controller (e.g. RPP). Fixes "drives off sideways at start" behavior. |

The controller also uses pluggable **progress checkers** (`SimpleProgressChecker`:
must move 0.3 m per 20 s here) and **goal checkers** (`SimpleGoalChecker`: within
0.25 m / 0.25 rad here) to decide "am I stuck?" and "am I done?".

Final output is clamped by the **velocity_smoother** (max vel `[0.5, 0.0, 1.0]`,
accel `[1.5, 0.0, 2.0]`) so the robot never gets commands it cannot follow.

---

## 5. Costmaps — what the planners/controllers see

Two rolling occupancy grids with pluggable **layers**, both fed by `/scan`:

| | global_costmap | local_costmap |
|---|---|---|
| Frame | `map` (from SLAM) | `odom` |
| Size | whole known map | 5×5 m rolling window |
| Used by | planner | controller |
| Layers | static (the SLAM map) + obstacle + inflation | obstacle + inflation |

- **StaticLayer** — mirrors `/map` from slam_toolbox (updates live while mapping).
- **ObstacleLayer** — marks lidar hits, raytrace-clears free space (`raytrace_max_range` 12 m).
- **InflationLayer** — pads every obstacle by an exponential cost slope
  (`inflation_radius` 0.55 m, robot radius 0.35 m) so paths keep a safe margin.
- (Also available in Nav2: VoxelLayer for 3D sensors, RangeLayer for sonars,
  DenoiseLayer for speckle removal.)

**Common failure — `Starting point in lethal space`**: the robot is (or thinks it is)
inside an obstacle, often after nudging a movable object. Fix:

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
# and if physically wedged, nudge it out:
ros2 topic pub -r 10 --times 30 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.2}}"
```

---

## 6. Behavior server — recoveries

Loaded behaviors (each is an action server the BT can call): **Spin** (rotate in
place), **BackUp** (reverse a distance), **DriveOnHeading** (forward a distance),
**Wait**. They check the local costmap ahead of the motion and abort with
*"Collision Ahead"* rather than hit something. Rotation limits here:
`max_rotational_vel` 1.0 rad/s, `min_rotational_vel` 0.3 rad/s.

---

## 7. SLAM setup (slam_toolbox)

`online_async` mode: builds `/map` live and publishes the `map→odom` TF correction
(mapping *and* localization; no AMCL, no map_server). Key params in
`config/slam_params.yaml`: scan matching on, loop closure on, 5 cm resolution,
12 m laser range, new scan processed every 0.2 m / 0.2 rad of travel.

To keep a finished map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/SLAM_OMOKAI/omokai_map
```

(Then a "localization-only" mode could run map_server + AMCL instead of slam_toolbox.)

---

## 8. The robot model (learned the hard way)

- **Drivetrain**: two driven wheels on the center axle + front/rear frictionless
  caster spheres (true differential drive). The original 4-fixed-wheel skid-steer
  *could not rotate*: all four wheels scrub sideways in a pivot, and above
  ~0.5 rad/s the contacts chatter — measured **0 %** of commanded rotation
  regardless of friction settings. With casters the robot tracks commanded rotation
  (~112 %) up to at least 1.2 rad/s.
- **Odometry**: `/odom` + `odom→base_footprint` come from the gz
  **OdometryPublisher** (ground truth, world-absolute — a sim-only shortcut).
  Wheel-encoder odometry from DiffDrive is still published on `/wheel_odom` for
  comparison. Ground truth was adopted because the skid-steer encoder odometry lied
  so badly (2–14× rotation error) that it shredded the SLAM map into rotated ghost
  walls. With the new drivetrain, `/wheel_odom` should be usable again if realistic
  odometry is ever wanted.
- **Lidar**: 360° / 360-sample `gpu_lidar`, 0.25–12 m, 10 Hz, σ = 5 mm noise.
- **`GZ_IP=127.0.0.1`** is set by `sim.launch.py`: with multiple network interfaces
  gz-transport can bind to `docker0` (often DOWN) and the sim↔bridge data channels
  silently die mid-run — `/clock`, `/scan`, `/odom` freeze and Nav2 stops responding
  while every process still looks alive.

---

## 9. File map

```
slam_omokai/
├── launch/
│   ├── bringup.launch.py   # everything: sim + SLAM + Nav2 + RViz (+ ready banner)
│   ├── sim.launch.py       # gz sim + robot spawn + ros_gz bridge + GZ_IP pin
│   ├── slam.launch.py      # slam_toolbox online_async
│   └── nav2.launch.py      # nav2_bringup navigation_launch.py with our params
├── config/
│   ├── nav2_params.yaml    # BT nodes, planner(s), DWB, costmaps, behaviors, smoother
│   └── slam_params.yaml    # slam_toolbox
├── urdf/
│   ├── omokai_bot.xacro    # chassis, 2 drive wheels, 2 casters, lidar
│   └── ign_plugins.xacro   # DiffDrive (actuation), OdometryPublisher, joint states
├── worlds/omokai_world.sdf # walls + movable boxes/cylinders (bumping them leaves
│                           #   stale ghosts in the map until re-scanned!)
└── rviz/slam_nav.rviz      # map + costmaps + Nav2 Goal tool preconfigured
```

## 10. Quick troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Nav2 Goal click does nothing | Nav2 not active yet — wait for the banner. A registered goal always logs `Begin navigating…` in the terminal. |
| Robot frozen, `/clock` silent | gz-transport interface issue (see §8) — should be fixed by `GZ_IP`; restart the launch if it ever recurs. |
| `Starting point in lethal space` | Clear costmaps + nudge robot (see §5). |
| `no valid path found` | Goal inside a wall/obstacle or its 0.55 m inflation ring — pick open space. |
| Map has ghost obstacle | A movable object was pushed; drive past the area with clear line of sight and SLAM will erase it. |
| Robot spins slowly / weakly | Check all three rotation caps agree (§4: DWB `max_vel_theta`, behavior server, velocity_smoother). |
