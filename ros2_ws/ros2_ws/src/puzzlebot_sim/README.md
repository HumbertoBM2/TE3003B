# puzzlebot_sim

ROS 2 (Humble) package that models the Puzzlebot differential drive robot and
visualises it in RVIZ using a URDF description and tf2 transforms.

This package is the student deliverable for Mini Challenge 1 of the course
TE3003B Integration of Robotics and Intelligent Systems.

---

## Contents

```
puzzlebot_sim/
├── launch/
│   └── puzzlebot_launch.py       Main launch file
├── meshes/
│   ├── Puzzlebot_Jetson_Lidar_Edition_Base.stl
│   ├── Puzzlebot_Wheel.stl
│   └── Puzzlebot_Caster_Wheel.stl
├── puzzlebot_sim/
│   ├── __init__.py
│   └── joint_state_publisher.py  ROS 2 node (odom->base_footprint TF + /joint_states)
├── resource/
│   └── puzzlebot_sim             ament resource index marker
├── rviz/
│   └── puzzlebot_rviz.rviz       Pre-configured RVIZ session
├── test/                         Standard ROS 2 code-quality tests
├── urdf/
│   └── puzzlebot.urdf            Robot description
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

## Architecture

### URDF and the robot model

The robot is described in `urdf/puzzlebot.urdf` as a tree of links connected
by joints.  All links are assigned STL meshes located in `meshes/`.

| Link | Mesh file | Role |
|---|---|---|
| base_footprint | *(none)* | Virtual 2-D pose anchor |
| base_link | Puzzlebot_Jetson_Lidar_Edition_Base.stl | Chassis |
| wheel_r_link | Puzzlebot_Wheel.stl | Right wheel |
| wheel_l_link | Puzzlebot_Wheel.stl | Left wheel |
| caster_link | Puzzlebot_Caster_Wheel.stl | Rear caster ball |

| Joint | Type | Parent | Child | Translation (m) |
|---|---|---|---|---|
| base_link_joint | fixed | base_footprint | base_link | 0, 0, 0.05 |
| wheel_r_joint | continuous | base_link | wheel_r_link | 0.052, -0.095, -0.0025 |
| wheel_l_joint | continuous | base_link | wheel_l_link | 0.052, 0.095, -0.0025 |
| caster_joint | fixed | base_link | caster_link | -0.095, 0, -0.03 |

### TF tree

```
map  (fixed frame in RVIZ)
 └── odom                  [static TF – published by static_transform_publisher in the launch file]
      └── base_footprint   [dynamic TF – published by the joint_state_publisher node]
           └── base_link   [static TF from fixed joint – published by robot_state_publisher]
                ├── wheel_r_link  [dynamic TF – robot_state_publisher reads /joint_states]
                ├── wheel_l_link  [dynamic TF – robot_state_publisher reads /joint_states]
                └── caster_link   [static TF from fixed joint – robot_state_publisher]
```

### Nodes

#### joint_state_publisher (`puzzlebot_sim/joint_state_publisher.py`)

This node has two responsibilities:

1. **odom -> base_footprint transform** – Broadcasts a dynamic transform that
   makes the robot follow a circular path around the origin.  The pose is
   computed at each timer tick from the elapsed time using:

   ```
   x(t)   = ORBIT_RADIUS * cos(ORBIT_OMEGA * t)
   y(t)   = ORBIT_RADIUS * sin(ORBIT_OMEGA * t)
   yaw(t) = ORBIT_OMEGA * t
   ```

2. **`/joint_states` topic** – Publishes a `sensor_msgs/JointState` message
   containing the accumulated angular position of each wheel, computed from
   the forward linear velocity divided by the wheel radius:

   ```
   theta_wheel(t) = (ORBIT_RADIUS * ORBIT_OMEGA / WHEEL_RADIUS) * t
   ```

   `robot_state_publisher` subscribes to this topic and uses the positions to
   compute and broadcast the `base_link -> wheel_r_link` and
   `base_link -> wheel_l_link` TF transforms.

Key constants (editable at the top of `puzzlebot_sim/joint_state_publisher.py`):

| Constant | Default | Effect when changed |
|---|---|---|
| `TIMER_PERIOD` | `0.02` s | Node update rate. Lower = smoother motion but more CPU. |
| `ORBIT_RADIUS` | `0.5` m | Radius of the circular path. Larger = wider orbit. |
| `ORBIT_OMEGA` | `0.4` rad/s | Angular speed. Larger = faster. Linear speed = `ORBIT_RADIUS * ORBIT_OMEGA`. |
| `WHEEL_RADIUS` | `0.05` m | Physical wheel radius used to convert linear speed to wheel spin rate. Must match the real robot geometry. |

After editing the file, rebuild and re-source the workspace before relaunching:

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_sim
source install/setup.bash
ros2 launch puzzlebot_sim puzzlebot_launch.py
```

To change the `map -> odom` offset (the visual separation between the two frames
in RVIZ), edit the `--x` and `--y` arguments of `map_to_odom_tf` in
`launch/puzzlebot_launch.py`. The launch file is a plain Python file, no
compilation step is required — just re-run the launch.

#### robot_state_publisher (from `robot_state_publisher` package)

Reads the `robot_description` parameter (the URDF content), subscribes to
`/joint_states`, and publishes all link-to-link transforms defined in the
URDF to the `/tf` and `/tf_static` topics.

#### static_transform_publisher (from `tf2_ros` package)

Publishes a static transform from `map` to `odom` with a constant offset of
`x=1.0 m, y=1.0 m`.  This makes both frames visually distinct in RVIZ, matching
the diagram in the challenge brief.  In a real robot this offset represents the
accumulated localisation correction; here it is a fixed approximation.

---

## What this node adds relative to the professor's reference template

The professor's skeleton (`Week1/Challenge/puzzlebot_sim/`) provides:

- An empty `puzzlebot.urdf` (only the `<robot>` root tag, no links or joints).
- A `joint_state_publisher.py` template with an empty `timer_cb`, an empty
  `define_TF` method, and a placeholder `DronePublisher` class.
- An empty `LaunchDescription` in `puzzlebot_launch.py`.
- The three STL mesh files.
- A pre-configured `puzzlebot_rviz.rviz` for reference.

This student package completes all of those blanks:

| Component | What was added |
|---|---|
| `urdf/puzzlebot.urdf` | All 5 links (`base_footprint`, `base_link`, `wheel_r_link`, `wheel_l_link`, `caster_link`) with mesh references and material colours; all 4 joints with the exact translations and types from the challenge spec (`fixed`, `continuous`). |
| `puzzlebot_sim/joint_state_publisher.py` | Full `PuzzlebotPublisher` node: dynamic `odom -> base_footprint` TF broadcaster; `/joint_states` publisher for both wheel joints; circular-orbit motion model; yaw-only quaternion computed without any external library (avoids the `transforms3d` / NumPy incompatibility present on this system). |
| `launch/puzzlebot_launch.py` | Complete `LaunchDescription`: static TF node (`map -> odom`), `robot_state_publisher`, the custom node, and `rviz2` with auto-shutdown on RVIZ close. |
| `rviz/puzzlebot_rviz.rviz` | RVIZ configuration adapted to the exact link names used in this URDF (`wheel_r_link`, `wheel_l_link`, `caster_link`) with RobotModel and TF displays enabled. |

---

## Build and installation

This package lives in its own workspace directory (`~/puzzlebot_sim`).
A separate ROS 2 workspace and `colcon` are required.

### 1. Create the workspace (first time only)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s ~/puzzlebot_sim .        # symlink the package into the workspace
```

Alternatively, copy the package directly:

```bash
mkdir -p ~/ros2_ws/src
cp -r ~/puzzlebot_sim ~/ros2_ws/src/
```

### 2. Source ROS 2 and build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_sim
```

### 3. Source the workspace overlay

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Running the simulation

After building and sourcing the workspace:

```bash
ros2 launch puzzlebot_sim puzzlebot_launch.py
```

This single command starts all four nodes listed above.  RVIZ opens
automatically with the pre-configured view.  Close RVIZ to stop the launch.

---

## Expected behaviour in RVIZ

- The `map` frame is fixed at the world origin.  The `odom` frame sits 1 m
  along both X and Y from `map`, making both frames clearly visible as separate
  axes in the TF display.
- The robot model (chassis, wheels, caster ball) orbits the `odom` origin in a
  circle of radius 0.5 m.  In the `map` frame this orbit is centred at (1, 1).
- Both wheels spin forward continuously in sync with the chassis translation.
- The TF panel shows the full transform tree: `map -> odom -> base_footprint ->
  base_link -> {wheel_r_link, wheel_l_link, caster_link}`.
- The `Fixed Frame` is `map`.  It can be changed to `odom` or `base_footprint`
  in the RVIZ Global Options panel at any time without restarting.

---

## Adjusting mesh visual origins

The STL files supplied by Manchester Robotics are not centred at the joint
origin.  If a mesh appears offset or rotated after launching:

1. Open the STL in any CAD tool (FreeCAD, Meshlab, Blender) and measure
   the distance from the geometric centre to the file's world origin.
2. Edit the corresponding `<origin xyz="..." rpy="..."/>` element inside the
   `<visual>` block in `urdf/puzzlebot.urdf`.
3. Rebuild the package with `colcon build --packages-select puzzlebot_sim` and
   source the workspace again.

---

## Dependencies

All dependencies are declared in `package.xml`.  They are satisfied by a
standard ROS 2 Humble desktop installation.

| Dependency | Purpose |
|---|---|
| `rclpy` | Python ROS 2 client library |
| `tf2_ros_py` | TF2 broadcaster for Python |
| `robot_state_publisher` | Converts URDF + JointState to TF broadcasts |
| `sensor_msgs` | `JointState` message type |
| `geometry_msgs` | `TransformStamped` and related types |
| `joint_state_publisher_gui` | Optional GUI sliders to manually set joint positions |

Note: `transforms3d` and `numpy` are listed in `package.xml` as inherited
dependencies but are not used in the node.  The quaternion computation is done
with the standard library `math` module to avoid a known incompatibility between
the system `transforms3d` package and the locally installed NumPy version
(NumPy >= 1.24 removed the `np.float` alias that `transforms3d` relied on).

---

## Course context

This package implements Mini Challenge 1 for the course
TE3003B Integration of Robotics and Intelligent Systems (Tecnologico de
Monterrey, 2026).  The concepts applied are:

- **URDF** – robot description format: links, joints, mesh references.
- **tf2** – coordinate frame transforms: static and dynamic broadcasters.
- **robot_state_publisher** – bridge between URDF and the live TF tree.
- **RVIZ** – visualisation of the robot model and the TF tree.
