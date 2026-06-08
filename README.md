# Autonomous Scale Forklift System — Team 7

**TE3003B | Instituto Tecnológico y de Estudios Superiores de Monterrey | Electric 80 Group | Manchester Robotics | 2026**

An end-to-end autonomous forklift system built on the Puzzlebot platform. The robot navigates a scaled 3.76 m × 4.86 m warehouse track, locates pallets using computer vision, executes precision docking maneuvers via 3D QR pose estimation, and completes two fully autonomous missions — including real-time Odoo ERP integration for delivery tracking. Every algorithm (SLAM, localization, path planning, obstacle avoidance, PnP docking, and voice recognition) was implemented from scratch without relying on external navigation libraries such as Nav2, gmapping, or AMCL.

---


## System Capabilities

| Capability | Implementation |
|---|---|
| Simultaneous Localization and Mapping | Custom occupancy grid SLAM, ArUco global correction, scan matching |
| Autonomous Navigation | A* path planning, Pure Pursuit control, Bug2 wall-following |
| Pallet Detection | YOLOv8 (brand logos + pallet detection), pyzbar QR decoding |
| Precision Docking | solvePnP IPPE\_SQUARE, 4-metric control law, REPOS maneuver |
| Monte Carlo Localization | 100-particle filter with circular-mean yaw, distance-field scoring |
| Obstacle Avoidance | Bug2 (M-line leave condition, dynamic side selection) |
| Lift Actuation | FPGA PWM via Jetson GPIO, 2.72 cm/s symmetric motor |
| Voice Control | HMM pipeline, 10 commands, MFCC + VQ codebook, Viterbi decoding |
| ERP Integration | Odoo XML-RPC (create + validate delivery orders asynchronously) |
| Remote Monitoring | Flask + Socket.IO web dashboard, 4 specialized tabs |

---

## Hardware Platform

| Component | Specification | Role |
|---|---|---|
| Onboard computer | NVIDIA Jetson Nano 2 GB | ROS2 node execution, GPIO |
| Low-level controller | ESP32 Hackerboard (CP2102N USB) | Motor PWM, encoder readout |
| FPGA | Tang Nano 20K (GoWin IDE) | Lift motor PWM state machine |
| LiDAR | RPLIDAR A2M8 — 720 beams/rev @ 10 Hz | SLAM, obstacle detection |
| Camera | Raspberry Pi CSI IMX219 — 640×480 @ 15 fps | ArUco, QR, YOLO |
| Drive | Two DC motors with encoders, differential drive | Locomotion |
| Lift actuator | DC motor via FPGA GPIO | Pallet lifting (5 cm/1.84 s) |
| Power | 20,000 mAh power bank | All onboard electronics |
| Remote PC | PC Master, Ubuntu 20.04, ROS2 Humble | Dashboard, YOLO, voice |

### Track and Marker Layout

The warehouse track measures 3.76 m (X) × 4.86 m (Y). The origin (0, 0) is placed at the south-west corner. Five ArUco markers (DICT\_4X4\_50, 10 cm side, IDs 0–4) are mounted on the track walls at known world positions:

| Marker ID | World Position (x, y) m | Facing | Notes |
|---|---|---|---|
| 0 | (0.00, 3.90) | East (yaw = +π/2) | West wall, upper |
| 1 | (1.88, 4.86) | South (yaw = 0) | North wall, centre — excluded from SLAM correction |
| 2 | (3.76, 3.90) | West (yaw = −π/2) | East wall, upper |
| 3 | (3.76, 1.04) | West (yaw = −π/2) | East wall, lower |
| 4 | (0.00, 1.04) | East (yaw = +π/2) | West wall, lower |

---

## Software Architecture

All nodes run under ROS2 Humble on the Jetson Nano. A separate PC Master handles the web dashboard, YOLO inference, and voice recognition.



### Key ROS2 Topics

| Topic | Message Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Dead-reckoning odometry with ZUPT |
| `/aruco/pose` | `geometry_msgs/PoseStamped` | solvePnP absolute pose in map frame |
| `/scan_stamped` | `sensor_msgs/LaserScan` | LiDAR scan with π-offset correction |
| `/map` | `nav_msgs/OccupancyGrid` | 86×108 occupancy grid at 5 cm/cell |
| `/qr/data` | custom `QRData` | bearing, psi, e\_lat, dist\_pnp, QR content |
| `/mission/waypoint_1` | `geometry_msgs/PointStamped` | Operator goal from dashboard |
| `/mission/go` | `std_msgs/Bool` | Voice authorization flag |
| `/mission/odoo_status` | `std_msgs/String` | ERP delivery state string |
| `/lift/command` | `std_msgs/Int8` | UP=1, STOP=0, DOWN=2 |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command to base |
| `/voice/listen_flag` | `std_msgs/Bool` | Trigger for 2-second recording |

---

## Algorithms

### 1. Simultaneous Localization and Mapping (SLAM)

A custom occupancy grid SLAM pipeline produces an 86 × 108 cell map at 5 cm/cell resolution (3.76 × 4.86 m). No external SLAM library is used. The pipeline consists of three fused components:

**Dead-reckoning odometry** integrates differential-drive kinematics from encoder ticks, applying ZUPT to suppress spurious motion when both wheels are stationary.

**Scan-to-map matching** scores candidate robot poses by counting LiDAR beam endpoints landing on already-occupied cells:

$$S(\mathbf{x}) = \sum_{i=1}^{N} \mathbf{1}\!\left[m\!\left(p_i(\mathbf{x})\right) = \text{occupied}\right]$$

**ArUco global correction** fuses marker observations into the `map → odom` transform using an exponential moving average with adaptive blending rate:

$$\mathbf{p}_\text{new} = \mathbf{p}_\text{prev} + \alpha \cdot (\mathbf{p}_\text{aruco} - \mathbf{p}_\text{prev})$$

where α = 0.25 for corrections < 0.8 m and decreases to 0.04 for corrections up to 7 m.

The LiDAR driver (RPLIDAR A2M8) publishes angle = 0 pointing rearward due to cable routing. A dedicated `scan_restamper` node corrects this before integration:

$$\theta'_i = (\theta_i + \pi) \bmod 2\pi$$

Rotation suppression discards scans during fast in-place turns (|ω| > 0.25 rad/s) to prevent fan-shaped map artefacts.

### 2. Monte Carlo Localization (MCL)

A 100-particle filter enables real-time pose estimation on a pre-built saved map. Particles are propagated using the differential-drive motion model with Gaussian noise, scored against the map distance field, and resampled proportionally. Heading is estimated using the **weighted circular mean** to avoid the ±π instability of argmax after uniform resampling:

$$\hat{\theta} = \text{atan2}\!\left(\sum_k w^{(k)} \sin\theta^{(k)},\;\sum_k w^{(k)} \cos\theta^{(k)}\right)$$

A zero-pose guard rejects the default (0, 0, 0) transform published before the first ArUco observation. A divergence detector triggers full re-initialization at the ArUco pose if the MCL estimate drifts more than 1.5 m from a marker observation.

### 3. A\* Path Planning and Pure Pursuit

Path planning runs on the occupancy grid with all occupied cells inflated by **r = 0.27 m** (5 cells) — accounting for the robot chassis and the 12 cm fork extension. A collinearity filter reduces raw paths from ~15 waypoints to 3–6 clean segments.

The Pure Pursuit controller tracks waypoints with lookahead distance L_d = 0.45 m:

$$\kappa = \frac{2\sin\alpha}{L_d}, \quad \alpha = \text{atan2}(y_l - y_r,\; x_l - x_r) - \theta_r$$

Linear velocity is clamped between 0.04 and 0.07 m/s and reduced proportionally to cos α during corrective turns. For heading errors > 90°, the robot performs a true in-place rotation with a cached target heading to prevent the circular-motion bug.

### 4. Bug2 Reactive Obstacle Avoidance

A reactive layer handles dynamic obstacles absent from the saved map. A forward detection cone of ±45° at D\_obs = 0.40 m triggers wall-following. The side of following is chosen dynamically based on available clearance at the moment of detection.

**Bug2 leave condition** — the robot exits wall-following when it crosses the start→goal M-line closer to the goal than the initial hit point H:

$$d(p,\;\text{M-line}) < 0.20\,\text{m} \quad \text{AND} \quad \|p - G\| < \|H - G\| - 0.25\,\text{m}$$

This is an improvement over Bug1 (full circumnavigation required) for obstacles with a clear passage on one side. A 30-second timeout with increased grid inflation (30 cm radius) handles the degenerate case.

### 5. PnP-Based QR Alignment (ALIGN\_QR)

Precision docking uses `cv2.solvePnP` with the `SOLVEPNP_IPPE_SQUARE` solver on the four corners of a 9 cm QR code, yielding four simultaneous control metrics:

| Metric | Definition | Tolerance |
|---|---|---|
| bearing (φ\_b) | Horizontal angle to QR centroid | < 5° |
| psi (ψ) | QR face perpendicularity angle | < 5° |
| e\_lat | Lateral offset on face normal (m) | < 0.03 m |
| dist\_pnp (d) | Euclidean distance to QR (m) | 0.40 m ± 0.04 |

The multi-metric angular control law:

$$\omega = \text{clip}\!\left(-k_b\,\phi_b - k_\psi\,\psi - k_e\,e_l,\;-\omega_{\max},\;\omega_{\max}\right)$$

Linear velocity uses a slowdown factor proportional to heading error:

$$v = \text{clip}\!\left(k_d(d - d_\text{target})\cdot s_f,\;-v_\text{rev},\;v_\text{max}\right), \quad s_f = \max\!\left(0.2,\;1 - \frac{|\phi_b|}{25°}\right)$$

The robot is declared **aligned** when all four conditions hold for N\_stable = 5 consecutive frames (~250 ms at 20 Hz).

The state machine has six phases: **SCAN → TRACK → REVERSE → REPOS\_ROT1 → REPOS\_DRIVE → REPOS\_ROT2**, with a 90-second global abort timeout.

Calibrated gains: k\_b = 1.2, k\_ψ = 0.5, k\_e = 0.7, k\_d = 0.5, ω\_max = 0.12 rad/s, v\_max = 0.10 m/s.

### 6. Intelligent Repositioning (REPOS)

When the robot arrives laterally displaced from the QR face normal, the single-point controller deadlocks because bearing and lateral-error corrections produce opposing angular commands. The REPOS maneuver triggers automatically when ψ > 15° persists for 12 s without a stable frame.

The approach point on the QR face normal:

$$p_\text{AP} = p_\text{QR} - (d_\text{target} + d_\text{extra}) \cdot [\cos\psi_\text{QR},\;\sin\psi_\text{QR}]^T$$

where p\_QR is the world-frame QR position from the last valid PnP frame, d\_target = 0.40 m, d\_extra = 0.20 m.

The maneuver sequence: **REPOS\_ROT1** (face AP) → **REPOS\_DRIVE** (navigate to AP at 0.06 m/s) → **REPOS\_ROT2** (align to QR yaw) → **SCAN** (re-acquire from frontal position). If the QR becomes visible at any point, the maneuver aborts and TRACK resumes immediately.

### 7. YOLOv11 Pallet and Logo Detection

A YOLOv8 model fine-tuned on the track environment runs on the Jetson Nano's Maxwell GPU (CUDA 10.2) at 5 Hz. The model detects three truck brand logos (POPSI, WOLMAR, EMEZON) and identifies pallets at distances up to 1.5 m at 320×240 resolution. Detected bounding box centers are back-projected through the calibrated camera model to generate 3D world-frame goals.

Camera intrinsic matrix (640×480, rational polynomial, RMS = 0.41 px):

$$K = \begin{pmatrix} 640.6 & 0 & 327.2 \\ 0 & 855.9 & 223.1 \\ 0 & 0 & 1 \end{pmatrix}$$

Camera extrinsics: 9 cm forward, 7 cm height from the wheel axis centre.

### 8. HMM Voice Recognition

The voice pipeline runs on the PC Master (USB microphone) and sends commands over the ROS2 network. Audio is captured in 2-second windows at 16 kHz, analysed with 13-coefficient MFCC features per 25 ms frame, and quantized against a 32-centroid VQ codebook. Classification uses per-word left-to-right 5-state HMMs with Viterbi decoding. A margin threshold Δ = 12.0 between top-two log-likelihoods prevents spurious commands.

| Command | Action |
|---|---|
| **empieza** | Publish `/mission/go` (authorize mission start) |
| avanza | Forward 0.8 s |
| retrocede | Reverse 0.8 s |
| derecha | Turn right 0.5 s |
| izquierda | Turn left 0.5 s |
| alto | Emergency stop |
| sube | Lift UP 1.5 s |
| baja | Lift DOWN 1.5 s |
| gira | Rotate 1.5 s |
| busca | Slow scan rotation 2.0 s |

### 9. Odoo ERP Integration

`odoo_client.py` is a lightweight XML-RPC client using only Python's `xmlrpc.client` stdlib module. It operates asynchronously in background threads to avoid blocking the mission timer.

- **On ALIGN\_QR complete (M2):** `create_delivery_async(qr_dest)` — creates a stock picking (delivery order) in Odoo linking the detected truck brand to the corresponding customer. Publishes `CREATED:{id}` to `/mission/odoo_status`.
- **On DEPOSIT complete (M2):** `validate_delivery_async(picking_id)` — marks the picking as `done`. Publishes `DONE:{id}`.

---

## Autonomous Missions

### Mission 1 — Conveyor Belt to Rack

```
IDLE → WAIT_WP1 → NAV_PICKUP → ALIGN_QR → LIFT → ADVANCE(22 cm) → NAV_DEST → APPROACH_DROP → DEPOSIT → RETURN
```

1. Operator selects Mission 1 in the dashboard and clicks the pickup waypoint on the map.
2. Robot lifts forks from 4 cm to 7 cm **during** NAV\_PICKUP navigation (no stop needed on arrival).
3. At the conveyor belt: ALIGN\_QR with PnP control — robot approaches to 0.40 m, achieves 5 stable frames on all four metrics.
4. Fork raises from 7 → 10 cm (1.10 s, engaging pallet). Robot advances 22 cm (fork tips at 0.18 m from QR).
5. Robot navigates to the rack waypoint (A\* + Pure Pursuit, Bug2 active).
6. APPROACH\_DROP: robot aligns to rack QR. DEPOSIT: fork lowers 6 cm in 2.21 s.
7. Robot returns to home position.

### Mission 2 — Rack to Truck with ERP Registration

```
IDLE → WAIT_WP1+go → NAV_PICKUP → SCAN(8 s) → ALIGN_QR → ADVANCE(9 cm) → LIFT → NAV_DEST → APPROACH_DROP → DEPOSIT → RETURN
```

1. Operator selects Mission 2, clicks rack waypoint on the dashboard map.
2. Operator says **"empieza"** → `/mission/go` published → robot begins NAV\_PICKUP.
3. At the rack: robot rotates for 8 seconds accumulating QR candidates. Selects the truck with minimum PnP distance.
4. ALIGN\_QR to selected pallet QR → on completion: Odoo delivery order created asynchronously.
5. Robot advances 9 cm (forks slide under pallet), then lifts to 7 cm (1.10 s).
6. Robot navigates to the selected truck (identified by YOLO logo detection + QR destination).
7. APPROACH\_DROP → DEPOSIT: forks lower 5 cm to ~2 cm above ground (1.84 s).
8. Odoo delivery validated → dashboard shows "Entregado ✓ #N" with green indicator.
9. Robot returns to home position.

---

## Lift Mechanism

The lift is actuated by a DC motor controlled by a Verilog state machine on the Tang Nano 20K FPGA. The Jetson drives two GPIO pins; the FPGA decodes the 2-bit command to a PWM duty cycle:

| GPIO Code | Command | PWM |
|---|---|---|
| `00` | STOP (hold) | 50% duty |
| `01` | UP | High duty |
| `10` | DOWN | Low duty |

Calibrated constants for the servo at 2.72 cm/s:

| Constant | Duration | Travel |
|---|---|---|
| LIFT\_PRE\_SECS (4→7 cm, during NAV\_PICKUP) | 1.10 s | 3 cm |
| LIFT\_PICK\_SECS (7→10 cm, M1 engage) | 1.10 s | 3 cm |
| LIFT\_DOWN\_M1 (10→4 cm, rack deposit) | 2.21 s | 6 cm |
| LIFT\_DOWN\_M2 (7→2 cm, truck deposit) | 1.84 s | 5 cm |

End-to-end command latency (dashboard → ROS2 → GPIO → FPGA → motor): ~120 ms.

---

## Web Dashboard

A Python Flask + Socket.IO application running on the PC Master provides real-time supervision and control at `http://localhost:5000`.

| Tab | Content |
|---|---|
| SLAM & Lift | Live camera (optional YOLO overlay), LiDAR scan, occupancy map with robot trajectory, lift UP/STOP/DOWN buttons, telemetry |
| A\* Navigation | Interactive map with click-to-goal, A\* path overlay as dashed line, navigation state sidebar |
| Voice Control | Animated record button, recognized word display, log-likelihood bar chart for all 10 commands, scrollable history |
| E80 Mission | Mission mode selector (M1/M2), waypoint selection on map, mission state display, Odoo ERP status card |

---

## Odometry Calibration

The effective wheel separation was calibrated using a bisection search on the in-place rotation angle. The nominal value L = 0.18 m consistently overestimated heading changes. After three bisection iterations, the calibrated value is **L = 0.174 m**, reducing heading drift from ~7°/m to under 2°/m.

---





## Running the System

### Jetson Nano

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch puzzlebot_ros e80_mission.launch.py
```

### PC Master — Dashboard

```bash
cd ~/ros2_ws/src/puzzlebot_ros/web_dashboard
python3 dashboard.py
# Open http://localhost:5000
```

### PC Master — Voice Recognition

```bash
# Terminal 1: HMM recognizer
source ~/reto_voz/install/setup.bash
ros2 launch voice_hmm launch_voice_hmm.py

# Terminal 2: voice → ROS2 bridge
python3 ~/ros2_ws/src/puzzlebot_ros/web_dashboard/voice_cmd_test.py
```

### PC Master — YOLO Inference

```bash
python3 yolo_master.py --model ~/Downloads/best.pt --hz 5
```

### Mapping (fresh map)

```bash
# Jetson:
ros2 launch puzzlebot_ros mapping.launch.py

# Teleop:
source ~/ros2_ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map when complete:
ros2 service call /map/save std_srvs/srv/Trigger {}
```

### Useful Commands

```bash
# LiDAR angle calibration (hot):
ros2 param set /scan_restamper angle_offset_rad 0.0
ros2 param set /scan_restamper invert_angles true
ros2 service call /map/clear std_srvs/srv/Trigger {}

# ALIGN_QR debug stream:
ros2 topic echo /rosout 2>/dev/null | grep "DBG ALIGN_QR"

# Monitor Odoo status:
ros2 topic echo /mission/odoo_status
```

---

## Key Parameters

| Parameter | Value | Notes |
|---|---|---|
| Track dimensions | 3.76 × 4.86 m | X × Y |
| Map resolution | 5 cm/cell (86×108 cells) | |
| Wheel radius | 0.05 m | |
| Wheel separation (calibrated) | 0.174 m | Bisection-calibrated |
| ROBOT\_RADIUS (A\* inflation) | 0.27 m | Includes 12 cm forks |
| D\_OBS (Bug2 trigger) | 0.40 m | |
| QR\_READY\_DIST | 0.40 m | pyzbar stable range |
| ADVANCE\_DIST\_M (M1) | 0.22 m | Fork tip to 0.18 m from QR |
| ADVANCE\_DIST\_M2 | 0.09 m | Pre-lift advance for M2 |
| SCAN\_COLLECT\_S | 8.0 s | Multi-candidate QR window |
| ALIGN\_W\_MAX | 0.12 rad/s | Oscillation-reduced |
| QR cooldown | 0.30 s | 3 Hz bearing updates |
| LIFT\_DOWN\_M2 | 1.84 s | Fork to ~2 cm floor clearance |
| V\_MAX navigation | 0.07 m/s | |
| Pure Pursuit lookahead | 0.45 m | |

---

## Team — Team 7

| Name | Matricula | GitHub |
|---|---|---|
| Humberto Barrera | A00836271 | [@HumbertoBM2](https://github.com/HumbertoBM2) |
| Mauricio Zavala | A00837332 | [@mzzzavalas](https://github.com/mzzzavalas) |
| María José Pardo | A01234356 | [@mariajosepardoc18](https://github.com/mariajosepardoc18) |
| Erick Campos | A01247257 | [@Erick-CamposA01247257](https://github.com/Erick-CamposA01247257) |


