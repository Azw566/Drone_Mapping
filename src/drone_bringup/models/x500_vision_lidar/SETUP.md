# x500_vision_lidar — Setup Guide

## What This Is

A custom PX4 drone model that extends the official `x500` with:
- **VLP-16 3D LiDAR** (on top, for LIO-SAM SLAM)
- **RGB Camera** (front-facing, angled 15° down, for ArUco detection)
- **Odometry Publisher** (ground truth for debugging)

The base drone (motors, IMU, airframe, PX4 integration) is 100% official PX4 code
via `<include merge="true"><uri>x500</uri></include>`.

## Files

```
x500_vision_lidar/
├── model.config          # Gazebo model database entry
├── model.sdf             # Gazebo model (physics + sensors)
└── model.urdf.xacro      # ROS2 TF tree (copy to your ROS2 workspace)
```

## Installation

### Option A: PX4 Source Tree (recommended — works with `make` targets)

```bash
# Copy the model folder into PX4's model directory
cp -r x500_vision_lidar ~/PX4-Autopilot/Tools/simulation/gz/models/

# You also need to create an airframe config so `make px4_sitl gz_x500_vision_lidar` works.
# The easiest way: symlink the x500_vision airframe (same flight controller config):
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
cp 4002_gz_x500_vision 4020_gz_x500_vision_lidar

# Edit 4020_gz_x500_vision_lidar — change the model name:
#   PX4_SIM_MODEL=${PX4_SIM_MODEL:=gz_x500_vision_lidar}
# Keep everything else the same (it's the same airframe, just different sensors).

# Register the new airframe ID in CMakeLists.txt:
# In ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
# Add a line: 4020_gz_x500_vision_lidar

# Rebuild PX4:
cd ~/PX4-Autopilot
make px4_sitl gz_x500_vision_lidar
```

### Option B: Standalone Mode (simpler — no PX4 rebuild)

```bash
# Copy to the Gazebo model cache
cp -r x500_vision_lidar ~/.simulation-gazebo/models/

# Start Gazebo with your world (your existing maze_gazebo or our warehouse):
gz sim -r your_world.sdf

# In another terminal, start PX4 and tell it to attach to the model by name:
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL=x500_vision_lidar \
  PX4_GZ_MODEL_POSE="-7,0,0.5,0,0,0" \
  ./build/px4_sitl_default/bin/px4 -i 0
```

### Option C: Spawn via gz service (from your ROS2 launch file)

```bash
# If Gazebo is already running with your world:
gz service -s /world/<your_world_name>/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "/path/to/x500_vision_lidar/model.sdf", name: "drone_1", pose: {position: {x: -7, y: 0, z: 0.5}}'
```

## Running Two Drones

```bash
# Terminal 1: Start Gazebo with your world
gz sim -r your_maze_world.sdf

# Terminal 2: PX4 instance 0 (drone 1)
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL=x500_vision_lidar \
  PX4_GZ_MODEL_POSE="-7,0,0.5,0,0,0" \
  ./build/px4_sitl_default/bin/px4 -i 0

# Terminal 3: PX4 instance 1 (drone 2)
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL=x500_vision_lidar \
  PX4_GZ_MODEL_POSE="-7,2,0.5,0,0,0" \
  ./build/px4_sitl_default/bin/px4 -i 1

# Terminal 4: MicroXRCE-DDS Agent (bridges PX4 ↔ ROS2)
MicroXRCEAgent udp4 -p 8888
```

## Verifying Sensors

After spawning, check that Gazebo publishes sensor data:

```bash
# List all Gazebo topics (look for lidar and camera):
gz topic -l

# You should see topics like:
#   /world/<world>/model/x500_vision_lidar_0/link/lidar_link/sensor/vlp16/scan/points
#   /world/<world>/model/x500_vision_lidar_0/link/camera_link/sensor/rgb_camera/image
#   /world/<world>/model/x500_vision_lidar_0/link/camera_link/sensor/rgb_camera/camera_info

# Echo LiDAR to confirm data flows:
gz topic -e -t /world/<world>/model/x500_vision_lidar_0/link/lidar_link/sensor/vlp16/scan/points

# NOTE: The exact topic paths depend on the model name Gazebo assigns.
# If you spawned with name "drone_1", replace x500_vision_lidar_0 with drone_1.
```

## ROS2 Bridge

Update your ros_gz_bridge to match the Gazebo topic names. The sensor topic
names in Gazebo follow this pattern:

```
/world/<WORLD_NAME>/model/<MODEL_NAME>/link/<LINK_NAME>/sensor/<SENSOR_NAME>/<DATA>
```

For this model:
- LiDAR points: `.../link/lidar_link/sensor/vlp16/scan/points`
- Camera image: `.../link/camera_link/sensor/rgb_camera/image`
- Camera info:  `.../link/camera_link/sensor/rgb_camera/camera_info`

The IMU comes from PX4 via MicroXRCE-DDS (not from Gazebo), so you get it on
the `/fmu/out/sensor_combined` topic automatically.

## Matching LIO-SAM Extrinsic Config

The LiDAR is at (0, 0, 0.12) relative to base_link.
The IMU is at (0, 0, 0) relative to base_link.
Therefore LiDAR→IMU transform = (0, 0, -0.12), no rotation.

In `lio_sam_params.yaml`:
```yaml
extrinsicTrans: [0.0, 0.0, -0.12]
extrinsicRot: [1, 0, 0,
               0, 1, 0,
               0, 0, 1]
```
