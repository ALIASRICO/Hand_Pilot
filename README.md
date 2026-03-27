# Hand Pilot

VR teleoperation system for the Dobot CR20 robot arm using Meta Quest hand tracking.

Control a 6-axis industrial robot with your hands in real time through VR glasses.

## Architecture

```
Meta Quest VR Glasses
        |
    UDP (Wi-Fi)
        |
hand_tracking_bridge ──> /hand_right/pose    ──> vr_teleop_node ──> /joint_states
                    ──> /hand_right/gesture                              |
                                                                   MoveIt2 IK
                                                                   (/compute_ik)
                                                                        |
                                                              Robot (sim or real)
```

**Simulation mode**: The teleop node publishes joint states directly; RViz visualizes the robot.

**Physical robot mode** (future): The teleop node sends ServoP/ServoJ commands via the Dobot TCP driver.

## Packages

| Package | Description |
|---------|-------------|
| `vr_teleop` | Main teleop node: async IK, EMA smoothing, safety limits, axis calibration |
| `hand_tracking_bridge` | UDP bridge from Meta Quest hand tracking to ROS2 topics |
| `dobot_bringup_v4` | C++ TCP driver for Dobot CR series (ServoP, ServoJ, MovJ, MovL) |
| `dobot_msgs_v4` | Custom ROS2 service/message definitions (120+ services) |
| `cr20_moveit` | MoveIt2 configuration for CR20 (IK, FK, joint limits, SRDF) |
| `cra_description` | CR20 URDF, meshes, and xacro files |
| `dobot_rviz` | RViz visualization config and robot model |
| `dobot_moveit` | MoveIt2 action server, joint state bridge, scene manager |

## Requirements

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- MoveIt2
- Meta Quest (2/3/Pro) with [hand-tracking-streamer](https://github.com/wengmister/hand-tracking-streamer) app
- Dobot CR20 (for physical robot mode)

## Installation

```bash
# Clone
git clone git@github.com:ALIASRICO/Hand_Pilot.git ~/dobot_ws
cd ~/dobot_ws

# Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source
source install/setup.bash
```

## Usage

### Simulation (RViz)

**Terminal 1** - Launch simulation:
```bash
source ~/dobot_ws/install/setup.bash
ros2 launch vr_teleop vr_teleop_sim.launch.py
```

**Terminal 2** - Launch hand tracking bridge:
```bash
source ~/dobot_ws/install/setup.bash
ros2 run hand_tracking_bridge hand_tracking_node
```

### Gestures

| Gesture | Action |
|---------|--------|
| GRIP (close fist) | Start/pause teleoperation |
| OPEN (open hand) | Move robot (active control) |
| THUMBS UP | Stop and return to idle |
| PINCH | Reserved for future gripper control |

### Axis Calibration

Run the calibration tool to verify axis mapping:
```bash
ros2 run vr_teleop calibrate_axes
```

### Configuration

Edit `src/vr_teleop/config/teleop_params.yaml` to adjust:
- `ema_alpha` - Smoothing (0.0-1.0, higher = more responsive)
- `position_scale_*` - Movement sensitivity per axis
- `max_step_mm` - Maximum step per control cycle (speed limit)
- `workspace_*` - Workspace boundaries in mm
- `vr_axis_to_robot_*` / `flip_*` - Axis mapping

## Credits

This project builds upon:
- [PALLET_HIT](https://github.com/ALIASRICO/PALLET_HIT) - Dobot CR20 ROS2 driver and MoveIt2 configuration
- [hand-tracking-streamer](https://github.com/wengmister/hand-tracking-streamer) - Meta Quest hand tracking UDP streamer
- [Dobot CR Series SDK](https://www.dobot-robots.com/) - Official robot communication protocol

## License

Apache License 2.0 - See [LICENSE](LICENSE) for details.
