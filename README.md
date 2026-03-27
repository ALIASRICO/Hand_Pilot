# Hand Pilot

VR teleoperation system for the Dobot CR20 robot arm using Meta Quest hand tracking.

Control a 6-axis industrial robot with your hands in real time through VR glasses.

---

Sistema de teleoperacion VR para el brazo robotico Dobot CR20 usando Meta Quest hand tracking.

Controla un robot industrial de 6 ejes con tus manos en tiempo real a traves de gafas VR.

## Architecture / Arquitectura

```
Meta Quest VR Glasses
        |
    UDP (Wi-Fi)
        |
hand_tracking_bridge --> /hand_right/pose    --> vr_teleop_node --> /joint_states
                    --> /hand_right/gesture                              |
                                                                   MoveIt2 IK
                                                                   (/compute_ik)
                                                                        |
                                                              Robot (sim or real)
```

**Simulation mode / Modo simulacion**: The teleop node publishes joint states directly; RViz visualizes the robot. / El nodo teleop publica joint states directamente; RViz visualiza el robot.

**Physical robot mode / Modo robot fisico** (future / futuro): The teleop node sends ServoP/ServoJ commands via the Dobot TCP driver. / El nodo teleop envia comandos ServoP/ServoJ via el driver TCP de Dobot.

## Packages / Paquetes

| Package / Paquete | Description / Descripcion |
|---------|-------------|
| `vr_teleop` | Main teleop node: async IK, EMA smoothing, safety limits / Nodo principal de teleop: IK asincrono, suavizado EMA, limites de seguridad |
| `hand_tracking_bridge` | UDP bridge from Meta Quest hand tracking to ROS2 / Bridge UDP de Meta Quest hand tracking a ROS2 |
| `dobot_bringup_v4` | C++ TCP driver for Dobot CR series (ServoP, ServoJ, MovJ, MovL) / Driver TCP en C++ para Dobot CR |
| `dobot_msgs_v4` | Custom ROS2 service/message definitions (120+ services) / Definiciones de servicios/mensajes ROS2 |
| `cr20_moveit` | MoveIt2 configuration for CR20 (IK, FK, joint limits, SRDF) / Configuracion MoveIt2 para CR20 |
| `cra_description` | CR20 URDF, meshes, and xacro files / URDF, mallas y archivos xacro del CR20 |
| `dobot_rviz` | RViz visualization config and robot model / Configuracion de visualizacion RViz y modelo del robot |
| `dobot_moveit` | MoveIt2 action server, joint state bridge, scene manager / Servidor de acciones MoveIt2, bridge de joint states, gestor de escena |

## Requirements / Requisitos

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- MoveIt2
- Meta Quest (2/3/Pro) with / con [hand-tracking-streamer](https://github.com/wengmister/hand-tracking-streamer) app
- Dobot CR20 (for physical robot mode / para modo robot fisico)

## Installation / Instalacion

```bash
# Clone / Clonar
git clone git@github.com:ALIASRICO/Hand_Pilot.git ~/dobot_ws
cd ~/dobot_ws

# Build / Compilar
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source
source install/setup.bash
```

## Usage / Uso

### Simulation / Simulacion (RViz)

**Terminal 1** - Launch simulation / Lanzar simulacion:
```bash
source ~/dobot_ws/install/setup.bash
ros2 launch vr_teleop vr_teleop_sim.launch.py
```

**Terminal 2** - Launch hand tracking bridge / Lanzar bridge de hand tracking:
```bash
source ~/dobot_ws/install/setup.bash
ros2 run hand_tracking_bridge hand_tracking_node
```

### Gestures / Gestos

| Gesture / Gesto | Action / Accion |
|---------|--------|
| GRIP (close fist / cerrar puno) | Start/pause teleoperation / Iniciar/pausar teleoperacion |
| OPEN (open hand / mano abierta) | Move robot / Mover robot (control activo) |
| THUMBS UP (pulgar arriba) | Stop and return to idle / Detener y volver a reposo |
| PINCH (pellizco) | Reserved for future gripper / Reservado para gripper futuro |

### Axis Calibration / Calibracion de ejes

Run the calibration tool to verify axis mapping / Ejecuta la herramienta de calibracion para verificar el mapeo de ejes:
```bash
ros2 run vr_teleop calibrate_axes
```

### Configuration / Configuracion

Edit / Editar `src/vr_teleop/config/teleop_params.yaml`:
- `ema_alpha` - Smoothing / Suavizado (0.0-1.0, higher = more responsive / mayor = mas responsivo)
- `position_scale_*` - Movement sensitivity per axis / Sensibilidad de movimiento por eje
- `max_step_mm` - Maximum step per control cycle / Paso maximo por ciclo de control (speed limit / limite de velocidad)
- `workspace_*` - Workspace boundaries in mm / Limites del espacio de trabajo en mm
- `vr_axis_to_robot_*` / `flip_*` - Axis mapping / Mapeo de ejes

## Credits / Creditos

This project uses / Este proyecto utiliza:
- [hand-tracking-streamer](https://github.com/wengmister/hand-tracking-streamer) - Meta Quest hand tracking UDP streamer

## License / Licencia

Apache License 2.0 - See / Ver [LICENSE](LICENSE).
