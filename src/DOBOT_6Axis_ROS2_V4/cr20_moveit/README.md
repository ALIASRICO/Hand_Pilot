# cr20_moveit - Configuración MoveIt2 para DOBOT CR20

Paquete de configuración MoveIt2 para el robot colaborativo DOBOT CR20 (6 ejes). Generado con el MoveIt Setup Assistant y adaptado para integrarse con el driver `dobot_moveit`.

## 📦 Estructura del paquete

```
cr20_moveit/
├── config/
│   ├── cr20_robot.srdf              # Grupos de planificación y colisiones
│   ├── cr20_robot.urdf.xacro        # URDF del robot (referencia)
│   ├── cr20_robot.ros2_control.xacro # Interfaces de control hardware
│   ├── kinematics.yaml              # Solver cinemático
│   ├── joint_limits.yaml            # Límites de velocidad y aceleración
│   ├── ros2_controllers.yaml        # Configuración del controlador
│   ├── moveit_controllers.yaml      # Mapeo MoveIt2 -> ros2_controllers
│   ├── ompl_planning.yaml           # Planificadores OMPL disponibles
│   ├── pilz_cartesian_limits.yaml   # Límites para planificador Pilz
│   ├── initial_positions.yaml       # Posición inicial de los joints
│   └── moveit.rviz                  # Configuración de RViz
└── launch/
    ├── real_robot.launch.py         # Launch principal para robot real
    ├── demo.launch.py               # Demo sin robot (simulación MoveIt)
    ├── moveit_gazebo.launch.py      # Integración con Gazebo
    ├── move_group.launch.py         # Solo el servidor MoveIt
    ├── rsp.launch.py                # Robot State Publisher
    ├── moveit_rviz.launch.py        # RViz con plugin MoveIt
    ├── spawn_controllers.launch.py  # Spawn de controladores ros2_control
    └── warehouse_db.launch.py       # Base de datos de trayectorias
```

## ⚙️ Archivos de Configuración

| Archivo                           | Descripción                                                                                                                                            |
| --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `cr20_robot.srdf`               | Define el grupo `cr20_group` (cadena `base_link` -> `Link6`), la posición named `home`, y las colisiones deshabilitadas entre links adyacentes |
| `kinematics.yaml`               | Solver KDL (`kdl_kinematics_plugin/KDLKinematicsPlugin`), timeout de 5 ms, resolución de búsqueda 0.005                                             |
| `joint_limits.yaml`             | Velocidad máxima 150 deg/s (joints 1-3) y 180 deg/s (joints 4-6); scaling por defecto al 10%                                                           |
| `ros2_controllers.yaml`         | `JointTrajectoryController` para los 6 joints con interfaz `position`; update rate 100 Hz                                                           |
| `moveit_controllers.yaml`       | Mapea el `cr20_group_controller` de ros2_control al action server de MoveIt2                                                                          |
| `cr20_robot.ros2_control.xacro` | Declara las interfaces `position` (command) y `position`/`velocity` (state) para cada joint                                                       |

## 🎯 Grupos de Planificación

### `cr20_group`

Cadena cinemática completa del brazo:

| Joint      | Descripción                      |
| ---------- | --------------------------------- |
| `joint1` | Rotación base (yaw)              |
| `joint2` | Hombro                            |
| `joint3` | Codo                              |
| `joint4` | Muñeca 1                         |
| `joint5` | Muñeca 2                         |
| `joint6` | Muñeca 3 (rotación herramienta) |

**Posición named `home`:**

```
joint1=0.0  joint2=0.798  joint3=-1.907
joint4=-0.104  joint5=1.631  joint6=0.0
```

## 🚀 Launch Files

| Archivo                     | Qué hace                                                                                          |
| --------------------------- | -------------------------------------------------------------------------------------------------- |
| `real_robot.launch.py`    | Lanza RSP + move_group + RViz +`scene_manager` + `action_move_server` (puente MoveIt -> Dobot) |
| `demo.launch.py`          | Demo completo sin robot real: move_group + RViz con fake controllers                               |
| `moveit_gazebo.launch.py` | Integración con Gazebo: spawn del robot + controllers + move_group                                |
| `move_group.launch.py`    | Solo el nodo `move_group` (servidor MoveIt2)                                                     |
| `rsp.launch.py`           | `robot_state_publisher` con el URDF del CR20                                                     |

## 🔧 Dependencias

```xml
<depend>moveit_ros_planning_interface</depend>
<depend>moveit_configs_utils</depend>
<depend>moveit_ros_move_group</depend>
<depend>ros2_controllers</depend>
<depend>joint_trajectory_controller</depend>
<depend>joint_state_broadcaster</depend>
```

## 💡 Cómo ejecutar

### Robot real

```bash
# Compilar el workspace
cd ~/dobot_ws
colcon build --packages-select cr20_moveit
source install/setup.bash

# Configurar variables de entorno
export DOBOT_TYPE=cr20
export IP_address=192.168.5.1   # IP del robot (ajustar si es diferente)

# Lanzar
ros2 launch cr20_moveit real_robot.launch.py
```

Esto arranca: `robot_state_publisher`, `move_group`, RViz, el gestor de escena de colisión y el servidor de acciones que traduce trayectorias MoveIt a comandos Dobot.

### Demo sin robot (simulación)

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch cr20_moveit demo.launch.py
```

Útil para probar planificación y visualizar trayectorias sin conectar hardware.

### Solo move_group

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch cr20_moveit move_group.launch.py
```

## 🐛 Troubleshooting

### La planificación falla con "No motion plan found"

El solver KDL tiene un timeout muy ajustado (5 ms). Si el objetivo está cerca de singularidades o fuera del espacio de trabajo, aumenta el timeout en `kinematics.yaml`:

```yaml
cr20_group:
  kinematics_solver_timeout: 0.05   # 50 ms
```

### IK no converge

KDL es un solver numérico iterativo. Para poses con muñeca en configuración singular (joint5 ~ 0), considera cambiar a `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` en `kinematics.yaml`.

### El controlador no acepta trayectorias

Verifica que el robot real está conectado y el driver está activo antes de lanzar. El `action_move_server` necesita comunicación con el robot en `IP_address`.

```bash
# Verificar que el controlador está activo
ros2 control list_controllers

# Debe aparecer:
# cr20_group_controller  [active]
# joint_state_broadcaster [active]
```

### RViz no muestra el robot

```bash
# Verificar que robot_state_publisher publica el TF
ros2 topic echo /robot_description --once
ros2 topic hz /joint_states
```
