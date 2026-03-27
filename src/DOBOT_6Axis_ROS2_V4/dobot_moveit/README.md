# dobot_moveit

Paquete principal del sistema despaletizador para el robot **Dobot CR20**. Coordina la cadena completa: detección YOLO → planificación MoveIt2 → ejecución en robot físico → control de ventosa de vacío (DO1).

## 📦 Estructura

```
dobot_moveit/
├── dobot_moveit/
│   ├── depalletizer.py          # Nodo principal del despaletizador
│   ├── action_move_server.py    # Action server FollowJointTrajectory
│   ├── scene_manager.py         # Gestión de colisiones MoveIt2
│   ├── calibration_node.py      # Calibración cámara-robot
│   ├── collision_calibrator.py  # Calibración interactiva del entorno
│   ├── joint_states.py          # Bridge de estados de articulaciones
│   └── vision_coordinator.py   # Integración visión+movimiento (legacy)
├── launch/
│   ├── dobot_moveit.launch.py   # Launch principal del sistema
│   ├── dobot_joint.launch.py    # Launch solo articulaciones
│   ├── moveit_demo.launch.py    # Demo MoveIt2
│   └── moveit_gazebo.launch.py  # Simulación Gazebo
└── scripts/
    └── test_collision_geometry.py  # Tests unitarios de geometría
```

## 🤖 Nodos

### `depalletizer`

Nodo principal del sistema. Orquesta el ciclo completo de pick & place con menú interactivo en terminal.

**Suscribe:**

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/detections/kartonger` | `vision_msgs/Detection2DArray` | Detecciones YOLO de cajas |
| `/joint_states` | `sensor_msgs/JointState` | Estado actual de articulaciones |
| `/dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/ToolVectorActual` | Posición TCP del robot |

**Usa:**

| Interfaz | Tipo | Descripción |
|----------|------|-------------|
| `/move_action` | Action `MoveGroup` | Planificación y ejecución MoveIt2 |
| `DOInstant` | Service | Control de salida digital (ventosa DO1) |

**Menú interactivo:**

```
1  → Ver detecciones actuales
2  → Ejecutar Pick & Place
H  → Mover a posición Home
Q  → Salir
```

```bash
ros2 run dobot_moveit depalletizer
```

---

### `action_move_server`

Action server que traduce trayectorias MoveIt2 al protocolo nativo del robot. Recibe un `FollowJointTrajectory` y ejecuta cada punto como un comando `ServoJ` con intervalos de 100 ms.

**Action server:**

| Nombre | Tipo | Descripción |
|--------|------|-------------|
| `/{DOBOT_TYPE}_group_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Recibe trayectorias de MoveIt2 |

> `DOBOT_TYPE` se configura como parámetro ROS2 (por defecto `cr20`).

```bash
ros2 run dobot_moveit action_move_server
```

---

### `scene_manager`

Gestiona los objetos de colisión en la escena de planificación MoveIt2. Lee la geometría desde `collision_config.json` y la publica al arrancar.

**Llama:**

| Service | Tipo | Descripción |
|---------|------|-------------|
| `/apply_planning_scene` | `moveit_msgs/ApplyPlanningScene` | Aplica objetos de colisión |

**Configuración:** `collision_config.json` (generado por `collision_calibrator`)

```bash
ros2 run dobot_moveit scene_manager
```

---

### `calibration_node`

Calibra la transformación cámara-robot usando marcadores AprilTag. Calcula una transformación afín XY más un plano Z para mapear coordenadas de imagen a coordenadas del robot.

**Suscribe:**

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/apriltag/detections_cm` | `apriltag_msgs/AprilTagDetectionArray` | Detecciones de AprilTags en cm |

**Ofrece:**

| Service | Tipo | Descripción |
|---------|------|-------------|
| `GetPose` | `dobot_msgs_v4/GetPose` | Consulta pose actual del TCP |

**Salida:** Guarda la calibración en `calibration_config.json`.

```bash
ros2 run dobot_moveit calibration_node
```

---

### `collision_calibrator`

Herramienta interactiva para definir el entorno de trabajo del robot. Permite mover el robot a posiciones clave y registrar las dimensiones de los objetos de colisión (mesas, paredes, palés) mediante un menú dinámico en terminal.

**Salida:** Guarda la geometría en `collision_config.json`.

```bash
ros2 run dobot_moveit collision_calibrator
```

---

### `joint_states`

Bridge que reenvía los estados de articulaciones del robot al topic estándar de ROS2.

**Suscribe:**

| Topic | Tipo |
|-------|------|
| `/joint_states_robot` | `sensor_msgs/JointState` |

**Publica:**

| Topic | Tipo |
|-------|------|
| `/joint_states` | `sensor_msgs/JointState` |

```bash
ros2 run dobot_moveit joint_states
```

---

### `vision_coordinator`

Integración visión-movimiento en modo legacy con AprilTags. Coordina detecciones de marcadores con comandos de movimiento. Sustituido funcionalmente por `depalletizer` + `calibration_node` en el flujo YOLO.

```bash
ros2 run dobot_moveit vision_coordinator
```

---

## 🚀 Cómo ejecutar

### Sistema completo (recomendado)

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch dobot_moveit dobot_moveit.launch.py
```

### Nodos individuales

```bash
# 1. Bridge de articulaciones (siempre primero)
ros2 run dobot_moveit joint_states

# 2. Action server de movimiento
ros2 run dobot_moveit action_move_server

# 3. Escena de colisiones
ros2 run dobot_moveit scene_manager

# 4. Nodo principal
ros2 run dobot_moveit depalletizer
```

### Calibración

```bash
# Calibración cámara-robot (requiere AprilTags visibles)
ros2 run dobot_moveit calibration_node

# Calibración del entorno de trabajo
ros2 run dobot_moveit collision_calibrator
```

## 🔧 Dependencias

```xml
<!-- package.xml -->
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>control_msgs</depend>
<depend>moveit_msgs</depend>
<depend>vision_msgs</depend>
<depend>apriltag_msgs</depend>
<depend>dobot_msgs_v4</depend>
```

## 📋 Scripts

### `test_collision_geometry.py`

14 tests unitarios que verifican la geometría de los objetos de colisión sin necesidad de ROS2 activo. Comprueba dimensiones, posiciones y orientaciones de todos los elementos del entorno de trabajo.

```bash
cd ~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_moveit/scripts
python3 test_collision_geometry.py
```

## 📄 Archivos de Configuración

| Archivo | Propósito | Generado por |
|---------|-----------|--------------|
| `calibration_config.json` | Transformación afín cámara-robot (XY + plano Z) | `calibration_node` |
| `collision_config.json` | Geometría de objetos de colisión del entorno | `collision_calibrator` |

Ambos archivos se guardan en el directorio de trabajo del nodo. Si no existen, los nodos que los leen arrancan con valores por defecto o avisan por consola.

## 🐛 Troubleshooting

### MoveGroup no conecta

```bash
# Verificar que MoveIt2 está activo
ros2 node list | grep move_group

# Lanzar MoveIt2 si falta
ros2 launch dobot_moveit moveit_demo.launch.py
```

### ServoJ timeout / movimiento no ejecuta

```bash
# Verificar que el action server está activo
ros2 action list | grep follow_joint_trajectory

# Comprobar conectividad con el robot
ros2 topic hz /joint_states_robot
```

### Ventosa no activa (DO1)

```bash
# Verificar servicio DOInstant disponible
ros2 service list | grep DOInstant

# El robot físico debe estar conectado y en modo remoto
```

### `collision_config.json` no encontrado

Ejecutar `collision_calibrator` antes de lanzar `scene_manager`. Sin este archivo la escena de planificación arranca vacía y el robot puede colisionar con el entorno.

### Calibración cámara-robot imprecisa

Asegurarse de que los AprilTags están planos, bien iluminados y dentro del campo de visión completo. Usar al menos 6 puntos de calibración distribuidos por toda el área de trabajo.
