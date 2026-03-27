# cra_description

Paquete de descripción del robot **Dobot CR20** para ROS2. Contiene el modelo URDF, los meshes STL de cada eslabón y los archivos de configuración para Isaac Sim. Es la base que usan `cra_moveit_config` y los nodos de control para visualizar y planificar movimientos del robot.

## 📦 Estructura

```
cra_description/
├── meshes/
│   └── cr20/
│       ├── base_link.STL     # Base fija del robot
│       ├── Link1.STL         # Eslabón 1 (rotación base)
│       ├── Link2.STL         # Eslabón 2 (hombro)
│       ├── Link3.STL         # Eslabón 3 (codo)
│       ├── Link4.STL         # Eslabón 4 (muñeca 1)
│       ├── Link5.STL         # Eslabón 5 (muñeca 2)
│       ├── Link6.STL         # Eslabón 6 (muñeca 3 / flange)
│       └── gripper_link.stl  # Efector final (gripper)
├── urdf/
│   ├── cr20_robot.xacro      # Modelo principal (fuente de verdad)
│   ├── cr20_robot.urdf       # URDF generado desde el xacro
│   ├── cr20_Moveit.urdf      # URDF adaptado para MoveIt2
│   ├── cr20_robot_isaac.usd  # Modelo para NVIDIA Isaac Sim
│   └── configuration/        # Archivos USD de configuración Isaac
│       ├── cr20_robot_isaac_base.usd
│       ├── cr20_robot_isaac_physics.usd
│       ├── cr20_robot_isaac_robot.usd
│       └── cr20_robot_isaac_sensor.usd
├── CMakeLists.txt
└── package.xml
```

## 🤖 Descripción del Robot

El **Dobot CR20** es un brazo robótico colaborativo de 6 grados de libertad diseñado para aplicaciones industriales de carga media.

| Característica | Valor |
|----------------|-------|
| Grados de libertad | 6 DOF |
| Payload máximo | 20 kg |
| Alcance máximo | 1700 mm |
| Repetibilidad | ±0.05 mm |
| Peso del robot | ~52 kg |
| Comunicación | TCP/IP (Ethernet) |
| Licencia del paquete | BSD |

La cadena cinemática va de `world` a `base_link` y luego por `Link1` hasta `Link6`, con un `gripper_link` opcional en el extremo. El archivo xacro define masas aproximadas por eslabón (base: 5 kg, Link1: 8 kg) con tensores de inercia simplificados aptos para simulación.

## 📁 Archivos Clave

### `urdf/cr20_robot.xacro`

El modelo principal. Define todos los links, joints, geometrías de colisión y propiedades inerciales. Si necesitas modificar el modelo (agregar sensores, cambiar el efector), edita este archivo y regenera el URDF.

```bash
# Regenerar el URDF desde el xacro
cd ~/dobot_ws
xacro src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr20_robot.xacro \
  > src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr20_robot.urdf
```

### `urdf/cr20_Moveit.urdf`

Variante del URDF con los joints y transmisiones configurados para MoveIt2. Lo carga automáticamente `cra_moveit_config` al planificar trayectorias.

### `meshes/cr20/*.STL`

Archivos de malla triangular para visualización y colisión. Cada STL corresponde a un eslabón. El xacro los referencia con `$(find cra_description)/meshes/cr20/`.

### `urdf/configuration/*.usd`

Archivos USD para NVIDIA Isaac Sim, divididos en capas: base, física, robot y sensores. Solo se usan en flujos de simulación con Isaac.

## 🔧 Dependencias

| Dependencia | Tipo | Uso |
|-------------|------|-----|
| `ament_cmake` | buildtool | Sistema de build de ROS2 |
| `robot_state_publisher` | runtime | Publica TF desde el URDF |
| `joint_state_publisher_gui` | runtime | Mover joints manualmente en RViz |
| `rviz2` | runtime | Visualización 3D |
| `xacro` | runtime | Procesar el archivo `.xacro` |

Instala las dependencias de ROS2 si no las tienes:

```bash
sudo apt install ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2
```

## 🚀 Uso

### Visualizar el robot en RViz

```bash
# Compilar el paquete
cd ~/dobot_ws
colcon build --packages-select cra_description --symlink-install
source install/setup.bash

# Lanzar robot_state_publisher con el URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro \
  $(ros2 pkg prefix cra_description)/share/cra_description/urdf/cr20_robot.xacro)"

# En otra terminal, abrir RViz con joint_state_publisher_gui
source ~/dobot_ws/install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
rviz2
```

En RViz agrega el display **RobotModel** y selecciona el topic `/robot_description`.

### Usar con MoveIt2

`cra_moveit_config` carga este paquete automáticamente. No necesitas lanzar `cra_description` por separado cuando usas MoveIt2.

```bash
# Lanzar el stack completo de MoveIt2 (incluye cra_description)
source ~/dobot_ws/install/setup.bash
ros2 launch cra_moveit_config demo.launch.py
```

### Verificar que el paquete está disponible

```bash
source ~/dobot_ws/install/setup.bash
ros2 pkg list | grep cra_description

# Ver la ruta de instalación
ros2 pkg prefix cra_description
```

## 🐛 Troubleshooting

### Error: "package 'cra_description' not found"

El paquete no está compilado o no hiciste source del workspace.

```bash
cd ~/dobot_ws
colcon build --packages-select cra_description
source install/setup.bash
```

### RViz muestra el robot sin meshes (solo ejes)

Los STL no se instalaron correctamente. Verifica que `CMakeLists.txt` instala la carpeta `meshes`:

```bash
# Debe aparecer la carpeta meshes en el share
ls $(ros2 pkg prefix cra_description)/share/cra_description/meshes/cr20/
```

Si no aparece, recompila con `--symlink-install` y vuelve a hacer source.

### Error al procesar el xacro: "file not found"

El comando `$(find cra_description)` dentro del xacro requiere que el paquete esté en el path de ROS2. Asegúrate de haber hecho source antes de llamar a `xacro`.

```bash
source ~/dobot_ws/install/setup.bash
xacro urdf/cr20_robot.xacro
```
