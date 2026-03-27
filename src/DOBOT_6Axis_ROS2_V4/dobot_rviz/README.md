# dobot_rviz - Visualización RViz2 para DOBOT CR20

Paquete de visualización del robot DOBOT CR20 en RViz2. Contiene los launch files para ver el modelo URDF del robot, ya sea en modo standalone con joint states manuales o conectado a NVIDIA Isaac Sim.

## 📦 Estructura

```
dobot_rviz/
├── launch/
│   ├── dobot_rviz.launch.py      # Visualización standalone con joint_state_publisher
│   └── display_isaac.launch.py   # Visualización conectada a Isaac Sim
├── rviz/
│   └── urdf.rviz                 # Configuración de RViz2
└── urdf/
    └── cr20_robot.urdf           # Modelo URDF del robot
```

## 🚀 Launch Files

| Archivo | Descripción | Cuándo usar |
|---------|-------------|-------------|
| `dobot_rviz.launch.py` | Visualiza el robot con joint states locales | Inspección del modelo, pruebas sin simulador |
| `display_isaac.launch.py` | Visualiza el robot recibiendo joint states de Isaac Sim | Cuando Isaac Sim ya está corriendo y enviando datos |

## 🔧 Dependencias

- `rviz2` — visualizador 3D
- `robot_state_publisher` — publica las transformadas TF del robot a partir del URDF
- `joint_state_publisher` — genera joint states manuales (solo en `dobot_rviz.launch.py`)
- `joint_state_publisher_gui` — sliders para mover las articulaciones manualmente (opcional)
- `xacro` — procesador de URDF

Instalar si faltan:

```bash
sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
```

## 💡 Cómo ejecutar

### Requisito previo

Definir el tipo de robot antes de lanzar:

```bash
export DOBOT_TYPE=cr20
```

Compilar el paquete si aún no está instalado:

```bash
cd ~/dobot_ws
colcon build --packages-select dobot_rviz --symlink-install
source install/setup.bash
```

---

### Visualización standalone

Lanza RViz2 con `joint_state_publisher` para mover el robot manualmente desde la terminal:

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```

Para abrir los sliders gráficos de articulaciones:

```bash
ros2 launch dobot_rviz dobot_rviz.launch.py gui:=true
```

Parámetros disponibles:

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `gui` | `false` | Activa `joint_state_publisher_gui` con sliders |
| `model` | `urdf/cr20_robot.urdf` | Ruta al archivo URDF |
| `rvizconfig` | `rviz/urdf.rviz` | Ruta a la configuración de RViz2 |

---

### Visualización con Isaac Sim

Usa este launch cuando Isaac Sim ya está corriendo y publicando `/joint_states`. No lanza `joint_state_publisher` porque Isaac Sim ya envía esos datos.

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch dobot_rviz display_isaac.launch.py
```

Si `DOBOT_TYPE` no está definido, el launch usa `cr20` por defecto.

## 🐛 Troubleshooting

### RViz no muestra el robot

Verificar que `robot_state_publisher` está publicando la descripción:

```bash
ros2 topic echo /robot_description --once
```

Verificar que hay joint states llegando:

```bash
ros2 topic hz /joint_states
```

Si no hay joint states y no usas Isaac Sim, asegúrate de lanzar con `dobot_rviz.launch.py` (no `display_isaac`).

### Error: "DOBOT_TYPE not set"

```bash
export DOBOT_TYPE=cr20
```

### El modelo aparece en gris o sin color

Revisar que el URDF tiene materiales definidos y que la ruta al archivo es correcta:

```bash
ros2 param get /robot_state_publisher robot_description
```

### RViz abre pero las TF están en rojo

Significa que `robot_state_publisher` no está publicando. Confirmar que el nodo está activo:

```bash
ros2 node list | grep robot_state_publisher
```
