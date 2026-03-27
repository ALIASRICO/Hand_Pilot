# dobot_bringup_v4 - Driver ROS2 para Dobot CR20

Driver C++ que conecta ROS2 con el robot físico **Dobot CR20** por TCP/IP. Crea los servicios ROS2, publica el estado del robot en tiempo real y expone la interfaz completa de movimiento.

## 📦 Estructura del paquete

```
dobot_bringup_v4/
├── config/
│   └── param.json              # Configuración de robots (IP, tipo, nodo)
├── include/dobot_bringup/
│   ├── command.h               # Struct RealTimeData_t y declaraciones de comandos
│   ├── cr_robot_ros2.h         # Clase principal del nodo ROS2
│   ├── parseTool.h             # Utilidades de parseo de respuestas TCP
│   └── tcp_socket.h            # Abstracción de sockets TCP
├── launch/
│   └── dobot_bringup_ros2.launch.py
├── src/
│   ├── main.cpp                # Punto de entrada, publishers de estado
│   ├── cr_robot_ros2.cpp       # Lógica del nodo: servicios, callbacks
│   ├── command.cpp             # Implementación de comandos al robot
│   ├── parseTool.cpp           # Parser de respuestas del Dashboard
│   └── tcp_socket.cpp          # Conexión TCP con el controlador
└── package.xml
```

## 🌐 Protocolo de Comunicación

El controlador Dobot expone tres puertos TCP simultáneos:

| Puerto | Nombre | Descripción |
|--------|--------|-------------|
| `29999` | Dashboard | Comandos sincrónicos (enable, disable, reset, modos) |
| `30003` | Feedback | Estado en tiempo real, ciclo de 1440 bytes |
| `30004` | Real-time Motion | Comandos de movimiento en tiempo real |

El driver abre conexión a los tres puertos al arrancar. El puerto 30003 se lee en un hilo dedicado que actualiza el estado interno del robot a cada ciclo.

## 📡 Topics

### Publicados

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/joint_states_robot` | `sensor_msgs/JointState` | Posiciones articulares de las 6 juntas (rad) |
| `/dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/ToolVectorActual` | Posición TCP: x, y, z (mm) + rx, ry, rz (deg) |
| `/dobot_msgs_v4/msg/RobotStatus` | `dobot_msgs_v4/RobotStatus` | Estado del robot: `is_enable`, `is_connected` |

> El topic `joint_states_robot` se remapea a `joint_states` en el launch file para compatibilidad con `robot_state_publisher`.

### Tasa de publicación

La tasa de publicación de joint states se controla con el parámetro `JointStatePublishRate` (default: 10 Hz).

## ⚙️ Variables de Entorno

El launch file lee estas variables antes de arrancar el nodo:

| Variable | Default | Descripción |
|----------|---------|-------------|
| `IP_address` | `192.168.5.1` | Dirección IP del controlador del robot |
| `DOBOT_TYPE` | `cr5` | Tipo de robot (ej: `cr20`, `cr5`, `cr10`) |

Si `IP_address` no está definida, el nodo falla al conectar. Si `DOBOT_TYPE` no está definida, el código usa `cr5` como fallback.

## 🚀 Cómo ejecutar

### 1. Compilar el paquete

```bash
cd ~/dobot_ws
colcon build --packages-select dobot_bringup_v4
source install/setup.bash
```

### 2. Configurar y lanzar

```bash
export IP_address=192.168.5.1
export DOBOT_TYPE=cr20
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

### 3. Verificar conexión

```bash
# Ver topics activos
ros2 topic list | grep -E "joint|dobot"

# Monitorear posición articular
ros2 topic echo /joint_states

# Ver estado del robot
ros2 topic echo /dobot_msgs_v4/msg/RobotStatus
```

## 🔧 Archivos fuente

| Archivo | Propósito |
|---------|-----------|
| `main.cpp` | Inicializa el nodo ROS2, crea publishers y ejecuta el loop principal de publicación |
| `cr_robot_ros2.cpp` | Clase `CRRobotRos2`: registra los ~121 servicios ROS2 y gestiona el ciclo de vida |
| `command.cpp` | Implementa los comandos que se envían al Dashboard (puerto 29999) |
| `parseTool.cpp` | Parsea las respuestas de texto del Dashboard y extrae valores |
| `tcp_socket.cpp` | Maneja la conexión TCP: apertura, lectura bloqueante y reconexión |
| `command.h` | Define `RealTimeData_t` (struct de 1440 bytes del puerto 30003) y prototipos |

## 🐛 Troubleshooting

### El robot no conecta

```bash
# Verificar conectividad de red
ping 192.168.5.1

# Confirmar que las variables de entorno están definidas
echo $IP_address
echo $DOBOT_TYPE
```

Si el ping falla, revisar que el cable Ethernet está conectado y que la IP del PC está en la misma subred (ej: `192.168.5.100/24`).

### `/joint_states` no llega

```bash
# Verificar que el nodo está corriendo
ros2 node list | grep dobot

# Ver frecuencia real del topic
ros2 topic hz /joint_states
```

Si el nodo arranca pero no publica, el robot puede estar en modo error. Revisar los logs del nodo y habilitar el robot desde el panel o con:

```bash
ros2 service call /EnableRobot dobot_msgs_v4/srv/EnableRobot
```

### El nodo muere al arrancar

El launch file lee `config/param.json` antes de lanzar el nodo. Si el archivo no existe o tiene formato incorrecto, el launch falla con un error de Python. Verificar que el JSON es válido:

```bash
python3 -c "import json; json.load(open('src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/config/param.json'))"
```

### Posiciones articulares en cero

Si `/joint_states` publica pero todos los valores son `0.0`, el robot está conectado pero no habilitado. Habilitar con:

```bash
ros2 service call /EnableRobot dobot_msgs_v4/srv/EnableRobot
```

## ⚠️ Notas

- El robot debe estar en modo **TCP/IP** y con el controlador encendido antes de lanzar el driver.
- No lanzar dos instancias del driver apuntando al mismo robot: el controlador solo acepta una conexión por puerto.
- El parámetro `robot_node_name` en `param.json` define el nombre del nodo ROS2. Cambiarlo si se usan varios robots en el mismo workspace.
