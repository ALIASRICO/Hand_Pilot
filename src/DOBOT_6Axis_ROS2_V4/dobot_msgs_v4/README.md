# dobot_msgs_v4

Interfaz de mensajes y servicios ROS2 para el robot **Dobot CR20**. Este paquete define 2 mensajes personalizados y 121 servicios que exponen cada función del robot, desde movimiento hasta control de fuerza y comunicación Modbus.

Compatible con ROS2 Jazzy e Isaac Sim 5.1.0.

## 📦 Estructura

```
dobot_msgs_v4/
├── msg/
│   ├── RobotStatus.msg        # Estado de conexión y habilitación
│   └── ToolVectorActual.msg   # Pose cartesiana actual del TCP
├── srv/
│   └── *.srv                  # 121 servicios (ver catálogo abajo)
├── CMakeLists.txt
└── package.xml
```

## 📨 Mensajes Personalizados

| Mensaje | Campos | Descripción |
|---------|--------|-------------|
| `RobotStatus` | `bool is_enable`, `bool is_connected` | Estado actual del robot: si está habilitado y conectado |
| `ToolVectorActual` | `float64 x, y, z, rx, ry, rz` | Pose cartesiana del TCP en el espacio (mm y grados) |

## ⚙️ Servicios por Categoría

### Movimiento

| Servicio | Descripción |
|----------|-------------|
| `MovJ` | Movimiento punto a punto en espacio articular |
| `MovL` | Movimiento lineal en espacio cartesiano |
| `MovJIO` / `MovLIO` | MovJ/MovL con control de E/S durante el trayecto |
| `Arc` | Movimiento en arco circular |
| `Circle` | Movimiento circular completo |
| `ServoJ` | Control servo en espacio articular (tiempo real) |
| `ServoP` | Control servo en espacio cartesiano (tiempo real) |
| `MoveJog` | Jog continuo en eje o articulación |
| `StopMoveJog` | Detiene el jog activo |
| `RelMovJTool` | Movimiento relativo articular respecto al tool |
| `RelMovJUser` | Movimiento relativo articular respecto al usuario |
| `RelMovLTool` | Movimiento relativo lineal respecto al tool |
| `RelMovLUser` | Movimiento relativo lineal respecto al usuario |
| `RelJointMovJ` | Movimiento relativo articular |
| `RunTo` | Mover a posición nombrada |
| `StartPath` | Ejecutar trayectoria pregrabada |
| `StartRTOffset` / `EndRTOffset` | Iniciar/terminar offset en tiempo real |

### Control del Robot

| Servicio | Descripción |
|----------|-------------|
| `EnableRobot` | Habilitar el robot |
| `DisableRobot` | Deshabilitar el robot |
| `ResetRobot` | Reiniciar el robot |
| `PowerOn` | Encender el robot |
| `EmergencyStop` | Parada de emergencia |
| `BrakeControl` | Control de frenos por articulación |
| `Pause` | Pausar ejecución de movimiento |
| `Continue` | Reanudar ejecución pausada |
| `Stop` | Detener ejecución |
| `SpeedFactor` | Ajustar factor de velocidad global (0-100%) |
| `RobotMode` | Consultar modo actual del robot |
| `RequestControl` | Solicitar control exclusivo |

### Entradas / Salidas

| Servicio | Descripción |
|----------|-------------|
| `DI` | Leer entrada digital |
| `DO` | Escribir salida digital |
| `DOInstant` | Escribir salida digital inmediata (sin cola) |
| `DIGroup` | Leer grupo de entradas digitales |
| `DOGroup` | Escribir grupo de salidas digitales |
| `DIGroupDEC` / `DOGroupDEC` | Grupos de E/S como entero decimal |
| `ToolDI` | Leer entrada digital del tool |
| `ToolDO` | Escribir salida digital del tool |
| `ToolDOInstant` | Escribir salida digital del tool inmediata |
| `ToolAI` | Leer entrada analógica del tool |
| `AI` | Leer entrada analógica |
| `AO` | Escribir salida analógica |
| `AOInstant` | Escribir salida analógica inmediata |
| `GetDO` / `GetDOGroup` / `GetDOGroupDEC` | Leer estado de salidas digitales |
| `GetToolDO` | Leer salida digital del tool |
| `GetAO` | Leer salida analógica |
| `DragSensivity` | Ajustar sensibilidad del modo arrastre |

### Cinemática y Poses

| Servicio | Descripción |
|----------|-------------|
| `GetPose` | Obtener pose cartesiana actual |
| `GetAngle` | Obtener ángulos articulares actuales |
| `GetStartPose` | Obtener pose de inicio de trayectoria |
| `PositiveKin` | Cinemática directa (ángulos a cartesiano) |
| `InverseKin` | Cinemática inversa (cartesiano a ángulos) |
| `InverseSolution` | Solución inversa con configuración específica |
| `CalcTool` | Calcular transformación de tool |
| `CalcUser` | Calcular transformación de usuario |
| `SetTool` | Establecer sistema de coordenadas del tool |
| `SetUser` | Establecer sistema de coordenadas del usuario |
| `Tool` | Seleccionar tool activo |
| `User` | Seleccionar sistema de usuario activo |

### Control de Fuerza

| Servicio | Descripción |
|----------|-------------|
| `GetForce` | Leer valores del sensor de fuerza/torque |
| `EnableFTSensor` | Habilitar sensor de fuerza/torque |
| `FCCollisionSwitch` | Activar/desactivar detección de colisión por fuerza |
| `FCForceMode` | Configurar modo de control de fuerza |
| `FCOff` | Desactivar control de fuerza |
| `FCSetDamping` | Configurar amortiguamiento |
| `FCSetDeviation` | Configurar desviación permitida |
| `FCSetForce` | Establecer fuerza objetivo |
| `FCSetForceLimit` | Establecer límite de fuerza |
| `FCSetForceSpeedLimit` | Establecer límite de velocidad en modo fuerza |
| `FCSetMass` | Configurar masa de la carga |
| `FCSetStiffness` | Configurar rigidez |
| `ForceDriveMode` | Modo de conducción por fuerza |
| `ForceDriveSpeed` | Velocidad en modo conducción por fuerza |
| `SetFCCollision` | Configurar parámetros de colisión por fuerza |
| `SixForceHome` | Ir a home usando sensor de 6 ejes |
| `EnableSafeSkin` | Habilitar piel de seguridad |
| `SetSafeSkin` | Configurar parámetros de piel de seguridad |

### Configuración

| Servicio | Descripción |
|----------|-------------|
| `AccJ` | Aceleración en movimiento articular |
| `AccL` | Aceleración en movimiento lineal |
| `VelJ` | Velocidad en movimiento articular |
| `VelL` | Velocidad en movimiento lineal |
| `CP` | Continuidad de trayectoria (blending) |
| `SetPayload` | Configurar carga útil (masa y centro de gravedad) |
| `SetCollisionLevel` | Nivel de sensibilidad de detección de colisión |
| `SetPostCollisionMode` | Comportamiento tras detectar colisión |
| `SetTool485` | Configurar comunicación RS485 del tool |
| `SetToolMode` | Modo del tool (eléctrico, neumático, etc.) |
| `SetToolPower` | Alimentación del tool |
| `SetBackDistance` | Distancia de retroceso tras colisión |
| `SetSafeWallEnable` | Habilitar paredes de seguridad |
| `SetWorkZoneEnable` | Habilitar zona de trabajo |

### Modbus

| Servicio | Descripción |
|----------|-------------|
| `ModbusCreate` | Crear conexión Modbus TCP |
| `ModbusClose` | Cerrar conexión Modbus |
| `ModbusRTUCreate` | Crear conexión Modbus RTU |
| `GetCoils` | Leer bobinas |
| `GetHoldRegs` | Leer registros de retención |
| `GetInBits` | Leer entradas discretas |
| `GetInRegs` | Leer registros de entrada |
| `GetInputBool` / `GetInputFloat` / `GetInputInt` | Leer entradas por tipo |
| `GetOutputBool` / `GetOutputFloat` / `GetOutputInt` | Leer salidas por tipo |
| `SetCoils` | Escribir bobinas |
| `SetHoldRegs` | Escribir registros de retención |
| `SetOutputBool` / `SetOutputFloat` / `SetOutputInt` | Escribir salidas por tipo |

### Trayectorias, Scripts y Diagnóstico

| Servicio | Descripción |
|----------|-------------|
| `RunScript` | Ejecutar script de robot |
| `StartDrag` | Iniciar modo arrastre manual |
| `StopDrag` | Detener modo arrastre manual |
| `GetCurrentCommandId` | Obtener ID del comando en ejecución |
| `GetError` | Obtener descripción del error activo |
| `GetErrorID` | Obtener código de error activo |
| `ClearError` | Limpiar errores del robot |
| `TcpDashboard` | Consulta directa al dashboard TCP del robot |

## 🔧 Dependencias

| Dependencia | Rol |
|-------------|-----|
| `ament_cmake` | Sistema de build |
| `rosidl_default_generators` | Generación de interfaces en tiempo de build |
| `rosidl_default_runtime` | Runtime de interfaces generadas |
| `std_msgs`, `geometry_msgs` | Tipos base de ROS2 |

## 🚀 Cómo usar en otros paquetes

### package.xml

```xml
<depend>dobot_msgs_v4</depend>
```

### CMakeLists.txt

```cmake
find_package(dobot_msgs_v4 REQUIRED)

ament_target_dependencies(mi_nodo
  rclcpp
  dobot_msgs_v4
)
```

### Ejemplo en Python

```python
from dobot_msgs_v4.srv import MovJ, EnableRobot
from dobot_msgs_v4.msg import RobotStatus, ToolVectorActual
```

### Compilar

```bash
cd ~/dobot_ws
colcon build --packages-select dobot_msgs_v4
source install/setup.bash
```
