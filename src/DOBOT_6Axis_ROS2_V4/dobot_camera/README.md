# 📷 dobot_camera

Pipeline de visión para el robot DOBOT CR20. Combina detección de AprilTags con cámara RealSense D435i y detección de cajas de jugo con YOLO OBB, publicando posiciones 3D en milímetros para el sistema de despaletizado.

## 📦 Estructura del paquete

```
dobot_camera/
├── dobot_camera/
│   ├── vision_node.py          # Nodo AprilTags con RealSense D435i
│   ├── yolo_detector.py        # Nodo YOLO OBB para cajas (kartonger)
│   ├── juice_logic.py          # Lógica de clasificación MANGO/MORA
│   └── test_juice_logic.py     # Tests unitarios de juice_logic
├── launch/                     # Launch files (vacío, usar ros2 run)
├── setup.py
└── package.xml
```

## 🤖 Nodos

| Nodo | Clase | Descripción |
|------|-------|-------------|
| `vision_node` | `VisionNode` | Detecta AprilTags (tag36h11) con RealSense D435i, publica posiciones 3D estabilizadas |
| `yolo_detector` | `YOLOKartongerDetector` | Detecta cajas de jugo con YOLOv8L-OBB, publica bounding boxes 3D con ángulo |

### vision_node

Procesa el stream de color y profundidad de la RealSense a 30 FPS. Usa `pupil-apriltags` para detectar marcadores tag36h11 y estabiliza las posiciones con un historial de 10 frames (mediana). Publica en mm.

### yolo_kartonger_detector

Corre a 20 FPS. Aplica filtros espacial, temporal y de relleno de huecos al frame de profundidad antes de deproyectar. Cada detección incluye posición 3D del centro de la caja y el ángulo del bounding box orientado.

## 📡 Topics

### Publicados

| Topic | Tipo | Formato / Descripción |
|-------|------|-----------------------|
| `/detections/kartonger` | `std_msgs/Float32MultiArray` | `[id, x_mm, y_mm, z_mm, conf, w_px, h_px, angle_rad]` x N detecciones |
| `/detections/image` | `sensor_msgs/Image` | Frame BGR con OBBs dibujados (encoding: bgr8) |
| `/apriltag/detections_cm` | `std_msgs/Float32MultiArray` | `[id, x_mm, y_mm, z_mm]` x N tags detectados |
| `/vision/camera_info` | `sensor_msgs/CameraInfo` | Intrínsecos de la cámara (fx, fy, ppx, ppy, distorsión) |

### Formato de `/detections/kartonger`

Cada detección ocupa 8 floats consecutivos:

```
índice 0: id          — ID de detección (0, 1, 2...)
índice 1: x_mm        — X en coordenadas de cámara (mm)
índice 2: y_mm        — Y en coordenadas de cámara (mm)
índice 3: z_mm        — Profundidad desde la cámara (mm)
índice 4: conf        — Confianza del modelo (0.0 - 1.0)
índice 5: w_px        — Ancho del OBB en píxeles
índice 6: h_px        — Alto del OBB en píxeles
índice 7: angle_rad   — Ángulo de rotación del OBB (radianes)
```

Para leer N detecciones: `data[i*8 : i*8+8]` con `N = len(data) // 8`.

## 🧃 Lógica de Jugos

`juice_logic.py` clasifica cada caja detectada como MANGO o MORA sin depender de ROS2, lo que permite testearlo de forma aislada.

### Constantes

| Constante | Valor | Significado |
|-----------|-------|-------------|
| `JUICE_TYPE_MANGO` | `0.0` | Caja de jugo de mango |
| `JUICE_TYPE_MORA` | `1.0` | Caja de jugo de mora |
| `JUICE_TYPE_UNKNOWN` | `-1.0` | Tipo no determinado |

### Clases del modelo YOLO

| `class_id` | Nombre | Descripción |
|------------|--------|-------------|
| `0` | `HIT_MANGO` | Etiqueta de mango visible en la caja |
| `1` | `HIT_MORA` | Etiqueta de mora visible en la caja |
| `2` | `CARA_F` | Cara frontal de la caja (la que se agarra) |

### Asociación cara-jugo

`associate_cara_f_with_hit(cara_f_list, hit_list)` busca el HIT más cercano a cada `CARA_F` dentro de un radio de 150 px. Si encuentra un HIT dentro del umbral, asigna el `juice_type` correspondiente. Si no hay ninguno cerca, devuelve `JUICE_TYPE_UNKNOWN`.

`sort_detections_for_pick(detections)` ordena las cajas para el pick: primero por `cam_z_mm` ascendente (las más cercanas a la cámara, es decir, las de arriba del pallet), y dentro de la misma capa (tolerancia 15 mm) por `robot_dist_mm` ascendente.

## 🔧 Dependencias Python

```bash
pip install pyrealsense2 ultralytics opencv-python pupil-apriltags
```

También requiere `cv_bridge` de ROS2 (viene con `ros-humble-cv-bridge`).

| Librería | Uso |
|----------|-----|
| `pyrealsense2` | Stream de color y profundidad de la RealSense D435i |
| `ultralytics` | Inferencia YOLOv8L-OBB |
| `opencv-python` | Procesamiento de imagen y visualización |
| `pupil-apriltags` | Detección de marcadores AprilTag |
| `cv_bridge` | Conversión entre OpenCV y mensajes ROS2 Image |

## 🤖 Modelo YOLO

El modelo entrenado para detección de cajas se llama `kartonger_best.pt` y se ubica en:

```
~/dobot_ws/yolo_training/models/kartonger_best.pt
```

Entrenado con el dataset **Kartonger** (workspace `jugos` en Roboflow). Arquitectura YOLOv8L-OBB, 1 clase: `box`. El nodo lo carga al arrancar y falla con error si no lo encuentra.

Para cambiar la ruta del modelo sin recompilar:

```bash
ros2 run dobot_camera yolo_detector --ros-args -p model_path:=/ruta/al/modelo.pt
```

## 🚀 Cómo ejecutar

### Detector de cajas (YOLO OBB)

```bash
source ~/dobot_ws/install/setup.bash
ros2 run dobot_camera yolo_detector
```

Con parámetros opcionales:

```bash
ros2 run dobot_camera yolo_detector --ros-args \
  -p model_path:=/home/iudc/dobot_ws/yolo_training/models/kartonger_best.pt \
  -p confidence:=0.5 \
  -p show_window:=true
```

### Nodo AprilTags

```bash
source ~/dobot_ws/install/setup.bash
ros2 run dobot_camera vision_node
```

### Compilar el paquete

```bash
cd ~/dobot_ws
colcon build --packages-select dobot_camera --symlink-install
source install/setup.bash
```

### Verificar topics activos

```bash
ros2 topic list | grep -E "detections|apriltag|vision"
ros2 topic hz /detections/kartonger
ros2 topic echo /apriltag/detections_cm
```

## 🐛 Troubleshooting

### Cámara no detectada

```bash
# Verificar que la RealSense está conectada
lsusb | grep Intel

# Verificar permisos USB
ls -la /dev/video*

# Probar con realsense-viewer
realsense-viewer
```

### Modelo no encontrado

```bash
# Verificar que el archivo existe
ls ~/dobot_ws/yolo_training/models/kartonger_best.pt

# Si no existe, copiar desde la ubicación de entrenamiento
cp ~/runs/obb/kartonger_yolov8l/weights/best.pt \
   ~/dobot_ws/yolo_training/models/kartonger_best.pt
```

### ultralytics no instalado

```bash
pip install ultralytics

# O con el entorno virtual del workspace
~/dobot_ws/yolo_venv/bin/pip install ultralytics
```

### Sin profundidad válida (Z=0)

La cámara necesita entre 0.3 m y 3.0 m de distancia al objeto. Verifica que el objeto esté dentro de ese rango y que el láser de la RealSense esté encendido. El nodo aplica filtros de profundidad automáticamente, pero superficies muy reflectantes o transparentes pueden dar Z=0 y se descartan.

### No aparece ventana OpenCV

```bash
# Desactivar la ventana si corres en headless
ros2 run dobot_camera yolo_detector --ros-args -p show_window:=false
```
