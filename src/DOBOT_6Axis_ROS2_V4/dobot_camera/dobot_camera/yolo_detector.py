#!/usr/bin/env python3
"""
================================================================
Detector YOLO OBB para Kartonger (Cajas) - Despaletizador
================================================================
Detecta cajas usando YOLOv8L-OBB y cámara RealSense D435i.
Publica las posiciones en coordenadas de cámara (mm).

Publica:
  - /detections/kartonger (Float32MultiArray) - [id, x_mm, y_mm, z_mm, conf, w_px, h_px, angle_rad] x N
  - /detections/image (Image) - Imagen con detecciones dibujadas

Modelo: kartonger_best.pt (1 clase: "box", OBB)
================================================================
"""

import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import math

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ ultralytics no instalado: pip install ultralytics")


class YOLOKartongerDetector(Node):
    """Detector de cajas (kartonger) con YOLOv8 OBB."""

    # Campos por detección en el mensaje publicado
    FIELDS_PER_DETECTION = 8  # id, x_mm, y_mm, z_mm, conf, w_px, h_px, angle_rad

    def __init__(self):
        super().__init__('yolo_kartonger_detector')

        self.get_logger().info('=' * 50)
        self.get_logger().info('  DETECTOR YOLO OBB - KARTONGER (CAJAS)')
        self.get_logger().info('=' * 50)

        # ── Parámetros ──
        self.declare_parameter(
            'model_path',
            os.path.expanduser('/home/iudc/dobot_ws/yolo_training/models/kartonger_best.pt'),
        )
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('show_window', True)

        self.model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        self.show_window = self.get_parameter('show_window').value

        # ── Configuración cámara ──
        self.WIDTH = 640
        self.HEIGHT = 480
        self.MIN_DEPTH = 0.3  # metros
        self.MAX_DEPTH = 3.0

        # ── Cargar modelo YOLO ──
        self.model = None
        if YOLO_AVAILABLE:
            if os.path.exists(self.model_path):
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'✅ Modelo cargado: {self.model_path}')
            else:
                self.get_logger().error(f'❌ Modelo no encontrado: {self.model_path}')
        else:
            self.get_logger().error('❌ YOLO no disponible')

        # ── Inicializar RealSense ──
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)

        # Filtros de profundidad
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filter = rs.hole_filling_filter()
        self.align = rs.align(rs.stream.color)

        # Iniciar cámara
        try:
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info('✅ Cámara RealSense iniciada')

            # Warmup con reintentos
            import time
            warmup_ok = 0
            for attempt in range(60):
                try:
                    self.pipeline.wait_for_frames(timeout_ms=2000)
                    warmup_ok += 1
                    if warmup_ok >= 15:
                        break
                except RuntimeError:
                    self.get_logger().info(f'   Esperando cámara... ({attempt+1})')
                    time.sleep(0.5)

            if warmup_ok < 5:
                raise RuntimeError('Cámara no responde después de warmup')

            # Obtener intrínsecos
            frames = self.pipeline.wait_for_frames(timeout_ms=10000)
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            self.intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            self.get_logger().info('✅ Intrínsecos obtenidos')

        except Exception as e:
            self.get_logger().error(f'❌ Error iniciando cámara: {e}')
            raise

        # ── Publicadores ──
        self.detections_pub = self.create_publisher(
            Float32MultiArray, '/detections/kartonger', 10
        )

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image, '/detections/image', 10
        )

        # ── Ventana ──
        if self.show_window:
            cv2.namedWindow('YOLO OBB - Detección de Cajas', cv2.WINDOW_AUTOSIZE)

        # ── Timer principal ──
        self.timer = self.create_timer(0.05, self.detect_loop)  # 20 FPS

        self.get_logger().info('✅ Detector kartonger iniciado')
        self.get_logger().info('   Topic: /detections/kartonger')
        self.get_logger().info('   Formato: [id, x_mm, y_mm, z_mm, conf, w_px, h_px, angle_rad]')

    # ──────────────────────────────────────────────
    #  Profundidad
    # ──────────────────────────────────────────────
    def get_depth_at_point(self, depth_frame, cx, cy, radius=3):
        """Obtiene profundidad mediana en un área cuadrada."""
        depths = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                x = max(0, min(cx + dx, self.WIDTH - 1))
                y = max(0, min(cy + dy, self.HEIGHT - 1))
                d = depth_frame.get_distance(x, y)
                if self.MIN_DEPTH < d < self.MAX_DEPTH:
                    depths.append(d)

        if len(depths) >= 3:
            return float(np.median(depths))
        return 0.0

    # ──────────────────────────────────────────────
    #  Bucle principal
    # ──────────────────────────────────────────────
    def detect_loop(self):
        """Detecta cajas y publica posiciones 3D + ángulo."""
        if self.model is None:
            return

        try:
            # Obtener frames
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()

            if not depth_frame or not color_frame:
                return

            # Filtros de profundidad
            depth_filtered = self.spatial_filter.process(depth_frame)
            depth_filtered = self.temporal_filter.process(depth_filtered)
            depth_filtered = self.hole_filter.process(depth_filtered)
            depth_frame = depth_filtered.as_depth_frame()

            # Convertir a OpenCV
            color_image = np.asanyarray(color_frame.get_data())
            display = color_image.copy()

            # ── Detección YOLO OBB ──
            results = self.model.predict(
                source=color_image,
                conf=self.confidence,
                verbose=False,
            )

            detections_data = []

            if len(results) > 0 and results[0].obb is not None and len(results[0].obb) > 0:
                for det_id, obb in enumerate(results[0].obb):
                    # Extraer OBB: centro, tamaño, ángulo
                    xywhr = obb.xywhr[0].cpu().numpy()
                    cx_f, cy_f, w_f, h_f, angle_rad = xywhr
                    cx_px = int(cx_f)
                    cy_px = int(cy_f)
                    conf = float(obb.conf)

                    # Profundidad → coordenadas 3D
                    z = self.get_depth_at_point(depth_frame, cx_px, cy_px)
                    if z <= 0:
                        continue

                    point = rs.rs2_deproject_pixel_to_point(
                        self.intrinsics,
                        [float(cx_px), float(cy_px)],
                        z,
                    )
                    x_mm = point[0] * 1000.0
                    y_mm = point[1] * 1000.0
                    z_mm = z * 1000.0

                    # ── Publicar ──
                    detections_data.extend([
                        float(det_id),     # ID
                        x_mm,              # X cámara (mm)
                        y_mm,              # Y cámara (mm)
                        z_mm,              # Z cámara / profundidad (mm)
                        conf,              # Confianza
                        float(w_f),        # Ancho OBB (px)
                        float(h_f),        # Alto OBB (px)
                        float(angle_rad),  # Ángulo rotación (rad)
                    ])

                    # ── Visualización ──
                    angle_deg = math.degrees(angle_rad)
                    color_box = (0, 255, 0)  # Verde

                    # Dibujar OBB rotado
                    rect = ((cx_f, cy_f), (w_f, h_f), angle_deg)
                    box_pts = cv2.boxPoints(rect).astype(int)
                    cv2.polylines(display, [box_pts], True, color_box, 2)

                    # Centro
                    cv2.circle(display, (cx_px, cy_px), 4, (0, 0, 255), -1)

                    # Etiquetas
                    cv2.putText(
                        display,
                        f'box #{det_id} ({conf:.0%})',
                        (cx_px, cy_px - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_box, 2,
                    )
                    cv2.putText(
                        display,
                        f'Z:{z_mm:.0f}mm  ang:{angle_deg:.1f}°',
                        (cx_px, cy_px + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_box, 1,
                    )

            # ── Publicar detecciones ──
            if detections_data:
                msg = Float32MultiArray()
                msg.data = detections_data
                self.detections_pub.publish(msg)

            # Info en pantalla
            num_det = len(detections_data) // self.FIELDS_PER_DETECTION
            cv2.putText(
                display,
                f'Cajas detectadas: {num_det}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
            )

            # Publicar imagen
            img_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            self.image_pub.publish(img_msg)

            # Mostrar ventana
            if self.show_window:
                cv2.imshow('YOLO OBB - Detección de Cajas', display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Cerrando...')
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error en detect_loop: {e}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        try:
            self.pipeline.stop()
        except Exception as e:
            self.get_logger().warning(f'Error stopping pipeline: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YOLOKartongerDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()