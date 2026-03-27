#!/usr/bin/env python3
"""
================================================================
Nodo de Visión para Cámara RealSense + AprilTags
================================================================
Basado en el código funcionando de paletizado_ws

Publica:
  - /vision/tags_detected (TagArray) - Detecciones de AprilTags
  - /vision/camera_info (CameraInfo) - Intrínsecos de la cámara
  - /apriltag/detections_cm (Float32MultiArray) - Para compatibilidad
  
Uso:
    ros2 run dobot_camera vision_node
================================================================
"""

import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector
from collections import deque

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray


class VisionNode(Node):
    """
    Nodo de visión que procesa la cámara RealSense y detecta AprilTags.
    Versión mejorada basada en paletizado_ws.
    """

    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Iniciando cámara RealSense con AprilTags...')

        # Configuración
        self.WIDTH = 640
        self.HEIGHT = 480
        self.MIN_DEPTH = 0.1
        self.MAX_DEPTH = 16.0

        # Historial para estabilización
        self.HISTORY_SIZE = 10
        self.tag_history = {}
        self.MOVEMENT_THRESHOLD = 0.05  # 5cm

        # Inicializar RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)

        # Filtros de profundidad
        self.spatial_filter = rs.spatial_filter()
        self.spatial_filter.set_option(rs.option.filter_magnitude, 2)
        self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial_filter.set_option(rs.option.filter_smooth_delta, 20)
        self.spatial_filter.set_option(rs.option.holes_fill, 2)

        self.temporal_filter = rs.temporal_filter()
        self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.3)
        self.temporal_filter.set_option(rs.option.filter_smooth_delta, 40)

        self.hole_filling_filter = rs.hole_filling_filter()
        self.hole_filling_filter.set_option(rs.option.holes_fill, 2)

        self.align = rs.align(rs.stream.color)

        # Iniciar cámara
        try:
            self.profile = self.pipeline.start(self.config)

            depth_sensor = self.profile.get_device().first_depth_sensor()
            try:
                if depth_sensor.supports(rs.option.visual_preset):
                    depth_sensor.set_option(rs.option.visual_preset, 4)  # High density
            except Exception:
                pass

            try:
                if depth_sensor.supports(rs.option.laser_power):
                    max_laser = depth_sensor.get_option_range(rs.option.laser_power).max
                    depth_sensor.set_option(rs.option.laser_power, max_laser)
            except Exception:
                pass

            self.get_logger().info('Cámara iniciada correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al iniciar cámara: {e}')
            raise

        # Warmup de la cámara
        for _ in range(30):
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            depth = aligned.get_depth_frame()
            if depth:
                _ = self.apply_filters(depth)

        # Obtener intrínsecos
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        self.intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        # Detector de AprilTags
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25
        )

        # Publicadores
        # Camera info
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, 
            '/vision/camera_info', 
            10
        )
        
        # Detecciones en formato compatible con coordinator
        # Formato: [id, x_mm, y_mm, z_mm, id, x_mm, y_mm, z_mm, ...]
        self.detections_pub = self.create_publisher(
            Float32MultiArray,
            '/apriltag/detections_cm',
            10
        )

        # Ventana OpenCV
        self.window_name = 'Vision - AprilTags'
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        # Timer principal
        self.timer = self.create_timer(0.033, self.loop)  # ~30 FPS
        self.running = True

        self.get_logger().info('Vision Node iniciado correctamente')
        self.get_logger().info('Topics: /vision/camera_info, /apriltag')

    def apply_filters(self, depth_frame):
        """Aplica filtros de profundidad"""
        filtered = depth_frame
        filtered = self.spatial_filter.process(filtered)
        filtered = self.temporal_filter.process(filtered)
        filtered = self.hole_filling_filter.process(filtered)
        return filtered

    def publish_camera_info(self):
        """Publica los intrínsecos de la cámara"""
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.width = self.WIDTH
        msg.height = self.HEIGHT
        
        # Matriz K (intrínsecos)
        msg.k = [
            float(self.intrinsics.fx), 0.0, float(self.intrinsics.ppx),
            0.0, float(self.intrinsics.fy), float(self.intrinsics.ppy),
            0.0, 0.0, 1.0
        ]
        
        # Matriz P (proyección)
        msg.p = [
            float(self.intrinsics.fx), 0.0, float(self.intrinsics.ppx), 0.0,
            0.0, float(self.intrinsics.fy), float(self.intrinsics.ppy), 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Matriz R (rotación) - identidad
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Modelo de distorsión
        msg.distortion_model = 'plumb_bob'
        msg.d = [float(c) for c in self.intrinsics.coeffs]
        
        self.camera_info_publisher.publish(msg)

    def get_depth_at_point(self, depth_frame, raw_depth_frame, cx, cy):
        """Obtiene la profundidad en un punto específico con promedio"""
        cx = max(5, min(cx, self.WIDTH - 6))
        cy = max(5, min(cy, self.HEIGHT - 6))

        depths = []
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                try:
                    d = depth_frame.get_distance(cx + dx, cy + dy)
                    if self.MIN_DEPTH < d < self.MAX_DEPTH:
                        depths.append(d)
                except Exception:
                    pass

        if len(depths) >= 3:
            return float(np.median(depths))

        # Fallback a profundidad cruda
        if raw_depth_frame:
            depths = []
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    try:
                        d = raw_depth_frame.get_distance(cx + dx, cy + dy)
                        if self.MIN_DEPTH < d < self.MAX_DEPTH:
                            depths.append(d)
                    except Exception:
                        pass
            if len(depths) >= 3:
                return float(np.median(depths))

        return 0.0

    def get_stable_position(self, tag_id, x, y, z, u, v):
        """Estabiliza las posiciones usando historial"""
        if tag_id not in self.tag_history:
            self.tag_history[tag_id] = deque(maxlen=self.HISTORY_SIZE)

        history = self.tag_history[tag_id]

        # Limpiar historial si hay movimiento grande
        if len(history) > 0:
            last_x, last_y, last_z, _, _ = history[-1]
            dist = np.sqrt((x - last_x) ** 2 + (y - last_y) ** 2 + (z - last_z) ** 2)
            if dist > self.MOVEMENT_THRESHOLD:
                history.clear()

        history.append((x, y, z, u, v))

        if len(history) < 3:
            return x, y, z, u, v, False

        # Usar mediana para estabilizar
        arr = np.array(list(history))
        stable_x = float(np.median(arr[:, 0]))
        stable_y = float(np.median(arr[:, 1]))
        stable_z = float(np.median(arr[:, 2]))
        stable_u = float(np.median(arr[:, 3]))
        stable_v = float(np.median(arr[:, 4]))
        is_stable = len(history) >= 5

        return stable_x, stable_y, stable_z, stable_u, stable_v, is_stable

    def loop(self):
        """Bucle principal de procesamiento"""
        if not self.running:
            return

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()

            if not depth_frame or not color_frame:
                return

            raw_depth = depth_frame.as_depth_frame()
            depth_filtered = self.apply_filters(depth_frame)
            depth_for_query = depth_filtered.as_depth_frame()

            # Convertir a OpenCV
            color_img = np.asanyarray(color_frame.get_data())
            gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
            display = color_img.copy()

            # Detectar AprilTags
            detections = self.detector.detect(gray_img)

            # Lista para publicar detecciones
            detections_data = []
            
            # Procesar detecciones
            for det in detections:
                tag_id = int(det.tag_id)
                cx = int(det.center[0])
                cy = int(det.center[1])

                if not (0 <= cx < self.WIDTH and 0 <= cy < self.HEIGHT):
                    continue

                # Obtener profundidad
                z = self.get_depth_at_point(depth_for_query, raw_depth, cx, cy)
                
                if z <= self.MIN_DEPTH or z > self.MAX_DEPTH:
                    # Dibujar sin Z válido
                    cv2.polylines(display, [det.corners.astype(int)], True, (128, 128, 128), 1)
                    cv2.putText(display, f'{tag_id} (sin Z)', (cx - 30, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
                    continue

                # Deproyectar a 3D
                pt = rs.rs2_deproject_pixel_to_point(self.intrinsics, [float(cx), float(cy)], float(z))
                x, y = float(pt[0]), float(pt[1])

                # Estabilizar posición
                stable_x, stable_y, stable_z, stable_u, stable_v, is_stable = \
                    self.get_stable_position(tag_id, x, y, z, float(cx), float(cy))

                # Convertir a mm para publicación
                x_mm = stable_x * 1000.0
                y_mm = stable_y * 1000.0
                z_mm = stable_z * 1000.0
                
                # Agregar a lista de detecciones
                detections_data.extend([float(tag_id), x_mm, y_mm, z_mm])

                # Visualización
                col = (0, 255, 0)  # Verde para tags normales
                if tag_id == 99:
                    col = (255, 0, 255)  # Magenta para tag 99
                
                cv2.polylines(display, [det.corners.astype(int)], True, col, 2)
                cv2.circle(display, (cx, cy), 4, col, -1)
                
                # Mostrar información
                status = '✓' if is_stable else '...'
                cv2.putText(display, f'ID:{tag_id} {status}', (cx - 20, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
                cv2.putText(display, f'X:{x_mm:.0f}mm', (cx - 30, cy + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, col, 1)
                cv2.putText(display, f'Y:{y_mm:.0f}mm', (cx - 30, cy + 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, col, 1)
                cv2.putText(display, f'Z:{z_mm:.0f}mm', (cx - 30, cy + 41),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, col, 1)

            # Publicar camera info
            self.publish_camera_info()
            
            # Publicar detecciones
            if detections_data:
                det_msg = Float32MultiArray()
                det_msg.data = detections_data
                self.detections_pub.publish(det_msg)

            # Mostrar en ventana
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Cerrando vision_node...')
                self.running = False
                self.timer.cancel()
                rclpy.shutdown()

            # Log periódico
            if len(detections_data) > 0:
                self.get_logger().debug(f'Detectados: {len(detections_data)//4} tags')

        except Exception as e:
            self.get_logger().error(f'Error en loop: {e}')

    def destroy_node(self):
        self.running = False
        cv2.destroyAllWindows()
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
