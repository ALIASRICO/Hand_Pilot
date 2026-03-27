#!/usr/bin/env python3
"""
Calibración Cámara-Robot:
- Ajuste XY: matriz afín (camera_x,y -> robot_x,y)
- Ajuste Z: plano de superficie (pallet/mesa)
    z_plane_cam:  z_cam_surface = a*cam_x + b*cam_y + c
    z_plane_robot: z_robot_surface = a*x_robot + b*y_robot + c

Nota:
- El plano describe la SUPERFICIE (tags están en plano).
- Para obtener altura de objeto: h = z_plane_cam(x,y) - z_obj_cam
"""

import rclpy
from rclpy.node import Node
import numpy as np
import json
import os
import time
import threading
import collections

from std_msgs.msg import Float32MultiArray
from dobot_msgs_v4.srv import GetPose


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        self.tag_info = {
            0: 'INFERIOR IZQUIERDA',
            1: 'INFERIOR DERECHA',
            2: 'SUPERIOR IZQUIERDA',
            3: 'SUPERIOR DERECHA',
        }
        self.calibration_sequence = [0, 1, 2, 3]

        self.current_detections = {}
        self.saved_camera_coords = {}
        self.calibration_points = {}
        self.detection_lock = threading.Lock()

        self.affine_matrix = None

        # Buffer de Z por tag (mediana)
        self.z_buffer = collections.defaultdict(lambda: collections.deque(maxlen=30))

        self.detections_sub = self.create_subscription(
            Float32MultiArray,
            '/apriltag/detections_cm',
            self.detections_callback,
            10
        )

        self.get_pose_client = self.create_client(
            GetPose,
            '/dobot_bringup_ros2/srv/GetPose'
        )

        self.pose_available = self.get_pose_client.wait_for_service(timeout_sec=5.0)
        if self.pose_available:
            self.get_logger().info("GetPose disponible")
        else:
            self.get_logger().warn("GetPose NO disponible (modo manual)")

        self.ui_thread = threading.Thread(target=self.guided_process)
        self.ui_thread.daemon = True
        self.ui_thread.start()

    # ---------- GetPose parse (robot_return string) ----------
    def get_robot_pose(self):
        if not self.get_pose_client.wait_for_service(timeout_sec=1.0):
            return None

        req = GetPose.Request()
        req.user = 0
        req.tool = 0

        future = self.get_pose_client.call_async(req)

        t0 = time.time()
        while rclpy.ok() and not future.done() and (time.time() - t0) < 2.0:
            time.sleep(0.01)

        if not future.done():
            self.get_logger().error("Timeout esperando GetPose")
            return None

        if future.exception() is not None:
            self.get_logger().error(f"GetPose exception: {future.exception()}")
            return None

        res = future.result()
        if res is None or res.res != 0:
            self.get_logger().error(f"GetPose res={getattr(res,'res',None)} robot_return={getattr(res,'robot_return',None)}")
            return None

        s = res.robot_return.strip().strip("{}")
        parts = [p.strip() for p in s.split(",") if p.strip()]
        if len(parts) < 6:
            self.get_logger().error(f"robot_return inválido: {res.robot_return}")
            return None

        x, y, z, rx, ry, rz = map(float, parts[:6])
        return {"x": x, "y": y, "z": z, "rx": rx, "ry": ry, "rz": rz}

    # ---------- Detections with Z median filter ----------
    def detections_callback(self, msg):
        data = msg.data
        with self.detection_lock:
            self.current_detections.clear()
            for i in range(0, len(data), 4):
                tag_id = int(data[i])
                cx = float(data[i+1])
                cy = float(data[i+2])
                cz = float(data[i+3])  # mm (depth)

                self.z_buffer[tag_id].append(cz)
                cz_med = float(np.median(self.z_buffer[tag_id]))

                self.current_detections[tag_id] = {
                    "x": cx,
                    "y": cy,
                    "z": cz_med,
                }

    def clear_screen(self):
        os.system('clear' if os.name == 'posix' else 'cls')

    def wait_enter(self, msg="Presiona ENTER..."):
        input(f"\n    {msg}")

    def get_input(self, prompt):
        return input(f"    {prompt}")

    def fit_z_plane_xyz(self, pts_xyz, label=""):
        """
        Ajusta z = a*x + b*y + c.
        pts_xyz: np.array (N,3) en mm
        """
        A = np.column_stack([pts_xyz[:, 0], pts_xyz[:, 1], np.ones(len(pts_xyz))])
        z = pts_xyz[:, 2]
        params, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
        a, b, c = params
        z_pred = A @ params
        err = np.abs(z_pred - z)

        self.get_logger().info(f"Ajuste plano Z {label}:")
        for k in range(len(pts_xyz)):
            self.get_logger().info(f"  P{k}: z={z[k]:.1f}  z_fit={z_pred[k]:.1f}  err={err[k]:.1f}mm")
        self.get_logger().info(f"  Error máximo: {err.max():.1f}mm | RMS: {np.sqrt(np.mean(err**2)):.1f}mm")

        return {'a': float(a), 'b': float(b), 'c': float(c)}

    def guided_process(self):
        time.sleep(1.5)
        self.clear_screen()
        print()
        print("    ╔═══════════════════════════════════════════╗")
        print("    ║    CALIBRACIÓN XY + PLANO Z (AUTÓNOMA)    ║")
        print("    ╚═══════════════════════════════════════════╝")
        print()
        print("    Para cada Tag (0,1,2,3):")
        print("      A) Guardar cámara (x,y,z_mediana) con robot lejos")
        print("      B) Tocar el tag con TCP y registrar robot (x,y,z)")
        print()
        self.wait_enter("ENTER para comenzar...")

        for tag_id in self.calibration_sequence:
            if not self.calibrate_tag(tag_id):
                print("\n    ❌ Cancelado")
                return

        self.calculate_and_save()
        self.wait_enter("ENTER para salir...")
        rclpy.shutdown()

    def calibrate_tag(self, tag_id):
        tag_name = self.tag_info[tag_id]
        step = self.calibration_sequence.index(tag_id) + 1

        # FASE A: cámara
        while True:
            self.clear_screen()
            print(f"\n    === Tag {tag_id} ({step}/4) - FASE A (CÁMARA) ===")
            print(f"    Posición: {tag_name}")
            print("    Robot lejos: NO tape el tag")
            print()

            with self.detection_lock:
                visible = tag_id in self.current_detections
                cam = self.current_detections.get(tag_id, None)

            if visible:
                print(f"    ✅ Detectado: X={cam['x']:.1f}  Y={cam['y']:.1f}  Zmed={cam['z']:.1f} mm")
                print("    ENTER=Guardar | r=Refrescar | q=Cancelar")
            else:
                print("    ❌ No visible")
                print("    r=Refrescar | q=Cancelar")

            choice = self.get_input("\n    Opción: ").strip().lower()
            if choice == 'q':
                return False
            if choice == '' and visible:
                self.saved_camera_coords[tag_id] = {'x': cam['x'], 'y': cam['y'], 'z': cam['z']}
                break

        # FASE B: robot
        while True:
            self.clear_screen()
            saved = self.saved_camera_coords[tag_id]
            print(f"\n    === Tag {tag_id} ({step}/4) - FASE B (ROBOT) ===")
            print(f"    Cámara guardada: X={saved['x']:.1f}  Y={saved['y']:.1f}  Zmed={saved['z']:.1f} mm")
            print()
            print("    Ahora toca el centro del tag con el TCP (aunque lo tape).")
            print()

            robot = self.get_robot_pose()
            if robot:
                print(f"    🤖 Pose TCP: X={robot['x']:.1f}  Y={robot['y']:.1f}  Z={robot['z']:.1f} mm")
                print("    ENTER=Registrar | r=Refrescar | q=Cancelar")
            else:
                print("    ⚠️ Sin lectura automática. Use manual.")
                print("    m=Manual | r=Refrescar | q=Cancelar")

            choice = self.get_input("\n    Opción: ").strip().lower()
            if choice == 'q':
                return False
            if choice == 'r':
                continue
            if choice == 'm':
                try:
                    x = float(self.get_input("X (mm): "))
                    y = float(self.get_input("Y (mm): "))
                    z = float(self.get_input("Z (mm): "))
                    robot = {'x': x, 'y': y, 'z': z}
                except ValueError:
                    continue
            if choice == '' and robot is None:
                continue

            self.calibration_points[tag_id] = {
                'camera': [saved['x'], saved['y'], saved['z']],
                'robot':  [robot['x'], robot['y'], robot['z']],
            }
            print("\n    ✅ Registrado")
            time.sleep(1.0)
            return True

    def calculate_and_save(self):
        self.clear_screen()
        self.get_logger().info("Calculando calibración")

        n = len(self.calibration_points)
        if n < 3:
            self.get_logger().error("Necesitas mínimo 3 puntos")
            return

        # --- XY affine ---
        cam_xy = np.array([self.calibration_points[i]['camera'][:2] for i in sorted(self.calibration_points.keys())])  # (N,2)
        rob_xy = np.array([self.calibration_points[i]['robot'][:2]  for i in sorted(self.calibration_points.keys())])  # (N,2)

        A = np.zeros((2*n, 6))
        b = np.zeros(2*n)
        for i in range(n):
            cx, cy = cam_xy[i]
            rx, ry = rob_xy[i]
            A[2*i]   = [cx, cy, 1, 0, 0, 0]
            A[2*i+1] = [0, 0, 0, cx, cy, 1]
            b[2*i]   = rx
            b[2*i+1] = ry

        params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        self.affine_matrix = np.array([
            [params[0], params[1], params[2]],
            [params[3], params[4], params[5]],
        ])

        # --- Z planes ---
        cam_xyz = np.array([self.calibration_points[i]['camera'] for i in sorted(self.calibration_points.keys())])  # (N,3)
        rob_xyz = np.array([self.calibration_points[i]['robot']  for i in sorted(self.calibration_points.keys())])  # (N,3)

        z_plane_cam = self.fit_z_plane_xyz(cam_xyz, label="(CAMARA - superficie)")
        z_plane_robot = self.fit_z_plane_xyz(rob_xyz, label="(ROBOT - superficie)")

        # --- Save ---
        z_work_mm = float(np.mean(rob_xyz[:, 2]))  # fallback útil

        config = {
            'status': 'calibrated',
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'affine_matrix_2x3': self.affine_matrix.tolist(),
            'z_plane_cam': z_plane_cam,
            'z_plane_robot': z_plane_robot,
            'z_work_mm_fallback': z_work_mm,
            'calibration_points': {str(k): v for k, v in self.calibration_points.items()},
            'note': 'Use z_plane_cam+depth para altura del objeto. z_plane_robot para z en robot.'
        }

        path = os.path.expanduser('~/dobot_ws/calibration_config.json')
        with open(path, 'w') as f:
            json.dump(config, f, indent=2)

        self.get_logger().info(f"Guardado: {path}")
        self.get_logger().info(f"z_work_mm_fallback: {z_work_mm:.1f} mm")
        self.wait_enter("ENTER para continuar...")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
