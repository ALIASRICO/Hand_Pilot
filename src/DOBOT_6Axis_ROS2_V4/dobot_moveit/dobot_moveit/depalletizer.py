#!/usr/bin/env python3
"""
Despaletizador — modo prueba
- Orientación TCP completamente fija (ventosa siempre abajo, yaw fijo)
- Sin gripper — solo movimientos de brazo
- Bloquea hasta que MoveIt termine cada movimiento
"""

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import json
import os
import time
import threading

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
                              JointConstraint, CollisionObject, PlanningScene)
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import DOInstant as DOInstantSrv


def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    r  = math.radians(roll_deg)
    p  = math.radians(pitch_deg)
    y  = math.radians(yaw_deg)
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return {
        'x': sr*cp*cy - cr*sp*sy,
        'y': cr*sp*cy + sr*cp*sy,
        'z': cr*cp*sy - sr*sp*cy,
        'w': cr*cp*cy + sr*sp*sy,
    }


# ── Orientación fija del TCP ──────────────────────────────────────────────────
TCP_FIXED_QUAT = rpy_to_quaternion(180.0, 0.0, 0.0)

ORI_TOL_ROLL  = 0.05
ORI_TOL_PITCH = 0.05
ORI_TOL_YAW   = 0.03

# ── Home Position (joint-space, degrees → radians) ────────────────────────────
HOME_JOINT_DEG = [90, 0, -90, 0, 90, 0]  # arm folded forward/down, below ceiling
HOME_JOINT_RAD = [math.radians(d) for d in HOME_JOINT_DEG]

JUICE_TYPE_MANGO   = 0.0
JUICE_TYPE_MORA    = 1.0
JUICE_TYPE_UNKNOWN = -1.0
    

class DepalletizerNode(Node):
    def __init__(self):
        super().__init__('depalletizer')
        self.cb_group = ReentrantCallbackGroup()

        self.get_logger().info('=' * 60)
        self.get_logger().info('   🧃 DESPALETIZADOR — MODO PRUEBA 🧃')
        self.get_logger().info('   Orientación TCP fija | Sin gripper')
        self.get_logger().info('=' * 60)

        # ── Config ───────────────────────────────────────────
        self.calibration_file  = os.path.expanduser('~/dobot_ws/calibration_config.json')
        self.z_approach        = 0.15
        self.z_lift            = 0.20
        self.z_pick_extra      = 0.0
        self.speed_scaling     = 0.03
        self.place_position    = [0.5, -0.4, 0.15]

        _collision_cfg = os.path.expanduser('~/dobot_ws/collision_config.json')
        if os.path.exists(_collision_cfg):
            try:
                with open(_collision_cfg, 'r') as _f:
                    _cc = json.load(_f)
                _pp = _cc.get('place_position_m')
                if _pp and len(_pp) == 3:
                    self.place_position = [float(_pp[0]), float(_pp[1]), float(_pp[2])]
                    self.get_logger().info(
                        f'✅ place_position cargado de collision_config: '
                        f'({self.place_position[0]:.3f}, {self.place_position[1]:.3f}, {self.place_position[2]:.3f})m')
                else:
                    self.get_logger().warn('⚠️ collision_config.json sin place_position_m válido — usando default')
            except Exception as _e:
                self.get_logger().warn(f'⚠️ Error leyendo collision_config.json: {_e} — usando place_position default')
        else:
            self.get_logger().warn(
                f'⚠️ No existe collision_config.json — place_position default: {self.place_position}')

        self.top_offset_mm     = 0.0
        self.h_max_mm          = 400.0
        self.xy_verify_tol_mm  = 30.0

        # ── Estado ───────────────────────────────────────────
        self.detections        = []
        self.detection_lock    = threading.Lock()
        self.calib             = None
        self.M_xy              = None
        self.z_plane_cam       = None
        self.z_plane_robot     = None
        self.z_work_fallback_m = None
        self.is_busy           = False
        self.running           = True
        self.picked_count      = 0
        self.juice_counts      = {JUICE_TYPE_MANGO: 0, JUICE_TYPE_MORA: 0, JUICE_TYPE_UNKNOWN: 0}
        self.current_joints    = None
        self._last_tcp         = None
        self.z_layer           = None

        self.planning_group    = 'cr20_group'
        self.end_effector_link = 'Link6'

        self.load_calibration()

        # ── Suscriptores ─────────────────────────────────────
        self.create_subscription(
            Float32MultiArray, '/detections/kartonger',
            self.detections_callback, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            JointState, '/joint_states',
            self.joint_states_callback, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            ToolVectorActual, '/dobot_msgs_v4/msg/ToolVectorActual',
            self._tcp_callback, 10,
            callback_group=self.cb_group)

        # ── MoveGroup ────────────────────────────────────────
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.cb_group)

        self.get_logger().info('Esperando MoveGroup...')
        if self.move_group_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().info('✅ MoveGroup conectado')
        else:
            self.get_logger().error('❌ MoveGroup no disponible — verifica que está corriendo')

        # DO1 — control vacío/ventosa
        self.do_instant_client = self.create_client(
            DOInstantSrv, 'dobot_bringup_ros2/srv/DOInstant',
            callback_group=self.cb_group)

        # ── Pallet collision slabs client ─────────────────────────
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene',
            callback_group=self.cb_group)

        # ── Pallet config (loaded from collision_config.json) ─────
        self._pallet_config = None
        self._pallet_z_surface_robot_mm = None
        self._box_height_mm = None
        self._layer_slabs = []           # list of slab names currently active
        self._current_n_layers = 0

        _coll_cfg_path = os.path.expanduser('~/dobot_ws/collision_config.json')
        if os.path.exists(_coll_cfg_path):
            try:
                with open(_coll_cfg_path, 'r') as _f:
                    _cc = json.load(_f)
                _objs = _cc.get('objects', {})
                _pallet_obj = _objs.get('pallet')
                if _pallet_obj and _pallet_obj.get('type') == 'pallet':
                    self._pallet_config = _pallet_obj
                    # Compute pallet surface Z in robot frame (avg of corner Z values)
                    _p1 = _pallet_obj.get('corner1_mm', [0, 0, 0])
                    _p2 = _pallet_obj.get('corner2_mm', [0, 0, 0])
                    self._pallet_z_surface_robot_mm = (_p1[2] + _p2[2]) / 2.0
                    self.get_logger().info(
                        f'✅ Pallet calibrado encontrado: superficie Z={self._pallet_z_surface_robot_mm:.1f}mm')
                else:
                    self.get_logger().info('ℹ️ No hay pallet calibrado en collision_config — sin losas dinámicas')
            except Exception as _e:
                self.get_logger().warn(f'⚠️ Error leyendo pallet config: {_e}')

        # ── UI thread ────────────────────────────────────────
        self.ui_thread = threading.Thread(target=self.user_interface)
        self.ui_thread.daemon = True
        self.ui_thread.start()

    # ─────────────────────────────────────────────────────────
    # Calibración
    # ─────────────────────────────────────────────────────────
    def load_calibration(self):
        if not os.path.exists(self.calibration_file):
            self.get_logger().error(f'❌ No existe calibración: {self.calibration_file}')
            return
        try:
            with open(self.calibration_file, 'r') as f:
                self.calib = json.load(f)
            self.M_xy          = np.array(self.calib['affine_matrix_2x3'])
            self.z_plane_cam   = self.calib.get('z_plane_cam', None)
            self.z_plane_robot = self.calib.get('z_plane_robot', None)
            zfb = self.calib.get('z_work_mm_fallback', None)
            self.z_work_fallback_m = float(zfb) / 1000.0 if zfb is not None else None
            self.get_logger().info('✅ Calibración cargada')
            self.get_logger().info(f'   z_plane_cam:   {"OK" if self.z_plane_cam   else "NO"}')
            self.get_logger().info(f'   z_plane_robot: {"OK" if self.z_plane_robot else "NO"}')
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando calibración: {e}')

    def eval_plane(self, plane, x_mm, y_mm):
        return plane['a'] * x_mm + plane['b'] * y_mm + plane['c']

    def camxy_to_robotxy_mm(self, cam_x_mm, cam_y_mm):
        M  = self.M_xy
        rx = M[0,0]*cam_x_mm + M[0,1]*cam_y_mm + M[0,2]
        ry = M[1,0]*cam_x_mm + M[1,1]*cam_y_mm + M[1,2]
        return rx, ry

    def compute_grasp_z_robot_mm(self, cam_x_mm, cam_y_mm, cam_z_mm,
                                  robot_x_mm, robot_y_mm):
        if self.z_plane_cam is None or self.z_plane_robot is None:
            if self.z_work_fallback_m is None:
                return None, None, None, None
            fb = self.z_work_fallback_m * 1000.0
            return fb, 0.0, fb, None
        z_surf_cam   = self.eval_plane(self.z_plane_cam, cam_x_mm, cam_y_mm)
        h            = max(0.0, min(z_surf_cam - cam_z_mm, self.h_max_mm))
        z_surf_robot = self.eval_plane(self.z_plane_robot, robot_x_mm, robot_y_mm)
        z_grasp      = z_surf_robot + max(0.0, h - self.top_offset_mm)
        return z_grasp, h, z_surf_robot, z_surf_cam

    # ─────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────
    def detections_callback(self, msg):
        data = msg.data
        with self.detection_lock:
            self.detections.clear()
            for i in range(0, len(data), 8):
                if i + 7 >= len(data):
                    continue
                det_id   = int(data[i])
                cam_x_mm = float(data[i+1])
                cam_y_mm = float(data[i+2])
                cam_z_mm = float(data[i+3])
                conf     = float(data[i+4])
                det = {'id': det_id, 'confidence': conf,
                       'cam_x_mm': cam_x_mm, 'cam_y_mm': cam_y_mm, 'cam_z_mm': cam_z_mm}
                det['juice_type'] = float(data[i+7])
                if self.M_xy is not None:
                    rx_mm, ry_mm = self.camxy_to_robotxy_mm(cam_x_mm, cam_y_mm)
                    z_grasp_mm, h_mm, z_surf_robot_mm, z_surf_cam_mm = \
                        self.compute_grasp_z_robot_mm(cam_x_mm, cam_y_mm, cam_z_mm, rx_mm, ry_mm)
                    det['robot_x']       = rx_mm / 1000.0
                    det['robot_y']       = ry_mm / 1000.0
                    det['robot_z_grasp'] = (z_grasp_mm / 1000.0) if z_grasp_mm is not None else None
                    det['h_obj_mm']      = h_mm
                self.detections.append(det)

    def joint_states_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])
            self.get_logger().debug(
                f'🔍 JS callback: joints={["{:.2f}".format(j) for j in msg.position[:6]]}')

    def _tcp_callback(self, msg):
        old_tcp = self._last_tcp
        self._last_tcp = {'x': msg.x, 'y': msg.y, 'z': msg.z}
        if old_tcp is not None:
            dx = msg.x - old_tcp['x']
            dy = msg.y - old_tcp['y']
            dz = msg.z - old_tcp['z']
            self.get_logger().debug(
                f'🔍 TCP update: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})mm '
                f'| delta=({dx:.1f}, {dy:.1f}, {dz:.1f})mm')

    # ─────────────────────────────────────────────────────────
    # Pre-flight: validación de joint states
    # ─────────────────────────────────────────────────────────
    def _wait_for_valid_joint_states(self, timeout_sec=15.0):
        """Espera hasta que /joint_states reporte posiciones no-cero."""
        self._js_valid = False
        _valid_pos = [None]

        def _js_check_cb(msg):
            if len(msg.position) >= 6:
                if not all(abs(p) < 1e-6 for p in msg.position):
                    self._js_valid = True
                    _valid_pos[0] = list(msg.position[:6])

        sub = self.create_subscription(
            JointState, '/joint_states',
            _js_check_cb, 10,
            callback_group=self.cb_group)

        t0 = time.time()
        last_log_time = 0.0
        while time.time() - t0 < timeout_sec:
            time.sleep(0.1)
            if self._js_valid:
                pos_str = ', '.join(f'{p:.4f}' for p in _valid_pos[0])
                self.get_logger().info(f'✅ Joint states válidos recibidos: [{pos_str}]')
                self.destroy_subscription(sub)
                return True
            now = time.time()
            if now - last_log_time >= 2.0:
                elapsed = now - t0
                self.get_logger().warn(
                    f'⏳ Esperando joint states válidos... ({elapsed:.1f}s/{timeout_sec:.1f}s)')
                last_log_time = now

        self.destroy_subscription(sub)
        return False

    def run_preflight(self):
        """Run pre-flight checks. Must be called AFTER executor is spinning."""
        if not self._wait_for_valid_joint_states(timeout_sec=15.0):
            self.get_logger().error('⚠️ Timeout esperando joint states válidos — continuar con precaución')

    # ─────────────────────────────────────────────────────────
    # MoveIt planning scene helpers
    # ─────────────────────────────────────────────────────────
    def _create_collision_box(self, name, dims_m, center_m, add=True):
        """Add or remove a box collision object in the MoveIt planning scene."""
        co = CollisionObject()
        co.id = name
        co.header.frame_id = 'base_link'

        if add:
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [float(dims_m[0]), float(dims_m[1]), float(dims_m[2])]

            pose = Pose()
            pose.position.x = float(center_m[0])
            pose.position.y = float(center_m[1])
            pose.position.z = float(center_m[2])
            pose.orientation.w = 1.0

            co.primitives.append(box)
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD
        else:
            co.operation = CollisionObject.REMOVE

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)

        if not self._apply_scene_client.service_is_ready():
            self.get_logger().warn(f'⚠️ /apply_planning_scene no disponible — skip {name}')
            return

        req = ApplyPlanningScene.Request()
        req.scene = ps
        future = self._apply_scene_client.call_async(req)
        _ev = threading.Event()
        future.add_done_callback(lambda f: _ev.set())
        if not _ev.wait(timeout=5.0):
            self.get_logger().warn(f'⚠️ Timeout al aplicar escena para {name}')

    def _remove_collision_object(self, name):
        """Remove a collision object from MoveIt planning scene."""
        self._create_collision_box(name, [0, 0, 0], [0, 0, 0], add=False)

    def _initialize_pallet_slabs(self, box_height_mm):
        """Initialize layer collision slabs based on current YOLO detections."""
        from dobot_moveit.collision_calibrator import (
            estimate_n_layers, compute_layer_slabs
        )

        if self._pallet_config is None:
            return

        self._box_height_mm = box_height_mm
        p1 = self._pallet_config.get('corner1_mm', [0, 0, 0])
        p2 = self._pallet_config.get('corner2_mm', [0, 0, 0])
        pallet_dx_mm = abs(p2[0] - p1[0])
        pallet_dy_mm = abs(p2[1] - p1[1])
        pallet_cx_mm = (p1[0] + p2[0]) / 2.0
        pallet_cy_mm = (p1[1] + p2[1]) / 2.0

        # Wait for first stable Z detection
        self.get_logger().info('⏳ Esperando primera detección YOLO para estimar capas...')
        z_top_robot_mm = None
        t0 = time.time()
        while time.time() - t0 < 15.0:
            with self.detection_lock:
                if self.detections:
                    z_values = [d.get('robot_z_grasp') for d in self.detections
                                if d.get('robot_z_grasp') is not None]
                    if z_values:
                        # Use maximum Z (highest in robot frame = top layer)
                        z_top_robot_mm = max(z_values) * 1000.0  # m→mm
                        break
            time.sleep(0.2)

        if z_top_robot_mm is None:
            self.get_logger().warn('⚠️ Sin detecciones YOLO para estimar capas — iniciando sin losas')
            return

        n_layers = estimate_n_layers(
            z_top_robot_mm,
            self._pallet_z_surface_robot_mm,
            box_height_mm
        )

        if n_layers <= 0:
            self.get_logger().warn(f'⚠️ N_layers={n_layers} (inválido) — sin losas')
            return

        # Ask user confirmation
        print(f'\n    📦 Se detectaron {n_layers} capa(s) en el pallet.')
        confirm = input(f'    ¿Es correcto? (s/N): ').strip().lower()
        if confirm not in ('s', 'si', 'sí', 'y', 'yes'):
            new_n = input(f'    Ingresa el número de capas manualmente: ').strip()
            try:
                n_layers = max(1, int(new_n))
            except ValueError:
                self.get_logger().warn('⚠️ Entrada inválida — usando valor detectado')

        # Create N-1 slabs (all layers below the active top)
        slabs = compute_layer_slabs(
            pallet_surface_z_mm=self._pallet_z_surface_robot_mm,
            box_height_mm=box_height_mm,
            n_layers=n_layers,
            pallet_dx_mm=pallet_dx_mm,
            pallet_dy_mm=pallet_dy_mm,
            pallet_cx_mm=pallet_cx_mm,
            pallet_cy_mm=pallet_cy_mm,
            safety_margin_mm=5.0,
        )

        self._current_n_layers = n_layers
        self._layer_slabs = []

        for slab in slabs:
            self._create_collision_box(slab['name'], slab['dims_m'], slab['center_m'], add=True)
            self._layer_slabs.append(slab['name'])
            self.get_logger().info(
                f'🧱 Losa añadida: {slab["name"]} | '
                f'Z={slab["center_m"][2]*1000:.0f}mm | '
                f'dims=({slab["dims_m"][0]*1000:.0f}×{slab["dims_m"][1]*1000:.0f}×{slab["dims_m"][2]*1000:.0f}mm)')

        if slabs:
            self.get_logger().info(
                f'✅ {len(slabs)} losas de colisión creadas para {n_layers} capas de pallet')
        else:
            self.get_logger().info(
                f'ℹ️ Pallet con 1 capa — sin losas necesarias (el brazo trabaja en la capa superior)')

    def _check_and_remove_layer(self, z_before_pick_robot_mm):
        """After a successful pick, check if the top layer is now empty.

        If the stable Z of current detections dropped by more than 0.7 * box_height,
        remove the topmost collision slab (the layer we just cleared).

        Args:
            z_before_pick_robot_mm: Z of the pick target BEFORE the pick (in mm, robot frame).
        """
        from dobot_moveit.collision_calibrator import should_remove_top_layer

        if self._box_height_mm is None or not self._layer_slabs:
            return  # No slab management configured

        # Get current stable Z from YOLO detections
        z_samples = []
        t0 = time.time()
        while len(z_samples) < 20 and time.time() - t0 < 3.0:
            with self.detection_lock:
                for d in self.detections:
                    z = d.get('robot_z_grasp')
                    if z is not None and d.get('confidence', 0) >= 0.75:
                        z_samples.append(z * 1000.0)  # m → mm
            time.sleep(0.1)

        if not z_samples:
            # No detections — pallet may be empty
            if self._layer_slabs:
                self.get_logger().info(
                    'Sin detecciones tras pick — pallet posiblemente vacio')
            return

        z_current_robot_mm = sorted(z_samples)[len(z_samples) // 2]  # median

        if should_remove_top_layer(z_current_robot_mm, z_before_pick_robot_mm, self._box_height_mm):
            if self._layer_slabs:
                slab_to_remove = self._layer_slabs[-1]  # Remove topmost slab
                self._remove_collision_object(slab_to_remove)
                self._layer_slabs.pop()
                self._current_n_layers = max(0, self._current_n_layers - 1)
                self.z_layer = None  # Force Z recalculation for new layer
                self.get_logger().info(
                    f'Capa vaciada — losa "{slab_to_remove}" eliminada del planning scene '
                    f'| Capas restantes: {self._current_n_layers}')

        # Check if all detections gone
        with self.detection_lock:
            if not self.detections and not self._layer_slabs:
                self.get_logger().info(
                    'Pallet vacio — todas las capas completadas')

    # ─────────────────────────────────────────────────────────
    # MoveIt — orientación 100% fija
    # ─────────────────────────────────────────────────────────
    def move_to_pose(self, x, y, z, label='', max_retries=3):
        for attempt in range(1, max_retries + 1):
            if attempt > 1:
                self.get_logger().warn(f'🔄 Reintento {attempt}/{max_retries}...')
                time.sleep(1.0)
        tag = f'[{label}] ' if label else ''
        self.get_logger().info(f'📍 {tag}Moviendo a: ({x:.3f}, {y:.3f}, {z:.3f})m')

        if self.current_joints is not None:
            joints_str = ', '.join(f'{j:.3f}' for j in self.current_joints)
            self.get_logger().debug(f'🔍 START_STATE preparation: joints=[{joints_str}]')
        else:
            self.get_logger().warn('⚠️ DEBUG: current_joints es None antes de planificar!')

        goal = MoveGroup.Goal()
        goal.request.group_name                      = self.planning_group
        goal.request.num_planning_attempts           = 50
        goal.request.allowed_planning_time           = 30.0
        goal.request.max_velocity_scaling_factor     = self.speed_scaling
        goal.request.max_acceleration_scaling_factor = self.speed_scaling

        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name       = self.end_effector_link
        sphere             = SolidPrimitive()
        sphere.type        = SolidPrimitive.SPHERE
        sphere.dimensions  = [0.01]
        pc.constraint_region.primitives.append(sphere)
        rp               = Pose()
        rp.position.x    = x
        rp.position.y    = y
        rp.position.z    = z
        rp.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(rp)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id           = 'base_link'
        oc.link_name                 = self.end_effector_link
        oc.orientation.x             = TCP_FIXED_QUAT['x']
        oc.orientation.y             = TCP_FIXED_QUAT['y']
        oc.orientation.z             = TCP_FIXED_QUAT['z']
        oc.orientation.w             = TCP_FIXED_QUAT['w']
        oc.absolute_x_axis_tolerance = ORI_TOL_ROLL
        oc.absolute_y_axis_tolerance = ORI_TOL_PITCH
        oc.absolute_z_axis_tolerance = ORI_TOL_YAW
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(constraints)
        time.sleep(0.5)
        self.get_logger().info(f'   ⏳ Planeando...')

        future = self.move_group_client.send_goal_async(goal)
        _accept_event = threading.Event()
        future.add_done_callback(lambda f: _accept_event.set())
        if not _accept_event.wait(timeout=15.0):
            self.get_logger().error('❌ Timeout en aceptación del goal')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rechazado por MoveGroup')
            return False

        self.get_logger().info('   ⏳ Ejecutando trayectoria...')
        result_future = goal_handle.get_result_async()
        _result_event = threading.Event()
        result_future.add_done_callback(lambda f: _result_event.set())
        if not _result_event.wait(timeout=120.0):
            self.get_logger().error('❌ Timeout en ejecución')
            return False

        result = result_future.result().result
        code   = result.error_code.val

        if code == MoveItErrorCodes.SUCCESS:
            tcp = self._last_tcp
            if tcp:
                self.get_logger().info(
                    f'   ✅ Llegó → TCP real: ({tcp["x"]:.1f}, {tcp["y"]:.1f}, {tcp["z"]:.1f})mm')
            else:
                self.get_logger().info('   ✅ Movimiento completado')
            return True

        error_names = {
            -1:  'FAILURE',
            -2:  'PLANNING_FAILED (IK no encontrada)',
            -3:  'INVALID_MOTION_PLAN',
            -4:  'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
            -5:  'CONTROL_FAILED',
            -6:  'UNABLE_TO_AQUIRE_SENSOR_DATA',
            -7:  'TIMED_OUT',
            -8:  'PREEMPTED',
            -9:  'START_STATE_IN_COLLISION',
            -10: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
            -11: 'GOAL_IN_COLLISION',
            -12: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
            -13: 'GOAL_CONSTRAINTS_VIOLATED',
            -14: 'INVALID_GROUP_NAME',
            -15: 'INVALID_GOAL_CONSTRAINTS',
            -16: 'INVALID_ROBOT_STATE',
            -17: 'INVALID_LINK_NAME',
            -18: 'INVALID_OBJECT_NAME',
        }
        desc = error_names.get(code, f'código {code}')
        self.get_logger().error(f'❌ MoveIt falló: {desc}')

        return False

    # ─────────────────────────────────────────────────────────
    # Home position — joint-space goal
    # ─────────────────────────────────────────────────────────
    def go_home(self):
        self.get_logger().info('🏠 Moviendo a posición Home...')
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        goal = MoveGroup.Goal()
        goal.request.group_name                      = self.planning_group
        goal.request.num_planning_attempts           = 20
        goal.request.allowed_planning_time           = 10.0
        goal.request.max_velocity_scaling_factor     = self.speed_scaling
        goal.request.max_acceleration_scaling_factor = self.speed_scaling

        constraints = Constraints()
        for name, position in zip(joint_names, HOME_JOINT_RAD):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info('   ⏳ Planeando hacia Home...')
        future = self.move_group_client.send_goal_async(goal)
        _accept_event = threading.Event()
        future.add_done_callback(lambda f: _accept_event.set())
        if not _accept_event.wait(timeout=15.0):
            self.get_logger().error('❌ Timeout en aceptación del goal (Home)')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Home rechazado por MoveGroup')
            return False

        self.get_logger().info('   ⏳ Ejecutando trayectoria Home...')
        result_future = goal_handle.get_result_async()
        _result_event = threading.Event()
        result_future.add_done_callback(lambda f: _result_event.set())
        if not _result_event.wait(timeout=120.0):
            self.get_logger().error('❌ Timeout en ejecución Home')
            return False

        result = result_future.result().result
        code   = result.error_code.val

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('   ✅ Robot en posición Home')
            return True

        error_names = {
            -1: 'FAILURE', -2: 'PLANNING_FAILED', -9: 'START_STATE_IN_COLLISION',
            -10: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
        }
        desc = error_names.get(code, f'código {code}')
        self.get_logger().error(f'❌ MoveIt falló (Home): {desc}')
        return False

    # ─────────────────────────────────────────────────────────
    # DO1 control
    # ─────────────────────────────────────────────────────────
    def set_do1(self, status: int):
        """Controla salida digital DO1 (vacío/ventosa). status: 1=ON, 0=OFF."""
        label = 'ON 🟢' if status == 1 else 'OFF 🔴'
        self.get_logger().info(f'   💨 DO1 → {label}')
        if not self.do_instant_client.service_is_ready():
            self.get_logger().warn('⚠️ DOInstant service no disponible — continuando sin DO1')
            return
        req = DOInstantSrv.Request()
        req.index = 1
        req.status = status
        future = self.do_instant_client.call_async(req)
        _do1_event = threading.Event()
        future.add_done_callback(lambda f: _do1_event.set())
        if not _do1_event.wait(timeout=3.0):
            self.get_logger().warn('⚠️ DOInstant timeout')

    # ─────────────────────────────────────────────────────────
    # Verificación XY
    # ─────────────────────────────────────────────────────────
    def verify_xy(self, target_x_m, target_y_m):
        if self._last_tcp is None:
            self.get_logger().warn('⚠️ Sin lectura TCP, omitiendo verificación')
            return True
        target_x_mm = target_x_m * 1000.0
        target_y_mm = target_y_m * 1000.0
        err_x = abs(self._last_tcp['x'] - target_x_mm)
        err_y = abs(self._last_tcp['y'] - target_y_mm)
        self.get_logger().info(
            f'   📐 Error XY → target=({target_x_mm:.1f},{target_y_mm:.1f})mm '
            f'actual=({self._last_tcp["x"]:.1f},{self._last_tcp["y"]:.1f})mm '
            f'dx={err_x:.1f}mm dy={err_y:.1f}mm (tol={self.xy_verify_tol_mm}mm)')
        if err_x > self.xy_verify_tol_mm or err_y > self.xy_verify_tol_mm:
            self.get_logger().error('❌ Error XY fuera de tolerancia — abortando')
            return False
        self.get_logger().info('   ✅ XY dentro de tolerancia')
        return True

    # ─────────────────────────────────────────────────────────
    # Z estabilizado
    # ─────────────────────────────────────────────────────────
    def get_stable_z_grasp(self, target_id, timeout_sec=3.0, n_frames=50):
        """Toma la Z más alta de TODAS las detecciones (misma capa)."""
        z_samples = []
        t0 = time.time()
        while len(z_samples) < n_frames and time.time() - t0 < timeout_sec:
            with self.detection_lock:
                if self.detections:
                    for d in self.detections:
                        z = d.get('robot_z_grasp')
                        if z is not None and d.get('confidence', 0) >= 0.80:
                            z_samples.append(z)
            time.sleep(0.05)
        
        if not z_samples:
            return None
        
        # Eliminar outliers — quedarse con el rango intercuartil
        z_arr = sorted(z_samples)
        result = z_arr[len(z_arr)//2]
        self.get_logger().info(
            f'📊 Z más alta seleccionada: {result:.4f}m ({len(z_samples)} muestras, '
            f'rango: [{min(z_samples):.4f}, {max(z_samples):.4f}])')
        return result

    # ─────────────────────────────────────────────────────────
    # Ciclo pick & place
    # ─────────────────────────────────────────────────────────
    def pick_and_place_cycle(self):
        if not self._wait_for_valid_joint_states(timeout_sec=5.0):
            self.get_logger().error('❌ Joint states aún en ceros — ¿está dobot_bringup corriendo?')
            return

        if self.is_busy or self.M_xy is None:
            return False

        with self.detection_lock:
            if not self.detections:
                self.get_logger().warn('⚠️ Sin detecciones')
                return False
            Z_LAYER_TOL_MM = 5.0
            sorted_dets = sorted(self.detections, key=lambda d: d.get('cam_z_mm', 9999.0))
            best = sorted_dets[0]
            best_z = best.get('cam_z_mm', 0.0)
            same_layer = [d for d in sorted_dets
                          if abs(d.get('cam_z_mm', 0.0) - best_z) <= Z_LAYER_TOL_MM]
            if len(same_layer) > 1:
                same_layer.sort(key=lambda d: math.sqrt(
                    (d.get('robot_x', 0.0) * 1000) ** 2 +
                    (d.get('robot_y', 0.0) * 1000) ** 2))
            target = same_layer[0].copy()

        if target.get('robot_z_grasp') is None:
            self.get_logger().error('❌ Z de agarre no calculada')
            return False

        self.is_busy = True
        try:
            x       = target['robot_x']
            y       = target['robot_y']
            if self.z_layer is not None:
                z_grasp = self.z_layer + self.z_pick_extra
                self.get_logger().info(f'📏 Usando Z de capa guardado: {self.z_layer:.4f}m')
            else:
                z_stable = self.get_stable_z_grasp(None, timeout_sec=3.0, n_frames=50)
                if z_stable is not None:
                 self.z_layer = z_stable
                z_grasp = (z_stable if z_stable is not None else target['robot_z_grasp']) + self.z_pick_extra
                self.get_logger().info(f'📏 Z de capa calculado: {z_grasp:.4f}m')
            
            self.get_logger().info(f'📊 z_grasp final usado: {z_grasp:.4f}m')
            # Capture pre-pick Z for layer depletion detection
            _z_before_pick_robot_mm = z_grasp * 1000.0 if z_grasp is not None else None
            self.get_logger().info('─' * 50)
            _juice_name = {JUICE_TYPE_MANGO: 'MANGO', JUICE_TYPE_MORA: 'MORA',
                           JUICE_TYPE_UNKNOWN: 'DESCONOCIDO'}.get(
                               target.get('juice_type', JUICE_TYPE_UNKNOWN), '???')
            self.get_logger().info(
                f"🎯 Target #{target['id']} [{_juice_name}] | "
                f"XY=({x*1000:.0f},{y*1000:.0f})mm | "
                f"h={target.get('h_obj_mm',0):.1f}mm | "
                f"z_grasp={z_grasp:.3f}m")
            self.get_logger().info('─' * 50)
            # PICK
            self.get_logger().info('   Iniciando ciclo pick & place...')
            self.speed_scaling = 0.1
            z_approach_dynamic = 0.20  # 20cm fijo sobre z_grasp
            if not self.move_to_pose(x, y, z_grasp + z_approach_dynamic, label='APPROACH'):
                return False
            if not self.verify_xy(x, y):
                return False
            if not self.move_to_pose(x, y, z_grasp, label='DESCENSO'):
                return False
            time.sleep(2)
            self.set_do1(1)
            time.sleep(1)
            self.speed_scaling = 0.1
            if not self.move_to_pose(x, y, z_grasp + z_approach_dynamic, label='LIFT'):
                return False
            # PLACE
            px, py, pz = self.place_position
            z_place_approach = 0.20
            if not self.move_to_pose(px, py, pz + z_place_approach, label='PLACE APPROACH'):
                return False
            if not self.move_to_pose(px, py, pz, label='PLACE'):
                return False
            time.sleep(1)
            self.set_do1(0)
            time.sleep(1)
            self.get_logger().info('   En posición de depósito')
            if not self.move_to_pose(px, py, pz + z_place_approach, label='PLACE LIFT'):
                return False
            self.speed_scaling = 0.1

            self.picked_count += 1
            _jt = target.get('juice_type', JUICE_TYPE_UNKNOWN)
            self.juice_counts[_jt] = self.juice_counts.get(_jt, 0) + 1
            _jname = {JUICE_TYPE_MANGO: 'MANGO', JUICE_TYPE_MORA: 'MORA',
                      JUICE_TYPE_UNKNOWN: 'DESCONOCIDO'}.get(_jt, '???')
            self.get_logger().info(
                f'🎉 Ciclo completado. Total: {self.picked_count} | 🧃 Tipo: {_jname}')

            # Check if the top pallet layer has been depleted
            if _z_before_pick_robot_mm is not None and self._pallet_config is not None:
                self._check_and_remove_layer(_z_before_pick_robot_mm)

        except Exception as e:
            self.get_logger().error(f'❌ Excepción: {e}')
            return False
        finally:
            self.is_busy = False

        return True

    # ─────────────────────────────────────────────────────────
    # UI
    # ─────────────────────────────────────────────────────────
    def user_interface(self):
        time.sleep(2.0)

        # ── Pallet layer height prompt (if pallet calibrated) ──
        if self._pallet_config is not None:
            print()
            print('    ╔═══════════════════════════════════════════════╗')
            print('    ║  CONFIGURACIÓN DE CAPAS DE PALLET            ║')
            print('    ╚═══════════════════════════════════════════════╝')
            print()
            while True:
                try:
                    h_str = input('    ¿Cuánto mide de alto la caja en mm? (ej: 100): ').strip()
                    h = float(h_str)
                    if h <= 0:
                        print('    ⚠️ El valor debe ser mayor que 0.')
                        continue
                    break
                except ValueError:
                    print('    ⚠️ Ingresa un número válido.')
            self.get_logger().info(f'📦 Altura de caja configurada: {h:.1f}mm')
            # Initialize slabs in a background thread to not block UI
            threading.Thread(target=self._initialize_pallet_slabs, args=(h,),
                             daemon=True).start()

        self.print_menu()
        while self.running:
            try:
                choice = input('\n    Opción: ').strip().lower()
                if choice == '1':
                    self.show_detections()
                elif choice == '2':
                    self.pick_and_place_cycle()
                elif choice == 'h':
                    self.go_home()
                elif choice == 'q':
                    self.running = False
                    rclpy.shutdown()
                else:
                    self.print_menu()
            except Exception:
                break

    def print_menu(self):
        print('\n    ╔═══════════════════════════════════╗')
        print('    ║   🧃 DESPALETIZADOR — PRUEBA 🧃   ║')
        print('    ╠═══════════════════════════════════╣')
        print('    ║  1 = Ver detecciones              ║')
        print('    ║  2 = Pick & Place                 ║')
        print('    ║  H = Ir a Home                    ║')
        print('    ║  Q = Salir                        ║')
        print('    ╚═══════════════════════════════════╝')
        ok = self.M_xy is not None
        tcp = self._last_tcp
        tcp_str = f'TCP:({tcp["x"]:.0f},{tcp["y"]:.0f},{tcp["z"]:.0f})' if tcp else 'TCP:---'
        print(f"    Calib:{'✅' if ok else '❌'} | {tcp_str} | Picks:{self.picked_count}")

    def show_detections(self):
        with self.detection_lock:
            det_copy = [d.copy() for d in self.detections if 'robot_x' in d]
        
        print(f'\n    Detecciones: {len(det_copy)}')
        if not det_copy:
            return
        
        print('    ⏳ Recolectando muestras para Z de capa...')
        z_layer = self.get_stable_z_grasp(None, timeout_sec=3.0, n_frames=50)
        if z_layer is not None:
            self.z_layer = z_layer
            print(f'    📏 Z de capa (común): {z_layer:.3f}m  ← GUARDADO')
        elif self.z_layer is not None:
            z_layer = self.z_layer
            print(f'    📏 Z de capa (guardado): {z_layer:.3f}m')

        for d in det_copy:
            _jn = {JUICE_TYPE_MANGO: 'MANGO', JUICE_TYPE_MORA: 'MORA',
                JUICE_TYPE_UNKNOWN: '???'}.get(d.get('juice_type', JUICE_TYPE_UNKNOWN), '???')
            z_instant = d.get('robot_z_grasp', 0)
            z_show = z_layer if z_layer is not None else z_instant
            print(
                f"    🧃 #{d['id']} [{_jn}] conf={d['confidence']:.2f} | "
                f"R=({d['robot_x']*1000:.0f},{d['robot_y']*1000:.0f})mm | "
                f"h={d.get('h_obj_mm',0):.1f}mm | "
                f"z_instant={z_instant:.3f}m | "
                f"z_capa={z_show:.3f}m ✅")

    def destroy_node(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('  📊 INFORME DE JUGOS RECOGIDOS')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  🥭 MANGO:      {self.juice_counts.get(JUICE_TYPE_MANGO, 0)}')
        self.get_logger().info(f'  🫐 MORA:        {self.juice_counts.get(JUICE_TYPE_MORA, 0)}')
        self.get_logger().info(f'  ❓ DESCONOCIDO: {self.juice_counts.get(JUICE_TYPE_UNKNOWN, 0)}')
        self.get_logger().info(f'  📦 TOTAL:       {self.picked_count}')
        self.get_logger().info('=' * 50)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node     = DepalletizerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)

        # Start spinning in background so callbacks fire before preflight
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # Now run pre-flight with executor active
        node.run_preflight()

        # Block until shutdown
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shut down (e.g. user pressed Q in UI)


if __name__ == '__main__':
    main()
