"""
Nodo de teleoperación VR para Dobot CR20 — Simulación directa.

Arquitectura sin ros2_control:
  - Publica directamente a /joint_states (RSP actualiza TFs → RViz mueve robot)
  - Timer a 20Hz NUNCA bloquea
  - IK completamente async (fire-and-forget + callback)
  - EMA smoothing en soluciones IK
  - Orientación libre (no fuerza orientación del efector)
  - FK de captura async

Máquina de estados:
  IDLE --[GRIP]--> CAPTURING --> TELEOP --[GRIP]--> PAUSED --[THUMBS_UP]--> IDLE
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest

from vr_teleop.coordinate_mapper import CoordinateMapper, VRPose, RobotPose
from vr_teleop.safety_monitor import SafetyMonitor

IDLE = 'IDLE'
CAPTURING = 'CAPTURING'
TELEOP = 'TELEOP'
PAUSED = 'PAUSED'

HOME_JOINT_DEG = [90.0, 0.0, -90.0, 0.0, 90.0, 0.0]
HOME_JOINT_RAD = [math.radians(d) for d in HOME_JOINT_DEG]
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
PLANNING_GROUP = 'cr20_group'


class VRTeleopNode(Node):
    def __init__(self):
        super().__init__('vr_teleop_node')

        self._sub_cb = ReentrantCallbackGroup()
        self._timer_cb = MutuallyExclusiveCallbackGroup()
        self._srv_cb = ReentrantCallbackGroup()

        # Parámetros
        self._declare_params()
        self.dry_run = self.get_parameter('dry_run').value
        rate_hz = self.get_parameter('servo_rate_hz').value
        self._ema_alpha = self.get_parameter('ema_alpha').value

        self.get_logger().info(f'VR Teleop — rate={rate_hz}Hz, ema={self._ema_alpha}')

        # Coordinate Mapper
        self.mapper = CoordinateMapper(
            scale_x=self.get_parameter('position_scale_x').value,
            scale_y=self.get_parameter('position_scale_y').value,
            scale_z=self.get_parameter('position_scale_z').value,
            vr_axis_to_robot_x=self.get_parameter('vr_axis_to_robot_x').value,
            vr_axis_to_robot_y=self.get_parameter('vr_axis_to_robot_y').value,
            vr_axis_to_robot_z=self.get_parameter('vr_axis_to_robot_z').value,
            flip_x=self.get_parameter('flip_x').value,
            flip_y=self.get_parameter('flip_y').value,
            flip_z=self.get_parameter('flip_z').value,
            map_orientation=False,
        )

        # Safety Monitor
        self.safety = SafetyMonitor(
            workspace_x_min=self.get_parameter('workspace_x_min').value,
            workspace_x_max=self.get_parameter('workspace_x_max').value,
            workspace_y_min=self.get_parameter('workspace_y_min').value,
            workspace_y_max=self.get_parameter('workspace_y_max').value,
            workspace_z_min=self.get_parameter('workspace_z_min').value,
            workspace_z_max=self.get_parameter('workspace_z_max').value,
            max_step_mm=self.get_parameter('max_step_mm').value,
        )

        # Estado
        self.state = IDLE
        self.latest_vr_pose: VRPose | None = None
        self.last_sent_pose = RobotPose()
        self._prev_gesture = '---'

        # Buffer IK async
        self._smoothed_joints: list[float] = list(HOME_JOINT_RAD)
        self._last_ik_solution: list[float] = list(HOME_JOINT_RAD)
        self._ik_pending = False
        self._pending_target: RobotPose | None = None
        # Orientación capturada del TCP (se obtiene via FK al hacer GRIP)
        self._captured_orientation: Quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Captura async
        self._capture_vr_pose: VRPose | None = None
        self._capture_joints: list[float] | None = None

        # Suscripciones
        self.create_subscription(
            PoseStamped, '/hand_right/pose',
            self._on_hand_pose, 10, callback_group=self._sub_cb)
        self.create_subscription(
            String, '/hand_right/gesture',
            self._on_gesture, 10, callback_group=self._sub_cb)

        # Servicios MoveIt2 (async)
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._srv_cb)
        self.fk_client = self.create_client(
            GetPositionFK, '/compute_fk', callback_group=self._srv_cb)

        # Publicador directo de joint states
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer de control
        self.create_timer(1.0 / rate_hz, self._control_loop, callback_group=self._timer_cb)

        # Publicar posición home inmediatamente
        self._publish_joints(self._smoothed_joints)
        self.get_logger().info('Esperando gesto GRIP para iniciar teleoperacion...')

    def _declare_params(self):
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('servo_rate_hz', 20.0)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('position_scale_x', 2000.0)
        self.declare_parameter('position_scale_y', 2000.0)
        self.declare_parameter('position_scale_z', 2000.0)
        self.declare_parameter('vr_axis_to_robot_x', 'z')
        self.declare_parameter('vr_axis_to_robot_y', 'x')
        self.declare_parameter('vr_axis_to_robot_z', 'y')
        self.declare_parameter('flip_x', False)
        self.declare_parameter('flip_y', True)
        self.declare_parameter('flip_z', False)
        self.declare_parameter('workspace_x_min', -1500.0)
        self.declare_parameter('workspace_x_max', 1500.0)
        self.declare_parameter('workspace_y_min', -1500.0)
        self.declare_parameter('workspace_y_max', 1500.0)
        self.declare_parameter('workspace_z_min', 50.0)
        self.declare_parameter('workspace_z_max', 2000.0)
        self.declare_parameter('max_step_mm', 25.0)

    # ── Callbacks ──

    def _on_hand_pose(self, msg: PoseStamped):
        self.latest_vr_pose = VRPose(
            x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z,
            qx=msg.pose.orientation.x, qy=msg.pose.orientation.y,
            qz=msg.pose.orientation.z, qw=msg.pose.orientation.w,
        )

    def _on_gesture(self, msg: String):
        gesture = msg.data.strip()
        self._handle_gesture(gesture)
        self._prev_gesture = gesture

    # ── Máquina de estados ──

    def _handle_gesture(self, gesture: str):
        if gesture == self._prev_gesture:
            return
        if self.state == CAPTURING:
            return

        if self.state == IDLE and gesture == 'GRIP':
            self._do_capture()
        elif self.state == TELEOP and gesture == 'GRIP':
            self.state = PAUSED
            self.get_logger().info('PAUSED')
        elif self.state == PAUSED and gesture == 'GRIP':
            self._do_capture()
        elif gesture == 'THUMBS_UP' and self.state in (TELEOP, PAUSED):
            self.state = IDLE
            self.mapper.reset()
            self._ik_pending = False
            self.get_logger().info('IDLE')

    def _do_capture(self):
        if self.latest_vr_pose is None:
            self.get_logger().warn('No hay datos VR')
            return

        self.state = CAPTURING
        self._capture_vr_pose = VRPose(
            x=self.latest_vr_pose.x, y=self.latest_vr_pose.y,
            z=self.latest_vr_pose.z,
            qx=self.latest_vr_pose.qx, qy=self.latest_vr_pose.qy,
            qz=self.latest_vr_pose.qz, qw=self.latest_vr_pose.qw,
        )
        self._capture_joints = list(self._smoothed_joints)

        if not self.fk_client.service_is_ready():
            self._finish_capture(None)
            return

        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = ['Link6']
        req.robot_state.joint_state.name = list(JOINT_NAMES)
        req.robot_state.joint_state.position = list(self._capture_joints)
        future = self.fk_client.call_async(req)
        future.add_done_callback(self._on_fk_result)

    def _on_fk_result(self, future):
        robot_pose = None
        try:
            result = future.result()
            if result.error_code.val == 1 and len(result.pose_stamped) > 0:
                p = result.pose_stamped[0].pose
                # Guardar orientación real del TCP para usar en IK
                self._captured_orientation = Quaternion(
                    x=p.orientation.x, y=p.orientation.y,
                    z=p.orientation.z, w=p.orientation.w)
                rx, ry, rz = _quat_to_euler(
                    p.orientation.x, p.orientation.y,
                    p.orientation.z, p.orientation.w)
                robot_pose = RobotPose(
                    x=p.position.x * 1000.0, y=p.position.y * 1000.0,
                    z=p.position.z * 1000.0,
                    rx=math.degrees(rx), ry=math.degrees(ry), rz=math.degrees(rz),
                )
        except Exception:
            pass
        self._finish_capture(robot_pose)

    def _finish_capture(self, robot_pose: RobotPose | None):
        if robot_pose is None:
            robot_pose = RobotPose(x=0.0, y=1000.0, z=500.0, rx=180.0, ry=0.0, rz=90.0)

        self.mapper.capture(self._capture_vr_pose, robot_pose)
        self.last_sent_pose = robot_pose
        self._last_ik_solution = list(self._capture_joints)
        self._smoothed_joints = list(self._capture_joints)
        self._ik_pending = False
        self.state = TELEOP
        self.get_logger().info(
            f'TELEOP: ref=({robot_pose.x:.0f}, {robot_pose.y:.0f}, {robot_pose.z:.0f})mm')

    # ── Control loop (NUNCA bloquea) ──

    def _control_loop(self):
        # Siempre publicar joints actuales (mantiene robot visible en RViz)
        self._publish_joints(self._smoothed_joints)

        if self.state != TELEOP or self.latest_vr_pose is None:
            return

        # Mapear + seguridad (pura matemática, instantáneo)
        target = self.mapper.map(self.latest_vr_pose)
        target = self.safety.check(self.last_sent_pose, target)

        if self.dry_run:
            self.get_logger().info(
                f'[DRY] ({target.x:.0f}, {target.y:.0f}, {target.z:.0f})mm',
                throttle_duration_sec=0.5)
            self.last_sent_pose = target
            return

        # Fire async IK solo si no hay pendiente
        if not self._ik_pending:
            self._request_ik_async(target)

    def _request_ik_async(self, target: RobotPose):
        if not self.ik_client.service_is_ready():
            return

        req = GetPositionIK.Request()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.avoid_collisions = False

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(
            x=target.x / 1000.0, y=target.y / 1000.0, z=target.z / 1000.0)

        # Orientación: usar la capturada al hacer GRIP (consistente)
        pose.pose.orientation = self._captured_orientation
        req.ik_request.pose_stamped = pose

        # Seed con última solución IK para continuidad articular
        seed = self._last_ik_solution
        req.ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        req.ik_request.robot_state.joint_state.position = list(seed)

        self._ik_pending = True
        self._pending_target = target
        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik_result)

    def _on_ik_result(self, future):
        self._ik_pending = False
        if future.cancelled():
            return
        try:
            result = future.result()
        except Exception:
            return
        if result.error_code.val != 1:
            return

        sol = result.solution.joint_state
        positions = [0.0] * 6
        for i, name in enumerate(JOINT_NAMES):
            if name in sol.name:
                positions[i] = sol.position[sol.name.index(name)]
            else:
                return

        # EMA smoothing
        alpha = self._ema_alpha
        self._smoothed_joints = [
            alpha * new + (1.0 - alpha) * old
            for new, old in zip(positions, self._smoothed_joints)
        ]
        self._last_ik_solution = positions
        if self._pending_target is not None:
            self.last_sent_pose = self._pending_target

    def _publish_joints(self, positions: list[float]):
        """Publica JointState directamente — RSP actualiza TFs → RViz mueve robot."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = list(JOINT_NAMES)
        msg.position = list(positions)
        self.js_pub.publish(msg)


def _quat_to_euler(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = VRTeleopNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
