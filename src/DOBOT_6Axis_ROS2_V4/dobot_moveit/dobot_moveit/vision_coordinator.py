#!/usr/bin/env python3
"""
================================================================
Nodo Integrador: Visión + MoveIt (Por Joints)
================================================================
Este nodo conecta la detección de visión con el movimiento del robot
usando MoveIt para planificación por JOINTS (no cartesiano).

Uso:
    ros2 run dobot_moveit vision_coordinator
================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import json
import os
import threading
import time

# ROS2 messages
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory

# MoveIt2
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander


class VisionCoordinator(Node):
    """
    Nodo que integra detección de visión con MoveIt para mover el robot POR JOINTS.
    """

    def __init__(self):
        super().__init__('vision_coordinator')
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Vision Coordinator (Joint Mode) - Iniciando...')
        self.get_logger().info('=' * 50)

        # ============================================
        # Parámetros
        # ============================================
        self.declare_parameter('target_tag_id', 99)
        self.declare_parameter('calibration_file', 'calibration_config.json')
        self.declare_parameter('z_work_height', 50.0)
        self.declare_parameter('move_speed', 0.5)
        
        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.z_work_height = self.get_parameter('z_work_height').value
        self.move_speed = self.get_parameter('move_speed').value

        # ============================================
        # Estado
        # ============================================
        self.current_detection = None
        self.detection_lock = threading.Lock()
        self.calibration_matrix = None
        self.camera_info = None
        self.running = True

        # Callback group
        self.callback_group = ReentrantCallbackGroup()

        # ============================================
        # Suscriptores
        # ============================================
        self.detections_sub = self.create_subscription(
            Float32MultiArray,
            '/apriltag/detections_cm',
            self.detections_callback,
            10,
            callback_group=self.callback_group
        )

        # ============================================
        # Cargar calibración
        # ============================================
        self.load_calibration()

        # ============================================
        # Inicializar MoveIt
        # ============================================
        self.init_moveit()

        # ============================================
        # Thread de teclado
        # ============================================
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.get_logger().info('=' * 50)
        self.get_logger().info('Vision Coordinator iniciado')
        self.get_logger().info('Modo: Movimiento por JOINTS')
        self.get_logger().info('Presiona M para mover al tag')
        self.get_logger().info('Presiona H para home')
        self.get_logger().info('Presiona Q para salir')
        self.get_logger().info('=' * 50)

    def load_calibration(self):
        """Carga la matriz de calibración"""
        possible_paths = [
            self.calibration_file,
            os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 
                        'paletizado_ws', 'calibration_config.json'),
            os.path.join(os.path.expanduser('~'), 'dobot_ws', 'paletizado_ws', 
                        'calibration_config.json'),
        ]
        
        calibration_path = None
        for path in possible_paths:
            if os.path.exists(path):
                calibration_path = path
                break
        
        if calibration_path:
            try:
                with open(calibration_path, 'r') as f:
                    config = json.load(f)
                
                if 'affine_belt_2x3' in config:
                    self.calibration_matrix = np.array(config['affine_belt_2x3'])
                    self.z_work = config.get('z_trabajo_mm', self.z_work_height)
                    self.get_logger().info(f'Calibración cargada: {calibration_path}')
                else:
                    self.calibration_matrix = np.eye(2, 3)
                    self.z_work = self.z_work_height
            except Exception as e:
                self.get_logger().error(f'Error cargando calibración: {e}')
                self.calibration_matrix = np.eye(2, 3)
        else:
            self.get_logger().warn('Sin calibración, usando identidad')
            self.calibration_matrix = np.eye(2, 3)

    def init_moveit(self):
        """Inicializa MoveIt2"""
        try:
            self.robot_commander = RobotCommander()
            self.move_group = MoveGroupCommander('cr20_arm')
            self.scene = PlanningSceneInterface()
            
            # Configuración para movimiento por joints
            self.move_group.set_max_velocity_scaling_factor(self.move_speed)
            self.move_group.set_max_acceleration_scaling_factor(0.5)
            
            self.planning_frame = self.move_group.get_planning_frame()
            self.joint_names = self.move_group.get_joints()
            
            self.get_logger().info(f'Planning frame: {self.planning_frame}')
            self.get_logger().info(f'Joints: {self.joint_names}')
            
            # Obtener posición actual
            current_joints = self.move_group.get_current_joint_values()
            self.get_logger().info(f'Joint actual: {current_joints}')
            
        except Exception as e:
            self.get_logger().error(f'Error inicializando MoveIt: {e}')
            raise

    def detections_callback(self, msg: Float32MultiArray):
        """Callback para detecciones"""
        data = msg.data
        if len(data) < 4:
            return
        
        with self.detection_lock:
            for i in range(0, len(data), 4):
                tag_id = int(data[i])
                if tag_id == self.target_tag_id:
                    self.current_detection = {
                        'id': tag_id,
                        'x': float(data[i+1]),
                        'y': float(data[i+2]),
                        'z': float(data[i+3]),
                    }
                    break

    def camera_to_robot(self, cam_x_mm, cam_y_mm):
        """Convierte coordenadas cámara a robot usando calibración"""
        if self.calibration_matrix is None:
            return -cam_x_mm / 1000.0, -cam_y_mm / 1000.0
        
        affine = self.calibration_matrix
        robot_x = affine[0, 0] * cam_x_mm + affine[0, 1] * cam_y_mm + affine[0, 2]
        robot_y = affine[1, 0] * cam_x_mm + affine[1, 1] * cam_y_mm + affine[1, 2]
        
        return robot_x / 1000.0, robot_y / 1000.0

    def move_to_tag(self):
        """
        Mueve el robot a la posición del tag usando movimiento por JOINTS.
        """
        with self.detection_lock:
            if self.current_detection is None:
                self.get_logger().warn('No hay detección')
                return False
            
            detection = self.current_detection.copy()
        
        self.get_logger().info('=' * 40)
        self.get_logger().info(f'Moviendo por JOINTS al tag {detection["id"]}')
        
        # Convertir a coordenadas del robot
        robot_x, robot_y = self.camera_to_robot(detection['x'], detection['y'])
        robot_z = self.z_work / 1000.0
        
        self.get_logger().info(f'Target: X={robot_x:.3f}m Y={robot_y:.3f} Z={robot_z:.3f}m')
        
        # ============================================
        # Movimiento por JOINTS
        # ============================================
        # Primero, ir a una pose cartesiana (esto calcula los joints automáticamente)
        # Luego ejecutamos la trayectoria
        try:
            # Obtener posición actual
            start_joints = self.move_group.get_current_joint_values()
            
            # Establecer pose objetivo
            pose_target = self.move_group.get_current_pose()
            pose_target.pose.position.x = robot_x
            pose_target.pose.position.y = robot_y
            pose_target.pose.position.z = robot_z
            
            self.move_group.set_pose_target(pose_target)
            
            # Planificar
            plan = self.move_group.go(wait=True)
            
            # Limpiar
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            if plan:
                self.get_logger().info('✅ Movimiento por joints completado')
                
                # Mostrar joints finales
                final_joints = self.move_group.get_current_joint_values()
                self.get_logger().info(f'Joints finales: {final_joints}')
                return True
            else:
                self.get_logger().warn('❌ Falló planificación')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False

    def move_to_named_target(self, target_name):
        """
        Mueve el robot a una posición nombrada (home, ready, etc) por JOINTS.
        """
        self.get_logger().info(f'Moviendo a posición: {target_name}')
        
        try:
            self.move_group.set_named_target(target_name)
            plan = self.move_group.go(wait=True)
            self.move_group.stop()
            
            if plan:
                self.get_logger().info(f'✅ Posición {target_name} alcanzada')
                return True
            return False
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False

    def move_joints_direct(self, joint_values):
        """
        Mueve el robot directamente a valores de joints específicos.
        """
        self.get_logger().info(f'Moviendo joints: {joint_values}')
        
        try:
            self.move_group.set_joint_value_target(joint_values)
            plan = self.move_group.go(wait=True)
            self.move_group.stop()
            
            if plan:
                self.get_logger().info('✅ Movimiento completado')
                return True
            return False
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False

    def keyboard_listener(self):
        """Listener de teclado"""
        import sys
        import termios
        import tty
        
        time.sleep(3.0)
        
        while self.running:
            try:
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setraw(fd)
                    key = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
                if key.lower() == 'm':
                    self.move_to_tag()
                elif key.lower() == 'h':
                    self.move_to_named_target('home')
                elif key.lower() == 'r':
                    self.move_to_named_target('ready')
                elif key.lower() == 'q':
                    print('\n👋 Saliendo...')
                    self.running = False
                    rclpy.shutdown()
                    break
                    
            except Exception as e:
                self.get_logger().warning(f"Error procesando entrada: {e}")

    def display_callback(self):
        """Muestra estado actual"""
        with self.detection_lock:
            if self.current_detection:
                det = self.current_detection
                print(f'🎯 Tag {det["id"]}: X={det["x"]:.0f} Y={det["y"]:.0f} Z={det["z"]:.0f}mm')
            else:
                print('❌ Sin detección')

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
