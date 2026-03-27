"""
Script de calibración de ejes para VR teleop.

Simula movimientos de mano en cada eje VR y muestra en qué dirección
se mueve el robot. Ejecutar mientras vr_teleop_sim.launch.py está corriendo.

Uso:
  ros2 run vr_teleop calibrate_axes
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class AxisCalibrator(Node):
    def __init__(self):
        super().__init__('axis_calibrator')
        self.pose_pub = self.create_publisher(PoseStamped, '/hand_right/pose', 10)
        self.gesture_pub = self.create_publisher(String, '/hand_right/gesture', 10)

    def publish_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def publish_gesture(self, gesture):
        msg = String()
        msg.data = gesture
        self.gesture_pub.publish(msg)

    def run_calibration(self):
        base_x, base_y, base_z = 0.0, 0.3, 0.0
        delta = 0.15  # 15cm de movimiento

        print('\n=== CALIBRACIÓN DE EJES VR TELEOP ===')
        print('Mira el robot en RViz mientras ejecuto los movimientos.\n')

        # 1. Publicar pose base y activar GRIP
        print('Publicando pose base y activando teleop (GRIP)...')
        for _ in range(20):
            self.publish_pose(base_x, base_y, base_z)
            self.publish_gesture('---')
            time.sleep(0.05)

        # Activar GRIP
        for _ in range(10):
            self.publish_pose(base_x, base_y, base_z)
            self.publish_gesture('GRIP')
            time.sleep(0.05)

        # Volver a sin gesto para que entre en TELEOP
        for _ in range(10):
            self.publish_pose(base_x, base_y, base_z)
            self.publish_gesture('---')
            time.sleep(0.05)

        time.sleep(1.0)
        print('Teleop activado. Esperando 2s en posición base...\n')

        # Mantener pose base
        for _ in range(40):
            self.publish_pose(base_x, base_y, base_z)
            self.publish_gesture('OPEN')
            time.sleep(0.05)

        # 2. Test VR X+ (mano a la DERECHA)
        print('>>> Moviendo mano VR hacia la DERECHA (VR X+)...')
        print('    Observa: ¿hacia dónde se mueve el robot?')
        for _ in range(60):
            self.publish_pose(base_x + delta, base_y, base_z)
            self.publish_gesture('OPEN')
            time.sleep(0.05)
        print('    Volviendo al centro...')
        for _ in range(40):
            self.publish_pose(base_x, base_y, base_z)
            self.publish_gesture('OPEN')
            time.sleep(0.05)

        time.sleep(0.5)

        # 3. Test VR Y+ (mano ARRIBA)
        print('\n>>> Moviendo mano VR hacia ARRIBA (VR Y+)...')
        print('    Observa: ¿hacia dónde se mueve el robot?')
        for _ in range(60):
            self.publish_pose(base_x, base_y + delta, base_z)
            self.publish_gesture('OPEN')
            time.sleep(0.05)
        print('    Volviendo al centro...')
        for _ in range(40):
            self.publish_pose(base_x, base_y, base_z)
            self.publish_gesture('OPEN')
            time.sleep(0.05)

        time.sleep(0.5)

        # 4. Test VR Z+ (mano ADELANTE)
        print('\n>>> Moviendo mano VR hacia ADELANTE (VR Z+)...')
        print('    Observa: ¿hacia dónde se mueve el robot?')
        for _ in range(60):
            self.publish_pose(base_x, base_y, base_z + delta)
            self.publish_gesture('OPEN')
            time.sleep(0.05)
        print('    Volviendo al centro...')
        for _ in range(40):
            self.publish_pose(base_x, base_y, base_z)
            self.publish_gesture('OPEN')
            time.sleep(0.05)

        # 5. Desactivar
        print('\n>>> Desactivando teleop (THUMBS_UP)...')
        for _ in range(10):
            self.publish_pose(base_x, base_y, base_z)
            self.publish_gesture('THUMBS_UP')
            time.sleep(0.05)

        print('\n=== CALIBRACIÓN COMPLETA ===')
        print('Dime qué observaste para cada movimiento:')
        print('  1. VR DERECHA (X+) → ¿el robot fue hacia...?')
        print('  2. VR ARRIBA (Y+) → ¿el robot fue hacia...?')
        print('  3. VR ADELANTE (Z+) → ¿el robot fue hacia...?')


def main(args=None):
    rclpy.init(args=args)
    node = AxisCalibrator()
    node.run_calibration()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
