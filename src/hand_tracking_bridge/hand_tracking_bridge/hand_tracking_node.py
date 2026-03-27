#!/usr/bin/env python3
"""
Nodo ROS2: Puente Hand Tracking Quest → ROS2 Topics
Recibe datos UDP del Quest, aplica filtrado y publica:
  - /hand_right/pose  (PoseStamped) → para MoveIt
  - /hand_right/gesture (String) → gesto activo
  - /hand_left/pose (PoseStamped) → informativo
  - /hand_right/pinch (Float32) → fuerza de pinch (0.0-1.0)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32

import socket
import numpy as np
from scipy.spatial.transform import Rotation

# Importar nuestros módulos
from hand_tracking_bridge.hand_data_parser import HandTrackingParser, HandData
from hand_tracking_bridge.hand_filters import FilteredHandTracker, GestureState

class HandTrackingNode(Node):
    """
    Nodo que recibe hand tracking por UDP y publica en ROS2
    """

    def __init__(self):
        super().__init__('hand_tracking_node')

        # Parámetros ROS2
        self.declare_parameter('udp_port', 7777)
        self.declare_parameter('filter_alpha', 0.4)
        self.declare_parameter('frame_id', 'world')

        self.udp_port = self.get_parameter('udp_port').value
        self.filter_alpha = self.get_parameter('filter_alpha').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.pub_right_pose = self.create_publisher(
            PoseStamped, '/hand_right/pose', 10
        )
        self.pub_left_pose = self.create_publisher(
            PoseStamped, '/hand_left/pose', 10
        )
        self.pub_gesture = self.create_publisher(
            String, '/hand_right/gesture', 10
        )
        self.pub_pinch = self.create_publisher(
            Float32, '/hand_right/pinch', 10
        )

        # Parser y filtros
        self.parser = HandTrackingParser()
        self.tracker = FilteredHandTracker(alpha=self.filter_alpha)

        # Estado persistente
        self.left_hand = None
        self.right_hand = None

        # Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.udp_port))
        self.sock.settimeout(0.01)  # No-blocking casi

        # Timer para recibir datos (~100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Estadísticas
        self.frame_count = 0

        self.get_logger().info(
            f'Hand Tracking Node iniciado — UDP:{self.udp_port}'
        )

    def timer_callback(self):
        """Lee datos UDP y publica en ROS2"""
        # Leer todos los paquetes disponibles
        for _ in range(10):  # Max 10 paquetes por ciclo
            try:
                data, _ = self.sock.recvfrom(4096)
                line = data.decode('utf-8').strip()
                raw_frame = self.parser.process_line(line)

                if raw_frame is None:
                    continue

                self.frame_count += 1

                # Guardar mano izquierda (raw)
                if raw_frame.left_hand:
                    self.left_hand = raw_frame.left_hand

                # Procesar mano derecha (filtrado + gestos)
                if raw_frame.right_hand:
                    filtered = self.tracker.process(raw_frame)
                    if filtered.right_hand:
                        self.right_hand = filtered.right_hand

            except socket.timeout:
                break
            except Exception as e:
                self.get_logger().debug(f'Error UDP: {e}')
                break

        # Publicar mano izquierda
        if self.left_hand:
            self.publish_hand_pose(
                self.left_hand, self.pub_left_pose
            )

        # Publicar mano derecha + gestos
        if self.right_hand:
            self.publish_hand_pose(
                self.right_hand, self.pub_right_pose
            )
            self.publish_gesture()

    def publish_hand_pose(self, hand: HandData, publisher):
        """Publicar pose de muñeca como PoseStamped"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Posición
        msg.pose.position.x = float(hand.wrist.position[0])
        msg.pose.position.y = float(hand.wrist.position[1])
        msg.pose.position.z = float(hand.wrist.position[2])

        # Orientación (quaternion)
        q = hand.wrist.orientation
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        publisher.publish(msg)

    def publish_gesture(self):
        """Publicar gesto activo y fuerza de pinch"""
        g = self.tracker.right_gestures

        # Gesto
        msg_gesture = String()
        msg_gesture.data = g.gesture_name
        self.pub_gesture.publish(msg_gesture)

        # Pinch strength
        msg_pinch = Float32()
        msg_pinch.data = float(g.pinch_strength)
        self.pub_pinch.publish(msg_pinch)

        # Log cada 100 frames
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Frame:{self.frame_count} | '
                f'Gesto:{g.gesture_name} | '
                f'Pinch:{g.pinch_strength:.0%}'
            )

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Detenido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
