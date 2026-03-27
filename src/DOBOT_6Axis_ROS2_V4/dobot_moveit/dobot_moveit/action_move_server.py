#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import threading
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from dobot_msgs_v4.srv import EnableRobot, ServoJ, MovJ
import os

SERVO_DT = 0.10  # segundos por punto ServoJ — 100ms es óptimo para CR20

class FollowJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('dobot_group_controller')
        name = os.getenv("DOBOT_TYPE")
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            f'/{name}_group_controller/follow_joint_trajectory',
            self.execute_callback)

        self.get_logger().info("FollowJointTrajectory Action Server is ready...")

        self.EnableRobot_l = self.create_client(
            EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot',
            callback_group=self.cb_group)
        self.ServoJ_l = self.create_client(
            ServoJ, '/dobot_bringup_ros2/srv/ServoJ',
            callback_group=self.cb_group)
        self.MovJ_l = self.create_client(
            MovJ, '/dobot_bringup_ros2/srv/MovJ',
            callback_group=self.cb_group)

        max_retries = 30
        retry_count = 0
        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):
            retry_count += 1
            if retry_count >= max_retries:
                self.get_logger().error('Service not available after 30s, continuing anyway...')
                break
            self.get_logger().info('service not available, waiting again...')

    def _ensure_robot_ready(self):
        if not self.EnableRobot_l.service_is_ready():
            self.get_logger().warn('EnableRobot service not ready — continuing anyway')
            return
        req = EnableRobot.Request()
        future = self.EnableRobot_l.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        if not event.wait(timeout=3.0):
            self.get_logger().warn('EnableRobot timeout — continuing anyway')
        else:
            self.get_logger().info('EnableRobot OK — servo mode reset')

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Received a new trajectory goal!")
        self._ensure_robot_ready()
        trajectory = goal_handle.request.trajectory
        self.execution_trajectory(trajectory)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

    def _interpolate_trajectory(self, trajectory: JointTrajectory):
        """
        Interpola los waypoints de MoveIt en pasos uniformes de SERVO_DT segundos.
        Devuelve lista de listas de 6 joints en grados.
        """
        points = trajectory.points
        if len(points) == 0:
            return []

        # Convertir todos los waypoints a (tiempo_seg, [joints_deg])
        waypoints = []
        for p in points:
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            joints_deg = [math.degrees(j) for j in p.positions]
            waypoints.append((t, joints_deg))

        # Asegurar que empiece desde t=0
        t_offset = waypoints[0][0]
        waypoints = [(t - t_offset, j) for t, j in waypoints]

        t_total = waypoints[-1][0]
        if t_total <= 0:
            return [waypoints[-1][1]]

        # Generar puntos densos cada SERVO_DT segundos
        dense_points = []
        t_current = 0.0

        while t_current <= t_total + 1e-6:
            # Encontrar segmento donde cae t_current
            joints = self._sample_at(waypoints, t_current)
            dense_points.append(joints)
            t_current += SERVO_DT

        self.get_logger().info(
            f"Interpolados {len(dense_points)} puntos ServoJ "
            f"desde {len(waypoints)} waypoints MoveIt "
            f"(duración total: {t_total:.2f}s)")
        return dense_points

    def _sample_at(self, waypoints, t):
        """Interpola linealmente en joint-space en el tiempo t."""
        # Antes del inicio
        if t <= waypoints[0][0]:
            return waypoints[0][1][:]
        # Después del final
        if t >= waypoints[-1][0]:
            return waypoints[-1][1][:]
        # Buscar segmento
        for i in range(len(waypoints) - 1):
            t0, j0 = waypoints[i]
            t1, j1 = waypoints[i + 1]
            if t0 <= t <= t1:
                if abs(t1 - t0) < 1e-9:
                    return j1[:]
                alpha = (t - t0) / (t1 - t0)
                return [j0[k] + alpha * (j1[k] - j0[k]) for k in range(len(j0))]
        return waypoints[-1][1][:]
    
    def execution_trajectory(self, trajectory: JointTrajectory):
        dense_points = self._interpolate_trajectory(trajectory)
        if not dense_points:
            self.get_logger().error('Sin puntos para ejecutar')
            return

        self.get_logger().info(f"Enviando {len(dense_points)} puntos ServoJ al robot...")

        t_start = time.monotonic()
        for i, joints in enumerate(dense_points):
            self.ServoJ_C(joints[0], joints[1], joints[2],
                        joints[3], joints[4], joints[5])
            # Timing absoluto — sin acumulación de error
            next_time = t_start + (i + 1) * SERVO_DT
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.get_logger().info("Trayectoria completada")

    def ServoJ_C(self, j1, j2, j3, j4, j5, j6):
        P1 = ServoJ.Request()
        P1.a = float(j1)
        P1.b = float(j2)
        P1.c = float(j3)
        P1.d = float(j4)
        P1.e = float(j5)
        P1.f = float(j6)
        P1.param_value = [f"t={SERVO_DT + 0.02:.3f}"]  # t ligeramente mayor que SERVO_DT
        self.ServoJ_l.call_async(P1)  # fire-and-forget


def main(args=None):
    rclpy.init(args=args)
    server = FollowJointTrajectoryServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(server)
    executor.spin()
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()