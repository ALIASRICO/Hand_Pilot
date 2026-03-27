"""
Monitor de seguridad para teleoperación VR del Dobot CR20.

Aplica límites de workspace, velocidad máxima y alcance
para evitar movimientos peligrosos.
"""

import math
import numpy as np
from vr_teleop.coordinate_mapper import RobotPose


class SafetyMonitor:
    """Valida y limita las poses objetivo del robot."""

    def __init__(
        self,
        workspace_x_min: float = -1500.0,
        workspace_x_max: float = 1500.0,
        workspace_y_min: float = -1500.0,
        workspace_y_max: float = 1500.0,
        workspace_z_min: float = 50.0,
        workspace_z_max: float = 2000.0,
        max_reach_mm: float = 1700.0,
        max_step_mm: float = 50.0,
    ):
        self.ws_min = np.array([workspace_x_min, workspace_y_min, workspace_z_min])
        self.ws_max = np.array([workspace_x_max, workspace_y_max, workspace_z_max])
        self.max_reach = max_reach_mm
        self.max_step = max_step_mm

    def clamp_to_workspace(self, target: RobotPose) -> RobotPose:
        """Limita la pose al workspace permitido."""
        pos = np.array([target.x, target.y, target.z])
        clamped = np.clip(pos, self.ws_min, self.ws_max)

        # Limitar alcance radial (distancia horizontal desde la base)
        dist_xy = math.sqrt(clamped[0] ** 2 + clamped[1] ** 2)
        if dist_xy > self.max_reach:
            scale = self.max_reach / dist_xy
            clamped[0] *= scale
            clamped[1] *= scale

        return RobotPose(
            x=float(clamped[0]),
            y=float(clamped[1]),
            z=float(clamped[2]),
            rx=target.rx,
            ry=target.ry,
            rz=target.rz,
        )

    def limit_step(self, current: RobotPose, target: RobotPose) -> RobotPose:
        """
        Limita el paso máximo entre la pose actual y la objetivo.
        Evita saltos bruscos por glitches de tracking.
        """
        delta = np.array([
            target.x - current.x,
            target.y - current.y,
            target.z - current.z,
        ])
        dist = np.linalg.norm(delta)

        if dist <= self.max_step or dist < 1e-6:
            return target

        # Escalar el delta al paso máximo
        limited = delta * (self.max_step / dist)
        return RobotPose(
            x=current.x + float(limited[0]),
            y=current.y + float(limited[1]),
            z=current.z + float(limited[2]),
            rx=target.rx,
            ry=target.ry,
            rz=target.rz,
        )

    def check(self, current: RobotPose, target: RobotPose) -> RobotPose:
        """Aplica todas las validaciones de seguridad."""
        safe = self.clamp_to_workspace(target)
        safe = self.limit_step(current, safe)
        return safe
