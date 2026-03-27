"""
Mapeo de coordenadas VR (Meta Quest) a coordenadas del robot Dobot CR20.

Modo relativo: captura una pose inicial de la mano VR y del robot,
luego traduce desplazamientos de la mano a desplazamientos del robot.

Sistemas de coordenadas:
  - VR (Meta Quest / Unity): Y-arriba, derecha = X+, adelante = Z+
  - Robot (Dobot CR20): Z-arriba, X+ adelante, Y+ izquierda
"""

import numpy as np
from dataclasses import dataclass, field


@dataclass
class RobotPose:
    """Pose del TCP del robot en mm y grados."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0


@dataclass
class VRPose:
    """Pose de la mano VR en metros y quaternion."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


# Mapeo de nombre de eje VR a indice (x=0, y=1, z=2)
AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}


class CoordinateMapper:
    """Mapea desplazamientos de mano VR a desplazamientos del robot."""

    def __init__(
        self,
        scale_x: float = 2500.0,
        scale_y: float = 2500.0,
        scale_z: float = 2500.0,
        vr_axis_to_robot_x: str = 'z',
        vr_axis_to_robot_y: str = 'x',
        vr_axis_to_robot_z: str = 'y',
        flip_x: bool = False,
        flip_y: bool = True,
        flip_z: bool = False,
        map_orientation: bool = False,
    ):
        self.scale = np.array([scale_x, scale_y, scale_z])

        # Qué eje VR alimenta cada eje del robot
        self.vr_index_for_robot = [
            AXIS_INDEX[vr_axis_to_robot_x],
            AXIS_INDEX[vr_axis_to_robot_y],
            AXIS_INDEX[vr_axis_to_robot_z],
        ]

        self.flip = np.array([
            -1.0 if flip_x else 1.0,
            -1.0 if flip_y else 1.0,
            -1.0 if flip_z else 1.0,
        ])

        self.map_orientation = map_orientation

        # Poses de referencia (se llenan al capturar)
        self._vr_ref: VRPose | None = None
        self._robot_ref: RobotPose | None = None

    @property
    def is_captured(self) -> bool:
        return self._vr_ref is not None and self._robot_ref is not None

    def capture(self, vr_pose: VRPose, robot_pose: RobotPose) -> None:
        """Captura las poses de referencia para el modo relativo."""
        self._vr_ref = VRPose(
            x=vr_pose.x, y=vr_pose.y, z=vr_pose.z,
            qx=vr_pose.qx, qy=vr_pose.qy, qz=vr_pose.qz, qw=vr_pose.qw,
        )
        self._robot_ref = RobotPose(
            x=robot_pose.x, y=robot_pose.y, z=robot_pose.z,
            rx=robot_pose.rx, ry=robot_pose.ry, rz=robot_pose.rz,
        )

    def map(self, vr_pose: VRPose) -> RobotPose:
        """
        Convierte una pose VR actual en una pose objetivo del robot.

        Retorna la pose en mm (posición) y grados (orientación).
        Si no se ha capturado, retorna la pose de referencia del robot.
        """
        if not self.is_captured:
            return RobotPose()

        # Desplazamiento en espacio VR (metros)
        vr_delta = np.array([
            vr_pose.x - self._vr_ref.x,
            vr_pose.y - self._vr_ref.y,
            vr_pose.z - self._vr_ref.z,
        ])

        # Remapear ejes VR -> Robot y aplicar escala + flip
        # scale ya convierte de metros a mm (ej: 1m * 2500 = 2500mm)
        robot_delta = np.array([
            vr_delta[self.vr_index_for_robot[0]] * self.scale[0] * self.flip[0],
            vr_delta[self.vr_index_for_robot[1]] * self.scale[1] * self.flip[1],
            vr_delta[self.vr_index_for_robot[2]] * self.scale[2] * self.flip[2],
        ])

        # Pose objetivo = referencia + delta
        target = RobotPose(
            x=self._robot_ref.x + robot_delta[0],
            y=self._robot_ref.y + robot_delta[1],
            z=self._robot_ref.z + robot_delta[2],
            rx=self._robot_ref.rx,
            ry=self._robot_ref.ry,
            rz=self._robot_ref.rz,
        )

        return target

    def update_scales(self, scale_x: float, scale_y: float, scale_z: float) -> None:
        self.scale = np.array([scale_x, scale_y, scale_z])

    def reset(self) -> None:
        """Limpia las poses de referencia."""
        self._vr_ref = None
        self._robot_ref = None
