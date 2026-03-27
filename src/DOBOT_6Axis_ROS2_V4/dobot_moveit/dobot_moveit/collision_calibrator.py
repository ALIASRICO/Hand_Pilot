"""
Collision calibration geometry helpers.

Pure Python functions (no ROS2 / numpy dependencies) that convert
millimetre-space calibration measurements into MoveIt-compatible
box parameters (center in metres, dimensions in metres).

These functions are consumed by the CollisionCalibrator node (Task 2)
and tested standalone by scripts/test_collision_geometry.py.
"""

import math


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def corners_to_box(p1_mm, p2_mm, height_mm):
    """Two diagonal top-surface corners + object height -> MoveIt box params.

    Args:
        p1_mm:     [x, y, z] first corner on the top surface (mm).
        p2_mm:     [x, y, z] second diagonal corner on the top surface (mm).
        height_mm: Positive height of the box (mm).

    Returns:
        (center_m, dimensions_m) where both are [x, y, z] lists in metres.
        center XY  = midpoint of p1, p2.
        center Z   = avg(p1.z, p2.z) - height_mm / 2  (half-height below surface).
        dimensions = [|x2-x1|, |y2-y1|, height] all converted to metres.

    Raises:
        ValueError: if any resulting dimension is < 50 mm.
    """
    dx = abs(p2_mm[0] - p1_mm[0])
    dy = abs(p2_mm[1] - p1_mm[1])
    dz = height_mm

    if dx < 50.0:
        raise ValueError(f"Width {dx:.1f} mm is below 50 mm minimum")
    if dy < 50.0:
        raise ValueError(f"Depth {dy:.1f} mm is below 50 mm minimum")
    if dz < 50.0:
        raise ValueError(f"Height {dz:.1f} mm is below 50 mm minimum")

    cx = (p1_mm[0] + p2_mm[0]) / 2.0
    cy = (p1_mm[1] + p2_mm[1]) / 2.0
    z_surface = (p1_mm[2] + p2_mm[2]) / 2.0
    cz = z_surface - height_mm / 2.0

    center_m = [cx / 1000.0, cy / 1000.0, cz / 1000.0]
    dimensions_m = [dx / 1000.0, dy / 1000.0, dz / 1000.0]
    return center_m, dimensions_m


def floor_plane(z_mm):
    """One floor Z measurement -> large collision plane.

    Returns:
        (center_m, dimensions_m) — a 6 m x 6 m x 0.01 m slab whose top
        surface sits at z_mm.
    """
    center_m = [0.0, 0.0, z_mm / 1000.0 - 0.005]
    dimensions_m = [6.0, 6.0, 0.01]
    return center_m, dimensions_m


def ceiling_plane(z_mm):
    """One ceiling Z measurement -> large collision plane.

    Returns:
        (center_m, dimensions_m) — a 6 m x 6 m x 0.01 m slab whose bottom
        surface sits at z_mm.
    """
    center_m = [0.0, 0.0, z_mm / 1000.0 + 0.005]
    dimensions_m = [6.0, 6.0, 0.01]
    return center_m, dimensions_m


def pole_box(bottom_mm, width_mm, ceiling_z_mm):
    """Camera pole: bottom point + width + ceiling Z -> vertical box.

    Args:
        bottom_mm:    [x, y, z] bottom centre of the pole (mm).
        width_mm:     Cross-section width (mm), used for both X and Y.
        ceiling_z_mm: Z coordinate of the ceiling (mm).

    Returns:
        (center_m, dimensions_m) where dimensions = [w, w, h] in metres.

    Raises:
        ValueError: if width_mm < 10 or computed height < 50 mm.
    """
    if width_mm < 10.0:
        raise ValueError(f"Pole width {width_mm:.1f} mm is below 10 mm minimum")

    h = ceiling_z_mm - bottom_mm[2]
    if h < 50.0:
        raise ValueError(f"Pole height {h:.1f} mm is below 50 mm minimum")

    cz = (bottom_mm[2] + ceiling_z_mm) / 2.0
    center_m = [bottom_mm[0] / 1000.0, bottom_mm[1] / 1000.0, cz / 1000.0]
    dimensions_m = [width_mm / 1000.0, width_mm / 1000.0, h / 1000.0]
    return center_m, dimensions_m


def wall_box(p1_mm, p2_mm, ceiling_z_mm, floor_z_mm, thickness_mm=100.0):
    """Two base points -> wall from floor to ceiling.

    The wall runs along the line from p1 to p2.  Thickness is perpendicular
    to that line.

    Args:
        p1_mm:         [x, y, z] first base point (mm).
        p2_mm:         [x, y, z] second base point (mm).
        ceiling_z_mm:  Z of the ceiling (mm).
        floor_z_mm:    Z of the floor (mm).
        thickness_mm:  Wall thickness (mm, default 100).

    Returns:
        (center_m, dimensions_m, rotation_rad)
        - center_m      = midpoint at half-height (metres).
        - dimensions_m   = [length, thickness, height] in metres.
        - rotation_rad   = math.atan2(dy, dx) — rotation around Z axis.

    Raises:
        ValueError: if wall length (distance p1 -> p2) < 100 mm.
    """
    dx = p2_mm[0] - p1_mm[0]
    dy = p2_mm[1] - p1_mm[1]
    length = math.sqrt(dx * dx + dy * dy)

    if length < 100.0:
        raise ValueError(f"Wall length {length:.1f} mm is below 100 mm minimum")

    height = ceiling_z_mm - floor_z_mm
    cx = (p1_mm[0] + p2_mm[0]) / 2.0
    cy = (p1_mm[1] + p2_mm[1]) / 2.0
    cz = (floor_z_mm + ceiling_z_mm) / 2.0

    center_m = [cx / 1000.0, cy / 1000.0, cz / 1000.0]
    dimensions_m = [length / 1000.0, thickness_mm / 1000.0, height / 1000.0]
    rotation_rad = math.atan2(dy, dx)

    return center_m, dimensions_m, rotation_rad


def derive_place_position(p1_mm, p2_mm, height_mm, offset_mm=20.0):
    """Derive place_position from cinta (conveyor belt) corners.

    Args:
        p1_mm:     [x, y, z] first corner on the top surface (mm).
        p2_mm:     [x, y, z] second diagonal corner (mm).
        height_mm: Object height used during calibration (mm) — not used
                   directly; z_top is taken from the corner Z values.
        offset_mm: Extra clearance above the surface (mm, default 20).

    Returns:
        [center_x_m, center_y_m, place_z_m] in metres.
        z_top_mm = avg(p1.z, p2.z)  (the surface of the cinta).
        place_z  = (z_top_mm + offset_mm) / 1000.
    """
    cx = (p1_mm[0] + p2_mm[0]) / 2.0
    cy = (p1_mm[1] + p2_mm[1]) / 2.0
    z_top = (p1_mm[2] + p2_mm[2]) / 2.0

    return [cx / 1000.0, cy / 1000.0, (z_top + offset_mm) / 1000.0]


def estimate_n_layers(z_top_detection_mm, z_pallet_surface_mm, box_height_mm):
    """Estimate the number of box layers on the pallet.

    Uses YOLO cam_z_mm (smaller = closer to camera = TOP of stack) and the
    pallet surface Z in robot frame to compute the layer count.

    Args:
        z_top_detection_mm: cam_z_mm of the topmost detection (mm, camera frame).
                            Smaller value = closer to camera = higher stack.
        z_pallet_surface_mm: Z of the empty pallet surface in robot frame (mm).
                             Typically negative (below base_link origin).
        box_height_mm: Height of each box/layer (mm). Must be > 0.

    Returns:
        int >= 0 — number of layers detected. 0 means nothing or bad input.

    Raises:
        ValueError: if box_height_mm <= 0.
    """
    if box_height_mm <= 0:
        raise ValueError(f"box_height_mm must be > 0, got {box_height_mm}")

    # Convert cam_z detection to robot Z using the relationship:
    # The pallet surface calibration gives us z_pallet_surface_mm in robot frame.
    # The cam_z_mm is depth from camera: larger value = farther = lower in scene.
    # However, the relationship between cam_z_mm and robot Z is captured by the
    # calibration plane. Here we work purely in robot-frame Z difference:
    # robot_z_top_surface ≈ z_pallet_surface_mm + (n_layers * box_height_mm)
    # We estimate n_layers from the detection Z difference.
    # Note: z_top_detection_mm here is expected in robot frame (robot_z_grasp * 1000).
    z_diff_mm = z_top_detection_mm - z_pallet_surface_mm
    if z_diff_mm <= 0:
        return 0

    n = round(z_diff_mm / box_height_mm)
    return max(0, n)


def compute_layer_slabs(pallet_surface_z_mm, box_height_mm, n_layers,
                        pallet_dx_mm, pallet_dy_mm,
                        pallet_cx_mm, pallet_cy_mm,
                        safety_margin_mm=5.0):
    """Generate collision slab descriptors for all layers BELOW the active top layer.

    The top layer (layer N) has NO slab — that is where the robot picks.
    Layers 1 through N-1 (bottom to second-from-top) each get one slab.

    Args:
        pallet_surface_z_mm: Z of the empty pallet surface in robot frame (mm).
        box_height_mm:        Height of each layer/box (mm).
        n_layers:             Total number of layers currently on the pallet.
        pallet_dx_mm:         Pallet width X (mm).
        pallet_dy_mm:         Pallet width Y (mm).
        pallet_cx_mm:         Pallet center X in robot frame (mm).
        pallet_cy_mm:         Pallet center Y in robot frame (mm).
        safety_margin_mm:     Vertical safety gap between slab top and active layer (mm).

    Returns:
        list of dict — one per slab (bottom to second-from-top), each dict:
            {
                'name':      str  — 'pallet_capa_N' (1-indexed from bottom),
                'center_m':  [cx, cy, cz] in metres,
                'dims_m':    [dx, dy, dz] in metres,
                'layer_idx': int  — 1-indexed layer number from bottom,
            }
        Returns [] if n_layers <= 1 (no lower layers to protect).
    """
    if n_layers <= 1:
        return []

    slabs = []
    for layer_idx in range(1, n_layers):  # layers 1..N-1 (exclude active top N)
        z_bottom_mm = pallet_surface_z_mm + (layer_idx - 1) * box_height_mm
        z_top_mm    = z_bottom_mm + box_height_mm - safety_margin_mm
        z_center_mm = (z_bottom_mm + z_top_mm) / 2.0
        dz_mm       = z_top_mm - z_bottom_mm

        slabs.append({
            'name':      f'pallet_capa_{layer_idx}',
            'center_m':  [pallet_cx_mm / 1000.0,
                          pallet_cy_mm / 1000.0,
                          z_center_mm  / 1000.0],
            'dims_m':    [pallet_dx_mm / 1000.0,
                          pallet_dy_mm / 1000.0,
                          max(dz_mm, 1.0) / 1000.0],  # at least 1mm thick
            'layer_idx': layer_idx,
        })
    return slabs


def should_remove_top_layer(current_z_robot_mm, previous_z_robot_mm,
                            box_height_mm, threshold_factor=0.7):
    """Determine if the top pallet layer has been cleared (robot Z dropped).

    When all boxes on the current top layer are picked, the next YOLO detection
    will show a lower Z (the next layer down). We detect this by comparing the
    current stable Z against the previous one.

    Args:
        current_z_robot_mm:  Current stable grasp Z in robot frame (mm).
        previous_z_robot_mm: Previous stable grasp Z in robot frame (mm).
        box_height_mm:       Height of each layer/box (mm).
        threshold_factor:    Fraction of box_height to trigger removal (0.7 default).
                             0.7 accounts for D435i depth noise (±40mm at 2m).

    Returns:
        True if the Z dropped more than threshold_factor * box_height_mm.
    """
    z_drop_mm = previous_z_robot_mm - current_z_robot_mm
    threshold_mm = threshold_factor * box_height_mm
    return z_drop_mm >= threshold_mm


def validate_config(config):
    """Validate a collision config dictionary.

    Checks:
        - 'objects' key exists
        - Exactly ONE object has type='floor'
        - Exactly ONE object has type='ceiling'
        - ceiling_z > floor_z
        - place_position_m has exactly 3 elements (if present)

    Args:
        config: dict — the collision calibration config.

    Returns:
        (is_valid, errors) where is_valid is bool and errors is list[str].
    """
    errors = []

    if 'objects' not in config:
        errors.append("Missing 'objects' key in config")
        return (False, errors)

    objects = config.get('objects', {})

    # Find floor and ceiling objects by type
    floor_objects = [(name, obj) for name, obj in objects.items() if obj.get('type') == 'floor']
    ceiling_objects = [(name, obj) for name, obj in objects.items() if obj.get('type') == 'ceiling']

    if len(floor_objects) == 0:
        errors.append("No floor-type object found (required: exactly 1)")
    elif len(floor_objects) > 1:
        names = [n for n, _ in floor_objects]
        errors.append(f"Multiple floor-type objects found: {names} (required: exactly 1)")

    if len(ceiling_objects) == 0:
        errors.append("No ceiling-type object found (required: exactly 1)")
    elif len(ceiling_objects) > 1:
        names = [n for n, _ in ceiling_objects]
        errors.append(f"Multiple ceiling-type objects found: {names} (required: exactly 1)")

    # Check ceiling_z > floor_z only if we have exactly one of each
    if len(floor_objects) == 1 and len(ceiling_objects) == 1:
        floor_z = floor_objects[0][1].get('z_mm')
        ceiling_z = ceiling_objects[0][1].get('z_mm')
        if floor_z is not None and ceiling_z is not None:
            if ceiling_z <= floor_z:
                errors.append(
                    f"ceiling_z ({ceiling_z}) must be greater than floor_z ({floor_z})"
                )

    # place_position_m
    place = config.get('place_position_m')
    if place is not None:
        if not isinstance(place, (list, tuple)) or len(place) != 3:
            errors.append(
                f"place_position_m must have 3 elements, got {len(place) if isinstance(place, (list, tuple)) else type(place).__name__}"
            )

    return (len(errors) == 0, errors)


# ---------------------------------------------------------------------------
# ROS2 Node — Interactive Collision Calibrator
# ---------------------------------------------------------------------------

import time
import json
import os
import tempfile
import datetime
import threading

try:
    import rclpy
    from rclpy.node import Node as _NodeBase
    from rclpy.executors import MultiThreadedExecutor
except ImportError:
    # Allow geometry-only imports (tests) without ROS2
    rclpy = None  # type: ignore[assignment]
    _NodeBase = object  # type: ignore[assignment,misc]
    MultiThreadedExecutor = None  # type: ignore[assignment,misc]


class CollisionCalibratorNode(_NodeBase):  # type: ignore[misc]
    """Interactive node for calibrating workspace collision objects."""

    def __init__(self):
        super().__init__('collision_calibrator')
        self._last_tcp = None
        self._tcp_time = 0.0

        # Subscribe to ToolVectorActual for TCP position (mm)
        from dobot_msgs_v4.msg import ToolVectorActual
        self._tcp_sub = self.create_subscription(
            ToolVectorActual,
            '/dobot_msgs_v4/msg/ToolVectorActual',
            self._tcp_callback,
            10
        )

        # Start interactive calibration in daemon thread
        t = threading.Thread(target=self._calibration_loop, daemon=True)
        t.start()

    # ── TCP helpers ───────────────────────────────────────────

    def _tcp_callback(self, msg):
        self._last_tcp = [msg.x, msg.y, msg.z]
        self._tcp_time = time.time()

    def get_tcp(self):
        """Get current TCP position in mm. Raises RuntimeError if no data or stale (>1s)."""
        if self._last_tcp is None:
            raise RuntimeError("Sin lectura TCP — ¿está publicando ToolVectorActual?")
        age = time.time() - self._tcp_time
        if age > 1.0:
            raise RuntimeError(f"Lectura TCP caducada ({age:.1f}s antigua)")
        return list(self._last_tcp)

    # ── UI helpers ────────────────────────────────────────────

    def _wait_enter(self, prompt=""):
        """Show prompt + live TCP, wait for ENTER. Returns tcp_at_press."""
        while True:
            tcp_str = ""
            try:
                tcp = self.get_tcp()
                tcp_str = f"    TCP actual: X={tcp[0]:.1f}  Y={tcp[1]:.1f}  Z={tcp[2]:.1f} mm"
            except RuntimeError:
                tcp_str = "    TCP actual: --- sin lectura ---"
            print(tcp_str)
            if prompt:
                print(f"    {prompt}")
            input("    >> Presiona ENTER para capturar... ")
            try:
                captured = self.get_tcp()
                return captured
            except RuntimeError as e:
                print(f"    ⚠️  Error al capturar: {e}")
                print("    Intenta de nuevo.\n")

    def _prompt_float(self, prompt, min_val=None, max_val=None):
        """Ask user for a float value with optional bounds. Loop until valid."""
        while True:
            try:
                raw = input(f"    {prompt}")
                val = float(raw.strip())
                if min_val is not None and val < min_val:
                    print(f"    ⚠️  Valor debe ser >= {min_val}")
                    continue
                if max_val is not None and val > max_val:
                    print(f"    ⚠️  Valor debe ser <= {max_val}")
                    continue
                return val
            except ValueError:
                print("    ⚠️  Introduce un número válido.")

    def _confirm(self, summary_lines):
        """Show summary_lines, ask confirmation. Return True/False."""
        print()
        for line in summary_lines:
            print(f"    {line}")
        while True:
            choice = input("    ¿Confirmar? (s/n): ").strip().lower()
            if choice in ('s', 'si', 'sí', 'y', 'yes'):
                return True
            if choice in ('n', 'no'):
                return False
            print("    ⚠️  Responde 's' o 'n'.")

    # ── Calibration sequence ──────────────────────────────────

    def _calibrate_suelo(self):
        """Calibrate floor plane."""
        print("\n    === SUELO (Plano del piso) ===")
        print("    Mueve el TCP al nivel del PISO y presiona ENTER...")
        tcp = self._wait_enter()
        print(f"    ✅ Suelo: Z = {tcp[2]:.1f} mm")
        return {'type': 'floor', 'z_mm': tcp[2]}

    def _calibrate_techo(self, suelo_z):
        """Calibrate ceiling plane. Validates techo_z > suelo_z."""
        while True:
            print("\n    === TECHO (Plano del techo) ===")
            print("    Mueve el TCP al nivel del TECHO y presiona ENTER...")
            tcp = self._wait_enter()
            if tcp[2] > suelo_z:
                print(f"    ✅ Techo: Z = {tcp[2]:.1f} mm")
                print(f"    Validación: techo_z ({tcp[2]:.1f}) > suelo_z ({suelo_z:.1f}) ✅")
                return {'type': 'ceiling', 'z_mm': tcp[2]}
            else:
                print(f"    ❌ Error: techo_z ({tcp[2]:.1f}) debe ser > suelo_z ({suelo_z:.1f})")
                print("    Intenta de nuevo.\n")

    def _calibrate_box(self, name, label):
        """Calibrate a box object (mesa, pallet, cinta). Returns dict or re-prompts."""
        while True:
            print(f"\n    === {label} ===")
            print("    ESQUINA 1: Mueve el TCP a la esquina diagonal 1 de la SUPERFICIE y ENTER...")
            p1 = self._wait_enter()
            print(f"    Esquina 1 capturada: [{p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.1f}] mm")

            print("    ESQUINA 2: Mueve el TCP a la ESQUINA OPUESTA diagonal de la superficie y ENTER...")
            p2 = self._wait_enter()
            print(f"    Esquina 2 capturada: [{p2[0]:.1f}, {p2[1]:.1f}, {p2[2]:.1f}] mm")

            height = self._prompt_float("¿Altura del objeto en mm? (ej: 800): ", min_val=50.0)

            try:
                center_m, dims_m = corners_to_box(p1, p2, height)
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                confirmed = self._confirm([
                    f"Dimensiones calculadas: Ancho={dx:.0f}mm, Prof={dy:.0f}mm, Alto={height:.0f}mm",
                    f"Centro (m): [{center_m[0]:.3f}, {center_m[1]:.3f}, {center_m[2]:.3f}]",
                    f"Tamaño (m): [{dims_m[0]:.3f}, {dims_m[1]:.3f}, {dims_m[2]:.3f}]",
                ])
                if confirmed:
                    print(f"    ✅ {name} registrado")
                    return {'type': 'box', 'corner1_mm': p1, 'corner2_mm': p2, 'height_mm': height}
                else:
                    print("    🔄 Repitiendo calibración...")
            except ValueError as e:
                print(f"    ❌ Error de validación: {e}")
                print("    Intenta de nuevo.\n")

    def _calibrate_camara(self, techo_z):
        """Calibrate camera support pole."""
        while True:
            print("\n    === SOPORTE DE CÁMARA (Palo vertical) ===")
            print("    Mueve el TCP a la parte INFERIOR del soporte (donde está la cámara) y ENTER...")
            bottom = self._wait_enter()
            print(f"    Punto inferior capturado: [{bottom[0]:.1f}, {bottom[1]:.1f}, {bottom[2]:.1f}] mm")

            width = self._prompt_float("¿Ancho del soporte en mm? (ej: 80): ", min_val=10.0)

            try:
                center_m, dims_m = pole_box(bottom, width, techo_z)
                h_mm = techo_z - bottom[2]
                confirmed = self._confirm([
                    f"Dimensiones: {width:.0f}mm x {width:.0f}mm, Altura={h_mm:.0f}mm (desde Z={bottom[2]:.1f} hasta techo Z={techo_z:.1f})",
                    f"Centro (m): [{center_m[0]:.3f}, {center_m[1]:.3f}, {center_m[2]:.3f}]",
                    f"Tamaño (m): [{dims_m[0]:.3f}, {dims_m[1]:.3f}, {dims_m[2]:.3f}]",
                ])
                if confirmed:
                    print("    ✅ camara_soporte registrado")
                    return {'type': 'pole', 'bottom_mm': bottom, 'width_mm': width}
                else:
                    print("    🔄 Repitiendo calibración...")
            except ValueError as e:
                print(f"    ❌ Error de validación: {e}")
                print("    Intenta de nuevo.\n")

    def _calibrate_pared(self, techo_z, suelo_z):
        """Calibrate wall."""
        while True:
            print("\n    === PARED ===")
            print("    Mueve el TCP al PUNTO 1 de la base de la pared y ENTER...")
            p1 = self._wait_enter()
            print(f"    Punto 1: [{p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.1f}] mm")

            print("    Mueve el TCP al PUNTO 2 de la base de la pared y ENTER...")
            p2 = self._wait_enter()
            print(f"    Punto 2: [{p2[0]:.1f}, {p2[1]:.1f}, {p2[2]:.1f}] mm")

            try:
                center_m, dims_m, rot = wall_box(p1, p2, techo_z, suelo_z)
                length_mm = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
                confirmed = self._confirm([
                    f"Longitud: {length_mm:.0f}mm | Desde suelo ({suelo_z:.1f}mm) hasta techo ({techo_z:.1f}mm) | Grosor: 100mm",
                    f"Centro (m): [{center_m[0]:.3f}, {center_m[1]:.3f}, {center_m[2]:.3f}]",
                    f"Rotación: {math.degrees(rot):.1f}°",
                ])
                if confirmed:
                    print("    ✅ pared registrada")
                    return {'type': 'wall', 'point1_mm': p1, 'point2_mm': p2, 'thickness_mm': 100}
                else:
                    print("    🔄 Repitiendo calibración...")
            except ValueError as e:
                print(f"    ❌ Error de validación: {e}")
                print("    Intenta de nuevo.\n")

    # ── Main calibration loop (runs in daemon thread) ─────────

    def _calibration_loop(self):
        """Menú interactivo de calibración de colisiones."""
        try:
            time.sleep(1.5)
            objects = {}  # Dict de nombre -> datos del objeto
            place_position = None

            # -- Selección de modo --
            print()
            print("    ╔═════════════════════════════════════════════════╗")
            print("    ║   CALIBRACIÓN DE COLISIONES — Dobot CR20       ║")
            print("    ╚═════════════════════════════════════════════════╝")
            print()
            print("    Mueve el robot con el teach pendant.")
            print("    Presiona ENTER para capturar la posición actual.")
            print()

            # Preguntar modo
            while True:
                print("    ¿Qué deseas hacer?")
                print("      1 = Nueva calibración (borrar config anterior)")
                print("      2 = Agregar objetos al config existente")
                mode = input("    Opción (1/2): ").strip()
                if mode in ('1', '2'):
                    break
                print("    ⚠️  Opción inválida.")

            config_path = os.path.expanduser('~/dobot_ws/collision_config.json')

            if mode == '2':
                # Cargar config existente
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            existing = json.load(f)
                        objects = existing.get('objects', {})
                        place_position_raw = existing.get('place_position_m')
                        if place_position_raw and len(place_position_raw) == 3:
                            place_position = list(place_position_raw)
                        print(f"\n    ✅ Config cargado: {len(objects)} objetos existentes")
                        for name, obj in objects.items():
                            print(f"       • {name} ({obj.get('type', '?')})")
                    except Exception as e:
                        print(f"\n    ❌ Error leyendo config: {e}")
                        print("    Iniciando calibración nueva...")
                        objects = {}
                else:
                    print("\n    ⚠️  No existe config previo. Iniciando calibración nueva...")
                    mode = '1'

            # -- Suelo y Techo: obligatorios en modo nuevo --
            if mode == '1':
                print()
                print("    PASO OBLIGATORIO: Calibrar Suelo y Techo")
                print()
                # Suelo
                suelo_data = self._calibrate_suelo()
                objects['suelo'] = suelo_data
                suelo_z = suelo_data['z_mm']
                # Techo
                techo_data = self._calibrate_techo(suelo_z)
                objects['techo'] = techo_data
                techo_z = techo_data['z_mm']
            else:
                # En modo agregar: obtener suelo_z y techo_z del config cargado
                suelo_z = None
                techo_z = None
                for name, obj in objects.items():
                    if obj.get('type') == 'floor':
                        suelo_z = obj.get('z_mm')
                    elif obj.get('type') == 'ceiling':
                        techo_z = obj.get('z_mm')

                # Ofrecer recalibrar suelo/techo si existen
                if suelo_z is not None or techo_z is not None:
                    print()
                    if suelo_z is not None:
                        print(f"    Suelo existente: Z = {suelo_z:.1f} mm")
                    if techo_z is not None:
                        print(f"    Techo existente: Z = {techo_z:.1f} mm")
                    recal = input("    ¿Recalibrar suelo/techo? (s/N): ").strip().lower()
                    if recal in ('s', 'si', 'sí', 'y', 'yes'):
                        # Eliminar floor/ceiling existentes
                        objects = {k: v for k, v in objects.items()
                                   if v.get('type') not in ('floor', 'ceiling')}
                        suelo_data = self._calibrate_suelo()
                        objects['suelo'] = suelo_data
                        suelo_z = suelo_data['z_mm']
                        techo_data = self._calibrate_techo(suelo_z)
                        objects['techo'] = techo_data
                        techo_z = techo_data['z_mm']

            # -- Menú principal --
            while True:
                print()
                print("    ╔═══════════════════════════════════════╗")
                print("    ║  CALIBRACIÓN DE COLISIONES — CR20    ║")
                print("    ╠═══════════════════════════════════════╣")
                print("    ║  1 = Añadir objeto                   ║")
                print("    ║  2 = Ver objetos calibrados          ║")
                print("    ║  3 = Eliminar objeto                 ║")
                print("    ║  4 = Definir posición de depósito    ║")
                print("    ║  5 = Guardar y salir                 ║")
                print("    ╚═══════════════════════════════════════╝")
                print(f"    Objetos calibrados: {len(objects)}")
                choice = input("\n    Opción: ").strip()

                # -- Opción 1: Añadir objeto --
                if choice == '1':
                    print()
                    print("    ¿Qué tipo de objeto?")
                    print("      1 = Pared (wall)")
                    print("      2 = Caja / Mesa / Superficie (box)")
                    print("      3 = Poste / Columna (pole)")
                    print("      4 = Suelo (floor)")
                    print("      5 = Techo (ceiling)")
                    print("      6 = Pallet (base del pallet)")
                    type_choice = input("    Tipo: ").strip()
                    if type_choice not in ('1', '2', '3', '4', '5', '6'):
                        print("    ⚠️  Tipo inválido.")
                        continue

                    # Pedir nombre
                    while True:
                        name = input("    Nombre del objeto (ej: pared_norte): ").strip()
                        if not name:
                            print("    ⚠️  El nombre no puede estar vacío.")
                            continue
                        if name in objects:
                            print(f"    ❌ Ya existe un objeto con ese nombre. Elimínalo primero (opción 3).")
                            break
                        break
                    else:
                        continue  # nombre duplicado, volver al menú

                    if name in objects:
                        continue  # duplicado rechazado

                    # Calibrar según tipo
                    try:
                        if type_choice == '1':  # wall
                            if techo_z is None or suelo_z is None:
                                print("    ⚠️  Se necesitan suelo y techo para calibrar paredes.")
                                continue
                            data = self._calibrate_pared(techo_z, suelo_z)
                        elif type_choice == '2':  # box
                            data = self._calibrate_box(name, name.upper())
                        elif type_choice == '3':  # pole
                            if techo_z is None:
                                print("    ⚠️  Se necesita techo para calibrar postes.")
                                continue
                            data = self._calibrate_camara(techo_z)
                        elif type_choice == '4':  # floor
                            floor_exists = any(v.get('type') == 'floor' for v in objects.values())
                            if floor_exists:
                                print("    ❌ Ya hay un objeto tipo suelo. Elimínalo primero (opción 3).")
                                continue
                            data = self._calibrate_suelo()
                            suelo_z = data['z_mm']
                        elif type_choice == '5':  # ceiling
                            ceil_exists = any(v.get('type') == 'ceiling' for v in objects.values())
                            if ceil_exists:
                                print("    ❌ Ya hay un objeto tipo techo. Elimínalo primero (opción 3).")
                                continue
                            if suelo_z is None:
                                print("    ⚠️  Calibra el suelo primero.")
                                continue
                            data = self._calibrate_techo(suelo_z)
                            techo_z = data['z_mm']
                        elif type_choice == '6':  # pallet
                            # Check: only one pallet allowed
                            pallet_exists = any(v.get('type') == 'pallet' for v in objects.values())
                            if pallet_exists:
                                print("    ❌ Ya hay un pallet calibrado. Elimínalo primero (opción 3).")
                                continue
                            # Force name to 'pallet'
                            name = 'pallet'
                            print()
                            print("    ⚠️  IMPORTANTE: El pallet debe estar COMPLETAMENTE VACÍO")
                            print("    ⚠️  (sin cajas) durante esta calibración.")
                            print()
                            data = self._calibrate_box('pallet', 'PALLET (BASE VACÍO)')
                            data['type'] = 'pallet'  # Override the 'box' type set by _calibrate_box
                            objects[name] = data
                            print(f"\n    ✅ 'pallet' calibrado (type=pallet)")
                            continue  # Skip the generic 'objects[name] = data' at the bottom

                        objects[name] = data
                        print(f"\n    ✅ '{name}' añadido ({data.get('type', '?')})")
                    except (KeyboardInterrupt, SystemExit):
                        print("\n    ⚠️  Calibración de objeto cancelada.")
                        continue

                # -- Opción 2: Ver objetos --
                elif choice == '2':
                    print()
                    if not objects:
                        print("    (sin objetos calibrados)")
                    else:
                        for i, (name, obj) in enumerate(objects.items(), 1):
                            otype = obj.get('type', '?')
                            if otype == 'floor':
                                detail = f"Z = {obj.get('z_mm', '?'):.1f} mm"
                            elif otype == 'ceiling':
                                detail = f"Z = {obj.get('z_mm', '?'):.1f} mm"
                            elif otype == 'box':
                                p1 = obj.get('corner1_mm', [0, 0, 0])
                                p2 = obj.get('corner2_mm', [0, 0, 0])
                                detail = f"esquinas [{p1[0]:.0f},{p1[1]:.0f}] -> [{p2[0]:.0f},{p2[1]:.0f}], h={obj.get('height_mm', 0):.0f}mm"
                            elif otype == 'pole':
                                b = obj.get('bottom_mm', [0, 0, 0])
                                detail = f"base=[{b[0]:.0f},{b[1]:.0f},{b[2]:.0f}], w={obj.get('width_mm', 0):.0f}mm"
                            elif otype == 'wall':
                                p1 = obj.get('point1_mm', [0, 0, 0])
                                p2 = obj.get('point2_mm', [0, 0, 0])
                                detail = f"[{p1[0]:.0f},{p1[1]:.0f}] -> [{p2[0]:.0f},{p2[1]:.0f}]"
                            else:
                                detail = str(obj)
                            print(f"    {i}. {name} ({otype}): {detail}")
                    if place_position:
                        print(f"\n    Place position: [{place_position[0]:.4f}, {place_position[1]:.4f}, {place_position[2]:.4f}] m")

                # -- Opción 3: Eliminar objeto --
                elif choice == '3':
                    if not objects:
                        print("    (sin objetos para eliminar)")
                        continue
                    print()
                    items = list(objects.items())
                    for i, (name, obj) in enumerate(items, 1):
                        print(f"    {i}. {name} ({obj.get('type', '?')})")
                    sel = input("    Número a eliminar (ENTER para cancelar): ").strip()
                    if not sel:
                        continue
                    try:
                        idx = int(sel) - 1
                        if 0 <= idx < len(items):
                            name_del, obj_del = items[idx]
                            # Verificar dependencias: no eliminar floor/ceiling si hay walls/poles
                            obj_type = obj_del.get('type')
                            if obj_type == 'floor':
                                has_walls_poles = any(
                                    v.get('type') in ('wall', 'pole') for v in objects.values()
                                )
                                if has_walls_poles:
                                    print("    ❌ No se puede eliminar el suelo mientras haya paredes o postes.")
                                    continue
                                suelo_z = None
                            elif obj_type == 'ceiling':
                                has_poles = any(v.get('type') == 'pole' for v in objects.values())
                                if has_poles:
                                    print("    ❌ No se puede eliminar el techo mientras haya postes.")
                                    continue
                                has_walls = any(v.get('type') == 'wall' for v in objects.values())
                                if has_walls:
                                    print("    ❌ No se puede eliminar el techo mientras haya paredes.")
                                    continue
                                techo_z = None
                            del objects[name_del]
                            print(f"    ✅ '{name_del}' eliminado.")
                        else:
                            print("    ⚠️  Número fuera de rango.")
                    except ValueError:
                        print("    ⚠️  Entrada inválida.")

                # -- Opción 4: Place position --
                elif choice == '4':
                    boxes = [(name, obj) for name, obj in objects.items() if obj.get('type') == 'box']
                    if boxes:
                        print()
                        print("    Objetos tipo caja disponibles:")
                        for i, (name, _) in enumerate(boxes, 1):
                            print(f"    {i}. {name}")
                        sel = input("    ¿Cuál es la superficie de depósito? (número): ").strip()
                        try:
                            idx = int(sel) - 1
                            if 0 <= idx < len(boxes):
                                chosen_name, chosen_obj = boxes[idx]
                                place_position = derive_place_position(
                                    chosen_obj['corner1_mm'],
                                    chosen_obj['corner2_mm'],
                                    chosen_obj.get('height_mm', 0),
                                    offset_mm=20.0
                                )
                                print(f"\n    ✅ Place position calculada de '{chosen_name}':")
                                print(f"       [{place_position[0]:.4f}, {place_position[1]:.4f}, {place_position[2]:.4f}] m")
                            else:
                                print("    ⚠️  Número fuera de rango.")
                        except (ValueError, KeyError) as e:
                            print(f"    ⚠️  Error: {e}")
                    else:
                        # Captura manual
                        print()
                        print("    No hay cajas calibradas. Captura manual con TCP:")
                        try:
                            tcp = self._wait_enter("Mueve el TCP a la posición de depósito y ENTER...")
                            place_position = [tcp[0] / 1000.0, tcp[1] / 1000.0, tcp[2] / 1000.0]
                            print(f"\n    ✅ Place position capturada:")
                            print(f"       [{place_position[0]:.4f}, {place_position[1]:.4f}, {place_position[2]:.4f}] m")
                        except Exception as e:
                            print(f"    ❌ Error capturando TCP: {e}")

                # -- Opción 5: Guardar y salir --
                elif choice == '5':
                    config = {
                        'status': 'calibrated',
                        'timestamp': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                        'units': 'mm',
                        'objects': objects,
                    }
                    if place_position:
                        config['place_position_m'] = place_position
                        config['place_position_note'] = 'Posición de depósito en METROS.'

                    # Validar
                    is_valid, errs = validate_config(config)
                    if not is_valid:
                        print("\n    ❌ Validación fallida:")
                        for e in errs:
                            print(f"       - {e}")
                        print("    Añade los objetos necesarios antes de guardar.")
                        continue

                    # Guardar (atomic)
                    os.makedirs(os.path.dirname(config_path) if os.path.dirname(config_path) else '.', exist_ok=True)
                    with tempfile.NamedTemporaryFile('w', dir=os.path.dirname(os.path.abspath(config_path)),
                                                     suffix='.tmp', delete=False) as f:
                        json.dump(config, f, indent=2)
                        tmp_path = f.name
                    os.rename(tmp_path, config_path)

                    # Resumen
                    print()
                    print("    ╔═════════════════════════════════════════════════╗")
                    print("    ║           CALIBRACIÓN COMPLETADA ✅             ║")
                    print("    ╚═════════════════════════════════════════════════╝")
                    print()
                    print(f"    Archivo: {config_path}")
                    print(f"    Timestamp: {config['timestamp']}")
                    print()
                    print("    Objetos calibrados:")
                    for obj_name, obj in objects.items():
                        otype = obj.get('type', '?')
                        if otype in ('floor', 'ceiling'):
                            print(f"      • {obj_name}: Z = {obj['z_mm']:.1f} mm")
                        elif otype == 'box':
                            print(f"      • {obj_name}: caja")
                        elif otype == 'pole':
                            print(f"      • {obj_name}: poste")
                        elif otype == 'wall':
                            print(f"      • {obj_name}: pared")
                    if place_position:
                        print(f"\n    Place position (m): {place_position}")
                    print()
                    raise SystemExit(0)

                else:
                    print("    ⚠️  Opción inválida. Usa 1-5.")

        except KeyboardInterrupt:
            print("\n\n    ⚠️  Calibración interrumpida — NO se guardó ningún archivo.")
            raise SystemExit(0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    import rclpy as _rclpy
    from rclpy.executors import MultiThreadedExecutor as _MTE
    _rclpy.init(args=args)
    node = None
    try:
        node = CollisionCalibratorNode()
        executor = _MTE(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            _rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
