#!/usr/bin/env python3
"""
Filtrado de señal y detección de gestos para Hand Tracking
Versión 2.0 - Detección robusta con:
  - Ángulos entre segmentos de dedo (más preciso que distancias)
  - Histéresis (evita parpadeo entre gestos)
  - Confianza por gesto (0.0 - 1.0)
  - Suavizado temporal de gestos
  - Dead zone para micro-movimientos
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple
from collections import deque
from hand_tracking_bridge.hand_data_parser import HandData, WristPose, FrameData


# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES Y DEFINICIONES
# ─────────────────────────────────────────────────────────────────────────────

# Índices de cada articulación por dedo
FINGER_JOINTS = {
    'thumb':  [1, 2, 3, 4],
    'index':  [5, 6, 7, 8],
    'middle': [9, 10, 11, 12],
    'ring':   [13, 14, 15, 16],
    'pinky':  [17, 18, 19, 20],
}

# Wrist = índice 0
WRIST_IDX = 0

# Tips de cada dedo
TIPS = {
    'thumb': 4, 'index': 8, 'middle': 12, 'ring': 16, 'pinky': 20
}

# MCPs (nudillos / base de cada dedo)
MCPS = {
    'thumb': 2, 'index': 5, 'middle': 9, 'ring': 13, 'pinky': 17
}

# PIPs (articulación media)
PIPS = {
    'thumb': 3, 'index': 6, 'middle': 10, 'ring': 14, 'pinky': 18
}


# ─────────────────────────────────────────────────────────────────────────────
# UTILIDADES MATEMÁTICAS
# ─────────────────────────────────────────────────────────────────────────────

def vec(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Vector de a hacia b"""
    return b - a


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """
    Ángulo en grados entre dos vectores (0° = paralelos, 180° = opuestos)
    Robusto a vectores de magnitud cero.
    """
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_a)))


def angle_at_joint(prev_pt: np.ndarray,
                   joint_pt: np.ndarray,
                   next_pt: np.ndarray) -> float:
    """
    Ángulo de flexión en una articulación (grados).
    0° = completamente extendido
    180° = completamente flexionado
    """
    v_in  = vec(prev_pt, joint_pt)
    v_out = vec(joint_pt, next_pt)
    return angle_between(v_in, v_out)


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-8 else v


# ─────────────────────────────────────────────────────────────────────────────
# ESTADO DE GESTOS
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class FingerState:
    """Estado de un dedo individual"""
    name: str
    # Ángulos en cada articulación (grados)
    mcp_angle: float = 0.0     # Nudillo
    pip_angle: float = 0.0     # Articulación media
    # Grado de extensión: 0.0 = cerrado, 1.0 = extendido
    extension: float = 0.0
    # ¿El dedo está extendido?
    is_extended: bool = False
    # ¿El dedo está flexionado?
    is_flexed: bool = False


@dataclass
class GestureState:
    """Estado completo de gestos de una mano"""
    # Dedos individuales
    fingers: Dict[str, FingerState] = field(default_factory=dict)

    # Gestos detectados (con confianza)
    pinch:      float = 0.0   # 0.0-1.0
    grip:       float = 0.0
    open_hand:  float = 0.0
    point:      float = 0.0
    thumbs_up:  float = 0.0

    # Distancia y fuerza de pinch
    pinch_distance: float = 0.1
    pinch_strength: float = 0.0

    # Gesto dominante
    gesture_name: str = "---"
    gesture_confidence: float = 0.0

    def __str__(self):
        ext = " ".join(
            f"{k[0].upper()}:{'E' if v.is_extended else 'F'}"
            for k, v in self.fingers.items()
        )
        return (f"[{self.gesture_name} {self.gesture_confidence:.0%}] "
                f"Pinch:{self.pinch_strength:.0%} | {ext}")


# ─────────────────────────────────────────────────────────────────────────────
# FILTROS DE SEÑAL
# ─────────────────────────────────────────────────────────────────────────────

class EMAFilter:
    """
    Filtro de suavizado exponencial (Exponential Moving Average)
    alpha ∈ (0, 1]:
      - Alto (0.8-1.0) = poca suavización, más reactivo
      - Bajo (0.1-0.3) = mucha suavización, más lento
    Recomendado para hand tracking: 0.35-0.50
    """

    def __init__(self, alpha: float = 0.4):
        self.alpha = float(np.clip(alpha, 0.01, 1.0))
        self._prev = None

    def update(self, value: np.ndarray) -> np.ndarray:
        if self._prev is None:
            self._prev = value.copy()
            return value.copy()
        out = self.alpha * value + (1.0 - self.alpha) * self._prev
        self._prev = out.copy()
        return out

    def reset(self):
        self._prev = None


class DeadZoneFilter:
    """
    Bloquea actualizaciones por debajo de 'threshold' metros.
    Elimina el temblor cuando la mano está quieta.
    """

    def __init__(self, threshold: float = 0.0015):
        self.threshold = threshold
        self._prev = None

    def update(self, value: np.ndarray) -> np.ndarray:
        if self._prev is None:
            self._prev = value.copy()
            return value.copy()
        if np.linalg.norm(value - self._prev) < self.threshold:
            return self._prev.copy()
        self._prev = value.copy()
        return value.copy()


class HysteresisFilter:
    """
    Evita que un valor booleano parpadee entre True/False.
    Sube rápido, baja lento (o configurable).
    """

    def __init__(self,
                 rise_alpha: float = 0.7,
                 fall_alpha: float = 0.3,
                 on_threshold: float = 0.6,
                 off_threshold: float = 0.35):
        self.rise_alpha = rise_alpha
        self.fall_alpha = fall_alpha
        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self._smooth = 0.0
        self._state = False

    def update(self, raw_confidence: float) -> Tuple[bool, float]:
        """
        raw_confidence: valor entre 0 y 1 de qué tan seguro es el gesto
        Retorna: (estado_booleano, confianza_suavizada)
        """
        alpha = self.rise_alpha if raw_confidence > self._smooth else self.fall_alpha
        self._smooth = alpha * raw_confidence + (1.0 - alpha) * self._smooth
        self._smooth = float(np.clip(self._smooth, 0.0, 1.0))

        if not self._state and self._smooth >= self.on_threshold:
            self._state = True
        elif self._state and self._smooth <= self.off_threshold:
            self._state = False

        return self._state, self._smooth


class HandSmoother:
    """
    Suaviza todos los datos de una mano:
    - Posición de muñeca (EMA + dead zone)
    - Quaternion de orientación (EMA + renormalización)
    - 21 landmarks (EMA individual)
    """

    def __init__(self, alpha: float = 0.4, dead_zone: float = 0.0015):
        self.wrist_ema = EMAFilter(alpha)
        self.wrist_dz  = DeadZoneFilter(dead_zone)
        self.quat_ema  = EMAFilter(min(alpha + 0.1, 1.0))   # quaternion puede ser más ágil
        self.lm_ema    = [EMAFilter(alpha) for _ in range(21)]

    def smooth(self, hand: HandData) -> HandData:
        # Posición
        pos = self.wrist_ema.update(hand.wrist.position)
        pos = self.wrist_dz.update(pos)

        # Quaternion (EMA + normalización para mantenerlo unitario)
        q = self.quat_ema.update(hand.wrist.orientation)
        q_norm = np.linalg.norm(q)
        if q_norm > 1e-8:
            q = q / q_norm

        # Landmarks
        lm = np.array([
            self.lm_ema[i].update(hand.landmarks[i])
            for i in range(21)
        ])

        return HandData(WristPose(pos, q), lm)


# ─────────────────────────────────────────────────────────────────────────────
# DETECCIÓN DE DEDOS
# ─────────────────────────────────────────────────────────────────────────────

class FingerAnalyzer:
    """
    Analiza el estado de cada dedo usando ángulos entre segmentos.
    Más robusto que métodos basados en distancias.
    """

    # Umbral de ángulo: por encima = flexionado
    FLEX_THRESHOLD_MCP = 35.0   # grados en el nudillo
    FLEX_THRESHOLD_PIP = 40.0   # grados en articulación media

    # Umbrales para clasificar como extendido / flexionado
    EXTEND_THRESHOLD = 0.60     # extension score
    FLEX_THRESHOLD   = 0.35

    def analyze_finger(self, name: str, landmarks: np.ndarray) -> FingerState:
        """Analiza un dedo y retorna su estado"""
        joints = FINGER_JOINTS[name]
        state = FingerState(name=name)

        lm = landmarks   # alias corto

        if name == 'thumb':
            # El pulgar tiene cinemática diferente
            # Usamos ángulo en CMC y MCP
            state.mcp_angle = angle_at_joint(
                lm[joints[0]], lm[joints[1]], lm[joints[2]]
            )
            state.pip_angle = angle_at_joint(
                lm[joints[1]], lm[joints[2]], lm[joints[3]]
            )
        else:
            # Dedos normales: ángulo en MCP y PIP
            state.mcp_angle = angle_at_joint(
                lm[WRIST_IDX], lm[joints[0]], lm[joints[1]]
            )
            state.pip_angle = angle_at_joint(
                lm[joints[0]], lm[joints[1]], lm[joints[2]]
            )

        # Score de extensión combinado
        # Normalizar ángulos: 0° = extendido, ~90°+ = flexionado
        mcp_flex = np.clip(state.mcp_angle / 90.0, 0.0, 1.0)
        pip_flex = np.clip(state.pip_angle / 90.0, 0.0, 1.0)

        # Pesos: PIP tiene más peso porque distingue mejor flexión
        flex_score = 0.35 * mcp_flex + 0.65 * pip_flex
        state.extension = float(1.0 - flex_score)

        state.is_extended = state.extension > self.EXTEND_THRESHOLD
        state.is_flexed   = state.extension < self.FLEX_THRESHOLD

        return state

    def analyze_all(self, landmarks: np.ndarray) -> Dict[str, FingerState]:
        """Analiza todos los dedos"""
        return {
            name: self.analyze_finger(name, landmarks)
            for name in FINGER_JOINTS
        }


# ─────────────────────────────────────────────────────────────────────────────
# DETECTOR DE GESTOS
# ─────────────────────────────────────────────────────────────────────────────

class GestureDetector:
    """
    Detecta gestos usando análisis de ángulos + histéresis.
    
    Cada gesto retorna una confianza entre 0.0 y 1.0.
    La histéresis evita parpadeos entre estados.
    """

    PINCH_CLOSE = 0.025   # metros: distancia mínima de pinch cerrado
    PINCH_OPEN  = 0.085   # metros: distancia máxima de pinch abierto

    def __init__(self):
        self.finger_analyzer = FingerAnalyzer()

        # Filtros de histéresis por gesto (uno por mano, pero
        # se crean aquí y GestureDetector se instancia por mano)
        self._pinch_hys     = HysteresisFilter(0.75, 0.25, 0.60, 0.35)
        self._grip_hys      = HysteresisFilter(0.70, 0.30, 0.55, 0.30)
        self._open_hys      = HysteresisFilter(0.65, 0.35, 0.60, 0.40)
        self._point_hys     = HysteresisFilter(0.75, 0.25, 0.60, 0.35)
        self._thumbsup_hys  = HysteresisFilter(0.70, 0.30, 0.60, 0.35)

    def detect(self, hand: HandData) -> GestureState:
        """Detectar todos los gestos de una mano"""
        lm = hand.landmarks
        state = GestureState()

        # ── Análisis de dedos ────────────────────────────────────────────────
        state.fingers = self.finger_analyzer.analyze_all(lm)
        f = state.fingers   # alias

        # Extensión individual
        ext = {name: f[name].is_extended for name in f}
        flx = {name: f[name].is_flexed   for name in f}
        ext_score = {name: f[name].extension for name in f}

        # ── Pinch ────────────────────────────────────────────────────────────
        thumb_tip = lm[TIPS['thumb']]
        index_tip = lm[TIPS['index']]
        state.pinch_distance = float(np.linalg.norm(thumb_tip - index_tip))

        # Confianza raw del pinch basada en distancia
        raw_pinch = np.clip(
            1.0 - (state.pinch_distance - self.PINCH_CLOSE) /
            (self.PINCH_OPEN - self.PINCH_CLOSE),
            0.0, 1.0
        )
        # Boost si los ángulos del índice también sugieren flexión
        if flx['index']:
            raw_pinch = min(1.0, raw_pinch * 1.15)

        state.pinch, state.pinch_strength = self._pinch_hys.update(raw_pinch)

        # ── Grip ─────────────────────────────────────────────────────────────
        # Todos los dedos (sin pulgar) flexionados + pulgar flexionado
        fingers_flexed = sum(
            1 for name in ['index', 'middle', 'ring', 'pinky']
            if flx[name]
        )
        raw_grip = fingers_flexed / 4.0
        # Bonus si el pulgar también está flexionado
        if flx['thumb']:
            raw_grip = min(1.0, raw_grip + 0.1)
        state.grip, _ = self._grip_hys.update(raw_grip)

        # ── Open hand ────────────────────────────────────────────────────────
        fingers_extended = sum(
            1 for name in ['index', 'middle', 'ring', 'pinky']
            if ext[name]
        )
        raw_open = fingers_extended / 4.0
        # Requiere también que el pulgar esté relativamente extendido
        if ext_score['thumb'] > 0.45:
            raw_open = min(1.0, raw_open + 0.15)
        state.open_hand, _ = self._open_hys.update(raw_open)

        # ── Point ────────────────────────────────────────────────────────────
        # Solo índice extendido, el resto flexionado
        raw_point = 0.0
        if ext['index']:
            flex_others = sum(
                1 for name in ['middle', 'ring', 'pinky']
                if flx[name]
            )
            raw_point = flex_others / 3.0
            # Refuerzo si el índice está muy extendido
            if ext_score['index'] > 0.75:
                raw_point = min(1.0, raw_point + 0.1)
        state.point, _ = self._point_hys.update(raw_point)

        # ── Thumbs up ────────────────────────────────────────────────────────
        # Pulgar extendido hacia arriba + el resto flexionado
        raw_thumb = 0.0
        if ext['thumb']:
            flex_others = sum(
                1 for name in ['index', 'middle', 'ring', 'pinky']
                if flx[name]
            )
            raw_thumb = flex_others / 4.0
            # Verificar que el pulgar apunta hacia arriba
            # (landmark[4] Y > landmark[2] Y en espacio local)
            if lm[TIPS['thumb']][1] > lm[MCPS['thumb']][1]:
                raw_thumb = min(1.0, raw_thumb + 0.15)
        state.thumbs_up, _ = self._thumbsup_hys.update(raw_thumb)

        # ── Gesto dominante ─────────────────────────────────────────────────
        # Prioridad y confianza
        candidates = [
            ('PINCH',     state.pinch_strength,  state.pinch),
            ('GRIP',      state.grip,             state.grip > 0.5),
            ('POINT',     state.point,            state.point > 0.5),
            ('THUMBS_UP', state.thumbs_up,        state.thumbs_up > 0.5),
            ('OPEN',      state.open_hand,        state.open_hand > 0.5),
        ]

        # Filtrar solo los activos y ordenar por confianza
        active = [(name, conf) for name, conf, active in candidates if active]
        if active:
            active.sort(key=lambda x: x[1], reverse=True)
            state.gesture_name = active[0][0]
            state.gesture_confidence = active[0][1]
        else:
            state.gesture_name = "---"
            state.gesture_confidence = 0.0

        return state


# ─────────────────────────────────────────────────────────────────────────────
# SISTEMA INTEGRADO
# ─────────────────────────────────────────────────────────────────────────────

class FilteredHandTracker:
    """
    Pipeline completo:
    Raw frame → Smooth → Detect gestures → Filtered frame + GestureState
    Solo procesa la mano DERECHA
    """

    def __init__(self, alpha: float = 0.4, dead_zone: float = 0.0015):
        # Solo instanciamos para mano derecha
        self.right_smoother  = HandSmoother(alpha, dead_zone)
        self.right_detector  = GestureDetector()
        self.right_gestures  = GestureState()

    def process(self, frame: FrameData) -> FrameData:
        """
        Procesa SOLO la mano derecha.
        La mano izquierda se ignora completamente.
        """
        right_out = None

        if frame.right_hand:
            smoothed = self.right_smoother.smooth(frame.right_hand)
            self.right_gestures = self.right_detector.detect(smoothed)
            right_out = smoothed

        # left_hand siempre None
        return FrameData(frame.timestamp, None, right_out)


# ─────────────────────────────────────────────────────────────────────────────
# TEST EN CONSOLA
# ─────────────────────────────────────────────────────────────────────────────

def _bar(value: float, width: int = 12) -> str:
    """Barra de progreso ASCII"""
    filled = int(np.clip(value, 0.0, 1.0) * width)
    return '█' * filled + '░' * (width - filled)


def main():
    from hand_tracking_bridge.hand_data_parser import HandTrackingReceiver

    receiver = HandTrackingReceiver(port=7777, timeout=5.0)
    tracker  = FilteredHandTracker(alpha=0.4)

    print("=" * 65)
    print("  HAND FILTERS v2.0 — Solo mano DERECHA")
    print("=" * 65)
    print("  Gestos soportados:")
    print("    ✋  OPEN      = mano abierta")
    print("    👌  PINCH     = pulgar + índice juntos")
    print("    ✊  GRIP      = puño cerrado")
    print("    👆  POINT     = solo índice extendido")
    print("    👍  THUMBS_UP = pulgar arriba")
    print("=" * 65)
    print()

    frame_count = 0

    try:
        for frame in receiver.stream_frames():
            filtered = tracker.process(frame)
            frame_count += 1

            # Solo mostrar si hay mano derecha
            if filtered.right_hand is None:
                continue

            # Mostrar cada 3 frames
            if frame_count % 3 != 0:
                continue

            gesture = tracker.right_gestures
            f = gesture.fingers

            # Extensión por dedo
            deds = "".join(
                "▲" if f[n].is_extended else ("▼" if f[n].is_flexed else "─")
                for n in ['thumb', 'index', 'middle', 'ring', 'pinky']
            )

            output = (
                f"\r  DER 👌 | "
                f"Dedos:[{deds}] | "
                f"{gesture.gesture_name:9s} {gesture.gesture_confidence:.0%} | "
                f"Pinch:{_bar(gesture.pinch_strength)} "
                f"{gesture.pinch_strength:.0%}\033[K"
            )
            print(output, end="", flush=True)

    except KeyboardInterrupt:
        print(f"\n\n📊 Frames procesados: {frame_count}")


if __name__ == "__main__":
    main()
