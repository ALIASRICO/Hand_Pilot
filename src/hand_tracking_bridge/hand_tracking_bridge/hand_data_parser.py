#!/usr/bin/env python3
"""
Parser completo para Hand Tracking Streamer
Convierte datos del Quest a estructuras Python limpias
"""

import socket
import re
import time
from dataclasses import dataclass
from typing import Optional, List
import numpy as np

# --- ESTRUCTURAS DE DATOS ---

@dataclass
class WristPose:
    """Pose 6-DoF de la muñeca"""
    position: np.ndarray  # [x, y, z] en metros
    orientation: np.ndarray  # [qx, qy, qz, qw] quaternion
    
    def __str__(self):
        p = self.position
        q = self.orientation
        return f"Pos:[{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}] Rot:[{q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f}]"

@dataclass
class HandData:
    """Datos completos de una mano"""
    wrist: WristPose
    landmarks: np.ndarray  # [21, 3] array de landmarks
    
    def get_landmark(self, index: int) -> np.ndarray:
        """Obtener landmark por índice (0=wrist, 1-4=thumb, 5-8=index, etc)"""
        return self.landmarks[index]
    
    def __str__(self):
        return f"Wrist: {self.wrist}, Landmarks: {self.landmarks.shape}"

@dataclass
class FrameData:
    """Frame completo con ambas manos"""
    timestamp: float  # en segundos desde inicio
    left_hand: Optional[HandData]
    right_hand: Optional[HandData]
    
    def __str__(self):
        left_str = "✓" if self.left_hand else "✗"
        right_str = "✓" if self.right_hand else "✗"
        return f"[t={self.timestamp:.3f}s] Left:{left_str} Right:{right_str}"

# --- PARSER ---

class HandTrackingParser:
    def __init__(self):
        self.start_time = None
        self.left_wrist = None
        self.left_landmarks = None
        self.right_wrist = None
        self.right_landmarks = None
        
    def parse_line(self, line: str) -> List[tuple]:
        """
        Parsea una línea del Quest que puede contener múltiples paquetes
        Returns: lista de (hand_side, data_type, values)
        """
        # Convertir coma a punto (problema del locale)
        line_fixed = line.replace(',', '.')
        
        results = []
        
        # Buscar todos los paquetes en la línea
        # Pueden venir varios concatenados
        wrist_pattern = r'(Left|Right)\s+wrist:'
        landmarks_pattern = r'(Left|Right)\s+landmarks:'
        
        # Extraer todos los números de la línea
        all_numbers = re.findall(r'-?\d+\.\d+', line_fixed)
        
        # Buscar paquetes de wrist
        for match in re.finditer(wrist_pattern, line):
            hand_side = match.group(1).lower()
            start_pos = match.end()
            
            # Los primeros 7 números después de "wrist:" son los datos de wrist
            # Buscar en el substring después del match
            substring = line_fixed[start_pos:]
            wrist_numbers = re.findall(r'-?\d+\.\d+', substring)[:7]
            
            if len(wrist_numbers) == 7:
                values = [float(n) for n in wrist_numbers]
                results.append((hand_side, 'wrist', values))
        
        # Buscar paquetes de landmarks
        for match in re.finditer(landmarks_pattern, line):
            hand_side = match.group(1).lower()
            start_pos = match.end()
            
            # Los siguientes 63 números son landmarks (21 puntos × 3)
            substring = line_fixed[start_pos:]
            landmark_numbers = re.findall(r'-?\d+\.\d+', substring)[:63]
            
            if len(landmark_numbers) >= 60:  # Al menos 20 landmarks
                values = [float(n) for n in landmark_numbers[:63]]
                results.append((hand_side, 'landmarks', values))
        
        return results
    
    def process_line(self, line: str) -> Optional[FrameData]:
        """
        Procesa una línea y retorna FrameData si está completo
        """
        if self.start_time is None:
            self.start_time = time.time()
        
        parsed_data = self.parse_line(line)
        
        for hand_side, data_type, values in parsed_data:
            if hand_side == 'left':
                if data_type == 'wrist' and len(values) >= 7:
                    position = np.array(values[0:3])
                    orientation = np.array(values[3:7])
                    self.left_wrist = WristPose(position, orientation)
                    
                elif data_type == 'landmarks' and len(values) >= 63:
                    # Reshape a 21 puntos × 3 coordenadas
                    landmarks = np.array(values[0:63]).reshape(21, 3)
                    self.left_landmarks = landmarks
                    
            elif hand_side == 'right':
                if data_type == 'wrist' and len(values) >= 7:
                    position = np.array(values[0:3])
                    orientation = np.array(values[3:7])
                    self.right_wrist = WristPose(position, orientation)
                    
                elif data_type == 'landmarks' and len(values) >= 63:
                    landmarks = np.array(values[0:63]).reshape(21, 3)
                    self.right_landmarks = landmarks
        
        # Intentar ensamblar frame completo
        return self.try_assemble_frame()
    
    def try_assemble_frame(self) -> Optional[FrameData]:
        """
        Intenta ensamblar un frame completo si hay datos suficientes
        """
        # Solo ensambla si al menos una mano está completa
        left_complete = self.left_wrist is not None and self.left_landmarks is not None
        right_complete = self.right_wrist is not None and self.right_landmarks is not None
        
        if not (left_complete or right_complete):
            return None
        
        # Crear HandData para cada mano completa
        left_hand = None
        if left_complete:
            left_hand = HandData(self.left_wrist, self.left_landmarks)
            # Reset para próximo frame
            self.left_wrist = None
            self.left_landmarks = None
        
        right_hand = None
        if right_complete:
            right_hand = HandData(self.right_wrist, self.right_landmarks)
            # Reset para próximo frame
            self.right_wrist = None
            self.right_landmarks = None
        
        timestamp = time.time() - self.start_time
        return FrameData(timestamp, left_hand, right_hand)

# --- RECEPTOR UDP ---

class HandTrackingReceiver:
    def __init__(self, port=7777, timeout=5.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(timeout)
        self.parser = HandTrackingParser()
        print(f"✅ Receptor UDP escuchando en puerto {port}")
        
    def stream_frames(self):
        """Generator que yield frames completos"""
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                line = data.decode('utf-8').strip()
                
                # Procesar línea
                frame = self.parser.process_line(line)
                
                # Si hay frame completo, yieldarlo
                if frame:
                    yield frame
                    
            except socket.timeout:
                print("⏳ Timeout - sin datos del Quest")
            except KeyboardInterrupt:
                print("\n⏹️ Detenido por el usuario")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

# --- LANDMARK NAMES (para referencia) ---

LANDMARK_NAMES = [
    "Wrist",           # 0
    "Thumb_0",         # 1
    "Thumb_1",         # 2
    "Thumb_2",         # 3
    "Thumb_3",         # 4 (tip)
    "Index_0",         # 5
    "Index_1",         # 6
    "Index_2",         # 7
    "Index_3",         # 8 (tip)
    "Middle_0",        # 9
    "Middle_1",        # 10
    "Middle_2",        # 11
    "Middle_3",        # 12 (tip)
    "Ring_0",          # 13
    "Ring_1",          # 14
    "Ring_2",          # 15
    "Ring_3",          # 16 (tip)
    "Pinky_0",         # 17
    "Pinky_1",         # 18
    "Pinky_2",         # 19
    "Pinky_3",         # 20 (tip)
]

# --- SCRIPT DE PRUEBA ---

def main():
    """Script de prueba del parser"""
    receiver = HandTrackingReceiver(port=7777)
    
    print("👉 Mueve las manos frente al Quest")
    print("   Presiona Ctrl+C para detener\n")
    
    frame_count = 0
    
    try:
        for frame in receiver.stream_frames():
            frame_count += 1
            
            print(f"\n{'='*70}")
            print(f"Frame #{frame_count} - {frame}")
            
            if frame.left_hand:
                print(f"\n🤚 MANO IZQUIERDA:")
                print(f"  {frame.left_hand.wrist}")
                print(f"  Wrist: {frame.left_hand.get_landmark(0)}")
                print(f"  Thumb tip: {frame.left_hand.get_landmark(4)}")
                print(f"  Index tip: {frame.left_hand.get_landmark(8)}")
                print(f"  Middle tip: {frame.left_hand.get_landmark(12)}")
                print(f"  Ring tip: {frame.left_hand.get_landmark(16)}")
                print(f"  Pinky tip: {frame.left_hand.get_landmark(20)}")
                
            if frame.right_hand:
                print(f"\n👌 MANO DERECHA:")
                print(f"  {frame.right_hand.wrist}")
                print(f"  Wrist: {frame.right_hand.get_landmark(0)}")
                print(f"  Thumb tip: {frame.right_hand.get_landmark(4)}")
                print(f"  Index tip: {frame.right_hand.get_landmark(8)}")
                print(f"  Middle tip: {frame.right_hand.get_landmark(12)}")
                print(f"  Ring tip: {frame.right_hand.get_landmark(16)}")
                print(f"  Pinky tip: {frame.right_hand.get_landmark(20)}")
            
            # Mostrar los primeros 20 frames
            if frame_count >= 20:
                print(f"\n\n✅ Test completado - {frame_count} frames capturados")
                print(f"📊 FPS aproximado: {frame_count / frame.timestamp:.1f} Hz")
                break
                
    except KeyboardInterrupt:
        print(f"\n\n📊 Total de frames capturados: {frame_count}")
        if frame_count > 0 and receiver.parser.start_time:
            elapsed = time.time() - receiver.parser.start_time
            print(f"⏱️  Tiempo total: {elapsed:.2f}s")
            print(f"📈 FPS promedio: {frame_count / elapsed:.1f} Hz")

if __name__ == "__main__":
    main()
