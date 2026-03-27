#!/usr/bin/env python3
"""
Gestor de Escena Industrial para MoveIt
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import time
import json
import os

COLLISION_CONFIG = os.path.expanduser('~/dobot_ws/collision_config.json')


class SceneManager(Node):
    def __init__(self):
        super().__init__('scene_manager')
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('   GESTOR DE ESCENA INDUSTRIAL')
        self.get_logger().info('=' * 50)

        # Servicio /apply_planning_scene — garantiza entrega (vs topic que se pierde si move_group no está listo)
        self.apply_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Esperando /apply_planning_scene...')
        if not self.apply_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('❌ /apply_planning_scene no disponible — usando topic fallback')
            self.apply_client = None
            self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
            time.sleep(2.0)
        else:
            self.get_logger().info('✅ /apply_planning_scene disponible')
            self.scene_pub = None

        self.load_scene()

    def _apply_scene(self, ps):
        """Aplica una PlanningScene usando servicio (confiable) o topic (fallback)."""
        if self.apply_client is not None:
            req = ApplyPlanningScene.Request()
            req.scene = ps
            future = self.apply_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.result() is not None and future.result().success:
                return True
            self.get_logger().warn('⚠️ apply_planning_scene falló, reintentando...')
            return False
        else:
            self.scene_pub.publish(ps)
            return True

    def create_box(self, name, size, position):
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = name
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size
        
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def load_from_config(self):
        """Load collision objects from ~/dobot_ws/collision_config.json.
        
        Returns True if successful, False otherwise.
        Raises FileNotFoundError if config doesn't exist.
        """
        with open(COLLISION_CONFIG, 'r') as f:
            config = json.load(f)
        
        if config.get('status') != 'calibrated':
            self.get_logger().warning('collision_config.json status is not "calibrated" — skipping')
            return False
        
        objects = config.get('objects', {})
        
        # REMOVE existing objects first (prevent stale collision objects)
        # Build list of names to remove: current config names + legacy fallback names
        legacy_names = ['mesa_trabajo', 'pallet_jugos', 'cinta_transportadora',
                        'camara_soporte', 'pared', 'suelo', 'techo',
                        'mesa_trabajo_hc', 'pallet_jugos_hc', 'banda_hc']
        names_to_remove = list(objects.keys()) + [n for n in legacy_names if n not in objects]
        for name in names_to_remove:
            obj = CollisionObject()
            obj.id = name
            obj.header.frame_id = 'base_link'
            obj.operation = CollisionObject.REMOVE
            ps = PlanningScene()
            ps.is_diff = True
            ps.world.collision_objects.append(obj)
            self._apply_scene(ps)
        
        # ADD objects from config
        for obj_name, obj_data in objects.items():
            obj_type = obj_data.get('type')
            
            if obj_type == 'floor':
                z_m = obj_data['z_mm'] / 1000.0
                co = self.create_box(obj_name, [6.0, 6.0, 0.01], [0.0, 0.0, z_m - 0.005])
            elif obj_type == 'ceiling':
                z_m = obj_data['z_mm'] / 1000.0
                co = self.create_box(obj_name, [6.0, 6.0, 0.01], [0.0, 0.0, z_m + 0.005])
            elif obj_type == 'box':
                p1 = obj_data['corner1_mm']
                p2 = obj_data['corner2_mm']
                h = obj_data['height_mm']
                cx = (p1[0] + p2[0]) / 2.0 / 1000.0
                cy = (p1[1] + p2[1]) / 2.0 / 1000.0
                z_surf = (p1[2] + p2[2]) / 2.0
                cz = (z_surf - h / 2.0) / 1000.0
                dx = abs(p2[0] - p1[0]) / 1000.0
                dy = abs(p2[1] - p1[1]) / 1000.0
                dz = h / 1000.0
                co = self.create_box(obj_name, [dx, dy, dz], [cx, cy, cz])
            elif obj_type == 'pallet':
                # Pallet base — same geometry as box but identified separately
                # for dynamic layer slab management in depalletizer.
                p1 = obj_data['corner1_mm']
                p2 = obj_data['corner2_mm']
                h = obj_data.get('height_mm', 0)
                if h <= 0:
                    # Zero-height pallet: just mark the surface, use minimal thickness
                    h = 10  # 10mm minimal slab to be visible in MoveIt scene
                cx = (p1[0] + p2[0]) / 2.0 / 1000.0
                cy = (p1[1] + p2[1]) / 2.0 / 1000.0
                z_surf = (p1[2] + p2[2]) / 2.0
                cz = (z_surf - h / 2.0) / 1000.0
                dx = abs(p2[0] - p1[0]) / 1000.0
                dy = abs(p2[1] - p1[1]) / 1000.0
                dz = h / 1000.0
                co = self.create_box(obj_name, [dx, dy, dz], [cx, cy, cz])
            elif obj_type == 'pole':
                bottom = obj_data['bottom_mm']
                width = obj_data['width_mm']
                # Find ceiling by type (not by name)
                def _find_by_type(objects_dict, type_str, default_z):
                    for obj in objects_dict.values():
                        if obj.get('type') == type_str:
                            return obj.get('z_mm', default_z)
                    return default_z
                techo_z_mm = _find_by_type(objects, 'ceiling', 1000.0)
                h_mm = obj_data.get('height_mm', techo_z_mm - bottom[2])
                cx = bottom[0] / 1000.0
                cy = bottom[1] / 1000.0
                # Cuelga desde el techo hacia abajo
                cz = (techo_z_mm - h_mm / 2.0) / 1000.0
                co = self.create_box(obj_name, [width/1000.0, width/1000.0, h_mm/1000.0], [cx, cy, cz])
            elif obj_type == 'wall':
                p1 = obj_data['point1_mm']
                p2 = obj_data['point2_mm']
                thickness = obj_data.get('thickness_mm', 100.0)
                # Find floor and ceiling by type (not by name)
                def _find_by_type(objects_dict, type_str, default_z):
                    for obj in objects_dict.values():
                        if obj.get('type') == type_str:
                            return obj.get('z_mm', default_z)
                    return default_z
                suelo_z_mm = _find_by_type(objects, 'floor', -600.0)
                techo_z_mm = _find_by_type(objects, 'ceiling', 1000.0)
                import math
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                length_mm = math.sqrt(dx*dx + dy*dy)
                height_mm = techo_z_mm - suelo_z_mm
                cx = (p1[0] + p2[0]) / 2.0 / 1000.0
                cy = (p1[1] + p2[1]) / 2.0 / 1000.0
                cz = (suelo_z_mm + techo_z_mm) / 2.0 / 1000.0
                # Note: wall rotation is not applied (MoveIt shape_msgs Box has no rotation field directly)
                # For MoveIt collision, a box without rotation is an approximation
                # A more accurate version would use oriented bounding box (future enhancement)
                co = self.create_box(obj_name, [length_mm/1000.0, thickness/1000.0, height_mm/1000.0], [cx, cy, cz])
            else:
                self.get_logger().warning(f'Unknown object type "{obj_type}" for "{obj_name}" — skipping')
                continue
            
            ps = PlanningScene()
            ps.is_diff = True
            ps.world.collision_objects.append(co)
            self._apply_scene(ps)
            self.get_logger().info(f'  Loaded: {obj_name} ({obj_type})')
        
        return True

    def load_scene_hardcoded(self):
        scene = PlanningScene()
        scene.is_diff = True
        
        # Mesa de trabajo
        mesa = self.create_box('mesa_trabajo', [1.5, 1.0, 0.05], [0.5, 0.0, -0.025])
        scene.world.collision_objects.append(mesa)
        self.get_logger().info('  ✅ Mesa agregada')
        
        # Pallet
        pallet = self.create_box('pallet_jugos', [0.8, 0.6, 0.15], [0.6, 0.3, 0.075])
        scene.world.collision_objects.append(pallet)
        self.get_logger().info('  ✅ Pallet agregado')
        
        # Banda transportadora
        conveyor = self.create_box('banda', [1.0, 0.3, 0.1], [0.5, -0.4, 0.05])
        scene.world.collision_objects.append(conveyor)
        self.get_logger().info('  ✅ Banda agregada')
        
        self._apply_scene(scene)
        self.get_logger().info(f'✅ Escena cargada: {len(scene.world.collision_objects)} objetos')

    def load_scene(self):
        """Load collision scene from config file, falling back to hardcoded."""
        if os.path.exists(COLLISION_CONFIG):
            try:
                self.get_logger().info(f'Loading collision scene from {COLLISION_CONFIG}')
                success = self.load_from_config()
                if success:
                    self.get_logger().info('✅ Collision scene loaded from collision_config.json')
                    return
            except json.JSONDecodeError as e:
                self.get_logger().error(f'JSON parse error in collision_config.json: {e} — using hardcoded fallback')
            except Exception as e:
                self.get_logger().error(f'Error loading collision_config.json: {e} — using hardcoded fallback')
        else:
            self.get_logger().warning(
                f'⚠️  collision_config.json not found at {COLLISION_CONFIG} — '
                'using hardcoded collision objects. Run: ros2 run dobot_moveit collision_calibrator'
            )
        
        self.load_scene_hardcoded()


def main(args=None):
    rclpy.init(args=args)
    node = SceneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
