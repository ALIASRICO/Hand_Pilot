"""
Pure logic functions for juice type detection and pick ordering.
No ROS2 dependencies — fully testable without a running robot.
"""
import math

# Juice type constants
JUICE_TYPE_MANGO   = 0.0
JUICE_TYPE_MORA    = 1.0
JUICE_TYPE_UNKNOWN = -1.0

# Association threshold
MAX_ASSOCIATION_DIST_PX = 150.0

# Class IDs (must match trained YOLO model: jugos_best.pt)
CLASS_ID_HIT_MANGO = 0
CLASS_ID_HIT_MORA  = 1
CLASS_ID_CARA_F    = 2


def associate_cara_f_with_hit(cara_f_list, hit_list, max_dist_px=MAX_ASSOCIATION_DIST_PX):
    """
    Associate each cara_F detection with the nearest HIT-MANGO or HIT-MORA detection.

    Args:
        cara_f_list: list of dicts with at minimum {'cx': float, 'cy': float}
        hit_list: list of dicts with at minimum {'cx': float, 'cy': float, 'class_id': int}
        max_dist_px: max Euclidean pixel distance for association

    Returns:
        list of dicts — copy of each cara_F dict with added 'juice_type' key.
        juice_type: 0.0=MANGO, 1.0=MORA, -1.0=UNKNOWN
    """
    result = []
    for cara_f in cara_f_list:
        det = dict(cara_f)  # copy
        best_dist = float('inf')
        best_type = JUICE_TYPE_UNKNOWN
        for hit in hit_list:
            dx = cara_f['cx'] - hit['cx']
            dy = cara_f['cy'] - hit['cy']
            dist = math.sqrt(dx * dx + dy * dy)
            if dist <= max_dist_px and dist < best_dist:
                best_dist = dist
                if hit['class_id'] == CLASS_ID_HIT_MANGO:
                    best_type = JUICE_TYPE_MANGO
                elif hit['class_id'] == CLASS_ID_HIT_MORA:
                    best_type = JUICE_TYPE_MORA
        det['juice_type'] = best_type
        result.append(det)
    return result


def sort_detections_for_pick(detections, z_layer_tol_mm=15.0):
    """
    Sort detections for optimal pick order.

    Primary: ascending cam_z_mm (smallest Z = closest to camera = top of stack).
    Secondary: within the same Z layer (within z_layer_tol_mm of the topmost),
               sort by ascending robot_dist_mm (closest robot distance first).

    Args:
        detections: list of dicts with at minimum {'cam_z_mm': float, 'robot_dist_mm': float}
        z_layer_tol_mm: tolerance in mm to consider detections in the "same layer"

    Returns:
        New sorted list (does NOT mutate input).
    """
    if not detections:
        return []

    # First sort by cam_z_mm ascending
    sorted_by_z = sorted(detections, key=lambda d: d.get('cam_z_mm', 9999.0))

    # Find detections in the same top layer (within tolerance of the topmost)
    top_z = sorted_by_z[0].get('cam_z_mm', 0.0)
    same_layer = [d for d in sorted_by_z if abs(d.get('cam_z_mm', 0.0) - top_z) <= z_layer_tol_mm]
    rest = [d for d in sorted_by_z if abs(d.get('cam_z_mm', 0.0) - top_z) > z_layer_tol_mm]

    # Sort same-layer group by robot distance
    same_layer_sorted = sorted(same_layer, key=lambda d: d.get('robot_dist_mm', 9999.0))

    return same_layer_sorted + rest
