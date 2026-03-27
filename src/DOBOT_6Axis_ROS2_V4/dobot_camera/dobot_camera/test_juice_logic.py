"""
Unit tests for juice_logic module.

Tests pure functions for:
- Associating cara_F detections with HIT detections
- Sorting detections for pick order
"""

from dobot_camera.juice_logic import (
    associate_cara_f_with_hit,
    sort_detections_for_pick,
    JUICE_TYPE_MANGO,
    JUICE_TYPE_MORA,
    JUICE_TYPE_UNKNOWN,
    MAX_ASSOCIATION_DIST_PX,
)


# ============================================================================
# Association Tests
# ============================================================================

def test_single_cara_f_single_hit_mango():
    """Single cara_F near HIT-MANGO (class_id=0) → juice_type=0.0"""
    cara_f_list = [{'cx': 100.0, 'cy': 100.0}]
    hit_list = [{'cx': 120.0, 'cy': 110.0, 'class_id': 0}]
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert len(result) == 1
    assert result[0]['juice_type'] == JUICE_TYPE_MANGO
    assert result[0]['juice_type'] == 0.0


def test_single_cara_f_single_hit_mora():
    """Single cara_F near HIT-MORA (class_id=1) → juice_type=1.0"""
    cara_f_list = [{'cx': 200.0, 'cy': 200.0}]
    hit_list = [{'cx': 190.0, 'cy': 210.0, 'class_id': 1}]
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert len(result) == 1
    assert result[0]['juice_type'] == JUICE_TYPE_MORA
    assert result[0]['juice_type'] == 1.0


def test_cara_f_no_hit_nearby():
    """cara_F at (100,100), HIT at (400,400) → distance > 150px → juice_type=-1.0"""
    cara_f_list = [{'cx': 100.0, 'cy': 100.0}]
    hit_list = [{'cx': 400.0, 'cy': 400.0, 'class_id': 0}]
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert len(result) == 1
    assert result[0]['juice_type'] == JUICE_TYPE_UNKNOWN
    assert result[0]['juice_type'] == -1.0


def test_cara_f_no_hits_at_all():
    """Single cara_F with empty hit_list → juice_type=-1.0"""
    cara_f_list = [{'cx': 100.0, 'cy': 100.0}]
    hit_list = []
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert len(result) == 1
    assert result[0]['juice_type'] == JUICE_TYPE_UNKNOWN


def test_multiple_cara_f_multiple_hits():
    """2 cara_F and 2 HITs → each cara_F associates with nearest HIT"""
    cara_f_list = [
        {'cx': 100.0, 'cy': 100.0},
        {'cx': 300.0, 'cy': 300.0}
    ]
    hit_list = [
        {'cx': 110.0, 'cy': 100.0, 'class_id': 0},  # MANGO
        {'cx': 290.0, 'cy': 300.0, 'class_id': 1}   # MORA
    ]
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert len(result) == 2
    assert result[0]['juice_type'] == JUICE_TYPE_MANGO
    assert result[1]['juice_type'] == JUICE_TYPE_MORA


def test_cara_f_exactly_at_threshold():
    """cara_F at (0,0), HIT at (150,0) → distance=150.0 (≤ threshold) → juice_type assigned"""
    cara_f_list = [{'cx': 0.0, 'cy': 0.0}]
    hit_list = [{'cx': 150.0, 'cy': 0.0, 'class_id': 0}]
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert len(result) == 1
    assert result[0]['juice_type'] == JUICE_TYPE_MANGO


def test_cara_f_just_beyond_threshold():
    """cara_F at (0,0), HIT at (151,0) → distance=151.0 (> threshold) → juice_type=-1.0"""
    cara_f_list = [{'cx': 0.0, 'cy': 0.0}]
    hit_list = [{'cx': 151.0, 'cy': 0.0, 'class_id': 0}]
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert len(result) == 1
    assert result[0]['juice_type'] == JUICE_TYPE_UNKNOWN


def test_empty_cara_f_list():
    """Empty cara_f_list → returns []"""
    cara_f_list = []
    hit_list = [{'cx': 100.0, 'cy': 100.0, 'class_id': 0}]
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert result == []


def test_preserves_original_cara_f_fields():
    """Output cara_F dict preserves original fields (e.g., cam_z_mm)"""
    cara_f_list = [{'cx': 100.0, 'cy': 100.0, 'cam_z_mm': 500.0}]
    hit_list = [{'cx': 110.0, 'cy': 100.0, 'class_id': 0}]
    
    result = associate_cara_f_with_hit(cara_f_list, hit_list)
    
    assert len(result) == 1
    assert result[0]['cx'] == 100.0
    assert result[0]['cy'] == 100.0
    assert result[0]['cam_z_mm'] == 500.0
    assert result[0]['juice_type'] == JUICE_TYPE_MANGO


# ============================================================================
# Sorting Tests
# ============================================================================

def test_sort_by_z_ascending():
    """3 detections with z=500, z=300, z=400 → sorted: 300, 400, 500"""
    detections = [
        {'cam_z_mm': 500.0, 'robot_dist_mm': 100.0},
        {'cam_z_mm': 300.0, 'robot_dist_mm': 100.0},
        {'cam_z_mm': 400.0, 'robot_dist_mm': 100.0}
    ]
    
    result = sort_detections_for_pick(detections)
    
    assert len(result) == 3
    assert result[0]['cam_z_mm'] == 300.0
    assert result[1]['cam_z_mm'] == 400.0
    assert result[2]['cam_z_mm'] == 500.0


def test_sort_same_z_layer_by_distance():
    """3 detections at z=300, z=305, z=310 (within ±15mm) → sorted by robot_dist_mm"""
    detections = [
        {'cam_z_mm': 300.0, 'robot_dist_mm': 800.0},
        {'cam_z_mm': 305.0, 'robot_dist_mm': 400.0},
        {'cam_z_mm': 310.0, 'robot_dist_mm': 600.0}
    ]
    
    result = sort_detections_for_pick(detections)
    
    assert len(result) == 3
    assert result[0]['robot_dist_mm'] == 400.0
    assert result[1]['robot_dist_mm'] == 600.0
    assert result[2]['robot_dist_mm'] == 800.0


def test_sort_empty_list():
    """Empty list → returns []"""
    detections = []
    
    result = sort_detections_for_pick(detections)
    
    assert result == []


def test_sort_does_not_mutate_input():
    """Original list unchanged after sort"""
    detections = [
        {'cam_z_mm': 500.0, 'robot_dist_mm': 100.0},
        {'cam_z_mm': 300.0, 'robot_dist_mm': 100.0}
    ]
    original_order = [d['cam_z_mm'] for d in detections]
    
    result = sort_detections_for_pick(detections)
    
    # Input should be unchanged
    assert [d['cam_z_mm'] for d in detections] == original_order
    # Result should be sorted
    assert result[0]['cam_z_mm'] == 300.0
    assert result[1]['cam_z_mm'] == 500.0
