#!/usr/bin/env python3
"""
Analyze STL files to find joint connection points by looking at
where adjacent links would physically meet.
"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Mesh
import Part
import os
import math
import numpy as np

# STL files from FreeCAD conversion
STL_DIR = "/home/i/ros2_ws/gofa_stl_from_step/stl_local"

stl_files = [
    ("base_link.stl", "link_1.stl", "joint_1"),
    ("link_1.stl", "link_2.stl", "joint_2"),
    ("link_2.stl", "link_3.stl", "joint_3"),
    ("link_3.stl", "link_4.stl", "joint_4"),
    ("link_4.stl", "link_5.stl", "joint_5"),
    ("link_5.stl", "link_6.stl", "joint_6"),
]

def load_stl(filepath):
    """Load STL and return mesh"""
    doc = FreeCAD.newDocument()
    Mesh.insert(filepath, doc.Name)

    mesh_obj = doc.Objects[0]
    return mesh_obj.Mesh, doc

def find_extreme_points_along_axis(mesh, axis='z'):
    """Find points with min and max values along an axis"""
    points = mesh.Points
    if not points:
        return None, None

    if axis == 'z':
        idx = 2
    elif axis == 'y':
        idx = 1
    else:
        idx = 0

    min_point = max_point = None
    min_val = float('inf')
    max_val = float('-inf')

    for p in points:
        val = p.x if idx == 0 else (p.y if idx == 1 else p.z)
        if val < min_val:
            min_val = val
            min_point = (p.x, p.y, p.z)
        if val > max_val:
            max_val = val
            max_point = (p.x, p.y, p.z)

    return min_point, max_point

def find_nearest_points(mesh_a, mesh_b, direction):
    """Find the pair of points, one from each mesh, that are nearest
    along the given direction vector"""
    points_a = mesh_a.Points
    points_b = mesh_b.Points

    min_dist = float('inf')
    best_a = best_b = None

    # Project all points onto the direction
    for p_a in points_a:
        for p_b in points_b:
            dist = math.sqrt((p_a.x - p_b.x)**2 + (p_a.y - p_b.y)**2 + (p_a.z - p_b.z)**2)
            if dist < min_dist:
                min_dist = dist
                best_a = (p_a.x, p_a.y, p_a.z)
                best_b = (p_b.x, p_b.y, p_b.z)

    return best_a, best_b, min_dist

print("=== Analyzing STL Files for Joint Connections ===\n")

results = []

for stl_a, stl_b, joint_name in stl_files:
    path_a = os.path.join(STL_DIR, stl_a)
    path_b = os.path.join(STL_DIR, stl_b)

    print(f"\n{joint_name}: {stl_a} -> {stl_b}")

    mesh_a, doc_a = load_stl(path_a)
    mesh_b, doc_b = load_stl(path_b)

    # Get centers
    points_a = list(mesh_a.Points)
    points_b = list(mesh_b.Points)

    center_a = np.mean([[p.x, p.y, p.z] for p in points_a], axis=0)
    center_b = np.mean([[p.x, p.y, p.z] for p in points_b], axis=0)

    print(f"  {stl_a} center: ({center_a[0]:.2f}, {center_a[1]:.2f}, {center_a[2]:.2f})")
    print(f"  {stl_b} center: ({center_b[0]:.2f}, {center_b[1]:.2f}, {center_b[2]:.2f})")

    # Vector from A center to B center
    vec = center_b - center_a
    print(f"  Vector A->B: ({vec[0]:.2f}, {vec[1]:.2f}, {vec[2]:.2f})")
    print(f"  Distance: {np.linalg.norm(vec):.2f}mm")

    # Find extreme points along Z (typical robot orientation)
    min_a_z, max_a_z = find_extreme_points_along_axis(mesh_a, 'z')
    min_b_z, max_b_z = find_extreme_points_along_axis(mesh_b, 'z')

    if min_a_z and max_b_z:
        print(f"  {stl_a} min Z: {min_a_z[2]:.2f}")
        print(f"  {stl_b} max Z: {max_b_z[2]:.2f}")

    # Find nearest point pairs
    near_a, near_b, dist = find_nearest_points(mesh_a, mesh_b, vec)
    if near_a and near_b:
        print(f"  Nearest points: A({near_a[0]:.2f}, {near_a[1]:.2f}, {near_a[2]:.2f})")
        print(f"                   B({near_b[0]:.2f}, {near_b[1]:.2f}, {near_b[2]:.2f})")
        print(f"  Nearest distance: {dist:.2f}mm")

    results.append({
        'joint': joint_name,
        'link_a': stl_a,
        'link_b': stl_b,
        'center_a': center_a,
        'center_b': center_b,
        'vec': vec
    })

    FreeCAD.closeDocument(doc_a.Name)
    FreeCAD.closeDocument(doc_b.Name)

print("\n\n=== Computed Joint Origins (in base_link frame) ===")
# joint_1 origin is at base_link top, computed relative to base_link
# Subsequent joints are computed relative to previous link

for i, r in enumerate(results):
    link_a = r['link_a'].replace('.stl', '')
    link_b = r['link_b'].replace('.stl', '')
    vec = r['vec']

    # The joint origin should be approximately where the two links meet
    # For a revolute joint, this is typically at one end of each link
    # We'll estimate it as a point along the connection vector

    # Find approximate joint position (at the edge of link A towards link B)
    # This is a rough approximation based on geometry

    print(f"\n{r['joint']} ({link_a} -> {link_b}):")
    print(f"  Vector: ({vec[0]:.4f}, {vec[1]:.4f}, {vec[2]:.4f})")

print("\n\n=== Joint Origin xyz for URDF ===")
print("Based on link centers (not exact joint locations):")
for i, r in enumerate(results):
    link_a = r['link_a'].replace('.stl', '')
    link_b = r['link_b'].replace('.stl', '')
    vec = r['vec']

    # For joint i+1, the origin is the position in parent frame
    # If parent is base_link, it's the world position of the joint
    # But for URDF, we need it in the parent's frame

    # Since all STLs are in world coordinates and base_link is at origin-ish,
    # we can use world coordinates directly

    # The joint position is approximately where link_a ends and link_b begins
    # Estimate: 70% into the vector from center_a towards center_b

    joint_pos = r['center_a'] + vec * 0.7  # Rough estimate

    print(f"{r['joint']}: xyz=\"{joint_pos[0]:.4f} {joint_pos[1]:.4f} {joint_pos[2]:.4f}\"")