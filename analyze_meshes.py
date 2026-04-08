#!/usr/bin/env python3
"""Analyze STL files to understand their coordinate systems."""

import numpy as np
from stl import mesh

VISUAL_DIR = "/home/i/ros2_ws/src/abb_omnicore_ros2/robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual"
ASSEMBLY_STL = "/home/i/ros2_ws/src/abb_omnicore_ros2/robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual/gofa_assembly.stl"

files = {
    "base_link": f"{VISUAL_DIR}/base_link.stl",
    "link_1": f"{VISUAL_DIR}/link_1.stl",
    "link_2": f"{VISUAL_DIR}/link_2.stl",
    "link_3": f"{VISUAL_DIR}/link_3.stl",
    "link_4": f"{VISUAL_DIR}/link_4.stl",
    "link_5": f"{VISUAL_DIR}/link_5.stl",
    "link_6": f"{VISUAL_DIR}/link_6.stl",
    "assembly": ASSEMBLY_STL
}

print("=== STL File Analysis ===\n")

for name, filepath in files.items():
    try:
        m = mesh.Mesh.from_file(filepath)
        min_coords = np.min(m.points.reshape(-1, 3), axis=0)
        max_coords = np.max(m.points.reshape(-1, 3), axis=0)
        center = np.mean(m.points.reshape(-1, 3), axis=0)

        print(f"{name}:")
        print(f"  Min: X={min_coords[0]:.4f}, Y={min_coords[1]:.4f}, Z={min_coords[2]:.4f}")
        print(f"  Max: X={max_coords[0]:.4f}, Y={max_coords[1]:.4f}, Z={max_coords[2]:.4f}")
        print(f"  Center (centroid): X={center[0]:.4f}, Y={center[1]:.4f}, Z={center[2]:.4f}")
        print(f"  Bounding box size: X={max_coords[0]-min_coords[0]:.4f}, Y={max_coords[1]-min_coords[1]:.4f}, Z={max_coords[2]-min_coords[2]:.4f}")
        print()
    except Exception as e:
        print(f"{name}: Error - {e}\n")