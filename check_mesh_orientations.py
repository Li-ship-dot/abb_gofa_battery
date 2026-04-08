#!/usr/bin/env python3
"""Check STL mesh orientations by analyzing point distributions"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Mesh
import os
import math

STL_DIR = "/home/i/ros2_ws/gofa_stl_from_step/stl_local"

files = [
    "base_link.stl",
    "link_1.stl",
    "link_2.stl",
    "link_3.stl",
    "link_4.stl",
    "link_5.stl",
    "link_6.stl",
]

def analyze_mesh_orientation(mesh, name):
    """Analyze mesh to determine its principal orientation"""
    points = mesh.Points
    if not points:
        return

    coords = [(p.x, p.y, p.z) for p in points]
    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]
    zs = [c[2] for c in coords]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    size_x = max_x - min_x
    size_y = max_y - min_y
    size_z = max_z - min_z

    print(f"\n{name}:")
    print(f"  X: {min_x:.1f} to {max_x:.1f} (size: {size_x:.1f})")
    print(f"  Y: {min_y:.1f} to {max_y:.1f} (size: {size_y:.1f})")
    print(f"  Z: {min_z:.1f} to {max_z:.1f} (size: {size_z:.1f})")

    # Determine primary orientation
    max_size = max(size_x, size_y, size_z)
    if max_size == size_x:
        orient = "X-aligned"
    elif max_size == size_y:
        orient = "Y-aligned"
    else:
        orient = "Z-aligned"

    print(f"  Primary orientation: {orient}")

    # Center
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    center_z = (min_z + max_z) / 2
    print(f"  Center: ({center_x:.1f}, {center_y:.1f}, {center_z:.1f})")

    return {
        'name': name,
        'size': (size_x, size_y, size_z),
        'center': (center_x, center_y, center_z),
        'orient': orient
    }

print("=== Checking Mesh Orientations ===\n")

doc = FreeCAD.newDocument()

infos = []
for fname in files:
    fpath = os.path.join(STL_DIR, fname)
    if os.path.exists(fpath):
        Mesh.insert(fpath, doc.Name)
        mesh = doc.Objects[-1].Mesh
        info = analyze_mesh_orientation(mesh, fname)
        if info:
            infos.append(info)
    else:
        print(f"{fname}: FILE NOT FOUND")

# Check relative orientations between adjacent links
print("\n\n=== Relative Link Orientations ===")
print("(If adjacent links have same orientation, they likely connect directly)")
print("(If orientations differ, there may be a frame rotation between them)")

for i in range(len(infos) - 1):
    info_a = infos[i]
    info_b = infos[i + 1]
    print(f"\n{info_a['name']} -> {info_b['name']}:")
    print(f"  {info_a['name']}: {info_a['orient']}, size {info_a['size']}")
    print(f"  {info_b['name']}: {info_b['orient']}, size {info_b['size']}")

    # Size ratios
    size_a = info_a['size']
    size_b = info_b['size']
    ratio_x = size_b[0] / size_a[0] if size_a[0] > 0 else 0
    ratio_y = size_b[1] / size_a[1] if size_a[1] > 0 else 0
    ratio_z = size_b[2] / size_a[2] if size_a[2] > 0 else 0
    print(f"  Size ratios (B/A): X={ratio_x:.2f}, Y={ratio_y:.2f}, Z={ratio_z:.2f}")

FreeCAD.closeDocument(doc.Name)