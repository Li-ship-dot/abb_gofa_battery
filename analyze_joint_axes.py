#!/usr/bin/env python3
"""
Analyze individual STEP files to determine joint axes for GoFa CRB15000-10/1.52
"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Part
import Mesh
import os
import math

def quaternion_to_rpy(q1, q2, q3, q4):
    """Convert quaternion to roll, pitch, yaw"""
    s = q1*q1 + q2*q2 + q3*q3 + q4*q4
    if s < 1e-10:
        return (0, 0, 0)
    s = 1.0 / math.sqrt(s)

    r11 = 1 - 2*s*(q2*q2 + q3*q3)
    r12 = 2*s*(q1*q2 + q3*q4)
    r13 = 2*s*(q3*q1 - q2*q4)
    r21 = 2*s*(q1*q2 - q3*q4)
    r22 = 1 - 2*s*(q3*q3 + q1*q1)
    r23 = 2*s*(q2*q3 + q1*q4)
    r31 = 2*s*(q3*q1 + q2*q4)
    r32 = 2*s*(q2*q3 - q1*q4)
    r33 = 1 - 2*s*(q1*q1 + q2*q2)

    if abs(r31) >= 1:
        pitch = math.copysign(math.pi/2, r31)
        roll = 0
        yaw = math.atan2(r21, r11)
    else:
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)

    return (roll, pitch, yaw)

# Individual STEP files from STEP_J folder
STEP_J_DIR = "/mnt/hgfs/Linux_shared/Gofa CRB15000_10kg-152 机械臂 技术资料/Gofa 机械臂模型/CRB15000_10kg-152_OmniCore_rev00_STEP_J"

files = [
    ("CRB15000_10kg-152_Omnicore_rev00_LINK00_CAD.step", "base_link"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK01_CAD.step", "link_1"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK02_CAD.step", "link_2"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK03_CAD.step", "link_3"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK04_CAD.step", "link_4"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK05_CAD.step", "link_5"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK06_CAD.step", "link_6"),
]

def load_step_with_mesh(filepath):
    """Load STEP and create mesh for analysis"""
    doc = FreeCAD.newDocument()
    Part.insert(filepath, doc.Name)

    shape = None
    for obj in doc.Objects:
        if hasattr(obj, 'Shape') and obj.Shape.isValid():
            shape = obj.Shape
            break

    if shape is None:
        FreeCAD.closeDocument(doc.Name)
        return None, None

    mesh_data = shape.tessellate(0.1)
    mesh = Mesh.Mesh(mesh_data)

    return doc, mesh

def analyze_link_geometry(mesh, link_name):
    """Analyze mesh to find geometric features"""
    points = mesh.Points
    if not points:
        return None

    coords = [(p.x, p.y, p.z) for p in points]
    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]
    zs = [c[2] for c in coords]

    min_coords = (min(xs), min(ys), min(zs))
    max_coords = (max(xs), max(ys), max(zs))
    center = ((min(xs)+max(xs))/2, (min(ys)+max(ys))/2, (min(zs)+max(zs))/2)

    return {
        'name': link_name,
        'min': min_coords,
        'max': max_coords,
        'center': center,
        'size': (max_coords[0]-min_coords[0], max_coords[1]-min_coords[1], max_coords[2]-min_coords[2])
    }

print("=== Analyzing Individual STEP Files ===\n")

# Load all links
links_info = []
for step_file, link_name in files:
    step_path = os.path.join(STEP_J_DIR, step_file)
    print(f"Loading: {link_name}")

    doc, mesh = load_step_with_mesh(step_path)
    if mesh is None:
        print(f"  ERROR: Could not load {step_path}")
        continue

    info = analyze_link_geometry(mesh, link_name)
    links_info.append(info)

    print(f"  Center: ({info['center'][0]:.2f}, {info['center'][1]:.2f}, {info['center'][2]:.2f})")
    print(f"  Size: ({info['size'][0]:.2f}, {info['size'][1]:.2f}, {info['size'][2]:.2f})")

    # Get rotation from shape placement
    for obj in doc.Objects:
        if hasattr(obj, 'Shape') and obj.Shape.isValid():
            placement = obj.Shape.Placement
            r = placement.Rotation
            q = r.Q
            print(f"  Placement Q: ({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})")

            # Compute RPY from quaternion
            rpy = quaternion_to_rpy(q[0], q[1], q[2], q[3])
            print(f"  RPY (deg): ({math.degrees(rpy[0]):.2f}, {math.degrees(rpy[1]):.2f}, {math.degrees(rpy[2]):.2f})")
            break

    FreeCAD.closeDocument(doc.Name)
    print()

print("\n=== Link Centers Summary ===")
for info in links_info:
    print(f"{info['name']}: ({info['center'][0]:.2f}, {info['center'][1]:.2f}, {info['center'][2]:.2f})")

# Sort by Z to get arm order
print("\n=== Sorted by Z (arm order) ===")
sorted_links = sorted(links_info, key=lambda x: x['center'][2])
for i, info in enumerate(sorted_links):
    print(f"{i}: {info['name']} Z={info['center'][2]:.2f}")

print("\n=== Computing Inter-Link Vectors ===")
for i in range(len(sorted_links) - 1):
    link_a = sorted_links[i]
    link_b = sorted_links[i + 1]

    # Vector from A to B
    vec = tuple(link_b['center'][j] - link_a['center'][j] for j in range(3))
    length = math.sqrt(sum(v*v for v in vec))

    print(f"\n{link_a['name']} -> {link_b['name']}:")
    print(f"  Vector: ({vec[0]:.2f}, {vec[1]:.2f}, {vec[2]:.2f})")
    print(f"  Length: {length:.2f}mm")

    # Normalize
    if length > 0.001:
        vec_norm = tuple(v/length for v in vec)
        print(f"  Normalized: ({vec_norm[0]:.4f}, {vec_norm[1]:.4f}, {vec_norm[2]:.4f})")

        # Determine primary direction
        max_component = max(abs(v) for v in vec_norm)
        if abs(vec_norm[2]) == max_component:
            axis = "Z" if vec_norm[2] > 0 else "-Z"
        elif abs(vec_norm[1]) == max_component:
            axis = "Y" if vec_norm[1] > 0 else "-Y"
        else:
            axis = "X" if vec_norm[0] > 0 else "-X"
        print(f"  Primary axis: {axis}")