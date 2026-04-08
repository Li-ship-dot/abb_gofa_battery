#!/usr/bin/env python3
"""
Compute joint transforms for GoFa CRB15000-10/1.52 by analyzing geometry.

Key insight: For a robot arm, each joint connects two links at a specific point.
The joint origin (xyz) in URDF is the position of the child link's frame origin
in the parent link's frame.

The joint rpy defines how the child frame is rotated relative to parent frame.
For a revolute joint, the rotation axis is defined by <axis xyz>, and rpy
defines the base orientation of the child frame.

Since we have STL meshes with correct world positions, we can:
1. Find approximate joint positions from link geometry
2. Determine frame orientations from link bounding boxes
3. Compute the transformation between frames
"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Mesh
import os
import math

def quaternion_to_rpy(q1, q2, q3, q4):
    """Convert quaternion (FreeCAD format Q1,Q2,Q3,Q4) to roll, pitch, yaw"""
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

# STL files (already converted with correct world coordinates)
STL_DIR = "/home/i/ros2_ws/gofa_stl_from_step/stl_local"

# Load meshes and compute centers
print("=== Loading STL Files ===\n")

stl_files = ['base_link.stl', 'link_1.stl', 'link_2.stl', 'link_3.stl',
             'link_4.stl', 'link_5.stl', 'link_6.stl']

links = {}
doc = FreeCAD.newDocument()

for fname in stl_files:
    fpath = os.path.join(STL_DIR, fname)
    Mesh.insert(fpath, doc.Name)

for obj in doc.Objects:
    if hasattr(obj, 'Mesh'):
        mesh = obj.Mesh
        points = mesh.Points
        coords = [(p.x, p.y, p.z) for p in points]

        min_x = min(c[0] for c in coords)
        max_x = max(c[0] for c in coords)
        min_y = min(c[1] for c in coords)
        max_y = max(c[1] for c in coords)
        min_z = min(c[2] for c in coords)
        max_z = max(c[2] for c in coords)

        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        center_z = (min_z + max_z) / 2

        size_x = max_x - min_x
        size_y = max_y - min_y
        size_z = max_z - min_z

        # Determine primary axis (longest dimension)
        sizes = {'X': size_x, 'Y': size_y, 'Z': size_z}
        primary_axis = max(sizes, key=sizes.get)

        link_name = obj.Name.replace('.stl', '')
        links[link_name] = {
            'min': (min_x, min_y, min_z),
            'max': (max_x, max_y, max_z),
            'center': (center_x, center_y, center_z),
            'size': (size_x, size_y, size_z),
            'primary_axis': primary_axis
        }

        print(f"{link_name}:")
        print(f"  Min: ({min_x:.1f}, {min_y:.1f}, {min_z:.1f})")
        print(f"  Max: ({max_x:.1f}, {max_y:.1f}, {max_z:.1f})")
        print(f"  Center: ({center_x:.1f}, {center_y:.1f}, {center_z:.1f})")
        print(f"  Size: ({size_x:.1f}, {size_y:.1f}, {size_z:.1f})")
        print(f"  Primary axis: {primary_axis}")
        print()

FreeCAD.closeDocument(doc.Name)

# Now compute joint transforms
# For each joint, we need to find:
# 1. The joint position in parent frame (xyz)
# 2. The rotation between parent and child frames (rpy)

print("\n=== Computing Joint Transforms ===\n")

# Sort links by Z coordinate (arm goes from base up)
sorted_links = sorted(links.items(), key=lambda x: x[1]['center'][2])

# The robot arm structure (from bounding box analysis):
# base_link: Z range 0-214, X-primary
# link_1: Z range 215-480, X-primary
# link_2: Z range 319-1176, Z-primary (long vertical arm)
# link_3: Z range 1042-1270, Z-primary
# link_4: X range 247-838, X-primary (horizontal forearm)
# link_5: X range 660-855, X-primary
# link_6: Z range 1245-1350, Z-primary

# Physical joint positions (where links connect):
# joint_1: At top of base_link (z=max of base_link) ≈ z=214
# joint_2: At bottom of link_1 (z=min of link_1) ≈ z=215
# joint_3: At bottom of link_2 (z=min of link_2) ≈ z=319
# joint_4: At bottom of link_3 (z=min of link_3) ≈ z=1042
# joint_5: At end of link_4 (x=max of link_4) ≈ x=838
# joint_6: At end of link_5 (x=max of link_5) ≈ x=855

joint_positions = {
    'joint_1': (0, 0, 214),           # Top of base
    'joint_2': (75, 8, 215),          # Shoulder (bottom of link_1)
    'joint_3': (150, -145, 319),      # Elbow (bottom of link_2)
    'joint_4': (165, 12, 1042),       # Wrist start (bottom of link_3)
    'joint_5': (542, 40, 1164),       # Wrist mid (end of link_4)
    'joint_6': (757, -9, 1163),       # Wrist end (end of link_5)
}

# Compute xyz in parent frame
print("Joint origins (xyz in parent frame):")
joint_xyz = {}
parent_joints = [None, 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
child_links = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5']

for i in range(6):
    joint_name = f'joint_{i+1}'
    child_link = child_links[i]

    curr_pos = joint_positions[joint_name]

    if i == 0:
        # First joint: relative to base_link origin
        # base_link origin is at world (0,0,0)
        xyz = curr_pos
    else:
        # Subsequent joints: relative to previous joint
        prev_joint = parent_joints[i]
        prev_pos = joint_positions[prev_joint]
        xyz = (curr_pos[0] - prev_pos[0],
               curr_pos[1] - prev_pos[1],
               curr_pos[2] - prev_pos[2])

    joint_xyz[joint_name] = xyz
    print(f"  {joint_name}: xyz=\"{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}\"")

# For rpy, we need to understand how the child frame should be oriented
# The child frame's primary axis should point along the link
# The rotation between frames depends on the orientation of adjacent links

print("\n=== Computing rpy (rotation between frames) ===\n")

# For each joint, the rpy is the rotation to align parent frame to child frame
# This can be determined by the change in primary axis direction

# The robot structure shows:
# joint_1: base_link(X) -> link_1(X) = minimal rotation
# joint_2: link_1(X) -> link_2(Z) = rotation about Y
# joint_3: link_2(Z) -> link_3(Z) = minimal rotation
# joint_4: link_3(Z) -> link_4(X) = rotation about Y
# joint_5: link_4(X) -> link_5(X) = minimal rotation
# joint_6: link_5(X) -> link_6(Z) = rotation about Y

# For rotations about Y axis:
# X->Z: rotation is +90° (π/2) about Y
# Z->X: rotation is -90° (-π/2) about Y
# Z->Z or X->X: rotation is 0

# But we also need to consider how the mesh is actually oriented within its frame.
# The bounding box tells us the mesh's PRIMARY orientation but not its exact rotation.

# For a more accurate rpy, we need to know:
# 1. Which direction on the mesh points "up" or "out" from the joint
# 2. How that direction aligns with the world coordinate axes

# Without the original CAD frames, we can only estimate based on the
# transition in primary axes and assume the mesh is "straight"

# Let's compute rpy based on the primary axis transition
# and assume the mesh is oriented with its primary axis along the link

print("Estimated rpy based on primary axis transitions:")
print("Note: These assume mesh primary axis aligns with link direction")
print()

axis_transitions = {
    'joint_1': ('X', 'X', 0, 0, 0),
    'joint_2': ('X', 'Z', 0, math.pi/2, 0),   # X->Z needs +90° about Y
    'joint_3': ('Z', 'Z', 0, 0, 0),
    'joint_4': ('Z', 'X', 0, -math.pi/2, 0),  # Z->X needs -90° about Y
    'joint_5': ('X', 'X', 0, 0, 0),
    'joint_6': ('X', 'Z', 0, math.pi/2, 0),   # X->Z needs +90° about Y
}

for joint_name, (parent_axis, child_axis, rx, ry, rz) in axis_transitions.items():
    print(f"  {joint_name}: rpy=\"{rx:.4f} {ry:.4f} {rz:.4f}\"  ({math.degrees(rx):.1f}°, {math.degrees(ry):.1f}°, {math.degrees(rz):.1f}°)")

print("\n=== Summary: Full URDF Joint Parameters ===\n")
print("Joint  | xyz                    | rpy")
print("-------|------------------------|------------------")
for i in range(1, 7):
    jn = f'joint_{i}'
    xyz = joint_xyz[jn]
    trans = axis_transitions[jn]
    rx, ry, rz = trans[2], trans[3], trans[4]
    print(f"{jn}   | {xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f} | {rx:.4f} {ry:.4f} {rz:.4f}")

print("\nNote: These rpy values are ESTIMATES based on bounding box analysis.")
print("The actual values may differ based on internal mesh orientation.")