#!/usr/bin/env python3
"""Compute rpy values from mesh geometry analysis"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Mesh
import Part
import os
import math
import numpy as np

STL_DIR = "/home/i/ros2_ws/gofa_stl_from_step/stl_local"

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

def rpy_to_quaternion(roll, pitch, yaw):
    """Convert roll, pitch, yaw to quaternion"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q1 = cr * cp * cy + sr * sp * sy
    q2 = sr * cp * cy - cr * sp * sy
    q3 = cr * sp * cy + sr * cp * sy
    q4 = cr * cp * sy - sr * sp * cy

    return (q1, q2, q3, q4)

def load_stl(filepath):
    doc = FreeCAD.newDocument()
    Mesh.insert(filepath, doc.Name)
    return doc.Objects[0].Mesh, doc

def get_mesh_info(mesh):
    """Get mesh bounding box and center"""
    points = mesh.Points
    coords = np.array([(p.x, p.y, p.z) for p in points])

    min_coords = coords.min(axis=0)
    max_coords = coords.max(axis=0)
    center = coords.mean(axis=0)

    return {
        'min': tuple(min_coords),
        'max': tuple(max_coords),
        'center': tuple(center),
        'size': tuple(max_coords - min_coords)
    }

print("=== Computing rpy from Mesh Geometry ===\n")

# Analyze each link
links = ['base_link.stl', 'link_1.stl', 'link_2.stl', 'link_3.stl',
         'link_4.stl', 'link_5.stl', 'link_6.stl']

infos = {}
for fname in links:
    fpath = os.path.join(STL_DIR, fname)
    mesh, doc = load_stl(fpath)
    info = get_mesh_info(mesh)
    infos[fname.replace('.stl', '')] = info
    FreeCAD.closeDocument(doc.Name)

# For each joint, compute the required frame transformation
# rpy describes how child frame is oriented relative to parent frame
# when both are expressed in parent's coordinate system

print("=== Link Geometries ===")
for name, info in infos.items():
    print(f"\n{name}:")
    print(f"  Size: X={info['size'][0]:.1f}, Y={info['size'][1]:.1f}, Z={info['size'][2]:.1f}")
    print(f"  Center: ({info['center'][0]:.1f}, {info['center'][1]:.1f}, {info['center'][2]:.1f})")

# Compute transformations between adjacent links
print("\n\n=== Computing Joint Transforms ===")

joint_pairs = [
    ('base_link', 'link_1', 'joint_1'),
    ('link_1', 'link_2', 'joint_2'),
    ('link_2', 'link_3', 'joint_3'),
    ('link_3', 'link_4', 'joint_4'),
    ('link_4', 'link_5', 'joint_5'),
    ('link_5', 'link_6', 'joint_6'),
]

# Current URDF values from abb_gofa_support
current_rpy = {
    'joint_1': (0, 0, 0),
    'joint_2': (0, 0.340, 0),
    'joint_3': (0, -0.577, 0),
    'joint_4': (0.145, 0, 0),
    'joint_5': (0, 0.205, 0),
    'joint_6': (1.429, 0, 0),
}

# My STL-derived xyz
my_xyz = {
    'joint_1': (0, 0.040, 0.001),
    'joint_2': (0, -0.205, -0.161),
    'joint_3': (0, 0.204, -0.134),
    'joint_4': (0, 0.042, -0.105),
    'joint_5': (0, -0.115, -0.107),
    'joint_6': (0, 0.037, -0.105),
}

for parent_name, child_name, joint_name in joint_pairs:
    parent_info = infos[parent_name]
    child_info = infos[child_name]

    # Connection vector (parent to child in world frame)
    conn_vec = np.array(child_info['center']) - np.array(parent_info['center'])

    # Parent link's "extension direction" (longest axis)
    parent_size = np.array(parent_info['size'])
    parent_longest_axis = np.argmax(parent_size)  # 0=X, 1=Y, 2=Z

    # Child link's "extension direction" (longest axis)
    child_size = np.array(child_info['size'])
    child_longest_axis = np.argmax(child_size)

    print(f"\n{joint_name} ({parent_name} -> {child_name}):")
    print(f"  Parent size: {parent_size}, longest axis: {parent_longest_axis} ({'XYZ'[parent_longest_axis]})")
    print(f"  Child size: {child_size}, longest axis: {child_longest_axis} ({'XYZ'[child_longest_axis]})")
    print(f"  Connection vector: ({conn_vec[0]:.2f}, {conn_vec[1]:.2f}, {conn_vec[2]:.2f})")

    # The axis of rotation is defined by the joint
    axis_map = {
        'joint_1': (0, 0, 1),   # Z
        'joint_2': (0, 1, 0),   # Y
        'joint_3': (0, 1, 0),   # Y
        'joint_4': (1, 0, 0),   # X
        'joint_5': (0, 1, 0),   # Y
        'joint_6': (1, 0, 0),   # X
    }
    axis = np.array(axis_map[joint_name])

    # For the child frame to be oriented correctly when joint is at 0,
    # we need the child frame's Z axis (or primary axis) to point towards the child link

    # Estimate: the rpy rotation should align the child's primary axis with the connection direction
    # But this is a simplification - the actual orientation also depends on how the mesh is designed

    # Calculate the rotation needed to go from parent's frame to child's frame
    # This is a simplified estimation based on the change in primary axis direction

    parent_dir = np.zeros(3)
    parent_dir[parent_longest_axis] = 1

    child_dir = np.zeros(3)
    child_dir[child_longest_axis] = 1

    # Cross product gives rotation axis
    rotation_axis = np.cross(parent_dir, child_dir)
    rotation_angle = np.arccos(np.dot(parent_dir, child_dir))

    if np.linalg.norm(rotation_axis) > 0.001:
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        print(f"  Estimated rotation: axis=({rotation_axis[0]:.2f}, {rotation_axis[1]:.2f}, {rotation_axis[2]:.2f}), angle={np.degrees(rotation_angle):.1f}deg")

    # Current rpy value
    curr_rpy = current_rpy[joint_name]
    print(f"  Current rpy (ABB): ({curr_rpy[0]:.3f}, {curr_rpy[1]:.3f}, {curr_rpy[2]:.3f}) = ({np.degrees(curr_rpy[0]):.1f}deg, {np.degrees(curr_rpy[1]):.1f}deg, {np.degrees(curr_rpy[2]):.1f}deg)")

    # What rpy should be based on geometry
    # For a Z->X transition, we need rotation about Y
    # For a X->Z transition, we need rotation about Y
    # For a Z->Z transition, we might need rotation about X or Y

    if parent_longest_axis == 0 and child_longest_axis == 2:  # X -> Z
        # Rotation about Y
        est_rpy = (0, rotation_angle, 0)
        print(f"  Estimated rpy (geometry): (0, {rotation_angle:.3f}, 0) = (0, {np.degrees(rotation_angle):.1f}deg, 0)")
    elif parent_longest_axis == 2 and child_longest_axis == 0:  # Z -> X
        # Rotation about Y (negative)
        est_rpy = (0, -rotation_angle, 0)
        print(f"  Estimated rpy (geometry): (0, {-rotation_angle:.3f}, 0) = (0, {np.degrees(-rotation_angle):.1f}deg, 0)")
    elif parent_longest_axis == 2 and child_longest_axis == 2:  # Z -> Z
        # Could be rotation about X or Y depending on orientation
        est_rpy = (0, 0, 0)  # Assume no rotation needed
        print(f"  Estimated rpy (geometry): (0, 0, 0) - same orientation")
    elif parent_longest_axis == 0 and child_longest_axis == 0:  # X -> X
        est_rpy = (0, 0, 0)
        print(f"  Estimated rpy (geometry): (0, 0, 0) - same orientation")
    else:
        print(f"  Cannot estimate rpy for this transition")

print("\n\n=== Summary ===")
print("The geometry suggests:")
print("- joint_1 (base->link1): Should have small or no rpy (X->X)")
print("- joint_2 (link1->link2): X->Z transition, needs Y rotation")
print("- joint_3 (link2->link3): Z->Z transition, may need no Y rotation")
print("- joint_4 (link3->link4): Z->X transition, needs Y rotation")
print("- joint_5 (link4->link5): X->X transition, may need no rotation")
print("- joint_6 (link5->link6): X->Z transition, needs Y rotation")

print("\nBUT: These are only estimates based on bounding box analysis.")
print("The actual rpy also depends on internal mesh design, not just overall shape.")