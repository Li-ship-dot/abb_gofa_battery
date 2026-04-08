#!/usr/bin/env python3
"""
Compute joint xyz from STL mesh geometry.
For each joint, find where adjacent links physically connect.
"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Mesh
import os
import math

STL_DIR = "/home/i/ros2_ws/gofa_stl_from_step/stl_local"

# Load all STL meshes
print("=== Loading STL Files ===\n")

stl_names = ['base_link.stl', 'link_1.stl', 'link_2.stl', 'link_3.stl',
             'link_4.stl', 'link_5.stl', 'link_6.stl']

meshes = {}
doc = FreeCAD.newDocument()

for fname in stl_names:
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

        link_name = obj.Name.replace('.stl', '')
        meshes[link_name] = {
            'min': (min_x, min_y, min_z),
            'max': (max_x, max_y, max_z),
            'center': ((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2)
        }

        print(f"{link_name}:")
        print(f"  Min: ({min_x:.1f}, {min_y:.1f}, {min_z:.1f})")
        print(f"  Max: ({max_x:.1f}, {max_y:.1f}, {max_z:.1f})")
        print(f"  Center: ({meshes[link_name]['center'][0]:.1f}, {meshes[link_name]['center'][1]:.1f}, {meshes[link_name]['center'][2]:.1f})")
        print()

FreeCAD.closeDocument(doc.Name)

# For a robot arm, adjacent links connect at their ends
# base_link top <-> link_1 bottom
# link_1 top <-> link_2 bottom
# etc.

# The joint origin xyz is expressed in the PARENT frame
# For joint_1: parent=base_link, child=link_1
# For joint_2: parent=link_1, child=link_2, etc.

print("=== Computing Joint Positions in World Coordinates ===\n")

# The connection point between two links is where they meet
# For base_link (Z: 0-214) to link_1 (Z: 215-480):
#   Connection is at Z≈214-215 (top of base, bottom of link_1)

connections = [
    ('base_link', 'link_1', 'joint_1'),   # base_link top <-> link_1 bottom
    ('link_1', 'link_2', 'joint_2'),     # link_1 top <-> link_2 bottom
    ('link_2', 'link_3', 'joint_3'),     # link_2 top <-> link_3 bottom
    ('link_3', 'link_4', 'joint_4'),     # link_3 top <-> link_4 bottom
    ('link_4', 'link_5', 'joint_5'),     # link_4 top <-> link_5 bottom
    ('link_5', 'link_6', 'joint_6'),     # link_5 top <-> link_6 bottom
]

# For each link, determine which end connects to next link
# Looking at Z ranges:
# base_link: Z 0-214 -> top at 214
# link_1: Z 215-480 -> bottom at 215, top at 480
# link_2: Z 319-1176 -> bottom at 319, top at 1176
# link_3: Z 1042-1270 -> bottom at 1042, top at 1270
# link_4: Z 1164-1270 (based on earlier analysis: Z: 1164.2 to 1269.8)
# link_5: Z 1163-1350 (based on earlier analysis)
# link_6: Z 1244-1350 (flange)

# For each link, the "connection point" to parent is at one end
# We need to determine which end based on Z values

joint_world_positions = {}

for parent_name, child_name, joint_name in connections:
    parent = meshes[parent_name]
    child = meshes[child_name]

    p_min = parent['min']
    p_max = parent['max']
    c_min = child['min']
    c_max = child['max']

    print(f"\n{joint_name} ({parent_name} -> {child_name}):")
    print(f"  {parent_name}: Z range [{p_min[2]:.1f}, {p_max[2]:.1f}]")
    print(f"  {child_name}: Z range [{c_min[2]:.1f}, {c_max[2]:.1f}]")

    # Determine connection point
    # Typically: parent's top connects to child's bottom
    parent_top_z = p_max[2]
    child_bottom_z = c_min[2]

    # But there might be overlap or gap
    gap = child_bottom_z - parent_top_z
    print(f"  Gap: {gap:.1f}mm")

    # Use midpoint of connection as joint position
    joint_z = (parent_top_z + child_bottom_z) / 2
    joint_x = (parent['center'][0] + child['center'][0]) / 2
    joint_y = (parent['center'][1] + child['center'][1]) / 2

    # More accurate: use the actual overlap region
    # If there's a gap, links don't connect
    # If there's overlap, they might be welded or we use the overlap center

    joint_world_positions[joint_name] = (joint_x, joint_y, joint_z)
    print(f"  Joint world position: ({joint_x:.1f}, {joint_y:.1f}, {joint_z:.1f})")

# Now compute xyz in parent frame
# For joint_i, parent is link_{i-1}, child is link_i
# xyz = child_frame_origin_in_world - parent_frame_origin_in_world
# But we need to know where each link frame is in world

# Assumption: base_link frame is at world (0, 0, 0)
# For other links, their frame origin is at the previous joint position

print("\n\n=== Computing xyz in Parent Frame ===\n")

# Link frame origins in world coordinates
# base_link frame at world (0, 0, 0)
# link_1 frame at joint_1 world position
# link_2 frame at joint_2 world position, etc.

link_frame_origins = {
    'base_link': (0.0, 0.0, 0.0),
}

# Compute frame origins: link_i frame is at joint_i world position
for i in range(1, 7):
    link_name = f'link_{i}'
    joint_name = f'joint_{i}'
    joint_pos = joint_world_positions[joint_name]
    link_frame_origins[link_name] = joint_pos

print("Link frame origins in world coordinates:")
for name, pos in link_frame_origins.items():
    print(f"  {name}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

# For each joint, compute xyz in parent frame
# joint_i is defined in link_{i-1} frame
# xyz = joint_i_world - link_{i-1}_world

print("\nJoint xyz in parent frame:")
for i in range(1, 7):
    joint_name = f'joint_{i}'
    parent_name = f'link_{i-1}' if i > 1 else 'base_link'

    joint_world = joint_world_positions[joint_name]
    parent_world = link_frame_origins[parent_name]

    xyz = (joint_world[0] - parent_world[0],
           joint_world[1] - parent_world[1],
           joint_world[2] - parent_world[2])

    print(f"  {joint_name} (parent={parent_name}): xyz=\"{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}\"")

print("\n\n=== Summary ===")
print("These xyz values put each joint origin at the physical connection point")
print("between adjacent links, expressed in the parent link's coordinate frame.")