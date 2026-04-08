#!/usr/bin/env python3
"""Detailed analysis of STEP assembly objects"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Part
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

ASM_PATH = "/mnt/hgfs/Linux_shared/Gofa CRB15000_10kg-152 机械臂 技术资料/Gofa 机械臂模型/CRB15000_10kg-152_OmniCore_rev00_STEP_C/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step"

doc = FreeCAD.newDocument("Assembly")
Part.insert(ASM_PATH, doc.Name)

print("=== Assembly Object Analysis ===\n")

objects = []
for i, obj in enumerate(doc.Objects):
    if hasattr(obj, 'Shape') and obj.Shape.isValid():
        bb = obj.Shape.BoundBox
        placement = obj.Shape.Placement
        p = placement.Base
        r = placement.Rotation
        # FreeCAD uses Q which is a tuple (x, y, z, w)
        q = r.Q
        rpy = quaternion_to_rpy(q[0], q[1], q[2], q[3])

        objects.append({
            'index': i,
            'name': obj.Name,
            'center': (p.x, p.y, p.z),
            'rpy_deg': (math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2])),
            'bb_center': (bb.Center.x, bb.Center.y, bb.Center.z),
            'bb_size': (bb.XLength, bb.YLength, bb.ZLength)
        })

        print(f"Object {i}: {obj.Name}")
        print(f"  Placement: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        print(f"  RPY (deg): ({math.degrees(rpy[0]):.2f}, {math.degrees(rpy[1]):.2f}, {math.degrees(rpy[2]):.2f})")
        print(f"  BoundBox Center: ({bb.Center.x:.2f}, {bb.Center.y:.2f}, {bb.Center.z:.2f})")
        print(f"  BoundBox Size: ({bb.XLength:.2f}, {bb.YLength:.2f}, {bb.ZLength:.2f})")
        print()

# Sort by Z coordinate to get approximate order from base to tip
print("\n=== Sorted by Z (ascending) ===")
sorted_by_z = sorted(objects, key=lambda x: x['center'][2])
for obj in sorted_by_z:
    print(f"{obj['name']}: Z={obj['center'][2]:.2f}")

# Try to identify which is base (should have smallest Z range, closest to 0)
# and compute relative transforms between adjacent sorted objects
print("\n=== Pairwise Transforms (Z-sorted) ===")
for i in range(len(sorted_by_z) - 1):
    obj_a = sorted_by_z[i]
    obj_b = sorted_by_z[i + 1]

    # Get actual placements
    pa = doc.Objects[obj_a['index']].Shape.Placement
    pb = doc.Objects[obj_b['index']].Shape.Placement

    # Relative transform in parent frame
    pa_inv = pa.inverse()
    rel = pb * pa_inv

    p = rel.Base
    r = rel.Rotation
    q = r.Q
    rpy = quaternion_to_rpy(q[0], q[1], q[2], q[3])

    print(f"\n{obj_a['name']} -> {obj_b['name']}:")
    print(f"  xyz=\"{p.x:.4f} {p.y:.4f} {p.z:.4f}\"")
    print(f"  rpy (rad)=\"{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}\"")
    print(f"  rpy (deg)=\"{math.degrees(rpy[0]):.2f} {math.degrees(rpy[1]):.2f} {math.degrees(rpy[2]):.2f}\"")

FreeCAD.closeDocument(doc.Name)