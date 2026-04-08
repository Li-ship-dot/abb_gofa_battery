#!/usr/bin/env python3
"""Analyze STEP assembly to extract joint transforms for GoFa CRB15000-10/1.52"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Part
import os
import math

# Input assembly file
ASM_PATH = "/mnt/hgfs/Linux_shared/Gofa CRB15000_10kg-152 机械臂 技术资料/Gofa 机械臂模型/CRB15000_10kg-152_OmniCore_rev00_STEP_C/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step"

# Link patterns to identify each link in the assembly
LINK_PATTERNS = {
    "LINK00": "base_link",
    "LINK01": "link_1",
    "LINK02": "link_2",
    "LINK03": "link_3",
    "LINK04": "link_4",
    "LINK05": "link_5",
    "LINK06": "link_6",
}

def analyze_assembly():
    print("=== Analyzing STEP Assembly ===\n")

    # Import assembly
    doc = FreeCAD.newDocument("Assembly")
    Part.insert(ASM_PATH, doc.Name)

    # Find all shapes and identify links
    links = {}
    for obj in doc.Objects:
        if hasattr(obj, 'Shape') and obj.Shape.isValid():
            name = getattr(obj, 'Name', '') or getattr(obj, 'Label', '')
            for pattern, link_name in LINK_PATTERNS.items():
                if pattern in name:
                    links[link_name] = obj
                    print(f"Found: {name} -> {link_name}")
                    break

    if len(links) < 7:
        print(f"\nWarning: Only found {len(links)} links!")

    print("\n=== Link Placements (Global) ===")
    for link_name in ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]:
        if link_name in links:
            obj = links[link_name]
            placement = obj.Shape.Placement
            p = placement.Base
            r = placement.Rotation
            print(f"\n{link_name}:")
            print(f"  Position: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
            # Get Euler angles
            print(f"  Rotation (Q): ({r.Q1:.4f}, {r.Q2:.4f}, {r.Q3:.4f}, {r.Q4:.4f})")

            # Convert quaternion to Euler angles (XYZ convention for URDF)
            # Roll, Pitch, Yaw from quaternion
            rpy = quaternion_to_rpy(r.Q1, r.Q2, r.Q3, r.Q4)
            print(f"  RPY (deg): ({math.degrees(rpy[0]):.2f}, {math.degrees(rpy[1]):.2f}, {math.degrees(rpy[2]):.2f})")

    print("\n=== Joint Transforms (Parent -> Child) ===")
    # Compute transforms between adjacent links
    parent_child_pairs = [
        ("base_link", "link_1"),
        ("link_1", "link_2"),
        ("link_2", "link_3"),
        ("link_3", "link_4"),
        ("link_4", "link_5"),
        ("link_5", "link_6"),
    ]

    for parent_name, child_name in parent_child_pairs:
        if parent_name in links and child_name in links:
            parent_placement = links[parent_name].Shape.Placement
            child_placement = links[child_name].Shape.Placement

            # Relative transform = child_global * inverse(parent_global)
            # In URDF: joint origin is expressed in parent frame
            parent_inv = parent_placement.inverse()
            relative = child_placement * parent_inv

            p = relative.Base
            r = relative.Rotation
            rpy = quaternion_to_rpy(r.Q1, r.Q2, r.Q3, r.Q4)

            joint_num = parent_name.replace("base_link", "1").replace("link_", "")
            print(f"\njoint_{joint_num} ({parent_name} -> {child_name}):")
            print(f"  xyz=\"{p.x:.4f} {p.y:.4f} {p.z:.4f}\"")
            print(f"  rpy=\"{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}\"")
            print(f"  rpy (deg)=\"{math.degrees(rpy[0]):.2f} {math.degrees(rpy[1]):.2f} {math.degrees(rpy[2]):.2f}\"")

    FreeCAD.closeDocument(doc.Name)

def quaternion_to_rpy(q1, q2, q3, q4):
    """Convert quaternion (q1,q2,q3,q4) to roll, pitch, yaw (XZY convention)"""
    # Using FreeCAD's rotation which is (Q1, Q2, Q3, Q4)
    # Convert to rotation matrix first
    from FreeCAD import Vector

    # Build rotation matrix from quaternion
    s = q1*q1 + q2*q2 + q3*q3 + q4*q4
    if s < 1e-10:
        return (0, 0, 0)

    # Normalize
    s = 1.0 / math.sqrt(s)

    # Rotation matrix elements
    r11 = 1 - 2*s*(q2*q2 + q3*q3)
    r12 = 2*s*(q1*q2 + q3*q4)
    r13 = 2*s*(q3*q1 - q2*q4)
    r21 = 2*s*(q1*q2 - q3*q4)
    r22 = 1 - 2*s*(q3*q3 + q1*q1)
    r23 = 2*s*(q2*q3 + q1*q4)
    r31 = 2*s*(q3*q1 + q2*q4)
    r32 = 2*s*(q2*q3 - q1*q4)
    r33 = 1 - 2*s*(q1*q1 + q2*q2)

    # Extract RPY (XZY convention - matches URDF rpy)
    # pitch (y-axis rotation)
    if abs(r31) >= 1:
        pitch = math.copysign(math.pi/2, r31)
        roll = 0
        yaw = math.atan2(r21, r11)
    else:
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)

    return (roll, pitch, yaw)

if __name__ == "__main__":
    analyze_assembly()