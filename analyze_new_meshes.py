#!/usr/bin/env python3
"""Analyze new SOLIDWORKS exported STL files."""

import numpy as np
from stl import mesh
import os

FOLDER = "/mnt/hgfs/Linux_shared/Gofa CRB15000_10kg-152 机械臂 技术资料/Gofa 机械臂模型/CRB15000_10kg-152_OmniCore_rev00_STEP_C"

files = [
    ("LINK00 (base_link)", f"{FOLDER}/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step - CRB15000_10kg-152_OmniCore_rev00_LINK00_CAD.step-1.STL"),
    ("LINK01 (link_1)", f"{FOLDER}/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step - CRB15000_10kg-152_OmniCore_rev00_LINK01_CAD.step-1.STL"),
    ("LINK02 (link_2)", f"{FOLDER}/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step - CRB15000_10kg-152_OmniCore_rev00_LINK02_CAD.step-1.STL"),
    ("LINK03 (link_3)", f"{FOLDER}/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step - CRB15000_10kg-152_OmniCore_rev00_LINK03_CAD.step-1.STL"),
    ("LINK04 (link_4)", f"{FOLDER}/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step - CRB15000_10kg-152_OmniCore_rev00_LINK04_CAD.step-1.STL"),
    ("LINK05 (link_5)", f"{FOLDER}/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step - CRB15000_10kg-152_OmniCore_rev00_LINK05_CAD.step-1.STL"),
    ("LINK06 (link_6)", f"{FOLDER}/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step - CRB15000_10kg-152_OmniCore_rev00_LINK06_CAD.step-1.STL"),
]

print("=== 新导出的 SOLIDWORKS STL 坐标分析 ===\n")

for name, filepath in files:
    try:
        m = mesh.Mesh.from_file(filepath)
        points = m.points.reshape(-1, 3)
        min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)
        center = np.mean(points, axis=0)

        print(f"{name}:")
        print(f"  Min: X={min_coords[0]:.2f}, Y={min_coords[1]:.2f}, Z={min_coords[2]:.2f}")
        print(f"  Max: X={max_coords[0]:.2f}, Y={max_coords[1]:.2f}, Z={max_coords[2]:.2f}")
        print(f"  Center: X={center[0]:.2f}, Y={center[1]:.2f}, Z={center[2]:.2f}")
        print(f"  Range Z: {min_coords[2]:.2f} ~ {max_coords[2]:.2f} (Height: {max_coords[2]-min_coords[2]:.2f}mm)")
        print()
    except Exception as e:
        print(f"{name}: Error - {e}\n")

print("\n=== 位置评估 ===")
print("如果 Z 坐标按 LINK00 < LINK01 < LINK02 < ... < LINK06 顺序递增，说明导出正确")
print("LINK00 应该在原点附近，LINK06 应该在约 1350mm 高度")