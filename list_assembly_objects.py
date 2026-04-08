#!/usr/bin/env python3
"""List all objects in STEP assembly"""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Part

ASM_PATH = "/mnt/hgfs/Linux_shared/Gofa CRB15000_10kg-152 机械臂 技术资料/Gofa 机械臂模型/CRB15000_10kg-152_OmniCore_rev00_STEP_C/CRB15000_10kg-152_Omnicore_rev00_ASM_CAD.step"

doc = FreeCAD.newDocument("Assembly")
Part.insert(ASM_PATH, doc.Name)

print("=== All Objects in Assembly ===\n")
for i, obj in enumerate(doc.Objects):
    name = getattr(obj, 'Name', '') or getattr(obj, 'Label', '')
    obj_type = getattr(obj, 'TypeId', '') or type(obj).__name__
    has_shape = hasattr(obj, 'Shape') and obj.Shape.isValid()
    print(f"{i}: Name='{name}' Type='{obj_type}' ShapeValid={has_shape}")

FreeCAD.closeDocument(doc.Name)