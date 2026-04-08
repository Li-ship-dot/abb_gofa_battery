# FreeCAD batch macro for converting STEP to STL
import FreeCAD
import Mesh
import Part
import os

INPUT_DIR = "/home/i/ros2_ws/gofa_stl_from_step"
OUTPUT_DIR = "/home/i/ros2_ws/gofa_stl_from_step/stl_local"
os.makedirs(OUTPUT_DIR, exist_ok=True)

files = [
    ("CRB15000_10kg-152_Omnicore_rev00_LINK00_CAD.step", "base_link.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK01_CAD.step", "link_1.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK02_CAD.step", "link_2.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK03_CAD.step", "link_3.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK04_CAD.step", "link_4.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK05_CAD.step", "link_5.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK06_CAD.step", "link_6.stl"),
]

print("=== FreeCAD STEP to STL ===")

for step_name, stl_name in files:
    step_path = os.path.join(INPUT_DIR, step_name)
    stl_path = os.path.join(OUTPUT_DIR, stl_name)
    print(f"\n{step_name}")

    try:
        doc = FreeCAD.newDocument()
        Part.insert(step_path, doc.Name)

        shape = None
        for obj in doc.Objects:
            if hasattr(obj, 'Shape') and obj.Shape.isValid():
                shape = obj.Shape
                break

        if shape is None:
            print("  ERROR: No shape")
            FreeCAD.closeDocument(doc.Name)
            continue

        bb = shape.BoundBox
        print(f"  BoundBox: X({bb.XMin:.0f},{bb.XMax:.0f}) Y({bb.YMin:.0f},{bb.YMax:.0f}) Z({bb.ZMin:.0f},{bb.ZMax:.0f})")
        print(f"  Center: ({bb.Center.x:.0f}, {bb.Center.y:.0f}, {bb.Center.z:.0f})")

        # Tessellate and export
        mesh_data = shape.tessellate(0.01)
        mesh = Mesh.Mesh(mesh_data)

        # Create mesh object for export
        mesh_obj = doc.addObject("Mesh::Feature", "Mesh")
        mesh_obj.Mesh = mesh

        Mesh.export([mesh_obj], stl_path)
        print(f"  -> {stl_path}")

        FreeCAD.closeDocument(doc.Name)

    except Exception as e:
        print(f"  ERROR: {e}")
        try:
            FreeCAD.closeDocument(doc.Name)
        except:
            pass

print("\n=== Done ===")