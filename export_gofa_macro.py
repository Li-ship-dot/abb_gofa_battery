import FreeCAD
import Part
import Mesh
import os

STEP_FILE = "/tmp/gofa_assembly.step"
OUTPUT_DIR = "/home/i/ros2_ws/src/abb_omnicore_ros2/robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual"

LINK_MAPPING = {
    "LINK00": "base_link.stl",
    "LINK01": "link_1.stl",
    "LINK02": "link_2.stl",
    "LINK03": "link_3.stl",
    "LINK04": "link_4.stl",
    "LINK05": "link_5.stl",
    "LINK06": "link_6.stl",
}

print("=== Exporting GoFa Links ===")

doc = FreeCAD.open(STEP_FILE)
print("Opened: " + doc.Name)

for link_name, output_filename in LINK_MAPPING.items():
    print("Processing " + link_name + "...")

    link_obj = None
    for obj in doc.Objects:
        obj_name = getattr(obj, 'Name', '')
        obj_label = getattr(obj, 'Label', '')
        if link_name in obj_name or link_name in obj_label:
            link_obj = obj
            break

    if link_obj is None:
        print("  Not found, skipping")
        continue

    if not hasattr(link_obj, "Shape"):
        print("  No Shape, skipping")
        continue

    gp = link_obj.getGlobalPlacement()
    print("  Global Placement: Pos=" + str(gp.Base) + ", Rot=" + str(gp.Rotation))

    shape = link_obj.Shape.copy()
    mesh = Mesh.Mesh()
    mesh.addMesh(shape.tessellate(0.01))

    output_path = os.path.join(OUTPUT_DIR, output_filename)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    mesh.write(output_path)
    print("  -> " + output_path)

FreeCAD.closeDocument(doc.Name)
print("Done!")