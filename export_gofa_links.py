#!/usr/bin/env python3
"""
FreeCAD script to export individual robot links from STEP assembly
while preserving their relative positions in the assembly.
"""

import sys
import os

# Add FreeCAD module path
freecad_path = "/usr/lib/freecad-python3/lib"
if os.path.exists(freecad_path):
    sys.path.insert(0, freecad_path)

try:
    import FreeCAD
    import Part
    import Mesh
except ImportError as e:
    print(f"Error importing FreeCAD modules: {e}")
    print("Trying alternative import method...")
    import subprocess
    # Try running FreeCAD in console mode with a macro
    sys.exit(1)

# Configuration
STEP_FILE = "/mnt/hgfs/Linux_shared/Gofa CRB15000_10kg-152 机械臂 技术资料/Gofa 机械臂模型/CRB15000_10kg-152_OmniCore_rev00_STEP_C/CRB15000_10kg-152_OmniCore_rev00_ASM_CAD.step"
OUTPUT_DIR = "/home/i/ros2_ws/src/abb_omnicore_ros2/robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual"

# Link mapping: Assembly component name -> Output STL name
LINK_MAPPING = {
    "LINK00": "base_link.stl",
    "LINK01": "link_1.stl",
    "LINK02": "link_2.stl",
    "LINK03": "link_3.stl",
    "LINK04": "link_4.stl",
    "LINK05": "link_5.stl",
    "LINK06": "link_6.stl",
}

def export_assembly_links():
    """Export each link from STEP assembly with global position preserved."""

    print(f"=== Exporting GoFa CRB15000 Links ===")
    print(f"STEP file: {STEP_FILE}")
    print(f"Output dir: {OUTPUT_DIR}")

    # Import STEP assembly
    print(f"\nImporting STEP file...")
    doc = FreeCAD.open(STEP_FILE)

    print(f"Document: {doc.Name}")

    # List all objects
    print(f"Objects in document:")
    for obj in doc.Objects:
        if hasattr(obj, "Shape"):
            print(f"  - {obj.Name} (Type: {obj.TypeId})")

    exported_count = 0

    for link_name, output_filename in LINK_MAPPING.items():
        print(f"\nSearching for {link_name}...")

        # Find the object
        link_obj = None
        for obj in doc.Objects:
            obj_name = getattr(obj, 'Name', '')
            obj_label = getattr(obj, 'Label', '')
            if link_name in obj_name or link_name in obj_label:
                link_obj = obj
                print(f"  Found: {obj_name} / {obj_label}")
                break

        if link_obj is None:
            print(f"  Warning: {link_name} not found, skipping...")
            continue

        if not hasattr(link_obj, "Shape"):
            print(f"  Warning: {link_name} has no Shape, skipping...")
            continue

        print(f"  Placement: {link_obj.Placement}")

        # Get global placement
        global_placement = link_obj.getGlobalPlacement()
        print(f"  Global Placement: {global_placement}")

        # Get the shape with placement applied
        shape = link_obj.Shape.copy()
        shape.Placement = global_placement

        # Create mesh from shape
        mesh_data = Mesh.Mesh()
        mesh_data.addMesh(shape.tessellate(0.01))

        # Ensure output directory exists
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, output_filename)

        # Export to STL
        mesh_data.write(output_path)
        print(f"  Exported: {output_path}")

        exported_count += 1

    print(f"\n=== Complete: {exported_count}/{len(LINK_MAPPING)} links exported ===")

    # Close document
    FreeCAD.closeDocument(doc.Name)

if __name__ == "__main__":
    export_assembly_links()