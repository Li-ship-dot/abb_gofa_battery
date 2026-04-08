#!/usr/bin/env python3
"""Convert individual STEP files to STL using FreeCAD."""

import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

import FreeCAD
import Mesh
import os

# Input and output directories
INPUT_DIR = "/home/i/ros2_ws/gofa_stl_from_step"
OUTPUT_DIR = "/home/i/ros2_ws/gofa_stl_from_step/stl"

# Create output directory
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Files to convert
files = [
    ("CRB15000_10kg-152_Omnicore_rev00_LINK00_CAD.step", "base_link.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK01_CAD.step", "link_1.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK02_CAD.step", "link_2.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK03_CAD.step", "link_3.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK04_CAD.step", "link_4.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK05_CAD.step", "link_5.stl"),
    ("CRB15000_10kg-152_Omnicore_rev00_LINK06_CAD.step", "link_6.stl"),
]

print("=== Converting STEP to STL ===\n")

for step_file, stl_file in files:
    step_path = os.path.join(INPUT_DIR, step_file)
    stl_path = os.path.join(OUTPUT_DIR, stl_file)

    print(f"Processing: {step_file}")

    try:
        # Import STEP
        doc = FreeCAD.newDocument()

        # Try to import the STEP file
        import Import
        shape = Import.insert(step_path, doc.Name)

        if shape is None:
            # Try Part module
            import Part
            shape = Part.Shape()
            shape.read(step_path)

        print(f"  Shape loaded: {shape}")

        # Create mesh from shape
        mesh_obj = doc.addObject("Mesh::Feature", "Mesh")
        mesh = Mesh.Mesh()
        mesh.addMesh(shape.tessellate(0.01))
        mesh_obj.Mesh = mesh

        # Export
        Mesh.export([mesh_obj], stl_path)
        print(f"  Exported: {stl_path}")

        # Get mesh data for analysis
        mesh_data = mesh_obj.Mesh
        points = mesh_data.Points
        if points:
            coords = [(p.x, p.y, p.z) for p in points]
            xs = [c[0] for c in coords]
            ys = [c[1] for c in coords]
            zs = [c[2] for c in coords]
            print(f"  Bounding box:")
            print(f"    X: {min(xs):.1f} to {max(xs):.1f}")
            print(f"    Y: {min(ys):.1f} to {max(ys):.1f}")
            print(f"    Z: {min(zs):.1f} to {max(zs):.1f}")
            print(f"  Center: X={(min(xs)+max(xs))/2:.1f}, Y={(min(ys)+max(ys))/2:.1f}, Z={(min(zs)+max(zs))/2:.1f}")

        FreeCAD.closeDocument(doc.Name)

    except Exception as e:
        print(f"  Error: {e}")
        import traceback
        traceback.print_exc()

    print()

print("=== Done ===")