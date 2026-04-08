import sys
sys.path.insert(0, '/usr/lib/freecad-python3/lib')

# Initialize FreeCAD
import FreeCAD

print("=== Exporting GoFa Links ===")
print("FreeCAD version: " + str(FreeCAD.Version()))

STEP_FILE = "/tmp/gofa_assembly.step"

# Check file
import os
if not os.path.exists(STEP_FILE):
    print("ERROR: STEP file not found")
    sys.exit(1)

print("STEP file size: " + str(os.path.getsize(STEP_FILE)) + " bytes")

# Try to import STEP using Part module
print("Trying Part.open...")
try:
    import Part
    shape = Part.open(STEP_FILE)
    print("Part.open result type: " + str(type(shape)))
    print("Shape is valid: " + str(shape.isValid()))
except Exception as e:
    print("Part.open error: " + str(e))
    import traceback
    traceback.print_exc()

print("Done!")