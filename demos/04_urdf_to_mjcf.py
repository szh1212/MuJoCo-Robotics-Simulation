
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

# ============================================
# 1. Define file paths
# ============================================

PROJECT_DIR = Path(__file__).resolve().parent.parent

URDF_PATH = PROJECT_DIR / "models" / "simple_arm.urdf"
MJCF_PATH = PROJECT_DIR / "models" / "converted_arm.xml"

# ============================================
# 2. Load URDF and export MJCF
# ============================================

model = mujoco.MjModel.from_xml_path(str(URDF_PATH))

print("URDF loaded successfully!")
print("Joints:", model.njnt)
print("Geoms:", model.ngeom)

mujoco.mj_saveLastXML(str(MJCF_PATH), model)

# ============================================
# 3. Add position actuators automatically
# ============================================

tree = ET.parse(MJCF_PATH)
root = tree.getroot()

# Remove any existing actuators to avoid duplicates
old_actuator = root.find("actuator")

if old_actuator is not None:
    root.remove(old_actuator)

# Create actuator section
actuator = ET.SubElement(root, "actuator")

for i in range(1, 4):

    ET.SubElement(
        actuator,
        "position",
        {
            "name": f"motor{i}",
            "joint": f"joint{i}",
            "kp": "100",
            "kv": "15"
        }
    )

# Save the updated MJCF
ET.indent(tree, space="    ")
tree.write(
    MJCF_PATH,
    encoding="utf-8",
    xml_declaration=True
)

# ============================================
# 4. Verify the generated model
# ============================================

final_model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))

assert final_model.njnt == 3, "Expected 3 joints!"
assert final_model.nu == 3, "Expected 3 actuators!"

print("\n=== Conversion Completed ===")
print("Output:", MJCF_PATH)
print("Joints:", final_model.njnt)
print("Actuators:", final_model.nu)
print("Geoms:", final_model.ngeom)

