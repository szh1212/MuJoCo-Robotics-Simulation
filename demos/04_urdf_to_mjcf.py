
from pathlib import Path
import mujoco

PROJECT_DIR = Path(__file__).resolve().parent.parent

urdf_path = PROJECT_DIR / "models" / "simple_arm.urdf"
output_path = PROJECT_DIR / "models" / "converted_arm.xml"

# 加载 URDF
model = mujoco.MjModel.from_xml_path(str(urdf_path))

# 将编译后的模型保存为 MJCF
mujoco.mj_saveLastXML(
    str(output_path),
    model
)

print("URDF successfully converted to MJCF!")
print("Output:", output_path)
print("Joints:", model.njnt)
print("Geoms:", model.ngeom)
