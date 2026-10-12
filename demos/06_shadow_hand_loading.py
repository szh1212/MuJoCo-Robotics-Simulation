
from pathlib import Path
import time

import mujoco
import mujoco.viewer

# 1. Shadow Hand 场景路径
PROJECT_DIR = Path(__file__).resolve().parent.parent

SCENE_PATH = (
    PROJECT_DIR
    / "third_party"
    / "mujoco_menagerie"
    / "shadow_hand"
    / "scene_right.xml"
)

# 2. 加载模型
model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
data = mujoco.MjData(model)

print("=== Demo 06: Shadow Hand Loading ===")
print("Model file:", SCENE_PATH.name)
print("Bodies:", model.nbody)
print("Joints:", model.njnt)
print("Actuators:", model.nu)
print("Geoms:", model.ngeom)

# 3. 打印关节
for i in range(model.njnt):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        i
    )
    print(f"Joint {i}: {name}")

# 4. 打印执行器
for i in range(model.nu):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )
    print(f"Actuator {i}: {name}")

# 5. 打开可视化窗口
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():
        viewer.sync()
        time.sleep(0.01)
