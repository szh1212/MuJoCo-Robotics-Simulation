
from pathlib import Path

import mujoco
import mujoco.viewer

# 1. 找到 URDF 文件
PROJECT_DIR = Path(__file__).resolve().parent.parent
URDF_PATH = PROJECT_DIR / "models" / "simple_arm.urdf"

# 2. 加载 URDF 模型
model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
data = mujoco.MjData(model)

print("=== Demo 03: URDF Robot Loading ===")
print("Model:", URDF_PATH.name)
print("Number of joints:", model.njnt)
print("Number of actuators:", model.nu)
print("Number of geoms:", model.ngeom)
print("Number of bodies:", model.nbody)

for i in range(model.ngeom):
    print(
        "Geom", i,
        "Type:", model.geom_type[i],
        "Position:", model.geom_pos[i]
    )

# 3. 输出关节名称
for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        i
    )
    print(f"Joint {i}: {joint_name}")

# 4. 打开 MuJoCo 可视化窗口
import time

with mujoco.viewer.launch_passive(model, data) as viewer:

    # 设置摄像机
    viewer.cam.lookat[:] = [0.45, 0, 0.4]
    viewer.cam.distance = 2.3
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25

    # 保持窗口运行
    while viewer.is_running():

        viewer.sync()

        time.sleep(0.01)

