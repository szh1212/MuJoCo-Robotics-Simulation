
import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer


# 1. 加载机械臂模型
PROJECT_DIR = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_DIR / "models" / "simple_arm.xml"

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

print("=== Demo 01: Automatic Joint Motion ===")
print("Model:", MODEL_PATH.name)
print("Number of joints:", model.njnt)
print("Number of actuators:", model.nu)

# 获取末端执行器的 ID
site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "end_effector"
)

# 2. 启动仿真
with mujoco.viewer.launch_passive(model, data) as viewer:

    viewer.cam.lookat[:] = [0.45, 0, 0.4]
    viewer.cam.distance = 2.3
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25

    last_print_second = -1

    while viewer.is_running():

        start_time = time.time()
        t = data.time

        # 3. 设置周期运动目标
        target_angles = np.array([
            0.8 * np.sin(t),
            0.5 * np.sin(0.7 * t),
            0.7 * np.sin(1.2 * t)
        ])

        # 发送三个关节的目标角度
        data.ctrl[:] = target_angles

        # 4. 推进一步仿真
        mujoco.mj_step(model, data)
        viewer.sync()

        # 5. 输出实际关节角度和末端坐标
        current_second = int(data.time)

        if current_second != last_print_second:
            last_print_second = current_second

            print(f"\nSimulation time: {data.time:.2f}s")
            print("Target angles:",
                  np.round(target_angles, 3))
            print("Actual angles:",
                  np.round(data.qpos, 3))
            print("End effector:",
                  np.round(data.site_xpos[site_id], 3))

        # 近似保持实时仿真速度
        elapsed = time.time() - start_time
        time.sleep(max(0, model.opt.timestep - elapsed))
