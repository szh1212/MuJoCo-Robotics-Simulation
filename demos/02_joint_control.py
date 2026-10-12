
import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

# 1. 加载机械臂
PROJECT_DIR = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_DIR / "models" / "simple_arm.xml"

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "end_effector"
)

# 2. 设定三个目标姿态（弧度）
poses = [
    np.array([0.0, 0.0, 0.0]),
    np.array([-0.8, 0.5, 0.7]),
    np.array([0.8, -0.5, -0.7])
]

print("=== Demo 02: Joint Position Control ===")
print("Joints:", model.njnt)
print("Actuators:", model.nu)

# 3. 启动仿真
with mujoco.viewer.launch_passive(model, data) as viewer:

    viewer.cam.lookat[:] = [0.45, 0, 0.4]
    viewer.cam.distance = 2.3
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25

    last_pose_index = -1
    last_print_second = -1

    while viewer.is_running():

        start_time = time.time()

        # 每3秒切换一个目标姿态
        pose_index = int(data.time / 3.0) % len(poses)
        target = poses[pose_index]

        # 发送位置控制指令
        data.ctrl[:] = target

        # 检测是否切换姿态
        if pose_index != last_pose_index:
            last_pose_index = pose_index
            print(f"\nMoving to Pose {pose_index + 1}")
            print("Target angles:", target)

        # 推进物理仿真
        mujoco.mj_step(model, data)
        viewer.sync()

        # 每秒打印状态
        current_second = int(data.time)

        if current_second != last_print_second:
            last_print_second = current_second

            actual = data.qpos.copy()
            error = target - actual

            print(f"\nTime: {data.time:.2f}s")
            print("Target:", np.round(target, 3))
            print("Actual:", np.round(actual, 3))
            print("Error:", np.round(error, 3))
            print(
                "End effector:",
                np.round(data.site_xpos[site_id], 3)
            )

        elapsed = time.time() - start_time
        time.sleep(max(0, model.opt.timestep - elapsed))
