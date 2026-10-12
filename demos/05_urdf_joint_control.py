
import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

# ============================================
# 1. Load the URDF-converted MJCF model
# ============================================

PROJECT_DIR = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_DIR / "models" / "converted_arm.xml"

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

print("=== Demo 05: URDF Joint Position Control ===")
print("Model:", MODEL_PATH.name)
print("Joints:", model.njnt)
print("Actuators:", model.nu)

# Verify three actuators
assert model.nu == 3, "Expected 3 actuators!"

# ============================================
# 2. Define target joint configurations
# ============================================

poses = [
    np.array([0.0, 0.0, 0.0]),
    np.array([-0.8, 0.5, 0.7]),
    np.array([0.8, -0.5, -0.7])
]

# ============================================
# 3. Launch simulation
# ============================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    viewer.cam.lookat[:] = [0.45, 0, 0.4]
    viewer.cam.distance = 2.3
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25

    last_pose_index = -1
    last_print_second = -1

    while viewer.is_running():

        start_time = time.time()

        # Switch target pose every 3 seconds
        pose_index = int(data.time / 3.0) % len(poses)
        target = poses[pose_index]

        # Send control commands
        data.ctrl[:] = target

        if pose_index != last_pose_index:
            last_pose_index = pose_index
            print(f"\nMoving to Pose {pose_index + 1}")

        # Step physics
        mujoco.mj_step(model, data)
        viewer.sync()

        # Print status every second
        current_second = int(data.time)

        if current_second != last_print_second:
            last_print_second = current_second

            actual = data.qpos.copy()
            error = target - actual

            print(f"\nSimulation time: {data.time:.2f}s")
            print("Target:", np.round(target, 3))
            print("Actual:", np.round(actual, 3))
            print("Error:", np.round(error, 3))

        elapsed = time.time() - start_time
        time.sleep(max(0, model.opt.timestep - elapsed))
