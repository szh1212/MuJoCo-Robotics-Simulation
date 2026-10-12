import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

# ==========================================
# 1. Load Shadow Hand Model
# ==========================================

PROJECT_DIR = Path(__file__).resolve().parent.parent

SCENE_PATH = (
    PROJECT_DIR
    / "third_party"
    / "mujoco_menagerie"
    / "shadow_hand"
    / "scene_right.xml"
)

model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
data = mujoco.MjData(model)

print("=== Demo 07: Shadow Hand Finger Control ===")
print("Number of joints:", model.njnt)
print("Number of actuators:", model.nu)

# ==========================================
# 2. Find Finger Actuators
# ==========================================

finger_names = [
    "rh_A_THJ4",  # Thumb 拇指
    "rh_A_FFJ3",  # Index 食指
    "rh_A_MFJ3",  # Middle 中指
    "rh_A_RFJ3",  # Ring 无名指
    "rh_A_LFJ3"   # Little 小指
]

finger_ids = []

for name in finger_names:
    actuator_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        name
    )

    if actuator_id == -1:
        raise ValueError(f"Actuator not found: {name}")

    finger_ids.append(actuator_id)

# ==========================================
# 3. Define Control Targets
# ==========================================

# Use the model's home keyframe if available
key_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home"
)

if key_id >= 0:
    mujoco.mj_resetDataKeyframe(model, data, key_id)

base_ctrl = data.ctrl.copy()

# Keep commands within allowed ranges
base_ctrl = np.clip(
    base_ctrl,
    model.actuator_ctrlrange[:, 0],
    model.actuator_ctrlrange[:, 1]
)

open_ctrl = base_ctrl.copy()
bent_ctrl = base_ctrl.copy()

for actuator_id in finger_ids:
    open_ctrl[actuator_id] = 0.0
    bent_ctrl[actuator_id] = 0.8

# ==========================================
# 4. Launch Simulation
# ==========================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    last_print_second = -1

    while viewer.is_running():

        start_time = time.perf_counter()
        t = data.time

        # Smooth flexion-extension cycle (3 seconds)
        alpha = 0.5 * (
            1 - np.cos(2 * np.pi * t / 3.0)
        )

        target = (
            (1 - alpha) * open_ctrl
            + alpha * bent_ctrl
        )

        data.ctrl[:] = target

        # Advance the physics simulation
        mujoco.mj_step(model, data)
        viewer.sync()

        # Print control signals every second
        second = int(data.time)

        if second != last_print_second:
            last_print_second = second

            print(f"\nTime: {data.time:.2f}s")
            print(
                "Finger targets:",
                np.round(data.ctrl[finger_ids], 3)
            )

            # 打印五根手指的实际关节角度
            print("\nActual finger joint angles:")

            for name in [
                "rh_FFJ3",
                "rh_MFJ3",
                "rh_RFJ3",
                "rh_LFJ3",
                "rh_THJ4"
            ]:
                joint_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    name
                )

                if joint_id >= 0:
                    qpos_id = model.jnt_qposadr[joint_id]
                    print(name, round(float(data.qpos[qpos_id]), 3))

        # Approximate real-time playback
        elapsed = time.perf_counter() - start_time
        time.sleep(
            max(0, model.opt.timestep - elapsed)
        )

