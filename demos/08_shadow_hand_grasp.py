
import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer


# ==================================================
# 1. Load Shadow Hand
# ==================================================

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

print("====================================")
print("Demo 08: Shadow Hand Grasping Motion")
print("====================================")
print("Joints:", model.njnt)
print("Actuators:", model.nu)


# ==================================================
# 2. Initialize hand pose
# ==================================================

# Initialize from the home keyframe if available
key_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home"
)

if key_id >= 0:
    mujoco.mj_resetDataKeyframe(model, data, key_id)
else:
    mujoco.mj_resetData(model, data)

# Default controller values
base_ctrl = data.ctrl.copy()

# Ensure the controls are inside their valid ranges
base_ctrl = np.clip(
    base_ctrl,
    model.actuator_ctrlrange[:, 0],
    model.actuator_ctrlrange[:, 1]
)

open_ctrl = base_ctrl.copy()
closed_ctrl = base_ctrl.copy()


# ==================================================
# 3. Define grasping actuator targets
# ==================================================

# Four fingers:
# J3 = proximal joint
# J0 = coupled middle + distal joints

grasp_targets = {

    # Index finger
    "rh_A_FFJ3": 1.10,
    "rh_A_FFJ0": 2.00,

    # Middle finger
    "rh_A_MFJ3": 1.10,
    "rh_A_MFJ0": 2.00,

    # Ring finger
    "rh_A_RFJ3": 1.10,
    "rh_A_RFJ0": 2.00,

    # Little finger
    "rh_A_LFJ3": 1.10,
    "rh_A_LFJ0": 2.00,
    "rh_A_LFJ5": 0.25,

    # Thumb: flexion and opposition-related motions
    "rh_A_THJ5": 0.35,
    "rh_A_THJ4": 0.85,
    "rh_A_THJ2": 0.25,
    "rh_A_THJ1": 0.65
}

controlled_ids = []

for name, target in grasp_targets.items():

    actuator_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        name
    )

    if actuator_id < 0:
        raise ValueError(f"Actuator not found: {name}")

    low, high = model.actuator_ctrlrange[actuator_id]

    # Safely clip every target to its valid range
    closed_ctrl[actuator_id] = np.clip(
        target, low, high
    )

    # Open pose for selected flexion actuators
    open_ctrl[actuator_id] = np.clip(
        0.0, low, high
    )

    controlled_ids.append(actuator_id)

print("\nControlled actuators:", len(controlled_ids))

for name in grasp_targets:
    print(name, "->", grasp_targets[name])


# ==================================================
# 4. Define smooth trajectory
# ==================================================

def smoothstep(x):
    """Smooth interpolation between 0 and 1."""
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def grasp_phase(t):
    """
    One cycle = 8 seconds

    0-3s: Close hand
    3-4s: Hold closed
    4-7s: Open hand
    7-8s: Hold open
    """

    phase = t % 8.0

    if phase < 3.0:
        alpha = smoothstep(phase / 3.0)
        state = "CLOSING"

    elif phase < 4.0:
        alpha = 1.0
        state = "HOLD CLOSED"

    elif phase < 7.0:
        alpha = 1.0 - smoothstep(
            (phase - 4.0) / 3.0
        )
        state = "OPENING"

    else:
        alpha = 0.0
        state = "HOLD OPEN"

    return alpha, state


# ==================================================
# 5. Launch MuJoCo simulation
# ==================================================

data.ctrl[:] = open_ctrl
mujoco.mj_forward(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:

    last_print_second = -1
    last_state = None

    while viewer.is_running():

        start_time = time.perf_counter()

        t = data.time

        # Obtain current grasp state
        alpha, state = grasp_phase(t)

        # Interpolate between open and closed commands
        target_ctrl = (
            (1.0 - alpha) * open_ctrl
            + alpha * closed_ctrl
        )

        data.ctrl[:] = target_ctrl

        # Advance physics
        mujoco.mj_step(model, data)

        # Update visualization
        viewer.sync()

        # Print when action stage changes
        if state != last_state:
            print("\nAction:", state)
            last_state = state

        # Print robot status every second
        second = int(data.time)

        if second != last_print_second:

            last_print_second = second

            print(f"\nTime: {data.time:.2f}s")
            print("Grasp progress:", round(alpha, 3))

            # Print actual proximal finger angles
            for joint_name in [
                "rh_FFJ3",
                "rh_MFJ3",
                "rh_RFJ3",
                "rh_LFJ3",
                "rh_THJ4"
            ]:

                joint_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    joint_name
                )

                if joint_id >= 0:

                    qpos_id = model.jnt_qposadr[joint_id]

                    angle = data.qpos[qpos_id]

                    print(
                        joint_name,
                        f"{angle:.3f} rad"
                    )

        # Approximate real-time playback
        elapsed = time.perf_counter() - start_time

        time.sleep(
            max(0.0, model.opt.timestep - elapsed)
        )
