
import time
import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path

# =====================================================
# 1. 加载机械臂模型
# =====================================================

# 获取当前 Python 文件所在目录
BASE_DIR = Path(__file__).resolve().parent

# 构造 XML 文件路径
xml_path = BASE_DIR / "models" / "simple_arm.xml"

# 从外部 XML 文件加载机械臂
model = mujoco.MjModel.from_xml_path(str(xml_path))

data = mujoco.MjData(model)

print("Robot arm loaded!")
print("Number of joints:", model.njnt)
print("Number of actuators:", model.nu)

# 获取末端执行器的 site ID
site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "end_effector"
)

# =====================================================
# 2. 开始机械臂仿真
# =====================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    viewer.cam.lookat[:] = [0.45, 0, 0.4]
    viewer.cam.distance = 2.3
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25

    last_print_time = -1

    while viewer.is_running():

        # 当前仿真时间
        t = data.time

        # 三个关节的目标角度（单位：弧度）
        target1 = 0.8 * np.sin(t)
        target2 = 0.5 * np.sin(t * 0.7)
        target3 = 0.7 * np.sin(t * 1.2)

        # 向关节位置控制器发送目标
        data.ctrl[0] = target1
        data.ctrl[1] = target2
        data.ctrl[2] = target3

        # 推进一次物理仿真
        mujoco.mj_step(model, data)

        # 更新可视化窗口
        viewer.sync()

        # 每经过约1秒仿真时间输出一次
        current_second = int(data.time)

        if current_second != last_print_time:

            last_print_time = current_second

            print("\nSimulation time:", round(data.time, 2))

            print("Joint angles:", np.round(data.qpos, 3))

            print(
                "End effector position:",
                np.round(data.site_xpos[site_id], 3)
            )

        # 近似实时运行
        time.sleep(model.opt.timestep)
