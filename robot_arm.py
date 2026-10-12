
import time
import numpy as np
import mujoco
import mujoco.viewer

# =====================================================
# 1. 使用 MJCF XML 创建机械臂
# =====================================================

xml = """
<mujoco model="MyRobotArm">

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <compiler angle="radian"/>

    <option timestep="0.002" gravity="0 0 0"/>

    <worldbody>

        <!-- 地面 -->
        <geom name="ground"
              type="plane"
              size="3 3 0.1"
              rgba="0.7 0.7 0.7 1"/>

        <!-- 固定底座 -->
        <body name="base" pos="0 0 0.2">

            <geom type="cylinder"
                  size="0.15 0.2"
                  rgba="0.3 0.3 0.3 1"/>

            <!-- 第一个关节：底座旋转 -->
            <body name="link1" pos="0 0 0.2">

                <joint name="joint1"
                       type="hinge"
                       axis="0 0 1"
                       range="-3.14 3.14"
                       damping="2"/>

                <geom type="sphere"
                      size="0.09"
                      rgba="0.2 0.2 0.2 1"/>

                <!-- 第二个关节：肩部 -->
                <body name="link2">

                    <joint name="joint2"
                           type="hinge"
                           axis="0 1 0"
                           range="-1.5 1.5"
                           damping="2"/>

                    <geom type="capsule"
                          fromto="0 0 0 0.55 0 0"
                          size="0.055"
                          rgba="0.1 0.4 0.9 1"/>

                    <!-- 第三个关节：肘部 -->
                    <body name="link3" pos="0.55 0 0">

                        <joint name="joint3"
                               type="hinge"
                               axis="0 1 0"
                               range="-1.5 1.5"
                               damping="2"/>

                        <geom type="capsule"
                              fromto="0 0 0 0.45 0 0"
                              size="0.045"
                              rgba="0.9 0.3 0.2 1"/>

                        <!-- 末端执行器位置 -->
                        <site name="end_effector"
                              pos="0.45 0 0"
                              size="0.065"
                              rgba="0 1 0 1"/>

                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <!-- 关节位置控制器 -->
    <actuator>

        <position name="motor1"
                  joint="joint1"
                  kp="100"
                  kv="15"/>

        <position name="motor2"
                  joint="joint2"
                  kp="100"
                  kv="15"/>

        <position name="motor3"
                  joint="joint3"
                  kp="100"
                  kv="15"/>

    </actuator>

</mujoco>
"""

# =====================================================
# 2. 加载机械臂模型
# =====================================================

model = mujoco.MjModel.from_xml_string(xml)
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
# 3. 开始机械臂仿真
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
        target1 = -0.8
        target2 = 0.5
        target3 = 0.7

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
