
"""控制食指运动到指定角度，并记录 目标角 / 实际角 / 关节力矩。

运行（默认打开 3D 观察窗口，关掉窗口后会把记录写入 CSV）:
    D:/curriculum/nusri/FYP/mujoco/.venv/Scripts/python.exe control_finger.py

不开窗口、只安静地跑一段时间取数据:
    ... control_finger.py --no-viewer --duration 2.0

记录文件 control_finger_torque.csv 每一行是一个关节在一个时间步上的:
    时间 / 关节 / 目标角(rad) / 实际角(rad) / 误差(rad) / 关节力矩(N·m)

"关节力矩"取 data.qfrc_actuator（执行器作用在该关节自由度上的广义力，单位 N·m）。
正号表示执行器朝正方向驱动关节，负号表示朝负方向（例如抵消重力/阻尼）。
运行结束会打印每个关节的峰值力矩和稳态力矩，用来回答
"把手指摆到这个角度需要多大力矩"。
"""

import argparse
import csv
import pathlib
import time

import mujoco
import mujoco.viewer

ROOT = pathlib.Path(__file__).resolve().parent
MODEL_PATH = ROOT / "leap_hand" / "scene_right.xml"
CSV_PATH = ROOT / "control_finger_torque.csv"

# 食指的四个关节及其对应执行器
joint_names = [
    "if_mcp",
    "if_rot",
    "if_pip",
    "if_dip",
]

actuator_names = [
    "if_mcp_act",
    "if_rot_act",
    "if_pip_act",
    "if_dip_act",
]

# 目标角度，单位是 rad
target_angles = [0.4, 0.0, 0.4, 0.4]

CSV_HEADER = [
    "time_s",
    "joint",
    "target_rad",
    "actual_rad",
    "error_rad",
    "torque_Nm",
]

# 每隔多少秒在终端打印一行当前状态（<=0 表示只在结束时汇总）
PRINT_INTERVAL = 0.25
# 结束时用最后这段时间的平均力矩当作"稳态力矩"
SETTLE_WINDOW = 0.5


def object_id(model, obj_type, name, what):
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise ValueError(f"找不到{what}: {name}")
    return obj_id


def run(show_viewer, duration, csv_path):
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    joint_ids = [
        object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name, "关节")
        for name in joint_names
    ]
    actuator_ids = [
        object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name, "执行器")
        for name in actuator_names
    ]
    # qpos 下标用来读实际角度，dof 下标用来读关节力矩
    qpos_ids = [model.jnt_qposadr[jid] for jid in joint_ids]
    dof_ids = [model.jnt_dofadr[jid] for jid in joint_ids]

    # 检查目标值是否在每个执行器允许的控制范围内
    for name, actuator_id, target in zip(
        actuator_names, actuator_ids, target_angles
    ):
        low, high = model.actuator_ctrlrange[actuator_id]
        print(f"{name}: target={target:.3f}, ctrlrange=[{low:.3f}, {high:.3f}]")

        if not low <= target <= high:
            raise ValueError(f"{name} 的目标角度超出控制范围")

    records = []  # 每行: [时间, 关节名, 目标角, 实际角, 误差, 关节力矩]
    peak = {name: (0.0, 0.0) for name in joint_names}  # 关节名 -> (|力矩|峰值, 时间)
    last_print = -1.0

    def step_and_record():
        nonlocal last_print

        for actuator_id, target in zip(actuator_ids, target_angles):
            data.ctrl[actuator_id] = target

        mujoco.mj_step(model, data)

        should_print = PRINT_INTERVAL > 0 and data.time - last_print >= PRINT_INTERVAL
        if should_print:
            last_print = data.time
            print(f"\n--- t = {data.time:.2f} s ---")

        for index, name in enumerate(joint_names):
            target = target_angles[index]
            actual = float(data.qpos[qpos_ids[index]])
            # 关节力矩：执行器施加在该关节上的广义力，单位 N·m
            torque = float(data.qfrc_actuator[dof_ids[index]])
            error = target - actual

            records.append([data.time, name, target, actual, error, torque])
            if abs(torque) > peak[name][0]:
                peak[name] = (abs(torque), float(data.time))

            if should_print:
                print(
                    f"{name}: target={target:+.3f} actual={actual:+.3f} "
                    f"err={error:+.3f} torque={torque:+.3f} N·m"
                )

    if show_viewer:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                step_and_record()
                viewer.sync()
                time.sleep(model.opt.timestep)
    else:
        steps = max(1, int(duration / model.opt.timestep))
        for _ in range(steps):
            step_and_record()

    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_HEADER)
        writer.writerows(records)

    # 汇总：峰值力矩 + 最后 SETTLE_WINDOW 秒的平均力矩
    t_end = records[-1][0] if records else 0.0
    print("\n==== 关节力矩汇总 ====")
    print(
        "（峰值 = 全程 |力矩| 最大；稳态 = 最后 "
        f"{SETTLE_WINDOW:.1f}s 的平均力矩）"
    )
    for index, name in enumerate(joint_names):
        samples = [
            row[5]
            for row in records
            if row[1] == name and row[0] >= t_end - SETTLE_WINDOW
        ]
        steady = sum(samples) / len(samples) if samples else 0.0
        actual = float(data.qpos[qpos_ids[index]])
        print(
            f"{name}: target={target_angles[index]:+.3f} actual={actual:+.3f} rad | "
            f"峰值|torque|={peak[name][0]:.3f} N·m (t={peak[name][1]:.2f}s) | "
            f"稳态torque={steady:+.3f} N·m"
        )

    print(f"\n记录 {len(records)} 行 -> {csv_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="不开 3D 窗口，按 --duration 跑完就退出（适合只取数据）",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="--no-viewer 模式下模拟多少秒（默认 3.0）",
    )
    parser.add_argument(
        "--csv",
        type=pathlib.Path,
        default=CSV_PATH,
        help="记录文件",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    run(
        show_viewer=not args.no_viewer,
        duration=args.duration,
        csv_path=args.csv,
    )


if __name__ == "__main__":
    main()
