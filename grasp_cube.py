"""LEAP Hand 右手抓取小方块：张开 -> 闭合 -> 抬升 + 横移 -> 拔出力判定。

palm 上装了 3 条滑轨（palm_slide_x / _y / _z，见 leap_hand/right_hand.xml），
手掌可以像龙门架一样在三个方向平动。流程：
    张开静置 -> 平滑闭合 -> 保持
    -> 抬升 z +8cm -> 横移 x +8cm -> 横移 y +8cm   (三个方向距离/速度一致, 2cm/s)
    -> 沿 +z（离开掌心方向）加力做拔出力测试

跑法:
    D:/curriculum/nusri/FYP/mujoco/.venv/Scripts/python.exe grasp_cube.py
    ... --viewer     边跑边看（需要图形界面）
    ... --move 0.10  三个方向各移动 10cm
    ... --close 1.3  闭合幅度改成 BASE_POSE*1.3（默认 1.6，越小捏得越松）

记录到 grasp_records.csv（每一步都记）:
    时间 / 方块 xyz / 手掌 xyz / 方块上的接触数量 / 接触法向力之和 / 接触合力大小
每个方向移完还会汇报"方块相对掌心的偏移"，用来判断移动过程中有没有滑落。

判定为什么不看"抬起来没掉":
    掌心朝上时方块本来就压在掌面上，掌心法向力自己就能把它抬上去（实测拇指
    完全张开也照样跟着升 10cm）。所以判据用拔出力：拔不动才算真的抓住。
"""

from __future__ import annotations

import argparse
import csv
import pathlib

import numpy as np

import mujoco

ROOT = pathlib.Path(__file__).resolve().parent
SCENE_XML = ROOT / "leap_hand" / "scene_grasp.xml"
CSV_PATH = ROOT / "grasp_records.csv"

JOINT_NAMES = [
    "if_mcp", "if_rot", "if_pip", "if_dip",
    "mf_mcp", "mf_rot", "mf_pip", "mf_dip",
    "rf_mcp", "rf_rot", "rf_pip", "rf_dip",
    "th_cmc", "th_axl", "th_mcp", "th_ipl",
]
ACTUATOR_NAMES = [f"{name}_act" for name in JOINT_NAMES]

# 手掌的三条滑轨执行器，顺序固定为 x / y / z，正值 = 世界坐标正方向
PALM_ACTUATORS = ["palm_slide_x_act", "palm_slide_y_act", "palm_slide_z_act"]
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
# 移动顺序：先抬起来，再横移
MOVE_ORDER = ("z", "x", "y")

# 张开：手指伸直，拇指略微抬起（与 control_hand.py 的 open_angles 一致）
OPEN_POSE = np.array([0.0] * 12 + [0.0, 0.0, 0.2, 0.2], dtype=float)

# 闭合：四指弯曲 + 拇指从对侧捏过来。
BASE_POSE = np.array(
    [0.85, 0.0, 0.95, 0.55] * 3 + [1.10, 0.55, 0.90, 0.35], dtype=float
)
# 闭合幅度 = BASE_POSE * CLOSE_SCALE。想自己试就改这一行，或者跑命令时加
# --close 1.3（不用改代码）。参考解用 1.3 倍（那是 4.4cm 方块）；5cm 方块
# 用 1.3 倍只抬 z 还行，一横移方块就容易从指笼里滑出去。越小捏得越松。
CLOSE_SCALE = 1.6

CUBE_MASS = 0.05
WEIGHT = CUBE_MASS * 9.81

# 时间轴（秒）
SETTLE_TIME = 0.6        # 张开静置：让方块落到掌心里
CLOSE_TIME = 1.2         # 平滑闭合
HOLD_TIME = 0.8          # 闭合后保持
MOVE_DISTANCE = 0.08     # 每个方向移动的距离 (m)，三个方向一样
MOVE_TIME = 4.0          # 每个方向的耗时（8cm / 4s = 2cm/s，慢到看不出惯性）
POST_MOVE_TIME = 0.5     # 每个方向移完再保持一段，观察有没有慢慢滑
PULL_TIME = 6.0          # 拔出力加载耗时
PULL_FMAX = 60.0         # 最大拔出力 (N)
SLIP_DISTANCE = 0.03     # 相对抓稳位置移动 3cm 即判定被拽走
SUCCESS_PULL_RATIO = 2.0     # 拔出力 >= 2 倍自重才算"真的抓起来"
DROP_RELATIVE = 0.04     # 相对掌心的偏移超过 4cm 视为滑落
FLOOR_CUBE_Z = 0.06      # 方块 z 掉到这个高度以下视为落地

# 抓住方块之后才算数的阶段（用来统计接触峰值 / 判断有没有滑落）
GRIP_PHASES = (
    "hold", "lift", "post_z", "move_x", "post_x", "move_y", "post_y",
)

CSV_HEADER = [
    "time_s",
    "phase",
    "cube_x_m",
    "cube_y_m",
    "cube_z_m",
    "palm_x_m",
    "palm_y_m",
    "palm_z_m",
    "contact_count",
    "normal_force_sum_N",
    "resultant_force_N",
]
PHASE_COLUMN = CSV_HEADER.index("phase")
CONTACT_COLUMN = CSV_HEADER.index("contact_count")


def object_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    object_id_value = mujoco.mj_name2id(model, obj_type, name)
    if object_id_value < 0:
        raise ValueError(f"模型里找不到 {name}")
    return object_id_value


def clip_pose(model: mujoco.MjModel, pose: np.ndarray) -> np.ndarray:
    """把 16 个目标角度裁到关节限位和执行器控制范围之间。"""
    result = pose.astype(float, copy=True)
    for index, joint_name in enumerate(JOINT_NAMES):
        joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        actuator_id = object_id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint_name}_act"
        )
        result[index] = np.clip(result[index], *model.jnt_range[joint_id])
        result[index] = np.clip(result[index], *model.actuator_ctrlrange[actuator_id])
    return result


def set_joint_targets(
    model: mujoco.MjModel, data: mujoco.MjData, targets: np.ndarray
) -> None:
    for actuator_name, target in zip(ACTUATOR_NAMES, targets):
        actuator_id = object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        data.ctrl[actuator_id] = float(target)


def contact_metrics(
    model: mujoco.MjModel, data: mujoco.MjData, geom_id: int
) -> tuple[int, float, float]:
    """方块上的接触数量、接触法向力之和、接触合力大小。

    mj_contactForce 给的是接触坐标系下的 6 维力。当 geom1 是方块时取反，
    统一成"作用在方块上"的力，多个接触的合力才有方向意义。
    """
    contact_count = 0
    normal_force_sum = 0.0
    force_vector = np.zeros(3)
    buffer = np.zeros(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.geom1 != geom_id and contact.geom2 != geom_id:
            continue
        contact_count += 1
        mujoco.mj_contactForce(model, data, index, buffer)
        force = buffer[:3].copy()
        if contact.geom1 == geom_id:
            force = -force
        normal_force_sum += abs(float(buffer[0]))
        force_vector += force
    return contact_count, normal_force_sum, float(np.linalg.norm(force_vector))


def run(
    show_viewer: bool = False,
    move_distance: float = MOVE_DISTANCE,
    csv_path: pathlib.Path = CSV_PATH,
    close_scale: float = CLOSE_SCALE,
) -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)

    cube_geom = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube")
    cube_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    palm_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "palm")
    palm_actuators = [
        object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in PALM_ACTUATORS
    ]

    open_pose = clip_pose(model, OPEN_POSE)
    grasp_pose = clip_pose(model, BASE_POSE * close_scale)

    dt = model.opt.timestep
    records: list[tuple] = []
    palm_target = np.zeros(3)

    viewer = None
    viewer_open = [False]  # 用列表是为了在下面的闭包里能改
    if show_viewer:
        # mujoco.viewer 是子模块，`import mujoco` 不会自动把它带进来，必须显式导入；
        # 而且不能用 `import mujoco.viewer`（那会把 mujoco 变成 run() 的局部名）
        from mujoco import viewer as _mj_viewer

        viewer = _mj_viewer.launch_passive(model, data)
        viewer_open[0] = True

    def step(pose: np.ndarray, phase: str) -> None:
        set_joint_targets(model, data, pose)
        for actuator, value in zip(palm_actuators, palm_target):
            data.ctrl[actuator] = float(value)
        mujoco.mj_step(model, data)
        contact_count, normal_force_sum, resultant = contact_metrics(
            model, data, cube_geom
        )
        records.append(
            (
                data.time,
                phase,
                float(data.xpos[cube_body][0]),
                float(data.xpos[cube_body][1]),
                float(data.xpos[cube_body][2]),
                float(data.xpos[palm_body][0]),
                float(data.xpos[palm_body][1]),
                float(data.xpos[palm_body][2]),
                contact_count,
                normal_force_sum,
                resultant,
            )
        )
        if viewer is not None and viewer_open[0]:
            viewer.sync()
            if not viewer.is_running():  # 用户把窗口关了: 剩下的算完, 数据照样记录
                viewer_open[0] = False
                viewer.close()
                print("(窗口已关闭, 剩下阶段继续跑完, 数据照样记录)")

    def execute(pose: np.ndarray, seconds: float, phase: str) -> None:
        for _ in range(max(1, int(seconds / dt))):
            step(pose, phase)

    def move_axis(axis_name: str, distance: float, seconds: float) -> None:
        """沿某个世界坐标轴平滑移动手掌（smoothstep，准静态）。"""
        axis = AXIS_INDEX[axis_name]
        start = palm_target.copy()
        end = palm_target.copy()
        end[axis] += distance
        steps = max(1, int(seconds / dt))
        for step_index in range(steps):
            alpha = (step_index + 1) / steps
            blend = 3.0 * alpha**2 - 2.0 * alpha**3
            palm_target[:] = start + blend * (end - start)
            step(grasp_pose, f"move_{axis_name}")
        palm_target[:] = end

    # ---- 阶段 0：张开静置，方块落到掌心上 ----
    execute(open_pose, SETTLE_TIME, "settle_open")
    cube_z_start = float(data.xpos[cube_body][2])

    # ---- 阶段 1：平滑闭合（smoothstep，避免撞飞方块）----
    close_steps = max(1, int(CLOSE_TIME / dt))
    for step_index in range(close_steps):
        alpha = (step_index + 1) / close_steps
        blend = 3.0 * alpha**2 - 2.0 * alpha**3
        step(open_pose + blend * (grasp_pose - open_pose), "close")

    # ---- 阶段 2：保持，量一下闭合后的接触情况 ----
    execute(grasp_pose, HOLD_TIME, "hold")
    cube_z_hold = float(data.xpos[cube_body][2])
    palm_z_hold = float(data.xpos[palm_body][2])
    contacts_hold, force_hold, _ = contact_metrics(model, data, cube_geom)
    # 掌心朝上判据：手掌接触面法向是 palm 体的局部 -z，在世界系里应该指向 +z
    palm_normal_world = -data.xmat[palm_body].reshape(3, 3)[:, 2]
    palm_up = float(palm_normal_world[2])
    # 抓稳时方块相对掌心的位置，之后每个方向移完都拿它比对
    relative_reference = data.xpos[cube_body].copy() - data.xpos[palm_body].copy()

    # ---- 阶段 3~5：先抬升 z，再横移 x、y，三个方向距离一样 ----
    stage_results = []
    for axis_name in MOVE_ORDER:
        move_axis(axis_name, move_distance, MOVE_TIME)
        execute(grasp_pose, POST_MOVE_TIME, f"post_{axis_name}")
        cube_now = data.xpos[cube_body].copy()
        palm_now = data.xpos[palm_body].copy()
        relative_error = float(np.linalg.norm((cube_now - palm_now) - relative_reference))
        on_floor = float(cube_now[2]) < FLOOR_CUBE_Z
        stage_results.append(
            {
                "axis": axis_name,
                "distance": move_distance,
                "cube_position": cube_now,
                "palm_position": palm_now,
                "relative_error": relative_error,
                "dropped": bool(on_floor or relative_error > DROP_RELATIVE),
            }
        )

    cube_z_lifted = float(data.xpos[cube_body][2])
    palm_z_lifted = float(data.xpos[palm_body][2])

    # ---- 阶段 6：拔出力测试 ----
    data.xfrc_applied[cube_body, :] = 0.0
    cube_position_before = data.xpos[cube_body].copy()
    slip_force = float("nan")
    pull_steps = max(1, int(PULL_TIME / dt))
    for step_index in range(pull_steps):
        force_now = PULL_FMAX * (step_index + 1) / pull_steps
        data.xfrc_applied[cube_body, 2] = force_now
        step(grasp_pose, "pull_out")
        if float(data.xpos[cube_body][2] - cube_position_before[2]) > SLIP_DISTANCE:
            slip_force = force_now
            break
    data.xfrc_applied[cube_body, :] = 0.0
    if slip_force != slip_force:  # NaN -> 拉满都没拽走
        slip_force = PULL_FMAX

    if viewer is not None and viewer_open[0]:
        viewer.close()

    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_HEADER)
        writer.writerows(records)

    pull_ratio = slip_force / WEIGHT
    # 闭合瞬间的接触数会抖（方块可能还在轻微挪动），所以取抓住之后各阶段的峰值
    contacts_peak = max(
        (
            row[CONTACT_COLUMN]
            for row in records
            if row[PHASE_COLUMN] in GRIP_PHASES
        ),
        default=0,
    )
    any_dropped = any(stage["dropped"] for stage in stage_results)
    grasped = (
        palm_up > 0.9
        and pull_ratio >= SUCCESS_PULL_RATIO
        and contacts_peak >= 2
        and not any_dropped
    )
    return {
        "csv": csv_path,
        "records": len(records),
        "cube_z_start": cube_z_start,
        "cube_z_hold": cube_z_hold,
        "cube_z_lifted": cube_z_lifted,
        "palm_z_hold": palm_z_hold,
        "palm_z_lifted": palm_z_lifted,
        "contacts_hold": contacts_hold,
        "contacts_peak": contacts_peak,
        "force_hold": force_hold,
        "palm_up": palm_up,
        "tracked": cube_z_lifted - cube_z_hold,
        "move_distance": move_distance,
        "stages": stage_results,
        "any_dropped": any_dropped,
        "slip_force": slip_force,
        "pull_ratio": pull_ratio,
        "grasped": grasped,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="边跑边看（需要图形界面）")
    parser.add_argument("--move", type=float, default=MOVE_DISTANCE, help="每个方向移动距离 (m)")
    parser.add_argument("--close", type=float, default=CLOSE_SCALE, help="闭合幅度 = BASE_POSE * 该倍数（默认 1.6）")
    parser.add_argument("--csv", type=pathlib.Path, default=CSV_PATH, help="记录文件")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = run(
        show_viewer=args.viewer,
        move_distance=args.move,
        csv_path=args.csv,
        close_scale=args.close,
    )

    print("LEAP Hand 右手抓方块")
    print(f"  闭合幅度  BASE_POSE x {args.close:.2f}")
    print(f"  掌心朝上  法向·世界向上 = {result['palm_up']:.3f}（1.0 = 完全朝上）")
    print(f"  初始      方块 z = {result['cube_z_start']:.4f} m")
    print(f"  闭合后    方块 z = {result['cube_z_hold']:.4f} m"
          f"  手掌 z = {result['palm_z_hold']:.4f} m"
          f"  接触 {result['contacts_hold']} 个"
          f"  法向力合计 {result['force_hold']:.2f} N")
    print(f"  接触峰值  {result['contacts_peak']} 个（抓住之后的各阶段）")
    print(f"  各方向移动（每个方向 {result['move_distance'] * 100:.0f}cm / {MOVE_TIME:.0f}s，"
          f"方块相对掌心的偏移）")
    for stage in result["stages"]:
        cube_now = np.asarray(stage["cube_position"])
        palm_now = np.asarray(stage["palm_position"])
        print(f"    {stage['axis']} {stage['distance'] * 100:+.0f}cm  "
              f"手掌=({palm_now[0]:+.3f}, {palm_now[1]:+.3f}, {palm_now[2]:.3f})  "
              f"方块=({cube_now[0]:+.3f}, {cube_now[1]:+.3f}, {cube_now[2]:.3f})  "
              f"偏移 {stage['relative_error'] * 1000:5.1f} mm  "
              f"{'掉了' if stage['dropped'] else '没掉'}")
    print(f"  拔出力    {result['slip_force']:.2f} N"
          f" = {result['pull_ratio']:.1f} 倍自重（自重 {WEIGHT:.2f} N）")
    print(f"  记录      {result['records']} 行 -> {result['csv']}")
    print()
    print("判定: 真的抓起来了" if result["grasped"] else "判定: 没抓住（详见上面各项）")
    print("（只移动/抬升不掉不算数：掌心朝上时掌心自己就能托住方块）")


if __name__ == "__main__":
    main()
