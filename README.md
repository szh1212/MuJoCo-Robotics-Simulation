# MuJoCo Robotics Simulation

**From Robot Description to Joint Control and Dexterous Hand Motion**

![Python](https://img.shields.io/badge/Python-3.11-blue) ![MuJoCo](https://img.shields.io/badge/MuJoCo-3.15.0-blue) ![Status](https://img.shields.io/badge/Status-8%20Demos-brightgreen) ![Model](https://img.shields.io/badge/Model-Shadow%20Hand-purple)

An educational robotics simulation project developed with **MuJoCo**, covering **MJCF/URDF robot modeling**, forward simulation, **joint-space position control**, robot model conversion, and **Shadow Hand dexterous manipulation**. This repository documents eight progressively developed experiments, including a reproducible multi-joint hand open/close motion sequence. It does **not** yet demonstrate stable object grasping or a physically combined arm–hand system.

> **Meeting demo roadmap:** describe a robot → load it in MuJoCo → command and inspect joints → compare MJCF and URDF workflows → load and control the Shadow Hand → examine measured motion across four cycles.

## 1. Demo gallery (8 experiments)

**Video setup:** Replace each preview GIF and MP4 with your **own recorded simulation**. GitHub README reliably displays looping GIF previews; for actual playable MP4 clips, open the [video gallery](https://szh1212.github.io/MuJoCo-Robotics-Simulation/) after enabling GitHub Pages from `/docs`. Until the videos are added, image previews may appear broken and links may not resolve. **These are placeholders, not claimed results.**

| Demo | Focus | Preview | Full video |
|:---|:---|:---:|:---:|
| **01** | MJCF: automatic periodic arm motion | ![Demo 01](assets/demo01.gif) | [Watch MP4](docs/videos/demo01.mp4) |
| **02** | MJCF: target poses & tracking error | ![Demo 02](assets/demo02.gif) | [Watch MP4](docs/videos/demo02.mp4) |
| **03** | URDF: load and visualize a 3-DOF arm | ![Demo 03](assets/demo03.gif) | [Watch MP4](docs/videos/demo03.mp4) |
| **04** | URDF → MJCF conversion (terminal demo) | ![Demo 04](assets/demo04.gif) | [Watch MP4](docs/videos/demo04.mp4) |
| **05** | URDF-derived MJCF: joint position control | ![Demo 05](assets/demo05.gif) | [Watch MP4](docs/videos/demo05.mp4) |
| **06** | Shadow Hand model loading & joint inventory | ![Demo 06](assets/demo06.gif) | [Watch MP4](docs/videos/demo06.mp4) |
| **07** | Five-finger cyclic motion | ![Demo 07](assets/demo07.gif) | [Watch MP4](docs/videos/demo07.mp4) |
| **08** | Multi-actuator open–close hand gesture | ![Demo 08](assets/demo08.gif) | [Watch MP4](docs/videos/demo08.mp4) |

**Note:** Demo 04 is a console-based model conversion, so its video should record the script output and the generated `converted_arm.xml` rather than imply a separate physical action. For readable embeds, produce cropped GIFs of approximately 3–6 seconds and link the full MP4 separately.

## 2. Project structure

```text
MuJoCo-Robotics-Simulation/
├── README.md                     # Overview, workflows, parameters, experimental results
├── requirements.txt              # Python dependencies
├── .gitignore                    # Excludes local environments and third-party assets
├── shadow_hand_info.txt          # Recorded Shadow Hand joints and actuator parameters
├── models/
│   ├── simple_arm.xml            # Native MJCF: 3-DOF robot arm
│   ├── simple_arm.urdf           # URDF: links, joints, visual and inertial elements
│   └── converted_arm.xml         # Generated MJCF with position actuators
├── demos/
│   ├── 01_arm_motion.py          # Sinusoidal arm joint targets
│   ├── 02_joint_control.py       # Three target poses and joint tracking errors
│   ├── 03_urdf_loading.py        # URDF import and inspection
│   ├── 04_urdf_to_mjcf.py        # Conversion + automatic actuator injection
│   ├── 05_urdf_joint_control.py  # Control the converted arm model
│   ├── 06_shadow_hand_loading.py # Official Shadow Hand import and enumeration
│   ├── 07_shadow_hand_control.py # Selected five-finger periodic control
│   └── 08_shadow_hand_grasp.py   # Multi-actuator open/close gesture (4 cycles tested)
├── assets/                       # Add your demo01.gif ... demo08.gif here
├── docs/
│   ├── index.html                # Optional GitHub Pages MP4 gallery
│   └── videos/                   # Add your demo01.mp4 ... demo08.mp4 here
└── third_party/
    └── mujoco_menagerie/        # Download locally; intentionally NOT tracked by Git
        └── shadow_hand/
            └── scene_right.xml
```

## 3. Eight experiments: what each file does

| Script | Robot/model | Implementation | What to observe |
|:---|:---|:---|:---|
| `01_arm_motion.py` | `models/simple_arm.xml` | Sends sinusoidal joint position targets to three actuators | Repeated arm motion; `data.ctrl`, `data.qpos`, end-effector position |
| `02_joint_control.py` | `models/simple_arm.xml` | Switches among three fixed joint-space target poses every 3 s | Setpoint following and per-joint error `target - actual` |
| `03_urdf_loading.py` | `models/simple_arm.urdf` | Loads URDF, enumerates joints and geometry, displays the model | Three revolute joints; URDF `link`/`joint` hierarchy |
| `04_urdf_to_mjcf.py` | URDF → `models/converted_arm.xml` | Exports compiled MJCF and inserts three position actuators automatically | Generated model loads with `njnt=3`, `nu=3` |
| `05_urdf_joint_control.py` | `models/converted_arm.xml` | Controls the URDF-derived arm through MuJoCo actuators | Target, actual joint angles, and tracking error |
| `06_shadow_hand_loading.py` | `shadow_hand/scene_right.xml` | Loads and visualizes the official right-hand model, lists joints/actuators | Hand geometry, articulation inventory |
| `07_shadow_hand_control.py` | Shadow Hand | Periodic control of four fingers and one thumb actuator | Finger target changes and measured joint motion |
| `08_shadow_hand_grasp.py` | Shadow Hand | Coordinates **13 actuators** to close, hold, reopen and hold the hand | Eight-second gesture cycle and repeatability over four cycles |

**Important terminology:** `joint` defines permitted relative motion, `actuator` provides a control pathway, and `data.qpos` stores generalized positions. `data.ctrl` is an actuator **input**, not necessarily an actual joint angle. The Shadow Hand includes tendon-coupled actuation.

## 4. Modeling and simulation workflow

### MJCF: native MuJoCo description

`models/simple_arm.xml` uses nested `<body>` elements with `hinge` joints, `geom` elements for physical geometry, and `<position>` actuators. The teaching arm has **3 revolute joints** and **3 position actuators**. The original MJCF demo uses zero gravity to focus on joint control rather than gravity compensation.

### URDF: a separate robot-description representation

`models/simple_arm.urdf` specifies the same conceptual arm using explicit `<link>` and `<joint>` definitions, parent/child relationships, and `<origin>` and `<axis>` transforms. The URDF initially imports with **three movable joints and zero MuJoCo actuators**. This project retains visual geometry on import using a MuJoCo URDF extension:

```xml
<mujoco>
    <compiler discardvisual="false"/>
</mujoco>
```

The conversion demo exports MJCF and automatically appends three position actuators. This avoids the earlier fragile workflow of manually editing a generated XML file after every conversion. **Run Demo 04 before Demo 05 if `converted_arm.xml` is missing or needs regeneration.** Differences in dynamics between the two arm models can result from differences in geometry, inertia and gravity settings; the two models should not be treated as a controlled apples-to-apples performance benchmark.

## 5. Shadow Hand: model and actuator information

The Shadow Hand model is sourced from [Google DeepMind MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand), using `shadow_hand/scene_right.xml`. The values below reproduce the **locally recorded actuator inventory** from the model used in these demos (see `shadow_hand_info.txt`); they are not universal specifications for every Shadow Hand revision.

| Property | Recorded value |
|:---|---:|
| Model | Shadow Hand, right hand |
| Scene | `scene_right.xml` |
| Joints | **25** |
| Actuators | **20** |
| Actuators commanded in Demo 08 | **13** |
| Tested gesture cycle duration | **8 s** |
| Recorded complete cycles | **4** |

### Actuator inventory

`Gain` below means the logged `actuator_gainprm[i, 0]`, **not necessarily motor torque**. Transmission values are MuJoCo enum codes: **0 = joint; 3 = tendon**, not site.

| ID | Actuator | Control range | Transmission | Gain |
|---:|:---|:---|:---|---:|
| 0 | `rh_A_WRJ2` | [-0.523599, 0.174533] | Joint (0) | 10.0 |
| 1 | `rh_A_WRJ1` | [-0.698132, 0.488692] | Joint (0) | 8.0 |
| 2 | `rh_A_THJ5` | [-1.047200, 1.047200] | Joint (0) | 0.4 |
| 3 | `rh_A_THJ4` | [0.000000, 1.221730] | Joint (0) | 1.0 |
| 4 | `rh_A_THJ3` | [-0.209440, 0.209440] | Joint (0) | 0.5 |
| 5 | `rh_A_THJ2` | [-0.698132, 0.698132] | Joint (0) | 1.5 |
| 6 | `rh_A_THJ1` | [-0.261799, 1.570800] | Joint (0) | 1.0 |
| 7 | `rh_A_FFJ4` | [-0.349066, 0.349066] | Joint (0) | 1.0 |
| 8 | `rh_A_FFJ3` | [-0.261799, 1.570800] | Joint (0) | 1.0 |
| 9 | `rh_A_FFJ0` | [0.000000, 3.141500] | Tendon (3) | 0.5 |
| 10 | `rh_A_MFJ4` | [-0.349066, 0.349066] | Joint (0) | 1.0 |
| 11 | `rh_A_MFJ3` | [-0.261799, 1.570800] | Joint (0) | 1.0 |
| 12 | `rh_A_MFJ0` | [0.000000, 3.141500] | Tendon (3) | 0.5 |
| 13 | `rh_A_RFJ4` | [-0.349066, 0.349066] | Joint (0) | 1.0 |
| 14 | `rh_A_RFJ3` | [-0.261799, 1.570800] | Joint (0) | 1.0 |
| 15 | `rh_A_RFJ0` | [0.000000, 3.141500] | Tendon (3) | 0.5 |
| 16 | `rh_A_LFJ5` | [0.000000, 0.785398] | Joint (0) | 1.0 |
| 17 | `rh_A_LFJ4` | [-0.349066, 0.349066] | Joint (0) | 1.0 |
| 18 | `rh_A_LFJ3` | [-0.261799, 1.570800] | Joint (0) | 1.0 |
| 19 | `rh_A_LFJ0` | [0.000000, 3.141500] | Tendon (3) | 0.5 |

**Naming:** WR = wrist, TH = thumb, FF = index finger, MF = middle finger, RF = ring finger, LF = little finger. The `J0` tendon actuators drive coupled finger motions; their numerical control values must not be interpreted as angles of one single joint.

## 6. Demo 08: four-cycle experimental results

**Experiment:** Coordinate 13 actuators to produce an open → close → reopen hand gesture. The motion uses smoothstep interpolation between open and closed control targets. Each eight-second cycle consists of **0–3 s closing**, **3–4 s hold closed**, **4–7 s opening**, and **7–8 s hold open**. All reported angles below are **measured joint positions from `data.qpos`**, in radians, not control targets.

### 6.1 Actual joint angles at closed-hold samples

| Cycle | Time (s) | FFJ3 | MFJ3 | RFJ3 | LFJ3 | THJ4 |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 4 | 1.058 | 1.093 | 1.093 | 1.086 | 0.773 |
| 2 | 12 | 1.058 | 1.093 | 1.097 | 1.082 | 0.773 |
| 3 | 20 | 1.058 | 1.093 | 1.099 | 1.080 | 0.774 |
| 4 | 28 | 1.058 | 1.093 | 1.093 | 1.086 | 0.774 |

### 6.2 Actual joint angles at open-hold samples

| Cycle | Time (s) | FFJ3 | MFJ3 | RFJ3 | LFJ3 | THJ4 |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 8 | -0.026 | -0.026 | -0.026 | -0.026 | -0.002 |
| 2 | 16 | -0.026 | -0.026 | -0.026 | -0.026 | -0.002 |
| 3 | 24 | -0.026 | -0.026 | -0.026 | -0.026 | -0.002 |
| 4 | 32 | -0.026 | -0.026 | -0.026 | -0.026 | -0.002 |

### 6.3 Full four-cycle trajectory samples (actual angles, rad)

The rows below are the recorded **1-second samples** from the complete log. `Progress` is the trajectory interpolation coefficient (0 = open target, 1 = closed target), **not a measured grasp quality score**.

| Time (s) | Progress | FFJ3 | MFJ3 | RFJ3 | LFJ3 | THJ4 |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0.000 | -0.000 | -0.000 | -0.000 | -0.000 | -0.000 |
| 1 | 0.258 | 0.225 | 0.225 | 0.225 | 0.225 | 0.158 |
| 2 | 0.740 | 0.764 | 0.764 | 0.761 | 0.762 | 0.571 |
| 3 | 1.000 | 1.035 | 1.082 | 1.077 | 1.076 | 0.762 |
| 4 | 1.000 | 1.058 | 1.093 | 1.093 | 1.086 | 0.773 |
| 5 | 0.741 | 0.834 | 0.835 | 0.810 | 0.925 | 0.631 |
| 6 | 0.259 | 0.294 | 0.294 | 0.295 | 0.294 | 0.216 |
| 7 | 0.000 | -0.015 | -0.015 | -0.015 | -0.015 | -0.002 |
| 8 | 0.000 | -0.026 | -0.026 | -0.026 | -0.026 | -0.002 |
| 9 | 0.259 | 0.226 | 0.226 | 0.226 | 0.226 | 0.159 |
| 10 | 0.740 | 0.764 | 0.764 | 0.761 | 0.762 | 0.572 |
| 11 | 1.000 | 1.034 | 1.082 | 1.082 | 1.070 | 0.762 |
| 12 | 1.000 | 1.058 | 1.093 | 1.097 | 1.082 | 0.773 |
| 13 | 0.742 | 0.835 | 0.836 | 0.817 | 0.916 | 0.632 |
| 14 | 0.260 | 0.295 | 0.295 | 0.295 | 0.295 | 0.217 |
| 15 | 0.000 | -0.015 | -0.015 | -0.015 | -0.015 | -0.002 |
| 16 | 0.000 | -0.026 | -0.026 | -0.026 | -0.026 | -0.002 |
| 17 | 0.258 | 0.225 | 0.225 | 0.225 | 0.225 | 0.158 |
| 18 | 0.740 | 0.764 | 0.764 | 0.761 | 0.762 | 0.572 |
| 19 | 1.000 | 1.034 | 1.082 | 1.084 | 1.069 | 0.762 |
| 20 | 1.000 | 1.058 | 1.093 | 1.099 | 1.080 | 0.774 |
| 21 | 0.741 | 0.834 | 0.835 | 0.814 | 0.917 | 0.632 |
| 22 | 0.259 | 0.294 | 0.294 | 0.295 | 0.294 | 0.216 |
| 23 | 0.000 | -0.015 | -0.015 | -0.015 | -0.015 | -0.002 |
| 24 | 0.000 | -0.026 | -0.026 | -0.026 | -0.026 | -0.002 |
| 25 | 0.259 | 0.226 | 0.226 | 0.226 | 0.226 | 0.159 |
| 26 | 0.741 | 0.765 | 0.765 | 0.763 | 0.763 | 0.573 |
| 27 | 1.000 | 1.034 | 1.082 | 1.078 | 1.075 | 0.763 |
| 28 | 1.000 | 1.058 | 1.093 | 1.093 | 1.086 | 0.774 |
| 29 | 0.741 | 0.834 | 0.834 | 0.812 | 0.949 | 0.632 |
| 30 | 0.259 | 0.294 | 0.294 | 0.294 | 0.294 | 0.216 |
| 31 | 0.000 | -0.015 | -0.015 | -0.015 | -0.015 | -0.002 |
| 32 | 0.000 | -0.026 | -0.026 | -0.026 | -0.026 | -0.002 |

**Interpretation.** The logged samples indicate repeatable gross joint-space motion through four eight-second cycles. In particular, the four `J3` finger joints stabilize near **1.06–1.10 rad** in the closed-hold phase and near **−0.026 rad** in the open-hold phase; the recorded thumb `THJ4` reaches approximately **0.773–0.774 rad** when closed. These measurements establish a **repeatable open/close gesture**, not object grasp success, grip force, contact stability, or tracking accuracy of the other eight commanded actuators.

## 7. Environment and reproduction

**Tested environment:** Windows, Python 3.11, MuJoCo 3.15.0, NumPy 2.4.6; examples launched from PyCharm using a local Python virtual environment. A discrete GPU is not required for these small interactive examples.

### Install

```powershell
py -3.11 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

If PowerShell blocks activation, select `.venv\Scripts\python.exe` as your PyCharm interpreter or use that executable directly.

### Download the Shadow Hand model

```powershell
mkdir third_party
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git third_party/mujoco_menagerie
```

The model assets are used locally and are not vendored into this repository. Consult the upstream repository's licenses/attribution when redistributing recordings or modified models.

### Run demonstrations

From the project root:

```powershell
python demos/01_arm_motion.py
python demos/02_joint_control.py
python demos/03_urdf_loading.py
python demos/04_urdf_to_mjcf.py
python demos/05_urdf_joint_control.py
python demos/06_shadow_hand_loading.py
python demos/07_shadow_hand_control.py
python demos/08_shadow_hand_grasp.py
```

Demo 04 generates or overwrites `models/converted_arm.xml` and adds the three position actuators programmatically. Run it before Demo 05 after changing the URDF. Demos 06–08 require the downloaded `shadow_hand/scene_right.xml` and its asset files.

## 8. Relationship to the robotics study objectives

| Study objective | Demonstrated work | Evidence |
|:---|:---|:---|
| Understand URDF `link` and `joint` | Built a 3-DOF URDF; imported into MuJoCo | Demos 03–04 |
| Understand MJCF `body`, `joint`, `geom`, `actuator` | Separate robot model and control code | Demos 01–02 |
| Load and inspect a robot | Loaded both arm and Shadow Hand models; printed model inventories | Demos 03 and 06 |
| Control joints | Sent actuator targets; compared with actual generalized positions | Demos 02, 05 and 07 |
| Coordinate dexterous-hand movements | Coordinated 13 actuator inputs for an open/close sequence | Demo 08; 4-cycle data |
| Combine robotic arm + Shadow Hand | **Not yet implemented**; future integration milestone | Future work |
| Object grasping and contact validation | **Not yet implemented**; no grasped object in these results | Future work |

## 9. Limitations and next steps

- Integrate a physical robot arm and the Shadow Hand into one kinematic tree.
- Add an object, collision contacts, grip-force measurement and grasp stability tests.
- Measure tracking errors for all actively commanded degrees of freedom.
- Compare MJCF and URDF-derived models under matched gravity, inertial properties and control gains.
- Add repeatable experiment configuration and saved trajectory outputs for future quantitative comparison.

## 10. References and acknowledgement

- [MuJoCo documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- [Shadow Hand model](https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand)
- [URDF model import in MuJoCo](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf)

The Shadow Hand model and associated assets are third-party work from MuJoCo Menagerie. This repository's scripts are educational examples and should not be construed as validated hardware control or stable grasp planning.
