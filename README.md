# LEAP Hand MuJoCo Demos

Two MuJoCo simulation demos built on the [LEAP Hand](https://leap-hand.github.io/) model: one that drives a single finger to a target pose while measuring joint torque, and one that grasps a cube and verifies the grip with a pull-out test.

| Demo | Script | Recording |
| --- | --- | --- |
| Finger joint control and torque measurement | `control_finger.py` | [watch](video/torch1.mp4) |
| Cube grasping with slip and pull-out test | `grasp_cube.py` | [watch](video/grasp1.mp4) |

The recordings live in the `video` folder and are linked by relative path, so they play directly from this repository.

## Requirements

- Python 3.10+ (tested on 3.12)
- [MuJoCo](https://mujoco.org/) Python bindings (tested with 3.15.0)
- NumPy (tested with 2.5.4)
- A display is only needed when running with the interactive viewer

```bash
pip install mujoco numpy
```

## Demo 1: Finger Joint Control and Torque Measurement

`control_finger.py` drives the four joints of the index finger to a target pose and records how much actuator torque each joint needs to get there and stay there.

- Joints controlled: `if_mcp`, `if_rot`, `if_pip`, `if_dip`
- Default target angles: `[0.4, 0.0, 0.4, 0.4]` rad
- The script checks each target against the actuator control range before running
- Torque is read from `data.qfrc_actuator`, the generalized force applied by the actuator at that joint, in N·m
- At the end it reports the peak torque and the steady-state torque (average over the final 0.5 s) for every joint

Run it with the interactive viewer:

```bash
python control_finger.py
```

Close the viewer window to finish the run and write the CSV. For a headless data-only run:

```bash
python control_finger.py --no-viewer --duration 2.0
```

Optional arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `--no-viewer` | off | Run without the 3D window |
| `--duration` | `3.0` | Simulated seconds for `--no-viewer` mode |
| `--csv` | `control_finger_torque.csv` | Output file |

Output: `control_finger_torque.csv`

| Column | Meaning |
| --- | --- |
| `time_s` | Simulation time |
| `joint` | Joint name |
| `target_rad` | Target angle |
| `actual_rad` | Measured joint angle |
| `error_rad` | `target_rad - actual_rad` |
| `torque_Nm` | Actuator torque at the joint |

This demo is useful for actuator sizing and for answering a practical question: how much torque does it take to hold the finger at a given pose.

## Demo 2: Cube Grasping with Slip and Pull-out Test

`grasp_cube.py` runs a full pick-and-place style sequence on a 5 cm, 50 g cube and evaluates whether the grasp actually holds.

Sequence:

1. `settle_open` - the hand starts open and the cube settles into the palm
2. `close` - the fingers close smoothly with a smoothstep profile to avoid knocking the cube away
3. `hold` - the grasp is held while contact metrics are measured
4. `lift` - the palm rises 8 cm in +z
5. `move_x`, `move_y` - the palm translates 8 cm in each horizontal direction, at 2 cm/s
6. `pull_out` - an external force is ramped on the cube in +z to test the grip

The palm is mounted on three prismatic slides (`palm_slide_x_act`, `palm_slide_y_act`, `palm_slide_z_act`), so it moves like a small gantry in x, y and z.

Why a pull-out test instead of "did the cube fall":

With the palm facing up, the cube simply rests on the palm surface and the normal force alone can carry it upward. Lifting the cube without dropping it therefore does not prove a real grasp. This demo instead pulls the cube away from the palm and reports the force at which it slips.

A run is counted as a successful grasp when all of the following hold:

- The palm contact normal points at least 0.9 upward
- The pull-out force reaches at least 2x the cube's weight (cube weight is about 0.49 N)
- At least 2 contacts are detected during the grip phases
- The cube does not slip more than 4 cm relative to its reference position in any move phase

Run it headless:

```bash
python grasp_cube.py
```

Run it with the interactive viewer:

```bash
python grasp_cube.py --viewer
```

Optional arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `--viewer` | off | Show the 3D window while running |
| `--move` | `0.08` | Distance moved in each direction, in meters |
| `--close` | `1.6` | Closing amount, applied as `BASE_POSE * value`; lower is looser |
| `--csv` | `grasp_records.csv` | Output file |

Output: `grasp_records.csv`

| Column | Meaning |
| --- | --- |
| `time_s` | Simulation time |
| `phase` | Current phase of the sequence |
| `cube_x_m`, `cube_y_m`, `cube_z_m` | Cube position |
| `palm_x_m`, `palm_y_m`, `palm_z_m` | Palm position |
| `contact_count` | Number of contacts on the cube |
| `normal_force_sum_N` | Sum of contact normal forces |
| `resultant_force_N` | Magnitude of the resultant contact force |

At the end of the run the script prints the cube's position after each move, the offset relative to the palm, whether the cube slipped, and the measured pull-out force as a multiple of the cube's weight.

## Video Demos

- [video/grasp1.mp4](video/grasp1.mp4) - cube grasp: open, close, lift, translate and pull-out test
- [video/torch1.mp4](video/torch1.mp4) - index finger driven to the target pose, with torque recorded during the motion

## Files

```
control_finger.py                 finger control and torque recording
control_finger_torque.csv         torque log written by control_finger.py
grasp_cube.py                     cube grasp and pull-out test
grasp_records.csv                 grasp log written by grasp_cube.py
leap_hand/
  right_hand.xml                  LEAP Hand right hand model
  left_hand.xml                   LEAP Hand left hand model
  scene_right.xml                 bare right hand scene
  scene_grasp.xml                 right hand plus a 5 cm cube
video/
  grasp1.mp4                      grasp demo recording
  torch1.mp4                      finger torque demo recording
```

## Model

The hand model comes from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) LEAP Hand entry, which is based on the [LEAP Hand](https://leap-hand.github.io/) platform. The grasp scene adds a free-floating 5 cm cube with a mass of 0.05 kg and the three palm prismatic actuators used by `grasp_cube.py`.

## License

The LEAP Hand model and related assets are distributed under the license included with the model in `leap_hand/LICENSE`. Refer to that file for the terms that apply to the model files.
