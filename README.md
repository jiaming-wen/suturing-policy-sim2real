# SurgicAI Suturing Policy Sim2Real

[中文版](README.zh-CN.md)

This repository packages the Approach stage of SurgicAI as a single entry point that can be inspected, takes swappable inputs, and localizes failures:

```text
AMBF scene
  └─ ECM RGB ──> Depth Anything (DA) ──> metric depth
       mask + RGB + depth ──> FoundationPose (FP) ──> first-frame physical gate
       └─ AMBF GT pose (comparison only)            │
                                                    ▼
                                           frozen needle goal
                                                    │
                        PSM measured pose ──────────┤
                                                    ▼
                                        D2 controller / RL policy
                                                    │
                                                    ▼
                                                  Reach
```

The release is validated up to **Reach**. It does not claim that automatic masks, hand-eye accuracy, jaw closing or needle lifting on a real dVRK are done. The full scope is in [`docs/KNOWN_LIMITATIONS.md`](docs/KNOWN_LIMITATIONS.md).

Candidate selection for the real robot is split into comparable interfaces: the needle source can be `fp_only / flat / support / flat_planar / support_planar`, and the PSM source `disabled / vision_only / fk_only / fused`. Every missing calibration fails closed. Modes, topics, JSON schema and the values still to be measured are in [`docs/REAL_PERCEPTION_MODE_SWITCHES.md`](docs/REAL_PERCEPTION_MODE_SWITCHES.md).

## 1. Layout

### Four external dependencies (not in this repository)

| Name | Role | Suggested location |
|---|---|---|
| AMBF | Physics simulation and ROS scene | `$HOME/surgicai_external/ambf` |
| SRC | Surgical Robotics Challenge ADF, world and needle mesh | `$HOME/surgicai_external/surgical_robotics_challenge` |
| DA | Metric depth from RGB | `$HOME/surgicai_external/Depth-Anything-V2` |
| FP | Needle 6D pose from RGB + depth + mask + CAD | `$HOME/surgicai_external/FoundationPose` |

You also need the AMBF ROS 2 bridge (suggested `$HOME/ambf_ros_ws`) and the external DA checkpoint (suggested `$HOME/surgicai_models/p5a_vitl_fp32_best.pth`).

### Main modules

```text
configs/pipeline.env.example   all local paths in one place
scripts/doctor.py              checks dependencies only; starts no simulator or robot
scripts/prepare_ambf_launch.py writes the local SRC path into the AMBF launch
scripts/run_simulation.py      the single simulation entry point: code / S3 / S4 / full
scripts/inspect_results.py     checks whether a run is actually complete
src/perception/                RGB, DA, FP, first-frame physical gate
src/control/                   frozen goal, D2 controller, Reach evaluation
src/SurgicAI/RL/               Approach environment and TD3-HER-BC
src/runners/                   S3/S4 runners called by the entry point
ros2_ws/src/suturing_runtime/  real-robot ROS topic adapters and a guarded Reach gateway
models/rl/                     released M3/R6 checkpoints
data/reference/                frozen S3 bank, used when DA/FP are not rerun
```

Follow the order below. Do not run `src/runners/*.sh` directly, and do not copy commands from historical experiment reports.

### Check real images stage by stage before running the main program

Once you have a real first ECM frame, use `scripts/run_real_perception_stage.py` to separate the possible failure points:

```text
inspect (image readable, exposure)
  -> da (checkpoint identity + single-image depth)
  -> mask (sensitivity of the manual polyline to its width)
  -> bundle (RGB/depth/mask/K/shape consistency)
  -> run_fp_stage_docker.sh (standalone FoundationPose register)
  -> fp-compare (does changing K or the mask make the pose jump?)
```

All of these are read-only offline checks; they start no AMBF and send no ROS motion commands. Run `python scripts/run_real_perception_stage.py --help` for each stage's arguments. A clean exit only means the API and file contracts hold; without a real depth and 6D pose reference it does not show that DA or FP is accurate.

## 2. Set the AMBF, SRC, DA and FP paths

Recommended: Ubuntu 22.04 (native or WSL2), ROS 2 Humble, Python 3.10, with everything under the Linux home directory to avoid spaces in paths.

```bash
cd "$HOME"
git clone https://github.com/jiaming-wen/suturing-policy-sim2real.git
cd suturing-policy-sim2real

cp configs/pipeline.env.example configs/pipeline.env
nano configs/pipeline.env
```

With the suggested directories, these five need no change; otherwise edit only the right-hand side:

```bash
AMBF_ROOT=${HOME}/surgicai_external/ambf
SRC_ROOT=${HOME}/surgicai_external/surgical_robotics_challenge
DA_ROOT=${HOME}/surgicai_external/Depth-Anything-V2
FOUNDATIONPOSE_ROOT=${HOME}/surgicai_external/FoundationPose
DA_CHECKPOINT=${HOME}/surgicai_models/p5a_vitl_fp32_best.pth
```

Then check the ROS and output paths:

```bash
SIM_S3_ROS_SETUP=/opt/ros/humble/setup.bash
SIM_S3_AMBF_ROS_SETUP=${HOME}/ambf_ros_ws/install/setup.bash
SURGICAI_RESULT_ROOT=${HOME}/surgicai_runs
ROS_DOMAIN_ID=220
```

`ROS_DOMAIN_ID` must be an unused value in `1..232`. Leftover AMBF/ROS nodes on the same domain contaminate results.

Load the configuration into the current shell:

```bash
set -a
source configs/pipeline.env
set +a
```

Check the paths now rather than after hours of running:

```bash
test -x "$AMBF_ROOT/core/build/bin/ambf_simulator" && echo "AMBF OK"
test -f "$SRC_ROOT/ADF/Phantoms/3D_MED/high_res/Needle_stage_d_v0.OBJ" && echo "SRC OK"
test -d "$DA_ROOT" && echo "DA repo OK"
test -d "$FOUNDATIONPOSE_ROOT" && echo "FP repo OK"
test -f "$DA_CHECKPOINT" && echo "DA checkpoint OK"
```

If the external sources are missing, create the suggested directories and clone them; AMBF, DA and FP still have to be built or containerized following their own READMEs:

```bash
mkdir -p "$HOME/surgicai_external" "$HOME/surgicai_models" "$HOME/surgicai_runs"
git clone https://github.com/WPI-AIM/ambf.git "$HOME/surgicai_external/ambf"
git clone https://github.com/surgical-robotics-ai/surgical_robotics_challenge.git \
  "$HOME/surgicai_external/surgical_robotics_challenge"
git clone https://github.com/DepthAnything/Depth-Anything-V2.git \
  "$HOME/surgicai_external/Depth-Anything-V2"
git clone https://github.com/NVlabs/FoundationPose.git \
  "$HOME/surgicai_external/FoundationPose"
git -C "$HOME/surgicai_external/FoundationPose" checkout \
  a1b694b83e633c2cb6115b9063d940a687759392
```

The DA checkpoint is about 3.8 GB and is not in Git. Get it from a project member and verify it:

```bash
sha256sum "$DA_CHECKPOINT"
# expected: fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88
```

The full FP path also needs the local GPU container tag:

```bash
docker image inspect foundationpose:blackwell >/dev/null && echo "FP image OK"
```

The pinned models, commits and image digests are in [`docs/MODEL_ASSETS.md`](docs/MODEL_ASSETS.md). If the FP image does not exist, build it following FoundationPose's Docker guide and set `SIM_S3_FP_IMAGE` to your tag; never substitute an empty container or another model silently.

## 3. Create the Python environment and the AMBF launch

```bash
cd "$HOME/suturing-policy-sim2real"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements/analysis.txt
```

Full AMBF/RL runs also need a PyTorch build that matches your CUDA:

```bash
# Install torch for your CUDA with the official PyTorch command first, then:
python -m pip install -r requirements/simulation.txt
```

ROS Python packages, `PyKDL` and `ambf_msgs` must come from the ROS/AMBF workspace, not from same-named PyPI packages:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/ambf_ros_ws/install/setup.bash"
```

Write the real SRC path into the released AMBF template:

```bash
python scripts/prepare_ambf_launch.py \
  --src-root "$SRC_ROOT" \
  --output-dir "$HOME/.cache/surgicai/launch"
```

With the suggested output directory, the defaults `SIM_S3_STEREO_LAUNCH` / `SIM_S4_STEREO_LAUNCH` in `configs/pipeline.env` already point to the generated files; otherwise copy the printed paths into the config.

## 4. Run the main code layer by layer

### 4.1 L0: code only

Starts no AMBF, DA, FP or robot:

```bash
python scripts/run_simulation.py --stage code --profile smoke
```

Expected ending:

```text
PIPELINE_COMPLETE stage=code profile=smoke ...
```

This checks the Python logic, the tests and the synthetic hand-eye software contract. It does not check GPU perception or real-robot accuracy.

### 4.2 L1: frozen perception bank → AMBF Reach

Restarts AMBF and the controller but reuses the frozen S3 bank in the repository instead of recomputing DA/FP:

```bash
set -a; source configs/pipeline.env; set +a
source /opt/ros/humble/setup.bash
source "$HOME/ambf_ros_ws/install/setup.bash"

python scripts/doctor.py \
  --profile reach \
  --config configs/pipeline.env \
  --check-domain-clean

python scripts/run_simulation.py \
  --stage s4 \
  --profile smoke \
  --goal both \
  --controller d2
```

`smoke` runs only 2 episodes to check the wiring. Once it passes, change `--profile smoke` to `--profile formal`.

### 4.3 L2: RGB → DA → FP gate → Reach

This is the full simulation pipeline. Check every dependency first:

```bash
python scripts/doctor.py \
  --profile full \
  --config configs/pipeline.env \
  --check-domain-clean
```

Run only after you see `DOCTOR_PASS`:

```bash
python scripts/run_simulation.py \
  --stage full \
  --profile smoke \
  --depth da \
  --goal both \
  --controller d2
```

After the smoke test, run the fixed formal protocol:

```bash
python scripts/run_simulation.py \
  --stage full \
  --profile formal \
  --depth da \
  --goal both \
  --controller d2
```

The formal sequence is: 40 reset/captures → DA → FP candidates → first-frame physical gate → frozen goal bank → AMBF restart → 30 Reach episodes each for the GT and FP goals.

## 5. Swapping GT, DA, FP and the controller

Do not edit the internal Python files; change only the switches on the main command:

| Argument | Choices | Meaning |
|---|---|---|
| `--depth` | `gt` / `da` | AMBF privileged depth / DA predicted depth |
| `--goal` | `gt` / `fp` / `both` | GT needle pose / gated FP needle pose / paired A/B on the same reset |
| `--controller` | `d2` / `rl` | staged SE(3) servo / released RL checkpoint |
| `--profile` | `smoke` / `formal` | 2-episode wiring test / fixed 40+30 formal protocol |

Common isolation experiments:

```bash
# FP only: GT depth + FP goal
python scripts/run_simulation.py --stage full --profile smoke \
  --depth gt --goal fp --controller d2

# Bypass FP: DA still runs, but control uses the GT goal
python scripts/run_simulation.py --stage full --profile smoke \
  --depth da --goal gt --controller d2

# Compare D2 and RL on the same frozen bank (separate runs)
python scripts/run_simulation.py --stage s4 --profile smoke \
  --goal fp --controller d2
python scripts/run_simulation.py --stage s4 --profile smoke \
  --goal fp --controller rl
```

Note: `--goal gt` bypasses FP and does not show that DA/FP work; `--stage s4` reuses the bank and does not show that perception was recomputed. Only `--stage full --depth da --goal fp|both` runs the full perception path.

## 6. Judging success and localizing failures

Every run prints its result directory at the end, for example:

```text
PIPELINE_COMPLETE ... root=/home/<user>/surgicai_runs/pipeline_smoke_<timestamp>
```

Pass that path to:

```bash
python scripts/inspect_results.py \
  "$HOME/surgicai_runs/pipeline_smoke_<timestamp>"
```

A pass prints `RESULT_AUDIT_PASS`. Do not rely on the AMBF window or the process exit code alone.

The entry point returns stable error codes by failing layer:

| Error | Failing layer | Look first at |
|---|---|---|
| `D9-E03-CONFIG` | config file / paths | `configs/pipeline.env` |
| `D9-E10-PREFLIGHT` | ROS, AMBF, models, domain | `doctor.json` in the result directory |
| `D9-E60-S3` | capture, DA, FP or physical gate | `sim_s3/*.log`, `fp_gate/result.json` |
| `D9-E90-S4` | goal bank, AMBF or control | `sim_s4/*/status.txt`, `result.json` |

More troubleshooting (in Chinese): [`docs/zh/参数与排错.md`](docs/zh/参数与排错.md). Keep the raw result directory of every failure; never delete failed episodes and recompute a success rate.

## 7. How the real-robot entry differs from simulation

The real robot is not just the AMBF pipeline with renamed topics. The ROS 2 runtime is in [`ros2_ws/src/suturing_runtime`](ros2_ws/src/suturing_runtime/README.md) and consumes:

```text
/suturing/ecm/left/image
/suturing/depth/metric
/suturing/needle/mask
/suturing/needle/pose_gated
/suturing/psm1/measured_cp
```

It outputs diagnostics, a TF goal and a single guarded Reach, and it is read-only by default. R2 includes the DA backend accepted in P5a, off by default; it needs the frozen checkpoint and the Depth-Anything-V2 path. A manual polygon/PNG first-frame mask publisher is included; automatic segmentation and the FoundationPose backend remain external inputs. A needle pose is published to `/suturing/needle/pose_gated` only after the R6 flat-placement gate and operator confirmation. This is deliberate failure isolation, not a finished automatic real-robot pipeline.

Build and read-only smoke test:

```bash
source /opt/ros/humble/setup.bash
mkdir -p "$HOME/suturing_ros_ws/src"
ln -s "$HOME/suturing-policy-sim2real/ros2_ws/src/suturing_runtime" \
  "$HOME/suturing_ros_ws/src/suturing_runtime"
cd "$HOME/suturing_ros_ws"
colcon build --symlink-install --packages-select suturing_runtime
source install/setup.bash

ros2 launch suturing_runtime real_read_only.launch.py
```

Verify topic types, camera intrinsics, hand-eye, frames and units, the mask, freshness, low speed and the operator stop before any guarded motion. Input-by-input differences between the real robot and simulation (in Chinese): [`docs/zh/真机与仿真的区别.md`](docs/zh/真机与仿真的区别.md); a file-by-file walkthrough of the real-robot code (in Chinese): [`docs/zh/真机主代码逐段讲义.md`](docs/zh/真机主代码逐段讲义.md).

## 8. What was verified for this release

- Pure-logic unit tests on Windows;
- the L0 entry point from a clean GitHub clone;
- `colcon build` of `suturing_runtime` on WSL Ubuntu 22.04 / ROS 2 Humble;
- mock RGB/depth/mask/needle pose/PSM pose → TF goal → `READY_READ_ONLY`;
- in read-only mode, no raw dVRK servo publisher exists and unauthorized motion is rejected.

The release refactor was not connected to a real dVRK, and the 40+30 L2 formal matrix was not rerun on a fresh GPU host. The historical 40/40 gate and 29/30 Reach in this repository are reference evidence, not a claim that any new machine reproduces them run for run. Evidence files are in [`docs/evidence/`](docs/evidence/).

## License

Code produced by this project is MIT-licensed. AMBF, SRC, FoundationPose, Depth Anything, the meshes and external checkpoints follow their own licenses.
