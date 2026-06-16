# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

ROS 1 (Noetic) catkin workspace for an xArm6 robotic arm that uses a RealSense
camera to identify black polyomino ("Tetris") blocks lying on a backlit board,
then picks them with a vacuum suction tool and places them into a fixed 14x10
grid to maximize the number of blocks placed (an exact-cover packing problem).

All active development happens in `src/tly`. Everything else under `src/`
(`xarm_ros`, `realsense-ros`, `vision_opencv`, `easy_handeye`) is a vendored
third-party dependency — `.gitignore` excludes `src/xarm_ros`,
`src/realsense-ros`, `src/easy_handeye`, `OpenCV_Source/`, `build/`, `devel/`
from this repo's history. Don't expect to need to modify them; treat them as
installed packages. `docs/` is stale and out of date — don't rely on it.

## Build / run

```bash
source /opt/ros/noetic/setup.bash
catkin_make            # from /root/catkin_ws — the only build flow in use
source devel/setup.bash
```

`src/tly/CMakeLists.txt` forces `-std=c++14` and `-O3` globally — the `-O3` is
specifically there because `strategy_node`'s DLX search is compute-bound;
don't drop it when touching the build config. There is no lint/test suite for
`tly` itself.

Run the full pipeline against the real arm:

```bash
roslaunch tly tly.launch robot_ip:=192.168.1.228
```

Per-node `test_*.launch` files (bring up only what one node needs, so you don't
have to run the whole `tly.launch` pipeline). All vision-bearing ones take
`vision_node:=vision_processor_node_dexined` to switch the vision impl.
- `test_vision.launch` — **vision only**: camera + `image_proc` + vision node.
  No arm/TF/strategy. Inspect `/vision/board_state`, `/vision/debug_image`,
  `/vision/pick_depth_debug`.
- `test_strategy.launch` — **strategy only**, no hardware. Publish a fake
  `/vision/board_state` to drive it; watch `/tetris_plan`.
- `test_path.launch` — **perception→strategy→path** chain. Needs the arm for TF
  (`robot_ip:=`) but never commands motion. Publishes `/motion_cmds`; compare
  `[PATH][TASK]` `z_plane`/`z_depth` logs.
- `test_controller.launch` — **controller only** + arm + camera. Feed a manual
  `/tetris_plan`; **drives the real arm** (capped to 1 task). Native driver,
  not MoveIt.
- `test_pick.launch` — older MoveIt-based integration test: vision +
  `xarm_controller_node` + `single_block_test.py`, one pick/place cycle.

Other launch files in `src/tly/launch/`:
- `tly.launch` — production run. Native xArm driver only (explicitly *not*
  MoveIt/Pilz). Brings up camera, hand-eye TF, vision, controller, strategy,
  and `path_planner_node` end to end.
- `affine.launch` / `calibrate_tool.launch` / `calibrate_xarm.launch` /
  `xarm_calibration_setup.launch` — calibration utilities, see below.

## Node pipeline (src/tly)

Three custom nodes talk over a fixed topic/service contract:

**1. `vision_processor_node` / `vision_processor_node_cpp`**
(built from `src/vision_processor_node.cpp`)
- Subscribes to the rectified color image (`image_proc` output) and camera
  info; segments dark blocks against the lit board (Otsu/manual threshold +
  lightboard mask), extracts contours, and classifies each against 7
  polyomino templates rotated in steps (`template_angle_step` /
  `template_refine_step`).
- Optionally refines block edges with a DexiNed ONNX model run on CUDA
  (`module/dexined.onnx`, toggled by `use_dexined`) — this is the neural-net
  edge detector that replaced the old classical edge detection.
- Tracks detections across frames (`track_history_len`,
  `stable_min_frames`, `stable_max_px_std`, ...) and only publishes once a
  block is stable.
- Publishes:
  - `/vision/board_state` (`std_msgs/Int32MultiArray`): 7-int shape inventory
    + 140-int board occupancy grid (`BOARD_ROWS` x `BOARD_COLS` = 14x10) +
    `num_blocks` + a flat `[shape, u, v, angle]` tuple per detected block.
    `strategy_node` requires `data.size() >= 147` (7+140) before reading the
    rest.
  - `/vision/tracked_blocks_table` (`geometry_msgs/PoseArray`) and debug
    image topics under `/vision/debug_*` and `/vision/preprocess/*`.
  - Service `/vision/get_precise_pose` (`tly/GetPrecisePose`): given a
    `target_shape_type`, returns a refined `dx`/`dy`/`angle`. Note: no node
    in this repo currently calls this service — it exists for future/manual
    use only.

**2. `strategy_node`** (`src/strategy_node.cpp`)
- Waits for `/vision/board_state` until the inventory holds steady at
  `expected_total_blocks` (default 35) for `inventory_stable_required_frames`
  frames, with a fallback to accept `min_usable_total_blocks` (default 34-35)
  after fewer stable frames.
- Solves placement as an exact-cover problem with a hand-rolled Dancing
  Links (DLX) engine over `BASE_SHAPES` (7 tetromino-like pieces) and their 4
  rotations, searching for the max-score cover with restarts
  (`max_search_nodes` / `max_restarts`).
- Publishes the plan once on `/tetris_plan` (`std_msgs/Int32MultiArray`,
  latched).

**3. `xarm_controller_node`** (`src/xarm_controller_node.cpp`)
- Drives the arm via the xArm's *native* services only —
  `/xarm/set_mode`, `/xarm/set_state`, `/xarm/move_line`, plus a digital-IO
  service for the suction cup (`suction_io_num`) — never MoveIt.
- Consumes `/tetris_plan` once, runs a per-task state machine (`IDLE` →
  `TAKE_NEXT_TASK` → `MOVE_TO_PICK_HOVER` → `EXECUTE_PICK` →
  `MOVE_TO_PLACE_HOVER` → `EXECUTE_PLACE` → `FINISH`), bounded by
  `max_tasks_per_plan`.
- Converts pixel detections to robot-base coordinates using the calibration
  data in `tetris_config.yaml` (pick-surface plane, board map, pick
  homography) combined with the easy_handeye eye-on-hand TF and the
  `table_frame` published by `table_tf_broadcaster.py`.
- Publishes `/robot_status` (`std_msgs/Bool`, latched) as a busy/idle flag.

Supporting piece: `scripts/table_tf_broadcaster.py` publishes `table_frame`
from the `table_tf` x/y/z/roll/pitch/yaw values in `tetris_config.yaml`.

## Calibration data and shape contract

`config/tetris_config.yaml` (everything under the `tetris:` key) holds all
calibration state: hand-eye frames, `table_tf`, the table/pick plane fits
(`TABLE_SURFACE_PLANE_BASE` / `PICK_SURFACE_PLANE_BASE`, point+normal+RMS
residuals), the 14x10 board grid samples/origin/step
(`BOARD_SAMPLES_TABLE`, `BOARD_ORIGIN_TABLE`, ...), the pick homography
(`PICK_HOMOGRAPHY`), and a board "bump height" model
(`BOARD_BUMP_HEIGHT_MODEL`, `USE_BUMP_HEIGHT_FOR_PLACE`) that corrects 2.5D
height parallax when blocks sit at slightly different heights. This file is
generated/overwritten by the calibration tools below — treat it as data, not
hand-edited config. The `.bak_before_*` siblings are point-in-time backups
from past calibration runs, kept for reference/rollback.

Shape IDs are a contract shared across `strategy_node.cpp`,
`vision_processor_node.cpp`/`.py`, and `single_block_test.py` — see
`BASE_SHAPES`: `0` line, `1` square, `2` T, `3` L_left, `4` L_right, `5`
Z_left, `6` Z_right. Changing this enum requires updating all of them in
lockstep.

Calibration workflow:
- `xarm_calibration_setup.launch` + `calibrate_xarm.launch` — ArUco-marker
  eye-on-hand calibration via `easy_handeye`, producing the
  `xarm6_realsense_calibration_eye_on_hand` data consumed by
  `easy_handeye/publish.launch` in `tly.launch`.
- `calibrate_tool.launch` → `scripts/calibration_tool.py` — interactive
  capture of the board grid / table plane fit (`BOARD_*`,
  `TABLE_SURFACE_PLANE_BASE`).
- `affine.launch` → `scripts/pick_affine_calibration_tool.py` (with the
  Python `vision_processor_node.py` variant) — calibrates the pick-side
  homography/affine correction.
- `scripts/hsv_tuner.py`, `scripts/test_hsv.py`, `scripts/test_aruco.py` —
  standalone manual debug utilities, not wired into any launch file.

## Things to watch for

- Only `strategy_node`, `xarm_controller_node`, and `vision_processor_node_cpp`
  (built from `src/vision_processor_node.cpp`) are compiled per
  `CMakeLists.txt`. `src/vision_processor_node_cuda.cpp` and
  `src/xarm_controller_node_pliz.cpp` are **not** part of the build — older
  variants kept for reference. Editing them has no runtime effect; check
  `CMakeLists.txt` before assuming a `.cpp` file is live.
- `scripts/vision_processor_node.py` is a Python reimplementation of the same
  node/topic/service contract (kept shape-ID-compatible with
  `strategy_node.cpp`, see its module docstring), used only by
  `affine.launch`. Production (`tly.launch`) uses the C++ node.
