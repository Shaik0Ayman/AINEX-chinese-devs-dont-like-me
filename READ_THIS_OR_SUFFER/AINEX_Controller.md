# ainex_controller.py – Central Walking Controller

## Overview
This file is the **central walking controller** for the robot. It:
- Initializes servos (or Gazebo controllers) to an initial pose.
- Loads and updates gait parameters.
- Uses inverse kinematics (LegIK) and `WalkingModule` to generate joint trajectories.
- Sends joint goals to hardware (MotionManager) or Gazebo topics.
- Exposes ROS services and topics to start/stop walking and tune gait.

---

## Key Components and Data

### Joint Mapping
- **joint_index**: logical joint ordering used by the gait generator.
- **joint_id**: servo ID map for hardware.
- **joint_name**: reverse lookup from ID to name.

### Unit Ranges and Safety
- Body height (z): `0.015–0.06 m`
- Step amplitudes x/y: `±0.05 m`
- Step height: `0–0.05 m`
- Turn angle: `±10°`
- Arm swing gain: `0–60°` (internally clamped in radians)

Clamping is enforced in `set_walking_param_callback`.

---

## Configuration Files Used
- **Initial pose & servo limits**: `config/init_pose.yaml`, `config/servo_controller.yaml`
- **Gait parameters**:
  - Real: `walking_param.yaml`
  - Simulation: `walking_param_sim.yaml`
- **Speed offsets**: `walking_offset.yaml` (used by APP interface)

---

## Conversions (Hardware Only)
- Per-servo pulse ↔ radian linear mapping based on `servo_controller.yaml`.
- Functions: `angle2pulse`, `pulse2angle` using `ENCODER_TICKS_PER_RADIAN`.

---

## Initialization Flow

### Node Init & Simulation Flag
- Parameter: `~gazebo_sim`
- **Real hardware**: sets `init_pose_finish=True` after physical initialization.
- **Simulation**: waits for `/joint_states` to set `init_pose_finish=True`.

### Servo Encoder Mapping
- Builds conversion coefficients per servo from controller configs.

### Initial Pose
- Loads from `~init_pose`.
- Sends blocking pose set via MotionManager (real).
- In simulation: sets publishers and waits for joint states.

### Gait Setup
- Loads `walking_param[_sim].yaml`.
- Initializes `WalkingModule` with:
  - IK reference
  - Parameters
  - Joint index
  - Initial position
  - Trajectory step size

---

## ROS Wiring

### Publishers
- **Hardware**: none (uses MotionManager internally)
- **Simulation**: `/joint_controller/command` (Float64) per joint
- `walking/is_walking` (Bool)

### Subscribers
- **Simulation**: `/joint_states`
- **Real**: head pan/tilt (`HeadState`) for servo relay

### Services
- `/walking/command` – start/stop/enable/disable walking
- `/walking/init_pose` – re-home to initial pose
- `/walking/get_param` – returns current gait parameters
- `/walking/is_walking`
- `/walking/get_offset`, `/walking/set_offset`, `/walking/save_offset`

### Topics
- `/walking/set_param` (WalkingParam)
- `/app/set_walking_param` (AppWalkingParam)
- `/app/set_action` (String – play action file)

---

## Main Loop (`run`)
- Executes periodically if `init_pose_finish` and `walking_enable` are true.
- Steps:
  1. Calls `WalkingModule.run()` → returns joint movement, timestamp, and joint goals.
  2. Converts rad → pulse (real) or publishes Float64 (sim).
  3. Controls speed:
     - Speed model: `0.2 s / 60° → 0.2 / radians(60)` seconds/radian
     - Compares with `servo_control_cycle`
  4. Detects stop conditions:
     - Low movement threshold
     - `WalkingModule.walk_finish()`
     - Publishes `walking/is_walking=False`

- **Fixed step count**: stops after `period_times > 0` cycles.

---

## Commands and Behaviors

### walking/command
- `start`: Enables walking and starts gait loop.
- `stop`: Stops walking and waits for standstill.
- `enable`/`disable`: High-level gate; disable also stops.
- `enable_control`/`disable_control`: Toggles control readiness.

### walking/init_pose
- Stops walking, disables control, moves to init pose, and re-enables.

### move_to_init_pose (hardware)
- Reads current servo pulses, clamps anomalies.
- Drives arms/head quickly to initial position.
- Seeds present_joint_state and rebuilds WalkingModule.

### set_action_callback
- Stops walking, plays predefined action, then returns to init pose.

---

## Setting and Getting Gait Parameters

### `/walking/set_param` (WalkingParam)
- Fields: init_x/y/z_offset, x/y/z_move_amplitude, angle_move_amplitude, pelvis_offset, hip_pitch_offset (degrees), arm_swing_gain (radians), period_time (ms), dsp_ratio.
- Values are clamped to safe ranges.

### `/app/set_walking_param` (AppWalkingParam)
- User-friendly: speed 1–4 presets.
- Applies offsets from `walking_offset.yaml`.

### `/walking/get_param`
- Returns current gait parameters from WalkingModule.

---

## Simulation vs Real Hardware
### Simulation
- Publishes Float64 commands.
- Uses `walking_param_sim.yaml`.
- No MotionManager.

### Real Hardware
- Batch servo control via MotionManager.
- Uses encoder mapping.
- Enforces servo speed limits.

---

## ROS API Quick Reference

### Topics
- **In**: `/walking/set_param`, `/app/set_walking_param`, `/app/set_action`
- **Out**: `walking/is_walking`
- **Sim only Out**: `/joint_controller/command`

### Services
- `/walking/command`
- `/walking/init_pose`
- `/walking/get_param`
- `/walking/is_walking`
- `/walking/get_offset`, `/walking/set_offset`, `/walking/save_offset`

---

## Changing the Walking Behavior
- Modify:
  - `x_move_amplitude` – forward/backward steps
  - `y_move_amplitude` – lateral movement
  - `angle_move_amplitude` – turning angle
- Tune:
  - `period_time` – gait cycle speed
  - `dsp_ratio` – double support phase
  - `y_swap_amplitude`, `z_swap_amplitude` – body sway

Use `/walking/set_param` for precise control or `/app/set_walking_param` for presets.

---

