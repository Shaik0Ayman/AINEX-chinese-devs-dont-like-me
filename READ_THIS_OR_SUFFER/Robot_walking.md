# How the Robot Walks (Control Flow)

## Node: `ainex_controller.py`

-   Initializes servos to an initial pose, loads gait parameters from
    YAML, and creates a `WalkingModule` that generates per-joint
    trajectories.
-   Uses `LegIK` to turn body/foot trajectories into joint angles.
-   **Real robot:** sends pulses via `MotionManager` to servos.\
-   **Gazebo:** publishes `Float64` to joint controllers.
-   Runs a loop at `servo_control_cycle`, calling `WalkingModule.run()`
    to get next joint goals; times stepping by measured motion and a
    speed model.

## Gait Parameters (The "Dial Knobs")

From YAML: - **Real:** `walking_param.yaml` - **Simulation:**
`walking_param_sim.yaml`

Core fields: - `init_*_offset`: neutral body pose offsets (m or deg)
before stepping\
- `period_time` (ms): duration of a gait cycle\
- `dsp_ratio`: percent of cycle with both feet on ground\
- `step_fb_ratio`: forward/back spacing between feet\
- `x/y/z_move_amplitude`: step length (m) fore-aft/left-right and foot
lift height\
- `angle_move_amplitude`: turning (deg)\
- `y_swap_amplitude`, `z_swap_amplitude`: body sway (lateral/vertical)
for balance\
- `pelvis_offset` (deg), `hip_pitch_offset` (deg), `arm_swing_gain`\
- `period_times`: if \>0, stop after N cycles

Bounds and safety clamping are enforced in
`ainex_controller.set_walking_param_callback`.

## Inputs/Outputs (ROS API)

-   **Publish parameters:** `/walking/set_param`
    (`ainex_interfaces/WalkingParam`)\
-   **Start/stop:** `/walking/command`
    (`ainex_interfaces/SetWalkingCommand`) with commands:
    `{start, stop, enable, disable, enable_control, disable_control}`\
-   **Init pose:** `/walking/init_pose` (`std_srvs/Empty`)\
-   **Read current parameters:** `/walking/get_param`
    (`ainex_interfaces/GetWalkingParam`)\
-   **Walking state:** topic `walking/is_walking` (`std_msgs/Bool`) and
    service `/walking/is_walking`

## Offsets by Speed Setting

When commanded via `AppWalkingParam` (`app/set_walking_param`), speed
1--4 branches tweak offsets using `walking_offset.yaml` (e.g.,
`high_speed_forward_offset`) to stabilize walking at different speeds.

## Bringup Sequence (Real vs Sim)

### Real Robot:

``` bash
roslaunch ainex_kinematics ainex_controller.launch
```

Optionally calibrate and set init pose using `/walking/init_pose`, then
start walking.

### Gazebo:

``` bash
roslaunch ainex_kinematics ainex_controller.launch gazebo_sim:=true
```

Joint commands are published to controllers; no `MotionManager`.

> **Tip:** `start.launch` and `bringup.launch` can wrap multiple nodes
> for a higher-level start.

------------------------------------------------------------------------

## Minimal Python: Make it Walk

``` python
#!/usr/bin/env python3
import time
import rospy
from ainex_interfaces.msg import WalkingParam
from ainex_interfaces.srv import SetWalkingCommand

def make_params(x=0.012, y=0.0, angle_deg=0.0):
    p = WalkingParam()
    # Base posture/gait timings matching config defaults
    p.init_x_offset = 0.0
    p.init_y_offset = -0.005
    p.init_z_offset = 0.025
    p.init_roll_offset = 0.0
    p.init_pitch_offset = 0.0
    p.init_yaw_offset = 0.0
    p.hip_pitch_offset = 15.0
    p.period_time = 400.0          # ms per cycle
    p.dsp_ratio = 0.2
    p.step_fb_ratio = 0.028
    # Motion amplitudes (meters and degrees)
    p.x_move_amplitude = float(x)   # + forward, - backward
    p.y_move_amplitude = float(y)   # + left, - right
    p.z_move_amplitude = 0.02       # foot lift height
    p.angle_move_amplitude = float(angle_deg)  # + CCW, - CW
    # Balance/feel
    p.y_swap_amplitude = 0.02
    p.z_swap_amplitude = 0.006
    p.pelvis_offset = 5.0
    p.move_aim_on = False
    p.arm_swing_gain = 0.5
    p.period_times = 0              # 0 = continuous
    return p

if __name__ == "__main__":
    rospy.init_node("walk_example")
    param_pub = rospy.Publisher("/walking/set_param", WalkingParam, queue_size=1, latch=True)
    start_stop = rospy.ServiceProxy("/walking/command", SetWalkingCommand)
    rospy.wait_for_service("/walking/command")

    # Set gait: forward; try y=0.01 for strafe, or angle_deg=8 for turn-in-place
    param_pub.publish(make_params(x=0.012, y=0.0, angle_deg=0.0))
    # Start
    start_stop("start")
    time.sleep(5.0)
    # Stop
    start_stop("stop")
```

This example publishes a forward-walk gait, starts walking for 5
seconds, then stops. Adjust `x`, `y`, `angle_deg` to strafe or turn.

### Variants

-   **Strafe left:**

``` python
make_params(x=0.0, y=0.013, angle_deg=0.0)
```

-   **Turn in place:**

``` python
make_params(x=0.0, y=0.0, angle_deg=8.0)
```

-   **Walk backward:**

``` python
make_params(x=-0.012, y=0.0, angle_deg=0.0)
```
