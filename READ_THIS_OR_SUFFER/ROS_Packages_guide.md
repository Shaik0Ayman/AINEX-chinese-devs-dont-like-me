# Core AINEX Packages

## ainex_bringup

**Purpose:** High-level bringup of the robot stack.\
**Init:**

``` bash
roslaunch ainex_bringup bringup.launch
# or
roslaunch ainex_bringup base.launch
```

## ainex_peripherals

**Purpose:** Sensor and joystick nodes.\
**Init:** - **Camera:**

``` bash
roslaunch ainex_peripherals usb_cam.launch
# or
roslaunch ainex_peripherals usb_cam_with_calib.launch
```

-   **IMU:**

``` bash
roslaunch ainex_peripherals imu.launch
```

-   **Joystick:**

``` bash
roslaunch ainex_peripherals joystick_control.launch
```

-   **Image calibration:**

``` bash
roslaunch ainex_peripherals image_calib.launch
```

## ainex_calibration

**Purpose:** Run device calibrations.\
**Init:** - **Camera:**

``` bash
roslaunch ainex_calibration camera_calibration.launch
```

-   **IMU:**

``` bash
roslaunch ainex_calibration imu_calibration.launch
```

-   **Magnetometer:**

``` bash
roslaunch ainex_calibration mag_calibration.launch
```

## ainex_driver/ainex_sdk

**Purpose:** SDK-facing sensor/control glue for the robot.\
**Init:**

``` bash
roslaunch ainex_sdk sensor_node.launch
```

## ainex_driver/ros_robot_controller

**Purpose:** Main robot controller node (topics, services for motion).\
**Init:**

``` bash
roslaunch ros_robot_controller ros_robot_controller_node.launch
```

## ainex_driver/ainex_kinematics

**Purpose:** Kinematics/controller logic (poses, gait, joint control).\
**Init:**

``` bash
roslaunch ainex_kinematics ainex_controller.launch
```

## ainex_description (under ainex_simulations/)

**Purpose:** URDF + RViz + Gazebo world config.\
**Init:** - **RViz model:**

``` bash
roslaunch ainex_description display.launch
```

-   **Gazebo:**

``` bash
roslaunch ainex_description gazebo.launch
```

## ainex_gazebo (under ainex_simulations/)

**Purpose:** Gazebo world/model spawner and controllers.\
**Init:** - **Empty world:**

``` bash
roslaunch ainex_gazebo empty_world.launch
```

-   **Worlds:**

``` bash
roslaunch ainex_gazebo worlds.launch
```

-   **Spawn model:**

``` bash
roslaunch ainex_gazebo spwan_model.launch
```

-   **Position control:**

``` bash
roslaunch ainex_gazebo position_controller.launch
```

## ainex_app

**Purpose:** Top-level application entrypoints and web bridge.\
**Init:** - **App:**

``` bash
roslaunch ainex_app start.launch
```

-   **Web:**

``` bash
roslaunch ainex_app rosbridge.launch
```

## ainex_example

**Purpose:** Ready-to-run task demos.\
**Init (examples):** - **Multi-control:**

``` bash
roslaunch ainex_example multi_control.launch
```

-   **Face detect/track:**

``` bash
roslaunch ainex_example face_detect_node.launch
roslaunch ainex_example face_track_node.launch
```

-   **Visual patrol:**

``` bash
roslaunch ainex_example visual_patrol_node.launch
```

-   **Kick ball:**

``` bash
roslaunch ainex_example kick_ball_node.launch
```

-   **Obstacles/hurdles/climb:** corresponding launch files under
    scripts/...\
-   **AprilTag detection/track:** scripts/apriltag_detection/... and
    scripts/apriltag_track/...

## ainex_tutorial

**Purpose:** Tutorial/sample code.\
**Init:** No central launch file found; check package scripts/nodes.

## ainex_interfaces

**Purpose:** Custom messages for AINEX stack.\
**Init:** No node to run. Build and import the message types.

------------------------------------------------------------------------

# Large Models (LLM/ASR/TTS)

## large_models

**Purpose:** LLM/ASR/TTS nodes and orchestration.\
**Init:** - **Start suite:**

``` bash
roslaunch large_models start.launch
```

-   **TTS only:**

``` bash
roslaunch large_models tts_node.launch
```

-   **Voice detection:**

``` bash
roslaunch large_models vocal_detect.launch
```

-   **Agent process:**

``` bash
roslaunch large_models agent_process.launch
```

## large_models_msgs

**Purpose:** Message definitions for large_models.\
**Init:** No node to run. Build to use the messages.

## large_models_examples

**Purpose:** End-to-end LLM demos (camera, control, tracking, etc.).\
**Init (pick one):** - **With camera:**

``` bash
roslaunch large_models_examples vllm_with_camera.launch
```

-   **Transport control:**

``` bash
roslaunch large_models_examples vllm_transport_control.launch
```

-   **Track:**

``` bash
roslaunch large_models_examples vllm_track.launch
```

-   **Dietitian:**

``` bash
roslaunch large_models_examples vllm_dietitianl.launch
```

-   **LLM patrol/pose/move/kick:**

``` bash
roslaunch large_models_examples llm_visual_patrol_control.launch
roslaunch large_models_examples llm_pose_check.launch
roslaunch large_models_examples llm_move_control.launch
roslaunch large_models_examples llm_kick_ball_control.launch
```

------------------------------------------------------------------------

# Third-party Packages

## apriltag_ros

**Purpose:** AprilTag 3 detection and TF publishing.\
**Init:** - **Continuous detection:**

``` bash
roslaunch apriltag_ros continuous_detection.launch
```

-   **Single-image server/client:**

``` bash
roslaunch apriltag_ros single_image_server.launch
roslaunch apriltag_ros single_image_client.launch
```

## imu_calib

**Purpose:** IMU calibration (C++ utilities and params).\
**Init:** No launch file in-tree; run supplied nodes/tools or use
ainex_calibration/imu_calibration.launch.

## calibration_imu (ros-calibration_imu)

**Purpose:** Calibration wrapper using sensor_msgs_ext.\
**Init:** No launch file in this package; typically used via
ainex_calibration or your own launch.

## sensor_msgs_ext

**Purpose:** Extra sensor message definitions.\
**Init:** No node to run. Build to use the messages.
