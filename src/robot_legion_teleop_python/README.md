# Robot Legion Teleop (robot_legion_teleop_python)

This package provides robot-side motor control and control-station teleop for heterogeneous robots. It uses a YAML profile registry to describe robots, drive types, and hardware wiring. This README explains how to add **new robots** and **new robot types** in a chronological, build-first order.

## 0) Prerequisites

- ROS 2 environment sourced on **both** the control machine and each robot.
- This workspace built at least once:

```bash
cd ~/ros2_ws
colcon build --packages-select robot_legion_teleop_python
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 1) Decide Your Drive Type

Pick one of the existing drive types (or add a new one later):

- `diff_drive` (tracked/tank or two-wheel differential)
- `omni` / `mecanum` (4-wheel mecanum)

Drive type determines how `Twist` messages are mixed into motor outputs. This mapping lives in `robot_legion_teleop_python/drive_types.py`.

## 2) Wire the Robot Hardware

Follow the wiring guide that matches your hardware:

- **Differential, dual TB6612**: `GPIO_for_differential_Tb6612_DUAL.md`
- **Mecanum, dual TB6612**: `GPIO_for_mecanum_Tb6612_DUAL.md`

These wiring guides are the authoritative pin mappings used by the profiles in `config/robot_profiles.yaml`.

## 3) Define or Select a Hardware Profile

Hardware profiles live in:

```
config/robot_profiles.yaml
```

Each profile defines GPIO pins and motor-driver parameters. Examples:

- `dual_tb6612_diff` (paired-motor diff drive)
- `dual_tb6612_mecanum` (mecanum with 4 channels)
- `L298N_diff` (L298N or 2-channel H-bridge diff drive)

If your wiring matches an existing profile, you can reuse it. If not, add a new `hardware_profiles.<name>` entry with your GPIO pins.

## 4) Define or Select a Drive Profile

Drive profiles also live in `config/robot_profiles.yaml` and define kinematic and control limits. Examples:

- `diff_drive`
- `mecanum_drive`

If your robot fits an existing drive profile, reuse it. If you need different limits (max speeds, teleop behavior, etc.), you can override them per robot (next step).

## 5) Add the Robot Entry (Per-Robot Configuration)

Add a new entry under `robots:` with:

- `drive_profile`
- `hardware_profile`
- optional `params` overrides

Example (differential, dual TB6612):

```yaml
robots:
  my_tank_bot:
    drive_profile: diff_drive
    hardware_profile: dual_tb6612_diff
    params:
      drive:
        max_linear_mps: 0.35
        max_angular_rps: 1.8
      hardware:
        max_pwm: 85
        pwm_deadband_pct: 2
```

Example (mecanum, dual TB6612):

```yaml
robots:
  my_mecanum_bot:
    drive_profile: mecanum_drive
    hardware_profile: dual_tb6612_mecanum
```

## 6) Build and Deploy to the Robot

On the robot:

```bash
cd ~/ros2_ws
colcon build --packages-select robot_legion_teleop_python
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 7) Launch the Robot Motor Driver

Use the bringup launch file with the correct drive type and hardware profile:

```bash
ros2 launch robot_legion_teleop_python robot_bringup.launch.py \
  drive_type:=diff_drive \
  hardware:=dual_tb6612_diff
```

For mecanum:

```bash
ros2 launch robot_legion_teleop_python robot_bringup.launch.py \
  drive_type:=mecanum \
  hardware:=dual_tb6612_mecanum
```

## 8) Control the Robot (Teleop)

On the control machine:

```bash
ros2 run robot_legion_teleop_python teleop_legion_key
```

Select your robot name when prompted (e.g., `my_tank_bot`).

## 9) Verify

- Check that the motor driver loads the GPIO map (no mock mode warnings).
- Confirm `/robot_name/cmd_vel` has subscribers.

## Adding a New Drive Type (Advanced)

Drive-type mixing is isolated in:

```
robot_legion_teleop_python/drive_types.py
```

Add a new `DriveType` class and register it in `get_drive_type()`. This allows new motion models (e.g., underwater, flight) without changing `motor_driver_node.py`.

## Troubleshooting

- **Robot doesn’t move**: verify GPIO map loads, check `/robot_name/cmd_vel`, confirm wiring.
- **Robot name mismatch**: add it to `robots:` or select it in teleop.
- **No network comms**: check `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, and Wi-Fi.

---

If you want, I can add templates for common robot types directly into `robot_profiles.yaml`.
