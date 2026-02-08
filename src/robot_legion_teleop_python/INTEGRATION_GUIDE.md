# Robot Legion Teleop Integration Guide

## Overview

`robot_legion_teleop_python` is a ROS 2 package providing GFE (Government-Furnished Equipment) reference implementation of heterogeneous robot control for the DIU Fleet Orchestrator. It enables:

- **Multi-robot command execution** via playbook-based primitives (transit, rotate, strafe, hold, diagonal, turn)
- **Hardware abstraction** supporting differential-drive and omnidirectional (mecanum) robots
- **Dynamic robot discovery** via heartbeat topics for zero-config multi-robot systems
- **Control arbitration** via control-lock manager for exclusive command ownership
- **Audit logging** for DIU compliance and after-action debugging
- **Keyboard teleoperation** with real-time robot switching and state feedback

## Architecture

### Core Modules

#### `playbook_contract.py`
Defines the command interface between the intent translator and robot execution.

**Canonical Primitives** (6 allowed commands):
- `hold`: Stop all motion
- `transit`: Move forward/backward
- `rotate`: Rotate left/right (diff) or rotate in place (mecanum)
- `strafe`: Side-to-side motion (mecanum only; safe no-op on diff)
- `diagonal`: Diagonal movement (mecanum only; safe no-op on diff)
- `turn`: Curved arc motion (diff) or in-place rotation (mecanum)

**Parameter Structure**:
```json
{
  "command_id": "unique_cmd_id",
  "intent_id": "user_intent_uuid",
  "robot": "robot_name",
  "command": "transit|rotate|...",
  "parameters": {
    "speed": 0.5,
    "direction": "fwd|bwd|left|right|...",
    "duration_s": 5.0
  }
}
```

#### `hardware_interface.py`
GPIO/PWM abstraction for motor control with mock fallback.

**Features**:
- RPi.GPIO integration for real hardware (H-bridge, TB6612FNG drivers)
- Mock mode for desktop development
- Per-motor inversion flags for polarity correction
- 1000 Hz PWM frequency

**Usage**:
```python
from hardware_interface import HardwareInterface

# Load GPIO map from drive profile
hw = HardwareInterface(gpio_map)

# Set motor outputs: (left_duty%, left_direction, right_duty%, right_direction)
hw.set_motor(50, 1, -50, -1)  # Rotate left
hw.stop()  # Safe shutdown
```

#### `drive_profiles.py`
Robot configuration registry supporting dynamic drive-type and hardware selection.

**Profile Structure**:
```yaml
defaults:
  wheel_separation: 0.18  # meters
  max_linear_speed: 0.4   # m/s

drive_profiles:
  diff_drive:    # Differential drive (2-motor)
    wheel_separation: 0.18
  mecanum_drive:    # Omnidirectional (4-motor mecanum)
    wheel_separation: 0.20

hardware_profiles:
  L298N_diff:   # 2-channel H-bridge relay
    pins:
      en_left: 17
      in1_left: 27
      in2_left: 22
      en_right: 23
      in1_right: 24
      in2_right: 25
    invert_left: false
    invert_right: true

robots:
  robot1:
    drive_profile: diff_drive
    hardware_profile: L298N_diff
  robot2:
    drive_profile: mecanum_drive
    hardware_profile: dual_tb6612_mecanum
```

**Discovery Modes**:
1. **Static YAML** (if installed): `$INSTALL/share/robot_legion_teleop_python/config/robot_profiles.yaml`
2. **Dynamic fallback**: Via heartbeat JSON messages if robot not in YAML

#### `audit_logger.py`
Structured JSON audit trail for compliance and debugging.

**Events Logged**:
- Motor command arrival (cmd_vel)
- Playbook goal acceptance/rejection
- Goal execution start/success/failure
- Control lock acquire/release
- Teleop commands (movement, robot switch)

**Output Format** (JSONL):
```json
{
  "timestamp": 1770072000.5,
  "robot": "robot1",
  "source": "motor_driver|executor|teleop|control_lock_manager",
  "source_id": "client_id",
  "command_id": "cmd_uuid",
  "parameters": {...},
  "status": "received|granted|denied|succeeded|failed",
  "details": "optional context",
  "duration_s": 2.5
}
```

**Log Files**:
- Motor driver: `/tmp/robot_{name}_audit.jsonl`
- Unit executor: (logs to motor driver file)
- Teleop: `/tmp/teleop_audit.jsonl`
- Control lock manager: `/tmp/control_lock_audit.jsonl`

#### `control_lock_manager.py`
Central arbiter for exclusive robot control (runs on laptop/control station).

**Protocol** (JSON/String messages):

Request:
```json
{
  "action": "acquire|release|heartbeat",
  "robot": "robot1",
  "client_id": "teleop_1a2b"
}
```

Response on `/control_lock/response/{client_id}`:
```json
{
  "ok": true,
  "action": "acquire",
  "robot": "robot1",
  "owner": "teleop_1a2b"
}
```

**Topics**:
- `/control_lock/request` - clients send requests
- `/control_lock/response/{client_id}` - responses directed to client
- `/control_lock/state` - periodically published lock status

**Lease Time**: 5 seconds (configurable via `lease_sec` parameter); locks expire if no heartbeat is received.

### Node Architecture

```
┌─────────────────────────────────────────────────────┐
│ Control Station (Laptop)                             │
├─────────────────────────────────────────────────────┤
│  teleop_legion_key (keyboard input)                 │
│  ├─ publishes Twist → /<robot>/cmd_vel             │
│  └─ subscribes /<robot>/heartbeat for discovery    │
│                                                      │
│  control_lock_manager                              │
│  ├─ arbitrates robot ownership                      │
│  └─ publishes /control_lock/state                   │
└─────────────────────────────────────────────────────┘
              ↓ (ROS 2 topics) ↓
┌─────────────────────────────────────────────────────┐
│ Robot (Raspberry Pi, etc.)                          │
├─────────────────────────────────────────────────────┤
│  motor_driver_node                                  │
│  ├─ subscribes /<robot>/cmd_vel                     │
│  ├─ publishes /<robot>/heartbeat                    │
│  └─ controls GPIO/PWM motors                        │
│                                                      │
│  unit_executor_action_server                        │
│  ├─ provides ExecutePlaybook action                 │
│  └─ maps primitives → Twist → motor_driver         │
│                                                      │
│  heartbeat_node                                     │
│  ├─ publishes /<robot>/heartbeat (drive_type, hw)  │
│  └─ enables control laptop discovery                │
└─────────────────────────────────────────────────────┘
```

## Usage

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select robot_legion_teleop_python
source install/setup.bash
```

### 2. Validate Installation

```bash
python3 src/robot_legion_teleop_python/test/test_playbook_validation.py
```

All 27 tests should pass.

### 3. Launch a Robot

On the robot (Pi):
```bash
# Start motor driver + heartbeat (publishes to /<robot>/cmd_vel and /<robot>/heartbeat)
ros2 launch robot_legion_teleop_python robot_bringup.launch.py \
  drive_type:=diff_drive \
  hardware:=L298N_diff
```

Parameters:
- `drive_type`: `diff_drive` (2-motor) or `mecanum` (4-motor)
- `hardware`: `L298N_diff` (H-bridge relay) or `dual_tb6612_mecanum` (TB6612FNG dual driver), etc.
- `profiles_path`: Optional custom YAML file path
- `use_camera`: `true` (default) or `false` to disable USB camera

#### Graceful Degradation for Missing Packages

If a robot doesn't have a camera or the `v4l2_camera` package is not installed, the launch will skip the camera node but continue with motor control:

```
[INFO] [launch.user]: [OK] motor_driver_node ready
[INFO] [launch.user]: [OK] heartbeat_node ready
[INFO] [launch.user]: [SKIP] camera: pkg 'v4l2_camera'
[INFO] [launch.user]:        not found
```

**To explicitly disable camera** (faster startup if not needed):
```bash
ros2 launch robot_legion_teleop_python robot_bringup.launch.py \
  use_camera:=false
```

This ensures robots without cameras or with camera hardware failures can still be controlled via the motor_driver node and heartbeat discovery.

### 4. Start Control Station

On the laptop:

**Terminal 1 - Control Lock Manager**:
```bash
ros2 run robot_legion_teleop_python control_lock_manager --ros-args -p lease_sec:=5.0
```

**Terminal 2 - Keyboard Teleop**:
```bash
ros2 run robot_legion_teleop_python teleop_legion_key
```

When started, teleop will:
1. Scan for available robots (robots publishing to `/cmd_vel`)
2. Prompt user to select a robot
3. Display keyboard controls
4. Detect robot offline and prompt for re-selection

**Keyboard Controls**:
```
8 = forward         4 = rotate-left       7 = circle forward-left
2 = backward        6 = rotate-right      9 = circle forward-right
5 = stop            m = switch robot      1 = circle backward-left
                    q = quit              3 = circle backward-right

+/- = adjust speed   (adjust linear/angular speed multipliers)
```

### 5. Send Playbook Goals (from Fleet Orchestrator)

Publish to `/<robot>/playbook_goal` action:

**Request**:
```json
{
  "command_id": "cmd_12345",
  "intent_id": "intent_abc",
  "robot": "robot1",
  "command": "transit",
  "parameters_json": "{\"speed\": 0.5, \"direction\": \"fwd\", \"duration_s\": 10.0}"
}
```

**Response**:
```json
{
  "success": true,
  "result_text": "ok"
}
```

Example (ROS 2 CLI):
```bash
ros2 action send_goal /robot1/playbook_goal robot_legion_teleop_python/action/ExecutePlaybook \
  '{command_id: "cmd_1", intent_id: "intent_1", robot: "robot1", command: "transit", 
    parameters_json: "{\"speed\": 0.5, \"direction\": \"fwd\", \"duration_s\": 5.0}"}'
```

### 6. Monitor Audit Logs

**Motor driver audit** (Robot-side):
```bash
tail -f /tmp/robot_robot1_audit.jsonl | jq .
```

**Teleop audit** (Control-side):
```bash
tail -f /tmp/teleop_audit.jsonl | jq .
```

**Control lock audit**:
```bash
tail -f /tmp/control_lock_audit.jsonl | jq .
```

## Integration with Fleet Orchestrator

The fleet orchestrator (e.g., running on a central server) should:

1. **Discover robots** by monitoring `/heartbeat` topics (broadcast from each robot)
2. **Publish playbook goals** to `/<robot>/playbook_goal` action server
3. **Monitor status** via audit logs or action feedback
4. **Manage control locks** via `/control_lock/request` and monitor `/control_lock/state`

### Intent Translation Pipeline

```
User Intent
  ↓
Intent Parser (NLP/LLM)
  ↓
Playbook Generator (converts intent to primitives)
  ↓
ExecutePlaybook Goal (published to robot)
  ↓
Unit Executor (maps primitive to Twist)
  ↓
Motor Driver (subscribes Twist, drives GPIO)
  ↓
Audit Log (records all steps for review)
```

## Advanced Configuration

### Custom GPIO Mappings

Create a custom `my_profiles.yaml`:

```yaml
robots:
  my_custom_robot:
    drive_profile: diff_drive
    hardware_profile: custom_tb6612

hardware_profiles:
  custom_tb6612:
    pins:
      en_left: 18
      in1_left: 17
      in2_left: 27
      en_right: 23
      in1_right: 24
      in2_right: 25
      fault_left: 4     # optional fault detection
      fault_right: 26
    invert_left: false
    invert_right: true
```

Launch with custom profiles:
```bash
ros2 launch robot_legion_teleop_python robot_bringup.launch.py \
  profiles_path:=/path/to/my_profiles.yaml
```

### Dynamic Robot Addition (Zero-Config)

If a robot is not in the YAML registry:

1. **Robot starts** with `drive_type` and `hardware` parameters:
   ```bash
   ros2 launch robot_legion_teleop_python robot_bringup.launch.py \
     drive_type:=diff_drive hardware:=L298N_diff
   ```

2. **Heartbeat published** as JSON with drive_type/hardware
3. **Teleop discovers** the robot and adapts behavior (circle turns, etc.) based on received drive_type
4. **Control works** without modifying any YAML

### Odometry Integration (Future)

The unit executor can publish odometry feedback:

```python
# In unit_executor_action_server.py execute_cb()
self._odometry_pub.publish(Odometry())  # After each primitive
```

This enables the fleet orchestrator to track robot position across playbook execution.

## Troubleshooting

### GPIO Map Empty (Motor Won't Move)

**Symptom**: `GPIO map empty` warning on robot startup

**Cause**: Custom YAML not found, or package not properly installed

**Fix**:
1. Rebuild package: `colcon build --packages-select robot_legion_teleop_python`
2. Source install: `source install/setup.bash`
3. Verify YAML exists: `ls install/robot_legion_teleop_python/share/robot_legion_teleop_python/config/robot_profiles.yaml`
4. Manually specify path: `profiles_path:=/path/to/profiles.yaml`

### Motor Direction Reversed

**Symptom**: Robot moves backward when told to move forward

**Fix**: Add `invert_left: true` or `invert_right: true` to hardware profile

### Teleop Can't Find Robot

**Symptom**: Teleop says "No robots available" despite robot running

**Cause**: Robot not publishing heartbeat or different network

**Fix**:
1. Check robot heartbeat: `ros2 topic list | grep heartbeat`
2. Verify network connectivity: `ros2 daemon status`
3. Check firewall: ROS 2 requires UDP ports 7400-7409 open

### Control Lock Timeout

**Symptom**: Teleop commands stop after 5 seconds of inactivity

**Cause**: Control lock lease expired (normal behavior)

**Fix**: Increase `lease_sec` in control_lock_manager:
```bash
ros2 run robot_legion_teleop_python control_lock_manager --ros-args -p lease_sec:=10.0
```

## Performance Characteristics

| Metric | Value |
|--------|-------|
| Motor PWM frequency | 1000 Hz |
| Twist publish rate | 50 Hz (teleop), 20 Hz (playbook) |
| Heartbeat interval | 2 Hz |
| Control lock lease | 5 seconds (configurable) |
| Audit log buffering | Real-time (unbuffered) |

## Compliance & Validation

**DIU Sprint 1 Requirements**:
- ✓ Correct mapping of input intent to valid playbook command (validate_and_normalize)
- ✓ Audit logs for traceability (audit_logger.py)
- ✓ Multi-robot support (heartbeat discovery, dynamic drive_type selection)
- ✓ Hardware agnostic (abstraction layer for GPIO, PWM, etc.)
- ✓ Heterogeneous fleet (diff_drive, mecanum, custom profiles)

**Validation Harness**:
```bash
python3 src/robot_legion_teleop_python/test/test_playbook_validation.py
```

Expected output: `27/27 tests passed ✓`

## References

- **ROS 2 Documentation**: https://docs.ros.org
- **DIU Fleet Orchestrator**: DIU_fleet_orchestrator_solicitation.md
- **Playbook Contract**: playbook_contract.py
- **Hardware Interface**: hardware_interface.py
- **Audit Logging**: audit_logger.py

## Contact & Support

For issues, feature requests, or integration help:

1. Check validation harness: `test_playbook_validation.py`
2. Review audit logs: `/tmp/*_audit.jsonl`
3. Enable ROS 2 debug logging: `export RCL_LOG_LEVEL=DEBUG`
4. Inspect topic traffic: `ros2 topic echo /<robot>/cmd_vel`

---

**Package Version**: 0.1.0  
**Last Updated**: 2025-02-02  
**DIU Compliance**: Sprint 1 Ready
