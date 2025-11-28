# OpenArm Servo - Bimanual Control

Package for controlling OpenArm bimanual robot using MoveIt Servo.

## Build

```bash
# Navigate to project root
cd SynchroHumanoid

# Navigate to OpenArm workspace and build
cd projects/openarm
colcon build --packages-select openarm_servo
source install/setup.bash

# Load environment setup
source scripts/system_load.sh

```

## Launch Servo Test (Bimanual Control)

Launch servo nodes and RViz to test both arms.

```bash
ros2 launch openarm_servo servo_test.launch.py
```

## Keyboard Control

Control each arm independently using keyboard.

### Left Arm Keyboard Control

```bash
# Terminal 1: Launch servo (if not already running)
ros2 launch openarm_servo servo_test.launch.py

# Terminal 2: Left arm keyboard control
ros2 run openarm_servo keyboard_servo_left

# Terminal 3: Right arm keyboard control
ros2 run openarm_servo keyboard_servo_right
```

### Keyboard Control Mapping

```text
[Cartesian Control]
W/S: X-axis forward/backward
A/D: Y-axis left/right
Q/E: Z-axis up/down
I/K: Roll rotation
J/L: Pitch rotation  
U/O: Yaw rotation

[Joint Control]
Number keys to select joint, +/- keys to control

ESC: Exit
```

## Topics

### Input Topics

- `/left_arm/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped) - Left arm Cartesian velocity commands
- `/right_arm/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped) - Right arm Cartesian velocity commands
- `/left_arm/servo_node/delta_joint_cmds` (control_msgs/JointJog) - Left arm joint velocity commands
- `/right_arm/servo_node/delta_joint_cmds` (control_msgs/JointJog) - Right arm joint velocity commands

### Output Topics

- `/left_joint_trajectory_controller/joint_trajectory` - Left arm trajectory commands
- `/right_joint_trajectory_controller/joint_trajectory` - Right arm trajectory commands
- `/left_arm/servo_node/status` - Left arm status
- `/right_arm/servo_node/status` - Right arm status

## Configuration Files

- `config/openarm_left_simulated_config.yaml` - Left arm servo configuration
- `config/openarm_right_simulated_config.yaml` - Right arm servo configuration

### Key Parameters

- `command_in_type`: "speed_units" (m/s, rad/s) or "unitless" ([-1:1])
- `publish_period`: Control period (seconds)
- `low_latency_mode`: When true, processes commands immediately upon receipt
- `planning_frame`: `openarm_body_link0` (common reference frame for both arms)
- `move_group_name`: `left_arm` or `right_arm`

## Troubleshooting

### Robot Not Moving

1. **Check joint states:**

    ```bash
    ros2 topic hz /joint_states
    ```

2. **Check servo status:**

    ```bash
    ros2 topic echo /right_arm/servo_node/status
    ros2 topic echo /left_arm/servo_node/status
    ```

    `data: 0` indicates normal operation.

3. **Check command reception:**

    ```bash
    ros2 topic hz /right_arm/servo_node/delta_twist_cmds
    ```

4. **Check controller output:**

    ```bash
    ros2 topic echo /right_joint_trajectory_controller/joint_trajectory
    ```

### Common Issues

- **Low command frequency**: Use `-r 50` option for at least 50Hz transmission
- **Wrong frame_id**: Use `openarm_left_hand` or `openarm_right_hand`
- **Velocity too high**: Start slowly with 0.01~0.05 m/s
- **Joint limit reached**: Check robot pose in RViz

## Notes

- Both arms are controlled independently
- Each arm uses its own namespace (`/left_arm`, `/right_arm`)
- Shares common planning frame (`openarm_body_link0`) for coordinate system consistency
- Auto-stops if no commands received for `incoming_command_timeout: 0.3` seconds
