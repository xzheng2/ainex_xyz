# Ainex Controller GUI - Functions Reference

## Location
- App runs inside Docker container `ainex`
- Source: `/home/ubuntu/software/ainex_controller/main.py`
- Launch: `docker exec -u ubuntu -w /home/ubuntu ainex /bin/zsh -c 'bash ~/software/ainex_controller/ainex_controller.sh'`
- Working dir for all python calls: `/home/ubuntu/software/ainex_controller`

---

## Function 1: Manual Button (servoPowerDown)

**Widget name:** `Button_ServoPowerDown`
- English label: `"Manual"` (line 264)
- Chinese label: `"手掰编程"` (line 236)
- Connected at line 112: `self.Button_ServoPowerDown.pressed.connect(lambda: self.button_editaction_clicked('servoPowerDown'))`

**GUI behavior** (`button_editaction_clicked('servoPowerDown')`, lines 424-430):
```python
if name == 'servoPowerDown':
    for ID in self.joints:
        if not self.checkBox_object[ID].isChecked():
            unload_servo(ID)   # disables torque so robot can be physically posed
    self.message_from('success')
```
Cuts torque on all servos (limp mode) for physical hand-posing.

**Simulate from CLI:**
```bash
docker exec -u ubuntu -w /home/ubuntu/software/ainex_controller ainex python3 -c "
from servo_controller import unload_servo
for ID in range(1, 21):
    unload_servo(ID)
    print(f'Unloaded servo {ID}')
print('servoPowerDown complete - all servos limp')
"
```

---

## Function 2: Run Action Group

**GUI behavior:** User selects an action name from the action list dropdown, clicks "Run action" (`Button_RunAction`). Calls `run_action_group(name)` which spawns a thread running `ActionGroupController.run_action(name)`.

**Action files location:** `/home/ubuntu/software/ainex_controller/ActionGroups/*.d6a` (SQLite databases)

**Available actions (key ones):**
`stand`, `stand_low`, `walk_ready`, `forward`, `back`, `turn_left`, `turn_right`,
`move_left`, `move_right`, `left_shot`, `right_shot`, `left_shot_long`, `right_shot_long`,
`greet`, `wave`, `lie_to_stand`, `recline_to_stand`, `calib`, `hand_back`, `hand_open`

**How run_action works** (`action_group_controller.py`):
```python
# Opens .d6a SQLite, reads rows from ActionGroup table
# Each row: (index, time_ms, pulse_id1, pulse_id2, ... pulse_idN)
# For each frame:
board.bus_servo_set_position(time_ms / 1000.0, [[id, pulse], ...])
time.sleep(time_ms / 1000.0)
```

**Simulate from CLI:**
```bash
docker exec -u ubuntu -w /home/ubuntu/software/ainex_controller ainex python3 -c "
import time
from servo_controller import run_action_group, controller

run_action_group('stand')   # replace 'stand' with any action name
print('Running action...')
time.sleep(0.1)
while controller.running_action:
    time.sleep(0.05)
print('Action complete')
"
```

**Key files:**
- `servo_controller.py` — `run_action_group(name)`, `stop_action_group()`, `unload_servo(id)`, `get_servo_pulse(id)`, `set_servo_pulse(id, pulse, time_ms)`
- `action_group_controller.py` — `ActionGroupController.run_action(name)`, reads `.d6a` SQLite files

---

## Servo / Joint IDs

| Joint | ID | Joint | ID |
|-------|----|-------|----|
| l_ank_roll | 1 | r_ank_roll | 2 |
| l_ank_pitch | 3 | r_ank_pitch | 4 |
| l_knee | 5 | r_knee | 6 |
| l_hip_pitch | 7 | r_hip_pitch | 8 |
| l_hip_roll | 9 | r_hip_roll | 10 |
| l_hip_yaw | 11 | r_hip_yaw | 12 |
| l_sho_pitch | 13 | r_sho_pitch | 14 |
| l_sho_roll | 15 | r_sho_roll | 16 |
| l_el_pitch | 17 | r_el_pitch | 18 |
| l_el_yaw | 19 | r_el_yaw | 20 |
| head_pan | 23 | head_tilt | 24 |

---

## ROS Walking Control (for future keyboard teleoperation)

**Service:** `/walking/command` (`SetWalkingCommand`) — commands: `enable_control`, `start`, `stop`
**Topic:** publish `AppWalkingParam` to `/app/set_walking_param`

```python
from ainex_interfaces.msg import AppWalkingParam
from ainex_interfaces.srv import SetWalkingCommand

rospy.ServiceProxy('/walking/command', SetWalkingCommand)('enable_control')
rospy.ServiceProxy('/walking/command', SetWalkingCommand)('start')

msg = AppWalkingParam()
msg.x = 0.02     # forward (+) / backward (-)
msg.y = 0.015    # strafe left/right
msg.angle = 5    # rotation degrees
msg.height = 0.025
param_pub.publish(msg)

rospy.ServiceProxy('/walking/command', SetWalkingCommand)('stop')
```

## Reference Scripts
- Joystick control (existing): `/home/ubuntu/ros_ws/src/ainex_peripherals/scripts/joystick_control.py`
- Walking launch: `/home/ubuntu/ros_ws/src/ainex_peripherals/launch/joystick_control.launch`
- Penalty kick example: `/home/pi/docker/tmp/penalty_kick_node.py`

---

## Planned Work (Incomplete)

The user wants to implement a **keyboard walking teleoperation mode** triggered by the Manual button:
- Manual button pressed → robot enters keyboard walking mode
- Arrow keys / WASD → forward/back/turn
- Manual button again → stop & exit mode
- Status to be shown in the GUI

**NOT YET IMPLEMENTED** — planning was interrupted. Resume from plan mode.
