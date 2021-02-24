## PACMod Game Controller ##

[![CircleCI](https://circleci.com/gh/astuff/pacmod_game_control/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod_game_control/tree/master)

An interface node to allow control of the PACMod system with a game controller
that is represented in ROS by a Joy node.

![Left: Front Layout of logitech Controller; Right: Side-button layout of logitech controller
](/controller_img.png "controller_img.png")
| Button | Action | Notes |
| - | - | - |
| **Directional Pad (left-hand side)** | **Headlights and Turn Signals** | |
| Up | Headlights | Some vehicles only |
| Left | Left turn signal | |
| Right | Right turn signal | |
| Down | Hazards | Some vehicles only |
| **Button Pad (right-hand side)** | **Gear Selection** | |
| A | Drive | |
| B | Reverse | |
| X | Neutral | |
| Y | Park | |
| **Center region** | | |
| Back | Enable/Disable | 'Back' and 'Start' buttons must be pressed simultaneously to enable by-wire mode. 'Back' button must be pressed to disable by-wire mode.|
| Start | Enable | 'Back' and 'Start' buttons must be pressed simultaneously to enable by-wire mode.|
| Mode | Not supported | Do not use. Mode light should be OFF at all times. Pressing mode button will change button mapping.|
| **Joystick** | **Steering** | **Joystick click buttons unused** |
| Left joystick | Steering | Steering defaults to left joystick, but can be set to right joystick by operator. |
| Right joystick | Steering | Only if set by operator; steering defaults to left joystick |
| Left bumper | Unused | |
| Left trigger | Brake | |
| Right bumper | Horn | |
| Right trigger | Throttle | |
### PUBLISHERS ###

 - `[pacmod/PacmodCmd]` *as_rx/turn_cmd* - Commands the turn signal subsystem to transition to a given state[enum].
 - `[pacmod/PacmodCmd]` *as_rx/shift_cmd* - Commands the gear/transmission subsystem to shift to a different gear[enum].
 - `[pacmod/PacmodCmd]` *as_rx/accel_cmd* - Commands the throttle subsystem to seek a specific pedal position[pct - 0.0 to 1.0].
 - `[pacmod/PositionWithSpeed]` *as_rx/steer_cmd* - Commands the steering subsystem to seek a specific steering wheel angle[rad] at a given rotation           speed[rad/sec].
 - `[pacmod/PacmodCmd]` *as_rx/brake_cmd* - Commands the brake subsystem to seek a specific pedal position[pct - 0.0 to 1.0].
 - `[std_msgs/Bool]` *as_rx/override* - Enables[true] or disables[false] the PACMod override flag.

### SUBSCRIBERS ###

 - `[std_msgs/Bool]` *as_tx/override* - The PACMod override flag.
 - `[sensor_msgs/Joy]` *game_control/joy* - Joystick commands from Joy node.
