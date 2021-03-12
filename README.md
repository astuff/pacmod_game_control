## PACMod Game Controller ##

[![CircleCI](https://circleci.com/gh/astuff/pacmod_game_control/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod_game_control/tree/master)

An interface node to allow control of the PACMod system with a game controller
that is represented in ROS by a Joy node.

For more information, see the [ROS Wiki](http://wiki.ros.org/pacmod_game_control).

Please see [PACMod3 readme](https://github.com/astuff/pacmod3/blob/master/README.md) to use correct version of driver for a vehicle.

## PACMod Game Controller ##

[![CircleCI](https://circleci.com/gh/astuff/pacmod_game_control/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod_game_control/tree/master)

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
| Left bumper | Wipers | Some vehicles only |
| Left trigger | Brake | |
| Right bumper | Horn | Some vehicles only |
| Right trigger | Throttle | |
