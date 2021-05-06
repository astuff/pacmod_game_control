## PACMod Game Controller ##

[![CircleCI](https://circleci.com/gh/astuff/pacmod_game_control/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod_game_control/tree/master)

An interface node to allow control of the PACMod system with a game controller
that is represented in ROS by a Joy node.

For more information, see the [ROS Wiki](http://wiki.ros.org/pacmod_game_control).

For access to the DBC file which defines the CAN interface for the PACMod, see the [pacmod_dbc](https://github.com/astuff/pacmod_dbc) repo.

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

Please see [PACMod3 readme](https://github.com/astuff/pacmod3/blob/master/README.md) and [astuff_sensor_msgs readme](https://github.com/astuff/astuff_sensor_msgs/blob/master/README.md) to use correct version of driver for a vehicle. For vehicles using drivers with message migration, astuff_sensor_msgs repo is not required.

| Supported Vehicles | ROS Version Available | PACMod Version | ROS Driver Branch |
| - | - | - | - |
| Polaris GEM Series (e2/e4/e6) MY 2016+ | ROS | PACMod2 | [PGC Driver](https://github.com/astuff/pacmod_game_control/tree/master)|
| Polaris eLXD MY 2016+ | ROS | PACMod2 | [PGC Driver](https://github.com/astuff/pacmod_game_control/tree/master)|
| International Prostar+ 122 | ROS | PACMod2 | [PGC Driver](https://github.com/astuff/pacmod_game_control/tree/master)|
| Lexus RX-450h MY 2016+ | ROS | PACMod3 | [PGC Driver](https://github.com/astuff/pacmod_game_control/tree/master) |
| Lexus RX-450h MY 2016+ V3| ROS | PACMod3 |[PGC Driver with message migration](https://github.com/astuff/pacmod_game_control/tree/maint/pacmod_msg_migration) |
| Kenworth T680 Semi 2017+ |ROS | PACMod3 | [PGC Driver](https://github.com/astuff/pacmod_game_control/tree/master)|
| Freightliner Cascadia DD13 DayCab/Sleeper/Extended-Sleeper | ROS | PACMod3 | [PGC Driver](https://github.com/astuff/pacmod_game_control/tree/master)|
| Tractor 2017+ | ROS | PACMod3 | [PGC Driver (Hexagon Tractor)](https://github.com/astuff/pacmod_game_control/tree/maint/hexagon_tractor)|
| Ford Ranger 2019+ | ROS | PACMod3 |[PGC Driver with message migration](https://github.com/astuff/pacmod_game_control/tree/maint/pacmod_msg_migration) |
| Polaris Ranger X900 | ROS | PACMod3 |[PGC Driver with message migration](https://github.com/astuff/pacmod_game_control/tree/maint/pacmod_msg_migration) |
| Toyota Minivan 2019+ | ROS | PACMod3 | [PGC Driver with message migration](https://github.com/astuff/pacmod_game_control/tree/maint/pacmod_msg_migration) |
| VEHICLE_HCV | ROS | PACMod3 | [PGC Driver with message migration](https://github.com/astuff/pacmod_game_control/tree/maint/pacmod_msg_migration) |
| VEHICLE_FTT | ROS | PACMod3 | [PGC Driver with message migration](https://github.com/astuff/pacmod_game_control/tree/maint/pacmod_msg_migration) |
More coming soon...

**Note**
- In case the joystick controller gets disconnected, the system will remain enabled and hold the last commands unless its overridden. Exception handling for this is currently being implemented and tested for time being.
- Please press the "disable" button firmly when commanding disable through joystick. Debounce implementation for this is currently being implemented and tested for time being.