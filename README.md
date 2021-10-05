# PACMod Game Controller #

[![CircleCI](https://circleci.com/gh/astuff/pacmod_game_control/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod_game_control/tree/master)

This ROS package provides software for controlling a PACMod system using a game controller.
The `pacmod_game_control` ROS node subscribes to gamepad input data coming from the ROS joy node and interfaces with the [`pacmod3`](https://github.com/astuff/pacmod3) ROS driver to control the PACMod.

## Installation 

Note: Previously pacmod_game_control was released via the ROS buildfarm. 
This has changed as of Ubuntu 20.04 (ROS2 Foxy and ROS1 Noetic) to keep old package versions available for download, which gives users greater control over their installed software and also allows downgrades if an upgrade breaks software dependencies.

**ROS1 Noetic and ROS2:**

Install pacmod_game_control using our debian repository:

```sh
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-pacmod-game-control
```

**ROS1 Melodic or older**

```sh
sudo apt install ros-$ROS_DISTRO-pacmod-game-control
```
## Controls

### Supported Controllers

The currently supported controllers are:

- Logitech F310
- [FORT Robotics Safe Remote Control](https://autonomoustuff.com/products/fort-remote-control-system)
- Logitech G29 Steering Wheel and Pedals
- XBOX ONE Wired or Wireless Controller

***NOTE***: The Logitech F310 controller has a switch on the back of the controller for switching between "X" and "D". 
Ensure the switch is always set to "X". 
In addition, the Logitech F310 also has a MODE button that toggles a green light, ensure the green light is always off.

In the event that you want to add support for a new controller: 
Update `controllers.h` to add a class for your new controller and specify the button and axes mapping. 
Override any functions that are be different from the default control scheme.

### Logitech F310 Control Scheme

Below is the control scheme for the Logitech F310, the control scheme is similar for other controllers.

![Left: Front Layout of logitech Controller; Right: Side-button layout of logitech controller
](/controller_img.png "controller_img.png")

*Note:* Do not use the MODE button. MODE light should be OFF at all times otherwise the button mapping will change.

| Action | Button | Notes |
| - | - | - |
| **Enable/Disable** | **Center region** | |
| Enable | BACK and START | Buttons must be pressed simultaneously to enable by-wire mode |
| Disable | BACK | button must be pressed to disable by-wire mode.|
| **Gear Selection** | **Button Pad (right-hand side)** | |
| Drive | A | |
| Reverse | B | |
| Neutral | X | |
| Park | Y | |
| **Steering** | **Joystick** | |
| Steering | Left Joystick | |
| Brake | Left Trigger | |
| Throttle | Right Trigger | |
| Windsheild Wipers | Left Bumper | Not supported by all vehicle platforms |
| Horn | Right Bumper | Not supported by all vehicle platforms |
| **Headlights and Turn Signals** | **Directional Pad (left-hand side)** | |
| Left turn signal | Left | |
| Right turn signal | Right | |
| Headlights | Up | Not supported by all vehicle platforms |
| Hazards | Down | Not supported by all vehicle platforms |

## ROS API

### Launch Arguments

- **launch_driver**: Set this to true if you wish to launch the pacmod3 driver along with the `pacmod_game_control` node. Defaults to `false`.
- **pacmod_vehicle_type**: Use this to set your vehicle type. See launch file for available options.
- **controller_type**: Use this to set your controller type. See launch file for available options.
- **steering_max_speed**: The maximum rotational speed of the steering wheel in (rad/s).
- **max_veh_speed**: Maximum vehicle speed in (m/s), only used for speed-based steering damping.
- **accel_scale_val**: Scale value applied to outgoing accel commands, useful for decreasing sensitivity. Defaults to `1.0`.
- **brake_scale_val**: Scale value applied to outgoing brake commands, useful for decreasing sensitivity. Defaults to `1.0`.
- **use_socketcan**: Set this to true if socketCAN is being used to connect to the PACMod. If set false it is assumed a Kvaser CAN device is being used with Kvaser canlib drivers to connect to the PACMod. Defaults to `false`.
- **pacmod_can_hardware_id**: The hardware id of the kvaser device, only applies if `use_socketcan` is false.
- **pacmod_can_circuit_id**: The circuit/channel id that the PACMod is plugged into on the kvaser device, only applies if `use_socketcan` is false.
- **pacmod_socketcan_device**: The device id of the SocketCAN channel the PACMod is plugged into, only applies if `use_socketcan` is true.
