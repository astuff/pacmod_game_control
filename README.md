## PACMod Game Controller ##

[![CircleCI](https://circleci.com/gh/astuff/pacmod_game_control/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod_game_control/tree/master)

An interface node to allow control of the PACMod system with a game controller
that is represented in ROS by a Joy node.

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
