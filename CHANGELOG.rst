^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pacmod_game_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add minor bug fixes
  Prior to this commit there were minor bugs due to the changes for easier joystick support. This commit fixes those bugs.
* Added additional buttons for logitech gamepad.
  Prior to this commit we were missing the full button suite for the logitech gamepad. This commit adds those buttons.
* Made adding additional gamepads way easier.
  Added basic framework for Logitech G29.
  Added support for Nintendo Switch Wired Controller Plus.
  Made selecting a game controller and a steering thumbstick much easier.
  Made adding additional gamepads much easier.
* Replacing pound-defines with enum.
* Merge pull request `#1 <https://github.com/astuff/pacmod_game_control/issues/1>`_ from astuff/lbucklandAS-patch-1
  Fix print error bug in startup sequence
* Updated license in package.xml.
* Added Travis support.
* Reversed steering output to match changes in firmware 2.0.0.
* Added vehicle type 4.
* Fixed bug with trigger reporting in joy node.
* Updated package.xml to format 2.
* Re-released under MIT license.
* Removed extra layer of launch folders on install.
* Added SocketCAN support.
* Changed launch file for kvaser_interface.
* Fixed subscription to wrong speed topic.
* Fixed brake scaling again.
* Changed pacmod_game_control to send positive brake commands.
* Set last speed report to NULL.
* Added license.
* Don't do speed scaling if speed isn't valid.
* Added brake_scale_val on Logitech controller.
* Added headlight and horn support.
* Removed 0.6 scaling value from Lexus accel.
* Reversed steer output for Lexus.
* Got rid of throttle offset for Lexus (only allow 0.6 max command).
* Adjusted max steering angle for Lexus.
* Reverted default vehicle type to GEM in launch file.
* Removed reference to unsupported vehicle.
* Added argument to enable/disable launching PACMod.
* Fixed windshield wiper ROS topic bug. Fixed bug for vehicle type 3. Added needed parameter to launch file (for fixed joy node).
* Added code to handle semi windshield wipers.
* Fixed issue with hazard light command.
* Added parameters for accel and brake scaling.
* Removed some restrictions on sending if values didn't change.
* Added max_veh_speed as required parameter. Added mutexes. Added C++11 support.
* Tested removal of publish safeguards.
* Removed redundant ROS spin.
* Adjusted defaults in launch file.
* Added launch file option to select either Logitech or HRI gamepad. Modified code to handle the different button/axis mappings.
* Changed LOW to the forward gear.
* Fixed bug when using left thumbstick for shifting.
* Lowered default steering speed in launch file.
* Added launch file parameters for steering axis on gamepad and max steering speed.
* Fixes for heartbeat and vehicle speed.
* Reflected name changes in pacmod and pacmod_msgs.
* Fixed namespacing issues in launch file and node.
* Made namespace for game_control nodes different from pacmod nodes.
* Fixed duplicate pacmod node in launch file.
* Added publish and subscribe to readme. Added comments.
* Removed pacmod_defines.h (not necessary).
* Added launch file.
* Moved callback to spinner.
* Added hazard light functionality.
* Added basic README.
* Removed product manufacturer name from code.
* Changes from topic changes in pacmod.
* Setting brake_cmd to noramlized value.
* Adding debouncing and checking for value changes.
* Setting override to not be latched.
* Code cleanup and working on smoothing the steering control.
* Finished changes for PCB v1.4.
* Working on surious disables and accelerator sticking
* Debugging slow response.
* Initial commit
* Contributors: Christopher Vigna, Daniel Stanek, Joe Driscoll, Joe Kale, Joshua Whitley, Lucas Buckland, Lyle Johnson
