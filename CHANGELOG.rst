^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pacmod_game_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2020-02-05)
------------------
* Merge pull request `#68 <https://github.com/astuff/pacmod_game_control/issues/68>`_ from astuff/fix/shift_low_bug
  changed from shift low on button press to shift high on button press
* Merge pull request `#67 <https://github.com/astuff/pacmod_game_control/issues/67>`_ from astuff/feature/removing_enums_and_using_ROS_message__values_instead
  Feature/removing enums and using ros message  values instead
* Merge pull request `#64 <https://github.com/astuff/pacmod_game_control/issues/64>`_ from astuff/freightliner_cascadia
  Adding Freightliner Cascadia DD13 Trucks
* Merge pull request `#65 <https://github.com/astuff/pacmod_game_control/issues/65>`_ from astuff/feature/door_system
  adding DoorSystem
* Adding JUPITER_SPIRIT Vehicle
* Merge pull request `#61 <https://github.com/astuff/pacmod_game_control/issues/61>`_ from astuff/fix/speed_limit
  Fix/speed limit - clipped steer rate limit
* Merge pull request `#60 <https://github.com/astuff/pacmod_game_control/issues/60>`_ from astuff/maint/ci_remove_lunar
  CI: Removing Lunar build.
* Merge pull request `#59 <https://github.com/astuff/pacmod_game_control/issues/59>`_ from astuff/fix/lexus_G29
  made changes to lexus max steer rotation(vehicle_2) and changes so that G29 steering wheel is functional
* Merge pull request `#58 <https://github.com/astuff/pacmod_game_control/issues/58>`_ from astuff/fix/veh_5_headlights
  Updated veh_5 headlight states to match current firmware
* Merge pull request `#55 <https://github.com/astuff/pacmod_game_control/issues/55>`_ from astuff/ranger
  Polaris Ranger support.
* CI: Removing Indigo build.
* Fixed HRI accel command max was limiting to 0.6
* Contributors: Amruta Mhaske, Benjamin Jansky, Chris, Joshua Whitley, Sam Rustan, Sanaz Fattahalhosseini, Zach Oakes, Mike Lemm, Rinda Gunjala, Sneha Ganesh

2.3.0 (2018-12-12)
------------------
* Merge pull request `#54 <https://github.com/astuff/pacmod_game_control/issues/54>`_ from astuff/maint/hri_remap_part_two
* A comment explaining the axes
* Fixed not publishing turn signal command on board rev 2
* Handle HRI turn signals seperately from the other controllers
* Merge pull request `#53 <https://github.com/astuff/pacmod_game_control/issues/53>`_ from astuff/maint/hri_remap
* Changed mapping of HRI enable/disable from U/D to R/L
  This fixes some aberrant behavior that we were seeing with
  EMI in the remote system
* Merge pull request `#51 <https://github.com/astuff/pacmod_game_control/issues/51>`_ from astuff/maint/update_url
* Updating README and adding URLs to package.xml.
* Merge pull request `#48 <https://github.com/astuff/pacmod_game_control/issues/48>`_ from astuff/feature/clear_fault
* Refactoring of how we're handling the shifter command
  Mostly just to get rid of repeated code, but also I think
  it's significantly more readable this way.
* When sending clear_overrides, also send clear_faults
  This update is req'd for Lexus 1.1 and later
* Merge pull request `#47 <https://github.com/astuff/pacmod_game_control/issues/47>`_ from astuff/maint/add_veh_6
* Contributors: Daniel-Stanek, Joshua Whitley, Mike Lemm, Sam Rustan, Zach Oakes

2.2.0 (2018-08-30)
------------------
* Merge pull request `#44 <https://github.com/astuff/pacmod_game_control/issues/44>`_ from astuff/feat/lexus_braking_curve
* 0.625 and 6.25 are not the same value
* Added cubic braking curve
* Merge pull request `#43 <https://github.com/astuff/pacmod_game_control/issues/43>`_ from astuff/feat/multiple_button_enable
* Press start AND select to enable
* Merge pull request `#42 <https://github.com/astuff/pacmod_game_control/issues/42>`_ from astuff/fix/resume_control_after_estop
* Changing pacmod_enable flag after return form e-stop
* Contributors: Joshua Whitley, Kyle Rector, Sam Rustan, Zach Oakes

2.1.0 (2018-08-15)
------------------
* Removing clone of joystick_drivers from Melodic build.
* Change required brake command from 50% to 25%
* Adding brake requirement to change currently published shift command
  Before this commit, a shift command could be issued without the brake being
  depressed. After this commit, the brake must be depressed to 50% before a new
  shift command can be issued via the game control node.
* Headlights fixed and tested.
* Formatting and trying to get headlights to latch state.
* Added state change debouncing for button presses between off state, regular headlights, and high beams.
* Making headlight and wiper states static.
* Fixed: horn no longer on same button as hazards
* Fix headlights and wipers remaining enabled.
  On PACMod3, the wipers and headlights would remain enabled on supported
  vehicles when the disable button was pressed. This change makes sure
  that an additional message with enable = false is published when a
  disable occurs and on any button press thereafter until an enable
  occurs.
* Adding vehicle 6
* Only cycle between LOW and HIGH on vehicle_5 headlights.
* Limiting MAX_ROT_RAD for VEH_5 based on testing.
* Making accel_scale_val apply to all controllers.
* Removing need to populate redundant launch file param.
* Adding additional vehicle types.
* Changes for tuning steering on vehicle 4
* Corrected formatting & removed deadzone parameter in launch file
* Added a max rotation calibration value for the G29 steering wheel center-to-lock.
* Modify startup checks and launch file
  Prior to this commit we used two seperate vehicle types for the pacmod_game_control launch file. This commit compresses this down to one to avoid confusion.
* Modify buttons for windshield wipers and horn
  Prior to this commit the windshield wipers and horn conflicted with the enable button. This commit fixes this issue by moving these buttons.
* This commit adds horn and headlights to vehicle 5.
* Changing MAX_ROT_RANGE for vehicle 4
* Fixing regression with triggers as accel/brake - takeoff.
  During the reorganization of code into publish_control, a
  regression bug was introduced involving the joystick trigger
  initial values. This commit fixes that bug.
* PACMod 2 and PACMod 3 use different topics for enabled feedback.
  PACMod 2 used the topic as_tx/enable while PACMod 3 uses
  as_tx/enabled. This fixes this node for use with the enabled version.
* Removes state_change_debounce. No longer necessary.
  We really only care about the transition of PACMod from enabled
  to disabled and when this transition happens, it should not be
  ignored no matter how recently a state transition has happened
  in the game control node. This implements this logic.
* Adding Vehicle Type 5.
* Fixing threading problem with pacmod_enable.
  The pacmod_enable variable can be modified in two separate threads:
  the callback for joy (in the case of a user-initiated enable/disable)
  or the callback for the PACMod's enable status topic (in case of an
  override or other disable). This necessitates having a "local" copy
  of the enable state through the publishing process to keep from having
  to lock/unlock a mutex every time we need the current enable/disable
  state. This commit converts the "local_enable" variable to one that
  is attached to PublishControl and is only updated on a joy callback.
* Actually implement debounce counter.
  The variables for the enable/disable debounce count existed but
  the recent_state_change variable was never set. Whoops.
* Adding send clear override on first message after enable
  Before this commit, the clear_override flag was not set on any
  message being sent by PACMod game control, as it did not exist.
  After this commit, the game control node will set the clear_override
  flag to true on the first message after enable on each of the
  PACMod system messages.
* Revising launch file to maintain support for ROS Indigo
  Before this commit, the launch file made use of "eval" for arguments
  to create a boolean by comparing against the "pacmod_board_rev" argument.
  This boolean was used to launch the appropriate PACMod driver. After this
  commit, the board rev parameter still exists to be sent to the game control
  node/factory, but there is an additional "is_pacmod_3" flag that needs to
  be set to launch the correct PACMod driver. ROS Indigo does not have support
  for "eval" in launch files.
* Simplifying invalid board exception message
  Before this commit, the exception message for an invalid board number
  selection was vague and provided no specific direction for remedy. After
  this commit, the requested board revision is returned in the error
  message, and the message describes where the problem lies.
* Adding Major Board Rev as Selector, unique_ptr, and board exception
  Before this commit, the PACMod board version was selected via a boolean in the
  launch file, and the factory class provided the publisher based on this boolean.
  Additionally, there was the potential for a memory leak when the factory class
  returned a raw pointer to the new publisher. After this commit, the PACMod
  board revision is denoted in the launch file as an integer (the major rev).
  This integer is used to evaluate booleans in the launch file, which then determine
  which PACMod driver version to launch. The major revision number is also used by
  the factory, allowing for extensibility, and the factory will also throw an
  invalid argument exception if an invalid board number is set. Finally, the raw
  pointer to the publisher has been replaced with a unique_ptr for memory
  management.
* Adding shift rpt and turn rpt subscribers for PM3
  Before this commit, pressing the enable/disable buttons on the
  controller would not send a turn command nor shift command with
  the enable flag set appropriately, resulting in the turn or shift
  system remaining enabled or disabled until the next regular command
  was sent. After this commit, hitting the enable or disable sends
  both turn and shift commands with the new flag set correctly.
* Adding message send on enable/disable logic
  Before this commit, pacmod_game_control would only send new messages when enabled,
  and would not send them ON enable or disable. After this commit, the node will
  send messages with the appropriate flag when the system is enabled OR when the
  enable state changes, either from disabled to enabled, or enabled to disabled.
* Adding publish control factory to return correct publish control version
  Before this commit, the publish control class for different boards had to be explicitly
  instantiated, resulting in one instance per board type needing to be instantiated.
  Currently, that would only be 2, but in the future, it could be many more. After this
  commit, the factory can be used to return an instance of any existing and future board
  types.
* Adding PACMod3 publishers with correct message type and launch file switch
  Before this commit, PACMod Game Control only worked with PACMod2 board revision. After this commit, there
  is a boolean parameter in the launch file (is_pacmod_3) to set the appropriate board rev and
  launch the associated pacmod driver. Additionally, the publish_control_board_rev3 class has been
  fleshed out with publish methods, as well as the correct message types.
* Adding enable/disable debounce for listening to PACMod feedback.
  When enabling or disabling via joystick, the global_enable variable
  can be inadvertently overwritten by a new global report being
  received with a stale value. This commit adds a debounce which causes the
  joystick application to stop listening to the global report for N messages
  after a state change.
* First pass - combining and creating framework.
  Moving many functions from PublishControlBoardRev2 to PublishControl
  because they are common between rev2 and rev3. Creating basic
  framework for PublishControlBoardRev3 and adding it to the build.
* Adding the AS::Joystick namespace to all files.
  Had to add some function declarations to incoude/startup_checks.h
  to complete the namespace addition.
* Add minor bug fixes
  Prior to this commit there were minor bugs in the code due to merging repos. This commit fixes bugs for gear shifting, throttle and brake.
* Re-add support for xbox and add support for LEXUS
  Prior to this commit there was no support for xbox controllers. Also the LEXUS constant was defined ambiguously. This commit adds support for xbox, and updates the lexus constant. It also removes some unused comments.
* Fix shifting bug and comment
  Prior to this commit there was a bug in the shifting logic due to legacy code. This commit fixes the logic and removes extra associated comments.
* Add constants for g29 to startup
  Prior to this commit the constants for the G29 control were not added. This commit adds those constants.
* Add fix for magic numbers and leftover comment
  Prior to this commit there were some magic numbers and a leftover TODO which was misleading. This commit fixes these issues.
* Adding Support for XBox One Controller
  This commit enables the use of the XBox One controller with pacmod_game_control.
  The XBox One controller uses the same button layout as the Logitech F310, which simplifies
  the code. Additionally, it should be noted that the XBox controller must be plugged
  into the computer using a USB to Micro USB cable.
* Add constants
  Prior to this commit we had used hard numbers instead of static constants. This commit replaces most of the hard numbers with static constants.
* Add formatting fixes
  Prior to this commit there were issues with formatting due to tabs. This commit fixes those issues.
* Add cleanup
  Prior to this commit the code was functional but not cleaned up or tested. This commit cleans up the code and gets it ready for merging.
* Add fix for callback issues
  Prior to this commit there were issues with the callbacks due to ROS context. This commit fixes these issues and cleans up the code.
* Fix errors in merge commit
  Prior to this commit there were some bugs introduced due to the merge commit. This commit resolves these bugs.
* Add class style restructing to code repo
  Prior to this commit we had not used classes to break up functionality and veriables. In this commit classes have been added to contain function calls specific to a certain board. Veriables have also been added to the class structure.
* Add class style restructing to code repo
  Prior to this commit we had not used classes to break up functionality and veriables. In this commit classes have been added to contain function calls specific to a certain board. Veriables have also been added to the class structure. The code does not currently compile but will be fixed in a amend commit.
* Add initial reorganization to refactor
  This commit reorganizes the pacmod game control code into seperate files and functions. It is designed to maintain functionality while breaking the code up into pieces to make it more readible.
* Contributors: Chris, Daniel-Stanek, Joe Driscoll, Joshua Whitley, Kyle Rector, Lucas Buckland, Nate Imig, Nishanth Samala, Samuel Rustan, Zach Oakes

2.0.0 (2018-05-14)
------------------
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
