/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

/*
HRI joystick mappings as found by ROS Kinetic Joy node on Ubuntu 16.04
Last modified 4-7-2017 by Joe Driscoll

Left thumbstick:
*Axis 0 is left (+1) to right (-1), centered=0.0
*Axis 1 is up (+1) to down (-1), centered=0.0

Triggers:
*Axis 2 is left trigger: up=1.0, center=0.0, down=-1.0
*Axis 5 is right trigger: up=1.0, center=0.0, down=-1.0

Right thumbstick:
*Axis 3 is left (+1) to right (-1), centered=0.0
*Axis 4 is up (+1) to down (-1), centered=0.0

Arrow buttons:
*Axis 6 is left (+1.0) and right (-1.0) arrow buttons, not pressed = 0.0
*Axis 7 is up (+1.0) and down (-1.0) arrow buttons, not pressed = 0.0

Number buttons:
"1" button pressed = button 0 = 1, not pressed = 0
"2" button pressed = button 1 = 1, not pressed = 0
"3" button pressed = button 2 = 1, not pressed = 0
"4" button pressed = button 3 = 1, not pressed = 0
*/

#include "publish_control_board_rev2.h"
#include "globals.h"
#include "startup_checks.h"

/*
 * Main method running the ROS Node
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pacmod_gamepad_control");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::Rate loop_rate(2.0);
  publish_control publish_control_class;

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);
  
  if(run_startup_checks_ok(&priv, &publish_control_class) == false)
    return 0;

  callback boundVehSpeedCallback = boost::bind(publish_control::callback_veh_speed, &publish_control_cast, _1);
  
  // Subscribe to generic messages
  //publish_control_class.subscribe_vehicle_speed( &n );
  ros::Subscriber speed_sub = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, 
                                            &publish_control::callback_veh_speed);
  ros::Subscriber enable_sub = n.subscribe("/pacmod/as_tx/enable", 20, 
                                            publish_control_class.callback_pacmod_enable);
  
  if(publish_control_class.board_rev == 2)
  {
    // construct child class
    publish_control_board_rev2 publish_control_class_board_rev2(&publish_control_class);
    
    // Subcribe to board specific messages
    ros::Subscriber joy_sub = n.subscribe("joy", 1000, publish_control_board_rev2.publish_control_messages);
    
    // TODO : we can add this into the publish_control_board_rev2 class 
    // Advertise published messages
    enable_pub = n.advertise<std_msgs::Bool>("/pacmod/as_rx/enable", 20);
    turn_signal_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/turn_cmd", 20);
    headlight_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/headlight_cmd", 20);
    horn_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/horn_cmd", 20);
    wiper_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/wiper_cmd", 20);
    shift_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/shift_cmd", 20);
    accelerator_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/accel_cmd", 20);
    steering_set_position_with_speed_limit_pub = n.advertise<pacmod_msgs::PositionWithSpeed>("/pacmod/as_rx/steer_cmd", 20);
    brake_set_position_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/brake_cmd", 20);
  }
  /*else if(board_rev == 3)
  {
    // construct child class
    publish_control_board_rev3 publish_control_class_board_rev3(&publish_control_class);
    
    // Subcribe to board specific messages
    ros::Subscriber joy_sub = n.subscribe("joy", 1000, publish_control_board_rev3.publish_control_messages);
    
    // TODO : we can add this into the publish_control_board_rev2 class  
    // Advertise published messages
    // ... 
    // ... 
    // ...
  }*/

  spinner.start();

  ros::waitForShutdown();

  return 0;
}

