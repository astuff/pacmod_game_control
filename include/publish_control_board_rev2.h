/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef PUBLISH_CONTROL_BOARD_REV2_H
#define PUBLISH_CONTROL_BOARD_REV2_H

#include "globals.h"
#include "publish_control.h"


class publish_control_board_rev2 : public publish_control
{
public:

  //publish_control_board_rev2(){};
  //publish_control_board_rev2(publish_control * publish_control_class);
  
  void publish_control_messages(const sensor_msgs::Joy::ConstPtr& msg);

private:

  // private functions
  bool check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg);

  std::vector<float> last_axes;
  std::vector<int> last_buttons;
};

#endif
