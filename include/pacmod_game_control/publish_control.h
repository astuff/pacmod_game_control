/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H
#define PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H

#include "pacmod_game_control/globals.h"

#include <vector>

namespace AS
{
namespace Joystick
{

class PublishControl
{
  public:
    // public functions
    PublishControl();
    virtual void callback_control(const sensor_msgs::Joy::ConstPtr& msg);
    static void callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg);
    static void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg);

    // public variables
    static JoyAxis steering_axis;
    static float max_rot_rad;
    static int vehicle_type;
    static GamepadType controller;
    static int board_rev;
    static double max_veh_speed;
    static double accel_scale_val;
    static double brake_scale_val;
    static double steering_max_speed;
    static std::unordered_map<JoyAxis, int, EnumHash> axes;
    static std::unordered_map<JoyButton, int, EnumHash> btns;
    static pacmod_msgs::VehicleSpeedRpt::ConstPtr last_speed_rpt;
    static bool pacmod_enable;
    static bool prev_enable;
    static bool last_pacmod_state;
    static bool accel_0_rcvd;
    static bool brake_0_rcvd;
    static int headlight_state;
    static bool headlight_state_change;
    static int wiper_state;

  protected:
    virtual void check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg);

    // ROS node handle
    ros::NodeHandle n;

    // ROS publishers
    ros::Publisher turn_signal_cmd_pub;
    ros::Publisher rear_pass_door_cmd_pub;
    ros::Publisher headlight_cmd_pub;
    ros::Publisher horn_cmd_pub;
    ros::Publisher wiper_cmd_pub;
    ros::Publisher shift_cmd_pub;
    ros::Publisher accelerator_cmd_pub;
    ros::Publisher steering_set_position_with_speed_limit_pub;
    ros::Publisher brake_set_position_pub;
    ros::Publisher enable_pub;

    // ROS subscribers
    ros::Subscriber joy_sub;
    ros::Subscriber speed_sub;
    ros::Subscriber enable_sub;

    // state vectors
    std::vector<float> last_axes;
    std::vector<int> last_buttons;

    // Other Variables
    static bool local_enable;
    static bool recent_state_change;
    static uint8_t state_change_debounce_count;

  private:
    // private functions
    virtual void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
    virtual void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
    virtual void publish_rear_pass_door_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
    virtual void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
    virtual void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
    virtual void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
    virtual void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
};

}  // namespace Joystick
}  // namespace AS

#endif  // PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H
