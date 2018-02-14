
#include "publish_control.h"

JoyAxis publish_control::steering_axis = LEFT_STICK_LR;
float publish_control::max_rot_rad = 10.9956;
int publish_control::vehicle_type = -1;
GamepadType publish_control::controller = LOGITECH_F310;
int publish_control::board_rev = -1;
double publish_control::max_veh_speed = -1.0;
double publish_control::accel_scale_val = 1.0;
double publish_control::brake_scale_val = 1.0;
double publish_control::steering_max_speed = -1.0;
std::unordered_map<JoyAxis, int, EnumHash> publish_control::axes;
std::unordered_map<JoyButton, int, EnumHash> publish_control::btns;
pacmod_msgs::VehicleSpeedRpt::ConstPtr publish_control::last_speed_rpt = NULL;
bool publish_control::pacmod_enable;

/*
 * Called when the node receives a message from the enable topic
 */
void publish_control::callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("ENABLE FUNCTION");

  enable_mutex.lock();
  pacmod_enable = msg->data;
  enable_mutex.unlock();
}

/*
 * Called when the node receives a message from the vehicle speed topic
 */
void publish_control::callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  speed_mutex.lock();
  last_speed_rpt = msg;
  speed_mutex.unlock();
}

