
#include "publish_control.h"

/*
 * Called when the node receives a message from the enable topic
 */
void publish_control::callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  enable_mutex.lock();
  pacmod_enable = msg->data;
  enable_mutex.unlock();
}

/*
 * Called when the node receives a message from the vehicle speed topic
 */
static void publish_control::callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  speed_mutex.lock();
  last_speed_rpt = msg;
  speed_mutex.unlock();
}

