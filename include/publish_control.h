

#ifndef PUBLISH_CONTROL_H
#define PUBLISH_CONTROL_H

#include "globals.h"


class publish_control
{

public:
  // public functions
  virtual void publish_control_messages(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  static void callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg);
  static void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg);
  
  // public veriables 
  // TODO : add gettors and settors for these
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
  static std::mutex enable_mutex;
  static std::mutex speed_mutex;
  
    // private functions
  virtual bool check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  
  
private:

};

#endif
