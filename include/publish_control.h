

#ifndef PUBLISH_CONTROL_H
#define PUBLISH_CONTROL_H

#include "globals.h"


class publish_control
{

public:
  // public functions
  virtual void publish_control_messages(const sensor_msgs::Joy::ConstPtr& msg);
  static void callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg);
  void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg);
  
  // public veriables 
  // TODO : add gettors and settors for these
  JoyAxis steering_axis = LEFT_STICK_LR;
  float max_rot_rad = 10.9956;
  int vehicle_type = -1;
  GamepadType controller = LOGITECH_F310;
  int board_rev = -1;
  double max_veh_speed = -1.0;
  double accel_scale_val = 1.0;
  double brake_scale_val = 1.0;
  double steering_max_speed = -1.0;
  std::unordered_map<JoyAxis, int, EnumHash> axes;
  std::unordered_map<JoyButton, int, EnumHash> btns;
  static bool pacmod_enable;
  static std::mutex enable_mutex;
  static std::mutex speed_mutex;
  static pacmod_msgs::VehicleSpeedRpt::ConstPtr last_speed_rpt = NULL;
  
    // private functions
  virtual bool check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg);
  virtual void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg);
  virtual void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg);
  virtual void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg);
  virtual void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg);
  virtual void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg);
  virtual void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg);
  
  void subscribe_vehicle_speed( ros::NodeHandle * nh )
  {
    nh->subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, callback_veh_speed);
  }
  
  
private:

};

#endif
