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

#include <cstdio>
#include <mutex>
#include <unordered_map>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <pacmod_msgs/PositionWithSpeed.h>
#include <pacmod_msgs/PacmodCmd.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>

ros::Publisher turn_signal_cmd_pub;
ros::Publisher headlight_cmd_pub;
ros::Publisher horn_cmd_pub;
ros::Publisher wiper_cmd_pub;
ros::Publisher shift_cmd_pub;
ros::Publisher accelerator_cmd_pub;
ros::Publisher steering_set_position_with_speed_limit_pub;
ros::Publisher brake_set_position_pub;
ros::Publisher enable_pub;

float MAX_ROT_RAD = 10.9956;
const float ROT_RANGE_SCALER_LB = 0.05;
const uint16_t NUM_WIPER_STATES = 8;
const uint16_t WIPER_STATE_START_VALUE = 0;
const uint16_t NUM_HEADLIGHT_STATES = 3;
const uint16_t HEADLIGHT_STATE_START_VALUE = 0;

enum ShiftState
{
  SHIFT_PARK = 0,
  SHIFT_REVERSE = 1,
  SHIFT_NEUTRAL = 2,
  SHIFT_LOW = 3,
  SHIFT_HIGH = 4
};

enum GamepadType
{
  LOGITECH_F310,
  HRI_SAFE_REMOTE,
  LOGITECH_G29,
  NINTENDO_SWITCH_WIRED_PLUS
};

enum JoyAxis
{
  LEFT_STICK_UD,
  LEFT_STICK_LR,
  RIGHT_STICK_UD,
  RIGHT_STICK_LR,
  DPAD_UD,
  DPAD_LR,
  LEFT_TRIGGER_AXIS,   // Sometimes button, sometimes axis
  RIGHT_TRIGGER_AXIS   // Sometimes button, sometimes axis
};

enum JoyButton
{
  TOP_BTN,
  LEFT_BTN,
  BOTTOM_BTN,
  RIGHT_BTN,
  LEFT_BUMPER,
  RIGHT_BUMPER,
  BACK_SELECT_MINUS,
  START_PLUS,
  LEFT_TRIGGER_BTN,   // Sometimes button, sometimes axis
  RIGHT_TRIGGER_BTN,  // Sometimes button, sometimes axis
  LEFT_STICK_PUSH,
  RIGHT_STICK_PUSH
};

struct EnumHash
{
  template <typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

std::unordered_map<JoyAxis, int, EnumHash> axes;
std::unordered_map<JoyButton, int, EnumHash> btns;

int vehicle_type = -1;
GamepadType controller = LOGITECH_F310;
JoyAxis steering_axis = LEFT_STICK_LR;
double steering_max_speed = -1.0;
bool pacmod_enable;
std::mutex enable_mutex;
pacmod_msgs::VehicleSpeedRpt::ConstPtr last_speed_rpt = NULL;
std::mutex speed_mutex;
std::vector<float> last_axes;
std::vector<int> last_buttons;
double max_veh_speed = -1.0;
double accel_scale_val = 1.0;
double brake_scale_val = 1.0;
uint16_t wiper_state = 0;
uint16_t headlight_state = 0;

bool enable_accel = false;
bool enable_brake = false;

/*
 * Called when the node receives a message from the enable topic
 */
void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  enable_mutex.lock();
  pacmod_enable = msg->data;
  enable_mutex.unlock();
} 

/*
 * Called when the node receives a message from the vehicle speed topic
 */
void callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  speed_mutex.lock();
  last_speed_rpt = msg;
  speed_mutex.unlock();
}

/*
 * Called when a game controller message is received
 */
void callback_joy(const sensor_msgs::Joy::ConstPtr& msg)
{
  try
  {
    bool local_enable = false;

    enable_mutex.lock();
    local_enable = pacmod_enable;
    enable_mutex.unlock();

    if (controller == HRI_SAFE_REMOTE)
    {  
      // Enable
      if (msg->axes[axes[DPAD_UD]] >= 0.9)
      {
        std_msgs::Bool bool_pub_msg;
        bool_pub_msg.data = true;
        local_enable = true;
        enable_pub.publish(bool_pub_msg);
      }

      // Disable
      if (msg->axes[axes[DPAD_UD]] <= -0.9)
      {
        std_msgs::Bool bool_pub_msg;
        bool_pub_msg.data = false;
        local_enable = false;
        enable_pub.publish(bool_pub_msg);
      }    
    }
    else
    {
      // Enable
      if (msg->buttons[btns[RIGHT_BUMPER]] == 1)
      {
      
        std_msgs::Bool bool_pub_msg;
        bool_pub_msg.data = true;
        local_enable = true;
        enable_pub.publish(bool_pub_msg);
      }

      // Disable
      if (msg->buttons[btns[LEFT_BUMPER]] == 1)
      { 
        std_msgs::Bool bool_pub_msg;
        bool_pub_msg.data = false;
        local_enable = false;
        enable_pub.publish(bool_pub_msg);
      }
    }

    enable_mutex.lock();
    pacmod_enable = local_enable;
    enable_mutex.unlock();
    
    if (local_enable)
    {
      // Steering
      // Axis 0 is left thumbstick, axis 3 is right. Speed in rad/sec.
      pacmod_msgs::PositionWithSpeed steer_msg;
      
      float range_scale;

      range_scale = (fabs(msg->axes[axes[steering_axis]]) * (1.0 - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB);

      float speed_scale = 1.0;
      bool speed_valid = false;
      float current_speed = 0.0;

      speed_mutex.lock();

      if (last_speed_rpt != NULL)
        speed_valid = last_speed_rpt->vehicle_speed_valid;

      if (speed_valid)
        current_speed = last_speed_rpt->vehicle_speed;

      speed_mutex.unlock();

      if (speed_valid)
        speed_scale = 1.0 - fabs((current_speed / (max_veh_speed * 1.5))); //Never want to reach 0 speed scale.

      steer_msg.angular_position = (range_scale * MAX_ROT_RAD) * msg->axes[axes[steering_axis]];

      steer_msg.angular_velocity_limit = steering_max_speed * speed_scale;
      steering_set_position_with_speed_limit_pub.publish(steer_msg);

      // Turn signal
      pacmod_msgs::PacmodCmd turn_signal_cmd_pub_msg;
      
      if (msg->axes[axes[DPAD_LR]] == 1.0)
        turn_signal_cmd_pub_msg.ui16_cmd = 2;
      else if (msg->axes[axes[DPAD_LR]] == -1.0)
        turn_signal_cmd_pub_msg.ui16_cmd = 0;
      else
        turn_signal_cmd_pub_msg.ui16_cmd = 1;    

      // Hazard lights (both left and right turn signals)
      if (controller == HRI_SAFE_REMOTE)
      {
        if(msg->axes[2] < -0.5)
          turn_signal_cmd_pub_msg.ui16_cmd = 3;

        if (last_axes.empty() ||
            last_axes[2] != msg->axes[2])
          turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
      }
      else
      {
        if (msg->axes[axes[DPAD_UD]] == -1.0)
          turn_signal_cmd_pub_msg.ui16_cmd = 3;

        if (last_axes.empty() ||
            last_axes[axes[DPAD_LR]] != msg->axes[axes[DPAD_LR]] ||
            last_axes[axes[DPAD_UD]] != msg->axes[axes[DPAD_UD]])
        {
          turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
        }
      }


      // Shifting: reverse
      if (msg->buttons[btns[RIGHT_BTN]] == 1)
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_REVERSE;
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }

      // Shifting: drive/low
      if (msg->buttons[btns[BOTTOM_BTN]] == 1)
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_LOW;
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }

      // Shifting: park
      if (msg->buttons[btns[TOP_BTN]] == 1)
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_PARK;        
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }

      // Shifting: neutral
      if (msg->buttons[btns[LEFT_BTN]] == 1)
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_NEUTRAL;
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }

      /* TODO: What??
      // Shifting: high
      if (msg->buttons[6] == 1 &&
         (last_buttons.empty() ||
          last_buttons[6] != msg->buttons[6]))
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_HIGH;
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }
      */
      
      // Acelerator
      pacmod_msgs::PacmodCmd accelerator_cmd_pub_msg;

      if (controller == HRI_SAFE_REMOTE)
      {
        // Accelerator
        if (msg->axes[axes[RIGHT_STICK_UD]] >= 0.0)
        {
          // only consider center-to-up range as accelerator motion
          accelerator_cmd_pub_msg.f64_cmd = accel_scale_val * (msg->axes[axes[RIGHT_STICK_UD]]) * 0.6 + 0.21;
        }
      }
      else
      {
        if (msg->axes[axes[RIGHT_TRIGGER_AXIS]] != 0)
          enable_accel = true;

        if (enable_accel)
        {
          if ((vehicle_type == 2) ||
              (vehicle_type == 4))
            accelerator_cmd_pub_msg.f64_cmd = (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0));
          else
            accelerator_cmd_pub_msg.f64_cmd = (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0)) * 0.6 + 0.21;
        }
        else
        {
          accelerator_cmd_pub_msg.f64_cmd = 0;
        }
      }

      accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);

      // Brake
      pacmod_msgs::PacmodCmd brake_msg;

      if (controller == HRI_SAFE_REMOTE)
      {
        brake_msg.f64_cmd = (msg->axes[axes[RIGHT_STICK_UD]] > 0.0) ? 0.0 : -(brake_scale_val * msg->axes[4]);
      }
      else
      {
        if (msg->axes[axes[LEFT_TRIGGER_AXIS]] != 0)
          enable_brake = true;

        if (enable_brake)
        {
          brake_msg.f64_cmd = -((msg->axes[axes[LEFT_TRIGGER_AXIS]] - 1.0) / 2.0) * brake_scale_val;
        }
        else
        {
          brake_msg.f64_cmd = 0;
        }
      }

      brake_set_position_pub.publish(brake_msg);    

      if (vehicle_type == 2 && controller != HRI_SAFE_REMOTE)
      {
        // Headlights
        if (msg->axes[axes[DPAD_UD]] == 1.0)
        {
          // Rotate through headlight states as button is pressed 
          headlight_state++;

          if(headlight_state >= NUM_HEADLIGHT_STATES)
            headlight_state = HEADLIGHT_STATE_START_VALUE;

          pacmod_msgs::PacmodCmd headlight_cmd_pub_msg;
          headlight_cmd_pub_msg.ui16_cmd = headlight_state;
          headlight_cmd_pub.publish(headlight_cmd_pub_msg);
        }

        // Horn
        pacmod_msgs::PacmodCmd horn_cmd_pub_msg;

        if (msg->buttons[7] == 1)
          horn_cmd_pub_msg.ui16_cmd = 1;
        else
          horn_cmd_pub_msg.ui16_cmd = 0;

        horn_cmd_pub.publish(horn_cmd_pub_msg);
      }

      if (vehicle_type == 3 && controller != HRI_SAFE_REMOTE) // Semi
      {
        // Windshield wipers
        if (msg->axes[7] == 1.0)
        {
          // Rotate through wiper states as button is pressed 
          wiper_state++;

          if(wiper_state >= NUM_WIPER_STATES)
            wiper_state = WIPER_STATE_START_VALUE;

          pacmod_msgs::PacmodCmd wiper_cmd_pub_msg;
          wiper_cmd_pub_msg.ui16_cmd = wiper_state;
          wiper_cmd_pub.publish(wiper_cmd_pub_msg);
        }
      }
    }
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR("An out-of-range exception was caught. This probably means you selected the wrong controller type.");
  }

  last_buttons.clear();
  last_buttons.insert(last_buttons.end(), msg->buttons.begin(), msg->buttons.end());
  last_axes.clear();
  last_axes.insert(last_axes.end(), msg->axes.begin(), msg->axes.end());
}  

/*
 * Main method running the ROS Node
 */
int main(int argc, char *argv[])
{ 
  bool willExit = false;
  ros::init(argc, argv, "pacmod_gamepad_control");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::Rate loop_rate(2.0);

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);
  
  std::string steering_stick_string;

  if (priv.getParam("steering_stick", steering_stick_string))
  {
    ROS_INFO("Got steering_stick: %s", steering_stick_string.c_str());

    if (steering_stick_string == "LEFT")
    {
      steering_axis = LEFT_STICK_LR;
    }
    else if (steering_stick_string == "RIGHT")
    {
      steering_axis = RIGHT_STICK_LR;
    }
    else
    {
      ROS_INFO("steering_stick is invalid. Exiting.");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter steering_stick is missing. Exiting.");
    willExit = true;
  }

  // Vehicle type 0 is Polaris GEM, type 1 is Polaris Ranger, type 3 is semi
  if (priv.getParam("vehicle_type", vehicle_type))
  {
    ROS_INFO("Got vehicle_type: %d", vehicle_type);

    if (vehicle_type != 0 &&
        vehicle_type != 1 &&
        vehicle_type != 2 &&
        vehicle_type != 3 &&
        vehicle_type != 4)
    {
      ROS_INFO("vehicle_type is invalid");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter vehicle_type is missing. Exiting.");
    willExit = true;
  }

  if (vehicle_type == 2)
    MAX_ROT_RAD = 6.5;

  if (vehicle_type == 4)
    MAX_ROT_RAD = 5.236;

  std::string controller_string;

  if (priv.getParam("controller_type", controller_string))
  {
    ROS_INFO("Got controller_type: %s", controller_string.c_str());

    if (controller_string == "LOGITECH_F310")
    {
      controller = LOGITECH_F310;

      axes[LEFT_STICK_LR] = 0;
      axes[LEFT_TRIGGER_AXIS] = 2;
      axes[RIGHT_STICK_LR] = 3;
      axes[RIGHT_TRIGGER_AXIS] = 5;
      axes[DPAD_LR] = 6;
      axes[DPAD_UD] = 7;
      axes[LEFT_STICK_UD] = 1;
      axes[RIGHT_STICK_UD] = 4;

      btns[BOTTOM_BTN] = 0;
      btns[RIGHT_BTN] = 1;
      btns[LEFT_BTN] = 2;
      btns[TOP_BTN] = 3;
      btns[LEFT_BUMPER] = 4;
      btns[RIGHT_BUMPER] = 5;
      btns[BACK_SELECT_MINUS] = 6;
      btns[START_PLUS] = 7;
      btns[LEFT_STICK_PUSH] = 9;
      btns[RIGHT_STICK_PUSH] = 10;
    }
    else if (controller_string == "HRI_SAFE_REMOTE")
    {
      controller = HRI_SAFE_REMOTE;

      // TODO: Complete missing buttons
      axes[LEFT_STICK_LR] = 0;
      axes[RIGHT_STICK_LR] = 3;
      axes[RIGHT_STICK_UD] = 4;
      axes[DPAD_LR] = 6;
      axes[DPAD_UD] = 7;

      btns[BOTTOM_BTN] = 0;
      btns[RIGHT_BTN] = 1;
      btns[TOP_BTN] = 2;
      btns[LEFT_BTN] = 3;
    }
    else if (controller_string == "LOGITECH_G29")
    {
      controller = LOGITECH_G29;
      // TODO: Complete missing buttons
    }
    else if (controller_string == "NINTENDO_SWITCH_WIRED_PLUS")
    {
      controller = NINTENDO_SWITCH_WIRED_PLUS;

      axes[LEFT_STICK_LR] = 0;
      axes[LEFT_STICK_UD] = 1;
      axes[RIGHT_STICK_LR] = 2;
      axes[RIGHT_STICK_UD] = 3;
      axes[DPAD_LR] = 4;
      axes[DPAD_UD] = 5;

      btns[LEFT_BTN] = 0;
      btns[BOTTOM_BTN] = 1;
      btns[RIGHT_BTN] = 2;
      btns[TOP_BTN] = 3;
      btns[LEFT_BUMPER] = 4;
      btns[RIGHT_BUMPER] = 5;
      btns[LEFT_TRIGGER_BTN] = 6;
      btns[RIGHT_TRIGGER_BTN] = 7;
      btns[BACK_SELECT_MINUS] = 8;
      btns[START_PLUS] = 9;
      btns[LEFT_STICK_PUSH] = 10;
      btns[RIGHT_STICK_PUSH] = 11;
    }
    else
    {
      ROS_INFO("Provided controller_type is invalid. Exiting.");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter controller_type is missing. Exiting.");
    willExit = true;
  }
      
  if (priv.getParam("steering_max_speed", steering_max_speed))
  {
    ROS_INFO("Got steering_max_speed: %f", steering_max_speed);

    if (steering_max_speed <= 0)
    {
      ROS_INFO("Parameter steering_max_speed is invalid. Exiting.");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter steering_max_speed_scale_val is missing. Exiting.");
    willExit = true;
  }

  if (priv.getParam("max_veh_speed", max_veh_speed))
  {
    ROS_INFO("Got max_veh_speed: %f", max_veh_speed);

    if (max_veh_speed <= 0)
    {
      ROS_INFO("Parameter max_veh_speed is invalid. Exiting.");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter max_veh_speed is missing. Exiting.");
    willExit = true;
  }

  if (priv.getParam("accel_scale_val", accel_scale_val))
  {
    ROS_INFO("Got accel_scale_val: %f", accel_scale_val);

    if (accel_scale_val <= 0 ||
        accel_scale_val > 1.0)
    {
      ROS_INFO("Parameter accel_scale_val is invalid. Exiting.");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter accel_scale_val is missing. Exiting.");
    willExit = true;
  }

  if (priv.getParam("brake_scale_val", brake_scale_val))
  {
    ROS_INFO("Got brake_scale_val: %f", brake_scale_val);

    if (brake_scale_val <= 0 ||
        brake_scale_val > 1.0)
    {
      ROS_INFO("Parameter brake_scale_val is invalid. Exiting.");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter brake_scale_val is missing. Exiting.");
    willExit = true;
  }
  
  if (willExit)
      return 0;
                 
  // Subscribe to messages
  ros::Subscriber joy_sub = n.subscribe("joy", 1000, callback_joy);
  ros::Subscriber speed_sub = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, callback_veh_speed);
  ros::Subscriber enable_sub = n.subscribe("/pacmod/as_tx/enable", 20, callback_pacmod_enable);
  
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

  spinner.start();

  ros::waitForShutdown();

  return 0;
}

