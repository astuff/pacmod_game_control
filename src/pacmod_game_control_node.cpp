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

#include <stdio.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <pacmod_msgs/PositionWithSpeed.h>
#include <pacmod_msgs/PacmodCmd.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>
#include <pacmod_msgs/SystemCmdBool.h>
#include <pacmod_msgs/SystemCmdFloat.h>
#include <pacmod_msgs/SystemCmdInt.h>
#include <pacmod_msgs/SteerSystemCmd.h>
#include <pacmod_msgs/SystemRptInt.h>

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

int steering_axis = -1;
int vehicle_type = -1;
int board_rev = -1;
int controller_type = -1;
double steering_max_speed = -1.0;
bool pacmod_enable;
std::mutex enable_mutex;
pacmod_msgs::VehicleSpeedRpt::ConstPtr last_speed_rpt = NULL;
std::mutex speed_mutex;
std::mutex shift_mutex;
std::mutex turn_mutex;
std::vector<float> last_axes;
std::vector<int> last_buttons;
double max_veh_speed = -1.0;
double accel_scale_val = 1.0;
double brake_scale_val = 1.0;
uint16_t wiper_state = 0;
uint16_t headlight_state = 0;
uint16_t last_shift_cmd = 2;
uint16_t last_turn_cmd = 1;

bool enable_accel = false;
bool enable_brake = false;
bool enable_shift = false;
bool enable_turn = false;

#define SHIFT_PARK 0
#define SHIFT_REVERSE 1
#define SHIFT_NEUTRAL 2
#define SHIFT_LOW 3
#define SHIFT_HIGH 4

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
 * Called when the node receives a message from the shift report
 */
void callback_shift_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg)
{
  shift_mutex.lock();
  // Store the latest value read from the gear state to be sent on enable/disable
  last_shift_cmd = msg->output;
  shift_mutex.unlock();
}

/*
 * Called when the node receives a message from the turn signal report
 */
void callback_turn_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg)
{
  turn_mutex.lock();
  // Store the latest value read from the gear state to be sent on enable/disable
  last_turn_cmd = msg->output;
  turn_mutex.unlock();
}

/*
 * Called when a game controller message is received
 */
void callback_joy(const sensor_msgs::Joy::ConstPtr& msg)
{
  bool local_enable = false;

  enable_mutex.lock();
  local_enable = pacmod_enable;
  enable_mutex.unlock();

  if (controller_type == 0)
  {
    std_msgs::Bool bool_pub_msg;

    // Enable
    if (msg->buttons[5] == 1)
    {
      local_enable = true;

      if (board_rev != 3)
      {
        enable_pub.publish(bool_pub_msg);
        std_msgs::Bool bool_pub_msg;
        bool_pub_msg.data = true;
      }
    }

    // Disable
    if (msg->buttons[4] == 1)
    { 
      local_enable = false;

      if (board_rev != 3)
      {
        bool_pub_msg.data = false;
        enable_pub.publish(bool_pub_msg);
      }
    }
  }
  else if (controller_type == 1)
  {  
    // Enable
    if (msg->axes[7] >= 0.9)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = true;
      local_enable = true;
      enable_pub.publish(bool_pub_msg);
    }

    // Disable
    if (msg->axes[7] <= -0.9)
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
  
  // Steering
  // Axis 0 is left thumbstick, axis 3 is right. Speed in rad/sec.
  // Same for both Logitech and HRI controllers
  float range_scale = (fabs(msg->axes[steering_axis]) * (1.0 - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB);
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

  if (board_rev == 3)
  {
    pacmod_msgs::SteerSystemCmd steer_msg;
    steer_msg.enable = local_enable;
    steer_msg.ignore_overrides = false;
    steer_msg.command = (range_scale * MAX_ROT_RAD) * msg->axes[steering_axis];
    steer_msg.rotation_rate = steering_max_speed * speed_scale;
    steering_set_position_with_speed_limit_pub.publish(steer_msg);
  }
  else
  {
    pacmod_msgs::PositionWithSpeed pub_msg1;
    pub_msg1.angular_position = (range_scale * MAX_ROT_RAD) * msg->axes[steering_axis];
    pub_msg1.angular_velocity_limit = steering_max_speed * speed_scale;
    steering_set_position_with_speed_limit_pub.publish(pub_msg1);
  }

   // Sends the last turn signal state back to the PACMod with the appropriate enable/disable flag
  // Only evaluated when last shift enable state does not match the local enable
  if (enable_turn != local_enable)
  {
    pacmod_msgs::SystemCmdInt turn_msg;
    turn_msg.enable = local_enable;
    turn_msg.ignore_overrides = false;
    turn_msg.command = last_turn_cmd;
    turn_signal_cmd_pub.publish(turn_msg);

    enable_turn = local_enable;
  }

  // Turn signal
  // Same for both Logitech and HRI controllers
  if (board_rev == 3)
  {
    pacmod_msgs::SystemCmdInt turn_msg;
    turn_msg.enable = local_enable;
    turn_msg.ignore_overrides = false;
    
    if (msg->axes[6] == 1.0)
      turn_msg.command = 2;
    else if (msg->axes[6] == -1.0)
      turn_msg.command = 0;
    else
      turn_msg.command = 1;

    if (controller_type == 0)
    {
      if (msg->axes[7] == -1.0)
        turn_msg.command = 3;
    }
    else if (controller_type == 1)
    {
      if (msg->axes[2] < 0.5)
        turn_msg.command = 3;
    }

    if (last_axes.empty() ||
       (last_axes[7] != msg->axes[7] ||
        last_axes[6] != msg->axes[6] ||
        last_axes[2] != msg->axes[2]))
      turn_signal_cmd_pub.publish(turn_msg);
  }
  else
  {
    pacmod_msgs::PacmodCmd turn_signal_cmd_pub_msg;
    
    if (msg->axes[6] == 1.0)
      turn_signal_cmd_pub_msg.ui16_cmd = 2;
    else if (msg->axes[6] == -1.0)
      turn_signal_cmd_pub_msg.ui16_cmd = 0;
    else
      turn_signal_cmd_pub_msg.ui16_cmd = 1;    

    // Hazard lights (both left and right turn signals)
    if (controller_type == 0)         
    {
      if (msg->axes[7] == -1.0)
        turn_signal_cmd_pub_msg.ui16_cmd = 3;
    }
    else if (controller_type == 1)
    {
      if(msg->axes[2] < -0.5)
        turn_signal_cmd_pub_msg.ui16_cmd = 3;
    }

    if (last_axes.empty() ||
       (last_axes[7] != msg->axes[7] ||
        last_axes[6] != msg->axes[6] ||
        last_axes[2] != msg->axes[2]))
    {
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
    }
  }

  // Sends the last read gear state back to the PACMod with the appropriate enable/disable flag
  // Only evaluated when last shift enable state does not match the local enable
  if (enable_shift != local_enable)
  {
    pacmod_msgs::SystemCmdInt shift_msg;
    shift_msg.enable = local_enable;
    shift_msg.ignore_overrides = false;
    shift_msg.command = last_shift_cmd;
    shift_cmd_pub.publish(shift_msg);

    enable_shift = local_enable;
  }

  // Shifting: reverse
  // Same for both Logitech and HRI controllers
  if(msg->buttons[1] == 1 &&
    (last_buttons.empty() ||
     last_buttons[1] != msg->buttons[1]))
  {
    if (board_rev == 3)
    {
      pacmod_msgs::SystemCmdInt shift_msg;
      shift_msg.enable = local_enable;
      shift_msg.ignore_overrides = false;
      shift_msg.command = SHIFT_REVERSE;
      shift_cmd_pub.publish(shift_msg);
    }
    else
    {
      pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
      shift_cmd_pub_msg.ui16_cmd = SHIFT_REVERSE;
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }
  }

  // Shifting: drive/low
  // Same for both Logitech and HRI controllers
  if (msg->buttons[0] == 1)
  {
    if (board_rev == 3)
    {
      pacmod_msgs::SystemCmdInt shift_msg;
      shift_msg.enable = local_enable;
      shift_msg.ignore_overrides = false;
      shift_msg.command = SHIFT_LOW;
      shift_cmd_pub.publish(shift_msg);
    }
    else
    {
      pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
      shift_cmd_pub_msg.ui16_cmd = SHIFT_LOW;
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }
  }

  // Shifting: high
  // Same for both Logitech and HRI controllers
  if (msg->buttons[6] == 1 &&
     (last_buttons.empty() ||
      last_buttons[6] != msg->buttons[6]))
  {
    if (board_rev == 3)
    {
      pacmod_msgs::SystemCmdInt shift_msg;
      shift_msg.enable = local_enable;
      shift_msg.ignore_overrides = false;
      shift_msg.command = SHIFT_HIGH;
      shift_cmd_pub.publish(shift_msg);
    }
    else
    {
      pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
      shift_cmd_pub_msg.ui16_cmd = SHIFT_HIGH;
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }
  }

  // Controller-specific Triggers
  if (controller_type == 0)
  {
    // Acelerator
    // Logitech right trigger (axis 5): not pressed = 1.0, fully pressed = -1.0
    if (msg->axes[5] != 0)
      enable_accel = true;

    if (board_rev == 3)
    {
      pacmod_msgs::SystemCmdFloat accel_msg;
      accel_msg.enable = local_enable;
      accel_msg.ignore_overrides = false;

      if (enable_accel)
      {
        if (vehicle_type == 2 || 
            vehicle_type == 4 ||
            vehicle_type == 5)
          accel_msg.command = (-0.5 * (msg->axes[5] - 1.0));
        else
          accel_msg.command = (-0.5 * (msg->axes[5] - 1.0)) * 0.6 + 0.21;
      }
      else
      {
        accel_msg.command = 0;
      }

      accelerator_cmd_pub.publish(accel_msg);
    }
    else
    {
      pacmod_msgs::PacmodCmd accelerator_cmd_pub_msg;

      if (enable_accel)
      {
        if (vehicle_type == 2 || 
            vehicle_type == 4 ||
            vehicle_type == 5)
          accelerator_cmd_pub_msg.f64_cmd = (-0.5*(msg->axes[5]-1.0));
        else
          accelerator_cmd_pub_msg.f64_cmd = (-0.5*(msg->axes[5]-1.0))*0.6+0.21;
      }
      else
      {
        accelerator_cmd_pub_msg.f64_cmd = 0;
      }

      accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
    }

    // Brake
    // Logitech left trigger (axis 2): not pressed = 1.0, fully pressed = -1.0
    if (msg->axes[2] != 0)
      enable_brake = true;

    if (board_rev == 3)
    {
      pacmod_msgs::SystemCmdFloat brake_msg;
      brake_msg.enable = local_enable;
      brake_msg.ignore_overrides = false;

      if (enable_brake)
        brake_msg.command = -((msg->axes[2] - 1.0) / 2.0) * brake_scale_val;
      else
        brake_msg.command = 0;

      brake_set_position_pub.publish(brake_msg);
    }
    else
    {
      pacmod_msgs::PacmodCmd pub_msg1;

      if (enable_brake)
      {
        pub_msg1.f64_cmd = -((msg->axes[2] - 1.0) / 2.0) * brake_scale_val;
      }
      else
      {
        pub_msg1.f64_cmd = 0;
      }

      brake_set_position_pub.publish(pub_msg1);    
    }

    // Shifting: park
    if (msg->buttons[3] == 1)
    {
      if (board_rev == 3)
      {
        pacmod_msgs::SystemCmdInt shift_msg;
        shift_msg.enable = local_enable;
        shift_msg.ignore_overrides = false;
        shift_msg.command = SHIFT_PARK;
        shift_cmd_pub.publish(shift_msg);
      }
      else
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_PARK;        
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }
    }

    // Shifting: neutral
    if (msg->buttons[2] == 1)
    {
      if (board_rev == 3)
      {
        pacmod_msgs::SystemCmdInt shift_msg;
        shift_msg.enable = local_enable;
        shift_msg.ignore_overrides = false;
        shift_msg.command = SHIFT_NEUTRAL;
        shift_cmd_pub.publish(shift_msg);
      }
      else
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_NEUTRAL;
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }
    }

    if (vehicle_type == 2)
    {
      // Headlights
      // TODO: Implement for HRI Controller
      if (msg->axes[7] == 1.0)
      {
        // Rotate through headlight states as button is pressed 
        headlight_state++;

        if (headlight_state >= NUM_HEADLIGHT_STATES)
          headlight_state = HEADLIGHT_STATE_START_VALUE;

        if (board_rev == 3)
        {
          pacmod_msgs::SystemCmdInt headlight_msg;
          headlight_msg.enable = local_enable;
          headlight_msg.ignore_overrides = false;
          headlight_msg.command = headlight_state;
          headlight_cmd_pub.publish(headlight_msg);
        }
        else
        {
          pacmod_msgs::PacmodCmd headlight_cmd_pub_msg;
          headlight_cmd_pub_msg.ui16_cmd = headlight_state;
          headlight_cmd_pub.publish(headlight_cmd_pub_msg);
        }
      }

      // Horn
      // TODO: Implement for HRI Controller
      if (board_rev == 3)
      {
        pacmod_msgs::SystemCmdBool horn_msg;
        horn_msg.enable = local_enable;
        horn_msg.ignore_overrides = false;
        
        if (msg->buttons[7] == 1)
          horn_msg.command = true;
        else
          horn_msg.command = false;

        horn_cmd_pub.publish(horn_msg);
      }
      else
      {
        pacmod_msgs::PacmodCmd horn_cmd_pub_msg;

        if(msg->buttons[7] == 1)
          horn_cmd_pub_msg.ui16_cmd = 1;
        else
          horn_cmd_pub_msg.ui16_cmd = 0;

        horn_cmd_pub.publish(horn_cmd_pub_msg);
      }
    }

    if (vehicle_type == 3) // Semi
    {
      // Windshield wipers
      // TODO: implement for HRI controller
      if (msg->axes[7] == 1.0)
      {
        // Rotate through wiper states as button is pressed 
        wiper_state++;

        if(wiper_state >= NUM_WIPER_STATES)
          wiper_state = WIPER_STATE_START_VALUE;

        if (board_rev == 3)
        {
          pacmod_msgs::SystemCmdInt wiper_msg;
          wiper_msg.enable = local_enable;
          wiper_msg.ignore_overrides = false;
          wiper_msg.command = wiper_state;
          wiper_cmd_pub.publish(wiper_msg);
        }
        else
        {
          pacmod_msgs::PacmodCmd wiper_cmd_pub_msg;
          wiper_cmd_pub_msg.ui16_cmd = wiper_state;
          wiper_cmd_pub.publish(wiper_cmd_pub_msg);
        }
      }
    }
  }
  else if (controller_type == 1)
  {
    // Accelerator
    // HRI right thumbstick vertical (axis 4): not pressed = 0.0, fully up = 1.0   
    if (msg->axes[4] >= 0.0)
    {
      // only consider center-to-up range as accelerator motion
      if (board_rev == 3)
      {
        pacmod_msgs::SystemCmdFloat accel_msg;
        accel_msg.enable = local_enable;
        accel_msg.ignore_overrides = false;
        accel_msg.command = accel_scale_val * (msg->axes[4]) * 0.6 + 0.21;
        accelerator_cmd_pub.publish(accel_msg);
      }
      else
      {
        pacmod_msgs::PacmodCmd accelerator_cmd_pub_msg;
        accelerator_cmd_pub_msg.f64_cmd = accel_scale_val * (msg->axes[4]) * 0.6 + 0.21;
        accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
      }
    }

    // Brake
    // HRI right thumbstick vertical (axis 4): not pressed = 0.0, fully down = -1.0
    if (board_rev == 3)
    {
      pacmod_msgs::SystemCmdFloat brake_msg;
      brake_msg.enable = local_enable;
      brake_msg.ignore_overrides = false;
      brake_msg.command = (msg->axes[4] > 0.0) ? 0.0 : -(brake_scale_val * msg->axes[4]);
      brake_set_position_pub.publish(brake_msg);
    }
    else
    {
      pacmod_msgs::PacmodCmd pub_msg1;
      pub_msg1.f64_cmd = (msg->axes[4] > 0.0) ? 0.0 : -(brake_scale_val * msg->axes[4]);
      brake_set_position_pub.publish(pub_msg1);    
    }

    // Shifting: park
    if (msg->buttons[2] == 1 &&
       (last_buttons.empty() ||
        last_buttons[2] != msg->buttons[2]))
    {
      if (board_rev == 3)
      {
        pacmod_msgs::SystemCmdInt shift_msg;
        shift_msg.enable = local_enable;
        shift_msg.ignore_overrides = false;
        shift_msg.command = SHIFT_PARK;
        shift_cmd_pub.publish(shift_msg);
      }
      else
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_PARK;        
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }
    }

    // Shifting: neutral
    if (msg->buttons[3] == 1 &&
       (last_buttons.empty() ||
        last_buttons[3] != msg->buttons[3]))
    {
      if (board_rev == 3)
      {
        pacmod_msgs::SystemCmdInt shift_msg;
        shift_msg.enable = local_enable;
        shift_msg.ignore_overrides = false;
        shift_msg.command = SHIFT_NEUTRAL;
        shift_cmd_pub.publish(shift_msg);
      }
      else
      {
        pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
        shift_cmd_pub_msg.ui16_cmd = SHIFT_NEUTRAL;
        shift_cmd_pub.publish(shift_cmd_pub_msg);
      }
    }
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
  
  // Axis 3 is right thumbstick, 0 is left thumbstick
  if (priv.getParam("steering_axis", steering_axis))
  {
    ROS_INFO("Got steering_axis: %d", steering_axis);

    if ((steering_axis != 0) &&
        (steering_axis!=3))
    {
      ROS_INFO("steering_axis is invalid");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter steering_axis is missing");
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
        vehicle_type != 4 &&
        vehicle_type != 5)
    {
      ROS_INFO("vehicle_type is invalid");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter vehicle_type is missing");
    willExit = true;
  }

  // Board Rev
  if (priv.getParam("board_rev", board_rev))
  {
    ROS_INFO("Got board rev: %u", board_rev);

    if (board_rev < 1 || board_rev > 3)
    {
      ROS_INFO("Got invalid board_rev. Assuming 2.");
      board_rev = 2;
    }
  }

  if (vehicle_type == 2)
    MAX_ROT_RAD = 6.5;

  if (vehicle_type == 4)
    MAX_ROT_RAD = 5.236;

  if (vehicle_type == 5)
    MAX_ROT_RAD = 8.378;

  // Controller type 0 is Logitech gamepad, type 1 is HRI controller
  if (priv.getParam("controller_type", controller_type))
  {
    ROS_INFO("Got controller_type: %d", controller_type);

    if (controller_type != 0 &&
        controller_type != 1)
    {
      ROS_INFO("controller_type is invalid");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter controller_type is missing");
    willExit = true;
  }
      
  if (priv.getParam("steering_max_speed", steering_max_speed))
  {
    ROS_INFO("Got steering_max_speed: %f", steering_max_speed);

    if (steering_max_speed <= 0)
    {
      ROS_INFO("steering_max_speed is invalid");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter steering_max_speed_scale_val is missing");
    willExit = true;
  }

  if (priv.getParam("max_veh_speed", max_veh_speed))
  {
    ROS_INFO("Got max_veh_speed: %f", max_veh_speed);

    if (max_veh_speed <= 0)
    {
      ROS_INFO("max_veh_speed is invalid");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter max_veh_speed is missing");
    willExit = true;
  }

  if (priv.getParam("accel_scale_val", accel_scale_val))
  {
    ROS_INFO("Got accel_scale_val: %f", accel_scale_val);

    if (accel_scale_val <= 0 ||
        accel_scale_val > 1.0)
    {
      ROS_INFO("accel_scale_val is invalid");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter accel_scale_val is missing");
    willExit = true;
  }

  if (priv.getParam("brake_scale_val", brake_scale_val))
  {
    ROS_INFO("Got brake_scale_val: %f", brake_scale_val);

    if (brake_scale_val <= 0 ||
        brake_scale_val > 1.0)
    {
      ROS_INFO("brake_scale_val is invalid");
      willExit = true;
    }
  }
  else
  {
    ROS_INFO("Parameter brake_scale_val is missing");
    willExit = true;
  }
  
  if (willExit)
      return 0;
                 
  // Subscribe to messages
  ros::Subscriber joy_sub = n.subscribe("joy", 1000, callback_joy);
  ros::Subscriber speed_sub = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, callback_veh_speed);
  ros::Subscriber enable_sub = n.subscribe("/pacmod/as_tx/enabled", 20, callback_pacmod_enable);
  ros::Subscriber shift_sub = n.subscribe("/pacmod/parsed_tx/shift_rpt", 20, callback_shift_rpt);
  ros::Subscriber turn_sub = n.subscribe("/pacmod/parsed_tx/turn_rpt", 20, callback_turn_rpt);
  
  // Advertise published messages
  enable_pub = n.advertise<std_msgs::Bool>("/pacmod/as_rx/enable", 20);

  if (board_rev == 3)
  {
    accelerator_cmd_pub = n.advertise<pacmod_msgs::SystemCmdFloat>("/pacmod/as_rx/accel_cmd", 20);
    brake_set_position_pub = n.advertise<pacmod_msgs::SystemCmdFloat>("/pacmod/as_rx/brake_cmd", 20);
    headlight_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/headlight_cmd", 20);
    horn_cmd_pub = n.advertise<pacmod_msgs::SystemCmdBool>("/pacmod/as_rx/horn_cmd", 20);
    shift_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/shift_cmd", 20);
    steering_set_position_with_speed_limit_pub = n.advertise<pacmod_msgs::SteerSystemCmd>("/pacmod/as_rx/steer_cmd", 20);
    turn_signal_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/turn_cmd", 20);
    wiper_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/wiper_cmd", 20);
  }
  else
  {
    accelerator_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/accel_cmd", 20);
    brake_set_position_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/brake_cmd", 20);
    headlight_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/headlight_cmd", 20);
    horn_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/horn_cmd", 20);
    shift_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/shift_cmd", 20);
    steering_set_position_with_speed_limit_pub = n.advertise<pacmod_msgs::PositionWithSpeed>("/pacmod/as_rx/steer_cmd", 20);
    turn_signal_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/turn_cmd", 20);
    wiper_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/wiper_cmd", 20);
  }

  spinner.start();

  ros::waitForShutdown();

  return 0;
}

