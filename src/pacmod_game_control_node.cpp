/*
* AutonomouStuff, LLC ("COMPANY") CONFIDENTIAL
* Unpublished Copyright (c) 2009-2016 AutonomouStuff, LLC, All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of COMPANY. The intellectual and technical concepts contained
* herein are proprietary to COMPANY and may be covered by U.S. and Foreign Patents, patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained
* from COMPANY.  Access to the source code contained herein is hereby forbidden to anyone except current COMPANY employees, managers or contractors who have executed
* Confidentiality and Non-disclosure agreements explicitly covering such access.
*
* The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code, which includes
* information that is confidential and/or proprietary, and is a trade secret, of  COMPANY.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE,
* OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF COMPANY IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
* LAWS AND INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
* TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
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
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <pacmod_msgs/PositionWithSpeed.h>
#include <pacmod_msgs/PacmodCmd.h>

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
int controller_type = -1;
double steering_max_speed = -1.0;
bool pacmod_enable;
std::mutex enable_mutex;
double veh_speed;
std::mutex speed_mutex;
std::vector<float> last_axes;
std::vector<int> last_buttons;
double max_veh_speed = -1.0;
double accel_scale_val = -1.0;
double brake_scale_val = -1.0;
uint16_t wiper_state = 0;
uint16_t headlight_state = 0;

#define SHIFT_PARK 0
#define SHIFT_REVERSE 1
#define SHIFT_NEUTRAL 2
#define SHIFT_LOW 3
#define SHIFT_HIGH 4

/*
 * Called when the node receives a message from the enable topic
 */
void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg) {
  enable_mutex.lock();
  pacmod_enable = msg->data;
  enable_mutex.unlock();
} 

void callback_veh_speed(const std_msgs::Float64::ConstPtr& msg)
{
  speed_mutex.lock();
  veh_speed = msg->data;
  speed_mutex.unlock();
}

/*
 * Called when a game controller message is received
 */
void callback_joy(const sensor_msgs::Joy::ConstPtr& msg) {
    std_msgs::Bool bool_pub_msg;
    std_msgs::Int16 int16_pub_msg;
    std_msgs::Float64 float64_pub_msg;
  
    // Enable
    if(controller_type == 0) {
        if(msg->buttons[5] == 1) {
            std_msgs::Bool bool_pub_msg;
            bool_pub_msg.data = true;
            enable_pub.publish(bool_pub_msg);
            enable_mutex.lock();
            pacmod_enable = true;
            enable_mutex.unlock();
        }
    } else if(controller_type == 1) {  
        if(msg->axes[7] >= 0.9) {
            std_msgs::Bool bool_pub_msg;
            bool_pub_msg.data = true;
            enable_pub.publish(bool_pub_msg);
            enable_mutex.lock();
            pacmod_enable = true;
            enable_mutex.unlock();
        }    
    }
    
    // Disable
    if(controller_type == 0) {
        if(msg->buttons[4] == 1) { 
            std_msgs::Bool bool_pub_msg;
            bool_pub_msg.data = false;
            enable_pub.publish(bool_pub_msg);
            enable_mutex.lock();
            pacmod_enable = false;
            enable_mutex.unlock();
        }
    } else if(controller_type==1) {  
        if(msg->axes[7] <= -0.9) {
            std_msgs::Bool bool_pub_msg;
            bool_pub_msg.data = false;
            enable_pub.publish(bool_pub_msg);
            enable_mutex.lock();
            pacmod_enable = false;
            enable_mutex.unlock();
        }    
    }

    bool enable;
    enable_mutex.lock();
    enable = pacmod_enable;
    enable_mutex.unlock();
    
    if (enable)
    {
        // Steering: axis 0 is left thumbstick, axis 3 is right. Speed in rad/sec.
        // Same for both Logitech and HRI controllers
        pacmod_msgs::PositionWithSpeed pub_msg1;
        float range_scale = (fabs(msg->axes[steering_axis]) * (1.0 - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB);
        float speed_scale = 1.0 - fabs((veh_speed / (max_veh_speed * 1.5))); //Never want to reach 0 speed scale.

        if (vehicle_type == 2)
        {
          pub_msg1.angular_position = (range_scale * MAX_ROT_RAD) * msg->axes[steering_axis];
        }
        else
        {
          pub_msg1.angular_position = -(range_scale * MAX_ROT_RAD) * msg->axes[steering_axis];
        }

        pub_msg1.angular_velocity_limit = steering_max_speed * speed_scale;
        steering_set_position_with_speed_limit_pub.publish(pub_msg1);
        
        // Brake
        if(controller_type == 0) {
          // Logitech left trigger (axis 2): not pressed = 1.0, fully pressed = -1.0
          pacmod_msgs::PacmodCmd pub_msg1;
          pub_msg1.f64_cmd = -((msg->axes[2] - 1.0) / 2.0);
          brake_set_position_pub.publish(pub_msg1);    
        } else if(controller_type == 1) {
          // HRI right thumbstick vertical (axis 4): not pressed = 0.0, fully down = -1.0
          pacmod_msgs::PacmodCmd pub_msg1;
          pub_msg1.f64_cmd = (msg->axes[4] > 0.0) ? 0.0 : (brake_scale_val * msg->axes[4]);
          brake_set_position_pub.publish(pub_msg1);    
        }

        // Turn signal
        // Same for both Logitech and HRI controllers
        pacmod_msgs::PacmodCmd turn_signal_cmd_pub_msg;
        
        if(msg->axes[6] == 1.0) {
            turn_signal_cmd_pub_msg.ui16_cmd = 2;
        } else if(msg->axes[6] == -1.0) {
            turn_signal_cmd_pub_msg.ui16_cmd = 0;
        } else {
            turn_signal_cmd_pub_msg.ui16_cmd = 1;    
        }

        // Hazard lights (both left and right turn signals)
        if(controller_type == 0) {         
          if(msg->axes[7] == -1.0) {
              turn_signal_cmd_pub_msg.ui16_cmd = 3;
          }
        } else if(controller_type == 1) {         
          if(msg->axes[2] < -0.5) {
              turn_signal_cmd_pub_msg.ui16_cmd = 3;
          }
        }

        if (last_axes.empty() || (last_axes[7] != msg->axes[7] || last_axes[6] != msg->axes[6] || last_axes[2] != last_axes[2]))
		turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);

        if(vehicle_type == 2)
        {
          // Headlights
          // TODO: implement for HRI controller
          if(controller_type == 0)
          {
            pacmod_msgs::PacmodCmd headlight_cmd_pub_msg;

            // Rotate through headlight states as button is pressed 
            if(msg->axes[7] == 1.0)
            {
              headlight_state++;
              if(headlight_state >= NUM_HEADLIGHT_STATES)
              {
                headlight_state = HEADLIGHT_STATE_START_VALUE;
              }
              headlight_cmd_pub_msg.ui16_cmd = headlight_state;
              headlight_cmd_pub.publish(headlight_cmd_pub_msg);
            }
          }

          if(controller_type == 0)
          {
            pacmod_msgs::PacmodCmd horn_cmd_pub_msg;
              if(msg->buttons[7] == 1)
              {
                horn_cmd_pub_msg.ui16_cmd = 1;
              }
              else
              {
                horn_cmd_pub_msg.ui16_cmd = 0;
              }
              horn_cmd_pub.publish(horn_cmd_pub_msg);
          }
        }
        
        if(vehicle_type == 3) { // semi
            // Windshield wipers
            // TODO: implement for HRI controller
            if(controller_type == 0) {
                pacmod_msgs::PacmodCmd wiper_cmd_pub_msg;

                // Rotate through wiper states as button is pressed 
                if(msg->axes[7] == 1.0) {
                    wiper_state++;
                    if(wiper_state >= NUM_WIPER_STATES) {
                        wiper_state = WIPER_STATE_START_VALUE;
                    }
                    wiper_cmd_pub_msg.ui16_cmd = wiper_state;
		            wiper_cmd_pub.publish(wiper_cmd_pub_msg);
                }
            }  
        }
        
        // Shifting: park
        if(controller_type==0) {
            if(msg->buttons[3] == 1) {
                pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
                shift_cmd_pub_msg.ui16_cmd = SHIFT_PARK;        
                shift_cmd_pub.publish(shift_cmd_pub_msg);
            }
        } else if(controller_type==1) {
            if(msg->buttons[2] == 1 && (last_buttons.empty() || last_buttons[2] != msg->buttons[2])) {
                pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
                shift_cmd_pub_msg.ui16_cmd = SHIFT_PARK;        
                shift_cmd_pub.publish(shift_cmd_pub_msg);
            }
        }

        // Shifting: reverse
        // Same for both Logitech and HRI controllers
        if(msg->buttons[1] == 1 && (last_buttons.empty() || last_buttons[1] != msg->buttons[1])) {
            pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
            shift_cmd_pub_msg.ui16_cmd = SHIFT_REVERSE;        
            shift_cmd_pub.publish(shift_cmd_pub_msg);
        }
        
        // Shifting: neutral
        if(controller_type == 0) {        
            if(msg->buttons[2] == 1) {
                pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
                shift_cmd_pub_msg.ui16_cmd = SHIFT_NEUTRAL;        
                shift_cmd_pub.publish(shift_cmd_pub_msg);
            }
        } else if(controller_type == 1) {                        
            if(msg->buttons[3] == 1 && (last_buttons.empty() || last_buttons[3] != msg->buttons[3])) {
                pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
                shift_cmd_pub_msg.ui16_cmd = SHIFT_NEUTRAL;        
                shift_cmd_pub.publish(shift_cmd_pub_msg);
            }
        }
                                       
        // Shifting: drive/low
        // Same for both Logitech and HRI controllers
        if(msg->buttons[0] == 1) {
            pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
            shift_cmd_pub_msg.ui16_cmd = SHIFT_LOW;        
            shift_cmd_pub.publish(shift_cmd_pub_msg);
        }

        // Shifting: high
        if(msg->buttons[6] == 1 && (last_buttons.empty() || last_buttons[6] != msg->buttons[6])) {
            pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
            shift_cmd_pub_msg.ui16_cmd = SHIFT_HIGH;        
            shift_cmd_pub.publish(shift_cmd_pub_msg);
        }

        // Accelerator  
        if(controller_type == 0) {
            // Logitech right trigger (axis 5): not pressed = 1.0, fully pressed = -1.0
            pacmod_msgs::PacmodCmd accelerator_cmd_pub_msg;
            //ROS_INFO("Raw value: %f", msg->axes[5]);
            if (vehicle_type == 2)
            {
              accelerator_cmd_pub_msg.f64_cmd = (-0.5*(msg->axes[5]-1.0));
            }
            else
            {
              accelerator_cmd_pub_msg.f64_cmd = (-0.5*(msg->axes[5]-1.0))*0.6+0.21;
            }

            //ROS_INFO("Calculated accel value: %f", accelerator_cmd_pub_msg.f64_cmd);
            accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
        } else if(controller_type == 1) {
            // HRI right thumbstick vertical (axis 4): not pressed = 0.0, fully up = 1.0   
            if(msg->axes[4] >= 0.0) {  // only consider center-to-up range as accelerator motion
              pacmod_msgs::PacmodCmd accelerator_cmd_pub_msg;
              ROS_INFO("Raw value: %f", msg->axes[4]);
              accelerator_cmd_pub_msg.f64_cmd = accel_scale_val * (msg->axes[4])*0.6+0.21;
              ROS_INFO("Calculated accel value: %f", accelerator_cmd_pub_msg.f64_cmd);
              accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
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
int main(int argc, char *argv[]) { 
    bool willExit = false;
    ros::init(argc, argv, "pacmod_gamepad_control");
    ros::AsyncSpinner spinner(2);
    ros::NodeHandle n;
    ros::NodeHandle priv("~");
    ros::Rate loop_rate(25.0);

    // Wait for time to be valid
    while (ros::Time::now().nsec == 0);
    
    // Axis 3 is right thumbstick, 0 is left thumbstick
    if (priv.getParam("steering_axis", steering_axis))
    {
        ROS_INFO("Got steering_axis: %d", steering_axis);
        if ((steering_axis!=0)&&(steering_axis!=3))
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
        if ((vehicle_type!=0)&&(vehicle_type!=1)&&(vehicle_type!=2)&&(vehicle_type!=3))
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

    if (vehicle_type == 2)
    {
        MAX_ROT_RAD = 6.5;
    }    

    // Controller type 0 is Logitech gamepad, type 1 is HRI controller
    if (priv.getParam("controller_type", controller_type))
    {
        ROS_INFO("Got controller_type: %d", controller_type);
        if ((controller_type!=0)&&(controller_type!=1))
        {
            ROS_INFO("steering_axis is invalid");
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
      if ( (accel_scale_val <= 0) || (accel_scale_val > 1.0) )
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
      if ( (brake_scale_val <= 0) || (brake_scale_val > 1.0) )
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
    ros::Subscriber speed_sub = n.subscribe("/pacmod/as_tx/vehicle_speed", 20, callback_veh_speed);
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
                  
    while(ros::ok()) {   
        // Wait for next loop
        loop_rate.sleep();
    }

    spinner.stop();
      
    return 0;
}

