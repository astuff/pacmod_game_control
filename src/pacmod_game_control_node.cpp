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

#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <pacmod/position_with_speed.h>
#include <pacmod/pacmod_cmd.h>
#include <pacmod_defines.h>

ros::Publisher turn_signal_cmd_pub;
ros::Publisher shift_cmd_pub;
ros::Publisher accelerator_cmd_pub;
ros::Publisher steering_set_position_with_speed_limit_pub;
ros::Publisher brake_set_position_pub;
ros::Publisher override_pub;

const float MAX_ROT_RAD = 10.9956;
const float ROT_RANGE_SCALER_LB = 0.05;

bool pacmod_override;
std::vector<float> last_axes;
std::vector<int> last_buttons;

void callback_pacmod_override(const std_msgs::Bool::ConstPtr& msg) {
    pacmod_override = msg->data;
} 

void callback_joy(const sensor_msgs::Joy::ConstPtr& msg) {
    std_msgs::Bool bool_pub_msg;
    std_msgs::Int16 int16_pub_msg;
    std_msgs::Float64 float64_pub_msg;
    bool axes_empty = last_axes.empty();
    bool buttons_empty = last_buttons.empty();

    if (!axes_empty && !buttons_empty &&
            std::equal(last_axes.begin(), last_axes.end(), msg->axes.begin()) &&
            std::equal(last_buttons.begin(), last_buttons.end(), msg->buttons.begin()))
    {
        return;
    }
  
    // Enable
    if(msg->buttons[5] == 1 && (buttons_empty || (last_buttons[5] != msg->buttons[5]))) {
        std_msgs::Bool bool_pub_msg;
        bool_pub_msg.data=false;
        override_pub.publish(bool_pub_msg);
    }
  
    // Disable
    if(msg->buttons[4] == 1 && (buttons_empty || (last_buttons[4] != msg->buttons[4]))) { 
        std_msgs::Bool bool_pub_msg;
        bool_pub_msg.data=true;
        override_pub.publish(bool_pub_msg);
    }

    if (!pacmod_override)
    {
        // Steering
        if(axes_empty || (last_axes[3] != msg->axes[3])) { 
            pacmod::position_with_speed pub_msg1;
            float range_scale = (fabs(msg->axes[3]) * (1.0 - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB);
            pub_msg1.angular_position = -(range_scale * MAX_ROT_RAD) * msg->axes[3];
            pub_msg1.angular_velocity_limit = 4.71239;
            steering_set_position_with_speed_limit_pub.publish(pub_msg1);
        }
        
        // Brake
        if(axes_empty || (last_axes[2] != msg->axes[2])) {
            pacmod::pacmod_cmd pub_msg1;
            pub_msg1.f64_cmd = ((msg->axes[2] - 1.0) / 2.0);
            brake_set_position_pub.publish(pub_msg1);    
        }

        // Turn signal
        if(axes_empty || (last_axes[6] != msg->axes[6])) {
            pacmod::pacmod_cmd turn_signal_cmd_pub_msg;
            
            if(msg->axes[6] == 1.0) {
                turn_signal_cmd_pub_msg.ui16_cmd = 2;
            } else if(msg->axes[6] == -1.0) {
                turn_signal_cmd_pub_msg.ui16_cmd = 0;
            } else {
                turn_signal_cmd_pub_msg.ui16_cmd = 1;    
            }

            turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
        }
            
        // Shifting: forward
        if(msg->buttons[0] == 1 && (buttons_empty || (last_buttons[0] != msg->buttons[0]))) {
            pacmod::pacmod_cmd shift_cmd_pub_msg;
            shift_cmd_pub_msg.ui16_cmd = 0;        
            shift_cmd_pub.publish(shift_cmd_pub_msg);
        }

        // Shifting: neutral
        if(msg->buttons[2] == 1 && (buttons_empty || (last_buttons[2] != msg->buttons[2]))) {
            pacmod::pacmod_cmd shift_cmd_pub_msg;
            shift_cmd_pub_msg.ui16_cmd = 1;        
            shift_cmd_pub.publish(shift_cmd_pub_msg);
        }
        
        // Shifting: reverse
        if(msg->buttons[1] == 1 && (buttons_empty || (last_buttons[1] != msg->buttons[1]))) {
            pacmod::pacmod_cmd shift_cmd_pub_msg;
            shift_cmd_pub_msg.ui16_cmd = 2;        
            shift_cmd_pub.publish(shift_cmd_pub_msg);
        }
          
        // Shifting: park
        if(msg->buttons[3] == 1 && (buttons_empty || (last_buttons[3] != msg->buttons[3]))) {
            // TODO
        }

        // Accelerator  
        if(axes_empty || (last_axes[5] != msg->axes[5])) { 
            pacmod::pacmod_cmd accelerator_cmd_pub_msg;
            ROS_INFO("Raw value: %f", msg->axes[5]);
            accelerator_cmd_pub_msg.f64_cmd = (-0.5*(msg->axes[5]-1.0))*0.6+0.21;
            ROS_INFO("Calculated accel value: %f", accelerator_cmd_pub_msg.f64_cmd);
            accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
        }
    }

    last_axes.clear();
    last_buttons.clear();
    last_axes.insert(last_axes.end(), msg->axes.begin(), msg->axes.end());
    last_buttons.insert(last_buttons.end(), msg->buttons.begin(), msg->buttons.end());
}  

int main(int argc, char *argv[]) { 
    ros::init(argc, argv, "pacmod_gamepad_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(1.0/0.01);
        
    // Subscribe to messages
    ros::Subscriber joy_sub = n.subscribe("/pacmod/joy", 1000, callback_joy);
    ros::Subscriber override_sub = n.subscribe("as_tx/override", 20, callback_pacmod_override);
    
    // Advertise published messages
    override_pub = n.advertise<std_msgs::Bool>("as_rx/override", 20);
    turn_signal_cmd_pub = n.advertise<pacmod::pacmod_cmd>("as_rx/turn_cmd", 20);
    shift_cmd_pub = n.advertise<pacmod::pacmod_cmd>("as_rx/shift_cmd", 20);
    accelerator_cmd_pub = n.advertise<pacmod::pacmod_cmd>("as_rx/accel_cmd", 20);
    steering_set_position_with_speed_limit_pub = n.advertise<pacmod::position_with_speed>("as_rx/steer_cmd", 20);
    brake_set_position_pub = n.advertise<pacmod::pacmod_cmd>("as_rx/brake_cmd", 20);
                  
    while(ros::ok()) {   
        // Wait for next loop
        loop_rate.sleep();
        ros::spinOnce(); 
    }
      
    return 0;
}

