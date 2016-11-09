#include <stdio.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"

#include "pacmod/pacmod_cmd.h"
#include "globe_epas/steering_position_with_speed.h"
#include "pacmod_defines.h"

ros::Publisher steering_set_command_mode_pub;
ros::Publisher steering_set_current_pub;
ros::Publisher steering_set_speed_pub;
ros::Publisher steering_set_position_pub;
ros::Publisher steering_set_position_with_speed_limit_pub;
ros::Publisher steering_set_inc_encoder_value_pub;

ros::Publisher brake_globe_set_command_mode_pub;
ros::Publisher brake_globe_set_current_pub;
ros::Publisher brake_globe_set_speed_pub;
ros::Publisher brake_globe_set_position_pub;
ros::Publisher brake_globe_set_position_with_speed_limit_pub;
ros::Publisher brake_globe_set_inc_encoder_value_pub;

ros::Publisher accelerator_cmd_pub;
ros::Publisher brake_globe_cmd_pub;
ros::Publisher turn_signal_cmd_pub;
ros::Publisher shift_cmd_pub;
ros::Publisher override_pub;

bool pacmod_override;
double last_axes_2=-999;
double last_axes_3=-999;
double last_axes_5=-999;
double last_axes_6=-999;
//int last_button_4 = -999;
//int last_button_5 = -999;

void callback_joy(const sensor_msgs::Joy::ConstPtr& msg) {
  std_msgs::Bool bool_pub_msg;
  std_msgs::Int16 int16_pub_msg;
  std_msgs::Float64 float64_pub_msg;
  
  // Enable        
 // if(msg->buttons[5]!=last_button_5) { 
   // last_button_5=msg->buttons[5];
    if(msg->buttons[5]==1) {    
      pacmod_override=false;
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data=pacmod_override;
      override_pub.publish(bool_pub_msg);
    }
  //}
  
  // Disable
 // if(msg->buttons[4]!=last_button_4) { 
  //  last_button_4=msg->buttons[4];  
    if(msg->buttons[4]==1) { 
      pacmod_override=true;
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data=pacmod_override;
      override_pub.publish(bool_pub_msg);
    }   
  //}
    
  // Steering -- Globe EPAS motor
  if(msg->axes[3]!=last_axes_3) { 
    last_axes_3=msg->axes[3];    
    if(!pacmod_override) { 
      globe_epas::steering_position_with_speed pub_msg1;
      pub_msg1.angular_position=-720.0*msg->axes[3];
      pub_msg1.speed_limit=180.0;//fabs(STEERING_SPEED_LIMIT*(msg->axes[3]));  // to help smooth the steering
      steering_set_position_with_speed_limit_pub.publish(pub_msg1);
    }
  }
      
  // Brake -- Globe EPAS motor
  if(msg->axes[2]!=last_axes_2) { 
    last_axes_2=msg->axes[2];
    if(!pacmod_override) {
      globe_epas::steering_position_with_speed pub_msg1;
      pub_msg1.angular_position=(-65.0*(msg->axes[2]-1.0)/2.0);
      pub_msg1.speed_limit=90.0;//fabs(BRAKE_GLOBE_SPEED_LIMIT*(msg->axes[2]));  // to help smooth the motion
      brake_globe_set_position_with_speed_limit_pub.publish(pub_msg1);    
    }
  }

  // Turn signal
  if(msg->axes[6]!=last_axes_6) {  
    last_axes_6=msg->axes[6];
    if(!pacmod_override) {
      pacmod::pacmod_cmd turn_signal_cmd_pub_msg;
      
      if((msg->axes[6])==1.0) {
        turn_signal_cmd_pub_msg.ui16_cmd=2;
      } else if((msg->axes[6])==(-1.0)) {
        turn_signal_cmd_pub_msg.ui16_cmd=0;
      } else {
        turn_signal_cmd_pub_msg.ui16_cmd=1;    
      } 
      
      turn_signal_cmd_pub_msg.enable=true;
      turn_signal_cmd_pub_msg.clear=true;
      turn_signal_cmd_pub_msg.ignore=false;  
      
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
    }
  }
      
  // Shifting: forward
  if(msg->buttons[0]==1) {
    if(!pacmod_override) {
      pacmod::pacmod_cmd shift_cmd_pub_msg;
      shift_cmd_pub_msg.ui16_cmd=0;        
      shift_cmd_pub_msg.enable=true;
      shift_cmd_pub_msg.clear=true;
      shift_cmd_pub_msg.ignore=false;  
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }
  }

  // Shifting: neutral
  if(msg->buttons[2]==1) {
    if(!pacmod_override) {
      pacmod::pacmod_cmd shift_cmd_pub_msg;
      shift_cmd_pub_msg.ui16_cmd=1;        
      shift_cmd_pub_msg.enable=true;
      shift_cmd_pub_msg.clear=true;
      shift_cmd_pub_msg.ignore=false;  
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }
  }  
  
  // Shifting: reverse
  if(msg->buttons[1]==1) {
    if(!pacmod_override) {
      pacmod::pacmod_cmd shift_cmd_pub_msg;
      shift_cmd_pub_msg.ui16_cmd=2;        
      shift_cmd_pub_msg.enable=true;
      shift_cmd_pub_msg.clear=true;
      shift_cmd_pub_msg.ignore=false;  
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }
  }
    
  // Shifting: park
  if(msg->buttons[3]==1) {
    if(!pacmod_override) {
      // TODO
    }
  }

  // Accelerator  
  if(msg->axes[5]!=last_axes_5) { 
    last_axes_5=msg->axes[5];
    if(!pacmod_override) {
      pacmod::pacmod_cmd accelerator_cmd_pub_msg;
      accelerator_cmd_pub_msg.f64_cmd=(-0.5*(msg->axes[5]-1.0))*0.6+0.21;
      accelerator_cmd_pub_msg.enable=true;
      accelerator_cmd_pub_msg.clear=true;
      accelerator_cmd_pub_msg.ignore=false;    
      accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
    }
  }
}  

int main(int argc, char *argv[]) { 
  ros::init(argc, argv, "pacmod_gamepad_control");
  ros::NodeHandle n;
  
  ros::Subscriber joy_sub = n.subscribe("/pacmod/joy", 1000, callback_joy);
  
  turn_signal_cmd_pub = n.advertise<pacmod::pacmod_cmd>("turn_signal/as_rx/set_cmd", 1000);
  shift_cmd_pub = n.advertise<pacmod::pacmod_cmd>("shift/as_rx/set_cmd", 1000);
  accelerator_cmd_pub = n.advertise<pacmod::pacmod_cmd>("accelerator/as_rx/set_cmd", 1000);
    
  override_pub = n.advertise<std_msgs::Bool>("as_rx/override", 1000, true);
        
  steering_set_current_pub = n.advertise<std_msgs::Float64>("steering/as_rx/set_current", 1000);
  steering_set_speed_pub = n.advertise<std_msgs::Float64>("steering/as_rx/set_speed", 1000);
  steering_set_position_pub = n.advertise<std_msgs::Float64>("steering/as_rx/set_position", 1000);
  steering_set_position_with_speed_limit_pub = n.advertise<globe_epas::steering_position_with_speed>("steering/as_rx/set_position_with_speed_limit", 1000);
  steering_set_inc_encoder_value_pub = n.advertise<std_msgs::Float64>("steering/as_rx/set_inc_encoder_value", 1000);

  brake_globe_set_current_pub = n.advertise<std_msgs::Float64>("brake/as_rx/set_current", 1000);
  brake_globe_set_speed_pub = n.advertise<std_msgs::Float64>("brake/as_rx/set_speed", 1000);
  brake_globe_set_position_pub = n.advertise<std_msgs::Float64>("brake/as_rx/set_position", 1000);
  brake_globe_set_position_with_speed_limit_pub = n.advertise<globe_epas::steering_position_with_speed>("brake/as_rx/set_position_with_speed_limit", 1000);
  brake_globe_set_inc_encoder_value_pub = n.advertise<std_msgs::Float64>("brake/as_rx/set_inc_encoder_value", 1000);
            
  ros::Rate loop_rate(20);
  while (ros::ok()) {   
    // Wait for next loop
    loop_rate.sleep();
    ros::spinOnce(); 
  }
    
  return 0;
}

