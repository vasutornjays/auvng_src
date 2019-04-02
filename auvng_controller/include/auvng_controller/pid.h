#ifndef PID_H
#define PID_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <auvng_controller/pid_paramConfig.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <auvng_controller/Eulers.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>
#include <string>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#define PI (3.14159)
#define ANGLE_WRAP (2.0 * PI)   

extern double K_p[6];
extern double K_i[6];
extern double K_d[6];
extern double current_state[6];
extern double set_point[6];
extern double error[3][6];
extern double div_error[6];
extern double int_error[6];
extern double control_effort[6];
extern bool control_enabled;

// extern ros::Time prev_time_;
// extern ros::Duration delta_t_;

// // Upper and lower saturation limits
// extern double upper_limit_ , lower_limit_ ;

// // Anti-windup term. Limits the absolute value of the integral term.
// extern double windup_limit_ ;
 
void do_calculate();

void calculate_error();

#endif