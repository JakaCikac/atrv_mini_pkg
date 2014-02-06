/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
//#include <include/sound_play.h>
#include <unistd.h>
#include "ros/console.h"
#include <iostream>
#include <string>
using namespace std;

class ATRVTeleop
{
public:
  ATRVTeleop();
  double l_scale_, a_scale_;
  bool deadman_pressed_, down_speed_pressed, up_speed_pressed, brake_enable_pressed, brake_disable_pressed, blind_button_pressed;
  //bool sonar_enable_pressed, sonar_disable_pressed;
  bool turbo_enable_pressed, turbo_disable_pressed;
  bool turbo_disabled;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_, up_speed, down_speed, brake_enable, brake_disable, blind_button;
  //int sonar_enable, sonar_disable;
  int turbo_enable, turbo_disable;

  // joystick velocity publisher / subscriber
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  // brake enable/disable
  ros::Publisher brake_pub;
  // sonar enable / disable
  //ros::Publisher sonar_pub;
  
  geometry_msgs::Twist last_published_;
  // brake/sonar control
  std_msgs::Bool enable;
  boost::mutex publish_mutex_;
  //sound_play::SoundClient sc;
  
  ros::Timer timer_;
};

ATRVTeleop::ATRVTeleop():
  ph_("~"),
  linear_(3), //1,0 combination gives D-pad control of the robot
  angular_(2), 
  deadman_axis_(4),
  up_speed(5),
  down_speed(7),
  brake_enable(8),
  brake_disable(9),
  blind_button(6),
  //sonar_enable(0),
  //sonar_disable(1),
  turbo_enable(0),
  turbo_disable(1),
  l_scale_(0.5),
  a_scale_(0.9) //Auto joystick scale
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ATRVTeleop::joyCallback, this); //10
  brake_pub = ph_.advertise<std_msgs::Bool>("/ATRVmini_node/cmd_brake_power", 1);
  //sonar_pub = ph_.advertise<std_msgs::Bool>("/ATRVmini_node/cmd_sonar_power", 1);
  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&ATRVTeleop::publish, this));
  turbo_disabled = true;
}

void ATRVTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  up_speed_pressed = joy->buttons[up_speed];
  down_speed_pressed = joy->buttons[down_speed];
  brake_enable_pressed = joy->buttons[brake_enable];
  brake_disable_pressed = joy->buttons[brake_disable];
  blind_button_pressed = joy->buttons[blind_button];
  //sonar_enable_pressed = joy->buttons[sonar_enable];
  //sonar_disable_pressed = joy->buttons[sonar_disable];
  turbo_enable_pressed = joy->buttons[turbo_enable];
  turbo_disable_pressed = joy->buttons[turbo_disable];
}

void ATRVTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (blind_button_pressed) {
	brake_enable_pressed = false;
	brake_disable_pressed = false;
	deadman_pressed_ = false;
	//sonar_disable_pressed = false;
	//sonar_enable_pressed = false;
	turbo_enable_pressed = false;
	turbo_disable_pressed = false;
	ros::Duration(0.5).sleep();
	// Code to disable movement when no button pressed	
	//vel.angualr.z = 0;
	//vel.linear.x = 0;
	//vel_pub_.publish(vel);
  }

  /* else if (sonar_enable_pressed) {
	enable.data = true;
	sonar_pub.publish(enable);
	ROS_WARN("Sonars enabled!");
	ros::Duration(0.5).sleep();
	}
	
  else if (sonar_disable_pressed) {
	enable.data = false;
	sonar_pub.publish(enable);
	ROS_WARN("Sonars disabled!");
	ros::Duration(0.5).sleep();
	} */

  else if (brake_enable_pressed) {
	enable.data = true;
	brake_pub.publish(enable);
	//sc.say("brake enabled, robot secure");
	ROS_WARN("BRAKE ENABLED, robot secure!");
	ros::Duration(0.5).sleep();
	}
	
  else if (brake_disable_pressed) {
	enable.data = false;
	brake_pub.publish(enable);
	//sc.say("brake disabled, robot hot");
	ROS_WARN("BRAKE DISABLED, robot hot!");
	ros::Duration(0.5).sleep();
	}
	
  else if (deadman_pressed_)
  {
	if(up_speed_pressed && a_scale_ < 1.21 && l_scale_ < 1.21 && turbo_disabled) {
		if((a_scale_ > 1.20) || (l_scale_ > 1.20)) {
			ROS_INFO("Can not further increase speed, currently at MAXIMUM.");
		} else {
		a_scale_ = a_scale_ + 0.01;
		l_scale_ = l_scale_ + 0.01;
		ROS_INFO("Speed increased! Ang: %.2f Lin: %.2f", a_scale_, l_scale_ );
		ros::Duration(0.1).sleep();
		}
	}
	else if(down_speed_pressed && a_scale_ > 0.01 && l_scale_ > 0.01 && turbo_disabled) {
		if((a_scale_ < 0.021) || (l_scale_ < 0.021)) {
			ROS_INFO("Can not further lower speed, currently at MINIMUM.");
		} else {
		a_scale_ = a_scale_ - 0.01;
		l_scale_ = l_scale_ - 0.01;
		ROS_INFO("Speed decreased! Ang: %.2f Lin: %.2f", a_scale_, l_scale_ );
		ros::Duration(0.1).sleep();
		}
	}
	else if (turbo_enable_pressed) {
		turbo_disabled = false;
		a_scale_ = 1.5;
		l_scale_ = 1.3;
		ROS_WARN("Turbo mode online!");
		ROS_INFO("Speed increased! Ang: %.2f Lin: %.2f", a_scale_, l_scale_ );
		ROS_INFO("You are allowed to increase speed to unlimited.");
		ROS_WARN("OPERATE WITH SAFETY IN MIND.");
		ros::Duration(0.5).sleep();	
	}
	else if (turbo_disable_pressed) {
		turbo_disabled = true;
		a_scale_ = 0.3;
		l_scale_ = 0.2;
		ROS_WARN("Turbo mode offline.");
		ROS_INFO("Speed decreased! Ang: %.2f Lin: %.2f", a_scale_, l_scale_ );
		ros::Duration(0.5).sleep();	
	}
	else if(up_speed_pressed && a_scale_ < 6 && l_scale_ < 6 && !turbo_disabled) {
		if((a_scale_ > 5.99) || (l_scale_ > 5.99)) {
			ROS_INFO("Can not further increase speed, won't have effect.");
		} else {
		a_scale_ = a_scale_ + 0.01;
		l_scale_ = l_scale_ + 0.01;
		ROS_INFO("Speed increased! Ang: %.2f Lin: %.2f", a_scale_, l_scale_ );
		ros::Duration(0.1).sleep();
		}
	}
	else if(down_speed_pressed && a_scale_ > 0.01 && l_scale_ > 0.01 && !turbo_disabled) {
		if((a_scale_ < 0.021) || (l_scale_ < 0.021)) {
			ROS_INFO("Can not further lower speed, currently at MINIMUM.");
		} else {
		a_scale_ = a_scale_ - 0.01;
		l_scale_ = l_scale_ - 0.01;
		ROS_INFO("Speed decreased! Ang: %.2f Lin: %.2f", a_scale_, l_scale_ );
		ros::Duration(0.1).sleep();
		}
	}
    vel_pub_.publish(last_published_);
	} 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ATRV_teleop");
  ATRVTeleop ATRV_teleop;
  ROS_INFO("Node ATRVmini_joy launched, enjoy driving.");
  ROS_INFO("Current speed: (angular, linear): %.2f %.2f", ATRV_teleop.a_scale_, ATRV_teleop.l_scale_ );
  cout << "\n";
  cout << "\n";
  cout << "Hold button 5 and then use the right joystick to drive the robot around.\n";
  cout << "You can use button 6 to increase the speed and button 8 to lower it. \n";
  cout << "\n";
  cout << "BRAKES:\n";
  cout << "Use button 9 to ENABLE the brake (= STOP the robot).\n";
  cout << "Use button 10 to DISABLE the brake.\n";
  cout << "\n";
  cout << "TURBO:\n";
  cout << "Use button 1 to ENABLE turbo mode. WARNING, operate with safety in mind!.\n";
  cout << "Use button 2 to DISABLE the turbo mode - speed will return to default.\n";
  cout << "\n";
  ros::spin();
}
