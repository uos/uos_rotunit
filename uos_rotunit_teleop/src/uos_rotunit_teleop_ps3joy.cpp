/*
 *
 * Copyright (C) 2015 University of Osnabrück, Germany
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * uos_rotunit_teleop_ps3joy.cpp
 *
 *  Created on: 05.03.2015
 *      Author: Sebastian Pütz <spuetz@uos.de>
 */

#include "uos_rotunit_teleop_ps3joy.h"

RotunitTeleopPS3Joy::RotunitTeleopPS3Joy(ros::NodeHandle &nh)
  : nh_(nh),
    ac("rotunit_snapshotter", true),
    vel_(0)
{
  ROS_INFO("Waiting for the uos_rotunit_snapshotter action server to start.");
  ac.waitForServer(); 
  ROS_INFO("Connected to uos_rotunit_snapshotter server.");

  nh_.param("acc", acc_, 0.01);           // acceleration
  nh_.param("scan_vel", scan_vel_, 0.6);  // velocity while scanning
  nh_.param("max_vel", max_vel_, 1.3);    // maximum velocity
  nh_.param("timeout", timeout_, 30.0);    // waiting time for a snapshot to be finished
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("rot_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 15,  &RotunitTeleopPS3Joy::PS3Callback, this);
  rot_vel_client_ = nh_.serviceClient<uos_rotunit_driver::RotVelSrv>("rotunit_velocity");
}

void RotunitTeleopPS3Joy::PS3Callback(const sensor_msgs::Joy::ConstPtr &joy){

  uos_rotunit_driver::RotVelSrv rot_vel_srv;
  
  // handle uos_rotunit velocity 
  // via ps3 buttons rear right 1 and 2
  if(joy->buttons[PS3_BUTTON_REAR_RIGHT_2])
    vel_ += acc_;
  if(joy->buttons[PS3_BUTTON_REAR_RIGHT_1])
    vel_ -= acc_;
  
  // ensure maxima
  if(vel_ > max_vel_)
    vel_ = max_vel_;
  if(vel_ < -max_vel_) 
    vel_ = -max_vel_;

  rot_vel_srv.request.twist.angular.z = vel_;
  rot_vel_client_.call(rot_vel_srv);
  
  // handle uos_rotunit_snapshotter scan command
  if(joy->buttons[PS3_BUTTON_ACTION_TRIANGLE]){
    sound_client_.say("Starting scan");

    // set scan velocity and wait for 3 seconds
    rot_vel_srv.request.twist.angular.z = scan_vel_;
    rot_vel_client_.call(rot_vel_srv);
    ROS_INFO("start snapshot in 3 seconds...");
    ros::Duration(3.0).sleep();
    
    // call snapshotter action server
    ROS_INFO("start snapshot...");
    uos_rotunit_snapshotter::RotunitSnapshotGoal goal;
    goal.angle = 2 * M_PI;
    ac.sendGoal(goal);
    bool finished_before_timeout =
      ac.waitForResult(ros::Duration(timeout_));

    if (finished_before_timeout){
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Snapshot action finished with the state: %s",state.toString().c_str());
      sound_client_.say("Finished scan");
    }
    else{
      ROS_INFO("Snapshot action did not finish before the timeout of %f seconds.", timeout_);
      sound_client_.say("Scan didn't finish in time");
    }

    // set standard velocity and call velocity service
    rot_vel_srv.request.twist.angular.z = vel_;
    rot_vel_client_.call(rot_vel_srv);
  }
}

int main(int args, char**argv){
  ros::init(args, argv, "uos_rotunit_teleop_ps3joy");
  ros::NodeHandle nh;
  RotunitTeleopPS3Joy teleop(nh);
  ros::spin();
  return EXIT_SUCCESS;
}
