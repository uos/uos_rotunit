/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  uos_rotunit_teleop_rviz.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */


#include "rviz_scan_tool.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uos_rotunit_teleop::RvizScanTool, rviz::Tool)

namespace uos_rotunit_teleop{


RvizScanTool::RvizScanTool()
 :ac_("rotunit_snapshotter", true)
{
}

RvizScanTool::~RvizScanTool(){};

void RvizScanTool::onInitialize(){
  ros::NodeHandle nh;

  nh.param("timeout", timeout_, 30.0);    // waiting time for a snapshot to be finished
  rot_vel_client_ = nh.serviceClient<uos_rotunit_driver::RotVelSrv>("rotunit_velocity");
  setName("Assemble Cloud");
}

int RvizScanTool::processMouseEvent( rviz::ViewportMouseEvent& event ){
  return flags_;
}

void RvizScanTool::deactivate(){

}

void RvizScanTool::activate(){
  flags_ = Render;

  bool connected = ac_.isServerConnected();
  
  if(!connected){
    ROS_INFO("Try to connect to the uos_rotunit_snapshotter action server.");
    connected = ac_.waitForServer(ros::Duration(1)); 
    if(connected){
        ROS_INFO("Connected to uos_rotunit_snapshotter server.");
    }else{
        ROS_WARN("Please start the uos_rotunit_snapshotter!");
    }
  }
  
  if(connected){
    uos_rotunit_snapshotter::RotunitSnapshotGoal goal;
    goal.angle = 2 * M_PI;
    ac_.sendGoal(goal);
    ac_.sendGoal(goal,
      boost::bind(&RvizScanTool::doneCb, this, _1, _2),
      actionlib::SimpleActionClient<uos_rotunit_snapshotter::RotunitSnapshotAction>::SimpleActiveCallback(),
      actionlib::SimpleActionClient<uos_rotunit_snapshotter::RotunitSnapshotAction>::SimpleFeedbackCallback());
    ROS_INFO("Assembling cloud.");
    setName("Assembling Cloud...");

  }
}

void RvizScanTool::doneCb(
  const actionlib::SimpleClientGoalState& state,
  const uos_rotunit_snapshotter::RotunitSnapshotResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
	setName("Assemble Cloud");
	flags_ = (Finished|Render);

  }


} /* namespace uos_rotunit_teleop */
