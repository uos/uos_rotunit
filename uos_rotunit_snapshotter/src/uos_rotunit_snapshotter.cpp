/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2015 University of Osnabrück
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
 * uos_rotunit_snapshotter.cpp
 *
 * Created on: 26.02.2015
 * 	Authors: Sebastian Puetz <spuetz@uni-osnabrueck.de>
 */
#include "uos_rotunit_snapshotter.h"

RotunitSnapshotter::RotunitSnapshotter(ros::NodeHandle& nh, std::string action_name)
  : nh_(nh),
    action_name_(action_name),
    started_(false),
    server_(nh_, action_name_, boost::bind(&RotunitSnapshotter::makeSnapshot, this, _1), false)
{
  ros::NodeHandle nh_private("~");
  nh_private.param("rotation_angle", rotation_angle , 2*M_PI);
  nh_private.param("continuous", continuous, true);

  ROS_INFO("rotation angle is %f and continuous is %s", rotation_angle, continuous ? "true" : "false");

  client_ = nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
  snapshot_pub_ = nh_.advertise<uos_rotunit_snapshotter::RotunitSnapshot> ("snapshot", 1);
  server_.start();
  if(continuous)
    state_sub_ = nh_.subscribe("joint_states", 1000, &RotunitSnapshotter::rotCallback, this);
  ROS_INFO("%s action server is running...", action_name_.c_str());
}

double RotunitSnapshotter::norm2PI(double angle){
  while(angle < 0)
    angle += 2 * M_PI;
  while(angle > 2 * M_PI)
    angle -= 2 * M_PI;
  return angle;
}

int RotunitSnapshotter::getIndex(const sensor_msgs::JointState::ConstPtr &jointState){
  for (size_t i = 0; i < jointState->name.size(); ++i)
  {
    if (jointState->name[i] == "laser_rot_joint")
      return i;
  }
  return -1;
}

bool RotunitSnapshotter::checkIfFinished(double current_rot, ros::Time stamp){
  if(!started_){
    begin_angle = current_rot;
    begin_stamp = stamp;
    previous_rot = current_rot;
    residual_rot = rotation_angle;
    destination_rot = norm2PI(current_rot + rotation_angle);
    started_ = true;
    ROS_INFO("Start snapshot at %f degree, the destination angle is %f, angle aim is %f",
      current_rot * 180.0 / M_PI, rotation_angle * 180.0 / M_PI, destination_rot * 180.0 / M_PI);
  }
  double diff = norm2PI(current_rot - previous_rot);
  if(diff > M_PI / 180.0 && diff <= MAX_STEP_SIZE)
    residual_rot -= diff;

  // TODO cehck rotational direction

  if(residual_rot <= 0)
  {
    end_angle = current_rot;
    end_stamp = stamp;
    started_ = false;
    ROS_INFO("Finished snapshot at %f degree!", current_rot * 180.0 / M_PI);
    return true;
  }
  previous_rot = current_rot;
  return false;
}

void RotunitSnapshotter::makeSnapshot(const uos_rotunit_snapshotter::RotunitSnapshotGoalConstPtr &goal){
  finished_ = false;
  if(!continuous)
    state_sub_ = nh_.subscribe("joint_states", 1000, &RotunitSnapshotter::rotCallback, this);
  boost::mutex::scoped_lock lock(mutex_);
  while (!finished_)
    condition_.wait(lock);

  server_.setSucceeded(result_);
}

void RotunitSnapshotter::rotCallback(const sensor_msgs::JointState::ConstPtr &jointState){
  boost::mutex::scoped_lock lock(mutex_);
  
  if(server_.isPreemptRequested() || !ros::ok()){
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    finished_ = true;
    server_.setPreempted();
    condition_.notify_one();
    return;
  }
  int index = getIndex(jointState);
  if(index == -1)
    return;

  double rot = jointState->position[index];
  ROS_DEBUG("Current angle: %f", rot * 180 /M_PI);

  if(checkIfFinished(rot, jointState->header.stamp)){
    laser_assembler::AssembleScans2 srv;
    srv.request.begin = begin_stamp;
    srv.request.end = end_stamp;

    ROS_INFO("Call assembler with a duration of %fsec",
    end_stamp.toSec()-begin_stamp.toSec());

    if (client_.call(srv)){
      cloud_pub_.publish(srv.response.cloud);
      ROS_INFO("Published scan with %d points", srv.response.cloud.height * srv.response.cloud.width);
      uos_rotunit_snapshotter::RotunitSnapshot snapshot;
      snapshot.header.stamp = end_stamp;
      snapshot.cloud = srv.response.cloud;
      snapshot.begin_angle = begin_angle;
      snapshot.end_angle = end_angle;
      snapshot.scan_time = end_stamp - begin_stamp;
      result_.snapshot = snapshot;
      
      snapshot_pub_.publish(snapshot);

      finished_ = true;
      if(!continuous)
        state_sub_.shutdown();
      condition_.notify_one();
    }
    else
      ROS_ERROR("Error making service call\n") ;
  }
}

int main(int argc, char** argv){

  ros::init(argc, argv, "RotunitSnapshotter");
  ros::NodeHandle nh;
  
  ROS_INFO("Waiting for [assemble_scans2] to be advertised");
  ros::service::waitForService("assemble_scans2");
  ROS_INFO("Found [assemble_scans2]! Starting the snapshotter");
  
  RotunitSnapshotter snapshotter(nh, std::string("rotunit_snapshotter"));
  ros::spin();
  return EXIT_SUCCESS;
}
