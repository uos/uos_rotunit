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
 * uos_rotunit_snapshotter.h
 *
 * Created on: 26.02.2015
 * 	Authors: Sebastian Puetz <spuetz@uni-osnabrueck.de>
 */

#ifndef UOS_ROTUNIT_SNAPSHOTTER_H
#define UOS_ROTUNIT_SNAPSHOTTER_H

#include <ros/ros.h>
#include <ros/topic.h>

#include <sensor_msgs/JointState.h>

#include <string>
#include <laser_assembler/AssembleScans2.h>

#include <uos_rotunit_snapshotter/RotunitSnapshotAction.h>
#include <actionlib/server/simple_action_server.h>

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#define MAX_STEP_SIZE M_PI / 8.0

typedef actionlib::SimpleActionServer<uos_rotunit_snapshotter::RotunitSnapshotAction> Server;

  class RotunitSnapshotter
  {
    public:
      RotunitSnapshotter(ros::NodeHandle& nh, std::string);

    private:
      int getIndex(const sensor_msgs::JointState::ConstPtr &jointState);
      bool checkIfFinished(double rot, ros::Time stamp);
      ros::Time begin_stamp;
      ros::Time end_stamp;
      double begin_angle;
      double end_angle;

      bool started_;
      bool finished_;
      boost::mutex mutex_;
      boost::condition_variable condition_;
      double previous_rot;
      double residual_rot;
      double destination_rot;
      double rotation_angle;
      bool continuous;

      std::string action_name_;

      void rotCallback(const sensor_msgs::JointState::ConstPtr &jointState);
      void makeSnapshot(const uos_rotunit_snapshotter::RotunitSnapshotGoalConstPtr& goal);
      double norm2PI(double angle);

      ros::NodeHandle nh_;
      ros::ServiceClient client_;
      ros::Publisher cloud_pub_;
      ros::Publisher snapshot_pub_;
      ros::Subscriber state_sub_;
      Server server_;
      uos_rotunit_snapshotter::RotunitSnapshotResult result_;
  };

  #endif /* uos_rotunit_snapshotter.h */
