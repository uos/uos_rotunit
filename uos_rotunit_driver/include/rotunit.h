#ifndef _ROTUNIT_H_
#define _ROTUNIT_H_


#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <uos_rotunit_driver/RotVelSrv.h>

#include <cmath>
#include <cstdio>

#include "can.h"

//CAN IDs
#define CAN_GETROTUNIT 0x00000010 // current rotunit angle
#define CAN_SETROTUNT  0x00000080 // send rotunit speed

class Rotunit
{
  public:
    Rotunit(ros::NodeHandle &nh);
    ~Rotunit();
    void can_rotunit_send(double speed);
    void can_rotunit(const can_frame &frame);
    int can_read_fifo();
    void rotunitCallback(
      const geometry_msgs::Twist::ConstPtr& msg);
    bool rotunitRotVelSrv(
      uos_rotunit_driver::RotVelSrv::Request &req,
      uos_rotunit_driver::RotVelSrv::Response &res);

  private:
    sensor_msgs::JointState previous_state;
    ros::ServiceServer rot_vel_srv_;
    double normalize2PI(double angle);
    ros::Subscriber sub_;
    ros::Publisher rot_pub_;
    ros::Publisher vel_pub_;
    CAN can_;
    ros::NodeHandle nh_;
};

#endif /* rotunit.h */
