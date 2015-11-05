#ifndef UOS_ROTUNIT_TELEOP__RVIZ_SCAN_TOOL_H_
#define UOS_ROTUNIT_TELEOP__RVIZ_SCAN_TOOL_H_

#include <ros/ros.h>
#include <rviz/tool.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <uos_rotunit_snapshotter/RotunitSnapshotAction.h>
#include <uos_rotunit_driver/RotVelSrv.h>


#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>
#include <QCursor>
#endif

namespace uos_rotunit_teleop{
  class RvizScanTool: public rviz::Tool
  {
    Q_OBJECT
    public:
      RvizScanTool();
      ~RvizScanTool();

      virtual void onInitialize();

      virtual void activate();
      virtual void deactivate();

      virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
	  
	  void doneCb(
		const actionlib::SimpleClientGoalState& state,
		const uos_rotunit_snapshotter::RotunitSnapshotResultConstPtr& result);
		
    private:
      ros::ServiceClient rot_vel_client_;
      double timeout_;
      actionlib::SimpleActionClient<uos_rotunit_snapshotter::RotunitSnapshotAction> ac_;
      int flags_;
  };
} /* namespace uos_rotunit_teleop */

#endif /* rviz_scan_tool.h */
