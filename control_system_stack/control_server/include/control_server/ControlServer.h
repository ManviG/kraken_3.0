#ifndef CONTROLSERVER_H
#define CONTROLSERVER_H
#include <ros/ros.h>
#include <kraken_msgs/krakenPose.h>
#include <kraken_msgs/thrusterData4Thruster.h>
#include <kraken_msgs/thrusterData6Thruster.h>
#include <control_server/AuvController.h>
#include <kraken_msgs/moveAlongLine.h>
#include <kraken_msgs/advancedControllerAction.h>
#include <kraken_msgs/advancedControllerGoal.h>
#include <kraken_msgs/controllerGoal.h>
#include <kraken_msgs/controllerAction.h>
#include <pose_server/KrakenPose.h>
#include <kraken_msgs/ipControllererror.h>
#include <actionlib/server/simple_action_server.h>
#include <kraken_msgs/switchControllers.h>

namespace kraken_controller
{
  class ControlServer
  {
    public:
      ControlServer(float freq=10);
      void timeCallBack(const ros::TimerEvent&);
      void setServers(actionlib::SimpleActionServer<kraken_msgs::advancedControllerAction>*,actionlib::SimpleActionServer<kraken_msgs::controllerAction>*);
      void poseFeedBack(const kraken_msgs::krakenPose::ConstPtr &msg);
      void ipErrorFeedBack(const kraken_msgs::ipControllererror::ConstPtr &msg);
      bool moveAlongLine(kraken_msgs::moveAlongLine::Request  &req,
                         kraken_msgs::moveAlongLine::Response &res);
      bool changeController(kraken_msgs::switchControllers::Request  &req,
                         kraken_msgs::switchControllers::Response &res);
      void executePoseChange(const kraken_msgs::advancedControllerGoalConstPtr &msg);
      void executeOrientationChange(const kraken_msgs::controllerGoalConstPtr &msg);
      virtual ~ControlServer();
    protected:
      
    private:
      ros::Publisher _pub;
      ros::Subscriber _sub_pose;
      ros::Subscriber _sub_ip_error;
      ros::Timer _time;
      AuvController _controller;
      kraken_msgs::krakenPose _feedBack;
      bool _do_control;
      actionlib::SimpleActionServer<kraken_msgs::advancedControllerAction> *_server1;
      actionlib::SimpleActionServer<kraken_msgs::controllerAction> *_server2;
      bool _ip_controller;
  };
}

#endif // CONTROLSERVER_H
