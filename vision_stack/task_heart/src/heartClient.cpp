#ifndef HEART_C
#define HEART_C


#include <heartTask/heartHeader.h>
#include <ip_msgs/heartAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
#include <heartTask/heartGoal.h>
#include <heartTask/heartResult.h>


typedef actionlib::SimpleActionClient<ip_msgs::heartAction> Client;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "heartclient");
    Client _client("heart_server", true);
    ROS_INFO("The Client has started and waiting for server to start ....");
    _client.waitForServer();
   ip_msgs::heartGoal _goal;
    cout << "action client";
   if(argc!=0)
   {
     _goal.heart = argc;
   }
   else
       _goal.heart=1;

    _client.sendGoal(_goal);
    bool _actionStatus = _client.waitForResult(ros::Duration(15.0));
  //  ROS_INFO("error x and error y : %f  and %f", )
    if(_actionStatus == true)
    {
        actionlib::SimpleClientGoalState _state = _client.getState();
        ROS_INFO("Action finished: %s",_state.toString().c_str());
    }
    else
    {
        ROS_INFO("The action did not finish within the specified time");
        _client.cancelGoal();
    }

    return 0;
}

#endif HEART_C
