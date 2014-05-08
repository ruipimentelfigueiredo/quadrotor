#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <swarm_msgs/SwarmFormationAction.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "swarm_control_client");

  // create the action client
  actionlib::SimpleActionClient<swarm_msgs::SwarmFormationAction> ac("swarm_control_action");
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  swarm_msgs::SwarmFormationGoal goal;
  goal.pose.pose.position.x=10.0;
  goal.pose.pose.position.y=10.0;
  goal.pose.pose.position.z=10.0;

  goal.pose.pose.orientation.x=0.0;
  goal.pose.pose.orientation.y=0.0;
  goal.pose.pose.orientation.z=0.0;
  goal.pose.pose.orientation.w=1.0;

  goal.precision=0.2;


  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}
