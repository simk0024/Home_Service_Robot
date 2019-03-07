#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects_node");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal goal2;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 8.0;
  goal.target_pose.pose.orientation.w = 1.0;
  goal2.target_pose.pose.position.x = -8.0;
  goal2.target_pose.pose.position.y = 0.0;
  goal2.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Pick-up zone...");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal 1
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reach Pick-up zone!");
  else
    ROS_INFO("The robot failed to reach Pick-up zone...");
  
  //pause 5 seconds after reaching 
  sleep(5);
  
  // Send the goal 2 position and orientation for the robot to reach
  ROS_INFO("Sending Drop-off zone...");
  ac.sendGoal(goal2);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal 2
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reach Drop-off zone!");
  else
    ROS_INFO("The robot failed to Drop-off zone...");
  
  //pause 5 seconds after reaching
  sleep(5);
  
  return 0;
}