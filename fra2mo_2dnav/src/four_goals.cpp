#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const float toRadians = M_PI/180.0;
 
int main(int argc, char** argv){
  ros::init(argc, argv, "four_goals");
  tf::TransformListener listener;
  tf::StampedTransform transform;
  move_base_msgs::MoveBaseGoal goal;

  ros::Rate r(1);

  // Vector of goals position
  std::vector<std::string> traj = {"goal3", "goal4", "goal2", "goal1"};
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  for(int i = 0; i < traj.size(); i++){
    try{
      // Collect goals and print to terminal as debug
      listener.waitForTransform( "map", traj[i], ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform( "map", traj[i], ros::Time(0), transform);
      ROS_INFO("Current goal position:\t %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
      ROS_INFO("Current goal orientation [x,y,z,w]:\t %f, %f, %f, %f\n", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    }
    catch( tf::TransformException &ex ) {
      ROS_ERROR("%s", ex.what());
      r.sleep();
      continue;
    }

    // Set and send the goal
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = transform.getOrigin().x();
    goal.target_pose.pose.position.y = transform.getOrigin().y();
    goal.target_pose.pose.orientation.x = transform.getRotation().x();
    goal.target_pose.pose.orientation.y = transform.getRotation().y();
    goal.target_pose.pose.orientation.z = transform.getRotation().z();
    goal.target_pose.pose.orientation.w = transform.getRotation().w();
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
  
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the base moved\n");
      ros::Duration(1.0).sleep();
    }
    else{
      ROS_INFO("The base failed to move for some reason");
      break;
    }
  }
   return 0;
}