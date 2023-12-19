#include <ros/ros.h>
#include <vector>
#include <cmath>
#include "Eigen/Dense"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const float toRadians = M_PI/180.0;
std::vector<double> aruco_pose(7,0.0);
bool aruco_pose_available = false, find_des_pose = false, goal_execution = true;

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg){
  aruco_pose_available = true;
  aruco_pose.clear();
  aruco_pose.push_back(msg.pose.position.x);
  aruco_pose.push_back(msg.pose.position.y);
  aruco_pose.push_back(msg.pose.position.z);
  aruco_pose.push_back(msg.pose.orientation.x);
  aruco_pose.push_back(msg.pose.orientation.y);
  aruco_pose.push_back(msg.pose.orientation.z);
  aruco_pose.push_back(msg.pose.orientation.w);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_goal");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform base_footprint_tf, aruco_pose_tf, prox_goal;

  // Subscribers
  ros::Subscriber aruco_pose_sub = nh.subscribe("/aruco_single/pose", 1, arucoPoseCallback);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Go to goal3
  try{
    // Collect goals and print to terminal as debug
    listener.waitForTransform( "map", "goal3", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform( "map", "goal3", ros::Time(0), prox_goal);
    ROS_INFO("Current goal position:\t %f, %f, %f", prox_goal.getOrigin().x(), prox_goal.getOrigin().y(), prox_goal.getOrigin().z());
    ROS_INFO("Current goal orientation [x,y,z,w]:\t %f, %f, %f, %f\n", prox_goal.getRotation().x(), prox_goal.getRotation().y(), prox_goal.getRotation().z(), prox_goal.getRotation().w());
  }
  catch( tf::TransformException &ex ){
    ROS_ERROR("%s", ex.what());
    ros::shutdown();
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = prox_goal.getOrigin().x();
  goal.target_pose.pose.position.y = prox_goal.getOrigin().y();
  goal.target_pose.pose.orientation.x = prox_goal.getRotation().x();
  goal.target_pose.pose.orientation.y = prox_goal.getRotation().y();
  goal.target_pose.pose.orientation.z = prox_goal.getRotation().z();
  goal.target_pose.pose.orientation.w = prox_goal.getRotation().w();
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  try{
    // Collect goals and print to terminal as debug
    listener.waitForTransform( "map", "aruco_prox", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform( "map", "aruco_prox", ros::Time(0), prox_goal);
    ROS_INFO("Current goal position:\t %f, %f, %f", prox_goal.getOrigin().x(), prox_goal.getOrigin().y(), prox_goal.getOrigin().z());
    ROS_INFO("Current goal orientation [x,y,z,w]:\t %f, %f, %f, %f\n", prox_goal.getRotation().x(), prox_goal.getRotation().y(), prox_goal.getRotation().z(), prox_goal.getRotation().w());
  }
  catch( tf::TransformException &ex ){
    ROS_ERROR("%s", ex.what());
    ros::shutdown();
  }
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = prox_goal.getOrigin().x();
  goal.target_pose.pose.position.y = prox_goal.getOrigin().y();
  goal.target_pose.pose.orientation.x = prox_goal.getRotation().x();
  goal.target_pose.pose.orientation.y = prox_goal.getRotation().y();
  goal.target_pose.pose.orientation.z = prox_goal.getRotation().z();
  goal.target_pose.pose.orientation.w = prox_goal.getRotation().w();
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  while(ros::ok()){

    if(goal_execution){
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved");
        ros::shutdown();
      }
      else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("The base failed to move for some reason");
        ros::shutdown();
      }
    }

    // ROS_INFO("aruco_pose_available: %d", int(aruco_pose_available));
    if(aruco_pose_available){ // Every time we detect the aruko marker we publish its tf
      if(goal_execution){
        ROS_INFO("Aruco Marker detected!");
        ac.cancelGoal();
        goal_execution = false;
      }
      // d435 -> aruco_marker
      Eigen::Vector3d p_cam_to_object  = {aruco_pose[0], aruco_pose[1], aruco_pose[2]};
      Eigen::Quaterniond quaternion_cam(aruco_pose[6], aruco_pose[3], aruco_pose[4], aruco_pose[5]);
      Eigen::Matrix3d rot_cam_to_object = quaternion_cam.toRotationMatrix();

      // base_footprint -> d435
      Eigen::Vector3d p_base_to_cam = {0.0975, 0.0, 0.065 + 0.059 /* base_footprint -> base_link -> d435_joint*/};
      Eigen::Matrix3d rot_base_to_cam = (Eigen::AngleAxisd(-90.0 * toRadians, Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(-90.0 * toRadians, Eigen::Vector3d::UnitX())).toRotationMatrix();

      // aruco pose respect to the base_footprint frame                                
      Eigen::Vector3d p_base_to_object = p_base_to_cam + rot_base_to_cam*p_cam_to_object; 

      // map -> base_footprint
      try{
        listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform( "map", "base_footprint", ros::Time(0), base_footprint_tf);
      }
      catch( tf::TransformException &ex ) {
        ROS_ERROR("%s", ex.what());
        loop_rate.sleep();
        continue;
      }
      Eigen::Vector3d p_map_to_base = {base_footprint_tf.getOrigin().x(), base_footprint_tf.getOrigin().y(), base_footprint_tf.getOrigin().z()};
      Eigen::Quaterniond quaternion_base(base_footprint_tf.getRotation().w(), base_footprint_tf.getRotation().x(), base_footprint_tf.getRotation().y(), base_footprint_tf.getRotation().z());
      Eigen::Matrix3d rot_map_to_base = quaternion_base.toRotationMatrix();

      // aruco pose respect to the map frame
      Eigen::Vector3d p_map_to_object = p_map_to_base + rot_map_to_base*p_base_to_object;
      Eigen::Matrix3d rot_map_to_object = rot_map_to_base*rot_base_to_cam*rot_cam_to_object;
      
      // ARUCO TF broadcast
      aruco_pose_tf.stamp_ = ros::Time::now();
      aruco_pose_tf.frame_id_ = "map";
      aruco_pose_tf.child_frame_id_ = "aruco_pose";
      Eigen::Quaterniond quat_map_to_object(rot_map_to_object);
      tf::Quaternion quat_map_to_object_tf(quat_map_to_object.x(), quat_map_to_object.y(), quat_map_to_object.z(), quat_map_to_object.w());
      aruco_pose_tf.setOrigin({p_map_to_object[0], p_map_to_object[1], p_map_to_object[2]});
      aruco_pose_tf.setRotation(quat_map_to_object_tf);
      broadcaster.sendTransform(aruco_pose_tf);

      // Set the desired position the first time we detect the aruco
      if(!find_des_pose){
        // Compute the desire position from the aruco pose
        Eigen::Vector3d offset = {1.0, 0.0, 0.0};
        Eigen::Vector3d des_pose = p_map_to_object + offset;
        find_des_pose = true;

        // Set and send the goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        tf2::Quaternion orientation_quat;
        orientation_quat.setRPY( 0, 0, 180.0*toRadians);
        goal.target_pose.pose.position.x = des_pose[0];
        goal.target_pose.pose.position.y = des_pose[1];
        goal.target_pose.pose.orientation.x = orientation_quat[0];
        goal.target_pose.pose.orientation.y = orientation_quat[1];
        goal.target_pose.pose.orientation.z = orientation_quat[2];
        goal.target_pose.pose.orientation.w = orientation_quat[3];
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
      
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("Hooray, the base moved");
        }
        else{
          ROS_INFO("The base failed to move for some reason");
        }
      }
      aruco_pose_available = false;
    }
  ros::spinOnce();
  loop_rate.sleep();
  }
  return 0;
}