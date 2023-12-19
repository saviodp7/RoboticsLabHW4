#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "base_footprint_pub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  tf::TransformListener listener;
  tf::StampedTransform base_footprint_tf;
  geometry_msgs::Pose pose_msg;
        
  // Subscribers
  ros::Publisher pose_publisher = nh.advertise<geometry_msgs::Pose>("/fra2mo/pose", 1);

  while(ros::ok()){
  // Collect base_footprint tf
  try{
    listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform( "map", "base_footprint", ros::Time(0), base_footprint_tf);
  }
  catch( tf::TransformException &ex ){
    ROS_ERROR("%s", ex.what());
    ros::shutdown();
  }

  pose_msg.position.x = base_footprint_tf.getOrigin().x();
  pose_msg.position.y = base_footprint_tf.getOrigin().y();
  pose_msg.position.z = base_footprint_tf.getOrigin().z();
  pose_msg.orientation.x = base_footprint_tf.getRotation().x();
  pose_msg.orientation.y = base_footprint_tf.getRotation().y();
  pose_msg.orientation.z = base_footprint_tf.getRotation().z();
  pose_msg.orientation.w = base_footprint_tf.getRotation().w();

  pose_publisher.publish(pose_msg);

  ros::spinOnce();
  loop_rate.sleep();
  }
}