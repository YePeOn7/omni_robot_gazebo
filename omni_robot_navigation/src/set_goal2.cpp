#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

geometry_msgs::PoseStamped setPosition(double x, double y, double th_degree) {
  double th_rad = th_degree * M_PI / 180;

  tf2::Quaternion q;
  q.setRPY(0, 0, th_rad);

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();

  goal.pose.position.x = x;
  goal.pose.position.y = y;

  goal.pose.orientation.x = q.x();
  goal.pose.orientation.y = q.y();
  goal.pose.orientation.z = q.z();
  goal.pose.orientation.w = q.w();

  return goal;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "set_goal_node");
  ros::NodeHandle nh;

  ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  while (goalPub.getNumSubscribers() == 0) ros::Duration(0.1).sleep();

  geometry_msgs::PoseStamped goal = setPosition(2, 0, 90);
  ROS_INFO_STREAM("x: " << goal.pose.position.x);
  ROS_INFO_STREAM("y: " << goal.pose.position.y);
  ROS_INFO_STREAM("z: " << goal.pose.position.z);
  ROS_INFO_STREAM("o.x: " << goal.pose.orientation.x);
  ROS_INFO_STREAM("o.y: " << goal.pose.orientation.y);
  ROS_INFO_STREAM("o.z: " << goal.pose.orientation.z);
  ROS_INFO_STREAM("o.w: " << goal.pose.orientation.w);

  goalPub.publish(goal);
  // ROS_INFO("Goal published");

  ros::spin();
  return 0;
}