#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

geometry_msgs::PoseStamped createGoal(double x, double y, double th_deg){
  geometry_msgs::PoseStamped goal;

  tf2::Quaternion q;
  q.setRPY(0,0,th_deg * M_PI / 180);
  
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

int main(int argc, char** argv){
  ros::init(argc, argv, "set_goal_node");
  ros::NodeHandle nh;

  ros::Publisher pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  while(pubGoal.getNumSubscribers() == 0) ros::Duration(0.1).sleep();

  geometry_msgs::PoseStamped goal = createGoal(4, 0, 180);
  pubGoal.publish(goal);

  ros::spin();
}