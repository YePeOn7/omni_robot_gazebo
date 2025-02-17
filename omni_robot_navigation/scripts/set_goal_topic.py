#!/usr/bin/python3

import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import math

def createGoal(x, y, th_deg):
  goal = PoseStamped()
  goal.header.frame_id = "map"
  goal.header.stamp = rospy.Time.now()

  goal.pose.position.x = x
  goal.pose.position.y = y
  
  q = quaternion_from_euler(0, 0, th_deg * math.pi / 180)
  goal.pose.orientation.x = q[0]
  goal.pose.orientation.y = q[1]
  goal.pose.orientation.z = q[2]
  goal.pose.orientation.w = q[3]

  return goal

rospy.init_node("set_goal_node")

pubGoal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
while pubGoal.get_num_connections() == 0 and not rospy.is_shutdown():
  rospy.sleep(0.1)

# rospy.sleep(0.1)

print("Creating goal")
goal = createGoal(7.5, -3, 90)

print("publishing goal")
pubGoal.publish(goal)

rospy.spin()
