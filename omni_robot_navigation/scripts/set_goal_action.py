#!/usr/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
import math

class GoalAction:
  def __init__(self):
    self.actionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

  def createGoal(self, x, y, th_deg):
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

  def createGoalAction(self, x, y, th_deg):
    goalAction = MoveBaseGoal()
    goalAction.target_pose = self.createGoal(x, y, th_deg)
    return goalAction

  def moveToGoal(self, x, y, th_deg, feedbackCallback = None):
    self.actionClient.wait_for_server()
    goal = self.createGoalAction(x, y, th_deg)
    self.actionClient.send_goal(goal, feedback_cb=feedbackCallback)

    # self.actionClient.wait_for_result()
  
  def getStatus(self) -> GoalStatus:
    return self.actionClient.get_state()

  def cancel(self):
    self.actionClient.cancel_goal()

if(__name__ == "__main__"):
  rospy.init_node("set_goal_node")

  goalAction = GoalAction()

  pubGoal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
  while pubGoal.get_num_connections() == 0 and not rospy.is_shutdown():
    rospy.sleep(0.1)

  # goalAction.moveToGoal(5, 0, 0)
  while(not rospy.is_shutdown()):
    print("State:", goalAction.getStatus())
    rospy.sleep(0.1)
  rospy.spin()
