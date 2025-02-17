#!/usr/bin/env python3
import rospy 
import smach
import smach_ros
from set_goal_action import GoalAction
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus

class State:
  IDLE = 0
  WAYPOOINT_LOOP = 1

class SharedData:
  def __init__(self):
    self.state = State.IDLE

class IdleState(smach.State):
  def __init__(self, sharedData:SharedData, outcomes=["to_waypoint_loop"]):
    super().__init__(outcomes)
    self.sharedData = sharedData
  
  def execute(self, ud):
    rospy.loginfo("Enter Idle State")

    while not rospy.is_shutdown():
      # print(">>>", self.sharedData.state)
      if(self.sharedData.state == State.WAYPOOINT_LOOP):
        return "to_waypoint_loop"
      
      rospy.sleep(1)

    return "finish"
  
class WaypointLoopState(smach.State):
  def __init__(self, sharedData:SharedData, outcomes=["to_idle"]):
    super().__init__(outcomes)
    self.sharedData = sharedData
    self.goalAction = GoalAction()
    self.iWp = 0
    self.wp = [
      [2, 3, 0], 
      [5, 0, 90],
      [7, 3, -30],
      [5, 3, 150],
      [5, 3, 45],
      [7.5, -3, 90],
      [5.5, -2.5, -90],
      [0, -2.5, 90],
      ]
    self.lenWp = len(self.wp)
  
  def execute(self, ud):
    rospy.loginfo("Enter Idle State")

    while not rospy.is_shutdown():
      if(self.sharedData.state == State.IDLE):
        self.sharedData.state = State.IDLE
        self.goalAction.cancel()
        return "to_idle"
      
      if(self.goalAction.getStatus() != GoalStatus.ACTIVE):
        if(self.goalAction.getStatus() == GoalStatus.SUCCEEDED):
          print("Get Status of Success (Goal Reached). Increasing the index")
          self.iWp = (self.iWp + 1) % self.lenWp
        
        x = self.wp[self.iWp][0]
        y = self.wp[self.iWp][1]
        th = self.wp[self.iWp][2]

        self.goalAction.moveToGoal(x, y, th)
      
      rospy.sleep(0.1)

    return "to_idle"

def stateControlCallback(msg:String, sharedData:SharedData):
  if(msg.data == "idle"):
    sharedData.state = State.IDLE
  elif(msg.data == "waypoint"):
    sharedData.state = State.WAYPOOINT_LOOP

if __name__ == "__main__":
  rospy.init_node("navigation_sm")

  sharedData = SharedData()
  rospy.Subscriber("/state_control", String, stateControlCallback, sharedData)

  sm = smach.StateMachine(outcomes=["finish"])

  with sm:
    smach.StateMachine.add("IDLE", IdleState(sharedData), transitions={"to_waypoint_loop": "WAYPOINT_LOOP"})
    smach.StateMachine.add("WAYPOINT_LOOP", WaypointLoopState(sharedData), transitions={"to_idle": "IDLE"})
  
  sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
  sis.start()

  outcome = sm.execute()
  sis.stop()