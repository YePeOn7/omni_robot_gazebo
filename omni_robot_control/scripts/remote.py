#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from kinematics import OmniKinematics

GAIN = 2 # use this to adjust the speed of the robot to match the real speed
ROBOT_RADIUS = 0.088
WHEEL_RADIUS = 0.028
omni = OmniKinematics(3, ROBOT_RADIUS, WHEEL_RADIUS)

def cmdVelCallback(msg):
  [m1, m2, m3] = omni.drive(msg.linear.x, msg.linear.y, msg.angular.z)

  pubMotor1.publish(m1 * GAIN)
  pubMotor2.publish(m2 * GAIN)
  pubMotor3.publish(m3 * GAIN)

if __name__ == '__main__':
  try:
    rospy.init_node('remote', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, cmdVelCallback)
    pubMotor1 = rospy.Publisher('/robot0/first_wheel_controller/command', Float64, queue_size = 10)
    pubMotor2 = rospy.Publisher('/robot0/second_wheel_controller/command', Float64, queue_size = 10)
    pubMotor3 = rospy.Publisher('/robot0/third_wheel_controller/command', Float64, queue_size = 10)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass