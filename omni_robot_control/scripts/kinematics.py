import math
import numpy as np
import rospy

NUMBER_OF_WHEELS = 3
l_enc = [0 for _ in range(NUMBER_OF_WHEELS)]
x_pos = 0
y_pos = 0

class OmniKinematics:
    def __init__(self, numOfWheels, robotRadius, wheelRadius, headingOffset = 0):
        self.numOfWheels = numOfWheels
        self.robotRadius = robotRadius
        self.headingOffset = headingOffset
        self.wheelRadius = wheelRadius

        self.xPos = 0
        self.yPos = 0

        self.lastOdometryTime = 0

    def drive(self, x, y, w, headingOffset = 0):
        del_angle = 360/self.numOfWheels
        motor = [0 for _ in range(self.numOfWheels)]
        for i in range(self.numOfWheels):
            motor[i] = (-x * math.sin((del_angle * i + headingOffset) * math.pi / 180))/self.wheelRadius
            motor[i] += (y * math.cos((del_angle * i + headingOffset) * math.pi / 180))/self.wheelRadius
            motor[i] += (w * self.robotRadius)/self.wheelRadius

        return motor

    def odometry(self, enc, yaw, headingOffset = 0):
        # self.lastOdometryTime = rospy.get_time()
        omega0, omega1, omega2 = enc
        r = self.wheelRadius
        R = self.robotRadius
        h = math.radians(headingOffset)

        w = (r / (3 * R)) * (omega0 + omega1 + omega2)
        # it works
        # x = (2 * r / math.sqrt(3)) * (omega0 * math.cos(2*math.pi/3 + h) - omega1 * math.cos(h) - (omega0 + omega1 + omega2) / 3 * (math.cos(2*math.pi/3 + h) - math.cos(h)))
        # y = (2 * r / math.sqrt(3)) * (omega0 * math.sin(2*math.pi/3 + h) - omega1 * math.sin(h) - (omega0 + omega1 + omega2) / 3 * (math.sin(2*math.pi/3 + h) - math.sin(h)))
        
        # another version 
        x = (2 * r / math.sqrt(3)) * (omega0 * (math.cos(2*math.pi/3 + h) - (math.cos(2*math.pi/3 + h) - math.cos(h)) / 3) + omega1 * (-math.cos(h) - (math.cos(2*math.pi/3 + h) - math.cos(h)) / 3) - omega2 * (math.cos(2*math.pi/3 + h) - math.cos(h)) / 3)
        y = (2 * r / math.sqrt(3)) * (omega0 * (math.sin(2*math.pi/3 + h) - (math.sin(2*math.pi/3 + h) - math.sin(h)) / 3) + omega1 * (-math.sin(h) - (math.sin(2*math.pi/3 + h) - math.sin(h)) / 3) - omega2 * (math.sin(2*math.pi/3 + h) - math.sin(h)) / 3)

        self.xPos += x * math.cos(yaw) - y * math.sin(yaw)
        self.yPos += x * math.sin(yaw) + y * math.cos(yaw)

        return x, y, w

class OmniKinematics2:
    # on development
    def __init__(self, numOfWheels, robotRadius, wheelRadius, tetha = 0):
        self.numOfWheels = numOfWheels
        self.robotRadius = robotRadius
        self.tetha = tetha
        self.wheelRadius = wheelRadius
        self.pos = np.array([0, 0])
        self.lastOdometryTime = 0
        
        self.driveMatrix = np.empty((0, 3))
        for i in range(self.numOfWheels):
            angle = math.radians(360/self.numOfWheels * i)
            self.driveMatrix = np.append(self.driveMatrix, [[-math.sin(angle)/self.wheelRadius, math.cos(angle)/self.wheelRadius, self.robotRadius/self.wheelRadius]], axis = 0)

        self.odometryMatrix = np.linalg.pinv(self.driveMatrix)

    def drive(self, x, y, w, headingOffset = 0):
        velocityVec = np.array([x, y, w])
        motor = np.dot(self.driveMatrix, velocityVec)
        return motor
    
    def odometry(self, enc, yaw, headingOffset = 0):
        # dt = rospy.get_time() - self.lastOdometryTime
        dt = 1
        encVector = np.array(enc)
        velocityVec = np.dot(self.odometryMatrix, encVector)
        matrixRotation2D = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        velocityVecGlobal = np.dot(matrixRotation2D, velocityVec[0:2])
        print("vg", velocityVecGlobal)
        self.pos = self.pos + velocityVecGlobal * dt
        print(self.pos)

        # self.lastOdometryTime = rospy.get_time()
        return self.pos

if __name__ == '__main__':
    # omni2 = OmniKinematics2(3, 0.1, 0.01)
    # [m1, m2, m3] = omni2.drive(10, 0, 0)
    # print(m1, m2, m3)

    # [x, y] = omni2.odometry([m1, m2, m3], 0)
    # print(x, y)

    omni = OmniKinematics(3, 0.1, 0.01)
    [m1, m2, m3] = omni.drive(-50, -10, 0)
    print(m1, m2, m3)

    [x, y, w] = omni.odometry([m1, m2, m3], 0)
    print(x, y)