import math
import numpy as np

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

    def drive(self, x, y, w, headingOffset = 0):
        del_angle = 360/self.numOfWheels
        motor = [0 for _ in range(self.numOfWheels)]
        for i in range(self.numOfWheels):
            motor[i] = (-x * math.sin((del_angle * i + headingOffset) * math.pi / 180))/self.wheelRadius
            motor[i] += (y * math.cos((del_angle * i + headingOffset) * math.pi / 180))/self.wheelRadius
            motor[i] += (w * self.robotRadius)/self.wheelRadius

        return motor

    def odometry(self, enc, yaw, headingOffset = 0):
        '''Still need to work on this'''
        global l_enc
        global x_pos
        global y_pos

        dx = 0
        dy = 0
        del_angle = 360/self.numOfWheels
        for i in range(self.numOfWheels):
            th = (del_angle * i + headingOffset + yaw) * math.pi / 180
            d = np.int16(enc[i] - l_enc[i])
            # d = enc[i] - l_enc[i]
            dx += d * math.sin(th)
            dy -= d * math.cos(th)

        # print(dx, dy)
        x_pos += dx
        y_pos += dy
        
        l_enc = enc.copy()