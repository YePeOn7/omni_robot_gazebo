#!/usr/bin/env python3
import rospy, os, sys, math, time
import numpy as np
import cv2

from std_msgs.msg import Header, String, Int32, Float64
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge() #make an instance

vel_x = 0
vel_y = 0
vel_theta = 0

font = cv2.FONT_HERSHEY_SIMPLEX

H = 480  #camera image width
W = 640  #cameta image height

Gx = W/2   #initial value of robot center in x direction
angle = 0  #initial value of robot angle

lft = 64 #lower filter threshold
uft = H - 64 #upper filter threshold

Black_filter_1 = np.zeros((H, W), np.uint8)
for i in range(H):
    if(i > 100 and i <= 153):
        Black_filter_1[i,:] = 255
    else:
        Black_filter_1[i,:] = 0

Black_filter_2 = np.zeros((H, W), np.uint8)
for j in range(H):
    if(j > 153 and j <= 206):
        Black_filter_2[j,:] = 255
    else:
        Black_filter_2[j,:] = 0

Black_filter_3 = np.zeros((H, W), np.uint8)
for i in range(H):
    if(i > 206 and i <= 260):
        Black_filter_3[i,:] = 255
    else:
        Black_filter_3[i,:] = 0

Black_filter_4 = np.zeros((H, W), np.uint8)
for j in range(H):
    if(j > 260):
        Black_filter_4[j,:] = 255
    else:
        Black_filter_4[j,:] = 0

def callback_color_img(data):
    global font, Gx, angle, H, W
    cv_color_image = bridge.imgmsg_to_cv2(data, "bgr8")
    #cv2.imshow("COLOR_IMAGE", cv_color_image)
    ret, img_thresh = cv2.threshold(cv_color_image, 20, 255, cv2.THRESH_BINARY)
    #cv2.imshow("IMAGE", img_thresh)
    image2 = cv2.bitwise_not(img_thresh)
    kernel = np.ones((5,5),np.uint8)
    erosion = cv2.erode(image2, kernel, iterations = 2)
    img_gray = cv2.cvtColor(erosion, cv2.COLOR_BGR2GRAY)

    #calculate center coordinates of the line
    G = cv2.moments(img_gray, False)
    if G["m00"] != 0:
        Gx, Gy = int(G["m10"]/G["m00"]) , int(G["m01"]/G["m00"])
    else:
        Gx = 0
        Gy = 0

    #calculate filter1
    result_1 = cv2.bitwise_and(img_gray, Black_filter_1)
    G1 = cv2.moments(result_1, False)
    if G1["m00"] != 0:
        Gx_1, Gy_1 = int(G1["m10"]/G1["m00"]) , int(G1["m01"]/G1["m00"])
    else:
        Gx_1 = 0
        Gy_1 = 0

    #calculate filter2
    result_2 = cv2.bitwise_and(img_gray, Black_filter_2)
    G2 = cv2.moments(result_2, False)
    if G2["m00"] != 0:
        Gx_2, Gy_2 = int(G2["m10"]/G2["m00"]) , int(G2["m01"]/G2["m00"])
    else:
        Gx_2 = 0
        Gy_2 = 0

    #calculate filter3
    result_3 = cv2.bitwise_and(img_gray, Black_filter_3)
    G3 = cv2.moments(result_3, False)
    if G3["m00"] != 0:
        Gx_3, Gy_3 = int(G3["m10"]/G3["m00"]) , int(G3["m01"]/G3["m00"])
    else:
        Gx_3 = 0
        Gy_3 = 0

    #calculate filter4
    result_4 = cv2.bitwise_and(img_gray, Black_filter_4)
    G4 = cv2.moments(result_4, False)
    if G4["m00"] != 0:
        Gx_4, Gy_4 = int(G4["m10"]/G4["m00"]) , int(G4["m01"]/G4["m00"])
    else:
        Gx_4 = 0
        Gy_4 = 0

    #show each point 
    #cv2.circle(erosion, (Gx, Gy), radius = 6, color = (0, 255, 0), thickness=-1)
    #cv2.putText(erosion, text='G : ' + str(Gx) + ", " + str(Gy), org=(Gx + 5 , Gy + 5), fontFace=font, fontScale = 1, color=(0, 255, 0), thickness = 2, lineType=cv2.LINE_AA)
    if(Gx_1 !=0 and Gy_1 != 0):
        cv2.circle(erosion, (Gx_1, Gy_1), radius = 6, color = (255, 0, 0), thickness=-1)
    #cv2.putText(erosion, text='G_1 : ' + str(Gx_1) + ", " + str(Gy_1), org=(Gx_1 + 5 , Gy_1 + 5), fontFace=font, fontScale = 1, color=(255, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.circle(erosion, (Gx_2, Gy_2), radius = 6, color = (0, 0, 255), thickness=-1)
    #cv2.putText(erosion, text='G_2 : ' + str(Gx_2) + ", " + str(Gy_2), org=(Gx_2 + 5 , Gy_2 + 5), fontFace=font, fontScale = 1, color=(0, 0, 255), thickness = 2, lineType=cv2.LINE_AA)
    cv2.circle(erosion, (Gx_3, Gy_3), radius = 6, color = (128, 0, 128), thickness=-1)
    #cv2.putText(erosion, text='G_3 : ' + str(Gx_3) + ", " + str(Gy_3), org=(Gx_3 + 5 , Gy_3 + 5), fontFace=font, fontScale = 1, color=(128, 128, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.circle(erosion, (Gx_4, Gy_4), radius = 6, color = (0, 128, 128), thickness=-1)
    #cv2.putText(erosion, text='G_4 : ' + str(Gx_4) + ", " + str(Gy_4), org=(Gx_4 + 5 , Gy_4 + 5), fontFace=font, fontScale = 1, color=(0, 128, 128), thickness = 2, lineType=cv2.LINE_AA)
    if(Gx_1 !=0 and Gy_1 != 0):    
        cv2.line(erosion, (Gx_1, Gy_1), (Gx_2, Gy_2), (128, 128, 0), thickness=3)
    cv2.line(erosion, (Gx_2, Gy_2), (Gx_3, Gy_3), (128, 128, 0), thickness=3)
    cv2.line(erosion, (Gx_3, Gy_3), (Gx_4, Gy_4), (128, 128, 0), thickness=3)
    cv2.imshow("COLOR_IMAGE", erosion)

    #calculate angle
    if(Gx_1 != 0 and Gy_1 != 0):
        denominator = Gy_1-Gy_4
        if(denominator == 0):
            print("denominator is 0")
            denominator = 1
        angle = math.atan(float(Gx_1-Gx_4)/denominator)
    elif(Gx_1 == 0 and Gy_1 == 0):
        denominator = Gy_2-Gy_4
        if(denominator == 0):
            print("denominator is 0")
            denominator = 1
        angle = math.atan(float(Gx_2-Gx_4)/denominator)
    elif(Gx_1 == 0 and Gy_1 == 0 and Gx_2 == 0 and Gy_2 == 0):
        denominator = Gy_3-Gy_4
        if(denominator == 0):
            print("denominator is 0")
            denominator = 1
        angle = math.atan(float(Gx_3-Gx_4)/denominator)

    cv2.waitKey(4)

def commander():
    global  angle, Gx, W    # gloabl variable load

    alpha = math.pi/2 
    L = 0.088

    Kp_a = 0.4
    Ki_a = 0.13
    Kd_a = 0.0023
    angle_disp = 0.0
    angle_disp_pre = 0.0

    Kp = 0.001
    Ki = 0.0005
    Kd = 0 
    x_disp = 0.0
    x_disp_pre = 0.0
    rot = 0
    pos = 0

    time_step = float(1) / 30
    V = 0
    vel_x = 0
    vel_y = 0

    pub1 = rospy.Publisher('/robot0/first_wheel_controller/command', Float64, queue_size = 10)
    pub2 = rospy.Publisher('/robot0/second_wheel_controller/command', Float64, queue_size = 10)
    pub3 = rospy.Publisher('/robot0/third_wheel_controller/command', Float64, queue_size = 10)
    pub4 = rospy.Publisher('/robot0/camera_bracket_controller/command', Float64, queue_size = 10)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        x_disp_pre = x_disp 
        x_disp = W/2 - Gx
        vel_x = Kp*x_disp + Ki*(x_disp + x_disp_pre)*time_step/2 + Kd*(x_disp - x_disp_pre)/time_step
        vel_x = -vel_x

        pos = 0

        angle_disp_pre = angle_disp
        angle_disp = 0 - angle
        vel_theta = Kp_a*angle_disp + Ki_a*(angle_disp + angle_disp_pre)*time_step/2 + Kd_a*(angle_disp - angle_disp_pre)/time_step
        vel_theta = -vel_theta

        vel_y = 0.20

        vel1 = 50*(-math.sin(alpha)*vel_x + math.cos(alpha)*vel_y + L*vel_theta)
        vel2 = 50*(-math.sin(alpha + 2*math.pi/3)*vel_x + math.cos(alpha + 2*math.pi/3)*vel_y + L*vel_theta)
        vel3 = 50*(-math.sin(alpha - 2*math.pi/3)*vel_x + math.cos(alpha - 2*math.pi/3)*vel_y + L*vel_theta)

        pub1.publish(vel1)  # publish topic to control wheel vel
        pub2.publish(vel2)
        pub3.publish(vel3)
        pub4.publish(pos)

        try:    # error check
            rate.sleep()
        except rospy.ROSException:
            print("restart simulation")

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)    # init ros node
    rospy.Subscriber("/robot0/main_camera/image_raw", Image, callback_color_img)    # subscribe the image topic
    commander() # main func
    rospy.spin()    # spin func

