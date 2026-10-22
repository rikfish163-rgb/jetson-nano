#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math

from PIL import Image

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
#from smartcar.msg import light
intrinsicMat = []
distortionCoe = []
prspct_trans_mat = []
perspective_transform_matrix = []
kernel = []

#not detect the green light
starter = True
#ignore the lidar
global lidarLaunch
lidarLaunch = False
##roadWidth = 65.0 /(80/300.0)
buf = [0,0,0]
count = 0
class KalmanFilter:

    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted

def initial_parameters():
    global intrinsicMat
    global distortionCoe
    global perspective_transform_matrix
    global perspective_transform_matrix_slope
    global kernel
    global ramp_control
    global pp_control
    global part_A

    ##ramp_control = True

    intrinsicMat = np.array([[726.5936, 0, 700.6015],
                            [0, 730.8660, 398.9269],
                            [0, 0, 1]])

    distortionCoe = np.array([-0.3770,0.1478,0,0, -0.0282])
   
    startx = -200
    starty = 0
    length_pers = 1280
    width_pers = 500
    srcps = np.float32([[(449,289), (65,536), (1268,556), (920,300)]])
 
    dstps = np.float32([[(startx, starty), (startx, starty + width_pers), (startx + length_pers, starty + width_pers), (startx + length_pers, starty)]])
    perspective_transform_matrix = cv2.getPerspectiveTransform(srcps, dstps)
    


    startx = 170
    starty = -600
    length_pers = 500
    width_pers = 1080#length_pers
    srcps = np.float32([[(176,271), (17,472), (534,448), (416,267)]])#[(176,271), (17,472), (534,448), (416,267)]
    
    dstps = np.float32([[(startx, starty), (startx, starty + width_pers), (startx + length_pers, starty + width_pers), (startx + length_pers, starty)]])
    perspective_transform_matrix_slope = cv2.getPerspectiveTransform(srcps, dstps)

    kernel = np.ones((3,3),np.uint8)



def perspectiveTrans(img):

    global perspective_transform_matrix    

    if perspective_transform_matrix==[]:
        print"Transform failed!"
        return img
    else:
        bird_view_img = cv2.warpPerspective(img, perspective_transform_matrix, img.shape[1::-1], flags=cv2.INTER_LINEAR)
         
        return bird_view_img

def perspectiveTrans_slope(img):

    #global perspective_transform_matrix_slope    

    if perspective_transform_matrix_slope==[]:
        print"Transform failed!"
        return img
    else:
        bird_view_img = cv2.warpPerspective(img, perspective_transform_matrix_slope, (500,500))
        return bird_view_img
def region_of_interest(img, vertices):
    #定义一个和输入图像同样大小的全黑图像mask，这个mask也称掩膜
    #掩膜的介绍，可参考：https://www.cnblogs.com/skyfsm/p/6894685.html
    mask = np.zeros_like(img)   
 
    #根据输入图像的通道数，忽略的像素点是多通道的白色，还是单通道的白色
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255


    #[vertices]中的点组成了多边形，将在多边形内的mask像素点保留，
    cv2.fillPoly(mask, [vertices], ignore_mask_color)
 
    #与mask做"与"操作，即仅留下多边形部分的图像
    masked_image = cv2.bitwise_and(img, mask)


    return masked_image


def img_hsv2gray(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # hsv色彩空间 CV_HSV2BGR
    lower_white = np.array([0, 0, 200])  
    upper_white = np.array([180, 30, 255]) 
    mask_white = cv2.inRange(hsv, lower_white, upper_white)  # 转换为hsv空间，去除背景
    # mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.bitwise_and(img,img, mask = mask_white)
    mask2gray = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
    # cv2.imshow('mask2gray',mask2gray)
    mask2gray[mask2gray<220] = 0
    return mask2gray

def holefilling (img):
    imgray = img
    

    # 原图取补得到MASK图像
    mask = 255 - imgray

    # 构造Marker图像
    marker = np.zeros_like(imgray)
    marker[0, :] = 255
    marker[-1, :] = 255
    marker[:, 0] = 255
    marker[:, -1] = 255
    marker_0 = marker.copy()

    # 形态学重建
    SE = cv2.getStructuringElement(shape=cv2.MORPH_CROSS, ksize=(3, 3))
    while True:
        marker_pre = marker
        dilation = cv2.dilate(marker, kernel=SE)
        marker = np.min((dilation, mask), axis=0)
        if (marker_pre == marker).all():
            break
    dst = 255 - marker
    filling = dst - imgray
    return dst


def light_detection(origin_img):
    global stop_judge_local
    global ramp_control
    global pp_control
    #global part_A
    #global lidarLaunch

    #c1 = cv2.getTickCount()
    #judge1 = self.line_detection(img)    #判断是否存在停车线
    judge1 =True                       
    hsv=cv2.cvtColor(origin_img,cv2.COLOR_BGR2HSV)   #转化为HSV格式图像，更好得处理光线对图像的影响。（由于处理器处理能力限制那个1w的摄像头没法用，读入的图像只有5帧以内，调节不及时）
    hsv1 = hsv.copy()
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    red_lower = np.array([0,95,230])        #这两个是阈值，红灯的上界和下界
    red_upper = np.array([5,255,255])
    red_mask = cv2.inRange(hsv,red_lower,red_upper)
    red_target = cv2.bitwise_and(hsv,hsv,mask = red_mask)
    red_target = cv2.erode(red_target,element)
    red_target = cv2.dilate(red_target,element)
    red_gray = cv2.cvtColor(red_target,cv2.COLOR_BGR2GRAY)
    r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)
    r_gray2 = cv2.Canny(r_binary, 100, 200) 
    r = r_gray2[:,:] == 255
    count_red = len(r_gray2[r])
    if count_red>700:
        redLight = 1
    else:
        redLight = 0

    green_lower = np.array([80,95,230])    #这个阈值有问题,在寝室调不出来
    green_upper = np.array([130,255,255])
    green_mask = cv2.inRange(hsv1,green_lower,green_upper)
    green_target = cv2.bitwise_and(hsv1,hsv1,mask = green_mask)
    green_target = cv2.erode(green_target,element)
    green_target = cv2.dilate(green_target,element)
    green_gray = cv2.cvtColor(green_target,cv2.COLOR_BGR2GRAY)
    g_ret,g_binary = cv2.threshold(green_mask,127,255,cv2.THRESH_BINARY)
    g_gray2 = cv2.Canny(g_binary, 100, 200)       
    g = g_gray2[:,:] == 255
    count_green = len(g_gray2[g])
    if count_green>700:
        greenLight = 1
    else:
        greenLight = 0

    print 'reddots %d'%(count_red)
    print 'greendots %d'%(count_green)

    if greenLight + redLight >0 :
        hasLight = 1
    if redLight == 1:
        Light = 2
    elif greenLight == 1:
        Light = 1       
    else:
        hasLight = 0
        Light = 0
    global count_light
    '''
        global all_time
        

        if time.time()-all_time <= 5 and hasLight == 1:
            all_time = time.time()
        if time.time()-all_time > 5 and hasLight ==1:
            count_light = count_light + 1
            all_time = time.time()
        print(time.time()-all_time) 
    '''
    if count_light ==0:
        count_light = 1
    print(count_light)                                     
    red_or_green = Light
    light_show_time = count_light ##yong yu huitiao han shu 
    if light_show_time >1 and red_or_green ==1:
         pp_control = True
##    if light_msg.red_or_green ==1 and light_msg.show_time ==1:
##        part_A = 1

    ## red light,stop
    if red_or_green ==2 :   
        stop_judge_local = True
    # green light,go
    elif red_or_green ==1 :   
        stop_judge_local = False
   # else:
   #     stop_judge_local = False
def blind_detection(gray):
    gray_Blur = cv2.GaussianBlur(gray, (5,5),0)
    
    rows,cols=gray_Blur.shape
   
    for i in range(rows):
        for j in range(cols):
            if (gray_Blur[i,j]<=255 and gray_Blur[i,j]>=0):
                gray_Blur[i,j]=0
            
    canny = cv2.Canny(gray_Blur, 100, 200)
    
    lines = cv2.HoughLines(canny, 1, np.pi/180, 100 )
    
    #print lines.shape

    y_nearest = 0

    for line in lines:
        rho, theta = line[0]
        if theta > np.pi/2 + 5*np.pi/180 or theta < np.pi/2 - 5*np.pi/180:
            continue
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = 0
        y1 = int(rho/b)
        x2 = gray.shape[1] - 1
        y2 = int((rho-x2*a) / b)

        if y1 > y_nearest:
            y_nearest = y1
        if y2 > y_nearest:
            y_nearest = y2
                
        if y_nearest > gray.shape[0]*9/10:
            blind_detected = True
            return blind_detected, y_nearest

    blind_detected = False
    return blind_detected, y_nearest

class command:
    def __init__(self):
        count = 0
        #摄像头端口号
        self.cap = cv2.VideoCapture(0)
        self.pubI = rospy.Publisher('images', Image, queue_size=1)
         
        self.pubtest = rospy.Publisher('test_images', Image, queue_size=1)
        self.pubtest2 = rospy.Publisher('test_images2', Image, queue_size=1)

        self.puborignialI = rospy.Publisher('orignial_images', Image, queue_size=1)
        rospy.init_node('command_core', anonymous=True)
        #rospy.Subscriber("redLight", light, RLcallback)
        rospy.Subscriber("laser_cmd", Twist, LScallback)
        self.rate = rospy.Rate(20)

        self.cvb = CvBridge()        

    def spin(self):
        global lidarLaunch
        threshold_value = 50
        global starter,pub,greenLight,aP,lastP, kernel
        global aP_kf
        global last_lane_base, last_LorR
        global final_cmd, cam_cmd, stop_judge_local
        global roadWidth
        global judge_end_tunnel
	last_lane_base = -1
        last_LorR = 1
        y_nearest = 0
        ##ramp_control = True          ################
        while not rospy.is_shutdown():
            ret, img = self.cap.read()
            if ret == True:
                if (not lidarLaunch):  
                    c1 = cv2.getTickCount()
                    kfObj = KalmanFilter()
        	    predictedCoords = np.zeros((2, 1), np.float32)
                    #img = cv2.pyrDown(img)
                    undstrt = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
                    #cv2.imwrite('001.png',binary_warped)
                    self.puborignialI.publish(self.cvb.cv2_to_imgmsg(undstrt))                  

        	    #gray0 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    gray = cv2.cvtColor(undstrt, cv2.COLOR_BGR2GRAY)
                    #gray = img_hsv2gray(undstrt)
                    global blind_detected
                    global blind_detection_flag
                    global delay_flag
                    print ".........Track......."
                    if pp_control == True:
                        cam_cmd.angular.x = 1
                    #gray[:gray.shape[0]/3,:] = 0

                    #gray_Blur = cv2.GaussianBlur(gray, (3,3),0)
                    gray_Blur = cv2.medianBlur(gray,3)     
                    origin_thr = cv2.Canny(gray_Blur, 150 , 300) #100, 200 75,200
                    #origin_thr = gray_Blur
                    origin_thr = cv2.dilate(origin_thr, np.ones((5,5), np.uint8))
                    #origin_thr = holefilling(origin_thr)  
                    origin_thr = cv2.erode(origin_thr, np.ones((5,5), np.uint8))  
                    origin_thr = cv2.dilate(origin_thr, np.ones((5,5), np.uint8))                 
                    left_bottom = [0, origin_thr .shape[0]]
                    right_bottom = [1050, origin_thr .shape[0]]
                    left_up = [650,250]
                    right_up = [850, 250]      
                    apex = [660,250]
                    self.pubtest.publish(self.cvb.cv2_to_imgmsg(cv2.dilate(perspectiveTrans(origin_thr), np.ones((5,5), np.uint8)) ))
                    vertices = np.array([ left_bottom, right_bottom, apex  ], np.int32)
                    origin_thr = region_of_interest(origin_thr, vertices)
                    self.pubtest2.publish(self.cvb.cv2_to_imgmsg(origin_thr))
                    binary_warped =  perspectiveTrans(origin_thr)
                    #cv2.imwrite('000.png',binary_warped)
                    lines = cv2.HoughLinesP(binary_warped,1,np.pi/180,100,100,10)
                    if lines is not None :
                        for x1,y1,x2,y2 in lines[0]:
                            cv2.line(binary_warped,(x1,y1),(x2,y2),(255,0,0),1)
                           

                    try:
                        for i in range(binary_warped.shape[1]/5):
                            x = 5*i
                            px = [x1,x2];
                            py = [y1,y2];
                            c1,c0 = np.polyfit(px, py, 1);
                            y = int(c1*x + c0)
                            cv2.circle(binary_warped, (x, y), 3, (255, 0, 0), -1)                                                                                            
                   except:
                        cam_cmd.angular.z = 0;
                        final_cmd = cam_cmd
                        pub.publish(final_cmd)
                        print "bbbbbbbbbbbbbbbbbbbbbbbbbbb"
                        continue

                  steerAngle = math.atan(c1)
		  print temp_angular_z
		  print steerAngle
                  if c1 != 0:
                      #buf[2] = buf[1]
                      #buf[1] = buf[0]
                      #buf[0] = -1/c1*0.26 np.sum(buf)-np.max(buf)-np.min(buf)
                      temp = 0
                      ang_z = -1/c1*0.14
                      if (ang_z-temp> 0.5)|(temp-ang_z> 0.5) :
                          cam_cmd.angular.z = temp
                      else :
                          cam_cmd.angular.z = ang_z
                      temp = ang_z
                    '''
                    if (cam_cmd.angular.z<=0.05)&(cam_cmd.angular.z>=-0.05):
                        cam_cmd.angular.z = 0
                    '''
		    
                    print 'steerAngle: %.5f'%(steerAngle)
                    print cam_cmd.angular.z
                    #print(k*steerAngle*180/3.14)
                    light_detection(img)
                    if stop_judge_local == True:
                        cam_cmd.linear.x = 0.0 
                    else:
                        cam_cmd.linear.x = 0.3
                    
                    print("linear111111111111")
                    #rospy.spinOnce()

                    #if judge_end_tunnel == 0 and math.fabs(cam_cmd.angular.z) > 0.13:  ##  changeable
                     #   cam_cmd.angular.z = 0.0                                        ##  changeable
                      
                    final_cmd = cam_cmd
                    print final_cmd.angular.z

                    pub.publish(final_cmd)
                    self.pubI.publish(self.cvb.cv2_to_imgmsg(binary_warped))
                    c2 = cv2.getTickCount()
                    ##print 'time'
                    print final_cmd.angular.z
                    ##print (c2 - c1)/cv2.getTickFrequency()
                    
                else:
                    pub.publish(final_cmd)
                    print 'Lidar'
                    print 'final_z %.5f'%(final_cmd.angular.z) 
                       
            self.rate.sleep()

        self.cap.release()
'''
def RLcallback(light_msg):
    global stop_judge_local
    global ramp_control
    global pp_control
    global part_A
    global lidarLaunch
    if light_msg.show_time >1 and light_msg.red_or_green ==1:
         pp_control = True

##    if light_msg.red_or_green ==1 and light_msg.show_time ==1:
##        part_A = 1
    print "green",light_msg.red_or_green
    if light_msg.red_or_green ==1 :   ##changeable
        stop_judge_local = True
    elif light_msg.red_or_green ==0 :   ##changeable
        stop_judge_local = False

   # else:
   #     stop_judge_local = False
'''


def LScallback(laser_cmd):
    global stop_judge_local, lidarLaunch
    global final_cmd
    global judge_end_tunnel
    judge_end_tunnel =  laser_cmd.linear.y
    if stop_judge_local == True:
        laser_cmd.linear.x = 0
        final_cmd = laser_cmd

    if laser_cmd.linear.z >= 0:  #laser control steer 
        lidarLaunch = True 
        final_cmd = laser_cmd
          
    else:
        lidarLaunch = False
        
        pass
    
        

if __name__ == '__main__':
    #device = rospy.get_param('device', 1)
    #width = rospy.get_param('width', 1280)
    #height = rospy.get_param('height', 720)
    #rates = rospy.get_param('rates', 10)
    global blind_detected
    blind_detected = False
    global blind_detection_flag
    blind_detection_flag = False
    global delay_flag
    delay_flag = False
    global ramp_control
    ramp_control = False
    global pp_control
    pp_control = False
    global part_A
    part_A = 0
    global judge_end_tunnel 
    judge_end_tunnel = 0
    global count_light
    count_light=2
    
    initial_parameters()
    #距离映射
    x_cmPerPixel = 61.5/400.0#90/400.0
    y_cmPerPixel = 61.5/400.0#90/400.0
    roadWidth = 70.0 / x_cmPerPixel #80.0 
    


    aP = [0.0, 0.0]
    lastP = [0.0, 0.0]
    aP_kf=[0.0,0.0]
    Timer = 0

    #轴间距
    I = 28
    #图像坐标系底部与车后轮轴心间距 #cm
    D = 67
    #计算cmdSteer的系数，舵机转向与之前相反，此处用正数
    #3.6
    k1 =3.5 #2.65		 # 3.6
    k2 = 3.5 #2.65
    kd1 = 10
    kd2 = 10		
    #steerAngle, cmdSteer;
    temp_angular_z=0
    global final_cmd, cam_cmd #laser_cmd = Twist()
    final_cmd = Twist()
    
    cam_cmd = Twist()
    ##cam_cmd.linear.x = 0.2
    
    global stop_judge_local
    stop_judge_local = False
    #stop_judge_local = True
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    try:
        cmd = command()
        cmd.spin()
        rospy.spin()
        final_cmd = cam_cmd
        

    except rospy.ROSInterruptException:
        pass
