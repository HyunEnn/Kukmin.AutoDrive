#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random



Offset = 330


# 네모 그리기 
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)   
    return img


# img에서 roi영역을 구하기 위해 마우스 클릭으로 x,y좌표 출력
def onMouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("point : ", x, y)

# 왼쪽 차선 roi
def roi_l(image):
    polygons = np.array([[(120,360),(390,360),(190,420),(1,420)]])
    image_mask = np.zeros_like(image)           # image_mask에 image 크기만큼 0인 배열 생성 
    cv2.fillPoly(image_mask,polygons,255)       # 관심영역 만큼만 255로 설정
    masking_image = cv2.bitwise_and(image,image_mask)   # bitwise연산을 통해 관심 영역만 추출해서 저장
    return masking_image

# 오른쪽 차선 roi
def roi_r(image):
    polygons = np.array([[(290,360),(500,360),(635,420),(340,420)]])
    image_mask = np.zeros_like(image)           # image_mask에 image 크기만큼 0인 배열 생성 
    cv2.fillPoly(image_mask,polygons,255)       # 관심영역 만큼만 255로 설정
    masking_image = cv2.bitwise_and(image,image_mask)   # bitwise연산을 통해 관심 영역만 추출해서 저장
    return masking_image

# 앞쪽 차선 
def roi(image):
    polygons = np.array([[(220,290),(410,290),(430,300),(210,300)]])
    image_mask = np.zeros_like(image)           # image_mask에 image 크기만큼 0인 배열 생성 
    cv2.fillPoly(image_mask,polygons,255)       # 관심영역 만큼만 255로 설정
    masking_image = cv2.bitwise_and(image,image_mask)   # bitwise연산을 통해 관심 영역만 추출해서 저장
    return masking_image

# 평균선 계산 
def average(lines):
    if lines is None:       # 아무것도 없으면 그냥 return
        return
    if (len(lines) == 0):   # 아무것도 없으면 그냥 return
        return
    elif len(lines) == 1:   # 선이 하나면 원래 lines return
        return lines

    num_lines = len(lines)  # lines 개수 파악

    sum_x1 = 0
    sum_y1 = 0
    sum_x2 = 0
    sum_y2 = 0

    # line 개수만큼 sum 변수에 더함 
    for line in lines:
        for x1, y1, x2, y2 in line:
            sum_x1 += x1
            sum_y1 += y1
            sum_x2 += x2
            sum_y2 += y2

    # sum 값을 line 갯수만큼 나눠주어 avg_line 계산 
    avg_line = np.array([[sum_x1/num_lines,sum_y1/num_lines,sum_x2/num_lines,sum_y2/num_lines]])
    return avg_line


#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)


# 화면에서 흰색부분만 추출하기 위한 mask 함수 
# 255, 255, 255 흰색이기 때문에 200, 200, 200 이상인 것만 추출
def mask_img(image, blue_threshold = 200, green_threshold = 200, red_threshold = 200):
    bgr_threshold = [blue_threshold, green_threshold, red_threshold]
    thresholds = (image[:,:,0] < bgr_threshold[0]) \
                | (image[:,:,1] < bgr_threshold[1]) \
                | (image[:,:,2] < bgr_threshold[2]) 
    image[thresholds] = [0,0,0]
    return image

def process_image(frame):
    global Offset
    lpos = 50                   # 차선 검출되지 않을 때 값  
    rpos = 590                  # 차선 검출되지 않을 때 값  

    # masking only white color 
    masking_tmp = frame.copy()  
    mask = mask_img(masking_tmp)
    # cv2.imshow("mask_img", mask)

    src = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)     # gray scale로 변환
    dst = cv2.Canny(src, 50,200,None, 3)            # canny edge로 edge 검출


    # roi divide left, right 
    dst_l = roi_l(dst)      # 관심영역 왼쪽 
    dst_r = roi_r(dst)      # 관심영역 오른쪽 
    
    dst_m = roi(dst)
    # cv2.imshow("roi", dst_m)

    cdst_l = cv2.cvtColor(dst_l, cv2.COLOR_GRAY2BGR) # gray to bgr 
    cdst_r = cv2.cvtColor(dst_r, cv2.COLOR_GRAY2BGR) # gray to bgr 
    cdstp_l = np.copy(cdst_l)   
    cdstp_r = np.copy(cdst_r)    
    cdstp_m = cv2.cvtColor(dst_m,cv2.COLOR_GRAY2BGR)


    # 처음에 houghlines로 시도 
    ############################# houghlines
    # lines= cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0,0)
    # if lines is not None:
    #     for i in range(0, len(lines)):
    #         rho = lines[i][0][0]
    #         theta = lines[i][0][1]
    #         a= math.cos(theta)
    #         b = math.cos(theta)
    #         x0 = a * rho
    #         y0 = b * rho
    #         pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
    #         pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
    #         cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    ######################################################

    # houghlinesP 사용 houghlines 보다 시간이 덜 걸린다. 
    ##########################
    linesP_l = cv2.HoughLinesP(dst_l, 1, np.pi / 180, 40, None, 50, 10)
    linesP_r = cv2.HoughLinesP(dst_r, 1, np.pi / 180, 40, None, 50, 10)

    # 급격한 회전을 방지하기 위해 앞에 있는 차선 검출
    linesP_m = cv2.HoughLinesP(dst_m, 1, np.pi / 180, 10, None, 10, 30)

    if linesP_m is not None:
        for line in linesP_m:                     
            for x1, y1, x2, y2 in line:                   #왼쪽의 개수만큼 반복하면서 x1, y1, x2, y2 검출
                cv2.line(cdstp_m, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)   # 선 그리기

    # cv2.imshow("asd", cdstp_m)
    
    
    if linesP_m is not None: 
        # print(len(linesP_m))
        front_num = len(linesP_m)
    if linesP_m is None:
        front_num = 0
    ###########################

    ############################ roi create only one 
    # linesP = cv2.HoughLinesP(dst_m, 1, np.pi / 180, 20, 30, 40)
    # test_lines = display(frame, linesP)
    # cv2.imshow("test_img", test_lines)

    # 선들의 평균을 구해서 하나의 선만 구함 
    left_line = average(linesP_l)
    right_line = average(linesP_r)

    # avg_lines 에 왼쪽 선, 오른쪽 선 추가 
    avg_lines = []
    if(left_line is not None):
        avg_lines.append(left_line)
    if(right_line is not None):
        avg_lines.append(right_line)

    # print(avg_lines)
    # print(avg_lines_l)average_line
    # print(avg_lines_r)
    # print(linesP_l[0], linesP_l[1])


    #처음에는 average를 사용하지않고 여러개의 선검출 
    #################### multi lines 
    # print(len(linesP_l))
    # if linesP_l is not None:
    #     for line in linesP_l:                     
    #         for x1, y1, x2, y2 in line:                   #왼쪽의 개수만큼 반복하면서 x1, y1, x2, y2 검출
    #             slope = float(y2-y1) / float(x2-x1)       # 기울기 계산 
    #             # print(slope)        
    #             if(slope) <= 0:                           # 기울기가 0보다 작으면 왼쪽선으로 인식 
    #                 cv2.line(cdstp_l, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)   # 선 그리기
    #                 lpos = (x1 + x2) /2                   # lpos 를 (x1 + x2) / 2 로 반환 
    
    # if linesP_r is not None:                          
    #     for line in linesP_r:                 
    #         for x1, y1, x2, y2 in line:                   #오른쪽의 개수만큼 반복하면서 x1, y1, x2, y2 검출
    #             slope = float(y2-y1) / float(x2-x1)       # 기울기 계산 
    #             if(slope) >= 0:                           # 기울기가 0보다 크면 오른쪽선으로 인식 
    #                 cv2.line(cdstp_r, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)       # 선 그리기
    #                 rpos = (x1 + x2 )/ 2                  # rpos 를 (x1 + x2) / 2 로 반환 

    ####################### only one line
    # print(linesP_l.shape)
    # print(avg_lines_l.shape)



    if avg_lines is not None:
        for line in avg_lines:
            x1, y1, x2, y2 = line.reshape(4)        # avg lines의 x1,y1,x2,y2 
            slope = float(y2-y1) / float(x2-x1)     # 기울기 계산 
            if(slope) <= 0:                         # 기울기가 0보다 작으면 왼쪽선으로 인식 
                cv2.line(cdstp_l, (x1,y1),(x2,y2), (0,0,255),15,cv2.LINE_AA)    # 선 그리기
                if rpos > lpos:
                    lpos = (x1 + x2) / 2                 # lpos 를 (x1 + x2) / 2 로 반환
            else:                                   # 기울기가 0보다 크면 오른쪽선으로 인식 
                cv2.line(cdstp_r, (x1,y1),(x2,y2), (0,0,255),15,cv2.LINE_AA)    # 선 그리기
                if lpos < rpos:
                    rpos = (x1 + x2 )/ 2                # rpos 를 (x1 + x2) / 2 로 반환


         
    # 검출된 선 확인 
    # cv2.imshow("left line show", cdstp_l)
    # cv2.imshow("right line show", cdstp_r)

    cdstp_add =cv2.add(cdstp_l, cdstp_r)                        # 왼쪽 선 오른 쪽 선 합치기 
    combo_image = cv2.addWeighted(frame, 0.9, cdstp_add,1,1)      # 카메라 img와 검출된 선의 img 합치기 
    # cv2.imshow("line show", cdstp_add)
    #cv2.imshow("lines", line_image)
    #print(x1," ", y1," ",x2, " ",y2)
    # print("lpos =", lpos, "rpos = ", rpos)

    combo_image = draw_rectangle(combo_image, lpos, rpos, offset=Offset)

    return (lpos, rpos), front_num, combo_image





#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================
def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown():

        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
        img = image.copy()  
        pos,num, frame = process_image(img)
        

        # 디버깅을 위해 모니터에 이미지를 디스플레이
        cv2.imshow("CAM View", frame)
        cv2.setMouseCallback('CAM View', onMouse)
        cv2.waitKey(1)       
       
        #=========================================
        # 핸들조향각 값인 angle값 정하기.
        # 차선의 위치 정보를 이용해서 angle값을 설정함.        
        #=========================================

        # 우선 테스트를 위해 직진(0값)으로 설정
        speed = 10


        #=========================================
        # 차량의 속도 값인 speed값 정하기.
        # 직선 코스에서는 빠른 속도로 주행하고 
        # 회전구간에서는 느린 속도로 주행하도록 설정함.
        #=========================================
         
        angle = (pos[0] +pos[1])/2 -320         # 중심으로 부터 얼마나 떨어져있는지 계산하기 위해 320을 빼줌
        # print(angle)
        angle = math.atan2(angle, 240)*180 /np.pi       # 각도 계산 atan를 이용하여 계산 
        # print(angle)

        # 각도에 따라 속도 조절 
        if abs(angle) < 6:
            angle = 0
            speed = 25
            if num < 4:         # 앞에 차선 검출이 일정수준 이하이면 회전으로
                speed = 15
                # print(num, "find front curved road")
        if abs(angle) < 7.5:
            agnle = angle * 0.9            # 안정적 주행을 위해 각도가 작으면 angle 수정을 최소화
            speed = 20
        elif abs(angle) <14:
            speed = 15
            angle = angle * 0.8
        elif abs(angle) < 20:
            speed = 13
            # print(angle)
            angle = angle * 0.7
        elif abs(angle) < 25:
            speed = 10
            # print(angle, '30')
            angle = angle * 0.75
        else:                               # 차선 이탈방지를 위해 차선이 너무 가까우면 핸들을 더 꺽어줌
            speed = 8
            angle = angle * 0.85
        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()

