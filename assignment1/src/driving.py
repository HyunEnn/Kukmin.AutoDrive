#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# �Բ� ���Ǵ� ���� ���̽� ��Ű������ import �����
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


# �׸� �׸��� 
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)   
    return img


# img���� roi������ ���ϱ� ���� ���콺 Ŭ������ x,y��ǥ ���
def onMouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("point : ", x, y)

# ���� ���� roi
def roi_l(image):
    polygons = np.array([[(120,360),(390,360),(190,420),(1,420)]])
    image_mask = np.zeros_like(image)           # image_mask�� image ũ�⸸ŭ 0�� �迭 ���� 
    cv2.fillPoly(image_mask,polygons,255)       # ���ɿ��� ��ŭ�� 255�� ����
    masking_image = cv2.bitwise_and(image,image_mask)   # bitwise������ ���� ���� ������ �����ؼ� ����
    return masking_image

# ������ ���� roi
def roi_r(image):
    polygons = np.array([[(290,360),(500,360),(635,420),(340,420)]])
    image_mask = np.zeros_like(image)           # image_mask�� image ũ�⸸ŭ 0�� �迭 ���� 
    cv2.fillPoly(image_mask,polygons,255)       # ���ɿ��� ��ŭ�� 255�� ����
    masking_image = cv2.bitwise_and(image,image_mask)   # bitwise������ ���� ���� ������ �����ؼ� ����
    return masking_image

# ���� ���� 
def roi(image):
    polygons = np.array([[(220,290),(410,290),(430,300),(210,300)]])
    image_mask = np.zeros_like(image)           # image_mask�� image ũ�⸸ŭ 0�� �迭 ���� 
    cv2.fillPoly(image_mask,polygons,255)       # ���ɿ��� ��ŭ�� 255�� ����
    masking_image = cv2.bitwise_and(image,image_mask)   # bitwise������ ���� ���� ������ �����ؼ� ����
    return masking_image

# ��ռ� ��� 
def average(lines):
    if lines is None:       # �ƹ��͵� ������ �׳� return
        return
    if (len(lines) == 0):   # �ƹ��͵� ������ �׳� return
        return
    elif len(lines) == 1:   # ���� �ϳ��� ���� lines return
        return lines

    num_lines = len(lines)  # lines ���� �ľ�

    sum_x1 = 0
    sum_y1 = 0
    sum_x2 = 0
    sum_y2 = 0

    # line ������ŭ sum ������ ���� 
    for line in lines:
        for x1, y1, x2, y2 in line:
            sum_x1 += x1
            sum_y1 += y1
            sum_x2 += x2
            sum_y2 += y2

    # sum ���� line ������ŭ �����־� avg_line ��� 
    avg_line = np.array([[sum_x1/num_lines,sum_y1/num_lines,sum_x2/num_lines,sum_y2/num_lines]])
    return avg_line


#=============================================
# �͹̳ο��� Ctrl-C Ű�Է����� ���α׷� ������ ���� ��
# �� ó���ð��� ���̱� ���� �Լ�
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# ���α׷����� ����� ��� �����
#=============================================
image = np.empty(shape=[0]) # ī�޶� �̹����� ���� ����
bridge = CvBridge() 
motor = None # ���� ������ ���� ����

#=============================================
# ���α׷����� ����� ��� �����
#=============================================
CAM_FPS = 30    # ī�޶� FPS - �ʴ� 30���� ������ ����
WIDTH, HEIGHT = 640, 480    # ī�޶� �̹��� ����x���� ũ��

#=============================================
# �ݹ��Լ� - ī�޶� ������ ó���ϴ� �ݹ��Լ�
# ī�޶� �̹��� ������ �����ϸ� �ڵ����� ȣ��Ǵ� �Լ�
# ���ȿ��� �̹��� ������ ���� image ������ �Ű� ����.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# ���� ������ �����ϴ� �Լ�  
# �Է����� ���� angle�� speed ���� 
# ���� ���ȿ� �Ű� ���� �Ŀ� ������ ������.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)


# ȭ�鿡�� ����κи� �����ϱ� ���� mask �Լ� 
# 255, 255, 255 ����̱� ������ 200, 200, 200 �̻��� �͸� ����
def mask_img(image, blue_threshold = 200, green_threshold = 200, red_threshold = 200):
    bgr_threshold = [blue_threshold, green_threshold, red_threshold]
    thresholds = (image[:,:,0] < bgr_threshold[0]) \
                | (image[:,:,1] < bgr_threshold[1]) \
                | (image[:,:,2] < bgr_threshold[2]) 
    image[thresholds] = [0,0,0]
    return image

def process_image(frame):
    global Offset
    lpos = 50                   # ���� ������� ���� �� ��  
    rpos = 590                  # ���� ������� ���� �� ��  

    # masking only white color 
    masking_tmp = frame.copy()  
    mask = mask_img(masking_tmp)
    # cv2.imshow("mask_img", mask)

    src = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)     # gray scale�� ��ȯ
    dst = cv2.Canny(src, 50,200,None, 3)            # canny edge�� edge ����


    # roi divide left, right 
    dst_l = roi_l(dst)      # ���ɿ��� ���� 
    dst_r = roi_r(dst)      # ���ɿ��� ������ 
    
    dst_m = roi(dst)
    # cv2.imshow("roi", dst_m)

    cdst_l = cv2.cvtColor(dst_l, cv2.COLOR_GRAY2BGR) # gray to bgr 
    cdst_r = cv2.cvtColor(dst_r, cv2.COLOR_GRAY2BGR) # gray to bgr 
    cdstp_l = np.copy(cdst_l)   
    cdstp_r = np.copy(cdst_r)    
    cdstp_m = cv2.cvtColor(dst_m,cv2.COLOR_GRAY2BGR)


    # ó���� houghlines�� �õ� 
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

    # houghlinesP ��� houghlines ���� �ð��� �� �ɸ���. 
    ##########################
    linesP_l = cv2.HoughLinesP(dst_l, 1, np.pi / 180, 40, None, 50, 10)
    linesP_r = cv2.HoughLinesP(dst_r, 1, np.pi / 180, 40, None, 50, 10)

    # �ް��� ȸ���� �����ϱ� ���� �տ� �ִ� ���� ����
    linesP_m = cv2.HoughLinesP(dst_m, 1, np.pi / 180, 10, None, 10, 30)

    if linesP_m is not None:
        for line in linesP_m:                     
            for x1, y1, x2, y2 in line:                   #������ ������ŭ �ݺ��ϸ鼭 x1, y1, x2, y2 ����
                cv2.line(cdstp_m, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)   # �� �׸���

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

    # ������ ����� ���ؼ� �ϳ��� ���� ���� 
    left_line = average(linesP_l)
    right_line = average(linesP_r)

    # avg_lines �� ���� ��, ������ �� �߰� 
    avg_lines = []
    if(left_line is not None):
        avg_lines.append(left_line)
    if(right_line is not None):
        avg_lines.append(right_line)

    # print(avg_lines)
    # print(avg_lines_l)average_line
    # print(avg_lines_r)
    # print(linesP_l[0], linesP_l[1])


    #ó������ average�� ��������ʰ� �������� ������ 
    #################### multi lines 
    # print(len(linesP_l))
    # if linesP_l is not None:
    #     for line in linesP_l:                     
    #         for x1, y1, x2, y2 in line:                   #������ ������ŭ �ݺ��ϸ鼭 x1, y1, x2, y2 ����
    #             slope = float(y2-y1) / float(x2-x1)       # ���� ��� 
    #             # print(slope)        
    #             if(slope) <= 0:                           # ���Ⱑ 0���� ������ ���ʼ����� �ν� 
    #                 cv2.line(cdstp_l, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)   # �� �׸���
    #                 lpos = (x1 + x2) /2                   # lpos �� (x1 + x2) / 2 �� ��ȯ 
    
    # if linesP_r is not None:                          
    #     for line in linesP_r:                 
    #         for x1, y1, x2, y2 in line:                   #�������� ������ŭ �ݺ��ϸ鼭 x1, y1, x2, y2 ����
    #             slope = float(y2-y1) / float(x2-x1)       # ���� ��� 
    #             if(slope) >= 0:                           # ���Ⱑ 0���� ũ�� �����ʼ����� �ν� 
    #                 cv2.line(cdstp_r, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)       # �� �׸���
    #                 rpos = (x1 + x2 )/ 2                  # rpos �� (x1 + x2) / 2 �� ��ȯ 

    ####################### only one line
    # print(linesP_l.shape)
    # print(avg_lines_l.shape)



    if avg_lines is not None:
        for line in avg_lines:
            x1, y1, x2, y2 = line.reshape(4)        # avg lines�� x1,y1,x2,y2 
            slope = float(y2-y1) / float(x2-x1)     # ���� ��� 
            if(slope) <= 0:                         # ���Ⱑ 0���� ������ ���ʼ����� �ν� 
                cv2.line(cdstp_l, (x1,y1),(x2,y2), (0,0,255),15,cv2.LINE_AA)    # �� �׸���
                if rpos > lpos:
                    lpos = (x1 + x2) / 2                 # lpos �� (x1 + x2) / 2 �� ��ȯ
            else:                                   # ���Ⱑ 0���� ũ�� �����ʼ����� �ν� 
                cv2.line(cdstp_r, (x1,y1),(x2,y2), (0,0,255),15,cv2.LINE_AA)    # �� �׸���
                if lpos < rpos:
                    rpos = (x1 + x2 )/ 2                # rpos �� (x1 + x2) / 2 �� ��ȯ


         
    # ����� �� Ȯ�� 
    # cv2.imshow("left line show", cdstp_l)
    # cv2.imshow("right line show", cdstp_r)

    cdstp_add =cv2.add(cdstp_l, cdstp_r)                        # ���� �� ���� �� �� ��ġ�� 
    combo_image = cv2.addWeighted(frame, 0.9, cdstp_add,1,1)      # ī�޶� img�� ����� ���� img ��ġ�� 
    # cv2.imshow("line show", cdstp_add)
    #cv2.imshow("lines", line_image)
    #print(x1," ", y1," ",x2, " ",y2)
    # print("lpos =", lpos, "rpos = ", rpos)

    combo_image = draw_rectangle(combo_image, lpos, rpos, offset=Offset)

    return (lpos, rpos), front_num, combo_image





#=============================================
# �������� ���� �Լ� 
# ī�޶� ������ �޾� ���� ����ó���� �˰����� ����
# ������ ��ġ�� �ľ��� �Ŀ� ���Ⱒ�� �����ϰ�,
# ���������� ���� ������ �����ϴ� ���� ������. 
#=============================================
def start():

    # ������ ������ ������ start() �ȿ��� ����ϰ��� ��
    global motor, image

    #=========================================
    # ROS ��带 �����ϰ� �ʱ�ȭ ��.
    # ī�޶� ������ �����ϰ� ���� ������ ������ ������ ����
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # ù��° ī�޶� ������ ������ ������ ��ٸ�.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    #=========================================
    # ���� ���� 
    # ī�޶� ������ �����ϴ� �ֱ⿡ ���� �ѹ��� ������ ���鼭 
    # "�̹���ó�� +������ġã�� +���Ⱒ���� +�������ȹ���" 
    # �۾��� �ݺ������� ������.
    #=========================================
    while not rospy.is_shutdown():

        # �̹���ó���� ���� ī�޶� �����̹����� img�� ���� ����
        img = image.copy()  
        pos,num, frame = process_image(img)
        

        # ������� ���� ����Ϳ� �̹����� ���÷���
        cv2.imshow("CAM View", frame)
        cv2.setMouseCallback('CAM View', onMouse)
        cv2.waitKey(1)       
       
        #=========================================
        # �ڵ����Ⱒ ���� angle�� ���ϱ�.
        # ������ ��ġ ������ �̿��ؼ� angle���� ������.        
        #=========================================

        # �켱 �׽�Ʈ�� ���� ����(0��)���� ����
        speed = 10


        #=========================================
        # ������ �ӵ� ���� speed�� ���ϱ�.
        # ���� �ڽ������� ���� �ӵ��� �����ϰ� 
        # ȸ������������ ���� �ӵ��� �����ϵ��� ������.
        #=========================================
         
        angle = (pos[0] +pos[1])/2 -320         # �߽����� ���� �󸶳� �������ִ��� ����ϱ� ���� 320�� ����
        # print(angle)
        angle = math.atan2(angle, 240)*180 /np.pi       # ���� ��� atan�� �̿��Ͽ� ��� 
        # print(angle)

        # ������ ���� �ӵ� ���� 
        if abs(angle) < 6:
            angle = 0
            speed = 25
            if num < 4:         # �տ� ���� ������ �������� �����̸� ȸ������
                speed = 15
                # print(num, "find front curved road")
        if abs(angle) < 7.5:
            agnle = angle * 0.9            # ������ ������ ���� ������ ������ angle ������ �ּ�ȭ
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
        else:                               # ���� ��Ż������ ���� ������ �ʹ� ������ �ڵ��� �� ������
            speed = 8
            angle = angle * 0.85
        # drive() ȣ��. drive()�Լ� �ȿ��� ���� ������ �����.
        drive(angle, speed)


#=============================================
# ���� �Լ�
# ���� ���� ȣ��Ǵ� �Լ��� ���⼭ start() �Լ��� ȣ����.
# start() �Լ��� �������� ���� �Լ���. 
#=============================================
if __name__ == '__main__':
    start()

