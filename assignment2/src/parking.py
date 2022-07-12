#! /usr/bin/env python
# -*- coding: utf-8 -*-

# =============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
# =============================================
import rospy, math
import cv2, time, rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor


# =============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
# =============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)


# =============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
# =============================================
arData = {"DX": 0.0, "DY": 0.0, "DZ": 0.0,
          "AX": 0.0, "AY": 0.0, "AZ": 0.0, "AW": 0.0}
roll, pitch, yaw = 0, 0, 0
motor_msg = xycar_motor()


# =============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
# =============================================
def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w


# =========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
# =========================================
rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)


# =========================================
# 자동차를 후진시키는 함수
# 후진 방향 조정을 위해 angle 값이 양수일 경우, 음수일 경우에 따라 조건이 나뉨.
# 모터제어 토픽을 발행하며, speed를 -15로 조정함.
# =========================================
def reverse_car(angle, tim):
    global motor_msg, motor_pub  # 조향각 값과 속도값 수정을 위한 전역변수 지정

    # 차량 진입 방향이 오른쪽일 경우, 반대방향 회전을 위한 조건문
    if angle > 0:
        for tim in range(tim):
            motor_msg.angle = angle       # motor의 angle값 지정
            motor_msg.speed = -15         # motor의 speed값 지정
            motor_pub.publish(motor_msg)  # 변화된 모터의 angle값, speed값 토픽 재발행
            time.sleep(0.1)  # 0.1초 일시 정지

    # 차량 진입 방향이 왼쪽일 경우, 반대방향 회전을 위한 조건문
    else:
        for tim in range(tim):
            motor_msg.angle = -angle      # motor의 angle값 지정
            motor_msg.speed = -15         # motor의 speed값 지정
            motor_pub.publish(motor_msg)  # 변화된 모터의 angle값, speed값 토픽 재발행
            time.sleep(0.1)  # 0.1초 일시 정지


# =========================================
# 메인 루프
# 끊임없이 루프를 돌면서
# "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행"
# 작업을 반복적으로 수행함.
# =========================================
while not rospy.is_shutdown():

    # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
    (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))

    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    # Row 100, Column 500 크기의 배열(이미지) 준비
    img = np.zeros((100, 500, 3))

    # 4개의 직선 그리기
    img = cv2.line(img, (25, 65), (475, 65), (0, 0, 255), 2)
    img = cv2.line(img, (25, 40), (25, 90), (0, 0, 255), 3)
    img = cv2.line(img, (250, 40), (250, 90), (0, 0, 255), 3)
    img = cv2.line(img, (475, 40), (475, 90), (0, 0, 255), 3)

    # DX 값을 그림에 표시하기 위한 좌표값 계산
    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25:
        point = 25

    # DX값에 해당하는 위치에 동그라미 그리기
    img = cv2.circle(img, (point, 65), 15, (0, 255, 0), -1)

    # DX값과 DY값을 이용해서 거리값 distance 구하기
    distance = math.sqrt(pow(arData["DX"], 2) + pow(arData["DY"], 2))

    # 그림 위에 distance 관련된 정보를 그려넣기
    cv2.putText(img, str(int(distance)) + " pixel", (350, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

    # DX값 DY값 Yaw값 구하기
    dx_dy_yaw = "DX:" + str(int(arData["DX"])) + " DY:" + str(int(arData["DY"])) \
                + " Yaw:" + str(round(yaw, 1))

    # 그림 위에 DX값 DY값 Yaw값 관련된 정보를 그려넣기
    cv2.putText(img, dx_dy_yaw, (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))

    # 만들어진 그림(이미지)을 모니터에 디스플레이 하기
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)

    # DX, DY 값을 이용하여 목표지점을 향해 angle각을 조정하며 접근
    deg = 0  # arData["DY"]의 데이터값이 0인 경우를 위를 위한 기본값 지정

    if arData["DY"]:
        deg = math.degrees(math.atan(arData["DX"] / arData["DY"]))  # (x축)÷(y축)의 아크탄젠트값을 deg에 저장함.
    angle = 0  # 목표 지점을 향해 알맞은 각도로 전진하고 있을 경우, angle 값을 0으로 지정하여 직진

    if abs(deg) > 20:  # deg 계산값의 절닷값이 20 이상일 경우
        angle = (yaw + deg) * 1.5  # 회전반경의 값과 deg을 더한 값에 1.5를 곱하여 angle 값으로 지정함
    else:  # 위의 조건이 만족하지 않는 경우
        if yaw < 0:  # 회전반경이 0 미만일 때
            angle = 50  # angle값을 50으로 지정, 오른쪽으로 회전
        else:  # 회전반경이 0 이상일 때
            angle = -50  # angle값을 -50으로 지정, 왼쪽으로 회전

    # 정확도 향상을 위해 목표 지점에 가까워질 수록 속도를 저하시키며 접근
    if (arData["DY"] > 300):  # 목표 지점에 대한 Y축 거리가 300 이상일 경우
        speed = 50  # speed값을 50으로 지정
    elif (arData["DY"] > 200):  # 목표 지점에 대한 Y축 거리가 200 이상일 경우
        speed = 30  # speed값을 30으로 지정
    elif (arData["DY"] > 100):  # 목표 지점에 대한 Y축 거리가 100 이상일 경우
        speed = 20  # speed값을 20으로 지정

    # 목표 지점에 접근하였으나 주차 각도가 적절하지 않을 경우, 후진 후 재접근
    elif (arData["DY"] > 70 and arData["DY"] < 100):  # 목표 지점에 대한 Y축 거리가 70이상 100 미만일 경우
        if (yaw > 10 or abs(arData["DX"] > 100)):  # 회전반경 값이 10 이상이거나 X축의 절댓값이 100 이상일 경우
            reverse_car((-1) * angle, 20)  # reverse_car 함수 접근, 후진 시 중앙에 가까운 궤도를 만들기 위해 현재 angle값에 -1을 곱하여 양수를 음수로 변환.
        elif (yaw < -10 or abs(arData["DX"] > 100)):  # 회전반경 값이 -10 이하이거나 X축의 절댓값이 100 이상일 경우
            reverse_car((-1) * angle, 20)  # reverse_car 함수 접근, 후진 시 중앙에 가까운 궤도를 만들기 위해 현재 angle값에 -1을 곱하여 음수를 양수로 변환.

    # 목표 지점에 접근하였을 경우 종료
    else:
        speed = 0  # speed값을 0으로 지정

    # 조향각값과 속도값을 넣어 모터 토픽을 발행하기
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_pub.publish(motor_msg)

# while 루프가 끝나면 열린 윈도우 모두 닫고 깔끔하게 종료하기
cv2.destroyAllWindows()