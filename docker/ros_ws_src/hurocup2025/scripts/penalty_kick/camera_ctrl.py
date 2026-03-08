#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/11
# @author:aiden
# 自动踢球(autonomous ball kicking)
import time
import math
import rospy
import signal
import cv2
from std_msgs.msg import Float64, String
from ainex_sdk import pid, misc, common
from ainex_example.color_common import Common
from ainex_example.pid_track import PIDTrack
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect
import time
from ainex_sdk import Board
import numpy as np

servo_control = Board()
#print('id23 serial servo move between 400 - 600')
head_pan_init = 500
head_tilt_init = 300

if __name__ == "__main__":
    frame = 255 * np.ones((200, 200, 3), dtype=np.uint8)  # 白图
    cv2.imshow("window", frame)
    while True:
        # 状态判断(determine state)
        #if self.start:
        if True:
            #print('True')
            ball_data = None 
            
            key = cv2.waitKey(0) & 0xFF
            #print(key)
            if key == ord('q'):
                break
            elif key == ord('w'):  # 按下 'q' 退出
                #self.head_pan_init = 500  # 左右舵机的初始值(initial value of left-right servo)
                head_tilt_init += 10
                servo_control.bus_servo_set_position(0.5, [[24, head_tilt_init]]) 
                print("up...")
            elif key == ord('s'):  # 按下 'q' 退出
                head_tilt_init -= 10
                servo_control.bus_servo_set_position(0.5, [[24, head_tilt_init]])
                print("down...")
            elif key == ord('a'):  # 按下 'q' 退出
                print("left...")
                head_pan_init += 10
                servo_control.bus_servo_set_position(0.5, [[23, head_pan_init]]) 
            elif key == ord('d'):  # 按下 'q' 退出
                print("right...")
                head_pan_init -= 10
                servo_control.bus_servo_set_position(0.5, [[23, head_pan_init]]) 

            time.sleep(0.01)
