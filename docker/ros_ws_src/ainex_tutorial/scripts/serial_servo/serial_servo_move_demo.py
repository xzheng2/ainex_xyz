#!/usr/bin/env python3
# encoding: utf-8
# 驱动id23的舵机在400-600范围转动(Drive the servos 2 and 3 to rotate in the range of 400 to 600)
import time
from ainex_sdk import Board

servo_control = Board()
print('id23 serial servo move between 400 - 600')

while True:
    try:
        servo_id = 24  # 舵机id(0-253)(servo ID (0-253))
        position = 290  # 位置(0-1000)(position (0-1000))
        duration = 0.05  # 时间(0.02-30s)(time (0.02-30))
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        time.sleep(duration)  
        
        position = 290
        duration = 0.75
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        time.sleep(duration)
        
        position = 360
        duration = 0.05
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        time.sleep(duration)
        
        position = 360
        duration = 0.15
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        time.sleep(duration)
        
        
        
        
    except KeyboardInterrupt:
        position = 370
        duration = 0.03
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        break
