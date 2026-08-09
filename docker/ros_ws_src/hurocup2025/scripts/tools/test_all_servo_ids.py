#!/usr/bin/env python3
# encoding: utf-8
# 测试所有舵机ID是否可读
import time
from ainex_sdk import Board

def test_servo_id(servo_control, servo_id):
    try:
        # 尝试读取舵机位置，如果成功则说明该ID可用
        pos = servo_control.bus_servo_read_position(servo_id)
        if pos:
            print(f"ID: {servo_id:2d} - Success")
        else:
            print(f"ID: {servo_id:2d} - Fail")
        return True
    except:
        print(f"ID: {servo_id:2d} - Fail")
        return False

def main():
    servo_control = Board()
    servo_control.enable_reception()
    print('开始测试所有舵机ID...')
    print('------------------------------')
    
    # 测试ID 1-24
    for servo_id in range(1, 25):
        test_servo_id(servo_control, servo_id)
        time.sleep(0.1)  # 短暂延时，避免通信过快
    
    print('------------------------------')
    print('测试完成')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n程序已终止') 